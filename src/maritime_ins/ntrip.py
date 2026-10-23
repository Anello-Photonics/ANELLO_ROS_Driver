"""ROS-independent NTRIP client and bounded RTCM3 delivery queue."""
import base64
import queue
import socket
import ssl
import threading
import time
from urllib.parse import quote
from .parser import decode, SentenceError

DEFAULTS = dict(ntrip_enabled=False, ntrip_host='', ntrip_port=2101,
                ntrip_mountpoint='', ntrip_username='', ntrip_password='',
                ntrip_tls=False, ntrip_ca_cert='', ntrip_gga_interval=5.0,
                ntrip_timeout=15.0, ntrip_reconnect_delay=5.0,
                rtcm_host='', rtcm_port=19552)


def crc24q(data):
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864cfb
    return crc & 0xffffff


class RTCMFramer:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        while self.buffer:
            if self.buffer[0] != 0xd3:
                del self.buffer[0]
                continue
            if len(self.buffer) < 3:
                break
            if self.buffer[1] & 0xfc:
                del self.buffer[0]
                continue
            size = ((self.buffer[1] & 3) << 8 | self.buffer[2]) + 6
            if len(self.buffer) < size:
                break
            frame = bytes(self.buffer[:size])
            if crc24q(frame[:-3]) == int.from_bytes(frame[-3:], 'big'):
                del self.buffer[:size]
                yield frame
            else:
                del self.buffer[0]


class CasterStream:
    """Incrementally strip HTTP/ICY headers and HTTP chunk framing."""
    def __init__(self):
        self.buffer = bytearray()
        self.ready = False
        self.chunked = False
        self.remaining = None

    def feed(self, data):
        self.buffer.extend(data)
        if not self.ready:
            end = self.buffer.find(b'\r\n')
            if end < 0:
                if len(self.buffer) > 16384:
                    raise ValueError('Caster response header too large')
                return b''
            status = bytes(self.buffer[:end]).split()
            if len(status) < 2 or status[1] != b'200' or status[0] not in (b'ICY', b'HTTP/1.0', b'HTTP/1.1'):
                raise ValueError('Caster rejected request (check mountpoint and credentials)')
            if status[0] == b'ICY':
                # NTRIP v1 can begin RTCM immediately after the status line.
                del self.buffer[:end + 2]
            else:
                end = self.buffer.find(b'\r\n\r\n')
                if end < 0:
                    if len(self.buffer) > 16384:
                        raise ValueError('Caster response header too large')
                    return b''
                headers = bytes(self.buffer[:end]).lower()
                if b'sourcetable' in headers:
                    raise ValueError('Caster returned sourcetable instead of corrections')
                for line in headers.split(b'\r\n')[1:]:
                    key, _, value = line.partition(b':')
                    if key == b'transfer-encoding':
                        if value.strip() != b'chunked':
                            raise ValueError('Unsupported caster transfer encoding')
                        self.chunked = True
                    if key == b'content-encoding' and value.strip() != b'identity':
                        raise ValueError('Unsupported caster content encoding')
                del self.buffer[:end + 4]
            self.ready = True
        if not self.chunked:
            data = bytes(self.buffer)
            self.buffer.clear()
            return data
        output = bytearray()
        while True:
            if self.remaining is None:
                end = self.buffer.find(b'\r\n')
                if end < 0:
                    if len(self.buffer) > 1024:
                        raise ValueError('Invalid HTTP chunk header')
                    break
                self.remaining = int(self.buffer[:end].split(b';')[0], 16)
                if not 0 < self.remaining <= 1048576:
                    raise ValueError('Caster stream ended or chunk is too large')
                del self.buffer[:end + 2]
            count = min(self.remaining, len(self.buffer))
            output.extend(self.buffer[:count])
            del self.buffer[:count]
            self.remaining -= count
            if self.remaining:
                break
            if len(self.buffer) < 2:
                break
            if self.buffer[:2] != b'\r\n':
                raise ValueError('Invalid HTTP chunk terminator')
            del self.buffer[:2]
            self.remaining = None
        return bytes(output)


class NtripClient:
    def __init__(self, options, warn=lambda message: None):
        self.options = dict(DEFAULTS, **options)
        self.warn = warn
        self.frames = queue.Queue(maxsize=100)
        self.stop = threading.Event()
        self.gga_lock = threading.Lock()
        self.gga = None
        self.thread = None
        self.connection = None
        o = self.options
        if o['ntrip_enabled']:
            if not o['ntrip_host'] or not o['ntrip_mountpoint']:
                raise ValueError('NTRIP requires ntrip_host and ntrip_mountpoint')
            if any(c in o['ntrip_host'] for c in '\r\n'):
                raise ValueError('Invalid caster hostname')
            if not 1 <= o['rtcm_port'] <= 65535:
                raise ValueError('Invalid RTCM destination port')
            if not 1 <= o['ntrip_port'] <= 65535:
                raise ValueError('Invalid caster port')
            if min(o['ntrip_timeout'], o['ntrip_reconnect_delay']) <= 0 or o['ntrip_gga_interval'] < 0:
                raise ValueError('Invalid NTRIP timeout/interval')

    def start(self):
        if self.options['ntrip_enabled']:
            self.thread = threading.Thread(target=self.run, name='maritime_ntrip', daemon=True)
            self.thread.start()

    def observe(self, sentence):
        if not self.options['ntrip_enabled']:
            return
        try:
            kind, fields = decode(sentence)
        except SentenceError:
            return
        import math
        if kind == 'GGA' and fields['fix_quality'] > 0 and math.isfinite(fields['latitude_deg']) and math.isfinite(fields['longitude_deg']):
            if isinstance(sentence, str):
                sentence = sentence.encode('ascii')
            with self.gga_lock:
                self.gga = (time.monotonic(), sentence.rstrip(b'\r\n') + b'\r\n')

    def request(self):
        o = self.options
        mount = quote(o['ntrip_mountpoint'].lstrip('/'), safe='/')
        request = ('GET /%s HTTP/1.1\r\nHost: %s:%s\r\n'
                   'Ntrip-Version: Ntrip/2.0\r\nUser-Agent: NTRIP MaritimeINS/1.0\r\n'
                   'Connection: close\r\n' % (mount, o['ntrip_host'], o['ntrip_port']))
        if o['ntrip_username'] or o['ntrip_password']:
            token = base64.b64encode((o['ntrip_username'] + ':' + o['ntrip_password']).encode()).decode()
            request += 'Authorization: Basic ' + token + '\r\n'
        return (request + '\r\n').encode('ascii')

    def drain(self, receiver):
        # Only the ROS receive loop writes to the device. Never open serial twice.
        for _ in range(100):
            try:
                stamp, frame = self.frames.get_nowait()
            except queue.Empty:
                break
            if time.monotonic() - stamp <= 2.0:
                receiver.write_rtcm(frame)

    def run(self):
        while not self.stop.is_set():
            try:
                self.session()
            except (OSError, ValueError) as exc:
                if not self.stop.is_set():
                    # Never include requests, credentials, or server response bodies.
                    self.warn('NTRIP connection interrupted: ' + str(exc))
            finally:
                if self.connection is not None:
                    self.connection.close()
                    self.connection = None
            self.stop.wait(self.options['ntrip_reconnect_delay'])

    def session(self):
        o = self.options
        self.connection = socket.create_connection((o['ntrip_host'], o['ntrip_port']), timeout=3.0)
        if o['ntrip_tls']:
            context = ssl.create_default_context(cafile=o['ntrip_ca_cert'] or None)
            self.connection = context.wrap_socket(self.connection, server_hostname=o['ntrip_host'])
        self.connection.sendall(self.request())
        self.connection.settimeout(0.2)
        stream, framer = CasterStream(), RTCMFramer()
        last_frame, last_gga = time.monotonic(), 0.0
        while not self.stop.is_set():
            now = time.monotonic()
            if now - last_frame > o['ntrip_timeout']:
                raise OSError('Timed out waiting for valid RTCM corrections')
            with self.gga_lock:
                gga = self.gga
            if stream.ready and o['ntrip_gga_interval'] > 0 and gga and now - gga[0] < 30 and now - last_gga >= o['ntrip_gga_interval']:
                self.connection.sendall(gga[1])
                last_gga = now
            try:
                data = self.connection.recv(8192)
            except socket.timeout:
                continue
            if not data:
                raise OSError('Caster closed the connection')
            for frame in framer.feed(stream.feed(data)):
                last_frame = time.monotonic()
                if self.frames.full():
                    try:
                        self.frames.get_nowait()
                    except queue.Empty:
                        pass
                self.frames.put_nowait((last_frame, frame))

    def close(self):
        self.stop.set()
        if self.connection is not None:
            try:
                self.connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self.thread is not None:
            self.thread.join(timeout=7.0)
