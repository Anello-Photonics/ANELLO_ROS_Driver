import base64
import os
import pty
import select
import socket
import threading
import time
import unittest

from ground_gen2.ntrip import CasterStream, NtripClient, RTCMFramer, crc24q
from ground_gen2.transport import SerialReceiver, UDPReceiver
from test_ground_gen2 import BODIES, sentence


def rtcm(payload=b'\x3e\xd0\x00\xff\x80\x13'):
    body = bytes([0xd3, len(payload) >> 8, len(payload) & 255]) + payload
    return body + crc24q(body).to_bytes(3, 'big')


class FakeCaster:
    """Local caster fixture, including GGA uplink and fragmented binary response."""
    def __init__(self, response=None):
        self.listener = socket.socket()
        self.listener.bind(('127.0.0.1', 0))
        self.listener.listen()
        self.listener.settimeout(0.1)
        self.port = self.listener.getsockname()[1]
        self.request = b''
        self.gga = b''
        self.connections = 0
        self.stop = threading.Event()
        frame = rtcm()
        self.response = response or (b'HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n' +
                                     ('%x\r\n' % len(frame)).encode() + frame + b'\r\n')
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            try:
                conn, _ = self.listener.accept()
            except socket.timeout:
                continue
            with conn:
                self.connections += 1
                conn.settimeout(0.1)
                request = b''
                try:
                    while b'\r\n\r\n' not in request and not self.stop.is_set():
                        try:
                            data = conn.recv(4096)
                        except socket.timeout:
                            continue
                        if not data:
                            break
                        request += data
                    self.request = request
                    for i in range(0, len(self.response), 3):
                        conn.sendall(self.response[i:i+3])
                    while not self.stop.is_set():
                        try:
                            data = conn.recv(4096)
                        except socket.timeout:
                            continue
                        if not data:
                            break
                        self.gga += data
                except OSError:
                    pass

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        self.listener.close()


def client_for(caster):
    return NtripClient(dict(ntrip_enabled=True, ntrip_host='127.0.0.1', ntrip_port=caster.port,
                           ntrip_mountpoint='TEST', ntrip_username='user', ntrip_password='pass',
                           ntrip_gga_interval=0.05, ntrip_reconnect_delay=0.05, ntrip_timeout=1.0))


def until(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class NtripTest(unittest.TestCase):
    def test_crc_and_fragmentation(self):
        self.assertEqual(crc24q(b'123456789'), 0xcde703)
        frame = rtcm()
        parser = RTCMFramer()
        output = []
        corrupt = frame[:-1] + bytes([frame[-1] ^ 1])
        for byte in b'junk' + corrupt + frame + frame:
            output.extend(parser.feed(bytes([byte])))
        self.assertEqual(output, [frame, frame])

    def test_http_icy_and_chunks_preserve_binary(self):
        frame = rtcm()
        responses = [b'ICY 200 OK\r\n' + frame,
                     b'HTTP/1.0 200 OK\r\nContent-Type: gnss/data\r\n\r\n' + frame,
                     b'HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n' +
                     b'3\r\n' + frame[:3] + b'\r\n' + ('%x;test=yes\r\n' % len(frame[3:])).encode() + frame[3:] + b'\r\n']
        for response in responses:
            stream = CasterStream()
            output = b''.join(stream.feed(bytes([b])) for b in response)
            self.assertEqual(output, frame)

    def test_errors_never_become_corrections(self):
        for response in [b'HTTP/1.1 401 Unauthorized\r\n\r\n', b'SOURCETABLE 200 OK\r\n',
                         b'HTTP/1.1 200 OK\r\nContent-Type: gnss/sourcetable\r\n\r\n']:
            with self.assertRaises(ValueError):
                CasterStream().feed(response)

    def test_ntrip_to_udp_and_gga_auth(self):
        caster = FakeCaster()
        client = client_for(caster)
        sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sink.bind(('127.0.0.1', 0))
        sink.settimeout(2)
        receiver = UDPReceiver('127.0.0.1', 0, rtcm_host='127.0.0.1', rtcm_port=sink.getsockname()[1])
        try:
            client.observe(sentence(BODIES['GGA']))
            client.start()
            self.assertTrue(until(lambda: not client.frames.empty()))
            client.drain(receiver)
            self.assertEqual(sink.recv(2048), rtcm())
            self.assertTrue(until(lambda: b'$GPGGA' in caster.gga))
            self.assertIn(b'GET /TEST HTTP/1.1', caster.request)
            self.assertIn(b'Authorization: Basic ' + base64.b64encode(b'user:pass'), caster.request)
        finally:
            client.close()
            caster.close()
            receiver.close()
            sink.close()
        self.assertFalse(client.thread.is_alive())

    def test_ntrip_to_shared_serial(self):
        caster = FakeCaster(b'ICY 200 OK\r\n' + rtcm())
        client = client_for(caster)
        master, slave = pty.openpty()
        receiver = SerialReceiver(os.ttyname(slave), 921600)
        try:
            client.start()
            self.assertTrue(until(lambda: not client.frames.empty()))
            client.drain(receiver)
            self.assertTrue(select.select([master], [], [], 2)[0])
            self.assertEqual(os.read(master, 2048), rtcm())
            os.write(master, sentence(BODIES['HDT']))
            output = []
            self.assertTrue(until(lambda: output.extend(receiver.read()) or bool(output)))
            self.assertEqual(output[0], sentence(BODIES['HDT']).rstrip())
        finally:
            client.close()
            caster.close()
            receiver.close()
            os.close(master)
            os.close(slave)

    def test_reconnect_after_rejection(self):
        caster = FakeCaster(b'HTTP/1.1 401 Unauthorized\r\n\r\n')
        client = client_for(caster)
        try:
            client.start()
            self.assertTrue(until(lambda: caster.connections >= 2))
            self.assertTrue(client.frames.empty())
        finally:
            client.close()
            caster.close()

    def test_stale_data_and_invalid_gga(self):
        client = NtripClient(dict(ntrip_enabled=True, ntrip_host='localhost', ntrip_mountpoint='test'))
        client.observe(sentence('GPGGA,000000.00,,,,,0,00,99.9,,M,,M,,'))
        self.assertIsNone(client.gga)
        client.frames.put((time.monotonic() - 3, rtcm()))
        class Sink:
            def write_rtcm(self, frame):
                raise AssertionError('Stale frame forwarded')
        client.drain(Sink())


if __name__ == '__main__':
    unittest.main()
