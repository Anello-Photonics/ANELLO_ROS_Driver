"""NMEA receive and RTCM transmit serial and UDP connections for GroundGen2 NMEA0183 output."""
import socket
from .parser import Framer


class UDPReceiver:
    def __init__(self, bind_address='0.0.0.0', port=19550, multicast_group='',
                 multicast_interface='0.0.0.0', source_ip='', rtcm_host='', rtcm_port=19552, nmea_host='', nmea_port=19551):
        self.source_ip = source_ip
        self.rtcm_destination = (rtcm_host, rtcm_port)
        self.nmea_destination = (nmea_host or rtcm_host, nmea_port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((bind_address, port))
            if multicast_group:
                group = socket.inet_aton(multicast_group)
                if not 224 <= group[0] <= 239:
                    raise ValueError('multicast_group must be an IPv4 multicast address')
                membership = group + socket.inet_aton(multicast_interface)
                self.socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
            self.socket.settimeout(0.2)
        except Exception:
            self.socket.close()
            raise

    def read(self):
        try:
            data, address = self.socket.recvfrom(65535)
        except socket.timeout:
            return []
        if self.source_ip and address[0] != self.source_ip:
            return []
        # Firmware sends complete sentences per datagram. Never join data from
        # different senders or recover a truncated datagram using the next one.
        return list(Framer().feed(data + b'\n'))

    def write_nmea(self, sentence):
        if not self.nmea_destination[0]:
            raise OSError('VBW UDP requires nmea_host or rtcm_host')
        self.socket.sendto(sentence, self.nmea_destination)

    def write_rtcm(self, frame):
        if not self.rtcm_destination[0]:
            raise ValueError('UDP NTRIP routing requires rtcm_host (INS IPv4 address)')
        self.socket.sendto(frame, self.rtcm_destination)

    def close(self):
        self.socket.close()


class SerialReceiver:
    def __init__(self, port, baudrate):
        import serial
        self.serial = serial.Serial(port, baudrate, timeout=0.2, write_timeout=0.5)
        self.framer = Framer()

    def read(self):
        return list(self.framer.feed(self.serial.read(self.serial.in_waiting or 1)))

    def write_rtcm(self, frame):
        if self.serial.write(frame) != len(frame):
            raise OSError('Incomplete RTCM serial write')

    def write_nmea(self, sentence):
        if self.serial.write(sentence) != len(sentence):
            raise OSError("Incomplete VBW serial write")

    def close(self):
        self.serial.close()
