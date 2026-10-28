import unittest
import socket
from types import SimpleNamespace
from ground_gen2.gps_control import encode_gps_control, GPSControlSender
from ground_gen2.transport import UDPReceiver

class GPSControlTests(unittest.TestCase):
    def test_encoding(self):
        for command, checksum in [(0, '10'), (1, '11'), (2, '12')]:
            self.assertEqual(encode_gps_control(command), ('$PAPGPSCTRL,%d*%s\r\n' % (command, checksum)).encode())
        for command in [-1, 3, 255, True, 1.0]:
            with self.assertRaises(ValueError): encode_gps_control(command)

    def test_failures_not_queued(self):
        sender = GPSControlSender()
        self.assertFalse(sender.send(1)[0])
        sent = []
        sender.receiver = SimpleNamespace(write_nmea=sent.append)
        self.assertFalse(sender.send(3)[0])
        self.assertEqual(sent, [])
        self.assertTrue(sender.send(1)[0])
        self.assertEqual(sent, [b'$PAPGPSCTRL,1*11\r\n'])
        def fail(data): raise OSError('write failed')
        sender.receiver = SimpleNamespace(write_nmea=fail)
        self.assertFalse(sender.send(1)[0])

    def test_udp(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sink:
            sink.bind(('127.0.0.1', 0)); sink.settimeout(1)
            receiver = UDPReceiver(port=0, nmea_host='127.0.0.1', nmea_port=sink.getsockname()[1])
            sender = GPSControlSender(); sender.receiver = receiver
            try:
                self.assertTrue(sender.send(2)[0])
                self.assertEqual(sink.recv(1024), b'$PAPGPSCTRL,2*12\r\n')
            finally: receiver.close()
