import socket
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from ground_gen2.vbw import encode_vbw, VBWInput
from ground_gen2.transport import UDPReceiver, SerialReceiver


def message(**kw):
    values = dict(water_longitudinal_knots=0., water_transverse_knots=0., water_valid=False,
                  ground_longitudinal_knots=2., ground_transverse_knots=-1., ground_valid=True)
    values.update(kw)
    return SimpleNamespace(**values)


class VBWTests(unittest.TestCase):
    def test_sentence(self):
        data = encode_vbw(message())
        body, check = data[1:].strip().split(b'*')
        self.assertEqual(body, b'VMVBW,,,V,2.000,-1.000,A')
        checksum = 0
        for byte in body: checksum ^= byte
        self.assertEqual(int(check, 16), checksum)
        self.assertTrue(data.endswith(b'\r\n'))

    def test_invalid(self):
        for value in [float('inf'), -float('inf'), 1000., -1000.]:
            with self.assertRaises(ValueError):
                encode_vbw(message(ground_longitudinal_knots=value))
        self.assertIn(b',,,V', encode_vbw(message(ground_valid=False, ground_longitudinal_knots=float('nan'))))

    def test_partial_pairs(self):
        for source in ('water', 'ground'):
            for known in ('longitudinal', 'transverse'):
                values = {s + '_' + axis + '_knots': float('nan')
                          for s in ('water', 'ground') for axis in ('longitudinal', 'transverse')}
                values.update(water_valid=True, ground_valid=True)
                values[source + '_' + known + '_knots'] = -2.5
                data = encode_vbw(message(**values))
                body, checksum = data[1:].strip().split(b'*')
                fields = body.decode().split(',')
                offset = 1 if source == 'water' else 4
                self.assertEqual(fields[offset:offset+3],
                                 ['-2.500', '', 'A'] if known == 'longitudinal' else ['', '-2.500', 'A'])
                other = 4 if source == 'water' else 1
                self.assertEqual(fields[other:other+3], ['', '', 'V'])
                expected = 0
                for byte in body: expected ^= byte
                self.assertEqual(int(checksum, 16), expected)

    def test_longitudinal_only_delivery(self):
        sent = []
        inp = VBWInput(self.fail)
        inp.submit(message(water_longitudinal_knots=float('nan'),
                           water_transverse_knots=float('nan'),
                           ground_transverse_knots=float('nan')))
        inp.drain(SimpleNamespace(write_nmea=sent.append))
        self.assertEqual(sent[0].split(b'*')[0], b'$VMVBW,,,V,2.000,,A')

    def test_latest_and_expiry(self):
        sent = []
        receiver = SimpleNamespace(write_nmea=sent.append)
        inp = VBWInput(lambda msg: None)
        with patch('ground_gen2.vbw.time.monotonic', return_value=10.):
            inp.submit(message())
            inp.submit(message(ground_longitudinal_knots=3.))
            inp.drain(receiver)
            inp.drain(receiver)
            inp.submit(message())
        with patch('ground_gen2.vbw.time.monotonic', return_value=12.):
            inp.drain(receiver)
        self.assertEqual(len(sent), 1)
        self.assertIn(b'3.000', sent[0])

    def test_udp_route(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as target:
            target.bind(('127.0.0.1', 0))
            target.settimeout(1.)
            receiver = UDPReceiver(port=0, rtcm_host='127.0.0.1', nmea_port=target.getsockname()[1])
            try:
                data = encode_vbw(message())
                receiver.write_nmea(data)
                self.assertEqual(target.recv(1024), data)
            finally:
                receiver.close()

    def test_serial_partial_write(self):
        receiver = SerialReceiver.__new__(SerialReceiver)
        receiver.serial = SimpleNamespace(write=lambda data: len(data)-1)
        with self.assertRaises(OSError): receiver.write_nmea(b'test')
