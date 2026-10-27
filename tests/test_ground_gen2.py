import importlib.util
import math
import os
from pathlib import Path
import pty
import socket
import sys
import types
import unittest
from unittest.mock import patch

from ground_gen2.parser import Framer, KINDS, SentenceError, decode
from ground_gen2.transport import SerialReceiver, UDPReceiver

ROOT = Path(__file__).resolve().parents[1]


def sentence(body):
    crc = 0
    for c in body:
        crc ^= ord(c)
    return ('$%s*%02x\r\n' % (body, crc)).encode('ascii')


# Representative layouts from NMEAOutput.cpp; distinct values expose index swaps.
BODIES = {
    'GGA': 'GPGGA,123456.78,3723.932530,N,12158.123456,W,4,18,0.8,12.3,M,-31.2,M,,',
    'RMC': 'GPRMC,123456.78,A,3723.932530,N,12158.123456,W,2.50,123.45,231026,,',
    'HDT': 'GPHDT,271.234,T',
    'XDR': 'YXXDR,A,271.23,D,HDG,A,-2.34,D,ROLL,A,3.45,D,PITCH',
    'ZDA': 'GPZDA,123456.78,23,10,2026,,',
    'PAPIMU': 'PAPIMU,1234.567,1200.000,0.1,-0.2,1.0,1.1,2.2,3.3,4.4,5.5,6.6,0.01,0.02,0.03,42.5,2,4,8',
    'PAPINS': 'PAPINS,1234.567,1792758896.123456,3,37.1,-121.2,8.5,1.1,2.2,-0.3,4.4,-5.5,6.6,',
    'PAPACC': 'PAPACC,1234.567,1792758896.123456,1,2,3,4,5,6,7,8',
}


class ParserTest(unittest.TestCase):
    def test_all_outputs_match_message_definitions(self):
        for kind, body in BODIES.items():
            with self.subTest(kind=kind):
                actual, fields = decode(sentence(body))
                self.assertEqual(kind, actual)
                schema = (ROOT / 'msg' / ('GroundGen2' + kind + '.msg')).read_text()
                expected = {line.split('#')[0].split()[1] for line in schema.splitlines()
                            if line.split('#')[0].strip()} - {'header'}
                self.assertEqual(expected, set(fields))

    def test_coordinates(self):
        _, fields = decode(sentence(BODIES['GGA']))
        self.assertAlmostEqual(fields['latitude_deg'], 37 + 23.932530 / 60)
        self.assertAlmostEqual(fields['longitude_deg'], -(121 + 58.123456 / 60))
        self.assertEqual(fields['fix_quality'], 4)
        self.assertEqual(fields['altitude_msl_m'], 12.3)
        self.assertEqual(fields['geoid_separation_m'], -31.2)

    def test_proprietary_units_and_field_order(self):
        _, imu = decode(sentence(BODIES['PAPIMU']))
        self.assertEqual(imu['time_ms'], 1234.567)
        self.assertEqual(imu['sync_time_ms'], 1200)
        self.assertEqual(imu['siphog_rate_x_deg_s'], 4.4)
        self.assertEqual(imu['siphog_rate_y_deg_s'], 5.5)
        self.assertEqual(imu['siphog_rate_z_deg_s'], 6.6)
        self.assertEqual(imu['status_z'], 8)
        _, ins = decode(sentence(BODIES['PAPINS']))
        self.assertEqual(ins['utc_time_s'], 1792758896.123456)
        self.assertEqual(ins['velocity_down_m_s'], -0.3)
        self.assertEqual(ins['zupt'], -1)
        _, acc = decode(sentence(BODIES['PAPACC']))
        self.assertEqual(acc['velocity_north_accuracy_m_s'], 3)
        self.assertEqual(acc['velocity_down_accuracy_m_s'], 5)
        self.assertEqual(acc['roll_accuracy_deg'], 6)
        self.assertEqual(acc['heading_accuracy_deg'], 8)

    def test_empty_firmware_outputs(self):
        bodies = [
            'GPGGA,000000.00,,,,,0,00,99.9,,M,,M,,',
            'GPRMC,000000.00,V,,,,,,,000000,,',
            'GPHDT,,T', 'YXXDR,A,,D,HDG,A,,D,ROLL,A,,D,PITCH',
            'GPZDA,,,,,,', 'PAPINS,1.000,0.000000,0' + ',' * 10,
            'PAPACC,1.000,0.000000' + ',' * 8,
        ]
        for body in bodies:
            with self.subTest(body=body):
                decode(sentence(body))
        self.assertTrue(math.isnan(decode(sentence('GPHDT,,T'))[1]['heading_deg']))
        self.assertFalse(decode(sentence('GPZDA,,,,,,'))[1]['local_zone_valid'])

    def test_xdr_uses_transducer_names(self):
        _, data = decode(sentence('YXXDR,A,3,D,PITCH,A,1,D,HDG,A,2,D,ROLL'))
        self.assertEqual(data['heading_deg'], 1)
        self.assertEqual(data['pitch_deg'], 3)

    def test_multiple_talkers(self):
        self.assertEqual(decode(sentence('GNHDT,90,T'))[0], 'HDT')

    def test_checksum_and_layout_rejected(self):
        invalid = [b'$GPHDT,1,T*00', b'$GPHDT,1,T', b'$GPHDT,1,T*zz',
                   sentence('GPHDT,1'), sentence('GPHDT,1,T,extra'),
                   sentence('PAPIMU,1,2'), sentence('APIMU,1,2'),
                   sentence('GPHDT,nan,T'), sentence('GPHDT,1junk,T'),
                   sentence('GPHDT,1,M'), sentence('GPZDA,,1,1,99999,,'),
                   sentence('YXXDR,A,1,D,HDG,A,2,D,HDG,A,3,D,PITCH')]
        for data in invalid:
            with self.subTest(data=data), self.assertRaises(SentenceError):
                decode(data)

    def test_bad_coordinate(self):
        for coord in ('3760.0', '9100.0', '-0100.0'):
            with self.subTest(coord=coord), self.assertRaises(SentenceError):
                decode(sentence(BODIES['GGA'].replace('3723.932530', coord)))

    def test_serial_fragmentation_and_combined_sentences(self):
        stream = b'noise\x00' + b''.join(sentence(body) for body in BODIES.values())
        framer = Framer()
        result = []
        for byte in stream:
            result.extend(framer.feed(bytes([byte])))
        self.assertEqual([decode(s)[0] for s in result], list(BODIES))
        self.assertEqual(len(list(Framer().feed(stream))), 8)

    def test_bounded_resynchronization(self):
        framer = Framer()
        data = b'$' + b'a' * 2000 + b'\r\n$broken' + sentence(BODIES['HDT'])
        self.assertEqual([decode(s)[0] for s in framer.feed(data)], ['HDT'])
        self.assertEqual(len(framer.buffer), 0)


class TransportTest(unittest.TestCase):
    def test_udp_localhost_multiple_and_unterminated(self):
        receiver = UDPReceiver('127.0.0.1', 0, source_ip='127.0.0.1')
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sender.sendto(sentence(BODIES['HDT']) + sentence(BODIES['XDR']).rstrip(),
                          receiver.socket.getsockname())
            self.assertEqual([decode(s)[0] for s in receiver.read()], ['HDT', 'XDR'])
            sender.sendto(b'$GPHDT,', receiver.socket.getsockname())
            for s in receiver.read():
                with self.assertRaises(SentenceError):
                    decode(s)
            sender.sendto(sentence(BODIES['ZDA']), receiver.socket.getsockname())
            self.assertEqual(decode(receiver.read()[0])[0], 'ZDA')
        finally:
            receiver.close()
            sender.close()

    def test_udp_source_filter(self):
        receiver = UDPReceiver('127.0.0.1', 0, source_ip='192.0.2.1')
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sender.sendto(sentence(BODIES['HDT']), receiver.socket.getsockname())
            self.assertEqual(receiver.read(), [])
        finally:
            receiver.close()
            sender.close()

    def test_serial_pseudo_terminal(self):
        try:
            import serial
        except ImportError:
            self.skipTest('pyserial not installed')
        master, slave = pty.openpty()
        receiver = SerialReceiver(os.ttyname(slave), 921600)
        try:
            data = sentence(BODIES['PAPIMU'])
            os.write(master, data[:15])
            self.assertEqual(receiver.read(), [])
            os.write(master, data[15:])
            result = []
            for _ in range(10):
                result.extend(receiver.read())
                if result:
                    break
            self.assertEqual(decode(result[0])[0], 'PAPIMU')
        finally:
            receiver.close()
            os.close(master)
            os.close(slave)


class PublisherTest(unittest.TestCase):
    def test_routing_and_invalid_sentence_no_publish(self):
        # ROS is not required for this routing test; use message slots from the
        # actual .msg definitions so unknown/missing field mappings fail.
        ros = types.ModuleType('rospy')
        ros.get_param = lambda name, default: default
        ros.Time = types.SimpleNamespace(now=lambda: 123)
        ros.logwarn_throttle = lambda *args: None
        pubs = {}
        class Pub:
            def __init__(self, name, cls, queue_size):
                self.values = []
                pubs[name] = self
            def publish(self, value):
                self.values.append(value)
        ros.Publisher = Pub
        msgs = types.ModuleType('anello_ros_driver.msg')
        def initialize(self):
            self.header = types.SimpleNamespace(stamp=None, frame_id=None)
        for kind in KINDS:
            schema = (ROOT / 'msg' / ('GroundGen2' + kind + '.msg')).read_text()
            slots = [s.split('#')[0].split()[1] for s in schema.splitlines() if s.split('#')[0].strip()]
            setattr(msgs, 'GroundGen2' + kind, type('GroundGen2' + kind, (), {'__slots__': slots, '__init__': initialize}))
        package = types.ModuleType('anello_ros_driver')
        package.msg = msgs
        with patch.dict(sys.modules, {'rospy': ros, 'anello_ros_driver': package, 'anello_ros_driver.msg': msgs}):
            spec = importlib.util.spec_from_file_location('ground_gen2_node', ROOT / 'scripts/ground_gen2_node.py')
            node = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(node)
            publisher = node.GroundGen2Publisher()
            for kind, body in BODIES.items():
                self.assertTrue(publisher.publish(sentence(body)))
                value = pubs['~' + kind].values[0]
                self.assertEqual(value.header.stamp, 123)
                self.assertEqual(value.header.frame_id, 'ground_gen2')
            self.assertFalse(publisher.publish(b'$GPHDT,1,T*00'))
            self.assertTrue(all(len(pub.values) == 1 for pub in pubs.values()))


if __name__ == '__main__':
    unittest.main()
