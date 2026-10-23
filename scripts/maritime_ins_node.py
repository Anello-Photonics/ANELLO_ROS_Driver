#!/usr/bin/env python3
"""Publish each Maritime INS NMEA0183 output as its own decoded ROS topic."""
import time
import rospy
from anello_ros_driver import msg as messages
from maritime_ins.parser import KINDS, SentenceError, decode
from maritime_ins.ntrip import DEFAULTS, NtripClient
from maritime_ins.transport import SerialReceiver, UDPReceiver


class MaritimePublisher:
    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'maritime_ins')
        self.types = {kind: getattr(messages, 'Maritime' + kind) for kind in KINDS}
        self.publishers = {kind: rospy.Publisher('~' + kind, self.types[kind], queue_size=100)
                           for kind in KINDS}

    def publish(self, sentence):
        stamp = rospy.Time.now()
        try:
            kind, fields = decode(sentence)
        except SentenceError as exc:
            rospy.logwarn_throttle(5.0, 'Discarding Maritime NMEA sentence: %s', str(exc))
            return False
        message = self.types[kind]()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        for name, value in fields.items():
            setattr(message, name, value)
        self.publishers[kind].publish(message)
        return True


def main():
    rospy.init_node('maritime_ins')
    publisher = MaritimePublisher()
    transport = rospy.get_param('~transport', 'serial').lower()
    if transport not in ('serial', 'udp'):
        raise ValueError('transport must be serial or udp')
    ntrip_options = {key: rospy.get_param('~' + key, value) for key, value in DEFAULTS.items()}
    if ntrip_options['ntrip_enabled'] and transport == 'udp' and not ntrip_options['rtcm_host']:
        raise ValueError('UDP NTRIP requires rtcm_host (INS address)')
    ntrip = NtripClient(ntrip_options, rospy.logwarn)
    if transport == 'serial':
        import serial
        errors = (OSError, serial.SerialException)
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baudrate = int(rospy.get_param('~baudrate', 921600))
        factory = lambda: SerialReceiver(port, baudrate)
    else:
        errors = (OSError,)
        options = dict(bind_address=rospy.get_param('~bind_address', '0.0.0.0'),
                       port=int(rospy.get_param('~udp_port', 19550)),
                       multicast_group=rospy.get_param('~multicast_group', ''),
                       multicast_interface=rospy.get_param('~multicast_interface', '0.0.0.0'),
                       source_ip=rospy.get_param('~source_ip', ''),
                       rtcm_host=ntrip_options['rtcm_host'], rtcm_port=ntrip_options['rtcm_port'])
        factory = lambda: UDPReceiver(**options)
    ntrip.start()
    rospy.on_shutdown(ntrip.close)
    while not rospy.is_shutdown():
        receiver = None
        try:
            receiver = factory()
            rospy.loginfo('Receiving Maritime INS NMEA0183 over %s', transport)
            while not rospy.is_shutdown():
                for sentence in receiver.read():
                    publisher.publish(sentence)
                    ntrip.observe(sentence)
                ntrip.drain(receiver)
        except errors as exc:
            rospy.logwarn_throttle(5.0, 'Maritime %s connection failed: %s; retrying', transport, str(exc))
            # Wall clock retry also works while /use_sim_time has no clock.
            time.sleep(1.0)
        finally:
            if receiver is not None:
                receiver.close()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
