#!/usr/bin/env python3
"""ROS 2 adapter for the shared Maritime INS decoder and receiver."""
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from anello_ros_driver import msg as messages
from maritime_ins.parser import KINDS, SentenceError, decode
from maritime_ins.ntrip import DEFAULTS, NtripClient
from maritime_ins.transport import SerialReceiver, UDPReceiver


class MaritimeNode(Node):
    def __init__(self):
        super().__init__('maritime_ins')
        defaults = dict(transport='serial', port='/dev/ttyUSB0', baudrate=921600,
                        bind_address='0.0.0.0', udp_port=19550, multicast_group='',
                        multicast_interface='0.0.0.0', source_ip='', frame_id='maritime_ins')
        defaults.update(DEFAULTS)
        self.options = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        transport = self.options['transport'].lower()
        if transport not in ('serial', 'udp'):
            raise ValueError('transport must be serial or udp')
        if self.options['ntrip_enabled'] and transport == 'udp' and not self.options['rtcm_host']:
            raise ValueError('UDP NTRIP requires rtcm_host (INS address)')
        self.ntrip = NtripClient(self.options, self.get_logger().warning)
        self.types = {kind: getattr(messages, 'Maritime' + kind) for kind in KINDS}
        self.publishers_by_kind = {kind: self.create_publisher(self.types[kind], '~/' + kind, 100)
                                   for kind in KINDS}
        if transport == 'serial':
            import serial
            self.connection_errors = (OSError, serial.SerialException)
            self.factory = lambda: SerialReceiver(self.options['port'], self.options['baudrate'])
        else:
            self.connection_errors = (OSError,)
            self.factory = lambda: UDPReceiver(
                self.options['bind_address'], self.options['udp_port'],
                self.options['multicast_group'], self.options['multicast_interface'], self.options['source_ip'],
                self.options['rtcm_host'], self.options['rtcm_port'])
        self.ntrip.start()
        self.receiver = None
        self.retry_after = 0.0
        # Reads have a bounded 0.2 s timeout; no background thread is required.
        self.timer = self.create_timer(0.001, self.receive,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self):
        import time
        if time.monotonic() < self.retry_after:
            return
        try:
            if self.receiver is None:
                self.receiver = self.factory()
                self.get_logger().info('Receiving Maritime INS over ' + self.options['transport'])
            for sentence in self.receiver.read():
                self.ntrip.observe(sentence)
                stamp = self.get_clock().now().to_msg()
                try:
                    kind, fields = decode(sentence)
                except SentenceError as exc:
                    self.get_logger().warning('Discarding Maritime NMEA sentence: ' + str(exc),
                                              throttle_duration_sec=5.0)
                    continue
                message = self.types[kind]()
                message.header.stamp = stamp
                message.header.frame_id = self.options['frame_id']
                for name, value in fields.items():
                    setattr(message, name, value)
                self.publishers_by_kind[kind].publish(message)
            self.ntrip.drain(self.receiver)
        except self.connection_errors as exc:
            self.get_logger().warning('Maritime connection failed: ' + str(exc), throttle_duration_sec=5.0)
            self.close_receiver()
            self.retry_after = time.monotonic() + 1.0

    def close_receiver(self):
        if self.receiver is not None:
            self.receiver.close()
            self.receiver = None

    def destroy_node(self):
        self.ntrip.close()
        self.close_receiver()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MaritimeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
