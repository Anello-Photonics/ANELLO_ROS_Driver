#!/usr/bin/env python3
"""ROS 2 adapter for the shared Ground Gen2 INS decoder and receiver."""
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from anello_ros_driver import msg as messages
from anello_ros_driver.srv import GPSControl
from ground_gen2.gps_control import GPSControlSender
from ground_gen2.parser import KINDS, SentenceError, decode
from ground_gen2.ntrip import DEFAULTS, NtripClient
from ground_gen2.vbw import VBWInput
from ground_gen2.transport import SerialReceiver, UDPReceiver


class GroundGen2Node(Node):
    def __init__(self):
        super().__init__('ground_gen2')
        defaults = dict(transport='serial', port='/dev/ttyUSB0', baudrate=921600,
                        bind_address='0.0.0.0', udp_port=19550, multicast_group='',
                        multicast_interface='0.0.0.0', source_ip='', nmea_host='', nmea_port=19551, frame_id='ground_gen2')
        defaults.update(DEFAULTS)
        self.options = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        transport = self.options['transport'].lower()
        if transport not in ('serial', 'udp'):
            raise ValueError('transport must be serial or udp')
        if self.options['ntrip_enabled'] and transport == 'udp' and not self.options['rtcm_host']:
            raise ValueError('UDP NTRIP requires rtcm_host (INS address)')
        self.ntrip = NtripClient(self.options, self.get_logger().warning)
        self.types = {kind: getattr(messages, 'GroundGen2' + kind) for kind in KINDS}
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
                self.options['rtcm_host'], self.options['rtcm_port'],
                self.options['nmea_host'], self.options['nmea_port'])
        self.vbw = VBWInput(self.get_logger().warning)
        self.vbw_subscriber = self.create_subscription(
            messages.GroundGen2VBW, '~/vbw_input', self.submit_vbw, 1)
        self.ntrip.start()
        self.receiver = None
        self.gps_control = GPSControlSender()
        self.gps_service = self.create_service(GPSControl, '~/gps_cntrl', self.send_gps_control)
        self.retry_after = 0.0
        # Reads have a bounded 0.2 s timeout; no background thread is required.
        self.timer = self.create_timer(0.001, self.receive,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))

    def send_gps_control(self, request, response):
        response.success, response.message = self.gps_control.send(request.command)
        return response

    def submit_vbw(self, message):
        if self.options['transport'].lower() == 'udp' and not (self.options['nmea_host'] or self.options['rtcm_host']):
            self.get_logger().warning('VBW UDP input requires nmea_host or rtcm_host', throttle_duration_sec=5.0)
            return
        self.vbw.submit(message)

    def receive(self):
        import time
        if time.monotonic() < self.retry_after:
            return
        try:
            if self.receiver is None:
                self.receiver = self.factory()
                self.gps_control.receiver = self.receiver
                self.get_logger().info('Receiving Ground Gen2 INS over ' + self.options['transport'])
            for sentence in self.receiver.read():
                self.ntrip.observe(sentence)
                stamp = self.get_clock().now().to_msg()
                try:
                    kind, fields = decode(sentence)
                except SentenceError as exc:
                    self.get_logger().warning('Discarding GroundGen2 NMEA sentence: ' + str(exc),
                                              throttle_duration_sec=5.0)
                    continue
                message = self.types[kind]()
                message.header.stamp = stamp
                message.header.frame_id = self.options['frame_id']
                for name, value in fields.items():
                    setattr(message, name, value)
                self.publishers_by_kind[kind].publish(message)
            self.ntrip.drain(self.receiver)
            self.vbw.drain(self.receiver)
        except self.connection_errors as exc:
            self.get_logger().warning('GroundGen2 connection failed: ' + str(exc), throttle_duration_sec=5.0)
            self.close_receiver()
            self.retry_after = time.monotonic() + 1.0

    def close_receiver(self):
        with self.gps_control.lock:
            self.gps_control.receiver = None
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
        node = GroundGen2Node()
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
