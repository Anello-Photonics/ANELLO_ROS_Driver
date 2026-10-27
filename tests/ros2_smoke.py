"""Run after sourcing a ROS 2 build. Launches the installed UDP node and checks all topics."""
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import sys

import rclpy
from anello_ros_driver import msg
from test_ground_gen2 import BODIES, sentence
from test_ground_gen2_ntrip import FakeCaster, rtcm, until


def main():
    # Isolate test discovery from other ROS applications on this machine.
    os.environ['ROS_DOMAIN_ID'] = '173'
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    caster = FakeCaster()
    correction_sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    correction_sink.bind(('127.0.0.1', 0))
    correction_sink.settimeout(3)
    rclpy.init()
    node = rclpy.create_node('ground_gen2_smoke_test')
    received = {}
    subscriptions = [node.create_subscription(
        getattr(msg, 'GroundGen2' + kind), '/ground_gen2/' + kind,
        lambda value, k=kind: received.__setitem__(k, value), 100) for kind in BODIES]
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    with tempfile.TemporaryFile(mode='w+') as log, tempfile.NamedTemporaryFile(mode='w+', suffix='.yaml') as config:
        command = [
            'ros2', 'launch', 'anello_ros_driver', 'ground_gen2.launch.py',
            'transport:=udp', 'bind_address:=127.0.0.1', 'udp_port:=' + str(port),
            'ntrip_enabled:=true', 'ntrip_host:=127.0.0.1', 'ntrip_port:=' + str(caster.port),
            'ntrip_mountpoint:=TEST', 'rtcm_host:=127.0.0.1',
            'rtcm_port:=' + str(correction_sink.getsockname()[1])]
        if '--config-file' in sys.argv:
            config.write('ground_gen2:\n  ros__parameters:\n')
            for argument in command[4:]:
                name, value = argument.split(':=', 1)
                config.write('    %s: %s\n' % (name, value))
            config.flush()
            # A conflicting argument checks that file values take precedence.
            command = command[:4] + ['transport:=serial', 'config_file:=' + config.name]
        process = subprocess.Popen(command,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 20
            packet = b''.join(sentence(body) for body in BODIES.values())
            while time.monotonic() < deadline and len(received) < len(BODIES):
                if process.poll() is not None:
                    raise AssertionError('ROS 2 launch exited early')
                sender.sendto(packet, ('127.0.0.1', port))
                rclpy.spin_once(node, timeout_sec=0.1)
            assert set(received) == set(BODIES), 'Missing topics: ' + str(set(BODIES) - set(received))
            assert received['PAPACC'].velocity_north_accuracy_m_s == 3.0
            assert received['PAPIMU'].siphog_rate_z_deg_s == 6.6
            assert received['PAPINS'].zupt == -1
            assert all(value.header.frame_id == 'ground_gen2' for value in received.values())
            assert correction_sink.recv(2048) == rtcm(), 'RTCM routing mismatch'
            assert until(lambda: b'$GPGGA' in caster.gga), 'Caster did not receive GGA'
            print('ROS 2 launch integration: all 8 topics, NTRIP-to-UDP RTCM, and GGA feedback verified')
        except Exception:
            log.seek(0)
            print(log.read())
            raise
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            sender.close()
            correction_sink.close()
            caster.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
