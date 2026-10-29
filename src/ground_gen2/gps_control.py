"""Synchronous GPS control with serialized access to the active transport."""
import threading


def encode_gps_control(command):
    if type(command) is not int or command not in (0, 1, 2):
        raise ValueError('command must be 0 (disable all), 1 (enable all), or 2 (disable internal)')
    body = ('PAPGPSCTRL,%d' % command).encode('ascii')
    checksum = 0
    for byte in body:
        checksum ^= byte
    return b'$' + body + ('*%02X\r\n' % checksum).encode('ascii')


class GPSControlSender:
    def __init__(self):
        self.lock = threading.RLock()
        self.receiver = None

    def send(self, command):
        try:
            sentence = encode_gps_control(command)
        except ValueError as exc:
            return False, str(exc)
        with self.lock:
            if self.receiver is None:
                return False, 'INS transport is not connected; command was not queued'
            try:
                self.receiver.write_nmea(sentence)
            except (OSError, ValueError) as exc:
                return False, 'Transport write failed: ' + str(exc)
        return True, 'GPS control command sent'
