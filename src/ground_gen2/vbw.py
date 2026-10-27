"""Bounded, latest-only VBW input, drained by the transport owner."""
import math
import threading
import time


def encode_vbw(message):
    fields = []
    for source in ('water', 'ground'):
        valid = getattr(message, source + '_valid')
        values = [getattr(message, source + '_' + axis + '_knots')
                  for axis in ('longitudinal', 'transverse')]
        if valid and not all(math.isfinite(v) and abs(v) <= 999.999 for v in values):
            raise ValueError('Valid VBW speeds must be finite and within +/-999.999 knots')
        fields.extend([format(v, '.3f') for v in values] if valid else ['', ''])
        fields.append('A' if valid else 'V')
    body = ('VMVBW,' + ','.join(fields)).encode('ascii')
    checksum = 0
    for byte in body:
        checksum ^= byte
    return b'$' + body + ('*%02X\r\n' % checksum).encode('ascii')


class VBWInput:
    def __init__(self, warn):
        self.warn = warn
        self.lock = threading.Lock()
        self.pending = None

    def submit(self, message):
        try:
            sentence = encode_vbw(message)
        except ValueError as exc:
            self.warn(str(exc))
            return
        with self.lock:
            self.pending = (time.monotonic(), sentence)

    def drain(self, receiver):
        with self.lock:
            pending, self.pending = self.pending, None
        # Do not replay old speed samples after reconnecting.
        if pending and time.monotonic() - pending[0] <= 1.0:
            receiver.write_nmea(pending[1])
