"""Decode the eight outputs in PX4 src/drivers/nmea0183/NMEAOutput.cpp.

This module has no ROS dependency. A checksum-valid sentence with an invalid
navigation status is still published; empty floats are NaN, empty integers -1.
"""
import math
import re

KINDS = ('GGA', 'RMC', 'HDT', 'XDR', 'ZDA', 'PAPIMU', 'PAPINS', 'PAPACC')
COUNTS = dict(zip(KINDS, (14, 11, 2, 12, 6, 18, 13, 10)))


class SentenceError(ValueError):
    """Corrupt or unsupported NMEA sentence."""


class Framer:
    """Bounded serial stream framer; resynchronize on every dollar sign."""
    def __init__(self, max_length=1024):
        self.buffer = bytearray()
        self.max_length = max_length

    def feed(self, data):
        for byte in data:
            if byte == ord('$'):
                self.buffer = bytearray(b'$')
            elif byte in (10, 13):
                if self.buffer:
                    sentence = bytes(self.buffer)
                    self.buffer.clear()
                    yield sentence
            elif self.buffer:
                if byte < 32 or byte > 126 or len(self.buffer) >= self.max_length:
                    self.buffer.clear()
                else:
                    self.buffer.append(byte)


def number(value):
    if not value:
        return float('nan')
    if not re.fullmatch(r'[+-]?(?:\d+(?:\.\d*)?|\.\d+)', value):
        raise SentenceError('Invalid numeric field: ' + value)
    result = float(value)
    if not math.isfinite(result):
        raise SentenceError('Non-finite numeric field')
    return result


def integer(value):
    if not value:
        return -1
    if not re.fullmatch(r'[+-]?\d+', value):
        raise SentenceError('Invalid integer field: ' + value)
    result = int(value)
    if not -32768 <= result <= 32767:
        raise SentenceError('Integer field outside int16 range')
    return result


def coordinate(value, hemisphere, latitude):
    if not value and not hemisphere:
        return float('nan')
    allowed = 'NS' if latitude else 'EW'
    if hemisphere not in tuple(allowed) or not value:
        raise SentenceError('Missing or invalid hemisphere/coordinate')
    raw = number(value)
    degrees = int(raw // 100)
    minutes = raw - 100 * degrees
    limit = 90 if latitude else 180
    if raw < 0 or minutes >= 60 or degrees > limit or (degrees == limit and minutes != 0):
        raise SentenceError('Coordinate out of range')
    result = degrees + minutes / 60.0
    return -result if hemisphere in 'SW' else result


def decode(sentence):
    """Return (topic suffix, decoded field dict), checking checksum and layout."""
    if isinstance(sentence, bytes):
        try:
            sentence = sentence.decode('ascii')
        except UnicodeDecodeError as exc:
            raise SentenceError('Non-ASCII sentence') from exc
    sentence = sentence.rstrip('\r\n')
    if not re.fullmatch(r'\$[^*$\r\n]+\*[0-9a-fA-F]{2}', sentence):
        raise SentenceError('Malformed sentence/checksum')
    body, check = sentence[1:].split('*')
    crc = 0
    for char in body:
        crc ^= ord(char)
    if crc != int(check, 16):
        raise SentenceError('Checksum mismatch')
    ident, *f = body.split(',')
    if ident in KINDS[5:]:
        kind = ident
    elif re.fullmatch(r'[A-Z]{2}(GGA|RMC|HDT|XDR|ZDA)', ident):
        kind = ident[2:]
    else:
        raise SentenceError('Unsupported sentence: ' + ident)
    if len(f) != COUNTS[kind]:
        raise SentenceError('%s requires %d fields, got %d' % (ident, COUNTS[kind], len(f)))
    out = {'sentence_id': ident}
    if kind == 'GGA':
        if f[9] != 'M' or f[11] != 'M':
            raise SentenceError('GGA altitude units must be meters')
        out.update(utc_time=f[0], latitude_deg=coordinate(f[1], f[2], True),
                   longitude_deg=coordinate(f[3], f[4], False), fix_quality=integer(f[5]),
                   satellites_used=integer(f[6]), hdop=number(f[7]), altitude_msl_m=number(f[8]),
                   geoid_separation_m=number(f[10]), differential_age_s=number(f[12]), station_id=f[13])
    elif kind == 'RMC':
        if f[1] not in ('A', 'V') or f[10] not in ('', 'E', 'W'):
            raise SentenceError('Invalid RMC status/direction')
        out.update(utc_time=f[0], status=f[1], latitude_deg=coordinate(f[2], f[3], True),
                   longitude_deg=coordinate(f[4], f[5], False), speed_knots=number(f[6]),
                   course_deg=number(f[7]), utc_date=f[8], magnetic_variation_deg=number(f[9]),
                   magnetic_variation_direction=f[10])
    elif kind == 'HDT':
        if f[1] != 'T':
            raise SentenceError('HDT reference must be true north')
        out['heading_deg'] = number(f[0])
    elif kind == 'XDR':
        names = {'HDG': 'heading_deg', 'ROLL': 'roll_deg', 'PITCH': 'pitch_deg'}
        seen = set()
        for i in range(0, len(f), 4):
            transducer, value, unit, name = f[i:i+4]
            if transducer != 'A' or unit != 'D' or name not in names or name in seen:
                raise SentenceError('Invalid or duplicate XDR transducer')
            seen.add(name)
            out[names[name]] = number(value)
    elif kind == 'ZDA':
        out['utc_time'] = f[0]
        out.update(zip(('day', 'month', 'year', 'local_zone_hours', 'local_zone_minutes'),
                       map(integer, f[1:])))
        out['local_zone_valid'] = bool(f[4] and f[5])
    else:
        if kind == 'PAPIMU':
            names = ('time_ms', 'sync_time_ms', 'accel_x_g', 'accel_y_g', 'accel_z_g',
                     'mems_rate_x_deg_s', 'mems_rate_y_deg_s', 'mems_rate_z_deg_s',
                     'siphog_rate_x_deg_s', 'siphog_rate_y_deg_s', 'siphog_rate_z_deg_s',
                     'magnetic_x_gauss', 'magnetic_y_gauss', 'magnetic_z_gauss', 'temperature_c',
                     'status_x', 'status_y', 'status_z')
            ints = {15, 16, 17}
        elif kind == 'PAPINS':
            names = ('time_ms', 'utc_time_s', 'status', 'latitude_deg', 'longitude_deg',
                     'altitude_ellipsoid_m', 'velocity_north_m_s', 'velocity_east_m_s',
                     'velocity_down_m_s', 'roll_deg', 'pitch_deg', 'heading_deg', 'zupt')
            ints = {2, 12}
        else:
            # Actual generator order, NOT the stale comment above GenerateNmeaApacc.
            names = ('time_ms', 'utc_time_s', 'horizontal_position_accuracy_m',
                     'vertical_position_accuracy_m', 'velocity_north_accuracy_m_s',
                     'velocity_east_accuracy_m_s', 'velocity_down_accuracy_m_s',
                     'roll_accuracy_deg', 'pitch_accuracy_deg', 'heading_accuracy_deg')
            ints = set()
        out.update((name, integer(f[i]) if i in ints else number(f[i])) for i, name in enumerate(names))
    return kind, out
