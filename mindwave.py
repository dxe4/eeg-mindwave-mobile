import codecs
import sys

import serial


class ChecksumFailed(Exception):
    pass


class MindwaveAdapter(object):
    # Allow users overriding (asic_eeg needs to be configured for it)
    waves = [
        'delta', 'theta', 'low_alpha', 'high_alpha',
        'low_beta', 'high_beta', 'low_gamma', 'mid_gamma'
    ]

    def __init__(self, port='/dev/rfcomm0', baudrate=57600,
                 timeout=10, skip_raw_wave=True):
        self.sensor = serial.Serial(port, baudrate, timeout=10)
        self.skip_raw_wave = skip_raw_wave

        self.codes = {
            2: ('poor_signal', self.read_default_value),
            4: ('attention', self.read_default_value),
            5: ('meditation', self.read_default_value),
            16: ('blink_strength', self.read_default_value),
            80: ('raw_wave', self.raw_wave),
            83: ('asic_eeg', self.asic_eeg),
        }

        version = sys.version_info.major
        if version == 3:
            self.encode = self._encode_v3
        elif version == 2:
            self.encode = self._encode_v2
        else:
            raise Exception('TODO, not sure if this breaks on < 2.7')

    def _encode_v2(self, input_bytes):
        return input_bytes.encode('hex')

    def _encode_v3(self, input_bytes):
        result = codecs.encode(input_bytes, 'hex')
        result = str(result, 'ascii')
        return result

    def _read(self, bytes_leng, to_int=False):
        result = self.sensor.read(bytes_leng)
        result = self.encode(result)

        if to_int:
            return int(result, 16)
        else:
            return result

    def _read_packet(self, length):
        result = []
        checksum = 0

        for i in range(0, length):
            value = self._read(1)
            checksum += int(value, 16)
            result.append(value)

        checksum = checksum & 0xFF
        checksum = ~checksum & 0xFF
        expected_checksum = self._read(1, to_int=True)

        if checksum != expected_checksum:
            msg = 'Wrong checksum {} != {} skipping'.format(
                checksum, expected_checksum
            )
            print(msg)
            raise ChecksumFailed()
        else:
            return result

    def _wait_for_ready_state(self):
        '''
        This reads the sync bytes.
        For every packet the first 2 bytes are "aa" and "aa"
        Indicating that we are ready to use the packet
        '''
        while self.sensor.isOpen():
            a = self._read(1)
            b = self._read(1)
            if a != 'aa' or b != 'aa':
                a = b
                b = self._read(1)
            else:
                return True

    def _big_endian(self, byte_input):
        return byte_input[0] * 2 ** 16 + byte_input[1] * 2 ** 8 + byte_input[2]

    def read_default_value(self, payload, i):
        i += 1
        return int(payload[i], 16), i + 1

    def raw_wave(self, payload, i):
        length = int(payload[i + 1])
        if length != 2:
            # The length should always be 2
            raise ValueError('Raw wave must be lenght of 2')

        start = i + 2
        end = i + 2 + length

        a, b = [int(k, 16) for k in payload[start:end]]
        value = a * 256 + b

        if value >= 32768:
            value -= 65536

        return value, end + 1

    def asic_eeg(self, payload, i):
        i = i + 1
        # According to the docs the lenght is 24
        # 3 bytes per wave (listed above)
        # This values look ok with the payload, but the length given is 18
        # So for now the length is ignored and 24 is used
        length = int(payload[i])
        length = 24
        i = i + 1

        result = {}
        start = i
        end = i + 3

        for count in range(0, 8):
            wave = self.waves[count]
            numbers = [int(num, 16) for num in payload[start:end]]

            start = end
            end = start + 3
            value = self._big_endian(numbers)
            result[wave] = value

        return result, i + length

    def get_payload_values(self, payload, length):
        result = {}
        i = 0

        while i < length:
            code = payload[i]
            func_name, func = self.codes[int(code)]
            value, i, = func(payload, i)
            result[func_name] = value

        return result

    def values(self):
        sensor = self.sensor

        while sensor.isOpen():
            self._wait_for_ready_state()
            length = self._read(1, to_int=True)

            if length > 169:
                # Docs: Any higher value indicates an error (PLENGTH TOO LARGE)
                print('Invalid length skipping packet ({})'.format(length))
                continue

            try:
                payload = self._read_packet(length)
                result = self.get_payload_values(payload, length)

                if self.skip_raw_wave and result.keys() == ['raw_wave']:
                    continue
                else:
                    yield result
            except ChecksumFailed:
                continue


if __name__ == '__main__':
    adapter = MindwaveAdapter()
    for value in adapter.values():
        print(value)
