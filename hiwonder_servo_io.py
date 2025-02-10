import time
import platform
import serial
from threading import Lock

exception = None
rx_pin = 7
tx_pin = 13


class ServoState:
    def __init__(self):
        self.start_timestamp = time.time()
        self.end_timestamp = time.time()
        self.speed = 200
        self.goal = 500
        self.estimated_pos = 500

class HiwonderServoIO:
    def __init__(self, port, baudrate):
        try:
            self.serial_mutex = Lock()
            self.ser = serial.Serial(port, baudrate, timeout=0.01)
            self.port_name = port
            self.servos = {i: ServoState() for i in range(1, 7)}
        except serial.SerialException:
            raise SerialOpenError(port, baudrate)

    def close(self):
        if self.ser:
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.close()

    def __write_serial(self, data):
        time.sleep(0.0005)
        self.ser.write(data)
        time.sleep(0.00035)

    def __read_response(self, servo_id):
        data = []
        self.ser.flushInput()
        time.sleep(0.003)
        try:
            data.extend(self.ser.read(4))
            if not data[:2] == [0x55, 0x55]:
                raise Exception(f'Wrong packet prefix {data[:2]}')
            data.extend(self.ser.read(data[3] - 1))
        except Exception as e:
            raise DroppedPacketError(f'Invalid response from servo {servo_id}: {e}')
        checksum = 255 - (sum(data[2: -1]) % 256)
        if checksum != data[-1]:
            raise ChecksumError(servo_id, data, checksum)
        return data

    def read(self, servo_id, cmd):
        length = 3
        checksum = 255 - ((servo_id + length + cmd) % 256)
        packet = [0x55, 0x55, servo_id, length, cmd, checksum]
        with self.serial_mutex:
            try:
                self.__write_serial(packet)
                return self.__read_response(servo_id)
            except Exception:
                return None

    def write(self, servo_id, cmd, params):
        length = 3 + len(params)
        checksum = 255 - ((servo_id + length + cmd + sum(params)) % 256)
        packet = [0x55, 0x55, servo_id, length, cmd] + params + [checksum]
        with self.serial_mutex:
            self.__write_serial(packet)

    def ping(self, servo_id):
        return bool(self.read(servo_id, 0x01))

    def get_position(self, servo_id, fake_read=False):
        if fake_read:
            return self.servos[servo_id].goal
        response = self.read(servo_id, 0x02)
        return response[5] + (response[6] << 8) if response else None

    def get_feedback(self, servo_id, fake_read=False):
        position = self.get_position(servo_id, fake_read)
        if position:
            goal = self.servos[servo_id].goal
            error = position - goal
            timestamp = time.time()
            return {'timestamp': timestamp, 'id': servo_id, 'goal': goal, 'position': position, 'error': error}
        return None

    def set_position(self, servo_id, position, duration=None):
        servo = self.servos[servo_id]
        duration = max(0, min(30000, duration if duration is not None else 20))
        position = max(0, min(1000, position))
        servo.goal = int(position)
        loVal, hiVal = position & 0xFF, position >> 8
        loTime, hiTime = duration & 0xFF, duration >> 8
        self.write(servo_id, 0x03, [loVal, hiVal, loTime, hiTime])

    def stop(self, servo_id):
        self.write(servo_id, 0x04, [])

class SerialOpenError(Exception):
    def __init__(self, port, baud):
        self.message = f"Cannot open port '{port}' at {baud} bps"
    def __str__(self):
        return self.message

class ChecksumError(Exception):
    def __init__(self, servo_id, response, checksum):
        self.message = f'Checksum mismatch for servo {servo_id} ({response[-1]} != {checksum})'
    def __str__(self):
        return self.message

class DroppedPacketError(Exception):
    def __init__(self, message):
        self.message = message
    def __str__(self):
        return self.message
