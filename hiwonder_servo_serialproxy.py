import sys
from collections import deque
from threading import Thread, Event
import time
from hiwonder_servo_io import HiwonderServoIO

class SerialProxy:
    def __init__(self, port_name='/dev/ttyUSB0', baud_rate='115200', min_motor_id=1, max_motor_id=6, connected_ids=[], update_rate=5, fake_read=False):
        self.port_name = port_name
        self.baud_rate = baud_rate
        self.min_servo_id = min_motor_id
        self.max_servo_id = max_motor_id
        self.servos = connected_ids
        self.update_rate = update_rate
        self.fake_read = fake_read
        self.running = Event()
        self.servo_io = None
        self.servos_static_info = {}
        self.actual_rate = update_rate
        self.error_counts = {'non_fatal': 0, 'checksum': 0, 'dropped': 0}
        self.current_state = []

    def set_position(self, servo_id, position, duration):
        self.servo_io.set_position(servo_id, position, duration)

    def set_multi_position(self, id_pos_dur_list):
        for id_pos_dur in id_pos_dur_list:
            self.servo_io.set_position(id_pos_dur["id"], id_pos_dur["position"], id_pos_dur["duration"])

    def connect(self):
        try:
            self.servo_io = HiwonderServoIO(self.port_name, self.baud_rate)
            self.__find_motors()
        except Exception as e:
            print(f"Fatal Error: {e}")
            sys.exit(1)
        self.running.set()
        if self.update_rate > 0:
            Thread(target=self.__update_servo_states).start()

    def disconnect(self):
        self.running.clear()

    def __find_motors(self):
        print(f'Pinging motor IDs {self.min_servo_id} through {self.max_servo_id}...')
        if not self.servos:
            for servo_id in range(self.min_servo_id, self.max_servo_id + 1):
                if self.servo_io.ping(servo_id):
                    self.servos.append(servo_id)
        if not self.servos:
            print(f'No motors found on port {self.port_name}.')
            sys.exit(1)
        print(f'Found {len(self.servos)} motors: {self.servos}')

    def __update_servo_states(self):
        rates = deque([float(self.update_rate)] * 50, maxlen=50)
        last_time = time.time()
        while self.running.is_set():
            servo_states = []
            for servo_id in self.servos:
                try:
                    state = self.servo_io.get_feedback(servo_id, self.fake_read)
                    if state:
                        servo_states.append(state)
                except Exception as e:
                    print(f"Error reading servo {servo_id}: {e}")
            
            if servo_states:
                self.current_state = servo_states
                current_time = time.time()
                rates.append(1.0 / (current_time - last_time))
                self.actual_rate = round(sum(rates) / len(rates), 2)
                last_time = current_time
            
            time.sleep(1.0 / self.update_rate)
