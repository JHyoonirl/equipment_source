from rmd_x6 import RMD_X6
import signal
import time
import argparse

class RMD_MOTOR:
    def __init__(self, port='COM3'):
        self.rmd = RMD_X6(0x141)
        self.rmd.setup('slcan', port)
        print('RMD motor start')
        self.MOTOR_ID = 1
        self.ANGLE_INIT = 10
        self.VELOCITY_LIMIT = 4000
        self.ANGLE_LIMIT = 360
        self.TORQUE_LIMIT = 2

        self.kp_pos = 120
        self.ki_pos = 0
        self.kp_vel = 120
        self.ki_vel = 0

        self.init_motor()
        self.init_acceleration()
        self.status_motor()
        self.position_closed_loop(self.ANGLE_INIT)  # 90도로 회전
        self.motor_control_loop()

    def init_motor(self):
        # 스케일링된 값들을 바이트 배열로 변환하여 전달
        response = self.rmd.read_pid()
        data = response.data
        self.kp_cur = data[3]
        self.ki_cur = data[4]
        # print(self.kp_cur, self.ki_cur)
        
        data = [
            self.byteArray(self.kp_cur, 1) ,
            self.byteArray(self.ki_cur, 1),
            self.byteArray(self.kp_pos, 1),
            self.byteArray(self.ki_pos, 1),
            self.byteArray(self.kp_vel, 1),
            self.byteArray(self.ki_vel, 1)
        ]
        # 바이트 배열을 하나의 플랫 리스트로 변환
        flat_data = [item for sublist in data for item in sublist]
        self.rmd.write_pid_ram(flat_data)

    def init_acceleration(self):
        for i in range(4):
            index = self.byteArray(i, 1)
            response = self.rmd.read_acceleration(index)
            data = response.data
            acc = int.from_bytes(data[4:8], byteorder='little', signed=True)
            input_acc = self.byteArray(20000, 4)
            self.rmd.write_acceleration(index, input_acc)
    
    def status_motor(self):
        response_1 = self.rmd.read_motor_status_1()
        response_2 = self.rmd.read_motor_status_2()
        data_1 = response_1.data
        data_2 = response_2.data

        voltage = int.from_bytes(data_1[4:6], byteorder='little', signed=True)
        temperature = data_2[1]
        torque_current = int.from_bytes(data_2[2:4], byteorder='little', signed=True)
        speed = int.from_bytes(data_2[4:6], byteorder='little', signed=True)
        self.read_multi_turns_angle()
        print(f"Temperature: {temperature}°C, Torque voltage: {voltage*0.1:.1f}V, Torque current: {torque_current*0.01:.2f}A, Speed: {speed}rpm, Angle: {self.angle_current:.2f}")
        
    def position_closed_loop(self, angle):
        angle_array = [0xF4, 0x01, 0, 0x10, 0, 0]
        angle_array[0:2] = self.byteArray(self.VELOCITY_LIMIT,2)
        angle_array[2:6] = self.byteArray(int(angle * 100), 4)
        self.rmd.position_closed_loop_2(angle_array)
    
    def torque_closed_loop(self, torque):
        torque_array = [0x00, 0x00]
        torque_array[0:2] = self.byteArray(torque,2)
        self.rmd.torque_closed_loop(torque_array)

    def motor_control_loop(self):
        # torque = 0
        while True:
            current_angle = self.read_multi_turns_angle()

            if current_angle > 90:
                current_angle = 90
            elif current_angle < 0:
                current_angle = 0

            self.position_closed_loop(current_angle)  # 각도 조절
            # print(f"{current_angle:.1f}")
            self.status_motor()
            # time.sleep(0.001)
    
    def motor_stop(self):
        self.rmd.motor_stop()

    def read_multi_turns_angle(self):
        data = self.rmd.read_multi_turns_angle().data
        self.angle_current = int.from_bytes(data[4:8], byteorder='little', signed=True) * 0.01
        return self.angle_current
    
    def motion_mode_control_command_can(self):
        desired_position = 10
        desired_velocity = 10
        feedforward_torque = 10
        kp = 10
        kd = 2
        self.rmd.motion_mode_control_command_can(self.MOTOR_ID, desired_position, desired_velocity, feedforward_torque, kp, kd)

    def byteArray(self, data, size):
        return data.to_bytes(size, 'little', signed=True)

    def close(self):
        self.rmd.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Control RMD Motor via CLI")
    parser.add_argument('--port', type=str, default='COM3', help='Serial port to connect to the motor')
    args = parser.parse_args()

    motor = RMD_MOTOR(port=args.port)
    try:
        # 예시 명령, 실제 명령에 따라 조정 필요
        pass
        
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        motor.close()
        print("Motor connection closed.")

