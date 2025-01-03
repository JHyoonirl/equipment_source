import sys    
import numpy as np
import random
import serial
import time
import argparse
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
from FT_sensor_util import sensor_util

class FTsensor(sensor_util):
    def __init__(self, port='COM8', baudrate=115200):
        self.ser = serial.Serial(
                    port=port,\
                    baudrate=baudrate,\
                    parity=serial.PARITY_NONE,\
                    stopbits=serial.STOPBITS_ONE,\
                    bytesize=serial.EIGHTBITS,\
                        timeout=0)
        
        self.STATUS = True

        # Data storage for plotting
        self.force_data = [[], [], []]  # Three components of force
        self.torque_data = [[], [], []]  # Three components of torque
        self.data_lock = threading.Lock()

    def calculate_checksum(self, data):
        checksum = sum(data) % 256
        return checksum

    def send_data_with_read(self, data):
        sop = bytes([0x55])
        eop = bytes([0xAA])
        checksum = self.calculate_checksum(data)
        packet = sop + bytes(data) + bytes([checksum]) + eop
        self.ser.write(packet)
        return self.ser.read(19)

    def send_data_without_read(self, data):
        sop = bytes([0x55])
        eop = bytes([0xAA])
        checksum = self.calculate_checksum(data)
        packet = sop + bytes(data) + bytes([checksum]) + eop
        self.ser.write(packet)

    def decode_received_data(self, data):
        sop_index = data.find(bytes([0x55]))
        eop_index = data.find(bytes([0xAA]))
        if sop_index == -1 or eop_index == -1 or eop_index <= sop_index:
            print("Invalid packet: Start or end marker not found correctly")
            return False

        received_packet = data[sop_index:eop_index+1]
        if len(received_packet) < 14:
            print("Incomplete packet: Expected length at least 14 bytes, received", len(received_packet))
            return False

        try:
            force_raw = [received_packet[i] << 8 | received_packet[i+1] for i in range(2, 8, 2)]
            torque_raw = [received_packet[i] << 8 | received_packet[i+1] for i in range(8, 14, 2)]

            force = [int.from_bytes(i.to_bytes(2, 'big'), "big", signed=True) / 50 for i in force_raw]
            torque = [int.from_bytes(i.to_bytes(2, 'big'), "big", signed=True) / 2000 for i in torque_raw]

            with self.data_lock:
                for i in range(3):
                    self.force_data[i].append(force[i])
                    self.torque_data[i].append(torque[i])

            return force + torque
        except IndexError as e:
            print("Error processing packet: ", e)
            return False
        except Exception as e:
            print("General error: ", e)
            return False

    def ft_sensor_init(self):
        data_bias = [0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        data_baudrate = [0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        data_filter = [0x08, 0x01, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00]
        try:
            msg = self.send_data_with_read(data_filter)
            time.sleep(0.1)
            print('filter = ', msg)
            self.send_data_without_read(data_bias)
            time.sleep(0.1)
            self.send_data_without_read(data_baudrate)
            time.sleep(0.1)
            print('FT sensor initialization successful')
            self.STATUS = True
        except:
            print('FT sensor initialization failed')
            self.STATUS = False

    def ft_sensor_receive_data(self):
        data_read = [0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        data_bias = [0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.send_data_without_read(data_bias)
        while True:
            msg = self.send_data_with_read(data_read)
            # print(msg)
            self.decode_received_data(msg)


def update_plot(frame, sensor, lines, ax):
    max_points = 300  # 최대 표시할 데이터 포인트 수
    with sensor.data_lock:
        # 각 데이터 배열에서 최신 300개의 데이터만 사용
        for i in range(3):
            if len(sensor.force_data[i]) > max_points:
                force_data_to_plot = sensor.force_data[i][-max_points:]
                torque_data_to_plot = sensor.torque_data[i][-max_points:]
                x_data = range(len(sensor.force_data[i]) - max_points, len(sensor.force_data[i]))
            else:
                force_data_to_plot = sensor.force_data[i]
                torque_data_to_plot = sensor.torque_data[i]
                x_data = range(len(sensor.force_data[i]))

            lines[i].set_data(x_data, force_data_to_plot)
            lines[i+3].set_data(x_data, torque_data_to_plot)

        # x축 범위 조정
        if len(sensor.force_data[0]) > max_points:
            for a in ax:
                a.set_xlim(len(sensor.force_data[0]) - max_points, len(sensor.force_data[0]))
        else:
            for a in ax:
                a.set_xlim(0, max(max_points, len(sensor.force_data[0])))

        for a in ax:
            a.relim()  # Recalculate limits
            a.autoscale_view()  # Autoscale view to fit data

    return lines

def run_sensor(sensor):
    try:
        sensor.ft_sensor_init()
        if sensor.STATUS:
            sensor.ft_sensor_receive_data()
    except KeyboardInterrupt:
        print("Program terminated.")
        sensor.ser.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Control FT Sensor via CLI")
    parser.add_argument('--port', type=str, default='COM8', help='Serial port to connect to')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baud rate for serial connection')
    args = parser.parse_args()

    sensor = FTsensor(port=args.port, baudrate=args.baudrate)

    fig, ax = plt.subplots(2)
    lines = []
    for i in range(3):
        lobj = ax[0].plot([], [], label=f'Force {i+1}')[0]
        lines.append(lobj)
    for i in range(3):
        lobj = ax[1].plot([], [], label=f'Torque {i+1}')[0]
        lines.append(lobj)
    for a in ax:
        a.legend(loc='upper right')

    ani = FuncAnimation(fig, update_plot, fargs=(sensor, lines, ax), interval=100)

    sensor_thread = threading.Thread(target=run_sensor, args=(sensor,))
    sensor_thread.start()

    try:
        plt.show()
    except KeyboardInterrupt:
        print("Keyboard interrupt received, exiting.")
    finally:
        sensor_thread.join()  # Ensure thread has finished execution
        if sensor.ser.is_open:
            sensor.ser.close()
        print("Program terminated and serial port closed.")

