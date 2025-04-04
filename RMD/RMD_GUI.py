from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QWidget, QDial, QPushButton, QTextBrowser, QCheckBox, QTextEdit
from PyQt5.QtCore import QTimer, Qt
from PyQt5 import uic
from pyqtgraph import PlotWidget  # 그래프를 위한 라이브러리
import sys
import os
print(os.getcwd())
# print(sys.path)
# sys.path.append('C:/Users/IRL/Knee rehab')
from RMD_custom import RMD# 가정한 모듈과 클래스 이름
import threading
import time
import argparse
import math
import traceback


class Motor:
    def __init__(self, port='/dev/ttyACM0'):
        self.RMD = RMD(0x141)
        self.RMD.setup('slcan', port)
        
        self.MOTOR_ID = 1
        self.ANGLE_INIT = 10
        self.VELOCITY_LIMIT = 200
        self.Desired_pos = 0.0
        self.control_check_status = False
        self.position_control_check_status = False # 작동 Main Switch
        self.sinusoidal_control_check_status = False # 작동 Main Switch
        self.position_control_activate_status = False # 작동 on/off Switch
        self.sinusoidal_control_activate_status = False # 작동 on/off Switch

        self.amplitude = 0.0
        self.period = 0.0
        self.pos_offset = 0.0
        self.RMD_timer = 0.0

        self.voltage = 0
        self.temperature = 0
        self.torque_current = 0
        self.velocity = 0
        self.angle = 0

        self.init_motor()
        self.init_acceleration()
        
    def init_motor(self):
        # 스케일링된 값들을 바이트 배열로 변환하여 전달
        response = self.RMD.read_pid()
        data = response.data
        self.kp_cur = data[2]
        self.ki_cur = data[3]
        self.kp_vel = data[4]
        self.ki_vel = data[5]
        self.kp_pos = data[6]
        self.ki_pos = data[7]
        # print(self.kp_cur, self.ki_cur)
        
        data = [
            self.RMD.byteArray(self.kp_cur, 1) ,
            self.RMD.byteArray(self.ki_cur, 1),
            self.RMD.byteArray(self.kp_pos, 1),
            self.RMD.byteArray(self.ki_pos, 1),
            self.RMD.byteArray(self.kp_vel, 1),
            self.RMD.byteArray(self.ki_vel, 1)
        ]
        # 바이트 배열을 하나의 플랫 리스트로 변환
        flat_data = [item for sublist in data for item in sublist]
        self.RMD.write_pid_ram(flat_data)
        print('initialized Motor')

    def init_acceleration(self):
        for i in range(4):
            index = self.RMD.byteArray(i, 1)
            response = self.RMD.read_acceleration(index)
            data = response.data
            acc = int.from_bytes(data[4:8], byteorder='little', signed=True)
            input_acc = self.RMD.byteArray(20000, 4)
            self.RMD.write_acceleration(index, input_acc)
        print('initialized Motor acc')

    def controller(self):
        while True:
            # print(self.control_check_status)
            try:
                self.voltage, self.temperature, self.torque_current, self.velocity, self.angle = self.RMD.status_motor()
            except:
                traceback_message = traceback.format_exc()
                print(traceback_message)
            if self.control_check_status == False:
                self.RMD.raw_motor_off()
                # pass
                


            if self.position_control_check_status == True:
                if self.position_control_activate_status == True:
                    
                    try:
                        _ = self.RMD.position_closed_loop(self.Desired_pos, self.VELOCITY_LIMIT)
                        # print('control')

                    except:
                        traceback_message = traceback.format_exc()
                        print(traceback_message)
                else:
                    self.RMD.raw_motor_off()
                    
            if self.sinusoidal_control_check_status == True:
                if self.sinusoidal_control_activate_status == True:
                    try:
                        sine_time = time.time() - self.RMD_timer
                        pos = self.amplitude * math.sin(self.period * sine_time) + self.pos_offset
                        print(pos)
                        _ = self.RMD.position_closed_loop(pos, self.VELOCITY_LIMIT)
                        

                    
                    except:
                        traceback_message = traceback.format_exc()
                        print(traceback_message)
                else:
                    self.RMD.raw_motor_off()
                    # pass

            
            
            time.sleep(0.01)


class MainWindow(QMainWindow):
    def __init__(self, RMD=None):
        QMainWindow.__init__(self)
        # self.rmd = RMD(port='COM3')  # 포트는 환경에 따라 변경
        self._RMD = RMD
        self.Angle_list = []
        self.Speed_list = []
        self.threadhold = 200
        self.ui = uic.loadUi('../UI/motor.ui', self)

        self.initUI()
        # self.initTimer()
        self.show()

    def initUI(self):
        # plot 위젯 찾기
        self.Plot_Angle = self.findChild(PlotWidget, 'Plot_Angle')
        self.Plot_Velocity = self.findChild(PlotWidget, 'Plot_Velocity')
        
        self.Plot_Angle_data = self.Plot_Angle.plot(pen='r', name='Angle')
        # self.Plot_Angle.setTitle("Angle Readings")
        self.Plot_Angle.setBackground("w")
        # self.Plot_Angle.setYRange(-30, 30)
        # self.Plot_Angle.addLegend(offset=(10, 30))

        self.Plot_Velocity_data = self.Plot_Velocity.plot(pen='r', name='Velocity')
        # self.Plot_Velocity.setTitle("Velocity Readings")
        # self.plot_torque.setYRange(-3, 3)
        self.Plot_Velocity.setBackground("w")
        # self.plot_torque.addLegend(offset=(10, 30))

        # textbrowser 위젲
        self.Angle_text = self.findChild(QTextBrowser, 'angle_data')
        self.Velocity_text = self.findChild(QTextBrowser, 'velocity_data')

        ### 타이머 설정 ###
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_data)
        self.timer.start(100)  # 100ms 간격으로 업데이트
        
        # 체크 박스 정의
        self.control_on_off = self.findChild(QCheckBox, 'Control_on_off_check')
        self.position_control_check = self.findChild(QCheckBox, 'position_control_check')
        self.sinusoidal_control_check = self.findChild(QCheckBox, 'sinusoidal_control_check')

        self.control_on_off.stateChanged.connect(self.control_on_off_changed)
        self.position_control_check.setDisabled(True)
        self.sinusoidal_control_check.setDisabled(True)

        self.position_control_check.stateChanged.connect(self.position_control_checked)
        self.sinusoidal_control_check.stateChanged.connect(self.sinusoidal_control_checked)

        #textedit 정의
        self.Desired_pos = self.findChild(QTextEdit, 'desired_pos')
        self.pos_kp = self.findChild(QTextEdit, 'pos_kp')
        self.pos_ki = self.findChild(QTextEdit, 'pos_ki')
        self.vel_kp = self.findChild(QTextEdit, 'vel_kp')
        self.vel_ki = self.findChild(QTextEdit, 'vel_ki')
        self.cur_kp = self.findChild(QTextEdit, 'cur_kp')
        self.cur_ki = self.findChild(QTextEdit, 'cur_ki')

        self.pos_kp.setPlainText(f"{self._RMD.kp_pos}")
        self.pos_ki.setPlainText(f"{self._RMD.ki_pos}")
        self.vel_kp.setPlainText(f"{self._RMD.kp_vel}")
        self.vel_ki.setPlainText(f"{self._RMD.ki_vel}")
        self.cur_kp.setPlainText(f"{self._RMD.kp_cur}")
        self.cur_ki.setPlainText(f"{self._RMD.ki_cur}")


        self.amplitude = self.findChild(QTextEdit, 'amplitude')
        self.period = self.findChild(QTextEdit, 'period')
        self.pos_offset = self.findChild(QTextEdit, 'pos_offset')

        # buttons 정의

        self.system_quit_btn = self.findChild(QPushButton, 'system_quit_btn')

        self.pos_setting_btn = self.findChild(QPushButton, 'pos_setting_btn')
        self.pos_start_btn = self.findChild(QPushButton, 'pos_start_btn')
        self.pos_stop_btn = self.findChild(QPushButton, 'pos_stop_btn')

        self.sinusoidal_setting_btn = self.findChild(QPushButton, 'sinusoidal_setting_btn')
        self.sinusoidal_start_btn = self.findChild(QPushButton, 'sinusoidal_start_btn')
        self.sinusoidal_stop_btn = self.findChild(QPushButton, 'sinusoidal_stop_btn')

        self.system_quit_btn.clicked.connect(self.system_quit_btn_clicked)
        
        self.pos_setting_btn.clicked.connect(self.pos_setting_btn_clicked)
        self.pos_start_btn.clicked.connect(self.pos_start_btn_clicked)
        self.pos_stop_btn.clicked.connect(self.pos_stop_btn_clicked)
        self.sinusoidal_setting_btn.clicked.connect(self.sinusoidal_setting_btn_clicked)
        self.sinusoidal_start_btn.clicked.connect(self.sinusoidal_start_btn_clicked)
        self.sinusoidal_stop_btn.clicked.connect(self.sinusoidal_stop_btn_clicked)

        self.btn_off()

    def system_quit_btn_clicked(self):
        self._RMD.RMD.raw_motor_off()
        sys.exit()
            # print(self._RMD.position_control_check_status, self._RMD.position_control_activate_status)

    def pos_setting_btn_clicked(self):
        if self._RMD.position_control_check_status != False:
            try:
                pos = self.Desired_pos.toPlainText()
                self._RMD.Desired_pos = float(pos)
            except Exception as e:
                    print(f"Error: {e}")
            try:        
                kp_pos = self.pos_kp.toPlainText()
                ki_pos = self.pos_ki.toPlainText()
                kp_vel = self.vel_kp.toPlainText()
                ki_vel = self.vel_ki.toPlainText()
                kp_cur = self.cur_kp.toPlainText()
                ki_cur = self.cur_ki.toPlainText()
                self._RMD.kp_pos = int(kp_pos)
                self._RMD.ki_pos = int(ki_pos)
                self._RMD.kp_vel = int(kp_vel)
                self._RMD.ki_vel = int(ki_vel)
                self._RMD.kp_cur = int(kp_cur)
                self._RMD.ki_cur = int(ki_cur)
                data = [
                    self._RMD.RMD.byteArray(self._RMD.kp_cur, 1) ,
                    self._RMD.RMD.byteArray(self._RMD.ki_cur, 1),
                    self._RMD.RMD.byteArray(self._RMD.kp_pos, 1),
                    self._RMD.RMD.byteArray(self._RMD.ki_pos, 1),
                    self._RMD.RMD.byteArray(self._RMD.kp_vel, 1),
                    self._RMD.RMD.byteArray(self._RMD.ki_vel, 1)
                ]
                # 바이트 배열을 하나의 플랫 리스트로 변환
                flat_data = [item for sublist in data for item in sublist]
                self._RMD.RMD.write_pid_ram(flat_data)
                print('setting complete')
            except Exception as e:
                print(f"Error: {e}")

    def pos_start_btn_clicked(self):
        if self._RMD.position_control_check_status != False:
            self._RMD.position_control_activate_status = True
            # print(self._RMD.position_control_check_status, self._RMD.position_control_activate_status)

    def pos_stop_btn_clicked(self):
        if self._RMD.position_control_check_status != False:
            self._RMD.position_control_activate_status = False

    def sinusoidal_setting_btn_clicked(self):
        if self._RMD.sinusoidal_control_check_status != False:
            amplitude = self.amplitude.toPlainText()
            period = self.period.toPlainText()
            pos_offset = self.pos_offset.toPlainText()
            try:
                self._RMD.amplitude = float(amplitude)
                self._RMD.period = float(period)
                self._RMD.pos_offset = float(pos_offset)
            except Exception as e:
                print(f"Error: {e}")


    def sinusoidal_start_btn_clicked(self):
        if self._RMD.sinusoidal_control_check_status != False:
            self._RMD.sinusoidal_control_activate_status = True
            self._RMD.RMD_timer = time.time()

    def sinusoidal_stop_btn_clicked(self):
        if self._RMD.sinusoidal_control_check_status != False:
            self._RMD.sinusoidal_control_activate_status = False
    
    def control_on_off_changed(self, state):
        if state == Qt.Checked:
            self._RMD.control_check_status = True
            self.position_control_check.setEnabled(True)
            self.sinusoidal_control_check.setEnabled(True)
            # self.btn_on()
        else:
            self._RMD.control_check_status = False
            self._RMD.position_control_check_status = False
            self._RMD.sinusoidal_control_check_status = False
            self.position_control_check.setDisabled(True)
            self.sinusoidal_control_check.setDisabled(True)
            # self.btn_off()

    def btn_on(self):
        self.pos_setting_btn.setEnabled(True)
        self.pos_start_btn.setEnabled(True)
        self.pos_stop_btn.setEnabled(True)
    
        self.sinusoidal_setting_btn.setEnabled(True)
        self.sinusoidal_start_btn.setEnabled(True)
        self.sinusoidal_stop_btn.setEnabled(True)

    def pos_btn_on(self):
        self.pos_setting_btn.setEnabled(True)
        self.pos_start_btn.setEnabled(True)
        self.pos_stop_btn.setEnabled(True)
    
        self.sinusoidal_setting_btn.setDisabled(True)
        self.sinusoidal_start_btn.setDisabled(True)
        self.sinusoidal_stop_btn.setDisabled(True)

    def sinusoidal_btn_on(self):
        self.pos_setting_btn.setDisabled(True)
        self.pos_start_btn.setDisabled(True)
        self.pos_stop_btn.setDisabled(True)
    
        self.sinusoidal_setting_btn.setEnabled(True)
        self.sinusoidal_start_btn.setEnabled(True)
        self.sinusoidal_stop_btn.setEnabled(True)
    
    def btn_off(self):
        self.pos_setting_btn.setDisabled(True)
        self.pos_start_btn.setDisabled(True)
        self.pos_stop_btn.setDisabled(True)
    
        self.sinusoidal_setting_btn.setDisabled(True)
        self.sinusoidal_start_btn.setDisabled(True)
        self.sinusoidal_stop_btn.setDisabled(True)

    def position_control_checked(self, state):
        # state == 0 : unchecked, state == 2 : checked
        if state == Qt.Checked:
            self._RMD.position_control_check_status = True
            self._RMD.sinusoidal_control_check_status = False
            
            self.pos_btn_on()
            if self._RMD.sinusoidal_control_check_status == False and self.sinusoidal_control_check.checkState() == 2:
                self.sinusoidal_control_check.toggle()
                # print(self._RMD.position_control_check_status, self._RMD.sinusoidal_control_check_status)
                
        else:
            self.position_control_check_status = False
    
    def sinusoidal_control_checked(self, state):
        if state == Qt.Checked:
            self._RMD.sinusoidal_control_check_status = True
            self._RMD.position_control_check_status = False
            self.sinusoidal_btn_on()
            if self._RMD.position_control_check_status == False and self.position_control_check.checkState() == 2:
                self.position_control_check.toggle()
                # print(self._RMD.position_control_check_status, self._RMD.sinusoidal_control_check_status)
        else:
            self.sinusoidal_control_check_status = False

    def update_data(self):
        angle = self._RMD.angle
        velocity = self._RMD.velocity
        self.Angle_text.setText(f"{angle:.2f}")
        
        # for i, label in enumerate(self.torque_labels):
        self.Velocity_text.setText(f"{velocity:.2f}")


        ### 데이터 저장 및 그래프 업데이트 ###
        # for i in range(3):
        if len(self.Angle_list) >= self.threadhold:  # 최대 threadhold개 데이터 유지
            self.Angle_list.pop(0)
        if len(self.Speed_list) >= self.threadhold:  # 최대 threadhold개 데이터 유지
            self.Speed_list.pop(0)
        self.Angle_list.append(angle)
        self.Speed_list.append(velocity)
        self.Plot_Angle_data.setData(self.Angle_list)
        self.Plot_Velocity_data.setData(self.Speed_list)

    def initTimer(self):
    #     self.timer = QTimer(self)
    #     self.timer.timeout.connect(self.updateMotorPosition)
    #     self.timer.start(100)  # 타이머 주기를 100ms로 변경

    # def updateMotorPosition(self):
    #     position = float(self._RMD.RMD.read_multi_turns_angle())
    #     self.position_label.setText(f'Current Position: {position} degrees')
    #     print()
    #     self.dial.blockSignals(True)
    #     self.dial.setValue(int(position))
    #     self.dial.blockSignals(False)

    # def dialMoved(self, position):
    #     self._RMD.RMD.position_closed_loop(self._RMD.VELOCITY_LIMIT, position)
    #     self.position_label.setText(f'Setting Position: {position} degrees')

    # def closeEvent(self, event):
    #     self._RMD.motor_stop()
    #     self._RMD.close()
    #     super().closeEvent(event)
        pass

if __name__ == '__main__':
    app = QApplication(sys.argv)
    parser = argparse.ArgumentParser(description="Control RMD Motor via CLI")
    parser.add_argument('--port', type=str, default='/dev/ttyACM0', help='Serial port to connect to')
    args = parser.parse_args()
    motor = Motor(args.port)
    motor_thread = threading.Thread(target=motor.controller, daemon=True)
    motor_thread.start()
    main_window = MainWindow(motor)
    # main_window.show()
    sys.exit(app.exec_())
