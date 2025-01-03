# RMD-X8 Python Library
# Copyright 2022 Sanjay Sunil

import can
import os
import time


class RMD_X6:
    """
    A class to read and write on the RMD-X8 motor.

    ...

    Attributes
    ----------
    bus : type
        the can bus channel used to communicate with the motor
    identifier : type
        the motor's identifier on the can bus

    Methods
    -------
    setup():
        Setup the can bus connection.
    close():
        Close the can bus connection.
    send_cmd(data, delay):
        Send a frame data to the motor.
    read_pid():
        Read the motor's current PID parameters.
    write_pid_ram(data):
        Write PID parameters to the RAM.
    write_pid_rom(data):
        Write PID parameters to the ROM.
    read_acceleration():
        Read the motor's acceleration data.
    write_acceleration_ram(data):
        Write the acceleration to the RAM of the motor.
    read_encoder():
        Read the current position of the encoder.
    write_encoder_offset(data):
        Set the motor's encoder offset.
    write_motor_zero_rom():
        Write the current position of the motor to the ROM 
        as the motor zero position.
    read_multi_turns_angle():
        Read the multi-turn angle of the motor.
    read_single_turn_angle():
        Read the single circle angle of the motor.
    motor_off():
        Turn off the motor, while clearing the motor operating 
        status and previously received control commands.
    motor_stop():
        Stop the motor, but do not clear the operating state and 
        previously received control commands.
    motor_running():
        Resume motor operation from the motor stop command.
    read_motor_status_1():
        Reads the motor's error status, voltage, temperature and 
        other information. 
    read_motor_status_2():
        Reads the motor temperature, voltage, speed and encoder 
        position.
    read_motor_status_3():
        Reads the phase current status data of the motor.
    clear_motor_error_flag():
        Clears the error status of the currrent motor.
    torque_closed_loop(data):
        Control torque current output of the motor.
    speed_closed_loop(data):
        Control the speed of the motor.
    position_closed_loop_1(data):
        Control the position of the motor (multi-turn angle).
    position_closed_loop_2(data):
        Control the position of the motor (multi-turn angle).
    position_closed_loop_3(data):
        Control the position of the motor (single-turn angle).
    position_closed_loop_4(data):
        Control the position of the motor (single-turn angle).
    """

    def __init__(self, identifier):
        """
        Constructs all the necessary attributes for the RMDX8 object.
        """
        self.bus = None
        self.identifier = identifier

    def setup(self, bustype, channel):
        """
        Setup the can bus connection.

        Returns
        -------
        self.bus : type
            The bus used to communicate with the motor.
        """

        if bustype == 'socketcan':
            try:
                os.system(f"sudo /sbin/ip link set {channel} up type can bitrate 1000000")
                time.sleep(0.1)
            except Exception as e:
                print(e)

            try:
                bus = can.interface.Bus(bustype='socketcan_native', channel=channel)
            except OSError:
                print('err: PiCAN board was not found.')
                exit()
            except Exception as e:
                print(e)

        elif bustype == 'slcan':
            try:
                bus = can.interface.Bus(bustype='slcan', channel=channel, bitrate=1000000)
            except OSError:
                print('err: SLCAN board was not found.')
                exit()
            except Exception as e:
                print(e)
        else:
            print('err: Bus type is not provided or unsupported type')
            exit()

        self.bus = bus
        return self.bus
    
    def close(self):
        self.bus.shutdown()


    def send_cmd(self, data, delay):
        """
        Send frame data to the motor.

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.
        delay : int/float
            The time to wait after sending data to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = can.Message(arbitration_id=self.identifier,
                              data=data, is_extended_id=False)
        self.bus.send(message)
        time.sleep(delay)
        received_message = self.bus.recv()
        return received_message

    def read_pid(self):
        """
        Read the motor's current PID parameters.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)

    def write_pid_ram(self, data):
        """
        Write PID parameters to the RAM.

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x31, 0x00, data[0], data[1],
                   data[2], data[3], data[4], data[5]]
        return self.send_cmd(message, 0.001)

    def write_pid_rom(self, data):
        """
        Write PID parameters to the ROM.

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x32, 0x00, data[0], data[1],
                   data[2], data[3], data[4], data[5]]
        return self.send_cmd(message, 0.001)

    def read_acceleration(self, data):
        """
        Read the motor's acceleration data.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x42, data[0], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)

    def write_acceleration(self, index, data):
        """
        Write the acceleration to the RAM of the motor.

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x43, index[0], 0x00, 0x00,
                   data[0], data[1], data[2], data[3]]
        return self.send_cmd(message, 0.001)

    def read_encoder(self):
        """
        Read the current position of the encoder.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)

    def write_encoder_offset(self, data):
        """
        Set the motor's encoder offset.

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x91, 0x00, 0x00, 0x00,
                   0x00, 0x00, data[0], data[1]]
        return self.send_cmd(message, 0.001)

    def write_motor_zero_rom(self):
        """
        Write the current position of the motor to the ROM as the motor zero position.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x64, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)

    def read_multi_turns_angle(self):
        """
        Read the multi-turn angle of the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x92, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)

    def read_single_turn_angle(self):
        """
        Read the single circle angle of the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x94, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)

    def motor_off(self):
        """
        Turn off the motor, while clearing the motor operating status and previously received control commands.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x80, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)

    def motor_stop(self):
        """
        Stop the motor, but do not clear the operating state and previously received control commands.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x81, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)

    def motor_run(self):
        """
        Resume motor operation from the motor stop command.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x88, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)

    def read_motor_status_1(self):
        """
        Reads the motor's error status, voltage, temperature and other information.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x9A, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)

    def read_motor_status_2(self):
        """
        Reads the motor temperature, voltage, speed and encoder position.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x9C, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)

    def read_motor_status_3(self):
        """
        Reads the phase current status data of the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x9D, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)

    def clear_motor_error_flag(self):
        """
        Clears the error status of the currrent motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x9B, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)

    def torque_closed_loop(self, data):
        """
        Control torque current output of the motor.

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xA1, 0x00, 0x00, 0x00,
                   data[0], data[1], 0x00, 0x00]
        return self.send_cmd(message, 0.001)

    def speed_closed_loop(self, data):
        """
        Control the speed of the motor.

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xA2, 0x00, 0x00, 0x00,
                   data[0], data[1], data[2], data[3]]
        return self.send_cmd(message, 0.001)

    def position_closed_loop_1(self, data):
        """
        Control the position of the motor (multi-turn angle).

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xA3, 0x00, 0x00, 0x00,
                   data[0], data[1], data[2], data[3]]
        return self.send_cmd(message, 0.001)

    def position_closed_loop_2(self, data):
        """
        Control the position of the motor (multi-turn angle).

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xA4, 0x00, data[0], data[1],
                   data[2], data[3], data[4], data[5]]
        return self.send_cmd(message, 0.001)

    def position_closed_loop_3(self, data):
        """
        Control the position of the motor (single-turn angle).

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xA5, data[0], 0x00, 0x00,
                   data[1], data[2], 0x00, 0x00]
        return self.send_cmd(message, 0.001)

    def position_closed_loop_4(self, data):
        """
        Control the position of the motor (single-turn angle).

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xA6, data[0], data[1], data[2],
                   data[3], data[4], 0x00, 0x00]
        return self.send_cmd(message, 0.001)
    def motion_mode_control_command_can(self, motor_id, desired_position, desired_velocity, feedforward_torque, kp, kd):
        """
        Send a Motion Mode Control Command to the motor via CAN including desired position,
        velocity, kp, kd, and feedforward torque.

        Parameters
        ----------
        motor_id : int
            The ID of the motor.
        desired_position : float
            The desired position in radians.
        desired_velocity : float
            The desired velocity in radians per second.
        feedforward_torque : float
            The feedforward torque in Newton-meters.
        kp : float
            Proportional gain.
        kd : float
            Derivative gain.

        Returns
        -------
        received_message : list
            Frame data received from the motor after sending the command.
        """
        # Calculate the command identifier based on the motor ID.
        command_id = 0x400 + motor_id #일단 고정하여 진행
        
        # Convert and scale values
        p_des = int( ( desired_position  + 12.5 ) *65535 / 25) & 0xFFFF  # Apply scaling
        v_des = int( ( desired_velocity + 45) * 4095 / 90 ) & 0xFFF   # Apply scaling
        t_ff = int( ( feedforward_torque + 24) * 4095 / 48 ) & 0xFFF  # Apply scaling
        kp_scaled = int(kp *4095 / 500) & 0xFFF              # Apply scaling
        kd_scaled = int(kd * 4095 /5) & 0xFFF              # Apply scaling
        print(p_des, v_des, t_ff, kp_scaled, kd_scaled)

        # Prepare the data bytes according to the provided format
        message = [
            (p_des >> 8) & 0xFF,      # p_des[8-15]
            p_des & 0xFF,             # p_des[0-7]
            (v_des >> 4) & 0xFF,      # v_des[4-11]
            ((v_des & 0xF) << 4) | ((kp_scaled >> 8) & 0xF),  # v_des[0-3] + kp[8-11]
            kp_scaled & 0xFF,         # kp[0-7]
            (kd_scaled >> 4) & 0xFF,  # kd[4-11]
            ((kd_scaled & 0xF) << 4) | ((t_ff >> 8) & 0xF),   # kd[0-3] + t_ff[8-11]
            t_ff & 0xFF              # t_ff[0-7]
        ]

        # Send the command and wait for response.
        return self.send_cmd(message, 0.001)
