from rmd_x6 import RMD_X6
import signal
import time
import keyboard

#Setup RMD with identifier
rmd = RMD_X6(0x140)
rmd.setup('slcan','COM5')

#Handle Ctrl+C
def handler(sigunm=None, frame=None):
    rmd.motor_off()
    rmd.close()
    exit()
signal.signal(signal.SIGINT, handler)

# Byte to signed int
def signedInt(data, size):
    return int.from_bytes(data.to_bytes(size, 'big'), 'big', signed=True)

# int to byte array
def byteArray(data, size):
    return data.to_bytes(size, 'little', signed=True)

# rmd torque control 부분
def torque_closed_loop(current):
    current_array=[0x64, 0]
    current_array[0:2] = byteArray(round(current*100),2)
    rmd.torque_closed_loop(current_array)

# rmd position_closed_loop_2 부분 
def position_closed_loop(angle):
    angle_array = [0xF4, 0x01, 0, 0x10, 0, 0]
    angle_array[2:6] = byteArray(angle*100,4)
    rmd.position_closed_loop_2(angle_array)

# speed_closed_loop 부분 
def speed_closed_loop(speed):
    speed_array = [0x10, 0x27, 0, 0]
    speed_array[0:4] = byteArray(speed*100,4)
    rmd.speed_closed_loop(speed_array)

# motor status 읽어오기
def read_motor_status_2():
    response = rmd.read_motor_status_2()
    data = response.data
    temperature = data[1]
    torque_current = int.from_bytes(data[2:4], byteorder='little', signed=True)
    speed = int.from_bytes(data[4:6], byteorder='little', signed=True)
   #angle = int.from_bytes(data[6:8], byteorder='little', signed=True)

    return {
        "temperature" : temperature,
        "torque_current" : torque_current*0.01,
        "speed" : speed,
        #"angle" : angle 
    }

# read_multi_turn_angle 
def read_multi_turns_angle():
    response = rmd.read_multi_turns_angle()
    data = response.data
    multi_turns_angle = int.from_bytes(data[4:7], byteorder='little', signed=True)
    
    return{
        "angle" : round(multi_turns_angle / 100)
    }

# input parameter
desired_torque = 0.5
limit_speed = 300
limit_angle = 360
limit_high_torque = 2
limit_low_torque = 0.01
counter_angle_move = 20
Kp = 0.005

# save initial angle 
angle_status = read_multi_turns_angle()
initial_angle = angle_status['angle']
previous_angle = initial_angle
print(f"initial angle is {initial_angle} degree")

Mode = int(input('Speed&Position Control: 1, Return to Initial: 2, Spring_Torque:3 \n'))

while True:
    status = read_motor_status_2()
    angle_status = read_multi_turns_angle()
    current_speed = status['speed']
    current_angle = angle_status['angle']
    current_torque = status['torque_current']
    spring_torque = Kp * (initial_angle - current_angle)

    if Mode == 1:
        print(f"Motor Speed: {status['speed']} dps", end=" ")
        print(f"Motor Angle: {angle_status['angle']} degree")
        # Speed limit control 부분
        if current_speed > limit_speed:
            print(f"Speed exceed limit.")
            Mode=4

        # Position limit control 부분
        if current_angle-initial_angle > limit_angle:
            current_direction =1 if (current_angle-initial_angle) > 0 else -1
            torque_closed_loop(0)
            print(f"Angle exceed limit. Controlling angle to {current_angle} degree")

        else:
            torque_closed_loop(desired_torque)

        #time.sleep(0.05)

    elif Mode==2:
        user_input = input("Return to initial position 'a', Off the motor 'b', Go back to Speed&Position Control 'c': \n")
        if user_input == 'a':
            position_closed_loop(initial_angle)
            print(f"Returning to initial anlge : {initial_angle} degrees")

        elif user_input == 'b':
            rmd.motor_off()
            rmd.close()
            print(f"Motor off")

        elif user_input == 'c':
            Mode = 1
        else:
            print("Ivalid input. Please enter 'a', 'b' or 'c'.")
    
    elif Mode==3:
        # Print : speed, torque, angle
        print(f"Motor Speed: {status['speed']}dps", end=" ")
        print(f"Torque Current: {status['torque_current']}A", end=" ")
        print(f"Motor Angle: {angle_status['angle']}degree")
        
        # Speed limit control 부분
        if current_speed>limit_speed:
            print(f"Speed exceed limit.")
            Mode=5

        # Torque Limt control 부분 
        if current_torque > limit_high_torque:
            Mode = 6
            print("go to Mode 6")
        else:
            torque_closed_loop(spring_torque)
    
    elif Mode==4:
        print(f"Motor Speed: {status['speed']} dps", end=" ")
        print(f"Motor Angle: {angle_status['angle']} degree")

        speed_closed_loop(limit_speed)
        #torque_closed_loop(0.1)
        print(f"Speed exceed limit. Controlling speed to {limit_speed} dps")

        if current_angle-initial_angle > limit_angle:
            current_direction =1 if (current_angle-initial_angle) > 0 else -1
            torque_closed_loop(0)
            print(f"Angle exceed limit. Controlling angle to {current_angle} degree")
            
        print(f"{current_angle}  $$$   {previous_angle}")

        if current_angle < previous_angle:
            print("go to mode 1")
            Mode=1

        previous_angle = current_angle

    elif Mode==5:
        print(f"Motor Speed: {status['speed']} dps", end=" ")
        print(f"Motor Angle: {angle_status['angle']} degree")

        speed_closed_loop(limit_speed)
        #torque_closed_loop(0.1)
        print(f"Speed exceed limit. Controlling speed to {limit_speed} dps")

        if current_angle-initial_angle > limit_angle:
            current_direction =1 if (current_angle-initial_angle) > 0 else -1
            torque_closed_loop(0)
            print(f"Angle exceed limit. Controlling angle to {current_angle} degree")
            
        print(f"{current_angle}  $$$   {previous_angle}")

        if current_angle < previous_angle:
            print("go to mode 3")
            Mode=3

        previous_angle = current_angle
    
    elif Mode==6:
        print("^^^^^^^^^^^^")
        torque_closed_loop(limit_high_torque)

        if current_torque < limit_high_torque:
            Mode = 3

    time.sleep(0.01)