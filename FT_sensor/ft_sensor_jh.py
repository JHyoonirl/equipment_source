import serial
import time

ser = serial.Serial(
    port='COM8',\
    baudrate=115200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)
print(ser.portstr) #연결된 포트 확인.

def calculate_checksum(data):
    """
    데이터의 체크섬을 계산합니다.
    """
    checksum = sum(data) % 256
    return checksum

# def decode_FT_data(data):
#     force_raw = []
#     torque_raw = []
#     force = []
#     torque = []
#     try:
#         if sop_index != -1 and eop_index != -1:  # SOP와 EOP 모두 존재하는 경우
#             received_packet = data[sop_index:eop_index+1]  # SOP부터 EOP까지의 패킷 추출
#             # print(received_packet)
#             # print(received_packet[0])
#             force_raw.append( received_packet[2] << 8 | received_packet[3])
#             force_raw.append( received_packet[4] << 8 | received_packet[5])
#             force_raw.append( received_packet[6] << 8 | received_packet[7])
#             torque_raw.append( received_packet[8] << 8 | received_packet[9])
#             torque_raw.append( received_packet[10] << 8 | received_packet[11])
#             torque_raw.append( received_packet[12] << 8 | received_packet[13])
#             for i in range(0, 3):
#                 force_temp  = force_raw[i].to_bytes(2, 'big')
#                 torque_temp  = torque_raw[i].to_bytes(2, 'big')
#                 # [Divider] Force: 50, Torque: 2000
#                 # Note: Resolution of RFT76-HA01 is same with RFT40-SA01
#                 force.append(int.from_bytes(force_temp, "big", signed=True) / 50)
#                 torque.append(int.from_bytes(torque_temp, "big", signed=True) / 2000)
#             # 여기서 데이터 처리를 수행하거나 필요한 작업을 수행할 수 있습니다.

#     except:
#         pass

def decode_received_data(data):
    """
    수신된 데이터에서 Start of Packet (SOP)와 End of Packet (EOP)을 찾아 데이터를 출력합니다.
    """
    sop_index = data.find(bytes([0x55]))  # SOP (Start of Packet)의 인덱스 찾기
    eop_index = data.find(bytes([0xAA]))  # EOP (End of Packet)의 인덱스 찾기
    force_raw = []
    torque_raw = []
    force = []
    torque = []
    try:
        if sop_index != -1 and eop_index != -1:  # SOP와 EOP 모두 존재하는 경우
            received_packet = data[sop_index:eop_index+1]  # SOP부터 EOP까지의 패킷 추출
            # print(received_packet)
            # print(received_packet[0])
            force_raw.append( received_packet[2] << 8 | received_packet[3])
            force_raw.append( received_packet[4] << 8 | received_packet[5])
            force_raw.append( received_packet[6] << 8 | received_packet[7])
            torque_raw.append( received_packet[8] << 8 | received_packet[9])
            torque_raw.append( received_packet[10] << 8 | received_packet[11])
            torque_raw.append( received_packet[12] << 8 | received_packet[13])
            for i in range(0, 3):
                force_temp  = force_raw[i].to_bytes(2, 'big')
                torque_temp  = torque_raw[i].to_bytes(2, 'big')
                # [Divider] Force: 50, Torque: 2000
                # Note: Resolution of RFT76-HA01 is same with RFT40-SA01
                force.append(int.from_bytes(force_temp, "big", signed=True) / 50)
                torque.append(int.from_bytes(torque_temp, "big", signed=True) / 2000)
            # 여기서 데이터 처리를 수행하거나 필요한 작업을 수행할 수 있습니다.
            # print(force)
            print(torque)
        # else:
            # print("Incomplete packet received")
    except:
        pass
    
def send_data_with_read(data):
    """
    데이터를 주어진 구조에 맞게 시리얼 포트로 전송합니다.
    """
    sop = bytes([0x55])  # SOP (Start of Packet)
    eop = bytes([0xAA])  # EOP (End of Packet)

    # 데이터와 체크섬 조합
    checksum = calculate_checksum(data)
    packet = sop + bytes(data) + bytes([checksum]) + eop

    # 시리얼 포트로 전송
    ser.write(packet)
    # print("send packet: {0}".format(packet.hex()))
    sent_data = ser.read(19)
    return sent_data

def send_data_without_read(data):
    """
    데이터를 주어진 구조에 맞게 시리얼 포트로 전송합니다.
    """
    sop = bytes([0x55])  # SOP (Start of Packet)
    eop = bytes([0xAA])  # EOP (End of Packet)

    # 데이터와 체크섬 조합
    checksum = calculate_checksum(data)
    packet = sop + bytes(data) + bytes([checksum]) + eop

    # 시리얼 포트로 전송
    ser.write(packet)
        
# 메인 루프
if __name__ == "__main__":
    data_read = [0x0A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    data_bias = [0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    data_baudrate = [0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    data_filter = [0x08, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00]
    data_name = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    try:
        send_data_without_read(data_bias)
        send_data_without_read(data_baudrate)
        # print(data_bias)
        time.sleep(0.1)
        send_data_without_read(data_filter)
        time.sleep(0.1)
        
        while True:
            # Note: 만약 0x0B(Command ID) 일 경우 Continuous data 이므로 while문 밖으로 빼기
            # name_msg = send_data_with_read(data_name)
            send_data_without_read(data_read)
            read_msg = ser.read(19)
            decode_received_data(read_msg)
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("프로그램을 종료합니다.")
        ser.close()  # 시리얼 포트 닫기