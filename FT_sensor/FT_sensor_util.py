import time

class sensor_util:
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