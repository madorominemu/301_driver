import time
import rospy
import serial
from std_msgs.msg import Float32MultiArray

'''
Addres 01: O2
Addres 02: CO
Addres 03: H2S
Addres 04: LEL

获取气体类型(改变第一个地址，切换气体):
01 03 00 69 00 01 CRCH CRCL

获取气体浓度(改变第一个地址，切换气体):
01 03 00 65 00 01 CRCH CRCL

获取气体小数点(改变第一个地址，切换气体):
01 03 00 68 00 01 CRCH CRCL
'''


class GasDetector(object):
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.gas_data = []
        self.gas_index = {
            "1": "20",
            "2": "1",
            "3": "2",
            "4": "22"
        }

        MAX_RETRIES = 10
        retry_count = 0
        while self.ser is None and retry_count < MAX_RETRIES:
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                if self.ser.is_open:
                    rospy.loginfo("Serial port opened: %s" % self.port)
                else:
                    rospy.logerr("Failed to open serial port: %s" % self.port)
            except serial.SerialException as e:
                rospy.logerr("SerialException: %s" % str(e))
                retry_count += 1
                rospy.logwarn("Retrying to connect in 5 seconds... (Attempt %d/%d)" % (retry_count, MAX_RETRIES))
        time.sleep(5)

        self.gas_pub = rospy.Publisher("/gas_detector", Float32MultiArray, queue_size=10)
    
    # modbus crc
    def calculate_crc(self, data):
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for i in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc & 0xFF, (crc >> 8) & 0xFF
    
    # 发送读气体浓度命令 (假设设备地址为1，功能码0x03，寄存器地址0x0065，读取2个寄存器)
    def send_modbus_command(self, address, function_code, start_reg, num_regs):
        if not self.ser or not self.ser.is_open:
            rospy.logwarn("Serial port is not open.")
            return None

        request = [address, function_code, (start_reg >> 8) & 0xFF, start_reg & 0xFF, (num_regs >> 8) & 0xFF, num_regs & 0xFF]
        crc_low, crc_high = self.calculate_crc(request)
        request.append(crc_low)
        request.append(crc_high)

        self.ser.write(bytearray(request))
        time.sleep(0.2)

        if self.ser.in_waiting:
            response = self.ser.read(self.ser.in_waiting)
            # print("Received response: " + ' '.join(format(x, '02X') for x in response))
            return response
        else:
            return None

    def get_one_gas(self, address):
        res = self.send_modbus_command(address, 3, 0x0065, 2)

        if res:
            concentration = (res[3] << 8) + res[4]
        else:
            concentration = None

        if concentration is not None:
            if address == 1:
                gas_val = concentration / 10
            else:
                gas_val = concentration
        else:
            gas_val = 0.0
        
        return float(gas_val)
    
    def get_all_gas_and_publish(self):
        self.gas_data = []  # 每次清空气体数据

        for address in [1, 2, 3, 4]:  # 地址1、2、3、4分别对应不同气体
            val = self.get_one_gas(address=address)
            self.gas_data.append(val)
        
        msg = Float32MultiArray()
        msg.data = self.gas_data
        self.gas_pub.publish(msg)
        rospy.loginfo("Published gas data: %s" % msg.data)

    
    def close(self):
        if self.ser:
            self.ser.close()
            rospy.loginfo("Serial port closed")

def gas_sensor_node():
    rospy.init_node('gas_detector_node', anonymous=True)

    gas_sensor = GasDetector()

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        gas_sensor.get_all_gas_and_publish()
        rate.sleep()

    gas_sensor.close()




if __name__ == '__main__':
    try:
        gas_sensor_node()
    except rospy.ROSInterruptException:
        pass
