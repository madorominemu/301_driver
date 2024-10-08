import time
import rospy
import serial
from std_msgs.msg import Float32MultiArray

'''
温度:
01 03 00 00 00 01 84 0A
湿度:
01 03 00 01 00 01 D5 CA
露点:
01 03 00 02 00 01 25 CA
温度 湿度 2合1:
01 03 00 00 00 02 C4 0B
温度 湿度 露点 3合1:
01 03 00 00 00 03 05 CB
'''


class GasTemp(object):
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.gas_data = []
        self.gas_cmd = {
            "1"  : "01 03 00 00 00 01 84 0A",
            "2"  : "01 03 00 01 00 01 D5 CA",
            "3"  : "01 03 00 02 00 01 25 CA",
            "12" : "01 03 00 00 00 02 C4 0B",
            "123": "01 03 00 00 00 03 05 CB"
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

        self.gas_pub = rospy.Publisher("/gas_temp", Float32MultiArray, queue_size=10)
    
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
    
    def send_modbus_command(self):
        if not self.ser or not self.ser.is_open:
            rospy.logwarn("Serial port is not open.")
            return None

        request = [1, 3, 0, 0, 0, 3]
        crc_low, crc_high = self.calculate_crc(request)
        request.append(crc_low)
        request.append(crc_high)

        self.ser.write(bytearray(request))
        time.sleep(0.2)

        if self.ser.in_waiting:
            response = self.ser.read(self.ser.in_waiting)
            response = bytearray(response)
            # print("Received response: " + ' '.join(format(x, '02X') for x in response))
            return response
        else:
            return None

    def get_data_then_format(self):
        res = self.send_modbus_command()

        # Received response: 01 03 06 01 05 01 58 00 60 6C A3
        
        if res:
            temperature = (res[3] << 8) + res[4]
            humidity = (res[5] << 8) + res[6]
            dewpoint = (res[7] << 8) + res[8]
        else:
            temperature = None
            humidity = None
            dewpoint = None

        if temperature is not None and humidity is not None and dewpoint is not None:
            temperature = temperature / 10
            humidity = humidity / 10
            dewpoint = dewpoint / 10
        else:
            temperature = 0
            humidity = 0
            dewpoint = 0

        return float(temperature), float(humidity), float(dewpoint)
    
    def publish(self):
        self.gas_data = []

        temp, hum, dew = self.get_data_then_format()
        self.gas_data.append(temp)
        self.gas_data.append(hum)
        self.gas_data.append(dew)

        msg = Float32MultiArray()
        msg.data = self.gas_data
        self.gas_pub.publish(msg)
        rospy.loginfo("Published gas data: %s" % msg.data)

    def close(self):
        if self.ser:
            self.ser.close()
            rospy.loginfo("Serial port closed")

def gas_sensor_node():
    rospy.init_node('gas_temp_node', anonymous=True)

    gas_sensor = GasTemp()

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        gas_sensor.publish()
        rate.sleep()

    gas_sensor.close()




if __name__ == '__main__':
    try:
        gas_sensor_node()
    except rospy.ROSInterruptException:
        pass
