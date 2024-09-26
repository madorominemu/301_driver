import rospy
from math import sin, cos, tan, atan, sqrt, degrees, radians, pi
from can_utils import *
from geometry_msgs.msg import Twist, Quaternion, TransformStamped



class Driver(object):
    def __init__(self):
        rospy.init_node("driver")
        rospy.Subscriber("/cmd_vel_out", Twist, self.vel_cb)
        
        self.vel_cmd = None
        
        print(">> Start to open CAN Device")
        open_and_init_device1()
        rospy.sleep(1)
        print(">> Start device1 successful")

        enable_motor(0x601)
        print(">> Enable device1 CAN1 motor successful")
        
        rospy.sleep(2)
        self.timer1 = rospy.Timer(rospy.Duration(0.05), self.control_loop)
        
        
    def vel_cb(self, msg):
        self.vel_cmd = msg
        
    def control_loop(self, event):
        set_motor_going(0x601, 100)
        # if self.vel_cmd != None:
        #     if abs(self.vel_cmd.linear.x) == 0:
        #         vel = 0
        #     else:
        #         vel = self.vel_cmd.linear.x
        #     set_motor_going(0x201, vel)
        #     self.vel_cmd = None
        
    def __del__(self):
        close_device(0)
        rospy.loginfo("Device1 closed")






########################################################################

def open_and_init_device1():
    device_opened = open_device(0, 0)
    rospy.sleep(0.01)
    can_initialized = init_can(0, 0, vci_initconfig1)
    rospy.sleep(0.01)
    can_started = start_can(0, 0)
    rospy.sleep(0.01)

def enable_motor(node_id):
    data = (c_ubyte * 8)(0x01, 0x06, 0x03, 0x03, 0x00, 0x01, 0x00, 0x00)
    crc_low, crc_high = calculate_crc(data[:6])  # CRC 计算只需要前六个字节
    data[7] = crc_low
    data[6] = crc_high

    print("Enable motor: ", ' '.join(f'{byte:02X}' for byte in data))

    transmit_data(0, 0, node_id, data)

# 给位移线速度，得出电机转速
def speed_to_rpm(speed, wheel_diameter):
    wheel_circumference = pi * wheel_diameter
    wheel_rotations_per_second = speed / wheel_circumference
    rpm = wheel_rotations_per_second * 60 * 15
    return abs(rpm)


# 输入speed单位：rpm
def set_motor_going(node_id, speed):
    # rpm = speed_to_rpm(speed, wheel_diameter=0.1)
    rpm = speed
    if speed >= 0:
        rpm_sent_to_motor = int(rpm)
    else:
        rpm_sent_to_motor = -int(rpm)
    
    low8 = rpm_sent_to_motor & 0xFF
    high8 = (rpm_sent_to_motor >> 8) & 0xFF
    
    data = (c_ubyte * 8)(0x01, 0x06, 0x06, 0x03, high8, low8, 0x00, 0x00)
    
    crc_low, crc_high = calculate_crc(data[:6])  # CRC 计算只需要前六个字节
    data[7] = crc_low
    data[6] = crc_high
    
    print("Set motor going: ", ' '.join(f'{byte:02X}' for byte in data))
    
    transmit_data(0, 0, node_id, data)



# modbus crc
def calculate_crc(data):
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




if __name__ == '__main__':
    try:
        Driver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass