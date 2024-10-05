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

        enable_motor(0x01)
        print(">> Enable device1 CAN1 motor successful")
        
        rospy.sleep(2)
        self.timer1 = rospy.Timer(rospy.Duration(0.05), self.control_loop)
        
        
    def vel_cb(self, msg):
        self.vel_cmd = msg
        
    def control_loop(self, event):
        set_motor_going(0x01, 100)
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

# 以上例报文内容为 0F 00 03 E8 03 00 00 
# 其中 RPDO1 映射对象 1 为 6040_00h 其数据宽度为两个字节，则 0F 00       即为 6040_00h 的设置值。
# 其中 RPDO1 映射对象 2 为 6060_00h 其数据宽度为一个字节，则 03          即为 6060_00h 的设置值。
# 其中 RPDO1 映射对象 3 为 60FF_00h 其数据宽度为四个字节，则 E8 03 00 00 即为 60FF_00h 的设置值。

# 输入speed单位：m/s
def set_motor_going(node_id, speed):
    rpm = speed_to_rpm(speed)
    pulses = rpm_to_pulses_per_second(rpm)
    pulses16arr = pulses_10_to_16(pulses)
    
    data = (c_ubyte * 8)(0x0F, 0x00, 0x03, pulses16arr[0], pulses16arr[1], pulses16arr[2], pulses16arr[3])
    print("Set motor going: ", ' '.join(f'{byte:02X}' for byte in data))
    transmit_data(0, 0, node_id, data)

def open_and_init_device1():
    device_opened = open_device(0, 0)
    rospy.sleep(0.01)
    can_initialized = init_can(0, 0, vci_initconfig1)
    rospy.sleep(0.01)
    can_started = start_can(0, 0)
    rospy.sleep(0.01)

def enable_motor(node_id):
    data = (c_ubyte * 8)(0x01, 0x06, 0x03, 0x03, 0x00, 0x01, 0x00, 0x00)
    # data = (c_ubyte * 8)(0x01, 0x06, 0x02, 0x00, 0x00, 0x01, 0x00, 0x00)
    # data = (c_ubyte * 8)(0x01, 0x03, 0x0B, 0x00, 0x00, 0x01, 0x00, 0x00)
    crc_low, crc_high = calculate_crc(data[:6])  # CRC 计算只需要前六个字节
    data[6] = crc_low
    data[7] = crc_high

    print("Enable motor: ", ' '.join(f'{byte:02X}' for byte in data))

    transmit_data(0, 0, node_id, data)

def speed_to_rpm(speed, wheel_diameter=0.1):
    wheel_circumference = pi * wheel_diameter
    wheel_rotations_per_second = speed / wheel_circumference
    rpm = wheel_rotations_per_second * 60 * 20
    return rpm

def rpm_to_pulses_per_second(rpm, pulses_per_revolution=1000):
    rps = rpm / 60
    pulses_per_second = rps * pulses_per_revolution
    return pulses_per_second

def pulses_per_second_to_rpm(pulses_per_second, pulses_per_revolution=1000):
    rps = pulses_per_second / pulses_per_revolution
    rpm = rps * 60
    return rpm
    
# 1000 to [E8, 03, 00, 00]
def pulses_10_to_16(val):
    if val < 0:
        val = (1 << 32) + val
    byte1 = val & 0xff
    byte2 = (val >> 8) & 0xff
    byte3 = (val >> 16) & 0xff
    byte4 = (val >> 24) & 0xff
    
    res = [byte1, byte2, byte3, byte4]
    return res







if __name__ == '__main__':
    try:
        Driver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
#     print("Set speed: ", ' '.join(f'{byte:02X}' for byte in req), req)


# 写运行模式=3 601 数据帧 标准帧 8 2F 60 60 00 03 00 00 00
# 从机应答 581 数据帧 标准帧 8 60 60 60 00 00 00 00 00
# 写目标速度 601 数据帧 标准帧 8 23 FF 60 00 xx xx xx xx 写入值
# 从机应答 581 数据帧 标准帧 8 60 FF 60 00 00 00 00 00
# 写加速度 601 数据帧 标准帧 8 23 83 60 00 xx xx xx xx 写入值
# 从机应答 581 数据帧 标准帧 8 60 83 60 00 00 00 00 00
# 写减速度 601 数据帧 标准帧 8 23 84 60 00 xx xx xx xx 写入值
# 从机应答 581 数据帧 标准帧 8 60 84 60 00 00 00 00 00
# [注 1]写控制字=6 601 数据帧 标准帧 8 2B 40 60 00 06 00 00 00
# 从机应答 581 数据帧 标准帧 8 60 40 60 00 00 00 00 00
# [注 1]写控制字=7 601 数据帧 标准帧 8 2B 40 60 00 07 00 00 00
# 从机应答 581 数据帧 标准帧 8 60 40 60 00 00 00 00 00
# [注 1]写控制字=0F 601 数据帧 标准帧 8 2B 40 60 00 0F 00 00 00
# 从机应答 581 数据帧 标准帧 8 60 40 60 00 00 00 00 00

'''
SDO
'''
# 描述             帧ID 帧类型 帧格式 DLC 数据段 DATA[0]~DATA[7] 16 进制 大端模式存放
#                                       DATA[0]   DATA[1]~DATA[2]   DATA[3]   DATA[4]~DATA[7]
# 写运行模式=3      601 数据帧 标准帧  8    2F           60 60          00         03 00 00 00
# 写目标速度        601 数据帧 标准帧  8    23           FF 60          00         xx xx xx xx   写入值
# 写加速度          601 数据帧 标准帧  8    23           83 60          00         xx xx xx xx   写入值
# 写减速度          601 数据帧 标准帧  8    23           84 60          00         xx xx xx xx   写入值
# [注 1]写控制字=6  601 数据帧 标准帧  8    2B           40 60          00         06 00 00 00
# [注 1]写控制字=7  601 数据帧 标准帧  8    2B           40 60          00         07 00 00 00
# [注 1]写控制字=0F 601 数据帧 标准帧  8    2B           40 60          00         0F 00 00 00

# 写 DATA[0]~DATA[7]
# 2F  60 60  00  03 00 00 00
# 23  FF 60  00  xx xx xx xx
# 23  83 60  00  xx xx xx xx
# 23  84 60  00  xx xx xx xx
# 2B  40 60  00  06 00 00 00
# 2B  40 60  00  07 00 00 00
# 2B  40 60  00  0F 00 00 00

# 读 DATA[0]~DATA[7]
# 40  41 60  00  00 00 00 00 读状态字
# 4B  41 60  00  xx xx xx xx 从机应答
# 40  6C 60  00  00 00 00 00 读当前速度
# 43  6C 60  00  xx xx xx xx 从机应答



'''
PDO
'''
# 描述                             帧ID 帧类型 帧格式 DLC 数据段 DATA[0]~DATA[7] 16 进制 大端模式存放
#                                                        DATA[0]  DATA[1]~DATA[2]  DATA[3]  DATA[4]~DATA[7]
# 将节点设为预操作状态               000  数据帧 标准帧 2     80         01
# 设置 RPDO1 的COB_ID 且失效RPDO1   601  数据帧 标准帧 8     23         00 14         01        01 02 00 80 
# 写 RPDO1 的传输类型               601  数据帧 标准帧 8     2F         00 14         02        01 00 00 00
# RPDO1 映射个数设0                 601  数据帧 标准帧 8     2F         00 16         00        00 00 00 00
# 写 RPDO1 的映射对象1              601  数据帧 标准帧 8     23         00 16         01        10 00 40 60 
# 写 RPDO1 的映射对象2              601  数据帧 标准帧 8     23         00 16         02        08 00 60 60
# 写 RPDO1 的映射对象3              601  数据帧 标准帧 8     23         00 16         03        20 00 FF 60
# RPDO1 映射个数设3                 601  数据帧 标准帧 8     2F         00 16         00        03 00 00 00
# 设置 RPDO1 的COB_ID 且生效RPDO1   601  数据帧 标准帧 8     23         00 14         01        01 02 00 00 
# 设置 RPDO2 的COB_ID 且失效RPDO2   601  数据帧 标准帧 8     23         01 14         01        01 03 00 80
# 设置 RPDO3 的COB_ID 且失效RPDO3   601  数据帧 标准帧 8     23         02 14         01        01 04 00 80
# 设置 RPDO4 的COB_ID 且失效RPDO4   601  数据帧 标准帧 8     23         03 14         01        01 05 00 80
                                
# 设置 TPDO1 的COB_ID 且失效TPDO1   601  数据帧 标准帧 8     23         00 18         01        81 01 00 80
# 设置 TPDO1 的传输类型             601  数据帧 标准帧 8     2F         00 18         02        01 00 00 00
# TPDO1 映射个数设0                 601  数据帧 标准帧 8     2F         00 1A         00        00 00 00 00
# 写 TPDO1 的映射对象1              601  数据帧 标准帧 8     23         00 1A         01        10 00 41 60
# 写 TPDO1 的映射对象2              601  数据帧 标准帧 8     23         00 1A         02        20 00 6C 60
# TPDO1 映射个数设2                 601  数据帧 标准帧 8     2F         00 1A         00        02 00 00 00
# 设置 TPDO1 的COB_ID 且生效TPDO1   601  数据帧 标准帧 8     23         00 18         01        81 01 00 00 
# 设置 TPDO2 的COB_ID 且失效TPDO2   601  数据帧 标准帧 8     23         01 18         01        81 02 00 80
# 设置 TPDO3 的COB_ID 且失效TPDO3   601  数据帧 标准帧 8     23         02 18         01        81 03 00 80
# 设置 TPDO4 的COB_ID 且失效TPDO4   601  数据帧 标准帧 8     23         03 18         01        81 04 00 80
# 将节点设为操作状态                 000  数据帧 标准帧 2     01         01



# 描述          帧ID 帧类型 帧格式 DLC 数据段 DATA[0]~DATA[7] 16 进制 大端模式存放
# 同步帧         080 数据帧 标准帧  0 
# TPDO 返回应答  181 数据帧 标准帧  6    37 02          E8 03 00 00
# RPDO 填充报文  201 数据帧 标准帧  7    0F 00    03    E8 03 00 00

# 在 TPDO 传输类型为 1-240 时，接收到相应个数的同步帧时，发送该 TPDO 应答报文。
# 当 RPDO 传输类型为 0-240 时，只要接收到一个同步帧则将该 RPDO 最新的数据更新到应用。

# 以上例报文内容为 37 02 E8 03 00 00 
# 其中 TPDO1 映射对象 1 为 6041_00h 其数据宽度为两个字节，在 TPDO1 返回的报文中 37 02       即为 6041_00h 的值。
# 其中 TPDO1 映射对象 2 为 606C_00h 其数据宽度为四个字节，在 TPDO1 返回的报文中 E8 03 00 00 即为 606C_00h 的值。

# 以上例报文内容为 0F 00 03 E8 03 00 00 
# 其中 RPDO1 映射对象 1 为 6040_00h 其数据宽度为两个字节，则 0F 00       即为 6040_00h 的设置值。
# 其中 RPDO1 映射对象 2 为 6060_00h 其数据宽度为一个字节，则 03          即为 6060_00h 的设置值。
# 其中 RPDO1 映射对象 3 为 60FF_00h 其数据宽度为四个字节，则 E8 03 00 00 即为 60FF_00h 的设置值。