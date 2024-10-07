import threading
import rospy
from math import sin, cos, tan, atan, sqrt, degrees, radians, pi
from can_utils import *
from geometry_msgs.msg import Twist, Quaternion, TransformStamped



class Driver(object):
    def __init__(self):
        rospy.init_node("driver")
        rospy.Subscriber("/cmd_vel_out", Twist, self.vel_cb)
        
        self.vel_cmd = None
        self.shdj_L_info = None
        
        open_and_init_device1()
        rospy.sleep(1)
        print(">> Start device1 successful, ready to receive data")
        
        open_and_init_device2()
        rospy.sleep(1)
        print(">> Start device2 successful, ready to receive data")
        
        
        
        rospy.sleep(2)
        # self.timer1 = rospy.Timer(rospy.Duration(0.05), self.control_loop_test)
        self.timer1 = rospy.Timer(rospy.Duration(0.01), self.control_loop)

        self.get_front_rpm_loop_thread = threading.Thread(target=self.get_shdj_loop)
        self.get_front_rpm_loop_thread.start()

        rospy.on_shutdown(self.shutdown)
        
        
    def vel_cb(self, msg):
        self.vel_cmd = msg
        
    def control_loop_test(self, event):
        # set_motor_going(1, 0.02)
        # set_motor_going(2, 0.02)
        set_swing_arm_going(1, 0.05)

    def control_loop(self, event):
        if self.vel_cmd is not None:
            L = 0.52 # 履带间距0.5米
            v_max = 0.31
            v_threshold = 0.02

            V = self.vel_cmd.linear.x
            W = self.vel_cmd.angular.z
            swing_arm_speed = self.vel_cmd.angular.y

            if abs(V) < v_threshold:
                speed_L = - W * L / 2
                speed_R = W * L / 2
            else:
                speed_L = V - W * L / 2
                speed_R = V + W * L / 2

            max_speed = max(abs(speed_L), abs(speed_R))
            if max_speed > v_max:
                scale = v_max / max_speed
                speed_L *= scale
                speed_R *= scale

            set_motor_going(1, speed_L)
            set_motor_going(2, -speed_R)

            print("speed_Left: ", speed_L, " | speed_right: ", speed_R)
            
            set_swing_arm_going(0, swing_arm_speed)

            self.vel_cmd = None

    def get_shdj_loop(self):
        loop_rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            loop_rate.sleep()
            res = canDLL.VCI_Receive(VCI_USBCAN2, 0, 0, byref(rx_vci_can_obj.ADDR), 2500, 0)
            if res > 0:
                for i in range(0, res):
                    data = list(rx_vci_can_obj.STRUCT_ARRAY[i].Data)
                    can_id = hex(rx_vci_can_obj.STRUCT_ARRAY[i].ID)
                    if can_id == "0x181":
                        # with self.rpm_FL_lock:
                        print(data)
                        if data[0] == 0x37:
                            self.shdj_L_info = data

    def shutdown(self):
        close_device(0)
        rospy.loginfo("Device1 closed")



########################################################################
########################################################################
        
# 输入speed单位：m/s
def set_motor_going(can_id, speed):
    puls = speed_to_pulses_16arr(speed, 0.3, 20, 1000)
    
    data = (c_ubyte*8)(0x0F, 0x00, 0x03, puls[0], puls[1], puls[2], puls[3], 0x00)

    if can_id == 1:
        transmit_data(0, 0, 0x201, data)
        # print("Set motor1 going: ", ' '.join(f'{byte:02X}' for byte in data))
    elif can_id == 2:
        transmit_data(0, 1, 0x201, data)
        # print("Set motor2 going: ", ' '.join(f'{byte:02X}' for byte in data))
    else:
        return
    
def set_swing_arm_going(can_id, speed):
    puls = speed_to_pulses_16arr(speed, 0.04, 70, 10000)
    
    data = (c_ubyte * 8)(0x23, 0xFF, 0x60, 0x00, puls[0], puls[1], puls[2], puls[3])

    if can_id == 1:
        transmit_data(1, 0, 0x201, data)
        res = transmit_data(1, can_id, 0x201, data)
        # print("Set motor1 going: ", ' '.join(f'{byte:02X}' for byte in data))
    elif can_id == 2:
        return # 预留给控制上部分人体电机
        transmit_data(1, 1, 0x201, data)
    else:
        return
    
def open_and_init_device1():
    device_opened = open_device(0, 0)
    rospy.sleep(0.1)
    can1_initialized = init_can(0, 0, vci_initconfig1)
    rospy.sleep(0.1)
    can1_started = start_can(0, 0)
    rospy.sleep(0.1)
    
    can2_initialized = init_can(0, 1, vci_initconfig1)
    rospy.sleep(0.1)
    can2_started = start_can(0, 1)
    rospy.sleep(0.1)
    
    enable_shdj_motor(0)
    enable_shdj_motor(1)
     
def open_and_init_device2():
    device_opened = open_device(1, 0)
    rospy.sleep(0.1)
    can_initialized = init_can(1, 0, vci_initconfig2)
    rospy.sleep(0.1)
    can_started = start_can(1, 0)
    rospy.sleep(0.1)
    
    enable_kinco_motor(0)

def enable_shdj_motor(can_id):
    # 控制字顺序依次写入 00h→06h→07h→0Fh→1F 电机启动运行
    data1 = (c_ubyte*8)(0x06, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00)
    res1 = transmit_data(0, can_id, 0x201, data1)
    rospy.sleep(1)
    
    data2 = (c_ubyte*8)(0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00)
    res2 = transmit_data(0, can_id, 0x201, data2)
    rospy.sleep(1)

    data3 = (c_ubyte*8)(0x0F, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00)
    res3 = transmit_data(0, can_id, 0x201, data3)
    rospy.sleep(1)

def enable_kinco_motor(can_id):
    # 开启PDO传输
    pdo = (c_ubyte*8)(0x01, 0x01, 0, 0, 0, 0, 0, 0)
    res_pdo = transmit_data(1, can_id, 0x000, pdo)
    
    
    # 150RPM成功案例
    init_data = (c_ubyte*8)(0x0F, 0x00, 0x03, 0x00, 0x40, 0x06, 0x00, 0x00)
    init_data = (c_ubyte*8)(0x0F, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00)
    res1 = transmit_data(1, can_id, 0x201, init_data)
    
    # 设置为速度模式
    set_mode = (c_ubyte*8)(0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00)
    res1 = transmit_data(1, can_id, 0x201, set_mode)
    # 松轴
    # lose_axle = (c_ubyte*8)(0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00)
    # res2 = transmit_data(1, can_id, 0x201, lose_axle)



# 计算类函数 ############################################################
########################################################################
########################################################################

def speed_to_pulses_16arr(speed, wheel_diameter, reduction_ratio, pulses_per_revolution):
    pulses = speed_to_pulese(speed, wheel_diameter, reduction_ratio, pulses_per_revolution)
    pulses16arr = value_10_to_16(pulses)
    # print(pulses)
    # print(' '.join(f'{byte:02X}' for byte in pulses16arr))
    return pulses16arr

def pulses_16arr_to_speed(pulses_16arr, wheel_diameter, reduction_ratio, pulses_per_revolution):
    pulses = value_16_to_10(pulses_16arr)
    speed = pulses_to_speed(pulses, wheel_diameter, reduction_ratio, pulses_per_revolution)
    return speed

# 输入: 速度 轮直径 减速比 转一圈所需脉冲数
def speed_to_pulese(speed, wheel_diameter, reduction_ratio, pulses_per_revolution):
    rpm = speed_to_rpm(speed, wheel_diameter, reduction_ratio)
    pulses = rpm_to_pulses_per_second(rpm, pulses_per_revolution)
    return int(pulses)

# 输入: 脉冲数 轮直径 减速比 转一圈所需脉冲数
def pulses_to_speed(pulses_per_second, wheel_diameter, reduction_ratio, pulses_per_revolution):
    rpm = pulses_per_second_to_rpm(pulses_per_second, pulses_per_revolution)
    speed = rpm_to_speed(rpm, wheel_diameter, reduction_ratio)
    return speed

def speed_to_rpm(speed, wheel_diameter, reduction_ratio):
    wheel_circumference = pi * wheel_diameter
    wheel_rotations_per_second = speed / wheel_circumference
    rpm = wheel_rotations_per_second * 60 * reduction_ratio
    return int(rpm)

def rpm_to_speed(rpm, wheel_diameter, reduction_ratio):
    wheel_circumference = pi * wheel_diameter
    wheel_rotations_per_second = rpm / 60 / reduction_ratio
    speed = wheel_rotations_per_second * wheel_circumference
    return speed

def rpm_to_pulses_per_second(rpm, pulses_per_revolution):
    rps = rpm / 60
    pulses_per_second = rps * pulses_per_revolution
    return int(pulses_per_second)

def pulses_per_second_to_rpm(pulses_per_second, pulses_per_revolution):
    rps = pulses_per_second / pulses_per_revolution
    rpm = rps * 60
    return int(rpm)
    
# 1000 to [E8, 03, 00, 00]
def value_10_to_16(val):
    if val < 0:
        val = (1 << 32) + val
    byte1 = val & 0xff
    byte2 = (val >> 8) & 0xff
    byte3 = (val >> 16) & 0xff
    byte4 = (val >> 24) & 0xff
    
    res = [byte1, byte2, byte3, byte4]
    return res

# [E8, 03, 00, 00] to 1000
def value_16_to_10(val):
    res = (val[3] << 24) + (val[2] << 16) + (val[1] << 8) + (val[0] & 0xff)
    if res > 0x7FFFFFFF:
        res -= 0x100000000
    return res





if __name__ == '__main__':
    try:
        Driver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
