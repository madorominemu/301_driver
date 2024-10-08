import threading
import rospy
from math import sin, cos, tan, atan, sqrt, degrees, radians, pi
from can_utils import *
from geometry_msgs.msg import Twist, Quaternion, TransformStamped



class Driver(object):
    def __init__(self):
        rospy.init_node("driver")
        rospy.Subscriber("/cmd_vel_out", Twist, self.vel_cb)
        
        self.kinco_can_device_id = 1
        self.kinco_swing_arm_can_id = 1        
        
        self.shdj_can_device_id = 0
        self.shdj_left_motor_can_id = 0
        self.shdj_right_motor_can_id = 1
        
        self.vel_cmd = None
        
        self.shdj_L_info = None
        self.shdj_R_info = None
        self.swing_arm_info = None
        
        
        
        
        open_and_init_device(self.shdj_can_device_id)
        rospy.sleep(1)
        open_and_init_device(self.kinco_can_device_id)
        rospy.sleep(1)
        
        
        # enable_shdj_motor(self.shdj_can_device_id, self.shdj_left_motor_can_id)
        # enable_shdj_motor(self.shdj_can_device_id, self.shdj_right_motor_can_id)
        # rospy.sleep(1)
        # print(">> Start shdj successful, ready to receive data")

        
        enable_kinco_motor(self.kinco_can_device_id, self.kinco_swing_arm_can_id)
        rospy.sleep(1)
        print(">> Start kinco successful, ready to receive data")

        
        


        
        rospy.sleep(2)
        # self.timer1 = rospy.Timer(rospy.Duration(0.01), self.control_loop_test)
        self.timer1 = rospy.Timer(rospy.Duration(0.01), self.control_loop)

        self.get_front_rpm_loop_thread_L = threading.Thread(target=self.get_shdj_L_loop, args=(self.shdj_can_device_id, self.shdj_left_motor_can_id))
        self.get_front_rpm_loop_thread_L.start()
        
        self.get_front_rpm_loop_thread_R = threading.Thread(target=self.get_shdj_R_loop, args=(self.shdj_can_device_id, self.shdj_right_motor_can_id))
        self.get_front_rpm_loop_thread_R.start()

        rospy.on_shutdown(self.shutdown)
        
        
    def vel_cb(self, msg):
        self.vel_cmd = msg
        
    def control_loop_test(self, event):
        # set_motor_going(1, 0.02)
        # set_motor_going(2, 0.02)
        # set_swing_arm_going(1, 0.05)
        self.vel_cmd = None

    def control_loop(self, event):
        if self.vel_cmd is not None:
            L = 0.52 # 履带间距0.5米
            v_max = 0.942
            v_threshold = 0.1

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

            set_motor_going(self.shdj_can_device_id, self.shdj_left_motor_can_id, speed_L)
            set_motor_going(self.shdj_can_device_id, self.shdj_right_motor_can_id, -speed_R)

            print("speed_Left: ", speed_L, " | speed_right: ", speed_R)
            
            set_swing_arm_going(self.kinco_can_device_id, self.kinco_swing_arm_can_id, swing_arm_speed)

            self.vel_cmd = None



    def get_shdj_L_loop(self, device_id, can_id):
        loop_rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            loop_rate.sleep()
            res = canDLL.VCI_Receive(VCI_USBCAN2, device_id, can_id, byref(rx_vci_can_obj.ADDR), 2500, 0)
            if res > 0:
                for i in range(0, res):
                    data = list(rx_vci_can_obj.STRUCT_ARRAY[i].Data)
                    cob_id = hex(rx_vci_can_obj.STRUCT_ARRAY[i].ID)
                    if cob_id == "0x181":
                        # with self.shdj_L_info_lock:
                        print(data)
                        if data[0] == 0x37:
                            self.shdj_L_info = data
                            
    def get_shdj_R_loop(self, device_id, can_id):
        loop_rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            loop_rate.sleep()
            res = canDLL.VCI_Receive(VCI_USBCAN2, device_id, can_id, byref(rx_vci_can_obj.ADDR), 2500, 0)
            if res > 0:
                for i in range(0, res):
                    data = list(rx_vci_can_obj.STRUCT_ARRAY[i].Data)
                    cob_id = hex(rx_vci_can_obj.STRUCT_ARRAY[i].ID)
                    if cob_id == "0x181":
                        # with self.shdj_R_info_lock:
                        print(data)
                        if data[0] == 0x37:
                            self.shdj_R_info = data
                            
    def get_kinco_swing_arm_loop(self, device_id, can_id):
        loop_rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            loop_rate.sleep()
            res = canDLL.VCI_Receive(VCI_USBCAN2, device_id, can_id, byref(rx_vci_can_obj.ADDR), 2500, 0)
            if res > 0:
                for i in range(0, res):
                    data = list(rx_vci_can_obj.STRUCT_ARRAY[i].Data)
                    cob_id = hex(rx_vci_can_obj.STRUCT_ARRAY[i].ID)
                    if cob_id == "0x181":
                        # with self.shdj_R_info_lock:
                        print(data)
                        if data[0] == 0x37:
                            self.swing_arm_info = data

    def shutdown(self):
        set_motor_going(self.shdj_can_device_id, self.shdj_left_motor_can_id, 0)
        set_motor_going(self.shdj_can_device_id, self.shdj_right_motor_can_id, 0)
        set_swing_arm_going(self.kinco_can_device_id, self.kinco_swing_arm_can_id, 0)
        rospy.sleep(1)

        close_device(self.shdj_can_device_id)
        close_device(self.kinco_can_device_id)
        rospy.loginfo("Motors stopped and CAN devices closed")



########################################################################
########################################################################
        
# 输入speed单位：m/s
def set_motor_going(device_id, can_id, speed):
    puls = speed_to_pulses_16arr(speed, 0.3, 20, 1000)
    
    data = (c_ubyte*8)(0x0F, 0x00, 0x03, puls[0], puls[1], puls[2], puls[3], 0x00)
    # print("Set motor1 going: ", ' '.join(f'{byte:02X}' for byte in data))
    transmit_data(device_id, can_id, 0x201, data)
    
def set_swing_arm_going(device_id, can_id, speed):
    puls = speed_to_pulses_16arr(speed, 0.04, 70, 10000)
    
    data = (c_ubyte*8)(0x0F, 0x00, 0x03, puls[0], puls[1], puls[2], puls[3], 0x00)
    # init_data = (c_ubyte*8)(0x0F, 0x00, 0x03, 0x00, 0x40, 0x06, 0x00, 0x00) 设置 150RPM 的成功案例数据
    transmit_data(device_id, can_id, 0x201, data)

def open_and_init_device(device_id):
    device_opened = open_device(device_id, 0)
    rospy.sleep(0.1)
    can1_initialized = init_can(device_id, 0, vci_initconfig1)
    rospy.sleep(0.1)
    can1_started = start_can(device_id, 0)
    rospy.sleep(0.1)
    
    can2_initialized = init_can(device_id, 1, vci_initconfig1)
    rospy.sleep(0.1)
    can2_started = start_can(device_id, 1)
    rospy.sleep(0.1)

def enable_shdj_motor(device_id, can_id):
    # 四横电机没有保存PDO的功能，每次开局都需要重新设置，设置如下, 共26步 至data26都为设置PDO
    data1 = (c_ubyte*8)(0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)
    res1 = transmit_data(device_id, can_id, 0x000, data1)
    rospy.sleep(0.1)
    
    data2 = (c_ubyte*8)(0x23, 0x00, 0x14, 0x01, 0x01, 0x02, 0x00, 0x00)
    res2 = transmit_data(device_id, can_id, 0x601, data2)
    rospy.sleep(0.1)
    
    data3 = (c_ubyte*8)(0x2F, 0x00, 0x14, 0x02, 0x01, 0x00, 0x00, 0x00)
    res3 = transmit_data(device_id, can_id, 0x601, data3)
    rospy.sleep(0.1)
    
    data4 = (c_ubyte*8)(0x2F, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00)
    res4 = transmit_data(device_id, can_id, 0x601, data4)
    rospy.sleep(0.1)
    
    data5 = (c_ubyte*8)(0x23, 0x00, 0x16, 0x01, 0x10, 0x00, 0x40, 0x60)
    res5 = transmit_data(device_id, can_id, 0x601, data5)
    rospy.sleep(0.1)
    
    data6 = (c_ubyte*8)(0x23, 0x00, 0x16, 0x02, 0x08, 0x00, 0x60, 0x60)
    res6 = transmit_data(device_id, can_id, 0x601, data6)
    rospy.sleep(0.1)
    
    data7 = (c_ubyte*8)(0x23, 0x00, 0x16, 0x03, 0x20, 0x00, 0xFF, 0x60)
    res7 = transmit_data(device_id, can_id, 0x601, data7)
    rospy.sleep(0.1)
    
    data8 = (c_ubyte*8)(0x2F, 0x00, 0x16, 0x00, 0x03, 0x00, 0x00, 0x00)
    res8 = transmit_data(device_id, can_id, 0x601, data8)
    rospy.sleep(0.1)
    
    data9 = (c_ubyte*8)(0x23, 0x00, 0x14, 0x01, 0x01, 0x02, 0x00, 0x00)
    res9 = transmit_data(device_id, can_id, 0x601, data9)
    rospy.sleep(0.1)
    
    data10 = (c_ubyte*8)(0x23, 0x01, 0x14, 0x01, 0x01, 0x03, 0x00, 0x80)
    res10 = transmit_data(device_id, can_id, 0x601, data10)
    rospy.sleep(0.1)
    
    data11 = (c_ubyte*8)(0x23, 0x02, 0x14, 0x01, 0x01, 0x04, 0x00, 0x80)
    res11 = transmit_data(device_id, can_id, 0x601, data11)
    rospy.sleep(0.1)
    
    data12 = (c_ubyte*8)(0x23, 0x03, 0x14, 0x01, 0x01, 0x05, 0x00, 0x80)
    res12 = transmit_data(device_id, can_id, 0x601, data12)
    rospy.sleep(0.1)
    
    data13 = (c_ubyte*8)(0x23, 0x00, 0x18, 0x01, 0x81, 0x01, 0x00, 0x80)
    res13 = transmit_data(device_id, can_id, 0x601, data13)
    rospy.sleep(0.1)
    
    data14 = (c_ubyte*8)(0x2F, 0x00, 0x18, 0x02, 0xFE, 0x00, 0x00, 0x00)
    res14 = transmit_data(device_id, can_id, 0x601, data14)
    rospy.sleep(0.1)
    
    data15 = (c_ubyte*8)(0x2B, 0x00, 0x18, 0x03, 0x00, 0x00, 0x00, 0x00)
    res15 = transmit_data(device_id, can_id, 0x601, data15)
    rospy.sleep(0.1)

    data16 = (c_ubyte*8)(0x2B, 0x00, 0x18, 0x05, 0x0A, 0x00, 0x00, 0x00)
    res16 = transmit_data(device_id, can_id, 0x601, data16)
    rospy.sleep(0.1)
    
    data17 = (c_ubyte*8)(0x2F, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00)
    res17 = transmit_data(device_id, can_id, 0x601, data17)
    rospy.sleep(0.1)

    data18 = (c_ubyte*8)(0x23, 0x00, 0x1A, 0x01, 0x10, 0x00, 0x41, 0x60)
    res18 = transmit_data(device_id, can_id, 0x601, data18)
    rospy.sleep(0.1)
    
    data19 = (c_ubyte*8)(0x23, 0x00, 0x1A, 0x02, 0x20, 0x00, 0x6C, 0x60)
    res19 = transmit_data(device_id, can_id, 0x601, data19)
    rospy.sleep(0.1)

    data20 = (c_ubyte*8)(0x23, 0x00, 0x1A, 0x03, 0x00, 0x00, 0x00, 0x00)
    res20 = transmit_data(device_id, can_id, 0x601, data20)
    rospy.sleep(0.1)

    data21 = (c_ubyte*8)(0x2F, 0x00, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00)
    res21 = transmit_data(device_id, can_id, 0x601, data21)
    rospy.sleep(0.1)

    data22 = (c_ubyte*8)(0x23, 0x00, 0x18, 0x01, 0x81, 0x01, 0x00, 0x00)
    res22 = transmit_data(device_id, can_id, 0x601, data22)
    rospy.sleep(0.1)

    data23 = (c_ubyte*8)(0x23, 0x01, 0x18, 0x01, 0x81, 0x02, 0x00, 0x80)
    res23 = transmit_data(device_id, can_id, 0x601, data23)
    rospy.sleep(0.1)

    data24 = (c_ubyte*8)(0x23, 0x02, 0x18, 0x01, 0x81, 0x03, 0x00, 0x80)
    res24 = transmit_data(device_id, can_id, 0x601, data24)
    rospy.sleep(0.1)
    
    data25 = (c_ubyte*8)(0x23, 0x03, 0x18, 0x01, 0x81, 0x04, 0x00, 0x80)
    res25 = transmit_data(device_id, can_id, 0x601, data25)
    rospy.sleep(0.1)

    data26 = (c_ubyte*8)(0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)
    res26 = transmit_data(device_id, can_id, 0x000, data26)
    rospy.sleep(0.1)

    # RPDO 和 TPDO 都设置好后才进行以下的控制字变化 到 控制速度
    # 控制字顺序依次写入 00h→06h→07h→0Fh→1F 电机启动运行
    data111 = (c_ubyte*8)(0x06, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00)
    res111 = transmit_data(device_id, can_id, 0x201, data111)
    rospy.sleep(1)
    
    data222 = (c_ubyte*8)(0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00)
    res222 = transmit_data(device_id, can_id, 0x201, data222)
    rospy.sleep(1)

    data333 = (c_ubyte*8)(0x0F, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00)
    res333 = transmit_data(device_id, can_id, 0x201, data333)
    rospy.sleep(1)

def enable_kinco_motor(device_id, can_id):
    # 开启PDO传输
    pdo = (c_ubyte*8)(0x01, 0x01, 0, 0, 0, 0, 0, 0)
    res_pdo = transmit_data(device_id, can_id, 0x000, pdo)
    rospy.sleep(1)
    
    # 设置为速度模式
    set_mode = (c_ubyte*8)(0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00)
    res1 = transmit_data(device_id, can_id, 0x201, set_mode)
    rospy.sleep(1)
    
    init_data = (c_ubyte*8)(0x0F, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00)
    res1 = transmit_data(device_id, can_id, 0x201, init_data)
    rospy.sleep(1)

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
