import math
from math import sin, cos, tan, atan, sqrt, degrees, radians, pi
import time
import threading
import rospy
import tf
from ctypes import *
from can_utils import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16, Float64MultiArray
from multiprocessing import Lock
from tf.msg import tfMessage

class Controller(object):
    def __init__(self):
        rospy.init_node("controller")
        self.mode = 0  # 0:阿克曼  1:全向模式  2:自旋转
        self.wheel_base = 1.44  # 轴距，单位：米
        self.track_width = 1.44  # 轮距，单位：米
        self.wheel_diameter = 0.61  # 车轮直径， 单位：米
        self.rear_ratio = 32  # 减速比
        self.front_trans_ratio = 0.0  #
        self.max_angle = 60  # 最大转向角度， 单位：米
        self.wheel_max_speed = 2.9943304979527703  # 单轮最大速度， 单位：米
        self.enable_tf = True  # 发布tf
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        self.tf_pub = rospy.Publisher("/tf", tfMessage, queue_size=10)
        rospy.Subscriber("/cmd_vel_out", Twist, self.vel_cb)
        rospy.Subscriber("/control_mode", Int16, self.mode_change)
        self.uart_display_pub = rospy.Publisher('/angle_and_speed_info', Float64MultiArray, queue_size=10)
        self.angle_pub = rospy.Publisher('/angle_info', Float64MultiArray, queue_size=10)  # uart display publish
        self.speed_pub = rospy.Publisher('/speed_info', Float64MultiArray, queue_size=10)  # uart display publish
        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"
        self.x = 0  # x方向初始位置
        self.y = 0  # y方向初始位置
        self.theta = 0  # 初始偏航角
        self.v_x = 0  # 小车x方向速度
        self.v_angle = 0  # 小车角速度

        self.angle_FL = 0  # 前左轮当前角度，单位：弧度
        self.angle_FR = 0  # 前右轮当前角度，单位：弧度
        self.rpm_FL = 0  # 前左轮当前rpm带偏移量, -33000 ~ +33000
        self.rpm_FR = 0  # 前右轮当前rpm带偏移量, -33000 ~ +33000
        self.rpm_RL = 0  # 后左轮当前rpm带偏移量, -33000 ~ +33000
        self.rpm_RR = 0  # 后右轮当前rpm带偏移量, -33000 ~ +33000

        self.angle_FL_info = None
        self.angle_FR_info = None
        self.angle_RL_info = None
        self.angle_RR_info = None
        self.rpm_FL_info = None
        self.rpm_FR_info = None
        self.rpm_RL_info = None
        self.rpm_RR_info = None
        
        self.turn_radius = 0  # 当前转弯半径
        self.current_angle = 0  # 当前车头偏向角度 角度: -63.43494882292199 ~ +63.43494882292199, 弧度: -1.1071487177940902 ~ +1.1071487177940902

        self.timestamp = rospy.Time.now()
        self.last_timestamp = rospy.Time.now()
        self.tf = tfMessage()
        self.tf.transforms = [0]*2
        self.tf.transforms[0] = TransformStamped()
        self.tf.transforms[0].header.frame_id = "odom"
        self.tf.transforms[0].child_frame_id = "base_link"
        self.tf.transforms[1] = TransformStamped()
        self.tf.transforms[1].header.frame_id = "base_link"
        self.tf.transforms[1].child_frame_id = "steer_link"
        self.vel_cmd = None

        
        open_and_init_device2()
        rospy.sleep(1)
        close_device(1)
        rospy.sleep(1)
        open_and_init_device2()
        rospy.sleep(1)
        self.get_front_angle_loop_thread = threading.Thread(target=self.get_front_angle_loop)
        self.get_front_angle_loop_thread.start()
        print(">> Start device2 successful")

        open_and_init_device1()
        rospy.sleep(1)
        print(">> Start device1 successful")
        self.get_front_rpm_loop_thread = threading.Thread(target=self.get_front_rpm_loop)
        self.get_front_rpm_loop_thread.start()
        self.get_rear_rpm_loop_thread = threading.Thread(target=self.get_rear_rpm_loop)
        self.get_rear_rpm_loop_thread.start()

        rospy.sleep(2)
        print(">> Ready to control")
        self.timer1 = rospy.Timer(rospy.Duration(0.033), self.control_loop)
        self.timer2 = rospy.Timer(rospy.Duration(0.01), self.status_update)

    def vel_cb(self, msg):
        self.vel_cmd = msg

    def mode_change(self, msg):
        self.mode = msg.data
        if self.mode == 0 or self.mode == 1:
            mode_0()
            rospy.sleep(2)
        elif self.mode == 2:
            mode_2()
            rospy.sleep(2)
        
    def control_loop(self, event):
        if self.vel_cmd != None:
            if self.mode == 0:
                if abs(self.vel_cmd.angular.z) == 0:
                    target_angle_FL = 0
                    target_angle_FR = 0
                    if self.vel_cmd.linear.x > self.wheel_max_speed:
                        self.vel_cmd.linear.x = self.wheel_max_speed
                    elif self.vel_cmd.linear.x < -self.wheel_max_speed:
                        self.vel_cmd.linear.x = -self.wheel_max_speed
                    vel = self.vel_cmd.linear.x
                    fl_vel = vel
                    fr_vel = vel
                    rl_vel = vel
                    rr_vel = vel
                else:
                    if self.vel_cmd.angular.z > 1.1071487177940902:
                        self.vel_cmd.angular.z = 1.1071487177940902
                    elif self.vel_cmd.angular.z < -1.1071487177940902:
                        self.vel_cmd.angular.z = -1.1071487177940902
                    target_angle = self.vel_cmd.angular.z
                    turn_radius = self.wheel_base / tan(target_angle)
                    target_angle_FL = atan(self.wheel_base / (turn_radius - self.track_width / 2))
                    target_angle_FR = atan(self.wheel_base / (turn_radius + self.track_width / 2))
                    omega = self.vel_cmd.linear.x / turn_radius
                    legal_omega = self.adjust_expected_omega_to_legal_omega(omega, turn_radius)
                    fl_vel = legal_omega * sqrt(self.wheel_base**2 + (turn_radius - self.track_width / 2)**2)
                    fr_vel = legal_omega * sqrt(self.wheel_base**2 + (turn_radius + self.track_width / 2)**2)
                    rl_vel = legal_omega * (turn_radius - self.track_width / 2)
                    rr_vel = legal_omega * (turn_radius + self.track_width / 2)
                # print("angle_FL, angle_FR: ", target_angle_FL, target_angle_FR)
                # print("speed_FL, speed_FR, speed_RL, speed_RR: ", fl_vel, fr_vel, rl_vel, rr_vel)
                # 转向
                set_motor_turning(0x603, target_angle_FL)
                set_motor_turning(0x602, target_angle_FR)
                # 直行
                if self.vel_cmd.linear.x > 0:
                    set_motor_going(0x201, abs(fl_vel))
                    set_motor_going(0x203, abs(rl_vel))
                    set_motor_going(0x202, -abs(fr_vel))
                    set_motor_going(0x204, -abs(rr_vel))
                elif self.vel_cmd.linear.x < 0:
                    set_motor_going(0x201, -abs(fl_vel))
                    set_motor_going(0x203, -abs(rl_vel))
                    set_motor_going(0x202, abs(fr_vel))
                    set_motor_going(0x204, abs(rr_vel))
                else:
                    if self.v_x == 0:
                        return
                    else:
                        set_motor_going(0x201, 0)
                        set_motor_going(0x203, 0)
                        set_motor_going(0x202, 0)
                        set_motor_going(0x204, 0)
                self.vel_cmd = None
            elif self.mode == 1:            
                if abs(self.vel_cmd.angular.z) == 0:
                    target_angle = 0
                else:
                    if self.vel_cmd.angular.z > 1.5707963267948966:
                        self.vel_cmd.angular.z = 1.5707963267948966
                    elif self.vel_cmd.angular.z < -1.5707963267948966:
                        self.vel_cmd.angular.z = -1.5707963267948966
                    target_angle = self.vel_cmd.angular.z
                if abs(self.vel_cmd.linear.x) == 0:
                    vel = 0
                else:
                    if self.vel_cmd.linear.x > self.wheel_max_speed:
                        self.vel_cmd.linear.x = self.wheel_max_speed
                    elif self.vel_cmd.linear.x < -self.wheel_max_speed:
                        self.vel_cmd.linear.x = -self.wheel_max_speed
                    vel = self.vel_cmd.linear.x
                set_motor_turning(0x603, target_angle)
                set_motor_turning(0x602, target_angle)
                set_motor_turning(0x604, target_angle)
                set_motor_turning(0x601, target_angle)
                if vel == 0:
                    if self.v_x == 0:
                        return
                    else:
                        set_motor_going(0x201, 0)
                        set_motor_going(0x203, 0)
                        set_motor_going(0x202, 0)
                        set_motor_going(0x204, 0)
                else:
                    set_motor_going(0x201, vel)
                    set_motor_going(0x203, vel)
                    set_motor_going(0x202, -vel)
                    set_motor_going(0x204, -vel)
                self.vel_cmd = None
            elif self.mode == 2:
                if self.vel_cmd.angular.z is not None:
                    R = self.wheel_base / sin(radians(45)) / 2
                    vel = self.vel_cmd.angular.z * R
                    if vel > self.wheel_max_speed:
                        vel = self.wheel_max_speed
                    elif vel < -self.wheel_max_speed:
                        vel = -self.wheel_max_speed
                else:
                    vel = 0
                
                if vel == 0:
                    if self.v_angle == 0:
                        return
                    else:
                        set_motor_going(0x201, 0)
                        set_motor_going(0x203, 0)
                        set_motor_going(0x202, 0)
                        set_motor_going(0x204, 0)
                else:
                    set_motor_going(0x201, vel)
                    set_motor_going(0x203, vel)
                    set_motor_going(0x202, vel)
                    set_motor_going(0x204, vel)
                self.vel_cmd = None
    
    def status_update(self, event):
        self.timestamp = rospy.Time.now()
        d_t = (self.timestamp - self.last_timestamp).to_sec()
        self.last_timestamp = self.timestamp
        try:
            if self.mode == 0:    # 阿克曼
                data1 = self.calculate_turnRadius_and_currentAngle(self.angle_FL_info, self.angle_FR_info, self.angle_RL_info, self.angle_RR_info)
                R = data1[0]
                current_angle = data1[1]
                data2 = self.calculate_linear_velocity(R, current_angle, self.rpm_FL_info, self.rpm_FR_info, self.rpm_RL_info, self.rpm_RR_info)
                self.v_x = data2[0]
                self.v_angle = data2[1]
                d_x     = 0        #d_t时间内x方向的位移(局部坐标系)
                d_y     = 0        #d_t时间内y方向的位移(局部坐标系)
                d_theta = 0        #d_t时间内转动的角度
                d_r     = 0        #d_t时间内运动的转弯半径
                if abs(current_angle) < radians(2):
                    d_x = self.v_x * d_t
                elif(abs(self.v_x) < 0.001):
                    d_theta = self.v_angle * d_t
                else:
                    if current_angle > 0:  # 左转
                        d_r     = R
                        d_theta = self.v_angle * d_t
                        d_x = d_r * sin(d_theta)
                        d_y = d_r * (1 - cos(d_theta))
                    else:  # 右转
                        d_r     = R
                        d_theta = -self.v_angle * d_t
                        d_x = -d_r * sin(d_theta)
                        d_y = -d_r * (1 - cos(d_theta))  # 右转时 d_y 为负
            elif self.mode == 1:    # 全向
                data1 = self.calculate_turnRadius_and_currentAngle(self.angle_FL_info, self.angle_FR_info, self.angle_RL_info, self.angle_RR_info)
                R = 0
                current_angle = data1[1]
                data2 = self.calculate_linear_velocity(R, current_angle, self.rpm_FL_info, self.rpm_FR_info, self.rpm_RL_info, self.rpm_RR_info)
                self.v_x = data2[0]
                d_x     = 0        #d_t时间内x方向的位移(局部坐标系)
                d_y     = 0        #d_t时间内y方向的位移(局部坐标系)
                d_theta = 0        #d_t时间内转动的角度
                d_r     = 0        #d_t时间内运动的转弯半径
                if abs(current_angle) < radians(1):
                    d_x = self.v_x * d_t
                elif(abs(self.v_x) < 0.001):
                    d_x = 0
                else:
                    d_r     = 0
                    d_theta = 0
                    d_x     = self.v_x * cos(current_angle) * d_t
                    d_y     = self.v_x * sin(current_angle) * d_t
            elif self.mode == 2:    # 自旋转
                data1 = self.calculate_turnRadius_and_currentAngle(self.angle_FL_info, self.angle_FR_info, self.angle_RL_info, self.angle_RR_info)
                R = self.wheel_base / sin(radians(45)) / 2
                current_angle = 0
                data2 = self.calculate_linear_velocity(R, current_angle, self.rpm_FL_info, self.rpm_FR_info, self.rpm_RL_info, self.rpm_RR_info)
                self.v_x = 0
                self.v_angle = data2[1]
                d_x     = 0        #d_t时间内x方向的位移(局部坐标系)
                d_y     = 0        #d_t时间内y方向的位移(局部坐标系)
                d_theta = 0        #d_t时间内转动的角度
                d_r     = 0        #d_t时间内运动的转弯半径
                if(abs(self.v_angle) < 0.001):
                    d_theta = 0
                else:
                    d_x     = 0                         #d_t时间内x方向的位移(局部坐标系)
                    d_y     = 0                         #d_t时间内y方向的位移(局部坐标系)
                    d_theta = -self.v_angle * d_t       #d_t时间内转动的角度
                    d_r     = 0                         #d_t时间内运动的转弯半径
            self.x += d_x * cos(self.theta) - d_y * sin(self.theta)
            self.y += d_x * sin(self.theta) + d_y * cos(self.theta)
            self.theta += d_theta
            # 里程计
            self.odom.pose.pose.position.x = self.x
            self.odom.pose.pose.position.y = self.y
            odom_quaternion = tf.transformations.quaternion_from_euler(0, 0, self.theta)
            self.odom.pose.pose.orientation.x = odom_quaternion[0]
            self.odom.pose.pose.orientation.y = odom_quaternion[1]
            self.odom.pose.pose.orientation.z = odom_quaternion[2]
            self.odom.pose.pose.orientation.w = odom_quaternion[3]
            self.odom.twist.twist.linear.x = self.v_x
            self.odom.twist.twist.linear.y = self.v_x * tan(current_angle)
            self.odom.twist.twist.angular.z = self.v_angle
            self.odom.header.stamp = rospy.Time.now()
            self.odom_pub.publish(self.odom)
            # tf
            if(self.enable_tf):
                self.tf.transforms[0].transform.translation.x = self.x
                self.tf.transforms[0].transform.translation.y = self.y
                self.tf.transforms[0].transform.translation.z = 0
                self.tf.transforms[0].transform.rotation.x = odom_quaternion[0]
                self.tf.transforms[0].transform.rotation.y = odom_quaternion[1]
                self.tf.transforms[0].transform.rotation.z = odom_quaternion[2]
                self.tf.transforms[0].transform.rotation.w = odom_quaternion[3]
                self.tf.transforms[0].header.stamp = rospy.Time.now()
                wheel_quaternion = tf.transformations.quaternion_from_euler(0, 0, current_angle)
                self.tf.transforms[1].transform.translation.x = 1.28
                self.tf.transforms[1].transform.translation.y = 0
                self.tf.transforms[1].transform.translation.z = 0
                self.tf.transforms[1].transform.rotation.x = wheel_quaternion[0]
                self.tf.transforms[1].transform.rotation.y = wheel_quaternion[1]
                self.tf.transforms[1].transform.rotation.z = wheel_quaternion[2]
                self.tf.transforms[1].transform.rotation.w = wheel_quaternion[3]
                self.tf.transforms[1].header.stamp = rospy.Time.now()
                self.tf_pub.publish(self.tf)
        except Exception as error:
            # import traceback
            # traceback.print_exc()
            # print("Error: ", error)
            pass

    def get_front_angle_loop(self):
        loop_rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            loop_rate.sleep()
            res = canDLL.VCI_Receive(VCI_USBCAN2, 1, 0, byref(rx_vci_can_obj.ADDR), 2500, 0)
            if res > 0:
                for i in range(0, res):
                    data = list(rx_vci_can_obj.STRUCT_ARRAY[i].Data)
                    can_id = hex(rx_vci_can_obj.STRUCT_ARRAY[i].ID)
                    if can_id == "0x183":
                        # with self.angle_FL_lock:
                        self.angle_FL_info = data
                    elif can_id == "0x182":
                        # with self.angle_FR_lock:
                        self.angle_FR_info = data
                    elif can_id == "0x184":
                        # with self.angle_FR_lock:
                        self.angle_RL_info = data
                    elif can_id == "0x181":
                        # with self.angle_FR_lock:
                        self.angle_RR_info = data

    def get_front_rpm_loop(self):
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
                        if data[0] == 0x1b or data[0] == 0x12:
                            self.rpm_FL_info = data
                    elif can_id == "0x182":
                        # with self.rpm_FR_lock:
                        if data[0] == 0x1b or data[0] == 0x12:
                            self.rpm_FR_info = data
    
    def get_rear_rpm_loop(self):
        loop_rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            loop_rate.sleep()
            res = canDLL.VCI_Receive(VCI_USBCAN2, 0, 1, byref(rx_vci_can_obj.ADDR), 2500, 0)
            if res > 0:
                for i in range(0, res):
                    data = list(rx_vci_can_obj.STRUCT_ARRAY[i].Data)
                    can_id = hex(rx_vci_can_obj.STRUCT_ARRAY[i].ID)
                    if can_id == "0x183":
                        # with self.rpm_RL_lock:
                        if data[0] == 0x1b or data[0] == 0x12:
                            self.rpm_RL_info = data
                    elif can_id == "0x184":
                        # with self.rpm_RR_lock:
                        if data[0] == 0x1b or data[0] == 0x12:
                            self.rpm_RR_info = data

    def adjust_expected_omega_to_legal_omega(self, omega, turn_radius):
        # self.wheel_max_speed = 2.9943304979527703
        fl_vel = omega * sqrt(self.wheel_base**2 + (turn_radius - self.track_width / 2)**2)
        fr_vel = omega * sqrt(self.wheel_base**2 + (turn_radius + self.track_width / 2)**2)
        if abs(fl_vel) >= self.wheel_max_speed:
            legal_omega = self.wheel_max_speed / sqrt(self.wheel_base**2 + (turn_radius - self.track_width / 2)**2)
        elif abs(fr_vel) >= self.wheel_max_speed:
            legal_omega = self.wheel_max_speed / sqrt(self.wheel_base**2 + (turn_radius + self.track_width / 2)**2)
        else:
            legal_omega = omega
        return legal_omega
    
    def calculate_turnRadius_and_currentAngle(self, position_data_FL, position_data_FR, position_data_RL, position_data_RR):
        # print(position_data_FL, position_data_FR)
        if position_data_FL is not None:
            pos_FL = angle16to10(position_data_FL[0:4])  # 脉冲数
            display_FL = int(pos_FL / 4915200 * 90)  # 角度
        else:
            pos_FL = 0
            display_FL = 0
        if position_data_FR is not None:
            pos_FR = angle16to10(position_data_FR[0:4])  # 脉冲数
            display_FR = int(pos_FR / 4915200 * 90)  # 角度
        else:
            pos_FR = 0
            display_FR = 0
        if position_data_RL is not None:
            pos_RL = angle16to10(position_data_RL[0:4])  # 脉冲数
            display_RL = int(pos_RL / 4915200 * 90)  # 角度
        else:
            pos_RL = 0
            display_RL = 0
        if position_data_RR is not None:
            pos_RR = angle16to10(position_data_RR[0:4])  # 脉冲数
            display_RR = int(pos_RR / 4915200 * 90)  # 角度
        else:
            pos_RR = 0
            display_RR = 0
        
        data = [0] * 4
        data[0:4] = [display_FL, display_FR, display_RL, display_RR]
        array_msg = Float64MultiArray(data=data)
        self.angle_pub.publish(array_msg) 

        if abs(pos_FL) <= 10000 or abs(pos_FR) <= 10000:
            pos_FL = pos_FR = 0
        angle_FL = radians(pos_FL / 4915200 * 90)  # 角度转换的弧度
        angle_FR = radians(pos_FR / 4915200 * 90)  # 角度转换的弧度
        if angle_FL != 0 and angle_FR != 0:
            if self.mode == 0:    # 阿克曼
                R = abs( self.wheel_base / tan(angle_FL) + self.wheel_base / tan(angle_FR) ) / 2
                if angle_FL >= 0 or angle_FR >= 0:
                    angle = atan(self.wheel_base / R)
                else:
                    angle = -atan(self.wheel_base / R)
            elif self.mode == 1:    # 全向模式
                R = 0
                angle = (angle_FL + angle_FR) / 2
            elif self.mode == 2:    # 自旋转
                R = 0
                angle = 0
        else:
            R = 0
            angle = 0
        return [R, angle]
    
    def calculate_linear_velocity(self, R, current_angle, rpm_FL_info, rpm_FR_info, rpm_RL_info, rpm_RR_info):
                
        data = [0] * 6
        # data[0:6] = [speed_FL, speed_FR, speed_RL, speed_RR, linear_velocity, self.mode]
        # data[0:6] = [1, 1, 1, 1, 1.0, self.mode]
        # array_msg = Float64MultiArray(data=data)
        # self.speed_pub.publish(array_msg)

        # print(rpm_FL_info, rpm_FR_info, rpm_RL_info, rpm_RR_info)
        rpm_data_FL = rpm_FL_info[3:5]
        rpm_data_FR = rpm_FR_info[3:5]
        rpm_data_RL = rpm_RL_info[3:5]
        rpm_data_RR = rpm_RR_info[3:5]
        rpm_FL = rpm_data_FL[0] + (rpm_data_FL[1] << 8)
        rpm_FR = rpm_data_FR[0] + (rpm_data_FR[1] << 8)
        rpm_RL = rpm_data_RL[0] + (rpm_data_RL[1] << 8)
        rpm_RR = rpm_data_RR[0] + (rpm_data_RR[1] << 8)
        # print(rpm_FL, rpm_FR, rpm_RL, rpm_RR)
        if self.mode == 0:
            real_rpm_FL = rpm_FL - 30000
            real_rpm_FR = 30000 - rpm_FR
            real_rpm_RL = rpm_RL - 30000
            real_rpm_RR = 30000 - rpm_RR
            # print("real_rpm_FL: ", real_rpm_FL, "real_rpm_FR: ", real_rpm_FR, "real_rpm_RL: ", real_rpm_RL, "real_rpm_RR: ", real_rpm_RR)

            speed_FL = rpm_to_speed(real_rpm_FL, self.wheel_diameter)
            speed_FR = rpm_to_speed(real_rpm_FR, self.wheel_diameter)
            speed_RL = rpm_to_speed(real_rpm_RL, self.wheel_diameter)
            speed_RR = rpm_to_speed(real_rpm_RR, self.wheel_diameter)
            # print("speed: ", speed_FL, speed_FR, speed_RL, speed_RR)

            

            speeds = [speed_FL, speed_FR, speed_RL, speed_RR]
            non_zero_speeds = [speed for speed in speeds if speed != 0]

            if not non_zero_speeds:
                omega_even = 0
                linear_velocity = 0
            elif abs(current_angle) < radians(2):
                speed_even = sum(non_zero_speeds) / len(non_zero_speeds)
                linear_velocity = speed_even
                omega_even = 0
            else:
                omegas = []
                omegas.append(speed_FL / sqrt(self.wheel_base**2 + (R - self.track_width / 2)**2))
                omegas.append(speed_FR / sqrt(self.wheel_base**2 + (R + self.track_width / 2)**2))
                # omegas.append(speed_RL / (R - self.track_width / 2))
                # omegas.append(speed_RR / (R + self.track_width / 2))
                omega_even = sum(omegas) / len(omegas)
                linear_velocity = omega_even * R
        elif self.mode == 1:
            real_rpm_FL = rpm_FL - 30000
            real_rpm_FR = 30000 - rpm_FR
            real_rpm_RL = rpm_RL - 30000
            real_rpm_RR = 30000 - rpm_RR
            # print("real_rpm_FL: ", real_rpm_FL, "real_rpm_FR: ", real_rpm_FR, "real_rpm_RL: ", real_rpm_RL, "real_rpm_RR: ", real_rpm_RR)

            speed_FL = rpm_to_speed(real_rpm_FL, self.wheel_diameter)
            speed_FR = rpm_to_speed(real_rpm_FR, self.wheel_diameter)
            speed_RL = rpm_to_speed(real_rpm_RL, self.wheel_diameter)
            speed_RR = rpm_to_speed(real_rpm_RR, self.wheel_diameter)
            # print("speed: ", speed_FL, speed_FR, speed_RL, speed_RR)

            speeds = [speed_FL, speed_FR, speed_RL, speed_RR]
            non_zero_speeds = [speed for speed in speeds if speed != 0]

            if not non_zero_speeds:
                linear_velocity = 0
            else:
                speed_even = sum(non_zero_speeds) / len(non_zero_speeds)
                linear_velocity = speed_even
            omega_even = 0
        elif self.mode == 2:
            real_rpm_FL = rpm_FL - 30000
            real_rpm_FR = rpm_FR - 30000
            real_rpm_RL = rpm_RL - 30000
            real_rpm_RR = rpm_RR - 30000
            # print("real_rpm_FL: ", real_rpm_FL, "real_rpm_FR: ", real_rpm_FR, "real_rpm_RL: ", real_rpm_RL, "real_rpm_RR: ", real_rpm_RR)

            speed_FL = rpm_to_speed(real_rpm_FL, self.wheel_diameter)
            speed_FR = rpm_to_speed(real_rpm_FR, self.wheel_diameter)
            speed_RL = rpm_to_speed(real_rpm_RL, self.wheel_diameter)
            speed_RR = rpm_to_speed(real_rpm_RR, self.wheel_diameter)
            # print(speed_FL, speed_FR, speed_RL, speed_RR)

            speeds = [speed_FL, speed_FR, speed_RL, speed_RR]
            non_zero_speeds = [speed for speed in speeds if speed != 0]

            if not non_zero_speeds:
                linear_velocity = 0
            else:
                speed_even = sum(non_zero_speeds) / len(non_zero_speeds)
                linear_velocity = speed_even
            omega_even = linear_velocity / R
        else:
            speed_FL = 0
            speed_FR = 0
            speed_RL = 0
            speed_RR = 0
            linear_velocity = 0
            omega_even = 0

        data[0:6] = [round(speed_FL, 2), round(speed_FR, 2), round(speed_RL, 2), round(speed_RR, 2), round(linear_velocity, 2), self.mode]
        array_msg = Float64MultiArray(data=data)
        self.speed_pub.publish(array_msg)
        print(data)
        
        return [linear_velocity, omega_even]

    def __del__(self):
        close_device(0)
        close_device(1)
        rospy.loginfo("Device closed")






##################################################
def init_a_motor(node_id):
    pdo = (c_ubyte*8)(0x01, (node_id - 0x600), 0, 0, 0, 0, 0, 0)
    res1 = transmit_data(1, 0, 0x000, pdo)
    rospy.sleep(0.01)

    # 通道1发送数据 控制字 2F 使能, 06去使能
    a2 = (c_ubyte*8)(0x2B, 0x40, 0x60, 0x00, 0x2F, 0x00, 0x00, 0x00)
    res2 = transmit_data(1, 0, node_id, a2)
    rospy.sleep(0.01)

    # 通道1发送数据 工作模式，位置控制，设置 1
    a3 = (c_ubyte*8)(0x2F, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00)
    res3 = transmit_data(1, 0, node_id, a3)
    rospy.sleep(0.01)

    # 通道1发送数据 梯形速度 0
    a4 = (c_ubyte*8)(0x23, 0x81, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00)
    res4 = transmit_data(1, 0, node_id, a4)
    rospy.sleep(0.01)

    # 获取当前位置
    # a7 = (c_ubyte*8)(0x40, 0x63, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00)
    # res7 = transmit_data(1, 0, node_id, a7)
    # rs = receive_a_data()
    # curPos = rs

    # 设置目标位置为0
    a5 = (c_ubyte*8)(0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00)
    res5 = transmit_data(1, 0, node_id, a5)
    rospy.sleep(0.01)

    # 控制字 103F 使能   发送这条就动了
    a6 = (c_ubyte*8)(0x2B, 0x40, 0x60, 0x00, 0x3F, 0x10, 0x00, 0x00)
    res6 = transmit_data(1, 0, node_id, a6)
    rospy.sleep(0.01)

    # 准备运动 先设置梯形速度2000
    a7 = (c_ubyte*8)(0x23, 0x81, 0x60, 0x00, 0x22, 0x22, 0x22, 0x20)
    res7 = transmit_data(1, 0, node_id, a7)
    
    return (res1 and res2 and res3 and res4 and res5 and res6 and res7)

def set_velocity_speed(node_id):
    # 准备运动 先设置梯形速度2000
    rospy.sleep(1)
    a7 = (c_ubyte*8)(0x23, 0x81, 0x60, 0x00, 0x22, 0x22, 0x22, 0x20)
    res7 = transmit_data(1, 0, node_id, a7)
    return res7

def open_and_init_device1():
    device_opened = open_device(0, 0)
    rospy.sleep(0.01)
    can_initialized = init_can(0, 0, vci_initconfig1)
    rospy.sleep(0.01)
    can_started = start_can(0, 0)
    rospy.sleep(0.01)

    can_initialized = init_can(0, 1, vci_initconfig1)
    rospy.sleep(0.01)
    can_started = start_can(0, 1)
    rospy.sleep(0.01)

def open_and_init_device2():
    device_opened = open_device(1, 0)
    rospy.sleep(0.01)
    can_initialized = init_can(1, 0, vci_initconfig2)
    rospy.sleep(0.01)
    can_started = start_can(1, 0)
    rospy.sleep(0.01)

    FL_initialized = init_a_motor(0x602)
    print("FL_initialized: ", FL_initialized)
    rospy.sleep(0.01)
    FR_initialized = init_a_motor(0x603)
    print("FR_initialized: ", FR_initialized)
    rospy.sleep(0.01)
    RL_initialized = init_a_motor(0x604)
    print("RL_initialized: ", RL_initialized)
    rospy.sleep(0.01)
    RR_initialized = init_a_motor(0x601)
    print("RR_initialized: ", RR_initialized)
    rospy.sleep(0.01)
    
def init_4_motor():
    # 初始化电机
    four_steering_initialized = init_a_motor(0x602)
    four_steering_initialized = init_a_motor(0x603)
    four_steering_initialized = init_a_motor(0x604)
    four_steering_initialized = init_a_motor(0x601)
    
    # 初始化电机梯形速度，给2000
    set_vs1 = set_velocity_speed(0x602)
    set_vs2 = set_velocity_speed(0x603)
    set_vs3 = set_velocity_speed(0x604)
    set_vs4 = set_velocity_speed(0x601)

    mode_0()

def turn_to_0(node_id):
    a = (c_ubyte*8)(0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00)
    res = transmit_data(1, 0, node_id, a)
    return res

# arkman mode0
def mode_0():
    turn_to_0(0x603)
    turn_to_0(0x602)
    turn_to_0(0x604)
    turn_to_0(0x601)

# self spinning mode2
def mode_2():
    FLBR = (c_ubyte*8)(0x23, 0x7A, 0x60, 0x00, 0x00, 0x80, 0xda, 0xff)
    FRBL = (c_ubyte*8)(0x23, 0x7A, 0x60, 0x00, 0x00, 0x80, 0x25, 0x00)
    res_FL = transmit_data(1, 0, 0x603, FLBR)
    res_FR = transmit_data(1, 0, 0x602, FRBL)
    res_BL = transmit_data(1, 0, 0x604, FRBL)
    res_BR = transmit_data(1, 0, 0x601, FLBR)
    return ( res_FL and res_FR and res_BL and res_BR)

# -4915200 to [0, 0, 181, 255]
def angle10to16(val):
    if val < 0:
        val = (1 << 32) + val
    byte1 = val & 0xff
    byte2 = (val >> 8) & 0xff
    byte3 = (val >> 16) & 0xff
    byte4 = (val >> 24) & 0xff
    
    res = [byte1, byte2, byte3, byte4]
    return res

# [0, 0, 181, 255] to -4915200
def angle16to10(val):
    res = (val[3] << 24) + (val[2] << 16) + (val[1] << 8) + (val[0] & 0xff)
    if res > 0x7FFFFFFF:
        res -= 0x100000000
    return res

# 给位移线速度，得出电机转速
def speed_to_rpm(speed, wheel_diameter):
    wheel_circumference = pi * wheel_diameter
    wheel_rotations_per_second = speed / wheel_circumference
    rpm = wheel_rotations_per_second * 60 * 32
    return abs(rpm)

# 给电机转速，得出位移线速度
def rpm_to_speed(rpm, wheel_diameter):
    wheel_rotations_per_second = rpm / 60 / 32
    wheel_circumference = pi * wheel_diameter
    speed = wheel_rotations_per_second * wheel_circumference
    return speed

# 输入speed单位：米/秒
def set_motor_going(node_id, speed):
    rpm = speed_to_rpm(speed, wheel_diameter=0.61)
    if speed >= 0:
        rpm_sent_to_motor = 30000 + int(rpm)
    else:
        rpm_sent_to_motor = 30000 - int(rpm)
    # print(rpm_sent_to_motor)
    low8 = rpm_sent_to_motor & 0xFF
    high8 = (rpm_sent_to_motor >> 8) & 0xFF
    data = (c_ubyte * 8)(0x0B, 0x00, 0x00, low8, high8, 0x00, 0x00, 0x00)
    if node_id == 0x201 or node_id == 0x202:
        transmit_data(0, 0, node_id, data)
    elif node_id == 0x203 or node_id == 0x204:
        transmit_data(0, 1, node_id, data)
    else:
        return

# 输入angle单位：弧度
def set_motor_turning(node_id, angle):
    angle_sent_to_motor = angle10to16( int((degrees(angle) / 90) * 4915200) )
    data = (c_ubyte * 8)(0x23, 0x7A, 0x60, 0x00, angle_sent_to_motor[0], angle_sent_to_motor[1], angle_sent_to_motor[2], angle_sent_to_motor[3])
    transmit_data(1, 0, node_id, data)



if __name__ == '__main__':
    try:
        Controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    

