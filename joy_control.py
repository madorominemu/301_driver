import math
from math import sin, cos
import time
import rospy
from ctypes import *
from can_utils import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16


class JoyDo(object):
    def __init__(self):
        rospy.init_node("joy_control", anonymous=True)
        rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size=1)
        self.cmd_pub = rospy.Publisher("/joy/cmd_vel", Twist, queue_size=10)
        self.mode_pub = rospy.Publisher("/control_mode", Int16, queue_size=10)

        self.cmd = Twist()

        self.msg = None

        self.mode = 0  # 0:阿克曼  1:全向模式  2:自旋转
        self.gear = 1  # 速度档位 1 2 3; 对应 1m/s, 2m/s, 3m/s; 对应~1000rpm, 2000rpm, 3000rpm

        self.max_wz = 1.4146262757571544  # 米/秒
        self.max_vx = 1.1760982  # 米/秒

        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.wx = 0
        self.wy = 0
        self.wz = 0


        self.btnR2PressedOnce = False
        self.btnL2PressedOnce = False

        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def joy_callback(self, joy_msg):
        if joy_msg is not None and len(joy_msg.axes) == 8:
            self.msg = joy_msg
            if joy_msg.buttons[4] == 1 and joy_msg.buttons[5] == 1:
                self.mode += 1
                if self.mode > 2:
                    self.mode = 0
                self.mode_pub.publish(self.mode)
                time.sleep(1)
            if joy_msg.buttons[2] == 1 and joy_msg.buttons[1] != 1:
                self.gear += 1
                if self.gear > 3:
                    self.gear = 3
                time.sleep(1)
            elif joy_msg.buttons[2] != 1 and joy_msg.buttons[1] == 1:
                self.gear -= 1
                if self.gear < 1:
                    self.gear = 1
                time.sleep(1)

    def timer_callback(self, event):
        if self.msg is not None and len(self.msg.axes) == 8:
            if self.msg.buttons[0] == 1:  # 按钮 0 被按下时设置速度为 0 并发布
                self.set_0()
                self.publish_command()
            elif self.msg.buttons[5] == 1:  # 仅在按钮 5 被按下时处理速度和发布命令
                self.process_input()
                self.publish_command()
            else:
                self.set_0()
        else:
            self.set_0()

    def process_input(self):
        # 根据模式处理输入数据
        if self.msg is not None and self.mode == 0:
            self.vx = self.msg.axes[1] * (1.0 * self.gear)
            self.wz = self.msg.axes[0] * 1.1071487177940902
        elif self.msg is not None and self.mode == 1:
            self.vx = self.msg.axes[1] * (1.0 * self.gear)
            self.wz = self.msg.axes[3] * 1.5707963267948966
        elif self.msg is not None and self.mode == 2:
            self.vx = 0
            self.wz = self.msg.axes[1] * (1.0 * self.gear)
        else:
            self.set_0()

    def publish_command(self):
        self.cmd.linear.x = self.vx
        self.cmd.linear.y = self.vy
        self.cmd.linear.z = self.vz
        self.cmd.angular.x = self.wx
        self.cmd.angular.y = self.wy
        self.cmd.angular.z = self.wz
        self.cmd_pub.publish(self.cmd)
                
    def set_0(self):
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.wx = 0
        self.wy = 0
        self.wz = 0

        
if __name__ == "__main__":
    try:
        JoyDo()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

