#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import math

msg = """
Control trobot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
space key, k : force stop
anything else : stop smoothly
CTRL-C to quit
"""

moveBindings = {
    'i':(1,0),
    'o':(1,1),
    'j':(0,1),
    'l':(0,-1),
    'u':(1,-1),
    ',':(-1,0),
    '.':(-1,1),
    'm':(-1,-1),
}

speedBindings={
    'q':(1.1,1),
    'z':(.9,1),
    'w':(1.1,1),
    'x':(.9,1),
}

speed = .2
turn = .145
wheel_base = 1.0
max_steering_angle = 3.14159 / 5


def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    global max_steering_angle
    if omega == 0 or v == 0:
        return 0
    radius = v / omega
    res = math.atan(wheelbase / radius)

    if(res > max_steering_angle):
        rospy.logwarn("Over Maximum Angle!%f",res)
        return max_steering_angle
    if(res < -max_steering_angle):
        rospy.logwarn("Over Maximum Angle!%f",res)
        return -max_steering_angle

    return res


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return ">>set_currently:\tspeed %s" % (speed)


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    last_control_speed = 0
    last_control_turn = 0
    try:
        print msg
        print "<===MAX:  speed %s  turn %s===>" % (speed,turn)
        while(1):
            key = getKey()
            # 运动控制方向键（1：正方向，-1负方向）
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]               
                count = 0
            # 速度修改键
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]  # 线速度增加0.1倍
                turn = turn * speedBindings[key][1]    # 角速度增加0.1倍
                count = 0

                print vels(speed,turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            # 停止键
            elif key == ' ' or key == 'k' :
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            # 目标速度=速度值*方向值
            target_speed = speed * x
            target_turn = turn * th

            # 速度限位，防止速度增减过快
            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.01 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.02 )
            else:
                control_turn = target_turn

            # 创建并发布twist消息
            twist = Twist()
            twist.linear.x = control_speed
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = convert_trans_rot_vel_to_steering_angle(control_speed, control_turn, wheel_base)
            pub.publish(twist)
            if control_speed != 0:
                print "publish \"/cmd_vel\" liner.x={}\tangular.z={}".format(control_speed, control_turn)
            elif last_control_speed != 0:
                print msg
                print "<===MAX:\tspeed %s\tturn %s===>" % (speed, turn)
            last_control_speed = control_speed
    except:
        print ("fail")

    finally:
        twist = Twist()
        pub.publish(twist)

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)