#!/usr/bin/python
# license removed for brevity
import rospy
import pygame
import threading
import os,time
from geometry_msgs.msg import *
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from std_msgs.msg import *
import sys, select, termios, tty

#PS joystick

triangle = 4
circle = 1
cross = 0
square = 3
L1 = 6
L2 = 8
R1 = 7
R2 = 9
select_b = 10
start_b = 11

#cmd_vel
speed = 0.2
turn = 0.5
speed_direction = 0
turn_direction = 0
max_speed = 1.5
max_turn = 1.5
motor_break = False
pub_cmd = False
last_time = 0
# Publisher
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
goal_change_pub = rospy.Publisher('goal_change', Int16, queue_size=2)

def start_joystick_loop():
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    pygame.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print joystick.get_name()

    axes = joystick.get_numaxes()
    print 'axes',axes

    buttons = joystick.get_numbuttons()
    print 'buttons',buttons

    hats = joystick.get_numhats()
    print 'hats', hats

    balls = joystick.get_numballs()
    print 'balls', balls

    for i in range(axes):
        axis = joystick.get_axis(i)
    for i in range(buttons) :
        button = joystick.get_button(i)
    for i in range(hats):
        hat = joystick.get_hat(i)
    for i in range(balls):
        ball = joystick.get_ball(i)

    while not rospy.is_shutdown():
        for event in pygame.event.get(): # User did something
            # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
            if event.type == pygame.JOYBUTTONDOWN:
                print "You pressed",event.button
                button_pressed(event.button)
            if event.type == pygame.JOYBUTTONUP:
                print "You released",event.button
                button_release(event.button)
            if event.type == pygame.JOYBALLMOTION:
                print "ball motion"
            if event.type == pygame.JOYAXISMOTION:
                print "axis motion",event.axis, event.value
                axis_move(event.axis,event.value)
            if event.type == pygame.JOYHATMOTION:
                print "hat motion",event.hat,event.value
                hat_move(event.value)
        time.sleep(0.05)
    pygame.quit ()

def button_pressed(button):
    global motor_break,pub_cmd,power,speed,turn
    if button == cross:
        motor_break = True
        pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        msg = GoalID()
        pub.publish(msg)
    elif button == start_b:
        speed = 0.2
        turn = 0.5
    elif button == select_b:
        pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=2)
        msg=PoseStamped()
        msg.header.frame_id="map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.orientation.w = 1
        pub.publish(msg)
        print 'go home'
    elif button == circle:
        pub_cmd = False
        pub = rospy.Publisher('/to_goal', Bool, queue_size=2)
        msg = Bool()
        msg.data = True
        pub.publish(msg)
    elif button == square:
        pub = rospy.Publisher('/record_goal', Bool, queue_size=2)
        msg = Bool()
        pub.publish(msg)
    else:
        tune_speed(button)

def button_release(button):
    global motor_break
    if button == cross:
        motor_break = False

def tune_speed(button):
    global speed,turn
    #tune speed
    if button == L1:
        speed = speed + 0.1
    if button == L2:
        speed = speed - 0.1
    if button == R1:
        turn = turn + 0.1
    if button == R2:
        turn = turn - 0.1
    #limitation 
    if speed > max_speed:
        speed = max_speed
    if speed < 0:
        speed = 0
    if turn > max_turn:
        turn = max_turn
    if turn < 0:
        turn = 0

def hat_move(direction):
    if direction[1] != 0:
        msg = Int16()
        msg.data = direction[1]
        goal_change_pub.publish(msg)
        print 'change goal'

def axis_move(axis,value):
    global speed_direction,turn_direction
    if axis == 1:
        if abs(value) > 0.5:
            speed_direction = -value
        else:
            speed_direction = 0
    if axis == 2:
        if abs(value) > 0.5:
            turn_direction = -value
        else:
            turn_direction = 0
    if axis == 6:
        if value != 0:
            msg = Int16()
            msg.data = value
            goal_change_pub.publish(msg)
            print 'change goal'


def publisher():
    global pub_cmd,last_time
    control_speed = 0
    control_turn = 0
    target_speed = 0
    target_turn = 0

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        target_speed = speed*speed_direction
        target_turn = turn*turn_direction

        if target_speed > control_speed:
            control_speed = min( target_speed, control_speed + 0.05 )
        elif target_speed < control_speed:
            control_speed = max( target_speed, control_speed - 0.05 )
        else:
            control_speed = target_speed

        if target_turn > control_turn:
            control_turn = min( target_turn, control_turn + 0.2 )
        elif target_turn < control_turn:
            control_turn = max( target_turn, control_turn - 0.2 )
        else:
            control_turn = target_turn

        if motor_break:
            control_speed = 0
            control_turn = 0
            target_speed = 0
            target_turn = 0

        twist = Twist()
        twist.linear.x = control_speed
        twist.angular.z = control_turn
        
        #Stop pub topic when idle
        if control_speed != 0 or control_turn != 0:
            last_time = time.time()
            pub_cmd = True
        if time.time() - last_time > 3:
            pub_cmd = False

        if pub_cmd:
            pub.publish(twist)
            pub.publish(twist)
            if motor_break:
                pub_cmd = False

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('joystick', anonymous=False)
    
    t = threading.Thread(target=start_joystick_loop)
    t.daemon = True
    t.start()

    publisher()
    rospy.spin()

    