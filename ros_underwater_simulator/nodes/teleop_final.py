#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64

import sys, select, termios, tty

msg = """
Joint Control
---------------------------
Revolute 1:
Rotation by 5 degrees in positive direction: press '1'
Rotation by 5 degrees in negative direction: press '2'

Revolute 2:
Rotation by 5 degrees in positive direction: press 'q'
Rotation by 5 degrees in negative direction: press 'w'
CTRL-C to quit
"""
#Read key press function
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    #Creating the node for teleop and the two publishes
    rospy.init_node('teleop_keyboard')
    pub = rospy.Publisher('/uw_arm/joint1_position_controller/command', Float64, queue_size=1) #This topic should match the topic of the model which one wishes to control
    pub2 = rospy.Publisher('/uw_arm/joint2_position_controller/command', Float64, queue_size=1) #This topic should match the topic of the model which one wishes to control
    float_msg = Float64()

    accumulated_first = 0
    accumulated_second = 0
    direction_first = 0
    direction_second = 0
    
    try:
        print(msg)
        while(1):
            accumulated_first = accumulated_first
            accumulated_second = accumulated_second
            key = getKey()
            if key == '1':
                print('key 1 pressed')
                direction_first = 0.09 #This ensures that key input '1' sends a command to rotate the joint approx 5deg in positive direction
            elif key == '2':
                print('key 2 pressed')
                direction_first = -0.09 #This ensures that key input '2' sends a command to rotate the joint approx 5deg in negative direction
            elif key == 'q':
                print('key q pressed')
                direction_second = 0.09 #This ensures that key input '1' sends a command to rotate the joint approx 5deg in positive direction
            elif key == 'w':
                print('key w pressed')
                direction_second = -0.09 #This ensures that key input '2' sends a command to rotate the joint approx 5deg in negative direction
            else:
                direction_first = 0
                direction_second = 0
                if (key == '\x03'):
                    break
            
            #Calculation of position for the first revolute
            accumulated_first = direction_first + accumulated_first
            if accumulated_first > 0.548:
                accumulated_first = 0.548
                float_msg.data = accumulated_first
                pub.publish(float_msg)
                print('Exceeding joint limits!')
            elif accumulated_first < -0.548:
                accumulated_first = -0.548
                float_msg.data = accumulated_first
                pub.publish(float_msg)
                print('Exceeding joint limits!')
            else:
                float_msg.data = accumulated_first
                pub.publish(float_msg)
            
            # float_msg.data = accumulated_first
            # pub.publish(float_msg)
            #Calculation of position for the second revolute
            accumulated_second = direction_second + accumulated_second
            if accumulated_second > 0.548:
                accumulated_second = 0.548
                float_msg.data = accumulated_second
                pub2.publish(float_msg)
                print('Exceeding joint limits!')
            elif accumulated_second < -0.548:
                accumulated_second = -0.548
                float_msg.data = accumulated_second
                pub2.publish(float_msg)
                print('Exceeding joint limits!')
            else:
                float_msg.data = accumulated_second
                pub2.publish(float_msg)

    except Exception as e:
        print(e)

    finally:
        pub.publish(float_msg)
        pub2.publish(float_msg)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)