#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

midline = 160
threshhold = 20

def turn_control(data):
    delta = data.x - midline

    if delta > threshhold:
        print "turn right"
    elif delta < -threshhold:
        print "turn left"

def main():
    rospy.init_node("turn_control")
    sub = rospy.Subscriber("/tracker/obj_center", Point, turn_control)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "We're done here."

if __name__ == "__main__":
    main()
