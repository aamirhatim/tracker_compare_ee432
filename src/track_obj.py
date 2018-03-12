#!/usr/bin/env python

import rospy
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

class Tracker:
    def __init__(self):
        print "Setting up object tracker..."
        self.setup = False
        self.check = True
        self.bounds = (0, 0, 0, 0)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.track)
        self.pub = rospy.Publisher("/tracker/obj_center", Point, queue_size = 5)
        self.avg = Point()
        self.averager = [self.avg for i in range(10)]
        self.count = 0
        print "Setup complete! Let's track something ;)"

    def track(self, data):
        # Convert and downsize incoming Image --> 640x480 to 320x240
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # frame = cv2.flip(cv2.resize(img_raw, (0,0), fx = .5, fy = .5), 1)

        # Set up tracker on first image frame
        if self.setup == False:
            (self.tracker, self.bounds) = self.tracker_setup(frame)
            self.setup = True
            return

        # If setup has been completed, update the tracker
        self.check, self.bounds = self.tracker.update(frame)

        # If tracking was successful, make the new bounding box
        if self.check:
            # p1 = (int(self.bounds[0]), int(self.bounds[1]))
            # p2 = (int(self.bounds[0] + self.bounds[2]), int(self.bounds[1] + self.bounds[3]))
            # cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)

            # Find center and get its average over 10 frames
            center = Point()
            center.x = int(self.bounds[0]+(self.bounds[2]/2))
            center.y = int(self.bounds[1]+(self.bounds[3]/2))
            if self.count < 10:
                self.avg.x += center.x/10
                self.avg.y += center.y/10
                self.count += 1
            else:
                # Publish averaged center
                self.pub.publish(center)
                self.count = 0
                self.avg.x = 0
                self.avg.y = 0

        else:
            if self.count < 10:
                # self.avg.x += center.x/10
                # self.avg.y += center.y/10
                self.count += 1
            else:
                # Publish averaged center
                self.pub.publish(center)
                self.count = 0
                self.avg.x = 0
                self.avg.y = 0
            # cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

        # Display tracker on image
        # cv2.imshow("Tracking", frame)
        # k = cv2.waitKey(5) & 0xff
        # if k == 27:
        #     return

    def tracker_setup(self, frame):
        print "Preparing tracker..."
        (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

        # List of tracking methods. Choose type
        tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
        tracker_type = tracker_types[0]

        # Set up the tracker specified in tracker_type
        if int(minor_ver) < 3:
            tracker = cv2.Tracker_create(tracker_type)
        else:
            if tracker_type == 'BOOSTING':
                tracker = cv2.TrackerBoosting_create()
            if tracker_type == 'MIL':
                tracker = cv2.TrackerMIL_create()
            if tracker_type == 'KCF':
                tracker = cv2.TrackerKCF_create()
            if tracker_type == 'TLD':
                tracker = cv2.TrackerTLD_create()
            if tracker_type == 'MEDIANFLOW':
                tracker = cv2.TrackerMedianFlow_create()
            if tracker_type == 'GOTURN':
                tracker = cv2.TrackerGOTURN_create()

        # Ask for a bounding box and then initiate tracker with it
        bbox = cv2.selectROI(frame, False)
        tracker_setup = tracker.init(frame, bbox)

        print "Tracker setup complete! Using:", tracker_type
        return (tracker, bbox)

def main():
    rospy.init_node("obj_tracker")
    tracker = Tracker()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "We're done here."
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
