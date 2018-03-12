#!/usr/bin/env python

import cv2
import sys
import csv
import os

def dir_exists(filename):
    name = filename[:-4]
    if not os.path.exists("../data/"+name):
        os.makedirs("../data/"+name)
        print name,"folder added."
    return name

def open_video(directory, filename):
    video = cv2.VideoCapture(os.path.join(directory, filename))
    if not video.isOpened():
        print "Could not open video"
        sys.exit()

    ok, frame = video.read()
    if not ok:
        print 'Cannot read video file'
        sys.exit()
    return frame, video

def get_center(bbox):
    x = int(bbox[0]+bbox[2]/2)
    y = int(bbox[1]+bbox[3]/2)
    return x, y

def export_csv(name, tracker_type, t, x, y):
    with open("../data/"+name+"/"+tracker_type+".csv", "a") as csvfile:
        filewriter = csv.writer(csvfile, delimiter = ",", quotechar = "|", quoting = csv.QUOTE_MINIMAL)
        filewriter.writerow([t, x, y])

def draw_box(frame, bbox):
    p1 = (int(bbox[0]), int(bbox[1]))
    p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
    cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)

def draw_fail(frame):
    cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

def display_tracker(frame, tracker_type, fps):
    cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
    cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
    cv2.imshow("Tracking", frame)

    # Exit if ESC pressed
    k = cv2.waitKey(3) & 0xff
    if k == 27:
        return

def main():
    directory = "../video"                                      # Specify video directory
    tracker_types = ['BOOSTING',                                # List of tracker algorithms
                     'MIL',
                     'KCF',
                     'TLD',
                     'MEDIANFLOW']

    for filename in os.listdir(directory):                      # Run trackers for every video
        if filename.endswith(".mp4"):
            name = dir_exists(filename)                         # Make directory for data if it doesn't exist
            frame, video = open_video(directory, filename)      # Open video file to get bounding box
            bounds = cv2.selectROI(frame, False)                # Set bounding box

            for i in range(len(tracker_types)):
                j = 0
                bbox = bounds                                   # Reset bounding box
                tracker_type = tracker_types[i]

                if tracker_type == 'BOOSTING':                  # Create tracker
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

                frame, video = open_video(directory, filename)  # Open video for tracking
                tracker.init(frame, bbox)                       # Initialize tracker
                time = cv2.getTickCount()                       # Get current time

                while True:
                    ok, frame = video.read()                    # Read a frame
                    if not ok:
                        break

                    timer = cv2.getTickCount()                  # Start timer
                    ok, bbox = tracker.update(frame)            # Update tracker

                    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);            # Get FPS
                    t = int(((cv2.getTickCount() - time)/cv2.getTickFrequency())*1000)      # Get time elapsed

                    if ok:
                        x, y = get_center(bbox)                         # Get center of bounding box
                        export_csv(name, tracker_type, t, x, y)         # Export success to CSV
                        # draw_box(frame, bbox)                           # Draw box
                    else:
                        export_csv(name, tracker_type, t, -1, -1)       # Export failure to CSV
                        # draw_fail(frame)                                # Write failure alert

                    # display_tracker(frame, tracker_type, fps)           # Display tracker on image



# (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

if __name__ == '__main__' :
    main()


    # directory = "../video"
    #
    # # Set up tracker types
    # tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
    #
    # # Implement trackers on all videos in the videos/ directory
    # for filename in os.listdir(directory):
    #     if filename.endswith(".mp4"):
    #         print filename
    #
    #         # Add new data directory if it doesn't exist
    #         name = filename[:-4]    # Remove the .mp4 from the filename
    #         if not os.path.exists("../data/"+name):
    #             os.makedirs("../data/"+name)
    #             print name,"folder added"
    #
    #         # Open video
    #         video = cv2.VideoCapture(os.path.join(directory, filename))
    #
    #         # Exit if video not opened.
    #         if not video.isOpened():
    #             print "Could not open video"
    #             sys.exit()
    #
    #         # Read first frame.
    #         ok, frame = video.read()
    #         if not ok:
    #             print 'Cannot read video file'
    #             sys.exit()
    #
    #         # Set bounding box for tracking
    #         bboxOG = cv2.selectROI(frame, False)
    #
    #         for i in range(2):
    #             j = 0
    #             bbox = bboxOG
    #             tracker_type = tracker_types[i]
    #             if tracker_type == 'BOOSTING':
    #                 tracker = cv2.TrackerBoosting_create()
    #             if tracker_type == 'MIL':
    #                 tracker = cv2.TrackerMIL_create()
    #             if tracker_type == 'KCF':
    #                 tracker = cv2.TrackerKCF_create()
    #             if tracker_type == 'TLD':
    #                 tracker = cv2.TrackerTLD_create()
    #             if tracker_type == 'MEDIANFLOW':
    #                 tracker = cv2.TrackerMedianFlow_create()
    #             if tracker_type == 'GOTURN':
    #                 tracker = cv2.TrackerGOTURN_create()
    #
    #             video = cv2.VideoCapture(os.path.join(directory, filename))
    #
    #             # Read first frame.
    #             ok, frame = video.read()
    #
    #             # Initialize tracker with first frame and bounding box
    #             ok = tracker.init(frame, bbox)
    #             time = cv2.getTickCount()
    #
    #             while True:
    #                 # Read a new frame
    #                 ok, frame = video.read()
    #                 if not ok:
    #                     break
    #
    #                 # Start timer
    #                 timer = cv2.getTickCount()
    #
    #                 # Update tracker
    #                 ok, bbox = tracker.update(frame)
    #
    #                 # Calculate Frames per second (FPS)
    #                 fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
    #                 t = int(((cv2.getTickCount() - time)/cv2.getTickFrequency())*1000)
    #
    #                 # Draw bounding box
    #                 if ok:
    #                     # Tracking success
    #                     # p1 = (int(bbox[0]), int(bbox[1]))
    #                     # p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
    #                     # cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
    #                     x = int(bbox[0]+bbox[2]/2)
    #                     y = int(bbox[1]+bbox[3]/2)
    #
    #                     # Add to CSV
    #                     with open("../data/"+name+"/"+tracker_type+".csv", "a") as csvfile:
    #                         filewriter = csv.writer(csvfile, delimiter = ",", quotechar = "|", quoting = csv.QUOTE_MINIMAL)
    #                         filewriter.writerow([t, x, y])
    #
    #                 else :
    #                     # Tracking failure
    #                     # cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
    #
    #                     # Add to CSV
    #                     with open(tracker_type+".csv", "a") as csvfile:
    #                         filewriter = csv.writer(csvfile, delimiter = ",", quotechar = "|", quoting = csv.QUOTE_MINIMAL)
    #                         filewriter.writerow([t, -1, -1])
    #
    #                 # # Display tracker type on frame
    #                 # cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
    #                 #
    #                 # # Display FPS on frame
    #                 # cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
    #                 #
    #                 # # Display result
    #                 # cv2.imshow("Tracking", frame)
    #                 #
    #                 # # Exit if ESC pressed
    #                 # k = cv2.waitKey(3) & 0xff
    #                 # if k == 27 : break
    #
    #         continue
    #     else:
    #         continue
