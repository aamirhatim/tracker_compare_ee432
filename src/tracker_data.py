#!/usr/bin/env python

import cv2
import sys
import csv
import os

def dir_exists(filename):
    name = filename[:-4]
    if not os.path.exists("../data/"+name):
        os.makedirs("../data/"+name)
        os.makedirs("../data/"+name+"/img")
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

def export_csv(name, tracker_type, t, x, y, fps):
    with open("../data/"+name+"/"+tracker_type+".csv", "a") as csvfile:
        filewriter = csv.writer(csvfile, delimiter = ",", quotechar = "|", quoting = csv.QUOTE_MINIMAL)
        filewriter.writerow([t, x, y, fps])

def draw_box(frame, bbox):
    p1 = (int(bbox[0]), int(bbox[1]))
    p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
    cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)

def draw_fail(frame):
    cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

def display_tracker(frame, frame_num, tracker_type, fps, name, count):
    cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
    cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
    # cv2.imshow("Tracking", frame)

    if count == 20:
        cv2.imwrite("../data/"+name+"/img/"+tracker_type+"_"+str(frame_num)+".jpg", frame)
        count = 0
    else:
        count += 1

    # Exit if ESC pressed
    k = cv2.waitKey(3) & 0xff
    if k == 27:
        return
    return count

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
                count = 0
                frame_num = 1
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

                    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);                # Get FPS
                    t = int(((cv2.getTickCount() - time)/cv2.getTickFrequency())*1000)          # Get time elapsed

                    if ok:
                        x, y = get_center(bbox)                                                 # Get center of bounding box
                        export_csv(name, tracker_type, t, x, y, fps)                            # Export success to CSV
                        draw_box(frame, bbox)                                                   # Draw box
                    else:
                        export_csv(name, tracker_type, t, -1, -1, 0)                            # Export failure to CSV
                        draw_fail(frame)                                                        # Write failure alert

                    count = display_tracker(frame, frame_num, tracker_type, fps, name, count)   # Create image
                    frame_num += 1                                                              # Increment frame number

if __name__ == '__main__' :
    main()
