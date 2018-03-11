# Real-Time Object Tracking Performance on Mobile Robots
## EECS 432: Advanced Computer Vision (Winter 2018)
### Northwestern University
### Aamir Husain

## Introduction
Object detection and tracking has been a widely studied area in computer vision. With constant innovations in technology, our computational efficiency has tremendously increased, allowing complicated processes like object tracking to be a realistic application. One main struggle with current technology, however, is getting reliable performance on lower-end devices or products that are severely limited by space and power - like a mobile robot. This project narrows in on this area of object tracking and detection, applying a series of object tracking algorithms on video samples and comparing their performance offline and in real-time.

## Setup
### Required Software/Packages
- ROS (Kinetic)
- OpenCV

### Hardware Specs
#### Laptop:
- Processor: Intel® Core™ i7-7500U CPU @ 2.70GHz × 4
- OS: Ubuntu 16.04 LTS
- RAM: 16GB

#### Mobile robot:
- Processor: Quad Core 1.2GHz Broadcom BCM2837 CPU (Raspberry Pi 3)
- OS: Ubuntu MATE 16.04.02 (Xenial) for Raspberry Pi 3
- RAM: 1GB

## Procedure
For comparison, a series of videos were taken in different settings (outdoor, indoor, low light, bright light, moving, stationary, etc.). While a tracker was run on each video, the location of the bounding box was recorded and plotted over time as well as its processing speed (measured in frames per second). Any detection failures were also noted.

This process was done first on videos recorded on the laptop to ensure proper data gathering. Once that was confirmed, the process was repeated for the mobile robot.

Finally, a series of videos were taken in real time on the mobile robot for each algorithm. The same metrics (bounding box location, processing speed, detection failures) were recorded.

### Running the Code
To start the tracker on a live video feed, run the following command:
```
$ roslaunch tracker start_cam.launch
```

## Results

## Challenges

## Conclusion
