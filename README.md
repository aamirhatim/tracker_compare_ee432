# Real-Time Object Tracking Performance on Mobile Robots
### Aamir Husain
### EECS 432: Advanced Computer Vision (Winter 2018)
### Northwestern University

## Introduction
Object detection and tracking has been a widely studied area in computer vision. With constant innovations in technology, our computational efficiency has tremendously increased, allowing complicated processes like object tracking to be a realistic application. One main struggle with current technology, however, is getting reliable performance on lower-end devices or products that are severely limited by space and power - like a mobile robot. This project narrows in on this area of object tracking and detection, applying a series of object tracking algorithms on video samples and comparing their performance off-line and in real-time.

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
1. For comparison, a series of videos were taken in different settings (outdoor, indoor, low light, bright light, moving, stationary, etc.). While a tracker was run on each video, the location of the bounding box was recorded and plotted over time as well as its processing speed (measured in frames per second). Any detection failures were also noted.

2. Using the data from step 1, the best algorithm was chosen for the mobile robot. A ROS publisher was added to the tracker algorithm to send out coordinates of the bounding box and a separate subscriber was written to create motor commands from the published points. The subscriber can be found in the [Argo](https://github.com/aamirhatim/argo.git) package that was made for controlling the robot.

3. Performance of the robot was observed using the chosen tracking method from step 2 and also with AR tags.

### Algorithms Tested
- Boosting
- KCF
- MedianFlow
- MIL
- TLD

### Running the Code
To start the tracker on a live video feed, run the following command:
```
$ roslaunch tracker start_cam.launch
```

To create your own data from pre-recorded videos, add the videos to the `/videos` directory. Then run the following command:
```
$ cd scripts/
$ ./tracker_data.py
```
This does not need ROS to run. A new folder will be created in `data/` for each video if it doesn't exist. The `tracker_data.py` code will then store the tracking data for each video in its respective folder. To view the data, run the `tracker()` function in `tracker.m` in MATLAB. This will plot the bounding box trails of each tracker for each video (every video will have its own window).

## Results

## Challenges
Getting reliable data from the mobile robot was a challenge due to its lower processing power.

## Conclusion
