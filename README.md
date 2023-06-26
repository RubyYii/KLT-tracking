# KLT-tracking

## Project Title: Real-Time Object Tracking
#Description:
This project provides an implementation of real-time object tracking in MATLAB. The code is designed to track a moving object in a video, whether that video comes from a live webcam feed or a pre-recorded video file. It uses the imfindcircles function for initial object detection, and the vision.PointTracker function, based on the Kanade-Lucas-Tomasi (KLT) algorithm, for the actual tracking of the object.

#Features:
Support for both webcam feeds and pre-recorded video files.
Allows for the tracking of a single object identified initially within a user-specified rectangle.
Utilizes imfindcircles for initial object detection within a specified ROI.
Employs vision.PointTracker for consistent tracking of the detected object across frames.
The capability of re-detecting features and reinitializing the tracker with the help of detectMinEigenFeatures function, in case the object tracking fails in a certain frame.
Real-time plotting of either the object's distance to a specified point or the object's location coordinates over time.
The final plot of the tracked object's path over the duration of the video.
How to Use:
Set the variable useWebcam to either true or false to select the source of the video feed.
If useWebcam is set to false, provide the path to the video file in the videoFile variable.
Choose the plot type by setting the plotType variable to either 'distance' or 'position'.
Run the script. The first frame of the video will be displayed.
Draw a rectangle around the object you want to track in the displayed frame.
The script will then automatically track the object in real time and display the video frames along with a plot of the tracked metric (either distance to a fixed point or location coordinates).
After all frames are processed, the script will also provide a plot of the object's trajectory.
Dependencies:
MATLAB R2019b or later.
Computer Vision System Toolbox in MATLAB.
Notes:
This script was developed for educational purposes and is not intended for use in high-performance or real-time systems.

License:
MIT

Please feel free to contribute or suggest improvements.
