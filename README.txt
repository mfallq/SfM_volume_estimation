Escenda AB
System for automatic volume estimation
Contact: Marcus Fallqvist 
0720055643
marcus.fallqvist@escenda.com

Programs needed:

1) Visual Studio C++ 
2) OpenCV C++
3) Matlab with computer vision package
4) Oracle VM Virtualbox (or similar to run Linux)
5) OpenMVG Linux 

Data can be collected on smartphones using:
1) Video recorded in 1080p at 60fps
2) IMU data: Accelerometer and gyroscope (App used: Sensor Fusion - Linköping University)

How to run system:
1) Export data to PC
2) Run "KLTtracker" main.cpp file in the C++ project in VS. 
	It creates images from the video with tracked feature 	points and stores the IMU data correctly.
	You need to add path to video and IMU data in .txt format
3) Run OpenMVG Linux program to create an 3D model in the SfM 	process using the data from above.
4) Parse 3D data using the last part of "KLTtracker" main.cpp 	file. This creates the data in .json format
5) Run the MATLAB script on the .json data. This creates a 3D 	model which has a solid surface and calculates that 	volume.

