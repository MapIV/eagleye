
# imu gnss localizer

## Overview

It is a program that combines IMU and GNSS to perform highly accurate self position estimation.

## Description

Write markdown text in this textarea.

## Install

1) First, download the modified RTKLIB to your home directory.

		cd  
		git clone https://github.com/MapIV/RTKLIB.git  

2) Build RTKLIB.

		cd RTKLIB/app  
		chmod 755 makeall.sh  
		./makeall.sh  

3) Change the permissions of the two files.

		cd  
		cd RTKLIB/app/rtkrcv/gcc  
		chmod 755 rtkstart.sh  
		chmod 755 rtkshut.sh  

4) Next, download and build imu gnss localizer.

		cd  
		cd catkin_ws/src  
		git clone https://github.com/MapIV/imu_gnss_localizer.git  
		cd ..  
		catkin_make  

[links](https://wikipedia.org)

----
## changelog
* 17-Feb-2013 re-design

----
## thanks
* [markdown-js](https://github.com/evilstreak/markdown-js)
