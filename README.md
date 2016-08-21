# Camera-Vision-2016

This is a clone of the repository where the camera tracking code for the Metal Mustangs (FIRST Robotics team 2410) 2016 season was kept. I created it for the purposes of consolidating all my work to one place. The original repository may be found [here](https://github.com/CAPS-Robotics/Camera-Vision-2016). It has not been changed except for this message. Additional information on what this code does and how it works may be found [here](http://www.spatrickdoyle.com/vision.html).<br/><br/>

Just the camera code for the 2016 season<br>
Dependancies:<br>
opencv<br>
cmake (I think opencv is integrated with cmake for some reason, so that's what I used to generate the Makefile and compile it, and that's why the CMakeLists.txt is here)<br>

To build:<br>
```
cmake .
make
```
<br><br>
Usage:<br>
./Camera-Vision-2016 [option]<br>
Options:<br>
quiet - don't give any output (except, y'know, for the camera stuff at the beginning)<br>
cal - bring up calibration info - threshold window, slider bars, and color view (by the way, you can press spacebar to make the changes persistant now!)<br>
view - just bring up a video feed (also outputs data to the terminal, like pretty much everything else)<br>
<br><br>
To start the VNC server on the Jetson, ssh into it (the default static IP address is 10.24.10.4) and run<br>
'''vncserver'''<br>
Then on the drive laptop connect to 10.24.10.4:5901<br>
