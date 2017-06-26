# contact_detection (reduced version)

contact_detection is a code to recognize contacts for a moving robotic hand (here the Pisa/IIT Soft Hand) with a Sensorized Glove (IMUs and a PSoC uC).

This package needs of [qb_interface_node]() on [Ubuntu 14.04](http://www.ubuntu.com/download/desktop).


## Use

The command to launch the programm is:

`roslaunch contact_detection rect.launch`

The code use cross-correlation in real time to recognise contacts using accelerometers data. There is a data set to compare the signals. Gyroscopes let a better identification of the contacts. 