# ROS_AHRS
Arduino AHRS code connected to ROS via rosserial. Atom &amp; Platformio environment required for build this project. Rosserial package also required for gather ahrs topic from arduino.
this package use Madgwick AHRS Algorithm

Atom : https://atom.io/

Platformio : http://platformio.org/

Rosserial : http://wiki.ros.org/rosserial

##Tutorial

1. Install Atom and Platformio and setup your hardware 

  Arduino UNO
  HMC5883L
  MPU6050

2. Clone this repo on your workspace

3. Upload this firmware to arduino (Ctrl + Alt + U)

4. Launch rosseiral node (CAUTION : DO NOT MOVE Gyro when power up arduino. && Serialport num(dev/ttyACMX) can be changed)
  ```{r, engine='bash', count_lines}
  rosrun rosserial_python serial_node.py _baud:=1000000 /dev/ttyACM0 
  ```

5. It gives you imu/Quaternion topic
