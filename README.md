# Arduino-Robot-Car-Line-Following-Obstacle-Avoidance

### Abstract

The goal of this project is to build and develop a robot car using Arduino, that is able to follow a line, avoid obstacles if found while following the line and then come back to the original path. In this repository will be provided the whole code used to achieve the goal, the circuit diagram, and the NewPing library used for the ultrasonic sensor. About the hardware part: has been used an Arduino Uno as board, a Sensor Shield v5.0 and an L298N Dual Bridge, both optional, used to manage cables of wheels. A servo motor to rotate the ultrasonic sensor, an ultrasonic sensor to measure the distance from obstacles and two infrared sensor to track the line. As will be showed in some videos, the result has been achieved. If you wish to implement the code available in this repository make sure to tune parameters of the delay() functions in the code. Since the delay changes basing on the voltage of your batteries and the quality of your sensors, more or less delay will be needed to perform actions like a turn or any other movement. 

### Component List
<ul>
  <li>1x Arduino Uno</li>
  <li>1x Battery Holder</li>
  <li>1x Ultrasonic Sensor</li>
  <li>2x IR Sensor</li>
  <li>1x Sensor Shield v5.0</li>
  <li>1x L298N Dual Bridge</li>

### Libraries used

For this project has been used the NewPing library, since it has pre built functions to measure distance in centimeters, the library is available in this repository or you can find the latest version on the [official website](https://www.arduino.cc/reference/en/libraries/newping/).

### Examples of usage

Here some videos of usage, mp4 videos of some example are available in the [Resource](https://github.com/salvatore-arienzo/Arduino-Robot-Car-Line-Following-Obstacle-Avoidance/tree/main/Resources) folder.

##### Simple line following: 

[![148898f698cab7c40.md.gif](https://s2.gifyu.com/images/148898f698cab7c40.md.gif)](https://gifyu.com/image/Uifr)

##### Line Following + Obstacle avoidance: 

[![2dd369ddd45ee19f3.th.gif](https://s2.gifyu.com/images/2dd369ddd45ee19f3.th.gif)](https://gifyu.com/image/UiYa)
