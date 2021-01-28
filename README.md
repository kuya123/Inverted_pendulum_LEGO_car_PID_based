# Inverted_pendulum_LEGO_car_PID_based
em....This is a hobby project. Classic INVERTED PENDULUM problem. I want to try build up a LEGO based car and use my chop stick to realize a interted pendulum system.  if you have interest, you may click the Inverted_Pendulum_Car.jpg to see this true system.

I am working on it and let's keep updating. 

here are some basic system introduction on HW and SW configuration:

## Hardware discription

### Angle sensor: analog potential meter

1. potential meter present a linear relationship between angle and resistance
2. need several calibration to fix out the mapping ratio between delta angel and delta R
3. for each driving loop, a initial horizontal auto calibration is required for later angel calculation.

### Speed sensor: 12 line motor encoder.

1. I tried a 200 line encoder at first, but that high resolution will jam up my interrupt routing and kill my mcu processor.  the reason is I am trying to use arduino uno ATMEL 328 , which is a quite slow mcu. 

2. at very very beginin, I was tring to use a ultrasound sonar sensor for distance detection, then translate it to speed info. but the problem is on vibration and speed.  when we do some basic calculation basing on sound speed and distance, we can see that the speed of sound reflection can not meet my requirement of sampling.

### MCU: ATMEL 328P original arduino uno board

1. why use this one? official reason is : i want to explore the potential of this part and see whetehr I can squeeze out its max.  but the real reason is : i only have this one at hand for that month. :-)  too sad to be poor.  just enjoy it. 

### Motor: 12V DCM
Gear ratio 1:34 , 12V rotation speed: 294rpm

### Motor Driver
L298 motor driver chip.

### Power supply: 12V Li-ion battery

### Wireless link
Blue tooth UART module : HC05 
config the part to 115200 bps. 


### Software Archetecture 

1. C/C++ for mcu 
2. Python for PC end morintor
3. on-air configuration and status monitor are done through BT wireless uart channel.  Python takes care of serial port on PC end. configure the device through GUI. and display status on graphic windows.



enjoy building , enjoy coding and enjoy life

Cheers    

Chen Jian

