# ME-PROJECT-BUILDING-A-DRONE-FROM-SCRATCH
Summer holiday between 1st and 2nd year EPFL ME. I want to build a fully functional drone

## Entry 1 (19/07/26)

### Goal :
- Document about how drones work  
- Find subjects I will have to look into. 
- Detail a 1st PRD (cahier des charges)  

### Today's status :  

Loads of electronics, solid mechanics. Some programming, thermodynamics, fluid mechanics. 

**Key components :**

- IMU : ONBOARD sensor that gives real-time drone orientation (accelerometer, gyroscope,...)
- Batteries : 1 for each propeller, gives power for thrust
- ESCs : 1 for each propeller, tells the battery how much power to allow at each moment (speed change)
- microcontroller : ONBOARD (probably) arduino (raspberry has OS-related delays, not optimal for high frequency commands)
- Radio controller : transmits the user's commands to the Arduino
- Radio receiver : wired to the arduino, makes the transit between the controller and the arduino

**Real-time process :**

1. The user does a command (ex. tilt by 10 degrees)
2. The command is radio-transmitted to the arduino
3. The IMU tells the arduino about current state (ex. titled by 8 degrees)
4. The arduino calculates the difference (e.g. here 2 degrees)
5. The arduino tells the ESCs the speed change needed to fill in that gap 

This sequence happens with a frequency of $\approx200\left\lbrack Hz\right\rbrack$

<img width="2016" height="1512" alt="IMG_6509" src="https://github.com/user-attachments/assets/d72e5b0b-19d3-4da3-b62c-cca9b946c278" />
<img width="2016" height="1512" alt="IMG_6510" src="https://github.com/user-attachments/assets/6b8217fb-f080-46a5-8d16-3c24ba5302ba" />


### Next steps : 

[[oscarliang.com]] for detailed drone info
- look at other DIY drone configuration to get an idea of what's doable and to-do
- get insight on electronic components, how to use them


## Entry 2 (20/08/26)


**What I did :**


- Learned how controller transmits to arduino

- Learned how arduino corrects errror using IMU and PID algorithm (detailed in [Tips])


- Started a "shopping list". To be completed later

- Start buying material (long shipping, and will let me start experiment coding). Bought electronics and soldering stuff

**Next steps :**


- Keep buying material (long shipping, and will let me start experiment coding)

- Set up coding environment

- Starting CADing drone frame, and do physics in parallel (e.g. total weight, to pick the right propellers)

- Do more of the physics (particularly dynamics, structure and material resistance)


## Entry 3 (22/08/26)

**What I did :**

- Decided to do CAD on Fusion (no need for Catia for a not-so-big construction)


- Started building the frame

<img width="1233" height="859" alt="Screenshot 2026-07-22 at 13 04 00" src="https://github.com/user-attachments/assets/5e556e6b-443d-404a-99b9-d808baaa474a" />

## Entry 4 (25/08/26)

**What I did :**

- More CAD done, first workable/printable version ready


- Ordered the propellers and motors, only battery stuff & controller left to buy



<img width="1467" height="853" alt="Screenshot 2026-07-25 at 20 15 56" src="https://github.com/user-attachments/assets/2b90b9ce-6fa8-456c-b038-0aa6487e2da5" />



**Next steps :**

- Maybe print the 1st version


- Look at open source code for inspo to configure the microcontroller


- (Order the other stuff)

## Entry 5 (5/08/26)

** What I did during 10 days** : 

- Ordered and received most of the material

<img width="4032" height="3024" alt="IMG_6576" src="https://github.com/user-attachments/assets/146ec5cb-42bd-4ec1-bf58-bb1121359821" />

- Edited the CAD and ran simulations

- 3D-Printed this workable frame

<img width="4032" height="3024" alt="IMG_6613" src="https://github.com/user-attachments/assets/031ef587-f977-49c5-83dc-54322249d66e" />

 - Started programming and wiring microcontroller and IMU
 

<img width="4032" height="3024" alt="IMG_6610" src="https://github.com/user-attachments/assets/abea6eb7-91ef-4cb9-8b00-fcf3a78e7315" />


## Entry 6 (11/08/26)

**What I did :**

- Coded for live IMU data report. It works well, giving me 1g of acceleration and 0±1 deg/s of rotation at rest state. It also detects change in acceleration and rotational speed (cf. *IMU_data.ino*)

<img width="1710" height="1107" alt="Screenshot 2026-08-11 at 13 59 31" src="https://github.com/user-attachments/assets/e40c5102-673e-4f35-85cf-7b5a36a70d92" />

- We can also have it plotted, graph 1 (rot. speed) after random rotation from rest, and graph 2 (accel.) is after dropping the IMU


<p align="center">
<img width="45%" alt="Screenshot 2026-08-11 at 14 29 04" src="https://github.com/user-attachments/assets/5bcc8c43-6768-4684-a011-f9c405991be5" />
<img width="45%" alt="Screenshot 2026-08-11 at 14 26 14" src="https://github.com/user-attachments/assets/2fd7a70f-e27c-4c49-9248-5340f9350827" />
</p>


**Next steps:**

- [x] Sensor Offset Calibration: Write a setup function that averages ~500 IMU readings while flat on your desk to calculate zero-drift offsets, then subtracts them from live readings

- [x] Attitude Filtering (Angle Calculation): Implement a Complementary Filter or Madgwick Algorithm to fuse raw accelerometer (g) and gyroscope (deg/s) data into clean Roll and Pitch angles in degrees

<img width="400" height="150" alt="Screenshot 2026-08-16 at 13 06 36" src="https://github.com/user-attachments/assets/d3b20c29-980e-4105-9ed3-2ae4483c186e" />

- [x] Non-Blocking Timing Loop: Convert your main code to run at a strict 250 Hz frequency (4000 μs period) using micros() instead of delay()

- [x] PID Controller Framework: Structure the basic Proportional-Integral-Derivative equations for Roll, Pitch, and Yaw to prepare for motor control output

  
- [ ] ESC Signal Generator: Set up 4 ESP32 PWM channels (or DShot protocols) on GPIO pins to verify motor throttle control signals on the Serial Plotter



- [ ] Buy radio controller + receiver + battery stuff


## Entry 7 (16/08/26)

**What I did :**

- I did the calibration. It allows for less noise and slightly better offset (*Graph 1 (left) = no calibration --- Graph 2 (right) = calibrated*)

<p align="center">
  <img width="400" height="250" alt="image" src="https://github.com/user-attachments/assets/40c2f90d-2614-43dd-b71c-5934efe723e3" />
<img width="400" height="250" alt="image" src="https://github.com/user-attachments/assets/e4627b89-4066-428f-8d0c-ac70c7e46380"/> 

</p>


- Did the complementary filter. With mechanics (see dynamics.md) we can use gyro and accel values to compute accurate pitch and roll (not yaw tho). Code is Pitch_roll_values.ino

- It also uses micros() instead of delay() to keep the drone from freezing meanwhile

<img width="801" height="496" alt="Screenshot 2026-08-17 at 18 43 52" src="https://github.com/user-attachments/assets/2219b6b7-dca2-4ddd-894a-f2ba8481bf78" />

- We see a clear discrepency (in smoothness) between values from accelerometer only and from accel+gyro.

## Entry 8 (20/08/26)

**To-do** : 

- [x] PID loop : no need for actual controller data, I can manually input desired commands and check on serial plotter

- [x] Motor matrix mixer : how each motor will act to correct the PID error

- [ ] ESP32 output on ESC : exactly how signal translates to motor speed

- [ ] Buy and plug in radio receiver


## Entry 9 (30/08/26)


*Note : Pitch and roll must be capped at < |90deg| (or even ±45deg possibly) through radio sticks. Above the ±90 mark, Euler Angles and atan methods are no longer applicable (pitch and roll get mixed up,...)*


**What I did** : 

- PID loop : integrated it to the Pitch-roll complementary filter. Few edits and adjustements made (used directly gyro angular speed for derivative term, slight tuning by orders of magnitude to get reasonable correction). File is PID_IMU_combination_v3

<img width=45% alt="Screenshot 2026-08-30 at 15 52 28" src="https://github.com/user-attachments/assets/a2616664-fc6f-4e1f-8c5c-3175ce93179b" />
<img width=45% alt="Screenshot 2026-08-30 at 15 52 17" src="https://github.com/user-attachments/assets/f7f117d5-6de8-48f2-a04e-6de0669c041a" />


**Healthy correction response, good frequency, not much noise. Tuning will be done once drone fully built. D_term and I_term will be more noticeable in real conditions, with wind and actual stick commands

## Entry 10 (31/08/26)

**What I did :**

Motor matrix mixer : I added the matrix to my program, and the pitch/roll/yawRate corrections now translates into motor pulse signal. 

<img width=33% alt="Screenshot 2026-08-31 at 13 58 48" src="https://github.com/user-attachments/assets/e5320839-02b4-47fe-af51-553e6a4b8ed3" />
<img width=33% alt="Screenshot 2026-08-31 at 13 58 37" src="https://github.com/user-attachments/assets/7582dfe1-c512-46f7-9b95-2723a2a48af5" />
<img width=33% alt="Screenshot 2026-08-31 at 13 58 03" src="https://github.com/user-attachments/assets/440f24cb-3494-4213-8059-3c2860fe8413" />

*above each motor pulse signals for a simple roll, pitch and yaw. For yaw, it corrects the rate, so the angular speed but not the orientation (what we want) *

**ESP32 ouput on ESC**

*will use the ESP32 servo library, lightweight and simple function writeMicroseconds(pin, val)*

Pins layout (kinda arbitrary, just avoid strapping and input-only pins):

M1 (Rear-Right): GPIO 13
M2 (Front-Right): GPIO 14
M3 (Rear-Left): GPIO 27
M4 (Front-Left): GPIO 26

Arming and safety :

- must first send a 2-3sec 1000us signal to inform the ESC that the drone is at rest

- must tell the max and min signal for all motors (1000us-2000us)

*added the output pins and set up PWM channels for each motor, that I combined with PID and motor mixing. Now, the ESC receives the right pulse signals, and translates it to RPM using internal programs*


