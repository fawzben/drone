# ME-PROJECT-BUILDING-A-DRONE-FROM-SCRATCH
Summer holiday between 1st and 2nd year EPFL ME. I want to build a fully functional drone

## Entry 1 (19/07/25)

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


## Entry 2 (20/08/25)


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


## Entry 3 (22/08/25)

**What I did :**

- Decided to do CAD on Fusion (no need for Catia for a not-so-big construction)


- Started building the frame

<img width="1233" height="859" alt="Screenshot 2026-07-22 at 13 04 00" src="https://github.com/user-attachments/assets/5e556e6b-443d-404a-99b9-d808baaa474a" />

## Entry 4 (25/08/25)

**What I did :**

- More CAD done, first workable/printable version ready


- Ordered the propellers and motors, only battery stuff & controller left to buy



<img width="1467" height="853" alt="Screenshot 2026-07-25 at 20 15 56" src="https://github.com/user-attachments/assets/2b90b9ce-6fa8-456c-b038-0aa6487e2da5" />



**Next steps :**

- Maybe print the 1st version


- Look at open source code for inspo to configure the microcontroller


- (Order the other stuff)

## Entry 5 (5/08/25)

** What I did during 10 days** : 

- Ordered and received most of the material

<img width="4032" height="3024" alt="IMG_6576" src="https://github.com/user-attachments/assets/146ec5cb-42bd-4ec1-bf58-bb1121359821" />

- Edited the CAD and ran simulations

- 3D-Printed this workable frame

<img width="4032" height="3024" alt="IMG_6613" src="https://github.com/user-attachments/assets/031ef587-f977-49c5-83dc-54322249d66e" />

 - Started programming and wiring microcontroller and IMU
 

<img width="4032" height="3024" alt="IMG_6610" src="https://github.com/user-attachments/assets/abea6eb7-91ef-4cb9-8b00-fcf3a78e7315" />


## Entry 6 (11/08/25)

**What I did :**

- Coded for live IMU data report. It works well, giving me 1g of acceleration and 0±1 deg/s of rotation at rest state. It also detects change in acceleration and rotational speed (cf. *IMU_data.ino*)

<img width="1710" height="1107" alt="Screenshot 2026-08-11 at 13 59 31" src="https://github.com/user-attachments/assets/e40c5102-673e-4f35-85cf-7b5a36a70d92" />

- We can also have it plotted, graph 1 (rot. speed) after random rotation from rest, and graph 2 (accel.) is after dropping the IMU

<img width="825" height="521" alt="Screenshot 2026-08-11 at 14 29 04" src="https://github.com/user-attachments/assets/5bcc8c43-6768-4684-a011-f9c405991be5" />
<img width="803" height="506" alt="Screenshot 2026-08-11 at 14 26 14" src="https://github.com/user-attachments/assets/2fd7a70f-e27c-4c49-9248-5340f9350827" />











