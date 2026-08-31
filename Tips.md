## Tips


To read the pulsations (PWM) from the controller on the arduino : 

	// 1. Read raw pulse length from receiver channel 2 (Pitch axis)
	int raw_pitch_pulse = pulseIn(PITCH_PIN, HIGH); // Output: ~1000 to 2000 microseconds
	
	// 2. Map raw microsecond pulse to a target tilt angle in degrees
	// If stick is centered (1500us), target angle = 0 degrees.
	// If stick is full forward (2000us), target angle = +20 degrees.
	float desired_pitch_angle = map(raw_pitch_pulse, 1000, 2000, -20.0, 20.0);


<img width="4032" height="3024" alt="IMG_6517" src="https://github.com/user-attachments/assets/cab47e3e-24fd-4795-a2ba-efac2beb558c" />


Once you have the desired pitched angle, the PID compares it with the IMU actual_pitch_angle to adjust the motors speeds


### Power management

11V is too much voltage for electronics. For this we use a little device (PDB (Power Distribution Board) with a built-in BEC (Battery Eliminator Circuit / Buck Converter), or an ESC board with a 5V BEC.):

<img width="754" height="545" alt="Screenshot 2026-07-23 at 15 39 29" src="https://github.com/user-attachments/assets/06dd68a8-921a-4649-8105-581d04f174b7" />

### Complementary filter

<img width="774" height="350" alt="Screenshot 2026-08-16 at 13 09 07" src="https://github.com/user-attachments/assets/dfa2b14c-f9ec-4144-8ea8-c4301f91bb13" />

**2 jobs :**

1. converts velocity and acceleration into tilt using integration

2. uses both gyroscope and acceleration to give a more precise tilt


### Motor mixing matrix 

- Matrix that tells each motor how much power (with microseconds units) to produce in order to achieve the wanted position/movement. The ESCs translates microseconds pulse signals to actual RPM.

<img width="804" height="580" alt="Screenshot 2026-08-31 at 13 27 39" src="https://github.com/user-attachments/assets/01dc2e9a-3cce-4f06-82ec-43b74017ccad" />



