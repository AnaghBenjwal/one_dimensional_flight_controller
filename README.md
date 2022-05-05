# One Dimensional Rig Test

## Calculating Angle of Inclination

### Gyroscope

- The gyroscope gives angular velocity (in degrees per second) as its output. Integrating angular velocity over time gives us the angle of inclination.

- To calculate present angle from the angular velocity, we do:

``` Gyro_angle_X = Angle_X + Gyro_Data_X * Elapsed_Time ```

- Angular velocity multiplied by elapsed time gives us the change in angle for that period of time.

- This value added to the previous angle in X will give us the current angle in X.

### Accelerometer

- The Accelerometer measures acceleration along its 3 axes.

- Since we know the gravity vector will be constantly downwards, we 

- To measure angle in X direction from accelerometer raw data -
 
``` Acc_angle_X = atan2(Acc_Y, sqrt(Acc_X * Acc_X + Acc_Z * Acc_Z))```

- This value is in radians, we will have to convert it to degrees (by multiplying it with 180/3.1415).

## Complimentary Filter

- We combine the values obtained from the accelerometer and the gyroscope with the complementary filter. This helps us in eliminating drift from gyroscope and twitchiness of accelerometer.

``` filtered_X = 0.98 * (Total_angle_X + Gyro_angle_X * elapsedTime) + 0.02 * Accelerometer_angle_X ```


## PID loop

The PID controller measures the error between our measured value of the angle and the setpoint (0). This error is then corrected by sending the appropriate PWM output signal to the motors.

### Proportional Term

- The proportional term is the proportional gain (Kp) multiplies with the error.

``` pid_p = Kp * error ```

### Integral Term

- The Integral term is the summation of all errors from the first iteration to the current iteration of the loop (Integral of error term) multiplied with Ki.

- To calculate this we add the present error to the value of the integral term calculated in the previous iteration.

``` pid_i = pid_i + (Ki * error) ```

- The integral term should only act to eliminate steady state error, when the error is close to zero. Therefore we constrain the integral term to only act between certain values of error. This is known as Integral Windup.

### Derivative term

- The derivative term is rate of change of error (the derivative of the error) multiplied with Kd.
 
- To find rate of change of error, we need to find the difference between the current value of error and the value of error in the previous iteration and divide it by the time elapsed between the two iterations.

- Once this iteration of the control loop is complete, we will have to store the value of error to use it again in the next iteration.

``` pid_d = Kd * ((error - pervious_error) / elapsed_time) ```

### PID Output

The final output of the PID controller is the sum of the proportional, integral, and derivative terms.

``` output = pid_p + pid_i + pid_d ```

## PWM signal

- Pulse width Modulation is a technique used for getting analog results with digital means.

- A PWM modulated signal is a square wave that rapidly switches between on and off (5V to 0V on Uno).

- This on-off pattern simulates voltages between the full Vcc (5V on Uno) and 0V by changing the time that the signal spends "on" versus the time the signal spends "off".

- This is how we obtain different values of throttle on our motors.

- We use the ```.writeMicroseconds() ``` command to send PWM signals to the motor.

- We cannot send a PWM signal greater than 2000 us or less than 1000 us. These constraints must be added to our code.