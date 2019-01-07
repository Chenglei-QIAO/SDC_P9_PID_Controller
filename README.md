# PID Control Project

Self-Driving Car Engineer Nanodegree Program

---

## Introductions

This project is the 4th project in Terms 2 of Udacity Self-Driving Car Engineer Nanodegree Program. The goal of this project is to build a PID controller and tune the PID hyperparameters by applying the general processing flow as described in the lessons to maneuver the vehicle around the track!

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`
5. Run the Term2 Simulator and visualize the result.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## What is PID controllers

$$K_pe + K_d\frac{d_e}{d_t} + K_i\int e(t)dt$$

A PID Controller in essence is a method of stabilizing a system, whether that be a quadcopter, the fuel rods in a nuclear reactor, or the angle of an arm for a bio-mechanical dog harness.

A PID takes an amount of error (more on that later) and a method of reducing said error and calculates a rate to progressively solve it. For example, it is the method of taking angle data from a gyroscope and telling each rotor how fast to rotate.

The P, I, and D stand for Proportional, Derivative, and Integral gain. More on that later as well!

## The role of each mode

Proportional: this mode determines the speed of response for the loop. The higher the controller gain, the faster the loop respoonds. However, higher controller gains lead to overshoot and oscilations in the response. For each loop, there is a maximum value of the controller gain, known as the ultimate gain, for which the loop is stable.

Integral: Contraty to popular belief, the integral mode does not produce a faster response. The only contribution of the integral mode is to ensure that the loop can line out only at its setpoints (i.e. there is no drop). It accomplishes this by adjusting the value of the controller output bias($M_R$). The bias must be changed at a rate consistent with the response characteristics of the process. Changing the bias too rapidly creates more oscillations and, in extreme cases, an unstable loop.

Derivative：In some applications, derivative reduces the overshoot and oscillations. In turn, this permits a higher value to be used for the controller gain, which increases the speed of response.

## The strategy to tune PID parameters

The key to implement a SAFETY and stable PID controller is to choose the optimal parameters for PID controller.

In the lessons, there is a method called 'twiddle algorithm', also known as 'coordinate ascent', is a  generic algorithm that tries to find a good choice of parameters for an algorithm that returns an error.

There ia another method in this [post](https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops):

    To tune a PID use the following steps:
       1. Set all gains to zero.
       2. Increase the P gain until the response to a disturbance is steady oscillation.
       3. Increase the D gain until the the oscillations go away (i.e. it's critically damped).
       4. Repeat steps 2 and 3 until increasing the D gain does not stop the oscillations.
       5. Set P and D to the last stable values.
       6. Increase the I gain until it brings you to the setpoint with the number of oscillations desired (normally zero but a quicker response can be had if you don't mind a couple oscillations of overshoot)

Following this method, I tried to manually interate with different parameters and finally managed to get good enough results. The final parameters I chose in my impelementation is：

1. P: 0.15
2. I: 0.00003
3. D: 3.

and The result looks good. The car was quite stable during the simulation and didn't leave the driable portion of the track.
![The screenshot of simulation](./images/__pid__.png)

The issue at the moment is that the car can only drive stably at about 30mph, and perhaps what needs to do next is to tune the parameters to get the car driving faster.