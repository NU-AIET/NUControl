![](imgs/NuControl.png)

NU Control is intended to serve as a multi-functional, high performance, motor control library for the Center for Robotics and Biosystems at Northwestern University. The library is current written for a Teensy 4.x microcontroller.

## Capabilities
- Brushless Direct Current Motors
    - Example Motors:
        - Maxon EC45 Flat
        - Mosrac U2535 (In progress)

## Brushless Direct Current Motors
### Field Oriented Control (Torque)
- Field Oriented Control (FOC) is a technique used in brushless motor control that regulates the torque production of a motor. Its main benefits are improved efficiency, precise torque control, and smoother motor operation, especially at low speeds. 
- The full control scheme is shown below. Note that sections (feedforward, feedback, backemf decoupler) can be individually enabled or disabled as desired. High performance control requires information about the motor. The key information required is the individual phase resitance and inductance as well as the motor constants Kt and Kv.
- Feedforward control is achieved through converting a desired torque into a desired current in the Q-axis using the motor torque coefficent Kt. This Q-Axis current is then conerted to the ABC Phase Frame and passed through a discrete filter which predicts the necessary voltage to achieve the desired current. This discrete filter is the inverse of the impedance filter composed of the motor's phase resistance and phase inductance. Therefore, good estimates of both values are critical to performance.

- Feedback control is primarily implemented for distrubrance rejection. It is not recommended to use this controller for the primary driving controller. A desired Q-axis current is compared against the measured Phase Currents which are transformed into the DQ0 frame. The D-axis current is regulated to 0 while the Q-axis current is regulated to the desired value. 

- Back EMF is voltage generated across the motor phases as the motor spins. Necessarily, this changes the current flow across the phases, causing the Q-axis current target to not be achieved. This controller takes the velocity of the motor rotor into account and attempts to remove the back EMF from the phases. 

![](imgs/Control_Diagram.png)


### Open Loop Velocity Control
- By stepping through an electrical cycle at a desired rate, a brushless motor can be spun matching that rate.


## Brushed Direct Current Motors (Coming Soon)
- Brushed motors do not require software commutation due to the presence of physical brushes. This greatly simplifies control at the cost of increased torque ripple.
