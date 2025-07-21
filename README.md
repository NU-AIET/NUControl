![](imgs/NuControl.png)

NU Control is intended to serve as a multi-functional, high performance, motor control library for the Center for Robotics and Biosystems at Northwestern University. The library is current written for a Teensy 4.x microcontroller.


## Capabilities
- Brushless Direct Current Motors
    - Example Motors:
        - Maxon EC45 Flat
        - Mosrac U2535
- Brushed Direct Current Motors
        - Pittman GM914


## Brushless Direct Current Motors
### Field Oriented Control (Torque)
- Field Oriented Control (FOC) is a technique used in brushless motor control that regulates the torque production of a motor. Its main benefits are improved efficiency, precise torque control, and smoother motor operation, especially at low speeds. 
- The full control scheme is shown below. Note that sections (feedforward, feedback, backemf decoupler) can be individually enabled or disabled as desired. High performance control requires information about the motor. The key information required is the individual phase resitance and inductance as well as the motor constants Kt and Kv. 

![](imgs/Control_Diagram.png)


### Open Loop Velocity Control
- By stepping through an electrical cycle at a desired rate, a brushless motor can be spun matching that rate.


## Brushed Direct Current Motors
- Brushed motors do not require software commutation due to the presence of physical brushes. This greatly simplifies control at the cost of increased torque ripple.
