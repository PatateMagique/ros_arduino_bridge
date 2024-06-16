# Arduino Motor Controller

This Arduino library turns an arduino into a multifunction hardware manager.
It provides a simple serial interface to communicate with a high-level computer (e.g. running ROS), and generates the appropriate PWM signals for motors and other components.

This is a fork from Josh Ewans's [ros_arduino_bridge](https://github.com/joshnewans/ros_arduino_bridge) which is also a fork of the original the original [ros_arduino_bridge](https://github.com/hbrobotics/ros_arduino_bridge) code. Our version is a lightweight and stripped down version of Josh's code. Check out `README-original.md` for the original README and `README-articulated.md` for Josh's README.

As we only need a subset of the functionality originally proposed by this library, the code has been simplified as much as possible and we kept only the features relevant to the project.

## Functionality

The main functionality provided is to receive motor speed requests over a serial connection, control a servo motor and provide system datat feedback.

The main commands to know are:

- `a` - Arduino responds with current open-loop wheel speeds.
- `b` - Arduino responds with current open-loop wheel speeds, battery percentage and the amount of duplos eaten.
- `o <RPM1> <RPM2>` - Set the raw RPM speed of each motor (-7500 to 7500)
- `s <Val>` - Actuate the servo motor depending on val (1 down, 2 up, else iddle)

## Gotchas

Some quick things to note:

- There is an auto timeout (default 1s) so you need to keep sending commands for it to keep moving
- Default baud rate is 9600 for the serial communication and 1 000 000 for the servo motor
- Needs carriage return (CR)
- Make sure serial is enabled (user in dialout group)
- Check out the original readme for more
