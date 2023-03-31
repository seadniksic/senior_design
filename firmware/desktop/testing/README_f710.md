# logitech-f710-evdev-interface

Tested using python 3.8.10 on Ubuntu 20.04.5 LTS

virtual_device.py connects to the controller via the linux event subsystem ('/dev/input/eventX'). By default, all of the controller inputs are mapped incorrectly, so the goal of virtual_device.py is to create a virtual game device that mimics the original one, but remap all of the values so it's easier to read and parse the inputs. virtual_device_read.py reads the values in from the virtual device and does all the parsing. 

It is requried to first run virtual_device.py (USING SUDO) and then virtual_device_read.py (USING SUDO)

NOTE: MODE LED SHOULD BE OFF ON THE CONTROLLER
NOTE: FOR THE CONTROLLER TO WORK ON JETSON NANO, NEED TO HAVE THE SWITCH SET TO X. virtual_device.py now have _Dmode and _Xmode appended for the different modes.