import evdev
import atexit
from evdev import categorize, ecodes

# plan
# define an update rate
# for 1/update rate seconds, read inputs
# sift through all the inputs and store it a struct that holds the state for everyting

# OR
# always update the struct as fast as possible but then only send the contents at update rate.
# should do this ebcause idk if there is a way to "flush" i.e on flushing if the user to constanly providing
# input, it will never stop

class g:
    update_rate = 10 # in hz
    time_to_read = 1 / update_rate
    joy_name = "devremap"
    test_input = True

dev = None
found = False
devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
for device in devices:
    print(device.path, device.name, device.phys)
    if(device.name == g.joy_name):
        dev = device
        found = True
        print("Found correct controller.")

if not found:
    print("unable to find joystick. exiting...")
    exit()

print("---------------")

print("printing device capabilities")
for item in dev.capabilities(verbose=True).items():
    print("")
    print(item) 

print("---------------")

atexit.register(dev.ungrab) #bind the function so its called at program exit
dev.grab() # grab the device so no other program can use it.
            # also prevents keyboard from omitting original events?

# this relies on the following files
# /usr/include/linux/input.h
# /usr/include/linux/input-event-codes.h

REMAP_DICT = {
    ecodes.BTN_C : ecodes.BTN_B
}


if g.test_input:
    for event in dev.read_loop():

        # all buttons (including the joystick switch) besides mode, vibration, and D pad.
        # and logitech button which idk what that does.
        if event.type == ecodes.EV_KEY:
            print(categorize(event))

        # mode looks like it switches D -pad with Left joystick in terms of the values that it reads.
        # but on these event is dpad rjoy and ljoy
        if event.type == ecodes.EV_ABS:
            # print(categorize(event))
            print(event)

        # none of the buttons are on here
        if event.type == ecodes.EV_REL:
            print(categorize(event))

else:
    with evdev.UInput.from_device(dev, name="devremap") as ui:
        for event in dev.read_loop():
            if event.type == ecodes.EV_KEY:
                if event.code in REMAP_DICT:
                    remapped_code = REMAP_DICT[event.code]
                    ui.write(ecodes.EV_KEY, remapped_code, event.value)
                else:
                    ui.write(ecodes.EV_KEY, event.code, event.value)
            else:
                ui.write(event.type, event.code, event.value)

    
    