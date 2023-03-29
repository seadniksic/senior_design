prereqs

linux

visual studio code
with the platformio extension

to get the code to build and upload:
need https://www.pjrc.com/teensy/loader.html for linux. See https://www.pjrc.com/teensy/loader_linux.html for installation instructions
    - make sure to download the rules file as well and follow the commands listed.
need arduino version ??
    - legacy version Arduino IDE 1.8.19, can find on their website. however arduino 2.0 is now supported.
    - https://docs.arduino.cc/software/ide-v1/tutorials/Linux installation instructions
    - seemed to fix missing Arduino.h core file. had issues with the string library but I just uncommented it because i wasn't using it I don't think
    and it is also part of the arduino.h anyway.

uploading to teensy:
    - broken on virtual box Ubuntu vm running on windows
    - fix : https://forum.pjrc.com/threads/63645-Programming-Teensy-4-0-in-a-Windows10-VirtualBox-machine
    - when the teensy is in bootloader, u need to select the halfkey bootloader device in the teensy menu. then run --upload target and itll work. 
