# How to make comms.py run at boot

Following https://timleland.com/how-to-run-a-linux-program-on-startup:

Use runatbootconfig to set everything up (WIP)
You may need to make runatbootconfig an executable:
`sudo chmod +x runatbootconfig.sh`

This script assumes a couple things:
- servicename will be `runcomms`
- username is seniordesign
- this repo has the root folder located at `/home/<user>/senior_design`
- python3 points to python3.8 and `/home/<user>/.local/lib/python3.8/site-packages` exists.



