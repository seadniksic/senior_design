#!/bin/bash -e

# set -x

# Setup comms permissions
sudo chmod 777 comms.py 
sudo chown root:root comms.py

servicename="runcomms"

# write <servicename>.service file
echo "Writing service file..."
sudo cat >/etc/systemd/system/$servicename.service<< EOL
[Unit]
Description=4.1 Teensy Comms

[Service]
WorkingDirectory=/home/seniordesign
ExecStart=/home/seniordesign/senior_design/firmware/desktop/comms.py
Restart=always
RestartSec=1
KillMode=process
Environment="PYTHONPATH=$PYTHONPATH:/home/seniordesign/.local/lib/python3.8/site-packages

[Install]
WantedBy=mulit-user.target

EOL
echo "Sucessfully wrote service file"

# restart 
sudo systemctl daemon-reload

# stop the service incase it running already
sudo systemctl stop $servicename

# enable the service so it runs always and at boot
sudo systemctl enable $servicename

#start it
sudo systemctl start $servicename

# check the status
sudo systemctl status $servicename

echo "Note, journalctl logs can grow greatly in size."
echo "Follow https://linuxhandbook.com/clear-systemd-journal-logs/ on how to clear them."
journalctl -u $servicename

# set +x

