#!/bin/bash -e

# copy runcomms.sh to /etc/init.d
sudo cp runcomms /etc/init.d/runcomms

# give permission to runcomms
cd /etc/init.d
sudo chmod 777 runcomms 
sudo chown root:root runcomms

# these dont work
# chkconfig --add runcomms
# chkconfig --level 2345 runcomms on

sudo update-rc.d runcomms defaults
sudo update-rc.d runcomms enable