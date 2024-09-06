#!/bin/bash

source ./include/local_setup.bash
echo "Send request..."

ros2 service call /red/referee_call std_srvs/srv/Empty "{}"