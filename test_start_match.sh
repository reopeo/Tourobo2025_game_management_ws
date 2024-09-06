#!/bin/bash

source ./include/local_setup.bash
echo "Send request..."

ros2 service call /match/start std_srvs/srv/Empty "{}"