#!/bin/bash

source ./include/local_setup.bash
echo "Send request..."

ros2 service call /match/load_next std_srvs/srv/Empty "{}"