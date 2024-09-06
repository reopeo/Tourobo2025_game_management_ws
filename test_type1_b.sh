#!/bin/bash

source ./include/local_setup.bash
echo "Send request..."

ros2 service call /red/update_score game_state_interfaces/srv/UpdateScore "{command: 4, data: $1}"