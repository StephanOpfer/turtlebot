#!/bin/bash
rostopic pub /DoorCmd ttb_msgs/DoorCmd '{name: '$1', state: '$2'}'


