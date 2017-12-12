#!/bin/bash
rostopic pub /DoorCmd hinged_door_controller/DoorCmd '{name: '$1', state: '$2'}'


