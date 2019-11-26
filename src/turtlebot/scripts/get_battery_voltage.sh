#!/usr/bin/env bash

bat=$(rostopic echo -n 1 /mobile_base/sensors/core 2> /dev/null | grep battery | tr -dc '0-9')

if [ -z "$bat" ]
then
	bat='-1'
else
	bat=$(echo "$bat * 0.1" | bc)
fi

echo "$bat"