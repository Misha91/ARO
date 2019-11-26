#!/usr/bin/env bash

bat=$(rostopic echo -n 1 /mobile_base/sensors/core 2> /dev/null | grep battery | tr -dc '0-9')

if [ -z "$bat" ]
then
	bat='-1'
else
	bat=$(echo "scale=3; (($bat * 0.1)-13.2)/3.3 * 100" | bc)
fi

echo "$bat"