#!/bin/bash
# if [[ "$TRAVIS" != "true" ]] ; then
# 	echo "Running this script makes no sense outside of travis-ci.org"
# 	exit 1
# fi

# Functions
tmstamp() { echo -n "[$(date '+%H:%M:%S')]" ; }

# Environment
NUMTHREADS=2

# if [[ -f /sys/devices/system/cpu/online ]]; then
# if [[ "$CXX" == "g++" ]]; then
#     factor = 1000
# else
#     factor = 1500
# fi
# 	# Calculates 1.5 times physical threads
# 	NUMTHREADS=$(( ( $(cut -f 2 -d '-' /sys/devices/system/cpu/online) + 1 ) * $factor / 1000  ))
# fi

echo "NUMTHREADS = $NUMTHREADS"
#NUMTHREADS=1 # disable MP
export NUMTHREADS

export PDAL_CMAKE_GENERATOR="Unix Makefiles"
