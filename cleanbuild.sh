#!/bin/bash

# save the version information
mv .version .version.bak
make mrproper
mv .version.bak .version
/bin/bash afterburnerbuild.sh

