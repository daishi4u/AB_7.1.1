#!/bin/bash

rm .version
ABVER="$(date +"%y.%m.%d")"
sed -i 's~\(CONFIG_LOCALVERSION="-Afterburner_N_v\).*"~\1'$ABVER'"~' arch/arm64/configs/j7elte_00_defconfig
sed -i 's~\(ini_set("rom_version",          "\).*");~\1'$ABVER'");~' afterburner/zipsrc/META-INF/com/google/android/aroma-config
sed -i 's~\(ini_set("rom_date",             "\).*");~\1'$(date +"%D")'");~' afterburner/zipsrc/META-INF/com/google/android/aroma-config
/bin/bash buildkernel.sh
/bin/bash builddtimg.sh $ABVER

