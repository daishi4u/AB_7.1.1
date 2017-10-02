#!/bin/bash

ABVER="$(date +"%y.%m.%d")"

if [ -e last_build_date.log ] && grep -q $ABVER last_build_date.log
then
	BVER=$(($(cat .version) + 1))
	ABVER="$(echo $ABVER).$BVER"
else
	echo $ABVER > last_build_date.log
	ABVER="$(echo $ABVER).1"
	
	if [ -e .version ]
	then
		rm .version
	fi
fi

sed -i 's~\(CONFIG_LOCALVERSION="-Afterburner_N_v\).*"~\1'$ABVER'"~' arch/arm64/configs/j7elte_00_defconfig
sed -i 's~\(ini_set("rom_version",          "\).*");~\1'$ABVER'");~' afterburner/zipsrc/META-INF/com/google/android/aroma-config
sed -i 's~\(ini_set("rom_date",             "\).*");~\1'$(date +"%D")'");~' afterburner/zipsrc/META-INF/com/google/android/aroma-config
/bin/bash buildkernel.sh
/bin/bash builddtimg.sh $ABVER

