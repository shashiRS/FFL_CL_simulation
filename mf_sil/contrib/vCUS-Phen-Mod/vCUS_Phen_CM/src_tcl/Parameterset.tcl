#! /bin/sh
# next line restarts using wish \
#exec CM -wish "$0" -- "$@"

namespace eval FC {

proc Popup {} {
	exec notepad Data/Sensor/vCUS_Parameterset &
}
# Popup
}
### end of namespace FC