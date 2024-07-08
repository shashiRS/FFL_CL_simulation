# Set TextEditor
#set Pgm(TextEditor.win32) {C:/Programme/ConTEXT/ConTEXT.exe $fn &}
#set Pgm(TextEditor.win64) {"C:\\PROGRAM FILES\\NOTEPAD++\\NOTEPAD++.EXE" $fn &}
#set Pgm(TextEditor.linux) {kate $fn &}

### CarMaker Add-ons
source src_tcl/Parameterset.tcl
menubutton .mbar.special -text "Conti vCUS" -menu .mbar.special.m
pack .mbar.special -side left
set m [menu .mbar.special.m]
$m add com -lab [l "vCUS Parameterset" "vCUS Parametersatz"]   -command {FC::Popup}
