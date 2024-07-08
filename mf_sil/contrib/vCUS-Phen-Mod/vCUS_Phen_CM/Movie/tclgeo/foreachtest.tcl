set a1 2566841
set a2 2566800
set b 100
set c [expr $a2 % $b]
puts "Line 5 - Value of c is $c\n"

	set modu [expr $Qu(CPU_Cycle) % 1000]
	set SPosX 0
	set SPosY 0
	set SPosZ 0
	set TPosX 0
	set TPosY 0
	
	if {$modu == 0} {
		foreach SensNo [list 0 0 0 1 1 1 2 2 2 3 3 3 4 4 4 5 5 5 6 6 6 7 7 7 8 8 8 9 9 9 10 10 10 11 11 11] TrafNo [list 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2] {
			if {$Qu(SensData.$SensNo.Target.NPPos.x.$TrafNo) != 0 && $Qu(SensData.$SensNo.Target.NPPos.y.$TrafNo) != 0} {
				glDisableLighting
				gl color {*}$ContiOrange;
				gl linewidth 1;
				gl begin line_strip		
				gl vertex $Qu(SensData.$SensNo.SensPos.x) $Qu(SensData.$SensNo.SensPos.y) $Qu(SensData.$SensNo.SensPos.z)
				gl vertex $Qu(SensData.$SensNo.Target.NPPos.x.$TrafNo) $Qu(SensData.0.Target.NPPos.y.$TrafNo) $Qu(SensData.$SensNo.SensPos.z)
				gl end
				glEnableLighting

				set SPosX $Qu(SensData.$SensNo.SensPos.x)
				set SPosY $Qu(SensData.$SensNo.SensPos.y)
				set SPosZ $Qu(SensData.$SensNo.SensPos.z)
				set TPosX $Qu(SensData.$SensNo.Target.NPPos.x.$TrafNo)
				set TPosY $Qu(SensData.$SensNo.Target.NPPos.y.$TrafNo)
			}
		}
	} else {
		foreach SensNo [list 0 0 0 1 1 1 2 2 2 3 3 3 4 4 4 5 5 5 6 6 6 7 7 7 8 8 8 9 9 9 10 10 10 11 11 11] TrafNo [list 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2] {
			if {$Qu(SensData.$SensNo.Target.NPPos.x.$TrafNo) != 0 && $Qu(SensData.$SensNo.Target.NPPos.y.$TrafNo) != 0} {
				glDisableLighting
				gl color {*}$ContiOrange;
				gl linewidth 1;
				gl begin line_strip		
				gl vertex $SPosX $SPosY $SPosZ
				gl vertex $TPosX $TPosY $SPosZ
				gl end
				glEnableLighting			
			}
		}
	}
	
	
	
### BEGIN IPG-MOVIE-INFO
#Subscribe SensData.0.Target.NPPos.x.0 SensData.0.Target.NPPos.y.0 SensData.0.Target.NPPos.x.1 SensData.0.Target.NPPos.y.1 SensData.0.Target.NPPos.x.2 SensData.0.Target.NPPos.y.2
#Subscribe SensData.1.Target.NPPos.x.0 SensData.1.Target.NPPos.y.0 SensData.1.Target.NPPos.x.1 SensData.1.Target.NPPos.y.1 SensData.1.Target.NPPos.x.2 SensData.1.Target.NPPos.y.2
#Subscribe SensData.2.Target.NPPos.x.0 SensData.2.Target.NPPos.y.0 SensData.2.Target.NPPos.x.1 SensData.2.Target.NPPos.y.1 SensData.2.Target.NPPos.x.2 SensData.2.Target.NPPos.y.2
#Subscribe SensData.3.Target.NPPos.x.0 SensData.3.Target.NPPos.y.0 SensData.3.Target.NPPos.x.1 SensData.3.Target.NPPos.y.1 SensData.3.Target.NPPos.x.2 SensData.3.Target.NPPos.y.2
#Subscribe SensData.4.Target.NPPos.x.0 SensData.4.Target.NPPos.y.0 SensData.4.Target.NPPos.x.1 SensData.4.Target.NPPos.y.1 SensData.4.Target.NPPos.x.2 SensData.4.Target.NPPos.y.2
#Subscribe SensData.5.Target.NPPos.x.0 SensData.5.Target.NPPos.y.0 SensData.5.Target.NPPos.x.1 SensData.5.Target.NPPos.y.1 SensData.5.Target.NPPos.x.2 SensData.5.Target.NPPos.y.2
#Subscribe SensData.6.Target.NPPos.x.0 SensData.6.Target.NPPos.y.0 SensData.6.Target.NPPos.x.1 SensData.6.Target.NPPos.y.1 SensData.6.Target.NPPos.x.2 SensData.6.Target.NPPos.y.2
#Subscribe SensData.7.Target.NPPos.x.0 SensData.7.Target.NPPos.y.0 SensData.7.Target.NPPos.x.1 SensData.7.Target.NPPos.y.1 SensData.7.Target.NPPos.x.2 SensData.7.Target.NPPos.y.2
#Subscribe SensData.8.Target.NPPos.x.0 SensData.8.Target.NPPos.y.0 SensData.8.Target.NPPos.x.1 SensData.8.Target.NPPos.y.1 SensData.8.Target.NPPos.x.2 SensData.8.Target.NPPos.y.2
#Subscribe SensData.9.Target.NPPos.x.0 SensData.9.Target.NPPos.y.0 SensData.9.Target.NPPos.x.1 SensData.9.Target.NPPos.y.1 SensData.9.Target.NPPos.x.2 SensData.9.Target.NPPos.y.2
#Subscribe SensData.10.Target.NPPos.x.0 SensData.10.Target.NPPos.y.0 SensData.10.Target.NPPos.x.1 SensData.10.Target.NPPos.y.1 SensData.10.Target.NPPos.x.2 SensData.10.Target.NPPos.y.2
#Subscribe SensData.11.Target.NPPos.x.0 SensData.11.Target.NPPos.y.0 SensData.11.Target.NPPos.x.1 SensData.11.Target.NPPos.y.1 SensData.11.Target.NPPos.x.2 SensData.11.Target.NPPos.y.2
#Subscribe SensData.0.SensPos.x SensData.0.SensPos.y SensData.0.SensPos.z
#Subscribe SensData.1.SensPos.x SensData.1.SensPos.y SensData.1.SensPos.z
#Subscribe SensData.2.SensPos.x SensData.2.SensPos.y SensData.2.SensPos.z
#Subscribe SensData.3.SensPos.x SensData.3.SensPos.y SensData.3.SensPos.z
#Subscribe SensData.4.SensPos.x SensData.4.SensPos.y SensData.4.SensPos.z
#Subscribe SensData.5.SensPos.x SensData.5.SensPos.y SensData.5.SensPos.z
#Subscribe SensData.6.SensPos.x SensData.6.SensPos.y SensData.6.SensPos.z
#Subscribe SensData.7.SensPos.x SensData.7.SensPos.y SensData.7.SensPos.z
#Subscribe SensData.8.SensPos.x SensData.8.SensPos.y SensData.8.SensPos.z
#Subscribe SensData.9.SensPos.x SensData.9.SensPos.y SensData.9.SensPos.z
#Subscribe SensData.10.SensPos.x SensData.10.SensPos.y SensData.10.SensPos.z
#Subscribe SensData.11.SensPos.x SensData.11.SensPos.y SensData.11.SensPos.z
#Subscribe CPU_Cycle
### END IPG-MOVIE-INFO


proc DrawBackground {} {
    global Qu
        
    set ContiOrange 	{1 0.647 0 2}
	set red				{1 0 0}
	set LightGrey		{0.65 0.65 0.65}

	# foreach SensNo [list 0 0 0 1 1 1 2 2 2 3 3 3 4 4 4 5 5 5 6 6 6 7 7 7 8 8 8 9 9 9 10 10 10 11 11 11] TrafNo [list 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2 0 1 2] {
		# if {$Qu(SensData.$SensNo.Target.NPPos.x.$TrafNo) != 0 && $Qu(SensData.$SensNo.Target.NPPos.y.$TrafNo) != 0} {
			# glDisableLighting
			# gl color {*}$ContiOrange;
			# gl linewidth 1;
			# gl begin line_strip
			
			# gl vertex $Qu(SensData.$SensNo.SensPos.x) $Qu(SensData.$SensNo.SensPos.y) $Qu(SensData.$SensNo.SensPos.z)
			# gl vertex $Qu(SensData.$SensNo.Target.NPPos.x.$TrafNo) $Qu(SensData.$SensNo.Target.NPPos.y.$TrafNo) $Qu(SensData.$SensNo.SensPos.z)
			
			# gl end
			# glEnableLighting
		# }
	# }



	if {$Qu(SensData.0.Target.NPPos.x.0) != 0 && $Qu(SensData.0.Target.NPPos.y.0) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.0.SensPos.x) $Qu(SensData.0.SensPos.y) $Qu(SensData.0.SensPos.z)
		gl vertex $Qu(SensData.0.Target.NPPos.x.0) $Qu(SensData.0.Target.NPPos.y.0) $Qu(SensData.0.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.0.Target.NPPos.x.1) != 0 && $Qu(SensData.0.Target.NPPos.y.1) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.0.SensPos.x) $Qu(SensData.0.SensPos.y) $Qu(SensData.0.SensPos.z)
		gl vertex $Qu(SensData.0.Target.NPPos.x.1) $Qu(SensData.0.Target.NPPos.y.1) $Qu(SensData.0.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.0.Target.NPPos.x.2) != 0 && $Qu(SensData.0.Target.NPPos.y.2) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.0.SensPos.x) $Qu(SensData.0.SensPos.y) $Qu(SensData.0.SensPos.z)
		gl vertex $Qu(SensData.0.Target.NPPos.x.2) $Qu(SensData.0.Target.NPPos.y.2) $Qu(SensData.0.SensPos.z)

		gl end
		glEnableLighting
    }
	
	
	
	if {$Qu(SensData.1.Target.NPPos.x.0) != 0 && $Qu(SensData.1.Target.NPPos.y.0) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.1.SensPos.x) $Qu(SensData.1.SensPos.y) $Qu(SensData.1.SensPos.z)
		gl vertex $Qu(SensData.1.Target.NPPos.x.0) $Qu(SensData.1.Target.NPPos.y.0) $Qu(SensData.1.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.1.Target.NPPos.x.1) != 0 && $Qu(SensData.1.Target.NPPos.y.1) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.1.SensPos.x) $Qu(SensData.1.SensPos.y) $Qu(SensData.1.SensPos.z)
		gl vertex $Qu(SensData.1.Target.NPPos.x.1) $Qu(SensData.1.Target.NPPos.y.1) $Qu(SensData.1.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.1.Target.NPPos.x.2) != 0 && $Qu(SensData.1.Target.NPPos.y.2) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.1.SensPos.x) $Qu(SensData.1.SensPos.y) $Qu(SensData.1.SensPos.z)
		gl vertex $Qu(SensData.1.Target.NPPos.x.2) $Qu(SensData.1.Target.NPPos.y.2) $Qu(SensData.1.SensPos.z)

		gl end
		glEnableLighting
    }
	
	
	
	if {$Qu(SensData.2.Target.NPPos.x.0) != 0 && $Qu(SensData.2.Target.NPPos.y.0) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.2.SensPos.x) $Qu(SensData.2.SensPos.y) $Qu(SensData.2.SensPos.z)
		gl vertex $Qu(SensData.2.Target.NPPos.x.0) $Qu(SensData.2.Target.NPPos.y.0) $Qu(SensData.2.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.2.Target.NPPos.x.1) != 0 && $Qu(SensData.2.Target.NPPos.y.1) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.2.SensPos.x) $Qu(SensData.2.SensPos.y) $Qu(SensData.2.SensPos.z)
		gl vertex $Qu(SensData.2.Target.NPPos.x.1) $Qu(SensData.2.Target.NPPos.y.1) $Qu(SensData.2.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.2.Target.NPPos.x.2) != 0 && $Qu(SensData.2.Target.NPPos.y.2) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.2.SensPos.x) $Qu(SensData.2.SensPos.y) $Qu(SensData.2.SensPos.z)
		gl vertex $Qu(SensData.2.Target.NPPos.x.2) $Qu(SensData.2.Target.NPPos.y.2) $Qu(SensData.2.SensPos.z)

		gl end
		glEnableLighting
    }
	
	

	if {$Qu(SensData.3.Target.NPPos.x.0) != 0 && $Qu(SensData.3.Target.NPPos.y.0) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.3.SensPos.x) $Qu(SensData.3.SensPos.y) $Qu(SensData.3.SensPos.z)
		gl vertex $Qu(SensData.3.Target.NPPos.x.0) $Qu(SensData.3.Target.NPPos.y.0) $Qu(SensData.3.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.3.Target.NPPos.x.1) != 0 && $Qu(SensData.3.Target.NPPos.y.1) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.3.SensPos.x) $Qu(SensData.3.SensPos.y) $Qu(SensData.3.SensPos.z)
		gl vertex $Qu(SensData.3.Target.NPPos.x.1) $Qu(SensData.3.Target.NPPos.y.1) $Qu(SensData.3.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.3.Target.NPPos.x.2) != 0 && $Qu(SensData.3.Target.NPPos.y.2) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.3.SensPos.x) $Qu(SensData.3.SensPos.y) $Qu(SensData.3.SensPos.z)
		gl vertex $Qu(SensData.3.Target.NPPos.x.2) $Qu(SensData.3.Target.NPPos.y.2) $Qu(SensData.3.SensPos.z)

		gl end
		glEnableLighting
    }
	
	
	
	if {$Qu(SensData.4.Target.NPPos.x.0) != 0 && $Qu(SensData.4.Target.NPPos.y.0) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.4.SensPos.x) $Qu(SensData.4.SensPos.y) $Qu(SensData.4.SensPos.z)
		gl vertex $Qu(SensData.4.Target.NPPos.x.0) $Qu(SensData.4.Target.NPPos.y.0) $Qu(SensData.4.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.4.Target.NPPos.x.1) != 0 && $Qu(SensData.4.Target.NPPos.y.1) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.4.SensPos.x) $Qu(SensData.4.SensPos.y) $Qu(SensData.4.SensPos.z)
		gl vertex $Qu(SensData.4.Target.NPPos.x.1) $Qu(SensData.4.Target.NPPos.y.1) $Qu(SensData.4.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.4.Target.NPPos.x.2) != 0 && $Qu(SensData.4.Target.NPPos.y.2) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.4.SensPos.x) $Qu(SensData.4.SensPos.y) $Qu(SensData.4.SensPos.z)
		gl vertex $Qu(SensData.4.Target.NPPos.x.2) $Qu(SensData.4.Target.NPPos.y.2) $Qu(SensData.4.SensPos.z)

		gl end
		glEnableLighting
    }
	
	
	
	if {$Qu(SensData.5.Target.NPPos.x.0) != 0 && $Qu(SensData.5.Target.NPPos.y.0) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.5.SensPos.x) $Qu(SensData.5.SensPos.y) $Qu(SensData.5.SensPos.z)
		gl vertex $Qu(SensData.5.Target.NPPos.x.0) $Qu(SensData.5.Target.NPPos.y.0) $Qu(SensData.5.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.5.Target.NPPos.x.1) != 0 && $Qu(SensData.5.Target.NPPos.y.1) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.5.SensPos.x) $Qu(SensData.5.SensPos.y) $Qu(SensData.5.SensPos.z)
		gl vertex $Qu(SensData.5.Target.NPPos.x.1) $Qu(SensData.5.Target.NPPos.y.1) $Qu(SensData.5.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.5.Target.NPPos.x.2) != 0 && $Qu(SensData.5.Target.NPPos.y.2) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.5.SensPos.x) $Qu(SensData.5.SensPos.y) $Qu(SensData.5.SensPos.z)
		gl vertex $Qu(SensData.5.Target.NPPos.x.2) $Qu(SensData.5.Target.NPPos.y.2) $Qu(SensData.5.SensPos.z)

		gl end
		glEnableLighting
    }
	
	
	
	if {$Qu(SensData.6.Target.NPPos.x.0) != 0 && $Qu(SensData.6.Target.NPPos.y.0) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.6.SensPos.x) $Qu(SensData.6.SensPos.y) $Qu(SensData.6.SensPos.z)
		gl vertex $Qu(SensData.6.Target.NPPos.x.0) $Qu(SensData.6.Target.NPPos.y.0) $Qu(SensData.6.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.6.Target.NPPos.x.1) != 0 && $Qu(SensData.6.Target.NPPos.y.1) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.6.SensPos.x) $Qu(SensData.6.SensPos.y) $Qu(SensData.6.SensPos.z)
		gl vertex $Qu(SensData.6.Target.NPPos.x.1) $Qu(SensData.6.Target.NPPos.y.1) $Qu(SensData.6.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.6.Target.NPPos.x.2) != 0 && $Qu(SensData.6.Target.NPPos.y.2) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.6.SensPos.x) $Qu(SensData.6.SensPos.y) $Qu(SensData.6.SensPos.z)
		gl vertex $Qu(SensData.6.Target.NPPos.x.2) $Qu(SensData.6.Target.NPPos.y.2) $Qu(SensData.6.SensPos.z)

		gl end
		glEnableLighting
    }
	
	
	
	if {$Qu(SensData.7.Target.NPPos.x.0) != 0 && $Qu(SensData.7.Target.NPPos.y.0) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.7.SensPos.x) $Qu(SensData.7.SensPos.y) $Qu(SensData.7.SensPos.z)
		gl vertex $Qu(SensData.7.Target.NPPos.x.0) $Qu(SensData.7.Target.NPPos.y.0) $Qu(SensData.7.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.7.Target.NPPos.x.1) != 0 && $Qu(SensData.7.Target.NPPos.y.1) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.7.SensPos.x) $Qu(SensData.7.SensPos.y) $Qu(SensData.7.SensPos.z)
		gl vertex $Qu(SensData.7.Target.NPPos.x.1) $Qu(SensData.7.Target.NPPos.y.1) $Qu(SensData.7.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.7.Target.NPPos.x.2) != 0 && $Qu(SensData.7.Target.NPPos.y.2) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.7.SensPos.x) $Qu(SensData.7.SensPos.y) $Qu(SensData.7.SensPos.z)
		gl vertex $Qu(SensData.7.Target.NPPos.x.2) $Qu(SensData.7.Target.NPPos.y.2) $Qu(SensData.7.SensPos.z)

		gl end
		glEnableLighting
    }
	
	
	
	if {$Qu(SensData.8.Target.NPPos.x.0) != 0 && $Qu(SensData.8.Target.NPPos.y.0) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.8.SensPos.x) $Qu(SensData.8.SensPos.y) $Qu(SensData.8.SensPos.z)
		gl vertex $Qu(SensData.8.Target.NPPos.x.0) $Qu(SensData.8.Target.NPPos.y.0) $Qu(SensData.8.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.8.Target.NPPos.x.1) != 0 && $Qu(SensData.8.Target.NPPos.y.1) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.8.SensPos.x) $Qu(SensData.8.SensPos.y) $Qu(SensData.8.SensPos.z)
		gl vertex $Qu(SensData.8.Target.NPPos.x.1) $Qu(SensData.8.Target.NPPos.y.1) $Qu(SensData.8.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.8.Target.NPPos.x.2) != 0 && $Qu(SensData.8.Target.NPPos.y.2) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.8.SensPos.x) $Qu(SensData.8.SensPos.y) $Qu(SensData.8.SensPos.z)
		gl vertex $Qu(SensData.8.Target.NPPos.x.2) $Qu(SensData.8.Target.NPPos.y.2) $Qu(SensData.8.SensPos.z)

		gl end
		glEnableLighting
    }
	
	
	
	if {$Qu(SensData.9.Target.NPPos.x.0) != 0 && $Qu(SensData.9.Target.NPPos.y.0) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.9.SensPos.x) $Qu(SensData.9.SensPos.y) $Qu(SensData.9.SensPos.z)
		gl vertex $Qu(SensData.9.Target.NPPos.x.0) $Qu(SensData.9.Target.NPPos.y.0) $Qu(SensData.9.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.9.Target.NPPos.x.1) != 0 && $Qu(SensData.9.Target.NPPos.y.1) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.9.SensPos.x) $Qu(SensData.9.SensPos.y) $Qu(SensData.9.SensPos.z)
		gl vertex $Qu(SensData.9.Target.NPPos.x.1) $Qu(SensData.9.Target.NPPos.y.1) $Qu(SensData.9.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.9.Target.NPPos.x.2) != 0 && $Qu(SensData.9.Target.NPPos.y.2) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.9.SensPos.x) $Qu(SensData.9.SensPos.y) $Qu(SensData.9.SensPos.z)
		gl vertex $Qu(SensData.9.Target.NPPos.x.2) $Qu(SensData.9.Target.NPPos.y.2) $Qu(SensData.9.SensPos.z)

		gl end
		glEnableLighting
    }
	
	
	
	if {$Qu(SensData.10.Target.NPPos.x.0) != 0 && $Qu(SensData.10.Target.NPPos.y.0) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.10.SensPos.x) $Qu(SensData.10.SensPos.y) $Qu(SensData.10.SensPos.z)
		gl vertex $Qu(SensData.10.Target.NPPos.x.0) $Qu(SensData.10.Target.NPPos.y.0) $Qu(SensData.10.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.10.Target.NPPos.x.1) != 0 && $Qu(SensData.10.Target.NPPos.y.1) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.10.SensPos.x) $Qu(SensData.10.SensPos.y) $Qu(SensData.10.SensPos.z)
		gl vertex $Qu(SensData.10.Target.NPPos.x.1) $Qu(SensData.10.Target.NPPos.y.1) $Qu(SensData.10.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.10.Target.NPPos.x.2) != 0 && $Qu(SensData.10.Target.NPPos.y.2) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.10.SensPos.x) $Qu(SensData.10.SensPos.y) $Qu(SensData.10.SensPos.z)
		gl vertex $Qu(SensData.10.Target.NPPos.x.2) $Qu(SensData.10.Target.NPPos.y.2) $Qu(SensData.10.SensPos.z)

		gl end
		glEnableLighting
    }
	
	
	
	if {$Qu(SensData.11.Target.NPPos.x.0) != 0 && $Qu(SensData.11.Target.NPPos.y.0) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.11.SensPos.x) $Qu(SensData.11.SensPos.y) $Qu(SensData.11.SensPos.z)
		gl vertex $Qu(SensData.11.Target.NPPos.x.0) $Qu(SensData.11.Target.NPPos.y.0) $Qu(SensData.11.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.11.Target.NPPos.x.1) != 0 && $Qu(SensData.11.Target.NPPos.y.1) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.11.SensPos.x) $Qu(SensData.11.SensPos.y) $Qu(SensData.11.SensPos.z)
		gl vertex $Qu(SensData.11.Target.NPPos.x.1) $Qu(SensData.11.Target.NPPos.y.1) $Qu(SensData.11.SensPos.z)

		gl end
		glEnableLighting
    }
	if {$Qu(SensData.11.Target.NPPos.x.2) != 0 && $Qu(SensData.11.Target.NPPos.y.2) != 0} {
		glDisableLighting
		gl color {*}$ContiOrange; #choose the color of trajectory
		gl linewidth 1; #change the width of trajectory
		gl begin line_strip
		
		gl vertex $Qu(SensData.11.SensPos.x) $Qu(SensData.11.SensPos.y) $Qu(SensData.11.SensPos.z)
		gl vertex $Qu(SensData.11.Target.NPPos.x.2) $Qu(SensData.11.Target.NPPos.y.2) $Qu(SensData.11.SensPos.z)

		gl end
		glEnableLighting
    }
}

return -1