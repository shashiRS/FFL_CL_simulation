	for (int k = 0; k < int_var->noTargets; k++) {
		double min = 9999;

		/* Determine next target entry */
		for (int l = 0; l < int_var->noTargets; l++) {
			if (int_var->distance[l] < min) {
				min = int_var->distance[l];
				int_var->ID = l;
			}
		}

		if (k == 0) {
			// Hier muss folgendes gemacht werden:
			// - In den boundary_points Angle stehten die Infos für jedes TO drin
			// - Man muss jetzt prüfen, welches TO denn laut NP am nächsten steht
			// - Diese Info zusammen mit der boundary info reicht aus
			// - um dann eine "Reihenfolge" zu definieren
			// - Diese Reihenfolge muss dann abgefragt werden
			// - Nur dann kann der Effekt, dass ein Verdecktes Objekt gesehen wird, weil das nähere Objekt keinen Reflektionspunkt hat
			// - Oben werden nur die TARGETS nach der Größe sortiert
			// - Das muss dann eigtl. auch für die Nicht-Reflektionpunk-TOs gemacht werden
			// - Für jeden ReflPoint wird dann überprüft, ob irgendwo noch ein TO dazwischen steht

			SensData[j].distance[k] = int_var->distance[int_var->ID];
			SensData[j].Detc_TO_ID[k] = int_var->Detc_TO_ID[int_var->ID];
			SensData[j].Detc_TO_Surface_ID[k] = int_var->Detc_TO_Surface_ID[int_var->ID];
			SensData[j].Fr0_position_NP[k][0] = int_var->Fr0_position_NP[int_var->ID][0];
			SensData[j].Fr0_position_NP[k][1] = int_var->Fr0_position_NP[int_var->ID][1];
			SensData[j].Fr0_position_NP[k][2] = int_var->Fr0_position_NP[int_var->ID][2];
			SensData[j].NPDir_Fr0[k] = int_var->NPDir_Fr0[int_var->ID];

			int_var->distance[int_var->ID] = 9999;
		}
		else {
			/* Starting one after least filled target in list 'k' */
			for (int l = 0; l < k; l++) {
				//if (TrfObj[SensData[j].Detc_TO_ID[l]].boundary_pointsAngles_min[j] > TrfObj[int_var->ID].NearestPointDir_Fr0[j] * (-1)
				//	&& TrfObj[SensData[j].Detc_TO_ID[l]].NearestPointDir_Fr0[j] * (-1) > TrfObj[int_var->ID].boundary_pointsAngles_max[j]) {
				if (TrfObj[SensData[j].Detc_TO_ID[l]].boundary_pointsAngles_min[j] > TrfObj[int_var->ID].NearestPointDir_Fr0[j] * (-1)
					|| TrfObj[int_var->ID].NearestPointDir_Fr0[j] * (-1) > TrfObj[SensData[j].Detc_TO_ID[l]].boundary_pointsAngles_max[j]) {
					SensData[j].distance[cnt] = int_var->distance[int_var->ID];
					SensData[j].Detc_TO_ID[cnt] = int_var->Detc_TO_ID[int_var->ID];
					SensData[j].Detc_TO_Surface_ID[cnt] = int_var->Detc_TO_Surface_ID[int_var->ID];
					SensData[j].Fr0_position_NP[cnt][0] = int_var->Fr0_position_NP[int_var->ID][0];
					SensData[j].Fr0_position_NP[cnt][1] = int_var->Fr0_position_NP[int_var->ID][1];
					SensData[j].Fr0_position_NP[cnt][2] = int_var->Fr0_position_NP[int_var->ID][2];
					SensData[j].NPDir_Fr0[cnt] = int_var->NPDir_Fr0[int_var->ID];
				}
				else {
					int_var->distance[int_var->ID] = 9999;
					break; // Go out of loop
				}
			}

			if (int_var->distance[int_var->ID] != 9999)
				cnt++;
		}
	}
	
	
	
		# set SensNo 1
	# if {$Qu(SensData.$SensNo.Target.NPPos.x.0) != 0 && $Qu(SensData.$SensNo.Target.NPPos.y.0) != 0} {
		# glDisableLighting
		# gl color {*}$ContiOrange; #choose the color of trajectory
		# gl linewidth 1; #change the width of trajectory
		# gl begin line_strip
		
		# gl vertex $Qu(SensData.$SensNo.SensPos.x) $Qu(SensData.$SensNo.SensPos.y) $Qu(SensData.$SensNo.SensPos.z)
		# gl vertex $Qu(SensData.$SensNo.Target.NPPos.x.0) $Qu(SensData.$SensNo.Target.NPPos.y.0) $Qu(SensData.$SensNo.SensPos.z)

		# gl end
		# glEnableLighting
    # }
	# if {$Qu(SensData.$SensNo.Target.NPPos.x.1) != 0 && $Qu(SensData.$SensNo.Target.NPPos.y.1) != 0} {
		# glDisableLighting
		# gl color {*}$ContiOrange; #choose the color of trajectory
		# gl linewidth 1; #change the width of trajectory
		# gl begin line_strip
		
		# gl vertex $Qu(SensData.$SensNo.SensPos.x) $Qu(SensData.$SensNo.SensPos.y) $Qu(SensData.$SensNo.SensPos.z)
		# gl vertex $Qu(SensData.$SensNo.Target.NPPos.x.1) $Qu(SensData.$SensNo.Target.NPPos.y.1) $Qu(SensData.$SensNo.SensPos.z)

		# gl end
		# glEnableLighting
    # }
	# if {$Qu(SensData.$SensNo.Target.NPPos.x.2) != 0 && $Qu(SensData.$SensNo.Target.NPPos.y.2) != 0} {
		# glDisableLighting
		# gl color {*}$ContiOrange; #choose the color of trajectory
		# gl linewidth 1; #change the width of trajectory
		# gl begin line_strip
		
		# gl vertex $Qu(SensData.$SensNo.SensPos.x) $Qu(SensData.$SensNo.SensPos.y) $Qu(SensData.$SensNo.SensPos.z)
		# gl vertex $Qu(SensData.$SensNo.Target.NPPos.x.2) $Qu(SensData.$SensNo.Target.NPPos.y.2) $Qu(SensData.$SensNo.SensPos.z)

		# gl end
		# glEnableLighting
    # }