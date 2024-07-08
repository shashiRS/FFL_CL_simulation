#INFOFILE1.1 - Do not remove this line!
FileIdent = CarMaker-TestSeries 2
FileCreator = CarMaker 8.1.1 2019-12-19
Description:
LastChange = 2021-07-28 15:22:02 uidq5915
StartTime = 2021-07-28 15:15:18
EndTime = 2021-07-28 15:15:54
ReportTemplate =
Step.0 = Settings
Step.0.Name = Global Settings
Step.1 = TestWare
Step.1.Name = TEST CONFIGURATION
Step.2 = Group
Step.2.Name = Parallel Right In
Step.2.0 = TestRun
Step.2.0.Name = AP/00_AP_Demo/CemSurrogate_Dynamic_Objects/Parallel_Right_In_SI_On_Pedestrian_From_Top
Step.2.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.0.Crit.0.Description:
Step.2.0.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.2.0.Crit.0.Warn =
Step.2.0.Crit.0.Bad =
Step.2.0.Crit.1.Name = No Collision
Step.2.0.Crit.1.Description:
Step.2.0.Crit.1.Good =
Step.2.0.Crit.1.Warn =
Step.2.0.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.2.0.Result = bad
Step.2.0.ResDate = 1627476838
Step.2.0.ManLst = 0 1 2 3 4 5
Step.2.0.Crit.0.Result = bad
Step.2.0.Crit.1.Result = good
Step.2.1 = TestRun
Step.2.1.Name = AP/00_AP_Demo/CemSurrogate_Dynamic_Objects/Parallel_Right_In_SI_On_Pedestrian_RoadCenter
Step.2.1.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.1.Crit.0.Description:
Step.2.1.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.2.1.Crit.0.Warn =
Step.2.1.Crit.0.Bad =
Step.2.1.Crit.1.Name = No Collision
Step.2.1.Crit.1.Description:
Step.2.1.Crit.1.Good =
Step.2.1.Crit.1.Warn =
Step.2.1.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.2.1.Result = bad
Step.2.1.ResDate = 1627476977
Step.2.1.ManLst = 0 1 2 3 4 5
Step.2.1.Crit.0.Result = bad
Step.2.1.Crit.1.Result = good
Step.2.2 = TestRun
Step.2.2.Name = AP/00_AP_Demo/CemSurrogate_Dynamic_Objects/Parallel_Right_In_SI_On_Pedestrian_RoadSide
Step.2.2.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.2.Crit.0.Description:
Step.2.2.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.2.2.Crit.0.Warn =
Step.2.2.Crit.0.Bad =
Step.2.2.Crit.1.Name = No Collision
Step.2.2.Crit.1.Description:
Step.2.2.Crit.1.Good =
Step.2.2.Crit.1.Warn =
Step.2.2.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.2.2.Result = bad
Step.2.2.ResDate = 1627477014
Step.2.2.ManLst = 0 1 2 3 4 5
Step.2.2.Crit.0.Result = bad
Step.2.2.Crit.1.Result = good
Step.2.3 = TestRun
Step.2.3.Name = AP/00_AP_Demo/CemSurrogate_Dynamic_Objects/Parallel_Right_In_SI_On_Vehicle_Behind
Step.2.3.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.3.Crit.0.Description:
Step.2.3.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.2.3.Crit.0.Warn =
Step.2.3.Crit.0.Bad =
Step.2.3.Crit.1.Name = No Collision
Step.2.3.Crit.1.Description:
Step.2.3.Crit.1.Good =
Step.2.3.Crit.1.Warn =
Step.2.3.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.2.3.Result = bad
Step.2.3.ResDate = 1627477293
Step.2.3.ManLst = 0 1 2 3 4 5
Step.2.3.Crit.0.Result = bad
Step.2.3.Crit.1.Result = good
Step.2.4 = TestRun
Step.2.4.Name = AP/00_AP_Demo/CemSurrogate_Dynamic_Objects/Parallel_Right_In_SI_On_Vehicle_Passing
Step.2.4.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.4.Crit.0.Description:
Step.2.4.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.2.4.Crit.0.Warn =
Step.2.4.Crit.0.Bad =
Step.2.4.Crit.1.Name = No Collision
Step.2.4.Crit.1.Description:
Step.2.4.Crit.1.Good =
Step.2.4.Crit.1.Warn =
Step.2.4.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.2.4.Result = good
Step.2.4.ResDate = 1627477499
Step.2.4.ManLst = 0 1 2 3 4 5
Step.2.4.Crit.0.Result = good
Step.2.4.Crit.1.Result = good
Step.2.5 = Group
Step.2.5.Name = Object appears based on time
Step.2.5.0 = TestRun
Step.2.5.0.Name = AP/00_AP_Demo/CemSurrogate_Dynamic_Objects/Parallel_Right_In_SI_On_Object_Appears_Time
Step.2.5.0.Param.0 = ObjPosX NValue
Step.2.5.0.Param.1 = ObjPosY NValue
Step.2.5.0.Param.2 = ObjHeight NValue
Step.2.5.0.Param.3 = ObjAppearTime NValue
Step.2.5.0.Param.4 = ObjAppearDuration NValue
Step.2.5.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.5.0.Crit.0.Description:
Step.2.5.0.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.2.5.0.Crit.0.Warn =
Step.2.5.0.Crit.0.Bad =
Step.2.5.0.Crit.1.Name = No Collision
Step.2.5.0.Crit.1.Description:
Step.2.5.0.Crit.1.Good =
Step.2.5.0.Crit.1.Warn =
Step.2.5.0.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.2.5.0.Var.0.Name = _Standard
Step.2.5.0.Var.0.Param = 35.0 -3.0 3 20.0 10.0
Step.2.5.0.Var.0.Result = bad
Step.2.5.0.Var.0.ResDate = 1627476487
Step.2.5.0.Var.0.ManLst = 0 1 2 3 4 5
Step.2.5.0.Var.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.5.0.Var.0.Crit.0.Result = bad
Step.2.5.0.Var.0.Crit.1.Name = No Collision
Step.2.5.0.Var.0.Crit.1.Result = good
Step.2.6 = Group
Step.2.6.Name = Object appears based on number of strokes
Step.2.6.0 = TestRun
Step.2.6.0.Name = AP/00_AP_Demo/CemSurrogate_Dynamic_Objects/Parallel_Right_In_SI_On_Object_Appears_NumStroke
Step.2.6.0.Param.0 = ObjPosX NValue
Step.2.6.0.Param.1 = ObjPosY NValue
Step.2.6.0.Param.2 = ObjHeight NValue
Step.2.6.0.Param.3 = ObjAppearStroke NValue
Step.2.6.0.Param.4 = ObjAppearDistance NValue
Step.2.6.0.Param.5 = ObjAppearDuration NValue
Step.2.6.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.6.0.Crit.0.Description:
Step.2.6.0.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.2.6.0.Crit.0.Warn =
Step.2.6.0.Crit.0.Bad =
Step.2.6.0.Crit.1.Name = No Collision
Step.2.6.0.Crit.1.Description:
Step.2.6.0.Crit.1.Good =
Step.2.6.0.Crit.1.Warn =
Step.2.6.0.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.2.6.0.Var.0.Name = _Standard
Step.2.6.0.Var.0.Param = 35.0 -3.0 3 1 1.5 10.0
Step.2.6.0.Var.0.Result = bad
Step.2.6.0.Var.0.ResDate = 1627476258
Step.2.6.0.Var.0.ManLst = 0 1 2 3 4 5
Step.2.6.0.Var.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.6.0.Var.0.Crit.0.Result = bad
Step.2.6.0.Var.0.Crit.1.Name = No Collision
Step.2.6.0.Var.0.Crit.1.Result = good
Step.3 = Group
Step.3.Name = Prependicular Right In
Step.3.0 = TestRun
Step.3.0.Name = AP/00_AP_Demo/CemSurrogate_Dynamic_Objects/Perpendicular_Right_In_SI_On_Pedestrian_RoadCenter
Step.3.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.3.0.Crit.0.Description:
Step.3.0.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.3.0.Crit.0.Warn =
Step.3.0.Crit.0.Bad =
Step.3.0.Crit.1.Name = No Collision
Step.3.0.Crit.1.Description:
Step.3.0.Crit.1.Good =
Step.3.0.Crit.1.Warn =
Step.3.0.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.3.0.Result = bad
Step.3.0.ResDate = 1627477540
Step.3.0.ManLst = 0 1 2 3 4 5
Step.3.0.Crit.0.Result = bad
Step.3.0.Crit.1.Result = good
Step.3.1 = TestRun
Step.3.1.Name = AP/00_AP_Demo/CemSurrogate_Dynamic_Objects/Perpendicular_Right_In_SI_On_Pedestrian_RoadSide
Step.3.1.Crit.0.Name = Reached Maneuver Finish Screen
Step.3.1.Crit.0.Description:
Step.3.1.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.3.1.Crit.0.Warn =
Step.3.1.Crit.0.Bad =
Step.3.1.Crit.1.Name = No Collision
Step.3.1.Crit.1.Description:
Step.3.1.Crit.1.Good =
Step.3.1.Crit.1.Warn =
Step.3.1.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.3.1.Result = bad
Step.3.1.ResDate = 1627477917
Step.3.1.ManLst = 0 1 2 3 4 5
Step.3.1.Crit.0.Result = bad
Step.3.1.Crit.1.Result = good
Step.3.2 = TestRun
Step.3.2.Name = AP/00_AP_Demo/CemSurrogate_Dynamic_Objects/Perpendicular_Right_In_SI_On_Vehicle_Behind
Step.3.2.Crit.0.Name = Reached Maneuver Finish Screen
Step.3.2.Crit.0.Description:
Step.3.2.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.3.2.Crit.0.Warn =
Step.3.2.Crit.0.Bad =
Step.3.2.Crit.1.Name = No Collision
Step.3.2.Crit.1.Description:
Step.3.2.Crit.1.Good =
Step.3.2.Crit.1.Warn =
Step.3.2.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.3.2.Result = bad
Step.3.2.ResDate = 1627477949
Step.3.2.ManLst = 0 1 2 3 4 5
Step.3.2.Crit.0.Result = bad
Step.3.2.Crit.1.Result = good
Step.3.3 = TestRun
Step.3.3.Name = AP/00_AP_Demo/CemSurrogate_Dynamic_Objects/Perpendicular_Right_In_SI_On_Vehicle_Passing
Step.3.3.Crit.0.Name = Reached Maneuver Finish Screen
Step.3.3.Crit.0.Description:
Step.3.3.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.3.3.Crit.0.Warn =
Step.3.3.Crit.0.Bad =
Step.3.3.Crit.1.Name = No Collision
Step.3.3.Crit.1.Description:
Step.3.3.Crit.1.Good =
Step.3.3.Crit.1.Warn =
Step.3.3.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.3.3.Result = bad
Step.3.3.ResDate = 1627478154
Step.3.3.ManLst = 0 1 2 3 4 5
Step.3.3.Crit.0.Result = bad
Step.3.3.Crit.1.Result = good
Step.3.4 = Group
Step.3.4.Name = Object appears based on time
Step.3.4.0 = TestRun
Step.3.4.0.Name = AP/00_AP_Demo/CemSurrogate_Dynamic_Objects/Perpendicular_Right_In_SI_On_Object_Appears_Time
Step.3.4.0.Param.0 = ObjPosX NValue
Step.3.4.0.Param.1 = ObjPosY NValue
Step.3.4.0.Param.2 = ObjHeight NValue
Step.3.4.0.Param.3 = ObjAppearTime NValue
Step.3.4.0.Param.4 = ObjAppearDuration NValue
Step.3.4.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.3.4.0.Crit.0.Description:
Step.3.4.0.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.3.4.0.Crit.0.Warn =
Step.3.4.0.Crit.0.Bad =
Step.3.4.0.Crit.1.Name = No Collision
Step.3.4.0.Crit.1.Description:
Step.3.4.0.Crit.1.Good =
Step.3.4.0.Crit.1.Warn =
Step.3.4.0.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.3.4.0.Var.0.Name = _Standard
Step.3.4.0.Var.0.Param = 31.0 -0.5 3 25.0 10.0
Step.3.4.0.Var.0.Result = err
Step.3.4.0.Var.0.ResDate = 1627476582
Step.3.4.0.Var.0.ManLst = 0 1 2
Step.3.4.0.Var.0.Log.0.Time = 11.184
Step.3.4.0.Var.0.Log.0.Kind = err
Step.3.4.0.Var.0.Log.0.Text = Simulation stopped by user
Step.3.5 = Group
Step.3.5.Name = Object appears based on number of strokes
Step.3.5.0 = TestRun
Step.3.5.0.Name = AP/00_AP_Demo/CemSurrogate_Dynamic_Objects/Perpendicular_Right_In_SI_On_Object_Appears_NumStroke
Step.3.5.0.Param.0 = ObjPosX NValue
Step.3.5.0.Param.1 = ObjPosY NValue
Step.3.5.0.Param.2 = ObjHeight NValue
Step.3.5.0.Param.3 = ObjAppearStroke NValue
Step.3.5.0.Param.4 = ObjAppearDistance NValue
Step.3.5.0.Param.5 = ObjAppearDuration NValue
Step.3.5.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.3.5.0.Crit.0.Description:
Step.3.5.0.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.3.5.0.Crit.0.Warn =
Step.3.5.0.Crit.0.Bad =
Step.3.5.0.Crit.1.Name = No Collision
Step.3.5.0.Crit.1.Description:
Step.3.5.0.Crit.1.Good =
Step.3.5.0.Crit.1.Warn =
Step.3.5.0.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.3.5.0.Var.0.Name = _Standard
Step.3.5.0.Var.0.Param = 31.0 -0.5 3 2 0.1 10.0
Step.3.5.0.Var.0.Result = bad
Step.3.5.0.Var.0.ResDate = 1627476149
Step.3.5.0.Var.0.ManLst = 0 1 2 3 4 5
Step.3.5.0.Var.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.3.5.0.Var.0.Crit.0.Result = bad
Step.3.5.0.Var.0.Crit.1.Name = No Collision
Step.3.5.0.Var.0.Crit.1.Result = good
