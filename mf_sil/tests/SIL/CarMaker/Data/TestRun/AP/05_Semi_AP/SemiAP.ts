#INFOFILE1.1 - Do not remove this line!
FileIdent = CarMaker-TestSeries 2
FileCreator = CarMaker 8.1.2 2020-7-17
Description:
LastChange = 2021-03-30 19:52:26 uia55352
StartTime = 2021-03-30 19:33:20
EndTime = 2021-03-30 19:34:30
ReportTemplate =
Step.0 = Settings
Step.0.Name = Global Settings
Step.1 = TestWare
Step.1.Name = TEST CONFIGURATION
Step.2 = Group
Step.2.Name = Parallel Left In 1 Stroke
Step.2.0 = TestRun
Step.2.0.Name = AP/05_Semi_AP/SemiAP_TestParallel_Left_1xStroke
Step.2.0.Param.0 = LongOffset NValue
Step.2.0.Param.1 = LatOffset NValue
Step.2.0.Param.2 = YawOffset NValue
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
Step.2.0.Var.0.Name = _Standard
Step.2.0.Var.0.Param = 35 4 0
Step.2.0.Var.0.Result = good
Step.2.0.Var.0.ResDate = 1617124099
Step.2.0.Var.0.ManLst = 0 1 2
Step.2.0.Var.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.0.Var.0.Crit.0.Result = good
Step.2.0.Var.0.Crit.1.Name = No Collision
Step.2.0.Var.0.Crit.1.Result = good
Step.2.1 = TestRun
Step.2.1.Name = AP/05_Semi_AP/SemiAP_TestParallel_Left_1xStroke_NoFrontVehicle
Step.2.1.Param.0 = LongOffset NValue
Step.2.1.Param.1 = LatOffset NValue
Step.2.1.Param.2 = YawOffset NValue
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
Step.2.1.Var.0.Name = _Standard
Step.2.1.Var.0.Param = 35 4 {}
Step.2.1.Var.0.Result = good
Step.2.1.Var.0.ResDate = 1617124132
Step.2.1.Var.0.ManLst = 0 1 2
Step.2.1.Var.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.1.Var.0.Crit.0.Result = good
Step.2.1.Var.0.Crit.1.Name = No Collision
Step.2.1.Var.0.Crit.1.Result = good
Step.3 = Group
Step.3.Name = Parallel Right In 1 Stroke
Step.3.0 = TestRun
Step.3.0.Name = AP/05_Semi_AP/SemiAP_TestParallel_Right_1xStroke
Step.3.0.Param.0 = LongOffset NValue
Step.3.0.Param.1 = LatOffset NValue
Step.3.0.Param.2 = YawOffset NValue
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
Step.3.0.Var.0.Name = _Standard
Step.3.0.Var.0.Param = 35 0 {}
Step.3.0.Var.0.Result = good
Step.3.0.Var.0.ResDate = 1617124308
Step.3.0.Var.0.ManLst = 0 1 2
Step.3.0.Var.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.3.0.Var.0.Crit.0.Result = good
Step.3.0.Var.0.Crit.1.Name = No Collision
Step.3.0.Var.0.Crit.1.Result = good
Step.3.1 = TestRun
Step.3.1.Name = AP/05_Semi_AP/SemiAP_TestParallel_Right_1xStroke_NoFrontVehicle
Step.3.1.Param.0 = LongOffset NValue
Step.3.1.Param.1 = LatOffset NValue
Step.3.1.Param.2 = YawOffset NValue
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
Step.3.1.Var.0.Name = _Standard
Step.3.1.Var.0.Param = 35 0 {}
Step.3.1.Var.0.Result = good
Step.3.1.Var.0.ResDate = 1617124334
Step.3.1.Var.0.ManLst = 0 1 2
Step.3.1.Var.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.3.1.Var.0.Crit.0.Result = good
Step.3.1.Var.0.Crit.1.Name = No Collision
Step.3.1.Var.0.Crit.1.Result = good
Step.4 = Group
Step.4.Name = Perpendicular Left In 1 Stroke
Step.4.0 = TestRun
Step.4.0.Name = AP/05_Semi_AP/SemiAP_TestPerp_Left_1xStroke
Step.4.0.Param.0 = LongOffset NValue
Step.4.0.Param.1 = LatOffset NValue
Step.4.0.Param.2 = YawOffset NValue
Step.4.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.4.0.Crit.0.Description:
Step.4.0.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.4.0.Crit.0.Warn =
Step.4.0.Crit.0.Bad =
Step.4.0.Crit.1.Name = No Collision
Step.4.0.Crit.1.Description:
Step.4.0.Crit.1.Good =
Step.4.0.Crit.1.Warn =
Step.4.0.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.4.0.Var.0.Name = _Standard_fwd
Step.4.0.Var.0.Param = 36 3 0
Step.4.0.Var.0.Result = good
Step.4.0.Var.0.ResDate = 1617124739
Step.4.0.Var.0.ManLst = 0 1 2
Step.4.0.Var.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.4.0.Var.0.Crit.0.Result = good
Step.4.0.Var.0.Crit.1.Name = No Collision
Step.4.0.Var.0.Crit.1.Result = good
Step.4.0.Var.1.Name = _Standard_bwd
Step.4.0.Var.1.Param = 33 2.5 0
Step.4.0.Var.1.Result = good
Step.4.0.Var.1.ResDate = 1617124934
Step.4.0.Var.1.ManLst = 0 1 2
Step.4.0.Var.1.Crit.0.Name = Reached Maneuver Finish Screen
Step.4.0.Var.1.Crit.0.Result = good
Step.4.0.Var.1.Crit.1.Name = No Collision
Step.4.0.Var.1.Crit.1.Result = good
Step.4.1 = TestRun
Step.4.1.Name = AP/05_Semi_AP/SemiAP_TestPerp_Left_1xStroke_NoFrontVehicle
Step.4.1.Param.0 = LongOffset NValue
Step.4.1.Param.1 = LatOffset NValue
Step.4.1.Param.2 = YawOffset NValue
Step.4.1.Crit.0.Name = Reached Maneuver Finish Screen
Step.4.1.Crit.0.Description:
Step.4.1.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.4.1.Crit.0.Warn =
Step.4.1.Crit.0.Bad =
Step.4.1.Crit.1.Name = No Collision
Step.4.1.Crit.1.Description:
Step.4.1.Crit.1.Good =
Step.4.1.Crit.1.Warn =
Step.4.1.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.4.1.Var.0.Name = _Standard_fwd
Step.4.1.Var.0.Param = 36 3 0
Step.4.1.Var.0.Result = good
Step.4.1.Var.0.ResDate = 1617125670
Step.4.1.Var.0.ManLst = 0 1 2
Step.4.1.Var.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.4.1.Var.0.Crit.0.Result = good
Step.4.1.Var.0.Crit.1.Name = No Collision
Step.4.1.Var.0.Crit.1.Result = good
Step.4.1.Var.1.Name = _Standard_bwd
Step.4.1.Var.1.Param = 32 4 0
Step.5 = Group
Step.5.Name = Perpendicular Right In 1 Stroke
Step.5.0 = TestRun
Step.5.0.Name = AP/05_Semi_AP/SemiAP_TestPerp_Right_1xStroke
Step.5.0.Param.0 = LongOffset NValue
Step.5.0.Param.1 = LatOffset NValue
Step.5.0.Param.2 = YawOffset NValue
Step.5.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.5.0.Crit.0.Description:
Step.5.0.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.5.0.Crit.0.Warn =
Step.5.0.Crit.0.Bad =
Step.5.0.Crit.1.Name = No Collision
Step.5.0.Crit.1.Description:
Step.5.0.Crit.1.Good =
Step.5.0.Crit.1.Warn =
Step.5.0.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.5.0.Var.0.Name = _Standard
Step.5.0.Var.0.Param = 41 0 0
Step.5.0.Var.0.Result = good
Step.5.0.Var.0.ResDate = 1589529945
Step.5.0.Var.0.ResFiles = SimOutput/FRL9E10W/20200515/AP_00_AP_Demo_Parallel_Right_In_Scan_Off_100501.erg
Step.5.0.Var.0.ManLst = 0 1 2
Step.5.0.Var.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.5.0.Var.0.Crit.0.Result = good
Step.5.0.Var.0.Crit.1.Name = No Collision
Step.5.0.Var.0.Crit.1.Result = good
Step.5.1 = TestRun
Step.5.1.Name = AP/05_Semi_AP/SemiAP_TestPerp_Right_1xStroke_NoFrontVehicle
Step.5.1.Param.0 = LongOffset NValue
Step.5.1.Param.1 = LatOffset NValue
Step.5.1.Param.2 = YawOffset NValue
Step.5.1.Crit.0.Name = Reached Maneuver Finish Screen
Step.5.1.Crit.0.Description:
Step.5.1.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.5.1.Crit.0.Warn =
Step.5.1.Crit.0.Bad =
Step.5.1.Crit.1.Name = No Collision
Step.5.1.Crit.1.Description:
Step.5.1.Crit.1.Good =
Step.5.1.Crit.1.Warn =
Step.5.1.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.5.1.Var.0.Name = _Standard
Step.5.1.Var.0.Param = {} {} {}
