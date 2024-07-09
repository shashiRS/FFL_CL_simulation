#INFOFILE1.1 - Do not remove this line!
FileIdent = CarMaker-TestSeries 2
FileCreator = CarMaker 8.1.1 2019-12-19
Description:
LastChange = 2023-01-17 07:57:02 uidn7467
StartTime = 2023-01-17 07:55:58
EndTime = 2023-01-17 07:56:53
ReportTemplate =
Step.0 = Settings
Step.0.Name = Global Settings
Step.1 = Group
Step.1.Name = Base Parking
Step.1.0 = TestRun
Step.1.0.Name = AP/06_Regression/SmallRegression_Missing/AUPSim_UC_ParRight_ST-1_09_03_B_SI_On_Remote
Step.1.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.1.0.Crit.0.Description:
Step.1.0.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.1.0.Crit.0.Warn =
Step.1.0.Crit.0.Bad =
Step.1.0.Crit.1.Name = No Collision
Step.1.0.Crit.1.Description:
Step.1.0.Crit.1.Good =
Step.1.0.Crit.1.Warn =
Step.1.0.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.1.0.Result = good
Step.1.0.ResDate = 1657708752
Step.1.0.ManLst = 0 1 2 3 4 5
Step.1.0.Crit.0.Result = good
Step.1.0.Crit.1.Result = good
Step.1.1 = TestRun
Step.1.1.Name = AP/06_Regression/SmallRegression_Missing/Parallel_Right_InOut_Scan_On
Step.1.1.Crit.0.Name = Reached Maneuver Finish Screen
Step.1.1.Crit.0.Description:
Step.1.1.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.1.1.Crit.0.Warn =
Step.1.1.Crit.0.Bad =
Step.1.1.Crit.1.Name = No Collision
Step.1.1.Crit.1.Description:
Step.1.1.Crit.1.Good =
Step.1.1.Crit.1.Warn =
Step.1.1.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.1.1.Result = good
Step.1.1.ResDate = 1657708787
Step.1.1.ManLst = 0 1 2 3 4 5 6 7 8
Step.1.1.Crit.0.Result = good
Step.1.1.Crit.1.Result = good
Step.1.2 = TestRun
Step.1.2.Name = AP/06_Regression/SmallRegression_Missing/Perpendicular_Right_InOut_Scan_On
Step.1.2.Crit.0.Name = Reached Maneuver Finish Screen
Step.1.2.Crit.0.Description:
Step.1.2.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.1.2.Crit.0.Warn =
Step.1.2.Crit.0.Bad =
Step.1.2.Crit.1.Name = No Collision
Step.1.2.Crit.1.Description:
Step.1.2.Crit.1.Good =
Step.1.2.Crit.1.Warn =
Step.1.2.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.1.2.Result = good
Step.1.2.ResDate = 1657708841
Step.1.2.ManLst = 0 1 2 3 4 5 6 7 8
Step.1.2.Crit.0.Result = good
Step.1.2.Crit.1.Result = good
Step.1.3 = TestRun
Step.1.3.Name = AP/06_Regression/SmallRegression_Missing/Angled_Right_InOut_Backward_Scan_On
Step.1.3.Crit.0.Name = Reached Maneuver Finish Screen
Step.1.3.Crit.0.Description:
Step.1.3.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.1.3.Crit.0.Warn =
Step.1.3.Crit.0.Bad =
Step.1.3.Crit.1.Name = No Collision
Step.1.3.Crit.1.Description:
Step.1.3.Crit.1.Good =
Step.1.3.Crit.1.Warn =
Step.1.3.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.1.3.Result = good
Step.1.3.ResDate = 1673938613
Step.1.3.ManLst = 0 1 2 3 4 5 6 7 8
Step.1.3.Crit.0.Result = good
Step.1.3.Crit.1.Result = good
Step.2 = Group
Step.2.Name = Entry parking (Reverse Assist Optional, vCUS_UDP Optional)
Step.2.0 = TestRun
Step.2.0.Name = ReverseAssist/01_RA_SCurve_20m_NoDynObj
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
Step.2.0.Result = good
Step.2.0.ResDate = 1657709045
Step.2.0.ManLst = 0 1 2 3 4 5 6 7
Step.2.0.Crit.0.Result = good
Step.2.0.Crit.1.Result = good
Step.2.1 = TestRun
Step.2.1.Name = AP/06_Regression/SmallRegression_Missing/Parallel_Right_InOut_Scan_On
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
Step.2.1.Result = good
Step.2.1.ResDate = 1657709060
Step.2.1.ManLst = 0 1 2 3 4 5 6 7 8
Step.2.1.Crit.0.Result = good
Step.2.1.Crit.1.Result = good
Step.2.2 = TestRun
Step.2.2.Name = AP/06_Regression/SmallRegression_Missing/Perpendicular_Right_InOut_Scan_On
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
Step.2.2.Result = good
Step.2.2.ResDate = 1657709070
Step.2.2.ManLst = 0 1 2 3 4 5 6 7 8
Step.2.2.Crit.0.Result = good
Step.2.2.Crit.1.Result = good
Step.2.3 = TestRun
Step.2.3.Name = AP/00_AP_Demo/vCUS_Model/Parallel_Right_In_Scan_On_vCUS_UDP
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
Step.2.3.Result = good
Step.2.3.ResDate = 1670500646
Step.2.3.ManLst = 0 1 2 3 4 5
Step.2.3.Crit.0.Result = good
Step.2.3.Crit.1.Result = good
Step.3 = Group
Step.3.Name = Performance Parking
Step.3.0 = TestRun
Step.3.0.Name = AP/06_Regression/SmallRegression_Missing/AUPSim_UC_ParRight_ST-1_09_03_B_SI_On_Remote
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
Step.3.0.Result = good
Step.3.0.ResDate = 1657709211
Step.3.0.ManLst = 0 1 2 3 4 5
Step.3.0.Crit.0.Result = good
Step.3.0.Crit.1.Result = good
Step.3.1 = TestRun
Step.3.1.Name = AP/06_Regression/SmallRegression_Missing/Parallel_Right_InOut_Scan_On
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
Step.3.1.Result = good
Step.3.1.ResDate = 1657709223
Step.3.1.ManLst = 0 1 2 3 4 5 6 7 8
Step.3.1.Crit.0.Result = good
Step.3.1.Crit.1.Result = good
Step.3.2 = TestRun
Step.3.2.Name = AP/06_Regression/SmallRegression_Missing/Perpendicular_Right_InOut_Scan_On
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
Step.3.2.Result = good
Step.3.2.ResDate = 1657709233
Step.3.2.ManLst = 0 1 2 3 4 5 6 7 8
Step.3.2.Crit.0.Result = good
Step.3.2.Crit.1.Result = good
Step.3.3 = TestRun
Step.3.3.Name = AP/06_Regression/SmallRegression_Missing/Angled_Right_InOut_Backward_Scan_On
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
Step.3.3.Result = good
Step.3.3.ResDate = 1673938511
Step.3.3.ManLst = 0 1 2 3 4 5 6 7 8
Step.3.3.Crit.0.Result = good
Step.3.3.Crit.1.Result = good
Step.3.4 = TestRun
Step.3.4.Name = AP/06_Regression/ParkOutIgnitionCycle/AUPSim_UC_PerpRight_ST-0006_B_InOut_IgnitionCycle
Step.3.4.Crit.0.Name = Reached Maneuver Finish Screen
Step.3.4.Crit.0.Description:
Step.3.4.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.3.4.Crit.0.Warn =
Step.3.4.Crit.0.Bad =
Step.3.4.Crit.1.Name = No Collision
Step.3.4.Crit.1.Description:
Step.3.4.Crit.1.Good =
Step.3.4.Crit.1.Warn =
Step.3.4.Crit.1.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.3.4.Result = good
Step.3.4.ResDate = 1657709211
Step.3.4.ManLst = 0 1 2 3 4 5
Step.3.4.Crit.0.Result = good
Step.3.4.Crit.1.Result = good
Step.4 = Group
Step.4.Name = Optional: Base parking MoCo
Step.4.0 = TestRun
Step.4.0.Name = AP/00_AP_Demo/Motion_Control/Perpendicular_Right_In_Scan_On
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
Step.4.0.Result = good
Step.4.0.ResDate = 1650357720
Step.4.0.ManLst = 0 1 2 3 4 5
Step.4.0.Crit.0.Result = good
Step.4.0.Crit.1.Result = good
