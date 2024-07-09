#INFOFILE1.1 - Do not remove this line!
FileIdent = CarMaker-TestSeries 2
FileCreator = CarMaker 8.1.1 2019-12-19
Description:
LastChange = 2022-08-08 14:59:49 uidq5915
StartTime = 2022-08-08 14:55:16
EndTime = 2022-08-08 14:59:14
ReportTemplate =
Step.0 = Settings
Step.0.Name = Global Settings
Step.1 = Group
Step.1.Name = SmallRegression_Base
Step.1.0 = TestRun
Step.1.0.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_UC_ParLeft_ST-1_04_02_B_MoCo_SI_On_WithoutCurb
Step.1.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.1.0.Crit.0.Description:
Step.1.0.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.1.0.Crit.0.Warn =
Step.1.0.Crit.0.Bad =
Step.1.0.Crit.1.Name = No Overshoot
Step.1.0.Crit.1.Description:
Step.1.0.Crit.1.Good =
Step.1.0.Crit.1.Warn = [get AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m] < -0.05
Step.1.0.Crit.1.Bad =
Step.1.0.Crit.2.Name = No Collision
Step.1.0.Crit.2.Description:
Step.1.0.Crit.2.Good =
Step.1.0.Crit.2.Warn =
Step.1.0.Crit.2.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.1.0.Result = good
Step.1.0.ResDate = 1659962841
Step.1.0.ManLst = 0 1 2 3 4 5
Step.1.0.Crit.0.Result = good
Step.1.1 = TestRun
Step.1.1.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_UC_ParRight_ST-1_04_02_B_MoCo_SI_On
Step.1.1.Crit.0.Name = Reached Maneuver Finish Screen
Step.1.1.Crit.0.Description:
Step.1.1.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.1.1.Crit.0.Warn =
Step.1.1.Crit.0.Bad =
Step.1.1.Crit.1.Name = No Overshoot
Step.1.1.Crit.1.Description:
Step.1.1.Crit.1.Good =
Step.1.1.Crit.1.Warn = [get AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m] < -0.05
Step.1.1.Crit.1.Bad =
Step.1.1.Crit.2.Name = No Collision
Step.1.1.Crit.2.Description:
Step.1.1.Crit.2.Good =
Step.1.1.Crit.2.Warn =
Step.1.1.Crit.2.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.1.1.Result = good
Step.1.1.ResDate = 1659962867
Step.1.1.ManLst = 0 1 2 3 4 5
Step.1.1.Crit.0.Result = good
Step.1.2 = TestRun
Step.1.2.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_UC_ParRight_ST-1_06_05_B_MoCo_SI_On
Step.1.2.Crit.0.Name = Reached Maneuver Finish Screen
Step.1.2.Crit.0.Description:
Step.1.2.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.1.2.Crit.0.Warn =
Step.1.2.Crit.0.Bad =
Step.1.2.Crit.1.Name = No Overshoot
Step.1.2.Crit.1.Description:
Step.1.2.Crit.1.Good =
Step.1.2.Crit.1.Warn = [get AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m] < -0.05
Step.1.2.Crit.1.Bad =
Step.1.2.Crit.2.Name = No Collision
Step.1.2.Crit.2.Description:
Step.1.2.Crit.2.Good =
Step.1.2.Crit.2.Warn =
Step.1.2.Crit.2.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.1.2.Result = good
Step.1.2.ResDate = 1659962894
Step.1.2.ManLst = 0 1 2 3 4 5
Step.1.2.Crit.0.Result = good
Step.1.3 = TestRun
Step.1.3.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_UC_ParRight_ST-0539_B_MoCo_SI_AngScanNeg_02_WithCurb
Step.1.3.Crit.0.Name = Reached Maneuver Finish Screen
Step.1.3.Crit.0.Description:
Step.1.3.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.1.3.Crit.0.Warn =
Step.1.3.Crit.0.Bad =
Step.1.3.Crit.1.Name = No Overshoot
Step.1.3.Crit.1.Description:
Step.1.3.Crit.1.Good =
Step.1.3.Crit.1.Warn = [get AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m] < -0.05
Step.1.3.Crit.1.Bad =
Step.1.3.Crit.2.Name = No Collision
Step.1.3.Crit.2.Description:
Step.1.3.Crit.2.Good =
Step.1.3.Crit.2.Warn =
Step.1.3.Crit.2.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.1.3.Result = good
Step.1.3.ResDate = 1659962920
Step.1.3.ManLst = 0 1 2 3 4 5
Step.1.3.Crit.0.Result = good
Step.1.4 = TestRun
Step.1.4.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_UC_ParRight_ST-0539_B_MoCo_SI_AngScanPos_02_WithCurb
Step.1.4.Crit.0.Name = Reached Maneuver Finish Screen
Step.1.4.Crit.0.Description:
Step.1.4.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.1.4.Crit.0.Warn =
Step.1.4.Crit.0.Bad =
Step.1.4.Crit.1.Name = No Overshoot
Step.1.4.Crit.1.Description:
Step.1.4.Crit.1.Good =
Step.1.4.Crit.1.Warn = [get AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m] < -0.05
Step.1.4.Crit.1.Bad =
Step.1.4.Crit.2.Name = No Collision
Step.1.4.Crit.2.Description:
Step.1.4.Crit.2.Good =
Step.1.4.Crit.2.Warn =
Step.1.4.Crit.2.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.1.4.Result = good
Step.1.4.ResDate = 1659962950
Step.1.4.ManLst = 0 1 2 3 4 5
Step.1.4.Crit.0.Result = good
Step.1.5 = TestRun
Step.1.5.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_UC_PerpLeft_ST-2_03_03_B_MoCo_SI_On
Step.1.5.Crit.0.Name = Reached Maneuver Finish Screen
Step.1.5.Crit.0.Description:
Step.1.5.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.1.5.Crit.0.Warn =
Step.1.5.Crit.0.Bad =
Step.1.5.Crit.1.Name = No Overshoot
Step.1.5.Crit.1.Description:
Step.1.5.Crit.1.Good =
Step.1.5.Crit.1.Warn = [get AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m] < -0.05
Step.1.5.Crit.1.Bad =
Step.1.5.Crit.2.Name = No Collision
Step.1.5.Crit.2.Description:
Step.1.5.Crit.2.Good =
Step.1.5.Crit.2.Warn =
Step.1.5.Crit.2.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.1.5.Result = warn
Step.1.5.ResDate = 1659962973
Step.1.5.ManLst = 0 1 2 3 4 5
Step.1.5.Crit.0.Result = good
Step.1.6 = TestRun
Step.1.6.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_UC_PerpRight_ST-2_03_03_B_MoCo_SI_On
Step.1.6.Crit.0.Name = Reached Maneuver Finish Screen
Step.1.6.Crit.0.Description:
Step.1.6.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.1.6.Crit.0.Warn =
Step.1.6.Crit.0.Bad =
Step.1.6.Crit.1.Name = No Overshoot
Step.1.6.Crit.1.Description:
Step.1.6.Crit.1.Good =
Step.1.6.Crit.1.Warn = [get AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m] < -0.05
Step.1.6.Crit.1.Bad =
Step.1.6.Crit.2.Name = No Collision
Step.1.6.Crit.2.Description:
Step.1.6.Crit.2.Good =
Step.1.6.Crit.2.Warn =
Step.1.6.Crit.2.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.1.6.Result = good
Step.1.6.ResDate = 1659962995
Step.1.6.ManLst = 0 1 2 3 4 5
Step.1.6.Crit.0.Result = good
Step.1.7 = TestRun
Step.1.7.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_UC_PerpRight_ST-2_04_01_B_MoCo_SI_On
Step.1.7.Crit.0.Name = Reached Maneuver Finish Screen
Step.1.7.Crit.0.Description:
Step.1.7.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.1.7.Crit.0.Warn =
Step.1.7.Crit.0.Bad =
Step.1.7.Crit.1.Name = No Overshoot
Step.1.7.Crit.1.Description:
Step.1.7.Crit.1.Good =
Step.1.7.Crit.1.Warn = [get AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m] < -0.05
Step.1.7.Crit.1.Bad =
Step.1.7.Crit.2.Name = No Collision
Step.1.7.Crit.2.Description:
Step.1.7.Crit.2.Good =
Step.1.7.Crit.2.Warn =
Step.1.7.Crit.2.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.1.7.Result = good
Step.1.7.ResDate = 1659963025
Step.1.7.ManLst = 0 1 2 3 4 5
Step.1.7.Crit.0.Result = good
Step.1.8 = TestRun
Step.1.8.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_UC_PerpRight_ST-2_04_03_B_MoCo_SI_On
Step.1.8.Crit.0.Name = Reached Maneuver Finish Screen
Step.1.8.Crit.0.Description:
Step.1.8.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.1.8.Crit.0.Warn =
Step.1.8.Crit.0.Bad =
Step.1.8.Crit.1.Name = No Overshoot
Step.1.8.Crit.1.Description:
Step.1.8.Crit.1.Good =
Step.1.8.Crit.1.Warn = [get AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m] < -0.05
Step.1.8.Crit.1.Bad =
Step.1.8.Crit.2.Name = No Collision
Step.1.8.Crit.2.Description:
Step.1.8.Crit.2.Good =
Step.1.8.Crit.2.Warn =
Step.1.8.Crit.2.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.1.8.Result = good
Step.1.8.ResDate = 1659963048
Step.1.8.ManLst = 0 1 2 3 4 5
Step.1.8.Crit.0.Result = good
Step.2 = Group
Step.2.Name = Specific tests (MoCo)
Step.2.0 = Group
Step.2.0.Name = Specific use case
Step.2.0.0 = Group
Step.2.0.0.Name = Driver override
Step.2.0.0.0 = TestRun
Step.2.0.0.0.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_NUC_PerpRight_ST-2_06_02_B_MoCo_SI_DriverBrakeStart
Step.2.0.0.0.Result = good
Step.2.0.0.0.ResDate = 1659963230
Step.2.0.0.0.ManLst = 0 1 2 3 4 5
Step.2.0.0.1 = TestRun
Step.2.0.0.1.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_NUC_PerpRight_ST-2_06_02_B_MoCo_SI_DriverBrakeManeuver
Step.2.0.0.1.Result = err
Step.2.0.0.1.ResDate = 1659963135
Step.2.0.0.1.ManLst = 0 1 2 3 4
Step.2.0.0.1.Log.0.Time = 169.489
Step.2.0.0.1.Log.0.Kind = err
Step.2.0.0.1.Log.0.Text = Simulation stopped by user
Step.2.1 = Group
Step.2.1.Name = Specific maneuver
Step.2.1.0 = Group
Step.2.1.0.Name = Short strokes
Step.2.1.0.0 = TestRun
Step.2.1.0.0.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_UC_ParRight_ST-0526_B_MoCo_SI_On_01
Step.2.1.0.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.1.0.0.Crit.0.Description:
Step.2.1.0.0.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.2.1.0.0.Crit.0.Warn =
Step.2.1.0.0.Crit.0.Bad =
Step.2.1.0.0.Crit.1.Name = No Overshoot
Step.2.1.0.0.Crit.1.Description:
Step.2.1.0.0.Crit.1.Good =
Step.2.1.0.0.Crit.1.Warn = [get AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m] < -0.05
Step.2.1.0.0.Crit.1.Bad =
Step.2.1.0.0.Crit.2.Name = No Collision
Step.2.1.0.0.Crit.2.Description:
Step.2.1.0.0.Crit.2.Good =
Step.2.1.0.0.Crit.2.Warn =
Step.2.1.0.0.Crit.2.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.2.1.0.0.Result = good
Step.2.1.0.0.ResDate = 1659963346
Step.2.1.0.0.ManLst = 0 1 2 3 4 5 6
Step.2.1.0.0.Crit.0.Result = good
Step.2.1.0.1 = TestRun
Step.2.1.0.1.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_UC_ParRight_ST-0526_B_MoCo_SI_On_02
Step.2.1.0.1.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.1.0.1.Crit.0.Description:
Step.2.1.0.1.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.2.1.0.1.Crit.0.Warn =
Step.2.1.0.1.Crit.0.Bad =
Step.2.1.0.1.Crit.1.Name = No Overshoot
Step.2.1.0.1.Crit.1.Description:
Step.2.1.0.1.Crit.1.Good =
Step.2.1.0.1.Crit.1.Warn = [get AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m] < -0.05
Step.2.1.0.1.Crit.1.Bad =
Step.2.1.0.1.Crit.2.Name = No Collision
Step.2.1.0.1.Crit.2.Description:
Step.2.1.0.1.Crit.2.Good =
Step.2.1.0.1.Crit.2.Warn =
Step.2.1.0.1.Crit.2.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.2.1.0.1.Result = good
Step.2.1.0.1.ResDate = 1659963379
Step.2.1.0.1.ManLst = 0 1 2 3 4 5 6
Step.2.1.0.1.Crit.0.Result = good
Step.2.1.0.2 = TestRun
Step.2.1.0.2.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_UC_ParRight_ST-0526_B_MoCo_SI_On_03
Step.2.1.0.2.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.1.0.2.Crit.0.Description:
Step.2.1.0.2.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.2.1.0.2.Crit.0.Warn =
Step.2.1.0.2.Crit.0.Bad =
Step.2.1.0.2.Crit.1.Name = No Overshoot
Step.2.1.0.2.Crit.1.Description:
Step.2.1.0.2.Crit.1.Good =
Step.2.1.0.2.Crit.1.Warn = [get AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m] < -0.05
Step.2.1.0.2.Crit.1.Bad =
Step.2.1.0.2.Crit.2.Name = No Collision
Step.2.1.0.2.Crit.2.Description:
Step.2.1.0.2.Crit.2.Good =
Step.2.1.0.2.Crit.2.Warn =
Step.2.1.0.2.Crit.2.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.2.1.0.2.Result = good
Step.2.1.0.2.ResDate = 1659963407
Step.2.1.0.2.ManLst = 0 1 2 3 4 5 6
Step.2.1.0.2.Crit.0.Result = good
Step.2.1.0.3 = TestRun
Step.2.1.0.3.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_UC_ParRight_ST-0526_B_MoCo_SI_On_04
Step.2.1.0.3.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.1.0.3.Crit.0.Description:
Step.2.1.0.3.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.2.1.0.3.Crit.0.Warn =
Step.2.1.0.3.Crit.0.Bad =
Step.2.1.0.3.Crit.1.Name = No Overshoot
Step.2.1.0.3.Crit.1.Description:
Step.2.1.0.3.Crit.1.Good =
Step.2.1.0.3.Crit.1.Warn = [get AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m] < -0.05
Step.2.1.0.3.Crit.1.Bad =
Step.2.1.0.3.Crit.2.Name = No Collision
Step.2.1.0.3.Crit.2.Description:
Step.2.1.0.3.Crit.2.Good =
Step.2.1.0.3.Crit.2.Warn =
Step.2.1.0.3.Crit.2.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.2.1.0.3.Result = good
Step.2.1.0.3.ResDate = 1659963430
Step.2.1.0.3.ManLst = 0 1 2 3 4 5 6
Step.2.1.0.3.Crit.0.Result = good
Step.2.1.0.4 = TestRun
Step.2.1.0.4.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_UC_ParRight_ST-0526_B_MoCo_SI_On_05
Step.2.1.0.4.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.1.0.4.Crit.0.Description:
Step.2.1.0.4.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.2.1.0.4.Crit.0.Warn =
Step.2.1.0.4.Crit.0.Bad =
Step.2.1.0.4.Crit.1.Name = No Overshoot
Step.2.1.0.4.Crit.1.Description:
Step.2.1.0.4.Crit.1.Good =
Step.2.1.0.4.Crit.1.Warn = [get AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m] < -0.05
Step.2.1.0.4.Crit.1.Bad =
Step.2.1.0.4.Crit.2.Name = No Collision
Step.2.1.0.4.Crit.2.Description:
Step.2.1.0.4.Crit.2.Good =
Step.2.1.0.4.Crit.2.Warn =
Step.2.1.0.4.Crit.2.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.2.1.0.4.Result = good
Step.2.1.0.4.ResDate = 1659963465
Step.2.1.0.4.ManLst = 0 1 2 3 4 5 6
Step.2.1.0.4.Crit.0.Result = good
Step.2.1.0.5 = TestRun
Step.2.1.0.5.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_UC_ParRight_ST-0526_B_MoCo_SI_On_06
Step.2.1.0.5.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.1.0.5.Crit.0.Description:
Step.2.1.0.5.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.2.1.0.5.Crit.0.Warn =
Step.2.1.0.5.Crit.0.Bad =
Step.2.1.0.5.Crit.1.Name = No Overshoot
Step.2.1.0.5.Crit.1.Description:
Step.2.1.0.5.Crit.1.Good =
Step.2.1.0.5.Crit.1.Warn = [get AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m] < -0.05
Step.2.1.0.5.Crit.1.Bad =
Step.2.1.0.5.Crit.2.Name = No Collision
Step.2.1.0.5.Crit.2.Description:
Step.2.1.0.5.Crit.2.Good =
Step.2.1.0.5.Crit.2.Warn =
Step.2.1.0.5.Crit.2.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.2.1.0.5.Result = good
Step.2.1.0.5.ResDate = 1659963492
Step.2.1.0.5.ManLst = 0 1 2 3 4 5 6
Step.2.1.0.5.Crit.0.Result = good
Step.2.1.0.6 = TestRun
Step.2.1.0.6.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_UC_ParRight_ST-0526_B_MoCo_SI_On_07
Step.2.1.0.6.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.1.0.6.Crit.0.Description:
Step.2.1.0.6.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.2.1.0.6.Crit.0.Warn =
Step.2.1.0.6.Crit.0.Bad =
Step.2.1.0.6.Crit.1.Name = No Overshoot
Step.2.1.0.6.Crit.1.Description:
Step.2.1.0.6.Crit.1.Good =
Step.2.1.0.6.Crit.1.Warn = [get AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m] < -0.05
Step.2.1.0.6.Crit.1.Bad =
Step.2.1.0.6.Crit.2.Name = No Collision
Step.2.1.0.6.Crit.2.Description:
Step.2.1.0.6.Crit.2.Good =
Step.2.1.0.6.Crit.2.Warn =
Step.2.1.0.6.Crit.2.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.2.1.0.6.Result = good
Step.2.1.0.6.ResDate = 1659963513
Step.2.1.0.6.ManLst = 0 1 2 3 4 5 6
Step.2.1.0.6.Crit.0.Result = good
Step.2.1.1 = Group
Step.2.1.1.Name = 1 stroke
Step.2.1.1.0 = TestRun
Step.2.1.1.0.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_UC_PerpRight_ST-2_06_02_B_MoCo_SI_On
Step.2.1.1.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.1.1.0.Crit.0.Description:
Step.2.1.1.0.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.2.1.1.0.Crit.0.Warn =
Step.2.1.1.0.Crit.0.Bad =
Step.2.1.1.0.Crit.1.Name = No Overshoot
Step.2.1.1.0.Crit.1.Description:
Step.2.1.1.0.Crit.1.Good =
Step.2.1.1.0.Crit.1.Warn = [get AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m] < -0.05
Step.2.1.1.0.Crit.1.Bad =
Step.2.1.1.0.Crit.2.Name = No Collision
Step.2.1.1.0.Crit.2.Description:
Step.2.1.1.0.Crit.2.Good =
Step.2.1.1.0.Crit.2.Warn =
Step.2.1.1.0.Crit.2.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.2.1.1.0.Result = warn
Step.2.1.1.0.ResDate = 1659963527
Step.2.1.1.0.ManLst = 0 1 2 3 4 5
Step.2.1.1.0.Crit.0.Result = good
Step.2.1.2 = Group
Step.2.1.2.Name = First stroke forward
Step.2.1.2.0 = TestRun
Step.2.1.2.0.Name = AP/06_Regression/SmallRegression_Base_MoCo/AUPSim_UC_PerpRight_ST-0546_B_MoCo_SI_FstStr
Step.2.1.2.0.Crit.0.Name = Reached Maneuver Finish Screen
Step.2.1.2.0.Crit.0.Description:
Step.2.1.2.0.Crit.0.Good = [get AP.headUnitVisualizationPort.screen_nu] == 5
Step.2.1.2.0.Crit.0.Warn =
Step.2.1.2.0.Crit.0.Bad =
Step.2.1.2.0.Crit.1.Name = No Overshoot
Step.2.1.2.0.Crit.1.Description:
Step.2.1.2.0.Crit.1.Good =
Step.2.1.2.0.Crit.1.Warn = [get AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m] < -0.05
Step.2.1.2.0.Crit.1.Bad =
Step.2.1.2.0.Crit.2.Name = No Collision
Step.2.1.2.0.Crit.2.Description:
Step.2.1.2.0.Crit.2.Good =
Step.2.1.2.0.Crit.2.Warn =
Step.2.1.2.0.Crit.2.Bad = [get Sensor.Collision.Vhcl.Fr1.Count] > 0
Step.2.1.2.0.Result = warn
Step.2.1.2.0.ResDate = 1659963554
Step.2.1.2.0.ManLst = 0 1 2 3 4 5
Step.2.1.2.0.Crit.0.Result = good
