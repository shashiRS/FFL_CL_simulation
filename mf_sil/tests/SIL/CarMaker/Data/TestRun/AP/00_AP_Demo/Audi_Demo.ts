#INFOFILE1.1 - Do not remove this line!
FileIdent = CarMaker-TestSeries 2
FileCreator = CarMaker 8.1.1 2019-12-19
Description:
LastChange = 2020-12-08 12:35:07 uib48745
StartTime = 2018-11-14 12:36:29
EndTime = 2018-11-14 12:36:35
ReportTemplate =
Step.0 = Settings
Step.0.Name = Global Settings
Step.1 = TestWare
Step.1.Name = TEST CONFIGURATION
Step.2 = Group
Step.2.Name = SimScenario01: Parallel Parking In → Increase slot size → Parallel park out
Step.2.0 = TestRun
Step.2.0.Name = AP/00_AP_Demo/AP_Demo_TestParallel_Right_InOut_Motion
Step.2.0.Param.0 = MoveTrafficStartTime NValue
Step.2.0.Param.1 = MoveTrafficDuration NValue
Step.2.0.Param.2 = MoveTrafficRearDist NValue
Step.2.0.Param.3 = MoveTrafficFrontDist NValue
Step.2.0.Var.0.Name = Standard
Step.2.0.Var.0.Param = 40 5 -0.5 0.0
Step.2.0.Var.0.Result = err
Step.2.0.Var.0.ResDate = 1542194683
Step.2.0.Var.0.ResFiles = SimOutput/FRL62RHG/20181114/AP_Demo_TestParallel_Right_InOut_Motion_Standard.erg
Step.2.0.Var.0.ManLst = 0
Step.2.0.Var.0.Log.0.Time = 0.701
Step.2.0.Var.0.Log.0.Kind = err
Step.2.0.Var.0.Log.0.Text = Simulation ended with errors
Step.3 = Group
Step.3.Name = SimScenario02: Slot without start and slot without end (parallel)
Step.3.0 = TestRun
Step.3.0.Name = AP/00_AP_Demo/Parallel_Right_In_Scan_Off
Step.3.0.Param.0 = GolfX NValue
Step.3.0.Param.1 = GolfY NValue
Step.3.0.Param.2 = GolfAng NValue
Step.3.0.Param.3 = PassatX NValue
Step.3.0.Param.4 = LongOffset NValue
Step.3.0.Var.0.Name = _No Vehicle in Front
Step.3.0.Var.0.Param = 55.0 4.5 180 26.233 40.904
Step.3.0.Var.0.Result = err
Step.3.0.Var.0.ResDate = 1542194745
Step.3.0.Var.0.ResFiles = {SimOutput/FRL62RHG/20181114/Parallel_Right_In_Scan_Off__No Vehicle in Front.erg}
Step.3.0.Var.0.ManLst = 0 1
Step.3.0.Var.0.Log.0.Time = 2.772
Step.3.0.Var.0.Log.0.Kind = err
Step.3.0.Var.0.Log.0.Text = Simulation stopped by user
Step.3.0.Var.1.Name = _No Vehicle in the Rear
Step.3.0.Var.1.Param = 26 4 90 26.233 40.904
Step.3.0.Var.1.Result = err
Step.3.0.Var.1.ResDate = 1542194847
Step.3.0.Var.1.ResFiles = {SimOutput/FRL62RHG/20181114/Parallel_Right_In_Scan_Off__No Vehicle in the Rear.erg}
Step.3.0.Var.1.ManLst = 0 1
Step.3.0.Var.1.Log.0.Time = 5.197
Step.3.0.Var.1.Log.0.Kind = err
Step.3.0.Var.1.Log.0.Text = Simulation stopped by user
Step.4 = Group
Step.4.Name = SimScenario04: Parallel parking without rear car. During maneuver another vehicle arrives.
Step.4.0 = TestRun
Step.4.0.Name = AP/00_AP_Demo/Parallel_Right_In_Scan_On_Curb_Motion
Step.4.0.Param.0 = PassatY NValue
Step.4.0.Param.1 = GolfY NValue
Step.4.0.Param.2 = PBoxY NValue
Step.4.0.Param.3 = PBoxWidth NValue
Step.4.0.Param.4 = CurbY NValue
Step.4.0.Param.5 = MoveTrafficRearDist NValue
Step.4.0.Var.0.Name = Standard
Step.4.0.Var.0.Param = -4.4 -4.4 -5.6 2.1 -6.0 8.6
Step.4.0.Var.0.Result = err
Step.4.0.Var.0.ResDate = 1542194869
Step.4.0.Var.0.ResFiles = SimOutput/FRL62RHG/20181114/Parallel_Right_In_Scan_On_Curb_Motion_Standard.erg
Step.4.0.Var.0.ManLst = 0 1 2 3 4
Step.4.0.Var.0.Log.0.Time = 18.095
Step.4.0.Var.0.Log.0.Kind = err
Step.4.0.Var.0.Log.0.Text = Simulation stopped by user
Step.4.0.Var.1.Name = Closer Rear Vehicle
Step.4.0.Var.1.Param = -4.4 -4.4 -5.6 2.1 -6.0 9.1
Step.4.0.Var.1.Result = good
Step.4.0.Var.1.ResDate = 1537445630
Step.4.0.Var.1.ResFiles = {SimOutput/FRL62RHG/20180920/Parallel_Right_In_Scan_On_Curb_Motion_Closer Rear Vehicle.erg}
Step.4.0.Var.1.ManLst = 0 1 2 3 4 5
Step.5 = Group
Step.5.Name = SimScenario05: Align to Curbstone (other vehicles misaligned)
Step.5.0 = TestRun
Step.5.0.Name = AP/00_AP_Demo/Parallel_Right_In_Scan_On_Curb
Step.5.0.Param.0 = LongOffset NValue
Step.5.0.Param.1 = GolfAng NValue
Step.5.0.Param.2 = PassatAng NValue
Step.5.0.Param.3 = PBoxAng NValue
Step.5.0.Param.4 = PBoxY NValue
Step.5.0.Param.5 = PBoxWidth NValue
Step.5.0.Param.6 = GolfX NValue
Step.5.0.Param.7 = CurbY NValue
Step.5.0.Var.0.Name = Misaligned vehicles 1
Step.5.0.Var.0.Param = 13 3 6 -4 -5.7 2.4 37.0 -5.95
Step.5.0.Var.0.Result = err
Step.5.0.Var.0.ResDate = 1542194925
Step.5.0.Var.0.ResFiles = {SimOutput/FRL62RHG/20181114/Parallel_Right_In_Scan_On_Curb_Misaligned vehicles 1.erg}
Step.5.0.Var.0.ManLst = 0 1 2 3
Step.5.0.Var.0.Log.0.Time = 15.480
Step.5.0.Var.0.Log.0.Kind = err
Step.5.0.Var.0.Log.0.Text = Simulation ended with errors
Step.5.0.Var.1.Name = Misaligned vehicles 1 - Curb closer
Step.5.0.Var.1.Param = 13 3 6 -4 -5.7 2.4 37.0 -5.85
Step.5.0.Var.1.Result = err
Step.5.0.Var.1.ResDate = 1542194933
Step.5.0.Var.1.ResFiles = {SimOutput/FRL62RHG/20181114/Parallel_Right_In_Scan_On_Curb_Misaligned vehicles 1 - Curb closer.erg}
Step.5.0.Var.1.ManLst = 0 1
Step.5.0.Var.1.Log.0.Time = 6.868
Step.5.0.Var.1.Log.0.Kind = err
Step.5.0.Var.1.Log.0.Text = Simulation stopped by user
Step.6 = Group
Step.6.Name = SimScenario06: Curved parking incl. scanning
Step.6.0 = TestRun
Step.6.0.Name = AP/00_AP_Demo/Parallel_Right_In_Scan_On_Curved
Step.6.0.Param.0 = ScanningDist NValue
Step.6.0.Param.1 = OdometryActive NValue
Step.6.0.Var.0.Name = Short Scanning Distance
Step.6.0.Var.0.Param = 19 0
Step.6.0.Var.0.Result = err
Step.6.0.Var.0.ResDate = 1542194949
Step.6.0.Var.0.ResFiles = {SimOutput/FRL62RHG/20181114/Parallel_Right_In_Scan_On_Curved_Short Scanning Distance.erg}
Step.6.0.Var.0.ManLst = 0 1 2 3
Step.6.0.Var.0.Log.0.Time = 11.774
Step.6.0.Var.0.Log.0.Kind = err
Step.6.0.Var.0.Log.0.Text = Simulation ended with errors
Step.6.0.Var.1.Name = Long Scanning Distance
Step.6.0.Var.1.Param = 25 0
Step.6.0.Var.1.Result = err
Step.6.0.Var.1.ResDate = 1542194966
Step.6.0.Var.1.ResFiles = {SimOutput/FRL62RHG/20181114/Parallel_Right_In_Scan_On_Curved_Long Scanning Distance.erg}
Step.6.0.Var.1.ManLst = 0 1 2 3
Step.6.0.Var.1.Log.0.Time = 13.208
Step.6.0.Var.1.Log.0.Kind = err
Step.6.0.Var.1.Log.0.Text = Simulation ended with errors
Step.7 = Group
Step.7.Name = SimScenario07: Tightest possible parking slot
Step.7.0 = TestRun
Step.7.0.Name = AP/00_AP_Demo/Parallel_Right_In_Scan_Off
Step.7.0.Param.0 = PBoxLength NValue
Step.7.0.Param.1 = PBoxWidth NValue
Step.7.0.Param.2 = PBoxY NValue
Step.7.0.Param.3 = GolfX NValue
Step.7.0.Param.4 = GolfY NValue
Step.7.0.Param.5 = LongOffset NValue
Step.7.0.Param.6 = CurbHeight NValue
Step.7.0.Var.0.Name = _Smaller Length
Step.7.0.Var.0.Param = 5.4 2.1 -5.984 36.4 -4.8 40.904 0.01
Step.7.0.Var.0.Result = err
Step.7.0.Var.0.ResDate = 1542195071
Step.7.0.Var.0.ResFiles = {SimOutput/FRL62RHG/20181114/Parallel_Right_In_Scan_Off__Smaller Length.erg}
Step.7.0.Var.0.ManLst = 0
Step.7.0.Var.0.Log.0.Time = 1.702
Step.7.0.Var.0.Log.0.Kind = err
Step.7.0.Var.0.Log.0.Text = Simulation ended with errors
Step.8 = Group
Step.8.Name = SimScenario08: Lines and obstacles
Step.8.0 = TestRun
Step.8.0.Name = AP/00_AP_Demo/Perpendicular_Right_In_Scan_Off_Lines
Step.8.0.Param.0 = LongOffset NValue
Step.8.0.Param.1 = GolfAng NValue
Step.8.0.Param.2 = GolfX NValue
Step.8.0.Param.3 = PBoxWidth NValue
Step.8.0.Param.4 = PBoxX NValue
Step.8.0.Var.0.Name = Standard
Step.8.0.Var.0.Param = 32 90 27.202 3.3 28.368
Step.8.0.Var.0.Result = err
Step.8.0.Var.0.ResDate = 1542195186
Step.8.0.Var.0.ResFiles = SimOutput/FRL62RHG/20181114/Perpendicular_Right_In_Scan_Off_Lines_Standard.erg
Step.8.0.Var.0.ManLst = 0 1
Step.8.0.Var.0.Log.0.Time = 5.721
Step.8.0.Var.0.Log.0.Kind = err
Step.8.0.Var.0.Log.0.Text = Simulation stopped by user
Step.8.0.Var.1.Name = Misaligned vehicles 1
Step.8.0.Var.1.Param = 32 82 27 3.2 28.468
Step.8.0.Var.1.Result = err
Step.8.0.Var.1.ResDate = 1542195268
Step.8.0.Var.1.ResFiles = {SimOutput/FRL62RHG/20181114/Perpendicular_Right_In_Scan_Off_Lines_Misaligned vehicles 1.erg}
Step.8.0.Var.1.ManLst = 0 1
Step.8.0.Var.1.Log.0.Time = 4.879
Step.8.0.Var.1.Log.0.Kind = err
Step.8.0.Var.1.Log.0.Text = Simulation stopped by user
Step.8.1 = TestRun
Step.8.1.Name = AP/00_AP_Demo/Perpendicular_Right_In_Scan_Off_Lines
Step.8.1.Param.0 = LongOffset NValue
Step.8.1.Param.1 = GolfAng NValue
Step.8.1.Param.2 = GolfX NValue
Step.8.1.Param.3 = PBoxWidth NValue
Step.8.1.Param.4 = PBoxX NValue
Step.8.1.Param.5 = PassatAng NValue
Step.8.1.Param.6 = PassatX NValue
Step.8.1.Var.0.Name = Misaligned vehicles 2
Step.8.1.Var.0.Param = 32 82 26.9 3.2 28.368 93 32.902
Step.8.1.Var.0.Result = err
Step.8.1.Var.0.ResDate = 1542195395
Step.8.1.Var.0.ResFiles = {SimOutput/FRL62RHG/20181114/Perpendicular_Right_In_Scan_Off_Lines_Misaligned vehicles 2.erg}
Step.8.1.Var.0.ManLst = 0
Step.8.1.Var.0.Log.0.Time = 1.312
Step.8.1.Var.0.Log.0.Kind = err
Step.8.1.Var.0.Log.0.Text = Simulation stopped by user
