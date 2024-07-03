#INFOFILE1.1 - Do not remove this line!
FileIdent = CarMaker-TestSeries 2
FileCreator = CarMaker 8.1.1 2019-12-19
Description:
LastChange = 2024-06-05 13:47:38 uie75869
StartTime = 2024-06-05 13:47:04
EndTime = 2024-06-05 13:47:15
ReportTemplate =
Step.0 = Settings
Step.0.Name = Global Settings
Step.1 = Group
Step.1.Name = 01_Explicit_MP
Step.1.0 = Group
Step.1.0.Name = Parallel
Step.1.0.0 = Group
Step.1.0.0.Name = Left
Step.1.0.0.0 = TestRun
Step.1.0.0.0.Name = AP/09_MP/Explicit/MP_NoHMI_Parallel_Left_userUpdate_Dev
Step.1.0.0.0.Crit.0.Name = Parking Slot
Step.1.0.0.0.Crit.0.Description:
Step.1.0.0.0.Crit.0.Good =
Step.1.0.0.0.Crit.0.Warn =
Step.1.0.0.0.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.1.0.0.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.1.0.0.0.Crit.1.Description:
Step.1.0.0.0.Crit.1.Good = [get MP_Finish] == 7
Step.1.0.0.0.Crit.1.Warn =
Step.1.0.0.0.Crit.1.Bad =
Step.1.0.0.0.Crit.2.Name = Position Comparison X
Step.1.0.0.0.Crit.2.Description:
Step.1.0.0.0.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.1.0.0.0.Crit.2.Warn =
Step.1.0.0.0.Crit.2.Bad =
Step.1.0.0.0.Crit.3.Name = Position Comparison Y
Step.1.0.0.0.Crit.3.Description:
Step.1.0.0.0.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.1.0.0.0.Crit.3.Warn =
Step.1.0.0.0.Crit.3.Bad =
Step.1.0.0.0.Crit.4.Name = Position Comparison YawRate
Step.1.0.0.0.Crit.4.Description:
Step.1.0.0.0.Crit.4.Good = [get AngposDelta] <= 0.175 && [get AngposDelta] >= -0.175
Step.1.0.0.0.Crit.4.Warn =
Step.1.0.0.0.Crit.4.Bad =
Step.1.0.0.0.Var.0.Name = Base
Step.1.0.0.0.Var.0.Param =
Step.1.0.0.0.Var.0.Result = good
Step.1.0.0.0.Var.0.ResDate = 1717606410
Step.1.0.0.0.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19
Step.1.0.0.0.Var.0.Crit.0.Name = Parking Slot
Step.1.0.0.0.Var.0.Crit.0.Result = good
Step.1.0.0.0.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.1.0.0.0.Var.0.Crit.1.Result = good
Step.1.0.0.0.Var.0.Crit.2.Name = Position Comparison X
Step.1.0.0.0.Var.0.Crit.2.Result = good
Step.1.0.0.0.Var.0.Crit.3.Name = Position Comparison Y
Step.1.0.0.0.Var.0.Crit.3.Result = good
Step.1.0.0.0.Var.0.Crit.4.Name = Position Comparison YawRate
Step.1.0.0.0.Var.0.Crit.4.Result = good
Step.1.0.0.1 = TestRun
Step.1.0.0.1.Name = AP/09_MP/Explicit/MP_NoHMI_Parallel_Left_No_userUpdate_Dev
Step.1.0.0.1.Crit.0.Name = Parking Slot
Step.1.0.0.1.Crit.0.Description:
Step.1.0.0.1.Crit.0.Good =
Step.1.0.0.1.Crit.0.Warn =
Step.1.0.0.1.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.1.0.0.1.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.1.0.0.1.Crit.1.Description:
Step.1.0.0.1.Crit.1.Good = [get MP_Finish] == 7
Step.1.0.0.1.Crit.1.Warn =
Step.1.0.0.1.Crit.1.Bad =
Step.1.0.0.1.Crit.2.Name = Position Comparison X
Step.1.0.0.1.Crit.2.Description:
Step.1.0.0.1.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.1.0.0.1.Crit.2.Warn =
Step.1.0.0.1.Crit.2.Bad =
Step.1.0.0.1.Crit.3.Name = Position Comparison Y
Step.1.0.0.1.Crit.3.Description:
Step.1.0.0.1.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.1.0.0.1.Crit.3.Warn =
Step.1.0.0.1.Crit.3.Bad =
Step.1.0.0.1.Crit.4.Name = Position Comparison YawRate
Step.1.0.0.1.Crit.4.Description:
Step.1.0.0.1.Crit.4.Good = [get AngposDelta] <= 0.175 && [get AngposDelta] >= -0.175
Step.1.0.0.1.Crit.4.Warn =
Step.1.0.0.1.Crit.4.Bad =
Step.1.0.0.1.Var.0.Name = Base
Step.1.0.0.1.Var.0.Param =
Step.1.0.0.1.Var.0.Result = good
Step.1.0.0.1.Var.0.ResDate = 1717606512
Step.1.0.0.1.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19
Step.1.0.0.1.Var.0.Crit.0.Name = Parking Slot
Step.1.0.0.1.Var.0.Crit.0.Result = good
Step.1.0.0.1.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.1.0.0.1.Var.0.Crit.1.Result = good
Step.1.0.0.1.Var.0.Crit.2.Name = Position Comparison X
Step.1.0.0.1.Var.0.Crit.2.Result = good
Step.1.0.0.1.Var.0.Crit.3.Name = Position Comparison Y
Step.1.0.0.1.Var.0.Crit.3.Result = good
Step.1.0.0.1.Var.0.Crit.4.Name = Position Comparison YawRate
Step.1.0.0.1.Var.0.Crit.4.Result = good
Step.1.0.1 = Group
Step.1.0.1.Name = Right
Step.1.0.1.0 = TestRun
Step.1.0.1.0.Name = AP/09_MP/Explicit/MP_NoHMI_Parallel_Right_userUpdate_Dev
Step.1.0.1.0.Crit.0.Name = Parking Slot
Step.1.0.1.0.Crit.0.Description:
Step.1.0.1.0.Crit.0.Good =
Step.1.0.1.0.Crit.0.Warn =
Step.1.0.1.0.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.1.0.1.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.1.0.1.0.Crit.1.Description:
Step.1.0.1.0.Crit.1.Good = [get MP_Finish] == 7
Step.1.0.1.0.Crit.1.Warn =
Step.1.0.1.0.Crit.1.Bad =
Step.1.0.1.0.Crit.2.Name = Position Comparison X
Step.1.0.1.0.Crit.2.Description:
Step.1.0.1.0.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.1.0.1.0.Crit.2.Warn =
Step.1.0.1.0.Crit.2.Bad =
Step.1.0.1.0.Crit.3.Name = Position Comparison Y
Step.1.0.1.0.Crit.3.Description:
Step.1.0.1.0.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.1.0.1.0.Crit.3.Warn =
Step.1.0.1.0.Crit.3.Bad =
Step.1.0.1.0.Crit.4.Name = Position Comparison YawRate
Step.1.0.1.0.Crit.4.Description:
Step.1.0.1.0.Crit.4.Good = [get AngposDelta] <= 0.175 && [get AngposDelta] >= -0.175
Step.1.0.1.0.Crit.4.Warn =
Step.1.0.1.0.Crit.4.Bad =
Step.1.0.1.0.Var.0.Name = Base
Step.1.0.1.0.Var.0.Param =
Step.1.0.1.0.Var.0.Result = good
Step.1.0.1.0.Var.0.ResDate = 1717606619
Step.1.0.1.0.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19
Step.1.0.1.0.Var.0.Crit.0.Name = Parking Slot
Step.1.0.1.0.Var.0.Crit.0.Result = good
Step.1.0.1.0.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.1.0.1.0.Var.0.Crit.1.Result = good
Step.1.0.1.0.Var.0.Crit.2.Name = Position Comparison X
Step.1.0.1.0.Var.0.Crit.2.Result = good
Step.1.0.1.0.Var.0.Crit.3.Name = Position Comparison Y
Step.1.0.1.0.Var.0.Crit.3.Result = good
Step.1.0.1.0.Var.0.Crit.4.Name = Position Comparison YawRate
Step.1.0.1.0.Var.0.Crit.4.Result = good
Step.1.0.1.1 = TestRun
Step.1.0.1.1.Name = AP/09_MP/Explicit/MP_NoHMI_Parallel_Right_No_userUpdate_Dev
Step.1.0.1.1.Crit.0.Name = Parking Slot
Step.1.0.1.1.Crit.0.Description:
Step.1.0.1.1.Crit.0.Good =
Step.1.0.1.1.Crit.0.Warn =
Step.1.0.1.1.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.1.0.1.1.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.1.0.1.1.Crit.1.Description:
Step.1.0.1.1.Crit.1.Good = [get MP_Finish] == 7
Step.1.0.1.1.Crit.1.Warn =
Step.1.0.1.1.Crit.1.Bad =
Step.1.0.1.1.Crit.2.Name = Position Comparison X
Step.1.0.1.1.Crit.2.Description:
Step.1.0.1.1.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.1.0.1.1.Crit.2.Warn =
Step.1.0.1.1.Crit.2.Bad =
Step.1.0.1.1.Crit.3.Name = Position Comparison Y
Step.1.0.1.1.Crit.3.Description:
Step.1.0.1.1.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.1.0.1.1.Crit.3.Warn =
Step.1.0.1.1.Crit.3.Bad =
Step.1.0.1.1.Crit.4.Name = Position Comparison YawRate
Step.1.0.1.1.Crit.4.Description:
Step.1.0.1.1.Crit.4.Good = [get AngposDelta] <= 0.175 && [get AngposDelta] >= -0.175
Step.1.0.1.1.Crit.4.Warn =
Step.1.0.1.1.Crit.4.Bad =
Step.1.0.1.1.Var.0.Name = Base
Step.1.0.1.1.Var.0.Param =
Step.1.0.1.1.Var.0.Result = good
Step.1.0.1.1.Var.0.ResDate = 1717606725
Step.1.0.1.1.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19
Step.1.0.1.1.Var.0.Crit.0.Name = Parking Slot
Step.1.0.1.1.Var.0.Crit.0.Result = good
Step.1.0.1.1.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.1.0.1.1.Var.0.Crit.1.Result = good
Step.1.0.1.1.Var.0.Crit.2.Name = Position Comparison X
Step.1.0.1.1.Var.0.Crit.2.Result = good
Step.1.0.1.1.Var.0.Crit.3.Name = Position Comparison Y
Step.1.0.1.1.Var.0.Crit.3.Result = good
Step.1.0.1.1.Var.0.Crit.4.Name = Position Comparison YawRate
Step.1.0.1.1.Var.0.Crit.4.Result = good
Step.1.1 = Group
Step.1.1.Name = Perpendicular
Step.1.1.0 = Group
Step.1.1.0.Name = Left
Step.1.1.0.0 = TestRun
Step.1.1.0.0.Name = AP/09_MP/Explicit/MP_NoHMI_Perpendicullar_Left_userUpdate_Dev
Step.1.1.0.0.Crit.0.Name = Parking Slot
Step.1.1.0.0.Crit.0.Description:
Step.1.1.0.0.Crit.0.Good =
Step.1.1.0.0.Crit.0.Warn =
Step.1.1.0.0.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.1.1.0.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.1.1.0.0.Crit.1.Description:
Step.1.1.0.0.Crit.1.Good = [get MP_Finish] == 7
Step.1.1.0.0.Crit.1.Warn =
Step.1.1.0.0.Crit.1.Bad =
Step.1.1.0.0.Crit.2.Name = Position Comparison X
Step.1.1.0.0.Crit.2.Description:
Step.1.1.0.0.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.1.1.0.0.Crit.2.Warn =
Step.1.1.0.0.Crit.2.Bad =
Step.1.1.0.0.Crit.3.Name = Position Comparison Y
Step.1.1.0.0.Crit.3.Description:
Step.1.1.0.0.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.1.1.0.0.Crit.3.Warn =
Step.1.1.0.0.Crit.3.Bad =
Step.1.1.0.0.Crit.4.Name = Position Comparison YawRate
Step.1.1.0.0.Crit.4.Description:
Step.1.1.0.0.Crit.4.Good = [get AngposDelta] <= 0.175 && [get AngposDelta] >= -0.175
Step.1.1.0.0.Crit.4.Warn =
Step.1.1.0.0.Crit.4.Bad =
Step.1.1.0.0.Var.0.Name = Base
Step.1.1.0.0.Var.0.Param =
Step.1.1.0.0.Var.0.Result = good
Step.1.1.0.0.Var.0.ResDate = 1717606823
Step.1.1.0.0.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19
Step.1.1.0.0.Var.0.Crit.0.Name = Parking Slot
Step.1.1.0.0.Var.0.Crit.0.Result = good
Step.1.1.0.0.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.1.1.0.0.Var.0.Crit.1.Result = good
Step.1.1.0.0.Var.0.Crit.2.Name = Position Comparison X
Step.1.1.0.0.Var.0.Crit.2.Result = good
Step.1.1.0.0.Var.0.Crit.3.Name = Position Comparison Y
Step.1.1.0.0.Var.0.Crit.3.Result = good
Step.1.1.0.0.Var.0.Crit.4.Name = Position Comparison YawRate
Step.1.1.0.0.Var.0.Crit.4.Result = good
Step.1.1.0.1 = TestRun
Step.1.1.0.1.Name = AP/09_MP/Explicit/MP_NoHMI_Perpendicullar_Left_No_userUpdate_Dev
Step.1.1.0.1.Crit.0.Name = Parking Slot
Step.1.1.0.1.Crit.0.Description:
Step.1.1.0.1.Crit.0.Good =
Step.1.1.0.1.Crit.0.Warn =
Step.1.1.0.1.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.1.1.0.1.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.1.1.0.1.Crit.1.Description:
Step.1.1.0.1.Crit.1.Good = [get MP_Finish] == 7
Step.1.1.0.1.Crit.1.Warn =
Step.1.1.0.1.Crit.1.Bad =
Step.1.1.0.1.Crit.2.Name = Position Comparison X
Step.1.1.0.1.Crit.2.Description:
Step.1.1.0.1.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.1.1.0.1.Crit.2.Warn =
Step.1.1.0.1.Crit.2.Bad =
Step.1.1.0.1.Crit.3.Name = Position Comparison Y
Step.1.1.0.1.Crit.3.Description:
Step.1.1.0.1.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.1.1.0.1.Crit.3.Warn =
Step.1.1.0.1.Crit.3.Bad =
Step.1.1.0.1.Crit.4.Name = Position Comparison YawRate
Step.1.1.0.1.Crit.4.Description:
Step.1.1.0.1.Crit.4.Good = [get AngposDelta] <= 0.175 && [get AngposDelta] >= -0.175
Step.1.1.0.1.Crit.4.Warn =
Step.1.1.0.1.Crit.4.Bad =
Step.1.1.0.1.Var.0.Name = Base
Step.1.1.0.1.Var.0.Param =
Step.1.1.0.1.Var.0.Result = good
Step.1.1.0.1.Var.0.ResDate = 1717606923
Step.1.1.0.1.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19
Step.1.1.0.1.Var.0.Crit.0.Name = Parking Slot
Step.1.1.0.1.Var.0.Crit.0.Result = good
Step.1.1.0.1.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.1.1.0.1.Var.0.Crit.1.Result = good
Step.1.1.0.1.Var.0.Crit.2.Name = Position Comparison X
Step.1.1.0.1.Var.0.Crit.2.Result = good
Step.1.1.0.1.Var.0.Crit.3.Name = Position Comparison Y
Step.1.1.0.1.Var.0.Crit.3.Result = good
Step.1.1.0.1.Var.0.Crit.4.Name = Position Comparison YawRate
Step.1.1.0.1.Var.0.Crit.4.Result = good
Step.1.1.1 = Group
Step.1.1.1.Name = Right
Step.1.1.1.0 = TestRun
Step.1.1.1.0.Name = AP/09_MP/Explicit/MP_NoHMI_Perpendicullar_Right_userUpdate_Dev
Step.1.1.1.0.Crit.0.Name = Parking Slot
Step.1.1.1.0.Crit.0.Description:
Step.1.1.1.0.Crit.0.Good =
Step.1.1.1.0.Crit.0.Warn =
Step.1.1.1.0.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.1.1.1.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.1.1.1.0.Crit.1.Description:
Step.1.1.1.0.Crit.1.Good = [get MP_Finish] == 7
Step.1.1.1.0.Crit.1.Warn =
Step.1.1.1.0.Crit.1.Bad =
Step.1.1.1.0.Crit.2.Name = Position Comparison X
Step.1.1.1.0.Crit.2.Description:
Step.1.1.1.0.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.1.1.1.0.Crit.2.Warn =
Step.1.1.1.0.Crit.2.Bad =
Step.1.1.1.0.Crit.3.Name = Position Comparison Y
Step.1.1.1.0.Crit.3.Description:
Step.1.1.1.0.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.1.1.1.0.Crit.3.Warn =
Step.1.1.1.0.Crit.3.Bad =
Step.1.1.1.0.Crit.4.Name = Position Comparison YawRate
Step.1.1.1.0.Crit.4.Description:
Step.1.1.1.0.Crit.4.Good = [get AngposDelta] <= 0.175 && [get AngposDelta] >= -0.175
Step.1.1.1.0.Crit.4.Warn =
Step.1.1.1.0.Crit.4.Bad =
Step.1.1.1.0.Var.0.Name = Base
Step.1.1.1.0.Var.0.Param =
Step.1.1.1.0.Var.0.Result = good
Step.1.1.1.0.Var.0.ResDate = 1717607043
Step.1.1.1.0.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20
Step.1.1.1.0.Var.0.Crit.0.Name = Parking Slot
Step.1.1.1.0.Var.0.Crit.0.Result = good
Step.1.1.1.0.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.1.1.1.0.Var.0.Crit.1.Result = good
Step.1.1.1.0.Var.0.Crit.2.Name = Position Comparison X
Step.1.1.1.0.Var.0.Crit.2.Result = good
Step.1.1.1.0.Var.0.Crit.3.Name = Position Comparison Y
Step.1.1.1.0.Var.0.Crit.3.Result = good
Step.1.1.1.0.Var.0.Crit.4.Name = Position Comparison YawRate
Step.1.1.1.0.Var.0.Crit.4.Result = good
Step.1.1.1.1 = TestRun
Step.1.1.1.1.Name = AP/09_MP/Explicit/MP_NoHMI_Perpendicullar_Right_No_userUpdate_Dev
Step.1.1.1.1.Crit.0.Name = Parking Slot
Step.1.1.1.1.Crit.0.Description:
Step.1.1.1.1.Crit.0.Good =
Step.1.1.1.1.Crit.0.Warn =
Step.1.1.1.1.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.1.1.1.1.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.1.1.1.1.Crit.1.Description:
Step.1.1.1.1.Crit.1.Good = [get MP_Finish] == 7
Step.1.1.1.1.Crit.1.Warn =
Step.1.1.1.1.Crit.1.Bad =
Step.1.1.1.1.Crit.2.Name = Position Comparison X
Step.1.1.1.1.Crit.2.Description:
Step.1.1.1.1.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.1.1.1.1.Crit.2.Warn =
Step.1.1.1.1.Crit.2.Bad =
Step.1.1.1.1.Crit.3.Name = Position Comparison Y
Step.1.1.1.1.Crit.3.Description:
Step.1.1.1.1.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.1.1.1.1.Crit.3.Warn =
Step.1.1.1.1.Crit.3.Bad =
Step.1.1.1.1.Crit.4.Name = Position Comparison YawRate
Step.1.1.1.1.Crit.4.Description:
Step.1.1.1.1.Crit.4.Good = [get AngposDelta] <= 0.175 && [get AngposDelta] >= -0.175
Step.1.1.1.1.Crit.4.Warn =
Step.1.1.1.1.Crit.4.Bad =
Step.1.1.1.1.Var.0.Name = Base
Step.1.1.1.1.Var.0.Param =
Step.1.1.1.1.Var.0.Result = good
Step.1.1.1.1.Var.0.ResDate = 1717607162
Step.1.1.1.1.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20
Step.1.1.1.1.Var.0.Crit.0.Name = Parking Slot
Step.1.1.1.1.Var.0.Crit.0.Result = good
Step.1.1.1.1.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.1.1.1.1.Var.0.Crit.1.Result = good
Step.1.1.1.1.Var.0.Crit.2.Name = Position Comparison X
Step.1.1.1.1.Var.0.Crit.2.Result = good
Step.1.1.1.1.Var.0.Crit.3.Name = Position Comparison Y
Step.1.1.1.1.Var.0.Crit.3.Result = good
Step.1.1.1.1.Var.0.Crit.4.Name = Position Comparison YawRate
Step.1.1.1.1.Var.0.Crit.4.Result = good
Step.2 = Group
Step.2.Name = 00_Basic_Test
Step.2.0 = TestRun
Step.2.0.Name = AP/MP_2_A_Create_ParkIn_Save_DriveAway_ComeBack_Redetect_ParkIn
Step.2.0.Crit.0.Name = Parking Slot
Step.2.0.Crit.0.Description:
Step.2.0.Crit.0.Good =
Step.2.0.Crit.0.Warn =
Step.2.0.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.2.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.2.0.Crit.1.Description:
Step.2.0.Crit.1.Good = [get AP.planningCtrlPort.mpStates] == 7
Step.2.0.Crit.1.Warn =
Step.2.0.Crit.1.Bad =
Step.2.0.Crit.2.Name = Position Comparison X
Step.2.0.Crit.2.Description:
Step.2.0.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.2.0.Crit.2.Warn =
Step.2.0.Crit.2.Bad =
Step.2.0.Crit.3.Name = Position Comparison Y
Step.2.0.Crit.3.Description:
Step.2.0.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.2.0.Crit.3.Warn =
Step.2.0.Crit.3.Bad =
Step.2.0.Crit.4.Name = Position Comparison YawRate
Step.2.0.Crit.4.Description:
Step.2.0.Crit.4.Good = [get AngposDelta] <= 0.175 && [get AngposDelta] >= -0.175
Step.2.0.Crit.4.Warn =
Step.2.0.Crit.4.Bad =
Step.2.0.Var.0.Name = Base
Step.2.0.Var.0.Param =
Step.3 = Group
Step.3.Name = 02_Implicit_MP_(Under_Development)
Step.3.0 = Group
Step.3.0.Name = Refine
Step.3.0.0 = TestRun
Step.3.0.0.Name = AP/09_MP/Implicit/Refine/MP_NoHMI_Implicit_Backward_Perpendicular_Right_1_Unit_Manual_Scanning_refining_traslation_up
Step.3.0.0.Crit.0.Name = Parking Slot
Step.3.0.0.Crit.0.Description:
Step.3.0.0.Crit.0.Good =
Step.3.0.0.Crit.0.Warn =
Step.3.0.0.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.3.0.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.0.0.Crit.1.Description:
Step.3.0.0.Crit.1.Good = [get MP_Finish] == 7
Step.3.0.0.Crit.1.Warn =
Step.3.0.0.Crit.1.Bad =
Step.3.0.0.Crit.2.Name = Position Comparison X
Step.3.0.0.Crit.2.Description:
Step.3.0.0.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.3.0.0.Crit.2.Warn =
Step.3.0.0.Crit.2.Bad =
Step.3.0.0.Crit.3.Name = Position Comparison Y
Step.3.0.0.Crit.3.Description:
Step.3.0.0.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta]  >= -0.1
Step.3.0.0.Crit.3.Warn =
Step.3.0.0.Crit.3.Bad =
Step.3.0.0.Crit.4.Name = Position Comparison Yaw
Step.3.0.0.Crit.4.Description:
Step.3.0.0.Crit.4.Good = [get AngposDelta] <= 0.1 && [get AngposDelta]>= -0.175
Step.3.0.0.Crit.4.Warn =
Step.3.0.0.Crit.4.Bad =
Step.3.0.0.Var.0.Name = Base
Step.3.0.0.Var.0.Param =
Step.3.0.0.Var.0.Result = good
Step.3.0.0.Var.0.ResDate = 1717607323
Step.3.0.0.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24
Step.3.0.0.Var.0.Crit.0.Name = Parking Slot
Step.3.0.0.Var.0.Crit.0.Result = good
Step.3.0.0.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.0.0.Var.0.Crit.1.Result = good
Step.3.0.0.Var.0.Crit.2.Name = Position Comparison X
Step.3.0.0.Var.0.Crit.2.Result = good
Step.3.0.0.Var.0.Crit.3.Name = Position Comparison Y
Step.3.0.0.Var.0.Crit.3.Result = good
Step.3.0.0.Var.0.Crit.4.Name = Position Comparison Yaw
Step.3.0.0.Var.0.Crit.4.Result = good
Step.3.0.1 = TestRun
Step.3.0.1.Name = AP/09_MP/Implicit/Refine/MP_NoHMI_Implicit_Backward_Perpendicular_Right_1_Unit_Manual_Scanning_refining_traslation_left
Step.3.0.1.Crit.0.Name = Parking Slot
Step.3.0.1.Crit.0.Description:
Step.3.0.1.Crit.0.Good =
Step.3.0.1.Crit.0.Warn =
Step.3.0.1.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.3.0.1.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.0.1.Crit.1.Description:
Step.3.0.1.Crit.1.Good = [get MP_Finish] == 7
Step.3.0.1.Crit.1.Warn =
Step.3.0.1.Crit.1.Bad =
Step.3.0.1.Crit.2.Name = Position Comparison X
Step.3.0.1.Crit.2.Description:
Step.3.0.1.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.3.0.1.Crit.2.Warn =
Step.3.0.1.Crit.2.Bad =
Step.3.0.1.Crit.3.Name = Position Comparison Y
Step.3.0.1.Crit.3.Description:
Step.3.0.1.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta]  >= -0.1
Step.3.0.1.Crit.3.Warn =
Step.3.0.1.Crit.3.Bad =
Step.3.0.1.Crit.4.Name = Position Comparison Yaw
Step.3.0.1.Crit.4.Description:
Step.3.0.1.Crit.4.Good = [get AngposDelta] <= 0.1 && [get AngposDelta]>= -0.175
Step.3.0.1.Crit.4.Warn =
Step.3.0.1.Crit.4.Bad =
Step.3.0.1.Var.0.Name = Base
Step.3.0.1.Var.0.Param =
Step.3.0.1.Var.0.Result = good
Step.3.0.1.Var.0.ResDate = 1717607529
Step.3.0.1.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24
Step.3.0.1.Var.0.Crit.0.Name = Parking Slot
Step.3.0.1.Var.0.Crit.0.Result = good
Step.3.0.1.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.0.1.Var.0.Crit.1.Result = good
Step.3.0.1.Var.0.Crit.2.Name = Position Comparison X
Step.3.0.1.Var.0.Crit.2.Result = good
Step.3.0.1.Var.0.Crit.3.Name = Position Comparison Y
Step.3.0.1.Var.0.Crit.3.Result = good
Step.3.0.1.Var.0.Crit.4.Name = Position Comparison Yaw
Step.3.0.1.Var.0.Crit.4.Result = good
Step.3.0.2 = TestRun
Step.3.0.2.Name = AP/09_MP/Implicit/Refine/MP_NoHMI_Implicit_Backward_Perpendicular_Right_1_Unit_Manual_Scanning_refining_rplust
Step.3.0.2.Crit.0.Name = Parking Slot
Step.3.0.2.Crit.0.Description:
Step.3.0.2.Crit.0.Good =
Step.3.0.2.Crit.0.Warn =
Step.3.0.2.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.3.0.2.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.0.2.Crit.1.Description:
Step.3.0.2.Crit.1.Good = [get MP_Finish] == 7
Step.3.0.2.Crit.1.Warn =
Step.3.0.2.Crit.1.Bad =
Step.3.0.2.Crit.2.Name = Position Comparison X
Step.3.0.2.Crit.2.Description:
Step.3.0.2.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.3.0.2.Crit.2.Warn =
Step.3.0.2.Crit.2.Bad =
Step.3.0.2.Crit.3.Name = Position Comparison Y
Step.3.0.2.Crit.3.Description:
Step.3.0.2.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta]  >= -0.1
Step.3.0.2.Crit.3.Warn =
Step.3.0.2.Crit.3.Bad =
Step.3.0.2.Crit.4.Name = Position Comparison Yaw
Step.3.0.2.Crit.4.Description:
Step.3.0.2.Crit.4.Good = [get AngposDelta] <= 0.1 && [get AngposDelta]>= -0.175
Step.3.0.2.Crit.4.Warn =
Step.3.0.2.Crit.4.Bad =
Step.3.0.2.Var.0.Name = Base
Step.3.0.2.Var.0.Param =
Step.3.0.2.Var.0.Result = good
Step.3.0.2.Var.0.ResDate = 1717607665
Step.3.0.2.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24
Step.3.0.2.Var.0.Crit.0.Name = Parking Slot
Step.3.0.2.Var.0.Crit.0.Result = good
Step.3.0.2.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.0.2.Var.0.Crit.1.Result = good
Step.3.0.2.Var.0.Crit.2.Name = Position Comparison X
Step.3.0.2.Var.0.Crit.2.Result = good
Step.3.0.2.Var.0.Crit.3.Name = Position Comparison Y
Step.3.0.2.Var.0.Crit.3.Result = good
Step.3.0.2.Var.0.Crit.4.Name = Position Comparison Yaw
Step.3.0.2.Var.0.Crit.4.Result = good
Step.3.0.3 = TestRun
Step.3.0.3.Name = AP/09_MP/Implicit/Refine/MP_NoHMI_Implicit_Backward_Perpendicular_Right_1_Unit_Manual_Scanning_refining_rotation
Step.3.0.3.Crit.0.Name = Parking Slot
Step.3.0.3.Crit.0.Description:
Step.3.0.3.Crit.0.Good =
Step.3.0.3.Crit.0.Warn =
Step.3.0.3.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.3.0.3.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.0.3.Crit.1.Description:
Step.3.0.3.Crit.1.Good = [get MP_Finish] == 7
Step.3.0.3.Crit.1.Warn =
Step.3.0.3.Crit.1.Bad =
Step.3.0.3.Crit.2.Name = Position Comparison X
Step.3.0.3.Crit.2.Description:
Step.3.0.3.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.3.0.3.Crit.2.Warn =
Step.3.0.3.Crit.2.Bad =
Step.3.0.3.Crit.3.Name = Position Comparison Y
Step.3.0.3.Crit.3.Description:
Step.3.0.3.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta]  >= -0.1
Step.3.0.3.Crit.3.Warn =
Step.3.0.3.Crit.3.Bad =
Step.3.0.3.Crit.4.Name = Position Comparison Yaw
Step.3.0.3.Crit.4.Description:
Step.3.0.3.Crit.4.Good = [get AngposDelta] <= 0.1 && [get AngposDelta]>= -0.175
Step.3.0.3.Crit.4.Warn =
Step.3.0.3.Crit.4.Bad =
Step.3.0.3.Var.0.Name = Base
Step.3.0.3.Var.0.Param =
Step.3.0.3.Var.0.Result = good
Step.3.0.3.Var.0.ResDate = 1717607833
Step.3.0.3.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24
Step.3.0.3.Var.0.Crit.0.Name = Parking Slot
Step.3.0.3.Var.0.Crit.0.Result = good
Step.3.0.3.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.0.3.Var.0.Crit.1.Result = good
Step.3.0.3.Var.0.Crit.2.Name = Position Comparison X
Step.3.0.3.Var.0.Crit.2.Result = good
Step.3.0.3.Var.0.Crit.3.Name = Position Comparison Y
Step.3.0.3.Var.0.Crit.3.Result = good
Step.3.0.3.Var.0.Crit.4.Name = Position Comparison Yaw
Step.3.0.3.Var.0.Crit.4.Result = good
Step.3.1 = Group
Step.3.1.Name = Angular_Bay_left
Step.3.1.0 = TestRun
Step.3.1.0.Name = AP/09_MP/Implicit/MP_NoHMI_Implicit_AngLeft_Unit_Manual_Scanning_Dev
Step.3.1.0.Crit.0.Name = Parking Slot
Step.3.1.0.Crit.0.Description:
Step.3.1.0.Crit.0.Good =
Step.3.1.0.Crit.0.Warn =
Step.3.1.0.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.3.1.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.1.0.Crit.1.Description:
Step.3.1.0.Crit.1.Good = [get MP_Finish] == 7
Step.3.1.0.Crit.1.Warn =
Step.3.1.0.Crit.1.Bad =
Step.3.1.0.Crit.2.Name = Position Comparison X
Step.3.1.0.Crit.2.Description:
Step.3.1.0.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.3.1.0.Crit.2.Warn =
Step.3.1.0.Crit.2.Bad =
Step.3.1.0.Crit.3.Name = Position Comparison Y
Step.3.1.0.Crit.3.Description:
Step.3.1.0.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.3.1.0.Crit.3.Warn =
Step.3.1.0.Crit.3.Bad =
Step.3.1.0.Crit.4.Name = Position Comparison YawRate
Step.3.1.0.Crit.4.Description:
Step.3.1.0.Crit.4.Good = [get AngposDelta] <= 0.175 && [get AngposDelta] >= -0.175
Step.3.1.0.Crit.4.Warn =
Step.3.1.0.Crit.4.Bad =
Step.3.1.0.Var.0.Name = Base
Step.3.1.0.Var.0.Param =
Step.3.1.0.Var.0.Result = err
Step.3.1.0.Var.0.ResDate = 1716868177
Step.3.1.0.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20
Step.3.1.0.Var.0.Log.0.Time = 57.040
Step.3.1.0.Var.0.Log.0.Kind = err
Step.3.1.0.Var.0.Log.0.Text = Simulation ended with errors
Step.3.1.0.Var.0.Crit.0.Name = Parking Slot
Step.3.1.0.Var.0.Crit.0.Result = good
Step.3.1.0.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.1.0.Var.0.Crit.1.Result = good
Step.3.1.0.Var.0.Crit.2.Name = Position Comparison X
Step.3.1.0.Var.0.Crit.2.Result = good
Step.3.1.0.Var.0.Crit.3.Name = Position Comparison Y
Step.3.1.0.Var.0.Crit.3.Result = good
Step.3.1.0.Var.0.Crit.4.Name = Position Comparison YawRate
Step.3.1.0.Var.0.Crit.4.Result = good
Step.3.2 = Group
Step.3.2.Name = Angular_Bay_right
Step.3.2.0 = TestRun
Step.3.2.0.Name = AP/09_MP/Implicit/MP_NoHMI_Implicit_AngRight_Unit_Manual_Scanning_Dev
Step.3.2.0.Crit.0.Name = Parking Slot
Step.3.2.0.Crit.0.Description:
Step.3.2.0.Crit.0.Good =
Step.3.2.0.Crit.0.Warn =
Step.3.2.0.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.3.2.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.2.0.Crit.1.Description:
Step.3.2.0.Crit.1.Good = [get MP_Finish] == 7
Step.3.2.0.Crit.1.Warn =
Step.3.2.0.Crit.1.Bad =
Step.3.2.0.Crit.2.Name = Position Comparison X
Step.3.2.0.Crit.2.Description:
Step.3.2.0.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.3.2.0.Crit.2.Warn =
Step.3.2.0.Crit.2.Bad =
Step.3.2.0.Crit.3.Name = Position Comparison Y
Step.3.2.0.Crit.3.Description:
Step.3.2.0.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.3.2.0.Crit.3.Warn =
Step.3.2.0.Crit.3.Bad =
Step.3.2.0.Crit.4.Name = Position Comparison YawRate
Step.3.2.0.Crit.4.Description:
Step.3.2.0.Crit.4.Good = [get AngposDelta] <= 0.175 && [get AngposDelta] >= -0.175
Step.3.2.0.Crit.4.Warn =
Step.3.2.0.Crit.4.Bad =
Step.3.2.0.Var.0.Name = Base
Step.3.2.0.Var.0.Param =
Step.3.2.0.Var.0.Result = good
Step.3.2.0.Var.0.ResDate = 1717607980
Step.3.2.0.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23
Step.3.2.0.Var.0.Crit.0.Name = Parking Slot
Step.3.2.0.Var.0.Crit.0.Result = good
Step.3.2.0.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.2.0.Var.0.Crit.1.Result = good
Step.3.2.0.Var.0.Crit.2.Name = Position Comparison X
Step.3.2.0.Var.0.Crit.2.Result = good
Step.3.2.0.Var.0.Crit.3.Name = Position Comparison Y
Step.3.2.0.Var.0.Crit.3.Result = good
Step.3.2.0.Var.0.Crit.4.Name = Position Comparison YawRate
Step.3.2.0.Var.0.Crit.4.Result = good
Step.3.3 = Group
Step.3.3.Name = Perpendicular
Step.3.3.0 = Group
Step.3.3.0.Name = Right
Step.3.3.0.0 = TestRun
Step.3.3.0.0.Name = AP/09_MP/Implicit/MP_NoHMI_Implicit_Backward_Perpendicular_Right_1_Unit_Manual_Scanning_Dev
Step.3.3.0.0.Crit.0.Name = Parking Slot
Step.3.3.0.0.Crit.0.Description:
Step.3.3.0.0.Crit.0.Good =
Step.3.3.0.0.Crit.0.Warn =
Step.3.3.0.0.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.3.3.0.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.3.0.0.Crit.1.Description:
Step.3.3.0.0.Crit.1.Good = [get MP_Finish] == 7
Step.3.3.0.0.Crit.1.Warn =
Step.3.3.0.0.Crit.1.Bad =
Step.3.3.0.0.Crit.2.Name = Position Comparison X
Step.3.3.0.0.Crit.2.Description:
Step.3.3.0.0.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.3.3.0.0.Crit.2.Warn =
Step.3.3.0.0.Crit.2.Bad =
Step.3.3.0.0.Crit.3.Name = Position Comparison Y
Step.3.3.0.0.Crit.3.Description:
Step.3.3.0.0.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.3.3.0.0.Crit.3.Warn =
Step.3.3.0.0.Crit.3.Bad =
Step.3.3.0.0.Crit.4.Name = Position Comparison YawRate
Step.3.3.0.0.Crit.4.Description:
Step.3.3.0.0.Crit.4.Good = [get AngposDelta] <= 0.175 && [get AngposDelta] >= -0.175
Step.3.3.0.0.Crit.4.Warn =
Step.3.3.0.0.Crit.4.Bad =
Step.3.3.0.0.Var.0.Name = Base
Step.3.3.0.0.Var.0.Param =
Step.3.3.0.0.Var.0.Result = good
Step.3.3.0.0.Var.0.ResDate = 1717608171
Step.3.3.0.0.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22
Step.3.3.0.0.Var.0.Crit.0.Name = Parking Slot
Step.3.3.0.0.Var.0.Crit.0.Result = good
Step.3.3.0.0.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.3.0.0.Var.0.Crit.1.Result = good
Step.3.3.0.0.Var.0.Crit.2.Name = Position Comparison X
Step.3.3.0.0.Var.0.Crit.2.Result = good
Step.3.3.0.0.Var.0.Crit.3.Name = Position Comparison Y
Step.3.3.0.0.Var.0.Crit.3.Result = good
Step.3.3.0.0.Var.0.Crit.4.Name = Position Comparison YawRate
Step.3.3.0.0.Var.0.Crit.4.Result = good
Step.3.3.0.1 = TestRun
Step.3.3.0.1.Name = AP/09_MP/Implicit/MP_NoHMI_Implicit_Backward_Perpendicular_Right_2_Units_Manual_Scanning_working_Dev
Step.3.3.0.1.Crit.0.Name = Parking Slot
Step.3.3.0.1.Crit.0.Description:
Step.3.3.0.1.Crit.0.Good =
Step.3.3.0.1.Crit.0.Warn =
Step.3.3.0.1.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.3.3.0.1.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.3.0.1.Crit.1.Description:
Step.3.3.0.1.Crit.1.Good = [get MP_Finish] == 7
Step.3.3.0.1.Crit.1.Warn =
Step.3.3.0.1.Crit.1.Bad =
Step.3.3.0.1.Crit.2.Name = Position Comparison X
Step.3.3.0.1.Crit.2.Description:
Step.3.3.0.1.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.3.3.0.1.Crit.2.Warn =
Step.3.3.0.1.Crit.2.Bad =
Step.3.3.0.1.Crit.3.Name = Position Comparison Y
Step.3.3.0.1.Crit.3.Description:
Step.3.3.0.1.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.3.3.0.1.Crit.3.Warn =
Step.3.3.0.1.Crit.3.Bad =
Step.3.3.0.1.Crit.4.Name = Position Comparison YawRate
Step.3.3.0.1.Crit.4.Description:
Step.3.3.0.1.Crit.4.Good = [get AngposDelta] <= 0.175 && [get AngposDelta] >= -0.175
Step.3.3.0.1.Crit.4.Warn =
Step.3.3.0.1.Crit.4.Bad =
Step.3.3.0.1.Var.0.Name = Base
Step.3.3.0.1.Var.0.Param =
Step.3.3.0.1.Var.0.Result = good
Step.3.3.0.1.Var.0.ResDate = 1717610826
Step.3.3.0.1.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24
Step.3.3.0.1.Var.0.Crit.0.Name = Parking Slot
Step.3.3.0.1.Var.0.Crit.0.Result = good
Step.3.3.0.1.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.3.0.1.Var.0.Crit.1.Result = good
Step.3.3.0.1.Var.0.Crit.2.Name = Position Comparison X
Step.3.3.0.1.Var.0.Crit.2.Result = good
Step.3.3.0.1.Var.0.Crit.3.Name = Position Comparison Y
Step.3.3.0.1.Var.0.Crit.3.Result = good
Step.3.3.0.1.Var.0.Crit.4.Name = Position Comparison YawRate
Step.3.3.0.1.Var.0.Crit.4.Result = good
Step.3.3.1 = Group
Step.3.3.1.Name = Left
Step.3.3.1.0 = TestRun
Step.3.3.1.0.Name = AP/09_MP/Implicit/MP_NoHMI_Implicit_Backward_Perpendicular_Left_1_Unit_Manual_Scanning_Dev
Step.3.3.1.0.Crit.0.Name = Parking Slot
Step.3.3.1.0.Crit.0.Description:
Step.3.3.1.0.Crit.0.Good =
Step.3.3.1.0.Crit.0.Warn =
Step.3.3.1.0.Crit.0.Bad = [get AP.memoryParkingStatusPort.numStoredMemoryParkingSlots_nu] == 0
Step.3.3.1.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.3.1.0.Crit.1.Description:
Step.3.3.1.0.Crit.1.Good = [get MP_Finish] == 7
Step.3.3.1.0.Crit.1.Warn =
Step.3.3.1.0.Crit.1.Bad =
Step.3.3.1.0.Crit.2.Name = Position Comparison X
Step.3.3.1.0.Crit.2.Description:
Step.3.3.1.0.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.3.3.1.0.Crit.2.Warn =
Step.3.3.1.0.Crit.2.Bad =
Step.3.3.1.0.Crit.3.Name = Position Comparison Y
Step.3.3.1.0.Crit.3.Description:
Step.3.3.1.0.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.3.3.1.0.Crit.3.Warn =
Step.3.3.1.0.Crit.3.Bad =
Step.3.3.1.0.Crit.4.Name = Position Comparison YawRate
Step.3.3.1.0.Crit.4.Description:
Step.3.3.1.0.Crit.4.Good = [get AngposDelta] <= 0.175 && [get AngposDelta] >= -0.175
Step.3.3.1.0.Crit.4.Warn =
Step.3.3.1.0.Crit.4.Bad =
Step.3.3.1.0.Var.0.Name = Base
Step.3.3.1.0.Var.0.Param =
Step.3.3.1.0.Var.0.Result = err
Step.3.3.1.0.Var.0.ResDate = 1711125750
Step.3.3.1.0.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20
Step.3.3.1.0.Var.0.Log.0.Time = 52.052
Step.3.3.1.0.Var.0.Log.0.Kind = err
Step.3.3.1.0.Var.0.Log.0.Text = Simulation ended with errors
Step.3.3.1.0.Var.0.Crit.0.Name = Parking Slot
Step.3.3.1.0.Var.0.Crit.0.Result = good
Step.3.3.1.0.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.3.1.0.Var.0.Crit.1.Result = bad
Step.3.3.1.0.Var.0.Crit.2.Name = Position Comparison X
Step.3.3.1.0.Var.0.Crit.2.Result = good
Step.3.3.1.0.Var.0.Crit.3.Name = Position Comparison Y
Step.3.3.1.0.Var.0.Crit.3.Result = good
Step.3.3.1.0.Var.0.Crit.4.Name = Position Comparison YawRate
Step.3.3.1.0.Var.0.Crit.4.Result = good
Step.3.3.1.1 = TestRun
Step.3.3.1.1.Name = AP/09_MP/Implicit/MP_NoHMI_Implicit_Backward_Perpendicular_Left_1_Unit_Manual_Scanning Drafty_Dev
Step.3.3.1.1.Crit.0.Name = Parking Slot
Step.3.3.1.1.Crit.0.Description:
Step.3.3.1.1.Crit.0.Good =
Step.3.3.1.1.Crit.0.Warn =
Step.3.3.1.1.Crit.0.Bad = [get AP.memoryParkingStatusPort.numStoredMemoryParkingSlots_nu] == 0
Step.3.3.1.1.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.3.1.1.Crit.1.Description:
Step.3.3.1.1.Crit.1.Good = [get MP_Finish] == 7
Step.3.3.1.1.Crit.1.Warn =
Step.3.3.1.1.Crit.1.Bad =
Step.3.3.1.1.Crit.2.Name = Position Comparison X
Step.3.3.1.1.Crit.2.Description:
Step.3.3.1.1.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.3.3.1.1.Crit.2.Warn =
Step.3.3.1.1.Crit.2.Bad =
Step.3.3.1.1.Crit.3.Name = Position Comparison Y
Step.3.3.1.1.Crit.3.Description:
Step.3.3.1.1.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.3.3.1.1.Crit.3.Warn =
Step.3.3.1.1.Crit.3.Bad =
Step.3.3.1.1.Crit.4.Name = Position Comparison YawRate
Step.3.3.1.1.Crit.4.Description:
Step.3.3.1.1.Crit.4.Good = [get AngposDelta] <= 0.175 && [get AngposDelta] >= -0.175
Step.3.3.1.1.Crit.4.Warn =
Step.3.3.1.1.Crit.4.Bad =
Step.3.3.1.1.Var.0.Name = Base
Step.3.3.1.1.Var.0.Param =
Step.3.3.1.1.Var.0.Result = err
Step.3.3.1.1.Var.0.ResDate = 1711125134
Step.3.3.1.1.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21
Step.3.3.1.1.Var.0.Log.0.Time = 175.198
Step.3.3.1.1.Var.0.Log.0.Kind = err
Step.3.3.1.1.Var.0.Log.0.Text = Simulation stopped by user
Step.3.4 = Group
Step.3.4.Name = Parallel
Step.3.4.0 = Group
Step.3.4.0.Name = Right
Step.3.4.0.0 = TestRun
Step.3.4.0.0.Name = AP/09_MP/Implicit/MP_NoHMI_Implicit_Parallel_Right_1_Unit_Manual_Scanning_working_Dev
Step.3.4.0.0.Crit.0.Name = Parking Slot
Step.3.4.0.0.Crit.0.Description:
Step.3.4.0.0.Crit.0.Good =
Step.3.4.0.0.Crit.0.Warn =
Step.3.4.0.0.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.3.4.0.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.4.0.0.Crit.1.Description:
Step.3.4.0.0.Crit.1.Good = [get MP_Finish] == 7
Step.3.4.0.0.Crit.1.Warn =
Step.3.4.0.0.Crit.1.Bad =
Step.3.4.0.0.Crit.2.Name = Position Comparison X
Step.3.4.0.0.Crit.2.Description:
Step.3.4.0.0.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.3.4.0.0.Crit.2.Warn =
Step.3.4.0.0.Crit.2.Bad =
Step.3.4.0.0.Crit.3.Name = Position Comparison Y
Step.3.4.0.0.Crit.3.Description:
Step.3.4.0.0.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.3.4.0.0.Crit.3.Warn =
Step.3.4.0.0.Crit.3.Bad =
Step.3.4.0.0.Crit.4.Name = Position Comparison YawRate
Step.3.4.0.0.Crit.4.Description:
Step.3.4.0.0.Crit.4.Good = [get AngposDelta] <= 0.175 && [get AngposDelta] >= -0.175
Step.3.4.0.0.Crit.4.Warn =
Step.3.4.0.0.Crit.4.Bad =
Step.3.4.0.0.Var.0.Name = Base
Step.3.4.0.0.Var.0.Param =
Step.3.4.0.0.Var.0.Result = good
Step.3.4.0.0.Var.0.ResDate = 1717608766
Step.3.4.0.0.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25
Step.3.4.0.0.Var.0.Crit.0.Name = Parking Slot
Step.3.4.0.0.Var.0.Crit.0.Result = good
Step.3.4.0.0.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.4.0.0.Var.0.Crit.1.Result = good
Step.3.4.0.0.Var.0.Crit.2.Name = Position Comparison X
Step.3.4.0.0.Var.0.Crit.2.Result = good
Step.3.4.0.0.Var.0.Crit.3.Name = Position Comparison Y
Step.3.4.0.0.Var.0.Crit.3.Result = good
Step.3.4.0.0.Var.0.Crit.4.Name = Position Comparison YawRate
Step.3.4.0.0.Var.0.Crit.4.Result = good
Step.3.4.1 = Group
Step.3.4.1.Name = Left
Step.3.4.1.0 = TestRun
Step.3.4.1.0.Name = AP/09_MP/Implicit/MP_NoHMI_Implicit_Parallel_Left_1_Unit_Manual_Scanning_working_Dev
Step.3.4.1.0.Crit.0.Name = Parking Slot
Step.3.4.1.0.Crit.0.Description:
Step.3.4.1.0.Crit.0.Good =
Step.3.4.1.0.Crit.0.Warn =
Step.3.4.1.0.Crit.0.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.3.4.1.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.4.1.0.Crit.1.Description:
Step.3.4.1.0.Crit.1.Good = [get MP_Finish] == 7
Step.3.4.1.0.Crit.1.Warn =
Step.3.4.1.0.Crit.1.Bad =
Step.3.4.1.0.Crit.2.Name = Position Comparison X
Step.3.4.1.0.Crit.2.Description:
Step.3.4.1.0.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.3.4.1.0.Crit.2.Warn =
Step.3.4.1.0.Crit.2.Bad =
Step.3.4.1.0.Crit.3.Name = Position Comparison Y
Step.3.4.1.0.Crit.3.Description:
Step.3.4.1.0.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.3.4.1.0.Crit.3.Warn =
Step.3.4.1.0.Crit.3.Bad =
Step.3.4.1.0.Crit.4.Name = Position Comparison YawRate
Step.3.4.1.0.Crit.4.Description:
Step.3.4.1.0.Crit.4.Good = [get AngposDelta] <= 0.175 && [get AngposDelta] >= -0.175
Step.3.4.1.0.Crit.4.Warn =
Step.3.4.1.0.Crit.4.Bad =
Step.3.4.1.0.Var.0.Name = Base
Step.3.4.1.0.Var.0.Param =
Step.3.4.1.0.Var.0.Result = good
Step.3.4.1.0.Var.0.ResDate = 1717609023
Step.3.4.1.0.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25
Step.3.4.1.0.Var.0.Crit.0.Name = Parking Slot
Step.3.4.1.0.Var.0.Crit.0.Result = good
Step.3.4.1.0.Var.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.3.4.1.0.Var.0.Crit.1.Result = good
Step.3.4.1.0.Var.0.Crit.2.Name = Position Comparison X
Step.3.4.1.0.Var.0.Crit.2.Result = good
Step.3.4.1.0.Var.0.Crit.3.Name = Position Comparison Y
Step.3.4.1.0.Var.0.Crit.3.Result = good
Step.3.4.1.0.Var.0.Crit.4.Name = Position Comparison YawRate
Step.3.4.1.0.Var.0.Crit.4.Result = good
Step.4 = Group
Step.4.Name = 03_Refining_Maneuver_MP
Step.4.0 = TestRun
Step.4.0.Name = AP/09_MP/MP_NoHMI_MP_2_Refine_translationRight
Step.4.0.Crit.0.Name = MP_Maneuver_complete_Confirmation
Step.4.0.Crit.0.Description:
Step.4.0.Crit.0.Good = [get MP_Finish] == 7
Step.4.0.Crit.0.Warn =
Step.4.0.Crit.0.Bad =
Step.4.0.Crit.1.Name = Position Comparison X
Step.4.0.Crit.1.Description:
Step.4.0.Crit.1.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1 && [get XposDeltaRef] <= 0.1 && [get XposDeltaRef] >= -0.1
Step.4.0.Crit.1.Warn =
Step.4.0.Crit.1.Bad =
Step.4.0.Crit.2.Name = Position Comparison Y
Step.4.0.Crit.2.Description:
Step.4.0.Crit.2.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1 && [get YposDeltaRef] <= 0.1 && [get YposDeltaRef] >= -0.1
Step.4.0.Crit.2.Warn =
Step.4.0.Crit.2.Bad =
Step.4.0.Crit.3.Name = Position Comparison Yaw
Step.4.0.Crit.3.Description:
Step.4.0.Crit.3.Good = [get AngposDelta] <= 0.1 && [get AngposDelta] >= -0.1 && [get AngposDeltaRef] <= 0.1 && [get AngposDeltaRef] >= -0.1
Step.4.0.Crit.3.Warn =
Step.4.0.Crit.3.Bad =
Step.4.0.Crit.4.Name = Parking Slot
Step.4.0.Crit.4.Description:
Step.4.0.Crit.4.Good =
Step.4.0.Crit.4.Warn =
Step.4.0.Crit.4.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.4.0.Var.0.Name = Base
Step.4.0.Var.0.Param =
Step.4.0.Var.0.Result = good
Step.4.0.Var.0.ResDate = 1717609856
Step.4.0.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33
Step.4.0.Var.0.Crit.0.Name = MP_Maneuver_complete_Confirmation
Step.4.0.Var.0.Crit.0.Result = good
Step.4.0.Var.0.Crit.1.Name = Position Comparison X
Step.4.0.Var.0.Crit.1.Result = good
Step.4.0.Var.0.Crit.2.Name = Position Comparison Y
Step.4.0.Var.0.Crit.2.Result = good
Step.4.0.Var.0.Crit.3.Name = Position Comparison Yaw
Step.4.0.Var.0.Crit.3.Result = good
Step.4.0.Var.0.Crit.4.Name = Parking Slot
Step.4.0.Var.0.Crit.4.Result = good
Step.4.1 = TestRun
Step.4.1.Name = AP/09_MP/MP_NoHMI_MP_2_Refine_translationUp
Step.4.1.Crit.0.Name = MP_Maneuver_complete_Confirmation
Step.4.1.Crit.0.Description:
Step.4.1.Crit.0.Good = [get MP_Finish] == 7
Step.4.1.Crit.0.Warn =
Step.4.1.Crit.0.Bad =
Step.4.1.Crit.1.Name = Position Comparison X
Step.4.1.Crit.1.Description:
Step.4.1.Crit.1.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1 && [get XposDeltaRef] <= 0.1 && [get XposDeltaRef] >= -0.1
Step.4.1.Crit.1.Warn =
Step.4.1.Crit.1.Bad =
Step.4.1.Crit.2.Name = Position Comparison Y
Step.4.1.Crit.2.Description:
Step.4.1.Crit.2.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1 && [get YposDeltaRef] <= 0.1 && [get YposDeltaRef] >= -0.1
Step.4.1.Crit.2.Warn =
Step.4.1.Crit.2.Bad =
Step.4.1.Crit.3.Name = Position Comparison Yaw
Step.4.1.Crit.3.Description:
Step.4.1.Crit.3.Good = [get AngposDelta] <= 0.1 && [get AngposDelta] >= -0.1 && [get AngposDeltaRef] <= 0.1 && [get AngposDeltaRef] >= -0.1
Step.4.1.Crit.3.Warn =
Step.4.1.Crit.3.Bad =
Step.4.1.Crit.4.Name = Parking Slot
Step.4.1.Crit.4.Description:
Step.4.1.Crit.4.Good =
Step.4.1.Crit.4.Warn =
Step.4.1.Crit.4.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.4.1.Var.0.Name = Base
Step.4.1.Var.0.Param =
Step.4.1.Var.0.Result = good
Step.4.1.Var.0.ResDate = 1717610020
Step.4.1.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33
Step.4.1.Var.0.Crit.0.Name = MP_Maneuver_complete_Confirmation
Step.4.1.Var.0.Crit.0.Result = good
Step.4.1.Var.0.Crit.1.Name = Position Comparison X
Step.4.1.Var.0.Crit.1.Result = good
Step.4.1.Var.0.Crit.2.Name = Position Comparison Y
Step.4.1.Var.0.Crit.2.Result = good
Step.4.1.Var.0.Crit.3.Name = Position Comparison Yaw
Step.4.1.Var.0.Crit.3.Result = good
Step.4.1.Var.0.Crit.4.Name = Parking Slot
Step.4.1.Var.0.Crit.4.Result = good
Step.4.2 = TestRun
Step.4.2.Name = AP/09_MP/MP_NoHMI_MP_2_Refine_RotationClkW
Step.4.2.Crit.0.Name = MP_Maneuver_complete_Confirmation
Step.4.2.Crit.0.Description:
Step.4.2.Crit.0.Good = [get MP_Finish] == 7
Step.4.2.Crit.0.Warn =
Step.4.2.Crit.0.Bad =
Step.4.2.Crit.1.Name = Position Comparison X
Step.4.2.Crit.1.Description:
Step.4.2.Crit.1.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1 && [get XposDeltaRef] <= 0.1 && [get XposDeltaRef] >= -0.1
Step.4.2.Crit.1.Warn =
Step.4.2.Crit.1.Bad =
Step.4.2.Crit.2.Name = Position Comparison Y
Step.4.2.Crit.2.Description:
Step.4.2.Crit.2.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1 && [get YposDeltaRef] <= 0.1 && [get YposDeltaRef] >= -0.1
Step.4.2.Crit.2.Warn =
Step.4.2.Crit.2.Bad =
Step.4.2.Crit.3.Name = Position Comparison Yaw
Step.4.2.Crit.3.Description:
Step.4.2.Crit.3.Good = [get AngposDelta] <= 0.1 && [get AngposDelta] >= -0.1 && [get AngposDeltaRef] <= 0.1 && [get AngposDeltaRef] >= -0.1
Step.4.2.Crit.3.Warn =
Step.4.2.Crit.3.Bad =
Step.4.2.Crit.4.Name = Parking Slot
Step.4.2.Crit.4.Description:
Step.4.2.Crit.4.Good =
Step.4.2.Crit.4.Warn =
Step.4.2.Crit.4.Bad = [get AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu] == 0
Step.4.2.Var.0.Name = Base
Step.4.2.Var.0.Param =
Step.4.2.Var.0.Result = good
Step.4.2.Var.0.ResDate = 1717610193
Step.4.2.Var.0.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33
Step.4.2.Var.0.Crit.0.Name = MP_Maneuver_complete_Confirmation
Step.4.2.Var.0.Crit.0.Result = good
Step.4.2.Var.0.Crit.1.Name = Position Comparison X
Step.4.2.Var.0.Crit.1.Result = good
Step.4.2.Var.0.Crit.2.Name = Position Comparison Y
Step.4.2.Var.0.Crit.2.Result = good
Step.4.2.Var.0.Crit.3.Name = Position Comparison Yaw
Step.4.2.Var.0.Crit.3.Result = good
Step.4.2.Var.0.Crit.4.Name = Parking Slot
Step.4.2.Var.0.Crit.4.Result = good
Step.5 = Group
Step.5.Name = 04_Implicit_MP_after_AUP (under dev til R5)
Step.5.0 = TestRun
Step.5.0.Name = AP/09_MP/MP_NoHMI_Implicit_Handover_after_AUP_rotation
Step.5.0.Crit.0.Name = Parking Slot
Step.5.0.Crit.0.Description:
Step.5.0.Crit.0.Good =
Step.5.0.Crit.0.Warn =
Step.5.0.Crit.0.Bad = [get AP.memoryParkingStatusPort.numStoredMemoryParkingSlots_nu] == 0
Step.5.0.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.5.0.Crit.1.Description:
Step.5.0.Crit.1.Good = [get AP.planningCtrlPort.mpStates] == 7
Step.5.0.Crit.1.Warn =
Step.5.0.Crit.1.Bad =
Step.5.0.Crit.2.Name = Position Comparison X
Step.5.0.Crit.2.Description:
Step.5.0.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.5.0.Crit.2.Warn =
Step.5.0.Crit.2.Bad =
Step.5.0.Crit.3.Name = Position Comparison Y
Step.5.0.Crit.3.Description:
Step.5.0.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.5.0.Crit.3.Warn =
Step.5.0.Crit.3.Bad =
Step.5.0.Crit.4.Name = Position Comparison Yaw
Step.5.0.Crit.4.Description:
Step.5.0.Crit.4.Good = [get AngposDelta] <= 0.1 && [get AngposDelta] >= -0.1
Step.5.0.Crit.4.Warn =
Step.5.0.Crit.4.Bad =
Step.5.0.Result = err
Step.5.0.ResDate = 1717613235
Step.5.0.ManLst = 0 1 2
Step.5.0.Log.0.Time = 10.957
Step.5.0.Log.0.Kind = err
Step.5.0.Log.0.Text = Simulation stopped by user
Step.5.1 = TestRun
Step.5.1.Name = AP/09_MP/MP_NoHMI_Implicit_Handover_after_AUP_translation
Step.5.1.Crit.0.Name = Parking Slot
Step.5.1.Crit.0.Description:
Step.5.1.Crit.0.Good =
Step.5.1.Crit.0.Warn =
Step.5.1.Crit.0.Bad = [get AP.memoryParkingStatusPort.numStoredMemoryParkingSlots_nu] == 0
Step.5.1.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.5.1.Crit.1.Description:
Step.5.1.Crit.1.Good = [get AP.planningCtrlPort.mpStates] == 7
Step.5.1.Crit.1.Warn =
Step.5.1.Crit.1.Bad =
Step.5.1.Crit.2.Name = Position Comparison X
Step.5.1.Crit.2.Description:
Step.5.1.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.5.1.Crit.2.Warn =
Step.5.1.Crit.2.Bad =
Step.5.1.Crit.3.Name = Position Comparison Y
Step.5.1.Crit.3.Description:
Step.5.1.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.5.1.Crit.3.Warn =
Step.5.1.Crit.3.Bad =
Step.5.1.Crit.4.Name = Position Comparison Yaw
Step.5.1.Crit.4.Description:
Step.5.1.Crit.4.Good = [get AngposDelta] <= 0.1 && [get AngposDelta] >= -0.1
Step.5.1.Crit.4.Warn =
Step.5.1.Crit.4.Bad =
Step.5.1.Result = err
Step.5.1.ResDate = 1717610873
Step.5.1.ManLst = 0 1 2
Step.5.1.Log.0.Time = 8.119
Step.5.1.Log.0.Kind = err
Step.5.1.Log.0.Text = Simulation stopped by user
Step.5.2 = TestRun
Step.5.2.Name = AP/09_MP/MP_NoHMI_Implicit_Handover_after_AUP_only_save
Step.5.2.Crit.0.Name = Parking Slot
Step.5.2.Crit.0.Description:
Step.5.2.Crit.0.Good =
Step.5.2.Crit.0.Warn =
Step.5.2.Crit.0.Bad = [get AP.memoryParkingStatusPort.numStoredMemoryParkingSlots_nu] == 0
Step.5.2.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.5.2.Crit.1.Description:
Step.5.2.Crit.1.Good = [get AP.planningCtrlPort.mpStates] == 7
Step.5.2.Crit.1.Warn =
Step.5.2.Crit.1.Bad =
Step.5.2.Crit.2.Name = Position Comparison X
Step.5.2.Crit.2.Description:
Step.5.2.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.5.2.Crit.2.Warn =
Step.5.2.Crit.2.Bad =
Step.5.2.Crit.3.Name = Position Comparison Y
Step.5.2.Crit.3.Description:
Step.5.2.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.5.2.Crit.3.Warn =
Step.5.2.Crit.3.Bad =
Step.5.2.Crit.4.Name = Position Comparison Yaw
Step.5.2.Crit.4.Description:
Step.5.2.Crit.4.Good = [get AngposDelta] <= 0.1 && [get AngposDelta] >= -0.1
Step.5.2.Crit.4.Warn =
Step.5.2.Crit.4.Bad =
Step.5.2.Result = good
Step.5.2.ResDate = 1717178864
Step.5.2.ResFiles = SimOutput/MP_NoHMI_Implicit_Handover_after_AUP_only_save.erg
Step.5.2.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18
Step.5.2.Crit.0.Result = good
Step.5.2.Crit.1.Result = good
Step.5.2.Crit.2.Result = good
Step.5.2.Crit.3.Result = good
Step.5.2.Crit.4.Result = good
Step.5.3 = TestRun
Step.5.3.Name = AP/09_MP/MP_NoHMI_Implicit_Handover_after_AUP_2
Step.5.3.Crit.0.Name = Parking Slot
Step.5.3.Crit.0.Description:
Step.5.3.Crit.0.Good =
Step.5.3.Crit.0.Warn =
Step.5.3.Crit.0.Bad = [get AP.memoryParkingStatusPort.numStoredMemoryParkingSlots_nu] == 0
Step.5.3.Crit.1.Name = MP_Maneuver_complete_Confirmation
Step.5.3.Crit.1.Description:
Step.5.3.Crit.1.Good = [get AP.planningCtrlPort.mpStates] == 7
Step.5.3.Crit.1.Warn =
Step.5.3.Crit.1.Bad =
Step.5.3.Crit.2.Name = Position Comparison X
Step.5.3.Crit.2.Description:
Step.5.3.Crit.2.Good = [get XposDelta] <= 0.1 && [get XposDelta] >= -0.1
Step.5.3.Crit.2.Warn =
Step.5.3.Crit.2.Bad =
Step.5.3.Crit.3.Name = Position Comparison Y
Step.5.3.Crit.3.Description:
Step.5.3.Crit.3.Good = [get YposDelta] <= 0.1 && [get YposDelta] >= -0.1
Step.5.3.Crit.3.Warn =
Step.5.3.Crit.3.Bad =
Step.5.3.Crit.4.Name = Position Comparison Yaw
Step.5.3.Crit.4.Description:
Step.5.3.Crit.4.Good = [get AngposDelta] <= 0.1 && [get AngposDelta] >= -0.1
Step.5.3.Crit.4.Warn =
Step.5.3.Crit.4.Bad =
Step.5.3.Result = good
Step.5.3.ResDate = 1717178992
Step.5.3.ResFiles = SimOutput/MP_NoHMI_Implicit_Handover_after_AUP_2.erg
Step.5.3.ManLst = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21
Step.5.3.Crit.0.Result = good
Step.5.3.Crit.1.Result = good
Step.5.3.Crit.2.Result = good
Step.5.3.Crit.3.Result = good
Step.5.3.Crit.4.Result = good
