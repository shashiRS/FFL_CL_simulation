import datetime
from cmdata_common import cDynamicProperty, cSimpleDataType, cSimple3Dfloat, cSimple6Dfloat
from cmdata_sensors import cCMSensors

### VEHICLE CLASS ###
class cCMVehicle(cDynamicProperty):
    def __init__(self, srcVehicleFile=None, specialPowertrainAttributes=False):
        self.Sensors = cCMSensors(srcVehicleFile)
        ################################
        self._srcFile = "Examples/DemoCar"
        self._Attributes = []
        self._Attributes.append(('InfoLine',                            cSimpleDataType("#INFOFILE", str, "1.1 - Do not remove this line!", splitter="")))
        self._Attributes.append(('FileIdent',                           cSimpleDataType("FileIdent", str, "CarMaker-Car 5")))
        self._Attributes.append(('FileCreator',                         cSimpleDataType("FileCreator", str, "CarMaker 5.1.3 " + str(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")) + " TEMPLATE")))
        self._Attributes.append(('Description',                         cSimpleDataType("Description", str, "\tDemo Passenger Car\n\tFront Drive\n\n", splitter=":\n")))
        self._Attributes.append(('General.headline',                    cSimpleDataType("## General ##", str, "#############################################################", splitter="")))        
        self._Attributes.append(('Picture.PicFName',                    cSimpleDataType("Picture.PicFName", str, "VW_NewBeetle_2005.png")))
        self._Attributes.append(('Movie.Skin.FName',                    cSimpleDataType("Movie.Skin.FName", str, "VW_NewBeetle_2005.obj")))
        self._Attributes.append(('Hitch.System',                        cSimpleDataType("Hitch.System", str, "Ball")))
        self._Attributes.append(('Vehicle.OuterSkin',                   cSimpleDataType("Vehicle.OuterSkin", str, "-0.1 0.85 0.2 4.05 -0.85 1.6")))
        self._Attributes.append(('Vehicle.OuterSkin',  cSimple6Dfloat("Vehicle.OuterSkin", "RearLowerLeftPoint", [-0.1, 0.85, 0.2], "FrontUpperRightPoint", [4.05, -0.85, 1.6])))
        self._Attributes.append(('RefPointInputSystem',                 cSimple3Dfloat("RefPointInputSystem", [0.0, 0.0, 0.0])))
        self._Attributes.append(('VehicleControl.Kind',                 cSimpleDataType("VehicleControl.Kind", str)))
        self._Attributes.append(('Misc.Comment',                        cSimpleDataType("Misc.Comment", str, "\n", splitter=":\n")))
        self._Attributes.append(('VehicleModel.headline',              cSimpleDataType("## Vehicle Model ##", str, "#######################################################", splitter="")))        
        self._Attributes.append(('VehicleModel.VhclMdl',                cSimpleDataType("VehicleModel", str, "Vhcl_2Axle")))
        self._Attributes.append(('VehicleModel.Kind',                   cSimpleDataType("VehicleModel.Kind", str, "RigidBody")))
        self._Attributes.append(('VehicleModel.Mode',                   cSimpleDataType("VehicleModel.Mode", str, "BodyA")))
        self._Attributes.append(('nAxle',                               cSimpleDataType("nAxle", int, 2, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('Body.mass',                           cSimpleDataType("Body.mass", float, 1301.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('Body.I',              cSimple6Dfloat("Body.I", "IMoment", [360.0, 1800.0, 1800.0], "IProduct", [-0.0, -0.0, -0.0])))
        self._Attributes.append(('Body.pos',                            cSimple3Dfloat("Body.pos", [2.15, 0.0, 0.58])))
        self._Attributes.append(('Flex.JointFr1Fr1B.pos',               cSimple3Dfloat("Flex.JointFr1Fr1B.pos", [2.15, 0.0, 0.58])))
        self._Attributes.append(('Flex.JointFr1Fr1B.Kind',              cSimpleDataType("Flex.JointFr1Fr1B.Kind", str, "Coeff")))
        self._Attributes.append(('Flex.JointFr1Fr1B.k.x.val',           cSimpleDataType("Flex.JointFr1Fr1B.k.x", float, 5000.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('Flex.JointFr1Fr1B.k.y.val',           cSimpleDataType("Flex.JointFr1Fr1B.k.y", float, 15000.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('Flex.JointFr1Fr1B.k.x.Amplify',       cSimpleDataType("Flex.JointFr1Fr1B.k.x.Amplify", float, 1.0)))
        self._Attributes.append(('Flex.JointFr1Fr1B.k.y.Amplify',       cSimpleDataType("Flex.JointFr1Fr1B.k.y.Amplify", float, 1.0)))
        self._Attributes.append(('Flex.JointFr1Fr1B.d.x.val',           cSimpleDataType("Flex.JointFr1Fr1B.d.x", float, 100.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('Flex.JointFr1Fr1B.d.y.val',           cSimpleDataType("Flex.JointFr1Fr1B.d.y", float, 100.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('Flex.JointFr1Fr1B.d.x.Amplify',       cSimpleDataType("Flex.JointFr1Fr1B.d.x.Amplify", float, 1.0)))
        self._Attributes.append(('Flex.JointFr1Fr1B.d.y.Amplify',       cSimpleDataType("Flex.JointFr1Fr1B.d.y.Amplify", float, 1.0)))
        self._Attributes.append(('Model.Comment',                       cSimpleDataType("Model.Comment", str, "\n", splitter=":\n")))
        ### ATTENTION: Important for later use. DO NOT DELETE !!!        
        self._Attributes.append(('MassInertia.headline',               cSimpleDataType("## Mass Inertia Geometry ##", str, "###############################################", splitter="")))        
        self._Attributes.append(('WheelCarrier.fl.mass',                cSimpleDataType("WheelCarrier.fl.mass", float, 18.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('WheelCarrier.fl.I',                   cSimpleDataType("WheelCarrier.fl.I", str, "0.2 0.2 0.2 -0.0 -0.0 -0.0")))
        self._Attributes.append(('WheelCarrier.fl.pos',                 cSimple3Dfloat("WheelCarrier.fl.pos", [3.258, 0.754, 0.298])))
        self._Attributes.append(('WheelCarrier.fr.mass',                cSimpleDataType("WheelCarrier.fr.mass", float, 18.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('WheelCarrier.fr.I',                   cSimpleDataType("WheelCarrier.fr.I", str, "0.2 0.2 0.2 -0.0 -0.0 -0.0")))
        self._Attributes.append(('WheelCarrier.fr.pos',                 cSimple3Dfloat("WheelCarrier.fr.pos", [3.258, -0.754, 0.298])))
        self._Attributes.append(('WheelCarrier.rl.mass',                cSimpleDataType("WheelCarrier.rl.mass", float, 13.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('WheelCarrier.rl.I',                   cSimpleDataType("WheelCarrier.rl.I", str, "0.1 0.1 0.1 -0.0 -0.0 -0.0")))
        self._Attributes.append(('WheelCarrier.rl.pos',                 cSimple3Dfloat("WheelCarrier.rl.pos", [0.73, 0.747, 0.293])))
        self._Attributes.append(('WheelCarrier.rr.mass',                cSimpleDataType("WheelCarrier.rr.mass", float, 13.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('WheelCarrier.rr.I',                   cSimpleDataType("WheelCarrier.rr.I", str, "0.1 0.1 0.1 -0.0 -0.0 -0.0")))
        self._Attributes.append(('WheelCarrier.rr.pos',                 cSimple3Dfloat("WheelCarrier.rr.pos", [0.73, -0.747, 0.293])))
        self._Attributes.append(('Wheel.fl.mass',                       cSimpleDataType("Wheel.fl.mass", float, 25.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('Wheel.fl.I',                          cSimple3Dfloat("Wheel.fl.I", [0.4, 1.2, 0.4])))
        self._Attributes.append(('Wheel.fl.pos',                        cSimple3Dfloat("Wheel.fl.pos", [3.258, 0.754, 0.298])))
        self._Attributes.append(('Wheel.fr.mass',                       cSimpleDataType("Wheel.fr.mass", float, 25.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('Wheel.fr.I',                          cSimple3Dfloat("Wheel.fr.I", [0.4, 1.2, 0.4])))
        self._Attributes.append(('Wheel.fr.pos',                        cSimple3Dfloat("Wheel.fr.pos", [3.258, -0.754, 0.298])))
        self._Attributes.append(('Wheel.rl.mass',                       cSimpleDataType("Wheel.rl.mass", float, 25.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('Wheel.rl.I',                          cSimple3Dfloat("Wheel.rl.I", [0.45, 0.7, 0.45])))
        self._Attributes.append(('Wheel.rl.pos',                        cSimple3Dfloat("Wheel.rl.pos", [0.73, 0.747, 0.293])))
        self._Attributes.append(('Wheel.rr.mass',                       cSimpleDataType("Wheel.rr.mass", float, 25.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('Wheel.rr.I',                          cSimple3Dfloat("Wheel.rr.I", [0.45, 0.7, 0.45])))
        self._Attributes.append(('Wheel.rr.pos',                        cSimple3Dfloat("Wheel.rr.pos", [0.73, -0.747, 0.293])))
        self._Attributes.append(('Aero.Marker.pos',                     cSimple3Dfloat("Aero.Marker.pos", [3.65, 0.0, 0.6])))
        self._Attributes.append(('Hitch.pos',                           cSimple3Dfloat("Hitch.pos", [0.0, 0.0, 0.4])))
        self._Attributes.append(('Jack.fl.pos',                         cSimple3Dfloat("Jack.fl.pos", [2.90, 0.754, 0.298])))
        self._Attributes.append(('Jack.fr.pos',                         cSimple3Dfloat("Jack.fr.pos", [2.90, -0.754, 0.298])))
        self._Attributes.append(('Jack.rl.pos',                         cSimple3Dfloat("Jack.rl.pos", [1.10, 0.747, 0.293])))
        self._Attributes.append(('Jack.rr.pos',                         cSimple3Dfloat("Jack.rr.pos", [1.10, -0.747, 0.293])))
        self._Attributes.append(('Mass.Comment',                        cSimpleDataType("Mass.Comment", str, "\n", splitter=":\n")))
        ### ATTENTION: Important for later use. DO NOT DELETE !!!
        # workaround to be able to load Mass Inertia Geometry from TestRun file:
#        self._Attributes.append(('MassInertia',                         cSimpleDataType("## Mass Inertia Geometry ##", str, """###############################################
#WheelCarrier.fl.mass = 18.0
#WheelCarrier.fl.I = 0.2 0.2 0.2 -0.0 -0.0 -0.0
#WheelCarrier.fl.pos = 3.258 0.754 0.298
#WheelCarrier.fr.mass = 18.0
#WheelCarrier.fr.I = 0.2 0.2 0.2 -0.0 -0.0 -0.0
#WheelCarrier.fr.pos = 3.258 -0.754 0.298
#WheelCarrier.rl.mass = 13.0
#WheelCarrier.rl.I = 0.1 0.1 0.1 -0.0 -0.0 -0.0
#WheelCarrier.rl.pos = 0.73 0.747 0.293
#WheelCarrier.rr.mass = 13.0
#WheelCarrier.rr.I = 0.1 0.1 0.1 -0.0 -0.0 -0.0
#WheelCarrier.rr.pos = 0.73 -0.747 0.293
#Wheel.fl.mass = 25.0
#Wheel.fl.I = 0.4 1.2 0.4
#Wheel.fl.pos = 3.258 0.754 0.298
#Wheel.fr.mass = 25.0
#Wheel.fr.I = 0.4 1.2 0.4
#Wheel.fr.pos = 3.258 -0.754 0.298
#Wheel.rl.mass = 25.0
#Wheel.rl.I = 0.45 0.7 0.45
#Wheel.rl.pos = 0.73 0.747 0.293
#Wheel.rr.mass = 25.0
#Wheel.rr.I = 0.45 0.7 0.45
#Wheel.rr.pos = 0.73 -0.747 0.293
#Aero.Marker.pos = 3.65 0.0 0.6
#Hitch.pos = 0.0 0.0 0.4
#Jack.fl.pos = 2.90 0.754 0.298
#Jack.fr.pos = 2.90 -0.754 0.298
#Jack.rl.pos = 1.10 0.747 0.293
#Jack.rr.pos = 1.10 -0.747 0.293
#Mass.Comment:
#
#""", splitter = "")))
        self._Attributes.append(('Eng.headline',                        cSimpleDataType("## Eng ##", str, "#################################################################", splitter="")))
        self._Attributes.append(('Eng.Active',                          cSimpleDataType("Eng.Active", int, 0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('Eng.Comment',                         cSimpleDataType("Eng.Comment", str, "\n", splitter=":\n")))
        self._Attributes.append(('Tires.headline',                      cSimpleDataType("## Tires ##", str, "###############################################################", splitter="")))
        self._Attributes.append(('Tire.fl',                             cSimpleDataType("Tire.0", str, "Examples/RT_195_65R15")))
        self._Attributes.append(('Tire.fr',                             cSimpleDataType("Tire.1", str, "Examples/RT_195_65R15")))
        self._Attributes.append(('Tire.rl',                             cSimpleDataType("Tire.2", str, "Examples/RT_195_65R15")))
        self._Attributes.append(('Tire.rr',                             cSimpleDataType("Tire.3", str, "Examples/RT_195_65R15")))
        self._Attributes.append(('AxleR.TwinTiresOn',                   cSimpleDataType("AxleR.TwinTiresOn", int, 0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('Tires.Comment',                       cSimpleDataType("Tires.Comment", str, "\n", splitter=":\n")))
        self._Attributes.append(('Brake.headline',                      cSimpleDataType("## Brake ##", str, "###############################################################", splitter="")))
        self._Attributes.append(('Brake.Kind',                          cSimpleDataType("Brake.Kind", str, "PresDistrib 1")))
        self._Attributes.append(('Brake.pMC_based_on',                  cSimpleDataType("Brake.pMC_based_on", str, "PedalAct")))
        self._Attributes.append(('Brake.Pedal2PedalFrc',                cSimpleDataType("Brake.Pedal2PedalFrc", float, 300.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('Brake.PedalAct2pMC',                  cSimpleDataType("Brake.PedalAct2pMC", float, 150.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('Brake.PedalForce2pMC',                cSimpleDataType("Brake.PedalForce2pMC", float, 0.5, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('Brake.tResp',                         cSimpleDataType("Brake.tResp", float, 0.005, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('Brake.tBuildUp',                      cSimpleDataType("Brake.tBuildUp", float, 0.08, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('Brake.pWB2Trq',                       cSimpleDataType("Brake.pWB2Trq", str, "10.0 10.0 5.0 5.0")))
        self._Attributes.append(('Brake.Park.Torque_max',               cSimpleDataType("Brake.Park.Torque_max", str, "0 0 1000 1000")))
        self._Attributes.append(('Brake.Torque.Amplify',                cSimpleDataType("Brake.Torque.Amplify", str, "1.0 1.0 1.0 1.0")))
        self._Attributes.append(('Brake.Comment',                       cSimpleDataType("Brake.Comment", str, "\n", splitter=":\n")))
        ### ATTENTION: Important for later use. DO NOT DELETE !!!
        self._Attributes.append(('PowerTrain.headline',                 cSimpleDataType("## Powertrain ##", str, "#########################################################", splitter="")))
        self._Attributes.append(('PowerTrain.Kind',                     cSimpleDataType("PowerTrain.Kind", str, "Generic 1")))
        self._Attributes.append(('PowerTrain.DL.Kind',                  cSimpleDataType("PowerTrain.DL.Kind", str, "GenFront 1")))
        self._Attributes.append(('PowerTrain.Engine.I',                 cSimpleDataType("PowerTrain.Engine.I", float, 0.07, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.Engine.Orientation',       cSimpleDataType("PowerTrain.Engine.Orientation", str, "Trans")))
        self._Attributes.append(('PowerTrain.ET.Kind',                  cSimpleDataType("PowerTrain.ET.Kind", str, "Mapping 1")))
        self._Attributes.append(('PowerTrain.ET.tBuildUp',              cSimpleDataType("PowerTrain.ET.tBuildUp", float, 0.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.ET.Mapping.Kind',          cSimpleDataType("PowerTrain.ET.Mapping.Kind", str, "DragFullLoad")))
        self._Attributes.append(('PowerTrain.ET.Starter.Trq',           cSimpleDataType("PowerTrain.ET.Starter.Trq", float, 150.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.ET.Starter.rotvOff',       cSimpleDataType("PowerTrain.ET.Starter.rotvOff", float, 800.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.ET.TrqKl15Off',            cSimpleDataType("PowerTrain.ET.TrqKl15Off", float, -80.0)))
        self._Attributes.append(('PowerTrain.ET.ISCtrl.Active',         cSimpleDataType("PowerTrain.ET.ISCtrl.Active", int, 1, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.ET.nIdle',                 cSimpleDataType("PowerTrain.ET.nIdle", float, 800.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.ET.Exponent',              cSimpleDataType("PowerTrain.ET.Exponent", float, 0.8, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.ET.DragPower.Amplify',     cSimpleDataType("PowerTrain.ET.DragPower.Amplify", float, 1.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.ET.DragPower.val',         cSimpleDataType("PowerTrain.ET.DragPower", str, "\t500.0 -10.0\n\t1000.0 -10.0\n\t2000.0 -20.0\n\t3000.0 -30.0\n\t4000.0 -40.0\n\t5000.0 -50.0\n\t6000.0 -60.0\n\t7000.0 -70.0\n\t8000.0 -80.0", splitter=":\n")))
        self._Attributes.append(('PowerTrain.ET.FullLoadPower.Amplify', cSimpleDataType("PowerTrain.ET.FullLoadPower.Amplify", float, 1.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.ET.FullLoadPower.val',     cSimpleDataType("PowerTrain.ET.FullLoadPower", str, "\t500.0 10.0\n\t1000.0 140.0\n\t2000.0 155.0\n\t3000.0 165.0\n\t4000.0 180.0\n\t4500.0 185.0\n\t5000.0 182.0\n\t6000.0 168.0\n\t7000.0 130.0\n\t8000.0 0.0", splitter=":\n")))
        self._Attributes.append(('PowerTrain.ET.FuelConsume',           cSimpleDataType("PowerTrain.ET.FuelConsume", float, 0.0)))
        self._Attributes.append(('PowerTrain.Clutch.Kind',              cSimpleDataType("PowerTrain.Clutch.Kind", str, "Manual 1")))
        self._Attributes.append(('PowerTrain.Clutch.I_in',              cSimpleDataType("PowerTrain.Clutch.I_in", float, 0.01, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.Clutch.I_out',             cSimpleDataType("PowerTrain.Clutch.I_out", float, 0.01, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.Clutch.ConnectPos',        cSimpleDataType("PowerTrain.Clutch.ConnectPos", float, 0.3, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.Clutch.DisconnectPos',     cSimpleDataType("PowerTrain.Clutch.DisconnectPos", float, 0.7, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.Clutch.Trq_max',           cSimpleDataType("PowerTrain.Clutch.Trq_max", float, 300.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.Clutch.slope',             cSimpleDataType("PowerTrain.Clutch.slope", float, 30.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.Clutch.c',                 cSimpleDataType("PowerTrain.Clutch.c", float, 1500.0, valueChecker=lambda x: x>=0)))
        if specialPowertrainAttributes:
            self._Attributes.append(('PowerTrain.Clutch.Adjust',               cSimpleDataType("PowerTrain.Clutch.Adjust", float, 1.0)))
            self._Attributes.append(('PowerTrain.Clutch.k_E',               cSimpleDataType("PowerTrain.Clutch.k_E", str, "", splitter=":\n")))
            self._Attributes.append(('PowerTrain.Clutch.mue',               cSimpleDataType("PowerTrain.Clutch.mue", str, "", splitter=":\n")))
        self._Attributes.append(('PowerTrain.GearBox.Kind',             cSimpleDataType("PowerTrain.GearBox.Kind", str, "Manual 1")))
        self._Attributes.append(('PowerTrain.GearBox.I_in',             cSimpleDataType("PowerTrain.GearBox.I_in", float, 1e-5, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.GearBox.I_out',            cSimpleDataType("PowerTrain.GearBox.I_out", float, 0.016, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.GearBox.iForwardGears',    cSimpleDataType("PowerTrain.GearBox.iForwardGears", str, "3.4 1.9 1.35 1.05 0.8")))
        self._Attributes.append(('PowerTrain.GearBox.iBackwardGears',   cSimpleDataType("PowerTrain.GearBox.iBackwardGears", float, -4.0, valueChecker=lambda x: x<=0)))
        self._Attributes.append(('PowerTrain.GearBox.EtaForwardGears',  cSimpleDataType("PowerTrain.GearBox.EtaForwardGears", str, "1.0 1.0 1.0 1.0 1.0")))
        self._Attributes.append(('PowerTrain.GearBox.EtaBackwardGears', cSimpleDataType("PowerTrain.GearBox.EtaBackwardGears", float, 1.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.GearBox.nFit',             cSimpleDataType("PowerTrain.GearBox.nFit", float, 50.0, valueChecker=lambda x: x>=0)))
        if specialPowertrainAttributes:
            self._Attributes.append(('PowerTrain.GearBox.Control.Kind',     cSimpleDataType("PowerTrain.GearBox.Control.Kind", str, "")))
            self._Attributes.append(('PowerTrain.GearBox.Control.RotvRef',  cSimpleDataType("PowerTrain.GearBox.Control.RotvRef", str, "")))
            self._Attributes.append(('PowerTrain.GearBox.Control.nShift',   cSimpleDataType("PowerTrain.GearBox.Control.nShift", str, "", splitter=":\n")))
            self._Attributes.append(('PowerTrain.GearBox.ManShift.SpeedRange',cSimpleDataType("PowerTrain.GearBox.ManShift.SpeedRange", str, "")))
        self._Attributes.append(('PowerTrain.DL.FlexShaft',             cSimpleDataType("PowerTrain.DL.FlexShaft", int, 0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.DL.FDiff.i',               cSimpleDataType("PowerTrain.DL.FDiff.i", float, 4.1, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.DL.FDiff.I_in',            cSimpleDataType("PowerTrain.DL.FDiff.I_in", float, 0.001, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.DL.FDiff.I_out',           cSimpleDataType("PowerTrain.DL.FDiff.I_out", float, 0.001, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.DL.FDiff.I_Cage',          cSimpleDataType("PowerTrain.DL.FDiff.I_Cage", float, 0.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.DL.FDiff.Cpl.Kind',        cSimpleDataType("PowerTrain.DL.FDiff.Cpl.Kind", str, "Visco 1")))
        self._Attributes.append(('PowerTrain.DL.FDiff.Cpl.Mounting',    cSimpleDataType("PowerTrain.DL.FDiff.Cpl.Mounting", str, "left2right")))
        self._Attributes.append(('PowerTrain.DL.FDiff.Cpl.k',           cSimpleDataType("PowerTrain.DL.FDiff.Cpl.k", float, 10.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.DL.FDiff.Cpl.Trq_Amplify', cSimpleDataType("PowerTrain.DL.FDiff.Cpl.Trq_Amplify", float, 1.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.DL.FDiff.Cpl.Trq_drotv',   cSimpleDataType("PowerTrain.DL.FDiff.Cpl.Trq_drotv", str, "\t0 0\n\t15 150\n\t30 250", splitter=":\n")))
        self._Attributes.append(('PowerTrain.DL.FDiff.Eta',             cSimpleDataType("PowerTrain.DL.FDiff.Eta", float, 1.0, valueChecker=lambda x: x>=0)))
        if specialPowertrainAttributes:
            self._Attributes.append(('PowerTrain.DL.RDiff.i',               cSimpleDataType("PowerTrain.DL.RDiff.i", float, 0.0, valueChecker=lambda x: x>=0)))
            self._Attributes.append(('PowerTrain.DL.RDiff.I_in',            cSimpleDataType("PowerTrain.DL.RDiff.I_in", float, 0.0, valueChecker=lambda x: x>=0)))
            self._Attributes.append(('PowerTrain.DL.RDiff.I_out',           cSimpleDataType("PowerTrain.DL.RDiff.I_out", float, 0.0, valueChecker=lambda x: x>=0)))
            self._Attributes.append(('PowerTrain.DL.RDiff.I_Cage',          cSimpleDataType("PowerTrain.DL.RDiff.I_Cage", float, 0.0, valueChecker=lambda x: x>=0)))
            self._Attributes.append(('PowerTrain.DL.RDiff.Cpl.Kind',        cSimpleDataType("PowerTrain.DL.RDiff.Cpl.Kind", str, "Visco 1")))
            self._Attributes.append(('PowerTrain.DL.RDiff.Eta',             cSimpleDataType("PowerTrain.DL.RDiff.Eta", float, 0.0, valueChecker=lambda x: x>=0)))
            self._Attributes.append(('PowerTrain.DL.CDiff.i_in2cent',       cSimpleDataType("PowerTrain.DL.CDiff.i_in2cent", float, 0.0, valueChecker=lambda x: x>=0)))
            self._Attributes.append(('PowerTrain.DL.CDiff.I_in',            cSimpleDataType("PowerTrain.DL.CDiff.I_in", float, 0.0, valueChecker=lambda x: x>=0)))
            self._Attributes.append(('PowerTrain.DL.CDiff.I_out_front',     cSimpleDataType("PowerTrain.DL.CDiff.I_out_front", float, 0.0, valueChecker=lambda x: x>=0)))
            self._Attributes.append(('PowerTrain.DL.CDiff.I_out_rear',      cSimpleDataType("PowerTrain.DL.CDiff.I_out_rear", float, 0.0, valueChecker=lambda x: x>=0)))
            self._Attributes.append(('PowerTrain.DL.CDiff.I_out_Cage',      cSimpleDataType("PowerTrain.DL.CDiff.I_out_Cage", float, 0.0, valueChecker=lambda x: x>=0)))
            self._Attributes.append(('PowerTrain.DL.CDiff.TrqRatio_front',  cSimpleDataType("PowerTrain.DL.CDiff.TrqRatio_front", float, 0.0, valueChecker=lambda x: x>=0)))
            self._Attributes.append(('PowerTrain.DL.CDiff.Cpl.Kind',        cSimpleDataType("PowerTrain.DL.CDiff.Cpl.Kind", str, "Visco 1")))
            self._Attributes.append(('PowerTrain.DL.CDiff.Eta',             cSimpleDataType("PowerTrain.DL.CDiff.Eta", float, 1.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.DL.TrqExt2DiffOn',         cSimpleDataType("PowerTrain.DL.TrqExt2DiffOn", int, 0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.DL.TrqExt2WheelOn',        cSimpleDataType("PowerTrain.DL.TrqExt2WheelOn", int, 0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.OSRate',                   cSimpleDataType("PowerTrain.OSRate", float, 5.0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('PowerTrain.Comment',                  cSimpleDataType("PowerTrain.Comment", str, "\n", splitter=":\n")))
        ### ATTENTION: Important for later use. DO NOT DELETE !!!
        # workaround to be able to load PowerTrain from TestRun file:
#        self._Attributes.append(('Powertrain',                         cSimpleDataType("## Powertrain ##", str, """#########################################################
#PowerTrain.Kind = Generic 1
#PowerTrain.DL.Kind = GenFront 1
#PowerTrain.Engine.I = 0.07
#PowerTrain.Engine.Orientation = Trans
#PowerTrain.ET.Kind = Mapping 1
#PowerTrain.ET.tBuildUp = 0.0
#PowerTrain.ET.Mapping.Kind = DragFullLoad
#PowerTrain.ET.Starter.Trq = 150.0
#PowerTrain.ET.Starter.rotvOff = 800
#PowerTrain.ET.TrqKl15Off = -80.0
#PowerTrain.ET.ISCtrl.Active = 1
#PowerTrain.ET.nIdle = 800
#PowerTrain.ET.Exponent = 0.8
#PowerTrain.ET.DragPower.Amplify = 1.0
#PowerTrain.ET.DragPower:
#\t500.0 -10.0
#\t1000.0 -10.0
#\t2000.0 -20.0
#\t3000.0 -30.0
#\t4000.0 -40.0
#\t5000.0 -50.0
#\t6000.0 -60.0
#\t7000.0 -70.0
#\t8000.0 -80.0
#PowerTrain.ET.FullLoadPower.Amplify = 1.0
#PowerTrain.ET.FullLoadPower:
#\t500.0 10.0
#\t1000.0 140.0
#\t2000.0 155.0
#\t3000.0 165.0
#\t4000.0 180.0
#\t4500.0 185.0
#\t5000.0 182.0
#\t6000.0 168.0
#\t7000.0 130.0
#\t8000.0 0.0
#PowerTrain.ET.FuelConsume = 0
#PowerTrain.Clutch.Kind = Manual 1
#PowerTrain.Clutch.I_in = 0.01
#PowerTrain.Clutch.I_out = 0.01
#PowerTrain.Clutch.ConnectPos = 0.3
#PowerTrain.Clutch.DisconnectPos = 0.7
#PowerTrain.Clutch.Trq_max = 300
#PowerTrain.Clutch.slope = 30.0
#PowerTrain.Clutch.c = 1500.0
#
#PowerTrain.GearBox.Kind = Manual 1
#PowerTrain.GearBox.I_in = 1e-5
#PowerTrain.GearBox.I_out = 0.016
#PowerTrain.GearBox.iForwardGears = 3.4 1.9 1.35 1.05 0.8
#PowerTrain.GearBox.iBackwardGears = -4.0
#PowerTrain.GearBox.EtaForwardGears = 1.0 1.0 1.0 1.0 1.0
#PowerTrain.GearBox.EtaBackwardGears = 1.0
#PowerTrain.GearBox.nFit = 50
#
#PowerTrain.DL.FlexShaft = 0
#PowerTrain.DL.FDiff.i = 4.1
#PowerTrain.DL.FDiff.I_in = 0.001
#PowerTrain.DL.FDiff.I_out = 0.001
#PowerTrain.DL.FDiff.I_Cage = 0.0
#PowerTrain.DL.FDiff.Cpl.Kind = Visco 1
#PowerTrain.DL.FDiff.Cpl.Mounting = left2right
#PowerTrain.DL.FDiff.Cpl.k = 10.0
#PowerTrain.DL.FDiff.Cpl.Trq_Amplify = 1.0
#PowerTrain.DL.FDiff.Cpl.Trq_drotv:
#\t0 0
#\t15 150
#\t30 250
#PowerTrain.DL.FDiff.Eta = 1.0
#PowerTrain.DL.TrqExt2DiffOn = 0
#PowerTrain.DL.TrqExt2WheelOn = 0
#PowerTrain.OSRate = 5
#PowerTrain.Comment:
#
#""", splitter="")))        
        self._Attributes.append(('Suspensions',                         cSimpleDataType("## Suspensions ##", str, """#########################################################
SuspF.Spring.Amplify = 1.0
SuspF.Spring.l0 = 0.338
SuspF.Spring:
\t-0.01 -240.0
\t0.0 0.0
\t1.0 24000.0
SuspF.SecSpring.Amplify = 1.0
SuspF.Damp_Push.Amplify = 2.0
SuspF.Damp_Push:
\t0.00 0
\t0.05 100
\t0.13 150
\t0.26 250
\t0.39 330
\t0.52 420
\t1.04 800
SuspF.Damp_Pull.Amplify = 2.0
SuspF.Damp_Pull:
\t0.00 0
\t0.05 110
\t0.13 350
\t0.26 650
\t0.39 725
\t0.52 800
\t1.04 1150
SuspF.Buf_Push.tz0 = 0.055
SuspF.Buf_Push.Amplify = 1
SuspF.Buf_Push:
\t0.0 0.0
\t0.002 32.0
\t0.004 88.0
\t0.006 167.0
\t0.008 269.0
\t0.010 393.0
\t0.012 596.0
\t0.015 1085.0
\t0.017 2098.0
\t0.019 6559.0
\t0.021 16527.0
\t0.023 31862.0
\t0.025 52427.0
\t0.027 78083.0
\t0.030 108691.0
SuspF.Buf_Pull.tz0 = -0.09
SuspF.Buf_Pull.Amplify = 1
SuspF.Buf_Pull:
\t0.0 0.0
\t0.002 32.0
\t0.004 88.0
\t0.006 167.0
\t0.008 269.0
\t0.010 393.0
\t0.012 596.0
\t0.015 1085.0
\t0.017 2098.0
\t0.019 6559.0
\t0.021 16527.0
\t0.023 31862.0
\t0.025 52427.0
\t0.027 78083.0
\t0.030 108691.0
SuspF.Stabi.Amplify = 1.0
SuspF.Stabi = 15000.0
SuspF.Stabi.Unit = N/m
SuspF.Kin.N = 1
SuspF.Kin.0.Kind = Linear2D 1
SuspF.Kin.0.ValidSide = left+right
SuspF.Kin.0.InputSide = left
SuspF.Kin.0.L.tx = 0.0 0.0 0.0
SuspF.Kin.0.L.ty = 0.0 0.0 0.0
SuspF.Kin.0.L.tz = 0.0 1.0 0.0
SuspF.Kin.0.L.rx = 0.0 0.0 0.0
SuspF.Kin.0.L.ry = 0.0 0.0 0.0
SuspF.Kin.0.L.rz = 0.0 0.0 5.0
SuspF.Kin.0.L.lSpring = 0.0 -1.0 0.0
SuspF.Kin.0.L.lDamp = 0.0 -1.0 0.0
SuspF.Kin.0.L.lBuf = 0.0 -1.0 0.0
SuspF.Kin.0.L.lStabi = 0.0 1.0 0.0
SuspF.Com.N = 0
SuspF.WhlBearing.On = 0
SuspR.Spring.Amplify = 1
SuspR.Spring.l0 = 0.112
SuspR.Spring:
\t-0.01 -250.0
\t0.0 0.0
\t1.0 25000.0
SuspR.SecSpring.Amplify = 1.0
SuspR.Damp_Push.Amplify = 7.0
SuspR.Damp_Push:
\t0.00 0
\t0.06 70
\t0.12 110
\t0.24 140
\t0.48 180
\t0.96 230
SuspR.Damp_Pull.Amplify = 10.0
SuspR.Damp_Pull:
\t0.00 0
\t0.06 70
\t0.12 140
\t0.24 310
\t0.48 390
\t0.96 550
SuspR.Buf_Push.tz0 = 0.08
SuspR.Buf_Push.Amplify = 1
SuspR.Buf_Push:
\t0.0 0.0
\t0.002 32.0
\t0.004 88.0
\t0.006 167.0
\t0.008 269.0
\t0.010 393.0
\t0.012 596.0
\t0.015 1085.0
\t0.017 2098.0
\t0.019 6559.0
\t0.021 16527.0
\t0.023 31862.0
\t0.025 52427.0
\t0.027 78083.0
\t0.030 108691.0
SuspR.Buf_Pull.tz0 = -0.065
SuspR.Buf_Pull.Amplify = 1
SuspR.Buf_Pull:
\t0.0 0.0
\t0.002 32.0
\t0.004 88.0
\t0.006 167.0
\t0.008 269.0
\t0.010 393.0
\t0.012 596.0
\t0.015 1085.0
\t0.017 2098.0
\t0.019 6559.0
\t0.021 16527.0
\t0.023 31862.0
\t0.025 52427.0
\t0.027 78083.0
\t0.030 108691.0
SuspR.Stabi.Amplify = 1.0
SuspR.Stabi = 15000.0
SuspR.Stabi.Unit = N/m
SuspR.Kin.N = 1
SuspR.Kin.0.Kind = Linear2D 1
SuspR.Kin.0.ValidSide = left+right
SuspR.Kin.0.InputSide = left
SuspR.Kin.0.L.tx = 0 0 0
SuspR.Kin.0.L.ty = 0 0 0
SuspR.Kin.0.L.tz = 0 1.0 0
SuspR.Kin.0.L.rx = 0.01 0 0
SuspR.Kin.0.L.ry = 0 0 0
SuspR.Kin.0.L.rz = 0.002 -0.035 0
SuspR.Kin.0.L.lSpring = 0 -1.0 0
SuspR.Kin.0.L.lDamp = 0 -0.6 0
SuspR.Kin.0.L.lBuf = 0 -1.0 0
SuspR.Kin.0.L.lStabi = 0 0.81 0
SuspR.Com.N = 0
SuspR.WhlBearing.On = 0
SuspExtFrcs.Kind =
Susp.Comment:

""", splitter="")))
        self._Attributes.append(('Steering',                 cSimpleDataType("## Steering ##", str,"""############################################################
Steering.Kind = GenAngle 1
Steering.Rack2StWhl = 100.0
Steering.Comment:""", splitter="")))        
        ###self._Attributes.append(('Steering.headline',                 cSimpleDataType("## Steering ##", str, "############################################################", splitter="")))
        ###self._Attributes.append(('Steering.Kind',                     cSimpleDataType("Steering.Kind", str, "GenAngle 1")))
        ###self._Attributes.append(('Steering.Rack2StWhl',               cSimpleDataType("Steering.Rack2StWhl", float, 100.0, valueChecker=lambda x: x>=0)))
        ###self._Attributes.append(('Steering.Comment',                  cSimpleDataType("Steering.Comment", str, "\n", splitter=":\n")))
        self._Attributes.append(('Aerodynamics',                      cSimpleDataType("## Aerodynamics ##", str, """########################################################
Aero.Kind = Coeff6x1 1
Aero.Crosswind.Kind = Step
Aero.Ax = 2.0
Aero.lReference = 2.508
Aero.pos = 2.25 0.0 0.6
Aero.Coeff:
\t-180 -0.4 0.0 0.1 0.0 -0.01 0.0
\t-120 -0.2 -1.4 0.7 -0.2 -0.021 0.06
\t-90 0.0 -1.7 0.9 -0.2 0.0 0.0
\t-45 0.3 -1.2 0.6 -0.16 0.025 -0.1
\t0 0.2 0.0 0.1 0.0 -0.03 0.0
\t45 0.3 1.2 0.6 0.16 0.025 0.1
\t90 0.0 1.7 0.9 0.2 0.000 0.0
\t120 -0.2 1.4 0.7 0.2 -0.021 -0.06
\t180 -0.4 0.0 0.1 0.0 -0.010 0.0
Aero.Comment:

""", splitter="")))
        # Sensor information will be added in between
        self._Attributes.append(('Show.headline',                     cSimpleDataType("## Show ##", str, "################################################################", splitter="")))
        self._Attributes.append(('Crypto.Show',                       cSimpleDataType("Crypto.Show", str, "\tPicture.PicFName\n\tMovie.Skin.FName\n\tVehicle.OuterSkin\n\tnAxle\n\tSteering.Kind\n\tSteering.FName\n\tBrake.Kind\n\tBrake.FName\n\tPowerTrain.Kind\n\tPowerTrain.FName\n\tCM4SL.ActivateFcn\n\tCM4SL.StartFcn\n\tCM4SL.StopFcn", splitter=":\n")))
        self._Attributes.append(('AdditionalParameters',              cSimpleDataType("\n\n## Additional Parameters ##", str, "###############################################", splitter="")))
        ################################
        self._setAttributes()

        if srcVehicleFile is not None:
            self._srcFile = srcVehicleFile[srcVehicleFile.find("Vehicle\\")+len("Vehicle\\"):]
            for element in self._Attributes:
                if type(element[1]) is cSimpleDataType:
                    cSimpleDataType.readfromFile(element[1], srcVehicleFile)
                elif type(element[1]) is cSimple3Dfloat:
                    cSimple3Dfloat.readfromFile(element[1], srcVehicleFile)
                elif type(element[1]) is cSimple6Dfloat:
                    cSimple6Dfloat.readfromFile(element[1], srcVehicleFile)

    def GetTestRunText(self):
        return "Vehicle = " + self._srcFile.replace('\\', '/') + "\n"

    def save(self, fileName):
        with open(fileName,"w") as f:
            for currAttribute in self._Attributes:
                f.write(currAttribute[1].getConfigFileText())
                if currAttribute[1].getConfigFileText().find("Aero.Comment:") >= 0:
                    f.write(self.Sensors.GetTestRunText())

    def changeSrcFile(self, srcVehicleFile):
        self.__init__(srcVehicleFile)

### VEHICLELOAD CLASS ###
class cCMVehicleLoad(cDynamicProperty):
    def __init__(self, srcTestRunFile=None):
        self._Attributes = []
        ################################
        self._Attributes.append(('fl.mass', cSimpleDataType("VehicleLoad.0.mass", int, 0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('fl.pos',  cSimple3Dfloat("VehicleLoad.0.pos", [0.0, 0.0, 0.0])))
        self._Attributes.append(('fr.mass', cSimpleDataType("VehicleLoad.1.mass", int, 0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('fr.pos',  cSimple3Dfloat("VehicleLoad.1.pos", [0.0, 0.0, 0.0])))
        self._Attributes.append(('rl.mass', cSimpleDataType("VehicleLoad.2.mass", int, 0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('rl.pos',  cSimple3Dfloat("VehicleLoad.2.pos", [0.0, 0.0, 0.0])))
        self._Attributes.append(('rr.mass', cSimpleDataType("VehicleLoad.3.mass", int, 0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('rr.pos',  cSimple3Dfloat("VehicleLoad.3.pos", [0.0, 0.0, 0.0])))
        ################################
        self._setAttributes()

        if srcTestRunFile is not None:
            for element in self._Attributes:
                if type(element[1]) is cSimpleDataType:
                    cSimpleDataType.readfromFile(element[1], srcTestRunFile)
                elif type(element[1]) is cSimple3Dfloat:
                    cSimple3Dfloat.readfromFile(element[1], srcTestRunFile)

    def GetTestRunText(self):
        testRunText = ""
        for currAttribute in self._Attributes:
            testRunText += currAttribute[1].getConfigFileText()
        return testRunText

