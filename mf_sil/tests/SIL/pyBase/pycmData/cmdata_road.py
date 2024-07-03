from cmdata_common import cDynamicProperty, cSimpleDataType

class cCMRoadMarking(cDynamicProperty):
    def __init__(self, Position=0, Width=0.15, Type=1):
        self.Position   = Position # center = 0, left border = 1, right border = -1
        self.Width      = Width
        self.Type       = Type # 1 = continuous, 2 = broken

class cCMProfilePoint(cDynamicProperty):
    def __init__(self, Offset=0, Value=0, Grade=-999):
        self.Offset   = Offset
        self.Value    = Value
        self.Grade    = Grade
        
class cCMRoadProfile(cDynamicProperty):
    def __init__(self):
        self.ProfilePoints = [] 

class cCMLane(cDynamicProperty):
    def __init__(self, WidthStart=0, WidthEnd=0, Type=0, RoadMarking=None):
        self.WidthStart   = WidthStart
        self.WidthEnd     = WidthEnd
        self.Type         = Type # 0 = driving lane, 5 = road side, 11 = pedestrian path, 13 = parking area
        if RoadMarking != None:
            self.RoadMarking = RoadMarking
        else:
            self.RoadMarking = None
            
    def addRoadMarking(self, Position=0, Width=0.15, Type=1):
        self.RoadMarking = cCMRoadMarking(Position, Width, Type)

class cCMLaneSection(cDynamicProperty):
    def __init__(self, Start=0, RoadMarking=None):
        self.Start   = Start
        self.LanesLeft   = [cCMLane(3.5, 3.5, 0, cCMRoadMarking(1)), cCMLane(2.5, 2.5, 5)]
        self.LanesRight  = [cCMLane(3.5, 3.5, 0, cCMRoadMarking(-1)), cCMLane(2.5, 2.5, 5)]
        if RoadMarking != None:
            self.RoadMarking = RoadMarking
        else:
            self.RoadMarking = cCMRoadMarking(Position=0, Type=2)
            
    def addLane(self, Side, WidthStart=0, WidthEnd=0, Type=0, ExistRM=False, Position=0, Width=0.15, RMType=1):
        if ExistRM:
            RoadMarking = cCMRoadMarking(Position, Width, RMType)
        else:
            RoadMarking = None
            
        if Side == "left":
            self.LanesLeft.insert(len(self.LanesLeft) - 1, cCMLane(WidthStart, WidthEnd, Type, RoadMarking))
        elif Side == "right":
            self.LanesRight.insert(len(self.LanesRight) - 1, cCMLane(WidthStart, WidthEnd, Type, RoadMarking))
        else:
            print("Incorrect side parameter in function addLane().")
            sys.exit(0)
            
    def addRoadMarking(self, Position=0, Width=0.15, Type=2):
        self.RoadMarking = cCMRoadMarking(Position, Width, Type)
            
class cCMSegment(cDynamicProperty):
    def __init__(self, Type='Straight', Dim1=0, Dim2=0):
        self.Type   = Type
        self.Dim1   = Dim1 # Dim1 is length for straight road / radius for curved road
        self.Dim2   = Dim2 # Dim2 is angle for curved road
        
class cCMBump(cDynamicProperty):
    def __init__(self, Type='Beam', StartOffset=0.0, LatOffset=0.0, Reference=0, Height=0.1, RampUp=0.2, Plateau=0.2, RampDown=0.2, Width=1.0, Material="Textures/Ground/SleepingPoliceman_1.png"):
        self.Type        = Type
        self.StartOffset = StartOffset
        self.LatOffset   = LatOffset
        self.Reference   = Reference # 0 = Center, 2 = Lane 0 Left, -2 = Lane 0 Right
        self.Height      = Height
        self.RampUp      = RampUp
        self.Plateau     = Plateau
        self.RampDown    = RampDown
        self.Width       = Width
        self.Material    = Material
            
class cCMLink(cDynamicProperty):
    def __init__(self, ElevationProfile=None, SlopeProfile=None, CamberProfile=None):
        self.Segments       = []
        self.LaneSections   = []
        self.Bumps          = []
        if ElevationProfile != None:
            self.ElevationProfile = ElevationProfile
        else:
            self.ElevationProfile = None
        if SlopeProfile != None:
            self.SlopeProfile = SlopeProfile
        else:
            self.SlopeProfile = None
        if CamberProfile != None:
            self.CamberProfile = CamberProfile
        else:
            self.CamberProfile = None
            
    def addSegment(self):
        self.Segments.append(cCMSegment())
        
    def addLaneSection(self):
        self.LaneSections.append(cCMLaneSection())
        
    def addBump(self):
        self.Bumps.append(cCMBump())

class cCMStraight(cDynamicProperty):
    def __init__(self, Length=100, Grad=0, Slope=0, Camber=0, RoadAttributes=None, BumpMarkerInfo=''):
        self.Length  = Length
        self.Grad    = Grad
        self.Slope   = Slope
        self.Camber  = Camber
        self.BumpMarkerInfo = BumpMarkerInfo
        if RoadAttributes != None:
            self.RoadAttributes = RoadAttributes
        else:
            self.RoadAttributes = None

class cCMTurnRight(cDynamicProperty):
    def __init__(self, Radius=100, Angle=90, Grad=0, Slope=0, Camber=0, RoadAttributes=None, BumpMarkerInfo=''):
        self.Radius = Radius
        self.Angle  = Angle
        self.Grad   = Grad
        self.Slope  = Slope
        self.Camber = Camber
        self.BumpMarkerInfo = BumpMarkerInfo
        if RoadAttributes != None:
            self.RoadAttributes = RoadAttributes
        else:
            self.RoadAttributes = None

class cCMTurnLeft(cDynamicProperty):
    def __init__(self, Radius=100, Angle=90, Grad=0, Slope=0, Camber=0, RoadAttributes=None, BumpMarkerInfo=''):
        self.Radius = Radius
        self.Angle  = Angle
        self.Grad   = Grad
        self.Slope  = Slope
        self.Camber = Camber
        self.BumpMarkerInfo = BumpMarkerInfo
        if RoadAttributes != None:
            self.RoadAttributes = RoadAttributes
        else:
            self.RoadAttributes = None

class cCMRoadAttributes(cDynamicProperty):
    def __init__(self, TrackWidth={'Left':3.5, 'Right':3.5}, MarginWidth={'Left':0.5, 'Right':0.5}, Friction={'Track':1.0, 'Margin':1.0}, FrictionStripe1={'Friction': '-', 'From':0.0, 'To':0.0}, FrictionStripe2={'Friction':'-', 'From':0.0, 'To':0.0}):
        self.TrackWidth      = TrackWidth
        self.MarginWidth     = MarginWidth
        self.Friction        = Friction
        self.FrictionStripe1 = FrictionStripe1
        self.FrictionStripe2 = FrictionStripe2


class cCMRoadDefinition(cDynamicProperty):
    def __init__(self, srcTestRunFile=None):
        self.Links = []
        self._DigRoadExists = False
        if srcTestRunFile is None:
            self.FileIdent = 'IPGRoad 6.0'
            # self.Origin = {'x':0.0, 'y':0.0, 'z':0.0, 'alpha':0.0}
            # self.DigRoad = ""
            self.Default = cCMRoadAttributes()
            
            elevationProfile = cCMRoadProfile()
            elevationProfile.ProfilePoints.append(cCMProfilePoint(0, 0))
            elevationProfile.ProfilePoints.append(cCMProfilePoint(40, 0))
            elevationProfile.ProfilePoints.append(cCMProfilePoint(80, 0))
            slopeProfile = cCMRoadProfile()
            slopeProfile.ProfilePoints.append(cCMProfilePoint(0, 0))
            slopeProfile.ProfilePoints.append(cCMProfilePoint(80, 0))
            camberProfile = cCMRoadProfile()
            camberProfile.ProfilePoints.append(cCMProfilePoint(0, 0))
            camberProfile.ProfilePoints.append(cCMProfilePoint(80, 0))
            
            link = cCMLink(elevationProfile, slopeProfile, camberProfile)
            link.Segments.append(cCMSegment('Straight', 80))
            link.LaneSections.append(cCMLaneSection())
            
            self.Links.append(link)
        else:
                with open(srcTestRunFile, 'r') as f:
                    RoadDef = False
                    for row in f:
                        addInfo = ''
                        if row.find("Road.Definition:") is 0:
                            nameline = next(f)
                            self.FileIdent = (nameline[len("\tFileIdent "):nameline.find("\n")])
                            nameline = f.next().split()
                            if nameline[0] == 'DigRoad':
                                self._DigRoadExists = True
                                self.DigRoad = nameline[1]
                                nameline = f.next().split()
                            self.Origin = {'x':float(nameline[1]), 'y':float(nameline[2]), 'z':float(nameline[3]), 'alpha':float(nameline[4])}
                            nameline = f.next().split()
                            #nameline = ['0' if x=='-' else x for x in nameline]
                            self.Default = cCMRoadAttributes(TrackWidth={'Left':(nameline[1]), 'Right':(nameline[2])}, MarginWidth={'Left':(nameline[3]), 'Right':(nameline[4])}, Friction={'Track':(nameline[5]), 'Margin':(nameline[6])}, FrictionStripe1={'Friction':(nameline[7]), 'From':(nameline[8]), 'To':(nameline[9])}, FrictionStripe2={'Friction':(nameline[10]), 'From':(nameline[11]), 'To':(nameline[12])})
                            RoadDef = True
                        if RoadDef is True and row.startswith('\tStraight'): #Straight
                            rowlist = row.split()
                            if rowlist[2:6] == ['-','-','-','-'] and len(rowlist) == 9:
                                RoadAttributes = None
                            else:
                                RoadAttributes = cCMRoadAttributes(TrackWidth={'Left':(rowlist[2]), 'Right':(rowlist[3])}, MarginWidth={'Left':(rowlist[4]), 'Right':(rowlist[5])}, Friction={'Track':(rowlist[9]), 'Margin':(rowlist[10])}, FrictionStripe1={'Friction':(rowlist[11]), 'From':(rowlist[12]), 'To':(rowlist[13])}, FrictionStripe2={'Friction':(rowlist[14]), 'From':(rowlist[15]), 'To':(rowlist[16])})
                            self.Segments.append(cCMStraight(Length=float(rowlist[1]), Grad=float(rowlist[6]), Slope=float(rowlist[7]), Camber=float(rowlist[8]), RoadAttributes=RoadAttributes))
                        elif RoadDef is True and row.startswith('\tTurnLeft'): #TurnLeft
                            rowlist = row.split()
                            if rowlist[3:7] == ['-','-','-','-'] and len(rowlist) == 10:
                                RoadAttributes = None
                            else:
                                RoadAttributes = cCMRoadAttributes(TrackWidth={'Left':(rowlist[3]), 'Right':(rowlist[4])}, MarginWidth={'Left':(rowlist[5]), 'Right':(rowlist[6])}, Friction={'Track':(rowlist[10]), 'Margin':(rowlist[11])}, FrictionStripe1={'Friction':(rowlist[12]), 'From':(rowlist[13]), 'To':(rowlist[14])}, FrictionStripe2={'Friction':(rowlist[15]), 'From':(rowlist[16]), 'To':(rowlist[17])})
                            self.Segments.append(cCMTurnLeft(Radius=float(rowlist[1]), Angle=float(rowlist[2]), Grad=float(rowlist[7]), Slope=float(rowlist[8]), Camber=float(rowlist[9]), RoadAttributes=RoadAttributes))
                        elif RoadDef is True and row.startswith('\tTurnRight'): #TurnRight
                            rowlist = row.split()
                            if rowlist[3:7] == ['-','-','-','-'] and len(rowlist) == 10:                        
                                RoadAttributes = None
                            else:
                                RoadAttributes = cCMRoadAttributes(TrackWidth={'Left':(rowlist[3]), 'Right':(rowlist[4])}, MarginWidth={'Left':(rowlist[5]), 'Right':(rowlist[6])}, Friction={'Track':(rowlist[10]), 'Margin':(rowlist[11])}, FrictionStripe1={'Friction':(rowlist[12]), 'From':(rowlist[13]), 'To':(rowlist[14])}, FrictionStripe2={'Friction':(rowlist[15]), 'From':(rowlist[16]), 'To':(rowlist[17])})
                            self.Segments.append(cCMTurnRight(Radius=float(rowlist[1]), Angle=float(rowlist[2]), Grad=float(rowlist[7]), Slope=float(rowlist[8]), Camber=float(rowlist[9]), RoadAttributes=RoadAttributes))
                        elif RoadDef is True and row.startswith('\t+'):
                            self.Segments[-1].BumpMarkerInfo += row
                            
    # def addLink(self) TODO: addLink is dependent on addJunction method (not available)
        

    def GetTestRunText(self):
        testRunText = 'Road.FileIdent = ' + self.FileIdent + '\n'
        if hasattr(self, "DigRoad"):
            if self.DigRoad != "":
                self._DigRoadExists = True   # workaround
        if self._DigRoadExists:
            testRunText += 'Road.FName = ' + self.DigRoad + '\n'
        else:
            # testRunText += '\tOrigin    ' + str(self.Origin['x']) + ' ' + str(self.Origin['y']) + ' ' + str(self.Origin['z']) + ' ' + str(self.Origin['alpha']) + '\n'
            # testRunText += ('\tDefault ' + str(self.Default.TrackWidth['Left']) + ' ' + str(self.Default.TrackWidth['Right']) + ' ' + str(self.Default.MarginWidth['Left']) + ' ' + str(self.Default.MarginWidth['Right'])
                            # + ' ' + str(self.Default.Friction['Track']) + ' ' + str(self.Default.Friction['Margin']) + ' ' + str(self.Default.FrictionStripe1['Friction']) + ' ' + str(self.Default.FrictionStripe1['From'])  + ' ' + str(self.Default.FrictionStripe1['To'])
                            # + ' ' + str(self.Default.FrictionStripe2['Friction']) + ' ' + str(self.Default.FrictionStripe2['From'])  + ' ' + str(self.Default.FrictionStripe2['To']) + '\n')
                            
            testRunText += "Road.Movie = 0.2 1 0.02 1.5 1.5 1 1" + '\n'
            testRunText += "Road.Movie.BgGeoFName = Plugins/AUP_Visualization.tclgeo" + '\n'
                            
            idxLink  = 0
            for currLink in self.Links:
                idxSeg = 0
                for currSegment in currLink.Segments:
                    testRunText += "Road.Link." + str(idxLink) + ".Seg." + str(idxSeg) + ".Type = " + currSegment.Type + "\n"
                    testRunText += "Road.Link." + str(idxLink) + ".Seg." + str(idxSeg) + ".Param = " + str(currSegment.Dim1) + " " + str(currSegment.Dim2) + " 0 0 0 0 0 0\n"
                    idxSeg += 1
                    
                idxLaneSec = 0
                for currLaneSection in currLink.LaneSections:
                    testRunText += "Road.Link." + str(idxLink) + ".LaneSection." + str(idxLaneSec) + ".Start = " + str(currLaneSection.Start) + "\n"
                    
                    idxLane = 0
                    for currLane in currLaneSection.LanesLeft:
                        testRunText += "Road.Link." + str(idxLink) + ".LaneSection." + str(idxLaneSec) + ".LaneL." + str(idxLane) + " = 0 " + str(currLane.WidthStart) + " " + str(currLane.WidthEnd) + " " + str(currLane.Type) + " 0 0 0\n"
                        if currLane.RoadMarking != None:
                            testRunText += "Road.Link." + str(idxLink) + ".LaneSection." + str(idxLaneSec) + ".LaneL." + str(idxLane) + ".RoadMarking.0 = 0 0 0 1 0 " + str(currLane.RoadMarking.Position) + " " + str(currLane.RoadMarking.Width) + " 0 " + str(currLane.RoadMarking.Type) + " 0 0 2 4 1 1 0 \"1.0,1.0,1.0\"\n"
                        idxLane += 1
                    idxLane = 0
                    for currLane in currLaneSection.LanesRight:
                        testRunText += "Road.Link." + str(idxLink) + ".LaneSection." + str(idxLaneSec) + ".LaneR." + str(idxLane) + " = 0 " + str(currLane.WidthStart) + " " + str(currLane.WidthEnd) + " " + str(currLane.Type) + " 0 0 0\n"
                        if currLane.RoadMarking != None:
                            testRunText += "Road.Link." + str(idxLink) + ".LaneSection." + str(idxLaneSec) + ".LaneR." + str(idxLane) + ".RoadMarking.0 = 0 0 0 1 0 " + str(currLane.RoadMarking.Position) + " " + str(currLane.RoadMarking.Width) + " 0 " + str(currLane.RoadMarking.Type) + " 0 0 2 4 1 1 0 \"1.0,1.0,1.0\"\n"
                        idxLane += 1
                    if currLaneSection.RoadMarking != None:
                        testRunText += "Road.Link." + str(idxLink) + ".LaneSection." + str(idxLaneSec) + ".RoadMarking.0 = 0 0 0 1 0 " + str(currLaneSection.RoadMarking.Position) + " " + str(currLaneSection.RoadMarking.Width) + " 0 " + str(currLaneSection.RoadMarking.Type) + " 0 0 2 4 1 1 0 \"1.0,1.0,1.0\"\n"
                    idxLaneSec += 1
                
                testRunText += "Road.Link." + str(idxLink) + ".ElevationProfile.Params = 0 0\n"
                testRunText += "Road.Link." + str(idxLink) + ".ElevationProfile:\n"
                for currPoint in currLink.ElevationProfile.ProfilePoints:
                    testRunText += "\t" + str(currPoint.Offset) + " " + str(currPoint.Value) + " " + str(currPoint.Grade) + "\n"
                    
                testRunText += "Road.Link." + str(idxLink) + ".SlopeProfile.Params = 0 0\n"
                testRunText += "Road.Link." + str(idxLink) + ".SlopeProfile:\n"
                for currPoint in currLink.SlopeProfile.ProfilePoints:
                    testRunText += "\t" + str(currPoint.Offset) + " " + str(currPoint.Value) + " " + str(currPoint.Grade) + "\n"
                    
                testRunText += "Road.Link." + str(idxLink) + ".CamberProfile.Params = 0 0\n"
                testRunText += "Road.Link." + str(idxLink) + ".CamberProfile:\n"
                for currPoint in currLink.CamberProfile.ProfilePoints:
                    testRunText += "\t" + str(currPoint.Offset) + " " + str(currPoint.Value) + " " + str(currPoint.Grade) + "\n"
                
                idxBump = 0
                for currBump in currLink.Bumps:
                    testRunText += "Road.Link." + str(idxLink) + ".Bump." + str(idxBump) + ".Type = " + currBump.Type + "\n"
                    testRunText += "Road.Link." + str(idxLink) + ".Bump." + str(idxBump) + ".Param = " + str(currBump.StartOffset) + " 0 " + str(currBump.LatOffset) + " " + str(currBump.Reference) + " 1 0 0 " + str(currBump.Height) + " " + str(currBump.RampUp) + " " + str(currBump.Plateau) + " " + str(currBump.RampDown) + " " + str(currBump.Width) + "\n"
                    testRunText += "Road.Link." + str(idxLink) + ".Bump." + str(idxBump) + ".Material = " + currBump.Material + "\n"
                    idxBump += 1
                
                idxLink += 1
            
            # custom code
            testRunText += "Road.Link." + str(idxLink - 1) + ".SignPlate.0 = 20 0 0 1 1 4 90 1 3 \"Contintneal.jpg\"\n"
            testRunText += "Road.Link." + str(idxLink - 1) + ".SignPlate.1 = 60 0 0 1 1 4 90 1 3 \"Contintneal.jpg\"\n"
            testRunText += "Road.Link." + str(idxLink - 1) + ".TreeStrip.0 = 0.348 0 79.773 0 5 1 10 2 1 1 0.5 0.5\n"
            testRunText += "Route.0.Name = Route_0\n"
            testRunText += "Route.0.LinkList:\n"
            testRunText += "\t0 0\n"
                    
                    
                    # if isinstance(currSegment, cCMStraight):    firstText = "\tStraight  " + str(currSegment.Length)
                    # elif isinstance(currSegment, cCMTurnRight): firstText = "\tTurnRight  " + str(currSegment.Radius) + " " + str(currSegment.Angle)
                    # elif isinstance(currSegment, cCMTurnLeft):  firstText = "\tTurnLeft  " + str(currSegment.Radius) + " " + str(currSegment.Angle)
                    # if currSegment.RoadAttributes == None: 
                        # testRunText += firstText + " - - - -  " + str(currSegment.Grad) + " " + str(currSegment.Slope) + " " + str(currSegment.Camber) + '\n' + currSegment.BumpMarkerInfo
                    # else: 
                        # testRunText += (firstText + " " + str(currSegment.RoadAttributes.TrackWidth['Left']) + " " + str(currSegment.RoadAttributes.TrackWidth['Right']) + " " + str(currSegment.RoadAttributes.MarginWidth['Left']) + " " + str(currSegment.RoadAttributes.MarginWidth['Right'])
                                        # + " " + str(currSegment.Grad) + " " + str(currSegment.Slope) + " " + str(currSegment.Camber) + " " + str(currSegment.RoadAttributes.Friction['Track']) + " " + str(currSegment.RoadAttributes.Friction['Margin'])
                                        # + " " + str(currSegment.RoadAttributes.FrictionStripe1['Friction']) + " " + str(currSegment.RoadAttributes.FrictionStripe1['From'])  + " " + str(currSegment.RoadAttributes.FrictionStripe1['To'])
                                        # + " " + str(currSegment.RoadAttributes.FrictionStripe2['Friction']) + " " + str(currSegment.RoadAttributes.FrictionStripe2['From'])  + " " + str(currSegment.RoadAttributes.FrictionStripe2['To']) + "\n" 
                                        # + currSegment.BumpMarkerInfo)
        return testRunText


### ROAD CLASS ###
class cCMRoad(cDynamicProperty):
    def __init__(self, srcTestRunFile=None):
        self.Definition = cCMRoadDefinition(srcTestRunFile)
        self._Attributes = []

        self._Attributes.append(('VhclStartPos',    cSimpleDataType("Road.VhclStartPos", str, "0 0 0")))
        self._Attributes.append(('Lane',            cSimpleDataType("Road.Lane", str, "right")))
        self._Attributes.append(('DigSmooth',       cSimpleDataType("Road.DigSmooth", str, "0.2 0 0")))
        self._Attributes.append(('DigOptions',      cSimpleDataType("Road.DigOptions", str)))
        self._Attributes.append(('Country',         cSimpleDataType("Road.Country", str, "DEU")))
        self._Attributes.append(('RouteId',         cSimpleDataType("Road.VhclRoute", str, "Route_0")))
        #self._Attributes.append(('FNameDigi',          cSimpleDataType("Road.FNameDigi", str)))

        self._Attributes.append(('GCS.RefPos_0',        cSimpleDataType("Road.GCS.RefPos_0", str, "0 0 0")))
        self._Attributes.append(('GCS.RefPos_GCS',      cSimpleDataType("Road.GCS.RefPos_GCS", str, "49 9 100")))
        self._Attributes.append(('GCS.Projection',      cSimpleDataType("Road.GCS.Projection", str, "FlatEarth")))
        self._Attributes.append(('ToMovie.MStrip',      cSimpleDataType("Road.2Movie.MStrip", str, "0 3 6")))
        self._Attributes.append(('ToMovie.SStrip',      cSimpleDataType("Road.2Movie.SStrip", str, "0 0 0 0.1")))
        self._Attributes.append(('ToMovie.GStrip',      cSimpleDataType("Road.2Movie.GStrip", str, "1.5 100")))
        self._Attributes.append(('ToMovie.MaxErrXY',    cSimpleDataType("Road.2Movie.MaxErrXY", float, 0.02)))
        self._Attributes.append(('ToMovie.GeoStepSize', cSimpleDataType("Road.2Movie.GeoStepSize", float, 0.20)))
        self._Attributes.append(('ToMovie.GenBridges',  cSimpleDataType("Road.2Movie.GenBridges", int, 0)))
        #self._Attributes.append(('ToMovie.GenFric',     cSimpleDataType("Road.2Movie.GenFric", int, 1)))
        #self._Attributes.append(('ToMovie.GenPylone',   cSimpleDataType("Road.2Movie.GenPylone", int, 1)))
        #self._Attributes.append(('ToMovie.GenVelSign',  cSimpleDataType("Road.2Movie.GenVelSign", int, 1)))
        #self._Attributes.append(('ToMovie.GenSideWind', cSimpleDataType("Road.2Movie.GenSideWind", int, 1)))
        self._Attributes.append(('ToMovie.BgGeoFName',  cSimpleDataType("Road.2Movie.BgGeoFName", str)))
        self._Attributes.append(('ToMovie.BgGeoOptions',  cSimpleDataType("Road.2Movie.BgGeoOptions", str)))
        self._Attributes.append(('ToMovie.TerrainFName',  cSimpleDataType("Road.2Movie.TerrainFName", str)))
        #self._Attributes.append(('Definition',         cSimpleDataType("Road.Definition", str, "\tFileIdent IPGRoad 5.0\n\tOrigin    0 0 0 0\n\tDefault 3.00 3.00 0.5 0.5 1.0 1.0 - 0 0 - 0 0", splitter=": \n")))
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
        testRunText += self.Definition.GetTestRunText()
        return testRunText