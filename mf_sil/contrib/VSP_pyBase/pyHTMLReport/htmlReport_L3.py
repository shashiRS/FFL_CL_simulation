#====================================================================
# System Imports
#====================================================================
import os
import sys
import imp
import copy
import shutil
import logging
logger = logging.getLogger("pyBase")
try:
    from win32api import GetSystemMetrics
except:
    import wx
#====================================================================
# Own Imports
#====================================================================
from pyBase.pyHTMLReport.template_contimodern import htmlReport_template_L3

template = os.path.join(os.path.dirname(htmlReport_template_L3.__file__), "htmlReport_template_L3.py")

def copytree(src, dst, symlinks=False, ignore=None):
    names = os.listdir(src)
    if ignore is not None:
        ignored_names = ignore(src, names)
    else:
        ignored_names = set()

    if not os.path.isdir(dst): # This one line does the trick
        os.makedirs(dst)
    errors = []
    for name in names:
        if name in ignored_names:
            continue
        srcname = os.path.join(src, name)
        dstname = os.path.join(dst, name)
        try:
            if symlinks and os.path.islink(srcname):
                linkto = os.readlink(srcname)
                os.symlink(linkto, dstname)
            elif os.path.isdir(srcname):
                copytree(srcname, dstname, symlinks, ignore)
            else:
                # Will raise a SpecialFileError for unsupported file types
                shutil.copy2(srcname, dstname)
        # catch the Error from the recursive copytree so that we can
        # continue with other files
        except EnvironmentError as why:
            errors.append((srcname, dstname, str(why)))
    try:
        shutil.copystat(src, dst)
    except OSError as why:
        if WindowsError is not None and isinstance(why, WindowsError):
            # Copying file access times may fail on Windows
            pass
        else:
            errors.extend((src, dst, str(why)))

def ReducePlotData(xData, yData):
    newXData = [xData[0]]
    newYData = copy.deepcopy(yData)
    for currNewYData in newYData:
        currNewYData["dataPoints"] = [currNewYData["dataPoints"][0]]
    #lastSeenYValues = [ currYData["dataPoints"][0] for currYData in yData ]
    for currPointIdx in range(len(xData))[1:]:
        changedYValue = False
        # Check whether any of the yGraph Values has changed
        for currYDataIdx, currYData in enumerate(yData):
            if abs(currYData["dataPoints"][currPointIdx] - newYData[currYDataIdx]["dataPoints"][-1]) > 0.00001:
                changedYValue = True
        if changedYValue:
            newXData.append(xData[currPointIdx-1])
            newXData.append(xData[currPointIdx])
            for currYDataIdx, currYData in enumerate(yData):
                newYData[currYDataIdx]["dataPoints"].append(currYData["dataPoints"][currPointIdx-1])
                newYData[currYDataIdx]["dataPoints"].append(currYData["dataPoints"][currPointIdx])

    # Always add last data point to make plot long enough
    newXData.append(xData[-1])
    for currYDataIdx, currYData in enumerate(yData):
        newYData[currYDataIdx]["dataPoints"].append(currYData["dataPoints"][-1])

    return newXData, newYData

#====================================================================
# HTML Report Table
#====================================================================
class cHTMLReportTableCell(object):
    def __init__(self, text=None, textColor=None, textAlign=None, textStyle=None, backgroundColor=None, tooltip=None, colSpan=None):
        self.text            = text
        self.tooltip         = tooltip
        self.textColor       = textColor
        self.textAlign       = textAlign
        self.textStyle       = textStyle
        self.backgroundColor = backgroundColor
        self.colSpan         = colSpan

    def propagateStyle(self, textColor, textAlign, textStyle, backgroundColor):
        if self.textColor       is None: self.textColor       = textColor
        if self.textAlign       is None: self.textAlign       = textAlign
        if self.textStyle       is None: self.textStyle       = textStyle
        if self.backgroundColor is None: self.backgroundColor = backgroundColor

class cHTMLReportTableRow(object):
    def __init__(self, cells=[], textColor=None, textAlign=None, textStyle=None, backgroundColor=None, classStr=''):
        self.cells           = cells
        self.textColor       = textColor
        self.textAlign       = textAlign
        self.textStyle       = textStyle
        self.backgroundColor = backgroundColor
        self.classStr = classStr

        # Shortcut for handling the row type: If the user just passes a string it is converted to a cell
        for idx,currCell in enumerate(self.cells):
            if type(currCell) is str:
                self.cells[idx] = cHTMLReportTableCell(text=currCell)
            elif type(currCell) is cHTMLReportTableCell:
                pass
            else:
                logger.exception("Wrong argument type: " + str(type(currCell)))

    def addCell(self, cell):
        self.cells.append(cell)

    def propagateStyle(self, textColor, textAlign, textStyle, backgroundColor):
        if self.textColor       is None: self.textColor       = textColor
        if self.textAlign       is None: self.textAlign       = textAlign
        if self.textStyle       is None: self.textStyle       = textStyle
        if self.backgroundColor is None: self.backgroundColor = backgroundColor

        for currCell in self.cells:
            try:
                currCell.propagateStyle(self.textColor, self.textAlign, self.textStyle, self.backgroundColor)
            except:
                print('Error: wrong type +' + str(currCell) + ' ' + str(type(currCell)))
            
class cHTMLReportTable(object):
    def __init__(self, label=None, headerRow=None, rows=None, textColor=None, textAlign=None, textStyle=None, backgroundColor=None, headerColSpanRows=None, sortable=True, width=None):
        self.label           = label
        self.headerRow       = headerRow
        self.rows            = rows
        self.textColor       = textColor
        self.textAlign       = textAlign
        self.textStyle       = textStyle
        self.backgroundColor = backgroundColor
        self.headerColSpanRows= headerColSpanRows
        self.sortable        = sortable
        self.width           = width

        if self.rows is None:
            self.rows = []
        if self.headerColSpanRows is None:
            self.headerColSpanRows = []

    def setHeaderRow(self, headerRow):
        self.headerRow = headerRow

    def addHeaderDescription(self, headerColSpan):
        self.headerColSpanRows.append(headerColSpan)

    def addRow(self, row):
        self.rows.append(row)

    def propagateStyle(self):
        self.headerRow.propagateStyle(self.textColor, self.textAlign, self.textStyle, self.backgroundColor)
        for currRow in self.rows:
            currRow.propagateStyle(self.textColor, self.textAlign, self.textStyle, self.backgroundColor)

#====================================================================
# HTML Report Graph
#====================================================================
class cHTMLReportGraphYData(object):
    def __init__(self, dataPoints, name=None, lineColor=None, lineWidth=None, lineType=None, axisAssign = None, yAxisLabel=None, legendLabel=None, NoInterpBetwDataPoints = False):
        self.dataPoints = dataPoints
        self.name       = name
        self.lineColor  = lineColor
        self.lineWidth  = lineWidth
        self.lineType   = lineType
        self.axisAssign = axisAssign
        self.yAxisLabel = yAxisLabel
        self.legendLabel = legendLabel
        self.NoInterpBetwDataPoints = NoInterpBetwDataPoints
        

class cHTMLReportGraph(object):
    def __init__(self, xData, yData=[], xDataName=None, title=None, lineWidth=None, syncGroup=None, width=None, height=200, xLim=None, yLim=None, xLabel=None, yLabel=None, vertLines=None, gridAlignAxis=None, highlightedIntervall=[None,None] , helpstring=None, legendLocation = 'right', rangeselector = False, dataReduction = False):
        self.xData      = xData
        self.yData      = yData
        self.xLabel     = xLabel
        self.yLabel     = yLabel
        self.xLim       = xLim
        self.yLim       = yLim
        self.xDataName  = xDataName
        self.title      = title
        self.lineWidth  = lineWidth
        self.syncGroup  = syncGroup
        self.width      = width
        self.height     = height
        self.vertLines  = vertLines
        self.gridAlignAxis        = gridAlignAxis
        self.highlightedIntervall = highlightedIntervall
        self.helpstring           = helpstring
        self.legendLocation       = legendLocation
        self.rangeselector        = rangeselector
        self.dataReduction        = dataReduction
        if self.width is None:
            try:
                self.width = GetSystemMetrics(0)*0.8
            except:
                app = wx.App(False)
                width, height = wx.GetDisplaySize()
                self.width = width*0.8

    def addYDataSeries(self, yDataSeries):
        self.yData.append(yDataSeries)

class cHTMLReportBarChart(object):
    def __init__(self, title=None, project=None, width=100, height=100, data=""):
        self.title      = title
        self.width      = width
        self.height     = height
        self.data       = data
        self.project    = project

class cHTMLReportVideo(object):
    def __init__(self, title, videoFileName, imgFileName):
        self.title = title
        self.fileName = videoFileName
        self.imgFileName = imgFileName

class cHTMLReportImage(object):
    def __init__(self, path, title=None, width=100, height=100 ):
        self.title      = title
        self.width      = width
        self.height     = height  
        self.path       = path  
        
class cHTMLReportLink(object):
    def __init__(self, path, text, prefix = '\n', title = None ):
        self.prefix     = prefix
        self.text       = text
        self.path       = path
        self.title      = title
        
class cHTMLReportVerticalLine(object):
    def __init__(self, sigName, xCoord, color):
        self.sigName = sigName
        self.xCoord = xCoord 
        self.color = color     

class cHTMLReportCode(object):
    def __init__(self, code):
        self.code = code

class cHTMLReportImageGraph(object):
    def __init__(self, xData, yData, xDataName=None, helpstring=None, imageFolderName=None, title=None, width=None, height=200, xLabel=None, yLabel=None, vertLines=None, legendLocation = 'right', dataReduction = False, cmPrjDir=None):
        self.xData      = xData
        self.yData      = yData
        self.xLabel     = xLabel
        self.yLabel     = yLabel
        self.xDataName = xDataName
        self.imageFolderName  = imageFolderName
        self.title      = title
        self.height     = height
        self.vertLines  = vertLines
        self.legendLocation       = legendLocation
        self.dataReduction        = dataReduction
        self.helpstring = helpstring
        self.cmPrjDir = cmPrjDir
        if self.width is None:
            try:
                self.width = GetSystemMetrics(0)*0.8
            except:
                app = wx.App(False)
                width, height = wx.GetDisplaySize()
                self.width = width*0.8

#====================================================================
#HTML Color Scale
#====================================================================
class cHTMLReportScale(object):
    def __init__(self,value,start=1,end=10,step=1,mode='ascending'):
        self.start = start
        self.end   = end
        self.step  = step
        self.value = value
        self.mode  = mode


#====================================================================
# HTML Report
#====================================================================
class cHTMLReport(object):
    def __init__(self, title="", theme="contimodern", toc=False):
        self._title   = title
        self._content = []
        self._toc     = toc

        # Load Template
        themePath      = os.path.dirname(__file__) + "/template_" + theme +"/htmlReport_template_L3.py"
        # cfgImport      = imp.load_source('currTemplate', template )
        self._template = htmlReport_template_L3.cHTMLReportTemplate()

    def _createAdditionalFolders(self, folder):
        additionalFiles = self._template.GetAdditionalFilePaths()

    def _createReportHTML(self, filePath, lazy_reload=True):
        graphFlag = False
        timestamp = {}
        if not os.path.exists(os.path.dirname(filePath)):
            try:
                os.makedirs(os.path.dirname(filePath))
            except:
                print('Cannot create output file directory ' + os.path.dirname(filePath))
                sys.exit(0)
               
        tableCount = 0
        with open(filePath, "w") as reportFile:
            if (lazy_reload == False):
                reportFile.write(self._template.CreateHeader(self._title, False))
            else:
                reportFile.write(self._template.CreateHeader(self._title))
            reportFile.write(self._template.CreateInitButtons())
            if self._toc:
                reportFile.write(self._template.CreateSectionBegin("Table of Contents"))
                reportFile.write(self._template.CreateTOC(self._content))
                reportFile.write(self._template.CreateSectionEnd())
                            
            # Process Content List
            for currContentItem in self._content:
                if currContentItem["type"] == "SectionBegin":
                    reportFile.write(self._template.CreateSectionBegin(currContentItem["Label"],anchor=currContentItem["anchor"]))
                elif currContentItem["type"] == "SectionEnd":
                    reportFile.write(self._template.CreateSectionEnd())
                elif currContentItem["type"] == "DivisionBegin":
                    reportFile.write(self._template.AddDivision(currContentItem["class"]))
                elif currContentItem["type"] == "DivisionEnd":
                    reportFile.write(self._template.EndDivision())
                elif currContentItem["type"] == "Text":
                    reportFile.write(self._template.CreateTextSection(currContentItem["TextContent"]))
                elif currContentItem["type"] == "Table":
                    # Table Begin
                    table = currContentItem["TableObject"]
                    table.propagateStyle()
                    reportFile.write(self._template.CreateTableBegin(table.label, tableCount=tableCount, sortable=table.sortable, width=table.width))

                    # Header Column Span
                    if table.headerColSpanRows is not None:
                        reportFile.write(self._template.CreateTableHeaderBegin())
                        for currHeaderColSpanRow in table.headerColSpanRows:
                            reportFile.write(self._template.CreateTableRowBegin())
                            for currHeaderColSpanCell in currHeaderColSpanRow.cells:
                                reportFile.write(self._template.CreateTableHeaderColSpanCell(colSpanWidth=currHeaderColSpanCell.colSpan,
                                                                                             text = currHeaderColSpanCell.text,
                                                                                             textColor       = currHeaderColSpanCell.textColor,
                                                                                             textAlign       = currHeaderColSpanCell.textAlign, 
                                                                                             textStyle       = currHeaderColSpanCell.textStyle, 
                                                                                             backgroundColor = currHeaderColSpanCell.backgroundColor))
                            reportFile.write(self._template.CreateTableRowEnd())

                    # Table Header
                    if table.headerRow is not None:
                        reportFile.write(self._template.CreateTableRowBegin())
                        for currHeaderCell in table.headerRow.cells:
                            reportFile.write(self._template.CreateTableHeaderCell(text            = currHeaderCell.text,
                                                                                  textColor       = currHeaderCell.textColor,
                                                                                  textAlign       = currHeaderCell.textAlign, 
                                                                                  textStyle       = currHeaderCell.textStyle, 
                                                                                  backgroundColor = currHeaderCell.backgroundColor))
                        reportFile.write(self._template.CreateTableRowEnd())
                        reportFile.write(self._template.CreateTableHeaderEnd())

                    # Table Rows
                    for currRow in table.rows:
                        c = 0
                        #reportFile.write(self._template.CreateTableRowBegin())
                        for currCell in currRow.cells:
                            c = c + 1
                            if c == 1:
                                reportFile.write(self._template.CreateTableRowBegin(currRow.classStr))
                            reportFile.write(self._template.CreateTableRowCell(text            = currCell.text,
                                                                               textColor       = currCell.textColor,
                                                                               textAlign       = currCell.textAlign, 
                                                                               textStyle       = currCell.textStyle, 
                                                                               backgroundColor = currCell.backgroundColor,
                                                                               tooltip         = currCell.tooltip))
                        reportFile.write(self._template.CreateTableRowEnd())
                    # Table End
                    reportFile.write(self._template.CreateTableEnd())
                    tableCount += 1
                elif currContentItem["type"] =="ColorScale":
                    Scale= currContentItem["ColorScale"]
                    reportFile.write(self._template.generateScale(Scale))
                elif currContentItem["type"] == "Graph":
                    g = currContentItem["GraphObject"]
                    yData = []
                    for currYDataSeries in g.yData:
                        currYData                = {}
                        currYData["dataPoints"]  = currYDataSeries.dataPoints
                        currYData["name"]        = currYDataSeries.name
                        currYData["lineColor"]   = currYDataSeries.lineColor
                        currYData["lineType"]    = currYDataSeries.lineType
                        currYData["lineWidth"]   = g.lineWidth
                        currYData["axisAssign"]  = currYDataSeries.axisAssign
                        currYData["yAxisLabel"]  = currYDataSeries.yAxisLabel
                        currYData["legendLabel"] = currYDataSeries.legendLabel
                        currYData["NoInterpBetwDataPoints"] = currYDataSeries.NoInterpBetwDataPoints
                        if currYDataSeries.lineWidth is not None: currYData["lineWidth"] = currYDataSeries.lineWidth
                        yData.append(currYData)

                    if g.dataReduction:
                        xData, yData = ReducePlotData(g.xData, yData)
                    else:
                        xData = g.xData

                    graphData = {"width": g.width, "height": g.height,\
                                 "title": g.title, "xData": xData, "yData": yData, "xDataName": g.xDataName,\
                                 "xLim": g.xLim, "yLim" : g.yLim, "xLabel":g.xLabel, "yLabel":g.yLabel,\
                                 "syncGroup": g.syncGroup, "vertLines": g.vertLines, "gridAlignAxis": g.gridAlignAxis,
                                 "highlightedIntervall": g.highlightedIntervall,
                                 "helpstring": g.helpstring,
                                 "legendLocation": g.legendLocation,
                                 "rangeselector" : g.rangeselector}
                    reportFile.write(self._template.CreateGraph(graphData, filePath))
                elif currContentItem["type"] == "Image":
                    o = currContentItem["ImageObject"]
                    imageData = {"width": o.width, "height": o.height, "title": o.title, "path" : o.path}
                    reportFile.write(self._template.CreateImage(imageData))
                elif currContentItem["type"] == "Chart":
                    o = currContentItem["ChartObject"]
                    chartData = {"width": o.width, "height": o.height, "title": o.title, "data": o.data, "project": o.project}
                    reportFile.write(self._template.CreateBarChart(chartData))
                elif currContentItem["type"] == "Video":
                    o = currContentItem["VideoObject"]
                    videoData = {"title": o.title, "fileName": o.fileName, "imgFileName": o.imgFileName}
                    reportFile.write(self._template.CreateVideo(videoData))
                elif currContentItem["type"] == "Link":
                    o = currContentItem["LinkObject"]   
                    linkData = {"path": o.path, "text": o.text, "title": o.title, "prefix": o.prefix}
                    reportFile.write(self._template.CreateLink(linkData))
                elif currContentItem["type"] == "Code":
                    o = currContentItem["CodeObject"]
                    codeData = {"code": o.code}
                    reportFile.write(self._template.AddHTMLCode(codeData))
                elif currContentItem["type"] == "ImageGraph":
                    g = currContentItem["ImageGraphObject"]
                    if g.dataReduction:
                        xData, yData = ReducePlotData(g.xData, g.yData)
                    else:
                        xData = g.xData

                    graphData = {"width": g.width, "height": g.height,\
                                 "title": g.title, "xData": xData, "yData": g.yData, "xDataName": g.xDataName,\
                                 "xLabel":g.xLabel, "yLabel":g.yLabel,\
                                 "vertLines": g.vertLines, "legendLocation": g.legendLocation,\
                                 "imageFolderName": g.imageFolderName, "cmPrjDir": g.cmPrjDir, "helpstring": g.helpstring}
                    graphFlag, data = self._template.CreateDynamicImageGraph(graphData, filePath)
                    if graphFlag:
                        reportFile.write(data)
                else:
                    pass
            if graphFlag:
                reportFile.write(self._template.CreateImageGraphScript())
            reportFile.write(self._template.CreateFooter())

    def Save(self, filePath):
        # Create graphData directory
        if not os.path.exists(os.path.join(os.path.dirname(filePath),'graphData')):
            try:
                os.makedirs(os.path.join(os.path.dirname(filePath),'graphData'))
            except:
                print('Cannot create output file directory ' + os.path.dirname(filePath))
                sys.exit(0)

        ### Remove old graphData files at start
        for root, subFolders, files in os.walk(os.path.join(os.path.dirname(filePath), 'graphData')):
            for file in files:
                os.remove(os.path.join(root,file))
            
        # Generate HTML File
        #self._createReportHTML(filePath, lazy_reload=False)
        self._createReportHTML(filePath)

        # Generate additional folders and copy additional files
        additionalFolders = self._template.GetFoldersInDirectory()

        for currAdditionalFolder in additionalFolders:
            copytree(currAdditionalFolder, os.path.join(os.path.dirname(filePath), os.path.basename(currAdditionalFolder)))
            
    def SectionBegin(self, sectionLabel,anchor=[], additional_html=[]):
        self._content.append({"type": "SectionBegin", "Label": sectionLabel,"anchor": anchor,"additional_html": additional_html})

    def SectionEnd(self):
        self._content.append({"type": "SectionEnd"})

    def DivisionBegin(self, classStr):
        self._content.append({"type": "DivisionBegin", "class": classStr})

    def DivisionEnd(self):
        self._content.append({"type": "DivisionEnd"})

    def AddText(self, text):
        if not type(text) is str:
            logger.exception("Wrong argument type: " + str(type(text)))
        self._content.append({"type": "Text", "TextContent": text})

    def AddTable(self, table):
        if not type(table) is cHTMLReportTable:
            logger.exception("Wrong argument type: " + str(type(table)))
        self._content.append({"type": "Table", "TableObject": table})

    def AddGraph(self, graph):
        if not type(graph) is cHTMLReportGraph:
            logger.exception("Wrong argument type: " + str(type(graph)))
        self._content.append({"type": "Graph", "GraphObject": graph})

    def AddChart(self, chart):
        if not type(chart) is cHTMLReportBarChart:
            logger.exception("Wrong argument type: " + str(type(chart)))
        self._content.append({"type": "Chart", "ChartObject": chart})

    def AddVideo(self, video):
        if not type(video) is cHTMLReportVideo:
            logger.exception("Wrong argument type: " + str(type(video)))
        self._content.append({"type": "Video", "VideoObject": video})
    
    def AddImage(self, img):
        if not type(img) is cHTMLReportImage:
            logger.exception("Wrong argument type: " + str(type(img)))
        self._content.append({"type": "Image", "ImageObject": img})
        
    def AddLink(self, link):
        """"
        Adds a link to the content list
        """
        if not type(link) is cHTMLReportLink:
            logger.exception("Wrong argument type: " + str(type(link)))
        self._content.append({"type": "Link", "LinkObject": link})

    def GetLink(self, link):
        """"
        Directly returns a html-link-string
        """
        if not type(link) is cHTMLReportLink:
            logger.exception("Wrong argument type: " + str(type(link)))
        linkData = {"path": link.path, "text": link.text,"prefix": link.prefix, "title": link.title}
        return self._template.CreateLink(linkData)

    def AddHTMLCode(self, code):
        return self._content.append({"type": 'Code', "CodeObject": code})

    def AddColorScale(self, Scale):
        if not type(Scale) is cHTMLReportScale:
            logger.exception("Wrong argument type: " + str(type(Scale)))
        self._content.append({"type": "ColorScale", "ColorScale": Scale})

    def AddImageGraph(self, graph):
        if not type(graph) is cHTMLReportImageGraph:
            logger.exception("Wrong argument type: " + str(type(graph)))

        self._content.append({"type": "ImageGraph", "ImageGraphObject": graph})

    def sectionnr(self):
        # This function returns the total number of sections in the report.
        contentList = self._content
        countSection = 1
        for currItem in contentList:
            if currItem['type'] == 'SectionBegin':
                countSection += 1
        return countSection
