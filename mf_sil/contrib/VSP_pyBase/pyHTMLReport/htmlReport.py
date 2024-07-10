#====================================================================
# System Imports
#====================================================================
import os
import sys
import copy
import shutil
import logging
logger = logging.getLogger("pyBase")
from win32api import GetSystemMetrics

#====================================================================
# Own Imports
#====================================================================
try:
    from pyBase.pyHTMLReport.template_contimodern import htmlReport_template
except(ImportError):
    from template_contimodern import htmlReport_template

template = os.path.join(os.path.dirname(htmlReport_template.__file__), "htmlReport_template.py")
#====================================================================
# Own Imports
#====================================================================
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
    """
    Class that defines a table cell for the html report.

    :param text: Text that shall be shown in the table cell
    :param textColor: Color of the text. Can be given as "red", "green", "blue" or any rgb values (0,0,0)
    :param textAlign: Align of the text: "left", "middle", "right". Default: left.
    :param textStyle: Style of the text
    :param backgroundColor: Background color of the table cell. Can be given as "red", "green", "blue" or any rgb values (0,0,0)
    :param tooltip: String with tooltip
    :param colSpan: Number of columns the table cell shall be wide. Default: 1
    """

    def __init__(self, text=None, textColor=None, textAlign=None, textStyle=None, backgroundColor=None, tooltip=None, colSpan=None, classStr =""):
        self.text            = text
        self.tooltip         = tooltip
        self.textColor       = textColor
        self.textAlign       = textAlign
        self.textStyle       = textStyle
        self.backgroundColor = backgroundColor
        self.colSpan         = colSpan
        self.classStr        = classStr

    def propagateStyle(self, textColor, textAlign, textStyle, backgroundColor):
        """
        Apply given attributes to table cell.

        :param textColor: Color of the text. Can be given as "red", "green", "blue" or any rgb values (0,0,0)
        :param textAlign: Align of the text: "left", "middle", "right". Default: left.
        :param textStyle: Style of the text
        :param backgroundColor: Background color of the table cell. Can be given as "red", "green", "blue" or any rgb values (0,0,0)
        """

        if self.textColor       is None: self.textColor       = textColor
        if self.textAlign       is None: self.textAlign       = textAlign
        if self.textStyle       is None: self.textStyle       = textStyle
        if self.backgroundColor is None: self.backgroundColor = backgroundColor

class cHTMLReportTableRow(object):
    """
    Class that defines a table row for the html report. This object consists of table cells.

    :param cells: List of table cells (cHTMLReportTableCell)
    :param textColor: Color of the text. Can be given as "red", "green", "blue" or any rgb values (0,0,0)
    :param textAlign: Align of the text: "left", "middle", "right". Default: left.
    :param textStyle: Style of the text
    :param backgroundColor: Background color of the table cell. Can be given as "red", "green", "blue" or any rgb values (0,0,0)
    """

    def __init__(self, cells=[], textColor=None, textAlign=None, textStyle=None, backgroundColor=None, classStr ="" ):
        self.cells           = cells
        self.textColor       = textColor
        self.textAlign       = textAlign
        self.textStyle       = textStyle
        self.backgroundColor = backgroundColor
        self.classStr        = classStr

        # Shortcut for handling the row type: If the user just passes a string it is converted to a cell
        for idx,currCell in enumerate(self.cells):
            if type(currCell) is str:
                self.cells[idx] = cHTMLReportTableCell(text=currCell)
            elif type(currCell) is cHTMLReportTableCell:
                pass
            else:
                logger.exception("Wrong argument type: " + str(type(currCell)))

    def addCell(self, cell):
        """
        Add a table cell to a table row.

        :param cells: Table cell (cHTMLReportTableCell) that shall be added
        """

        self.cells.append(cell)

    def propagateStyle(self, textColor, textAlign, textStyle, backgroundColor):
        """
        Apply given attributes to table cell.

        :param textColor: Color of the text. Can be given as "red", "green", "blue" or any rgb values (0,0,0)
        :param textAlign: Align of the text: "left", "middle", "right". Default: left.
        :param textStyle: Style of the text
        :param backgroundColor: Background color of the table cell. Can be given as "red", "green", "blue" or any rgb values (0,0,0)
        """

        if self.textColor       is None: self.textColor       = textColor
        if self.textAlign       is None: self.textAlign       = textAlign
        if self.textStyle       is None: self.textStyle       = textStyle
        if self.backgroundColor is None: self.backgroundColor = backgroundColor

        for currCell in self.cells:
            try:
                currCell.propagateStyle(self.textColor, self.textAlign, self.textStyle, self.backgroundColor)
            except:
                print(('Error: wrong type +' + str(currCell) + ' ' + str(type(currCell))))
            
class cHTMLReportTable(object):
    """
    Class that defines a table for the html report. This object consists of table rows.

    :param label: Text that will be visible at the bottom of the table
    :param headerRow: Header row (cHTMLReportTableRow) of the table
    :param rows: List of rows of the table
    :param textColor: Color of the text. Can be given as "red", "green", "blue" or any rgb values (0,0,0)
    :param textAlign: Align of the text: "left", "middle", "right". Default: left.
    :param textStyle: Style of the text
    :param backgroundColor: Background color of the table cell. Can be given as "red", "green", "blue" or any rgb values (0,0,0)
    :param headerColSpanRows: Number of columns one cell of the header row shall be wide. Default: 1
    :param sortable: If True the table is sortable, if false it is not
    :param width: Width of the table in pixels. Default: Adapt to screen width
    """

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
        """
        Set a header row for the table.

        :param headerRow: Header row (cHTMLReportTableRow) of the table
        """

        self.headerRow = headerRow

    def addHeaderDescription(self, headerColSpan):
        """
        Set a description row above the header row for the table.

        :param headerColSpan: Ex: addHeaderDescription(cHTMLReportTableRow(cells=[cHTMLReportTableCell(text="HR",colSpan="2"), cHTMLReportTableCell(text="Test",colSpan="2")])) \
        which will produce a description row above the header row with two cells "HR" and "Test". Both of them are two columns wide.
        """

        self.headerColSpanRows.append(headerColSpan)

    def addRow(self, row):
        """
        Add a table row.

        :param row: Table row (cHTMLReportTableRow) that shall be added
        """

        self.rows.append(row)

    def propagateStyle(self):
        """
        Apply the attributes textColor, textAlign, textStyle and backgroundColor to all rows of the table so that it looks uniformly \
        even if some rows or cells have configured differently.
        """

        self.headerRow.propagateStyle(self.textColor, self.textAlign, self.textStyle, self.backgroundColor)
        for currRow in self.rows:
            currRow.propagateStyle(self.textColor, self.textAlign, self.textStyle, self.backgroundColor)

#====================================================================
# HTML Report Graph
#====================================================================
class cHTMLReportGraphYData(object):
    """
    Class that defines data for vertical axis of a graph for the html report.

    :param dataPoints: List of data values for vertical axis.
    :param name: Name of the signal
    :param lineColor: Color of the line in rgb format (r,g,b)
    :param lineWidth: Width of the line in pixels
    :param lineType: Type of the line. If None is given, it will connect all points with a line. If "drawPoints" is given, it will only plot the points.
    :param axisAssign: Axis the y data belong to ("y1" or "y2")
    :param yAxisLabel: Label for the y axis
    :param legendLabel: Label that is shown in the graph's legend
    :param NoInterpBetwDataPoints: Dygraph will automatically connect two given points, if lineType is not "drawPoints". If False, it will connect two points following points \
    by a straight line. If True, it will draw the line so that the value stays the same until the next point is given (no straight line but "stairs" or "plains").
    """

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
    """
    Class that defines a graph for the html report.

    :param xData: List of x values
    :param yData: List of objects of type cHTMLReportGraphYData
    :param xDataName: Label for horizontal axis
    :param title: Title for the graph
    :param lineWidth: Width of the lines in the graph. It is recommended to define this parameter in the cHTMLReportGraphYData object instead.
    :param syncGroup: Name of the group of graphs for that the zoom in the x axis shall be synchronized. The name is a string and every graph with the same \
    sync group name will be synchronized.
    :param width: Width of the graph in pixels. If no width is given, 80 percent of 1920 pixels are assumed which fits most screen resolutions.
    :param height: Height of the graph in pixels.
    :param xLim: List of limits of the horizontal axis, e.g. [0, 10] which means the horizontal axis will show values from 0 to 10
    :param yLim: List of limits for the vertical axis, e.g. [0, 20] which means the vertical axis will show values from 0 to 20
    :param xLabel: Label for the horizontal axis
    :param yLabel: Label for the vertical axis
    :param vertLines: Draw vertical lines in the graph, e.g. to visualize the time for a specific event. List of objects of class cHTMLReportVerticalLine.
    :param gridAlignAxis: Relevant if more than one vertical axis is shown. Choose to which vertical axis the grid shall be aligned. Can be "y1" or "y2".
    :param highlightedIntervall: Highlights an intervall in the graph. [5, 12] will highlight from x=5 to x=12. [None, 12] will highlight from the beginning until x=12. \
    [5, None] will highlight from x=5 until the end.
    :param helpstring: Help string for more information about the graph.
    :param legendLocation: Location of the legend for the graph. Can be "inside", "below" or "right".
    :param rangeselector: If True, there will be a range selector at the bottom of the graph to choose the displayed range of the horizontal axis by a cursor. If False \
    normal zoom in the graph itself is possible.
    :param dataReduction: If True, it will reduce the amount of data written in the js file which contains the data values that will be displayed by intelligently eliminating \
    measurement values in which the value stays the same. Can increase the performance for huge amou.t of data. If False, full measurement data are written to the js file.
    """

    def __init__(self, xData, yData=[], xDataName=None, title=None, lineWidth=None, syncGroup=None, width=GetSystemMetrics(0)*0.8, height=200, xLim=None, yLim=None, xLabel=None, yLabel=None, vertLines=None, gridAlignAxis=None, highlightedIntervall=[None,None] , helpstring=None, legendLocation = 'right', rangeselector = False, dataReduction = False):
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
            # assuming 1920 x 1080 resolution
            self.width = GetSystemMetrics(0)*0.8
            #app = wx.App(False)
            #width, height = wx.GetDisplaySize()
            #self.width = width*0.8

    def addYDataSeries(self, yDataSeries):
        """
        Add a data series to the graph.

        :param yDataSeries: Object of class cHTMLReportGraphYData.
        """

        self.yData.append(yDataSeries)

class cHTMLReportBarChart(object):
    def __init__(self, title=None, project=None, width=100, height=100, data=""):
        self.title      = title
        self.width      = width
        self.height     = height
        self.data       = data
        self.project    = project

class cHTMLReportVideo(object):
    """
    Class that defines a video for the html report.

    :param title: Title of the video
    :param videoFileName: Absolute path to the video (mp4 supported)
    :param imgFileName: Absolute path to the thumbnail image for the video
    """

    def __init__(self, title, videoFileName, imgFileName):
        self.title = title
        self.fileName = videoFileName
        self.imgFileName = imgFileName

class cHTMLReportImage(object):
    """
    Class that defines an image for the html report.

    :param path: Absolute path to the image
    :param title: Title of the image
    :param width: Width of the image in pixels
    :param height: Height of the image in pixels
    """

    def __init__(self, path, title=None, width=100, height=100 ):
        self.title      = title
        self.width      = width
        self.height     = height  
        self.path       = path  
        
class cHTMLReportLink(object):
    """
    Class that defines a link for the html report.

    :param path: Address the link shall refer to
    :param text: Text for the link
    :param prefix: Text to show in the same line in front of the link's text
    :param title: Title for the link
    """

    def __init__(self, path, text, prefix = '\n', title = None ):
        self.prefix     = prefix
        self.text       = text
        self.path       = path
        self.title      = title
        
class cHTMLReportVerticalLine(object):
    """
    Class that defines a vertical line for a graph of the html report.

    :param sigName: Name of the vertical line
    :param xCoord: Coordinate on the horizonal axis where the vertical line shall be shown
    :param color: Color of the vertical line, e.g. "yellow", "red".
    """

    def __init__(self, sigName, xCoord, color):
        self.sigName = sigName
        self.xCoord = xCoord 
        self.color = color     

class cHTMLReportCode(object):
    """
    Class that defines user-defined html code to be inserted in the html report.

    :param code: HTML code that shall be inserted in the page.
    """

    def __init__(self, code):
        self.code = code


class cHTMLReportImageGraph(object):
    def __init__(self, xData, yData, xDataName=None, helpstring=None, imageFolderName=None, title=None, width=GetSystemMetrics(0)*0.8, height=200, xLabel=None, yLabel=None, vertLines=None, legendLocation = 'right', dataReduction = False, cmPrjDir=None):
        self.xData      = xData
        self.yData      = yData
        self.xLabel     = xLabel
        self.yLabel     = yLabel
        self.xDataName = xDataName
        self.imageFolderName  = imageFolderName
        self.title      = title
        self.width      = width
        self.height     = height
        self.vertLines  = vertLines
        self.legendLocation       = legendLocation
        self.dataReduction        = dataReduction
        self.helpstring = helpstring
        self.cmPrjDir = cmPrjDir

#====================================================================
#HTML Color Scale
#====================================================================
class cHTMLReportScale(object):
    """
    Class to set the color scale for the html report.

    :param value: Scale value
    :param start: Start value
    :param end: End value
    :param step: Step size
    :param mode: Mode for the scale. Can be "ascending" or "descending"
    """

    def __init__(self,value,start=0,end=10,step=1,mode='ascending'):
        self.start = start
        self.end   = end
        self.step  = step
        self.value = value
        self.mode  = mode


#====================================================================
# HTML Report
#====================================================================
class cHTMLReport(object):
    """
    Class to defines the html report itself.

    :param title: Title of the html report
    :param theme: Theme that shall be used. Currently only "contimodern"
    :param toc: Tabe of contents. If True, it will be shown. If False, it will not be shown.
    """

    def __init__(self, title="", theme="contimodern", toc=False, project_name='ARS540BW11'):
        self._title   = title
        self._content = []
        self._toc     = toc
        self.project_name = project_name

        # Load Template
        themePath      = os.path.dirname(__file__) + "/template_" + theme +"/htmlReport_template.py"
        # cfgImport      = imp.load_source('currTemplate', template )
        self._template = htmlReport_template.cHTMLReportTemplate()

    def _createAdditionalFolders(self, folder):
        """
        If additional directory paths have to be created for a specific theme, call this function to create them.
        """

        additionalFiles = self._template.GetAdditionalFilePaths()

    def _createReportHTML(self, filePath, lazy_reload=True):
        """
        Creates the html report by collecting all items that were added.

        :param filePath: Absolute path to the location the report shall be saved to
        :param lazy_reload: If True, only parts of the report will be loaded by the browser, if they are visible to the user If False, the whole report will be \
        loaded at the beginning. It is strongly recommended to use lazy_reload since it improves the performance and stability of the report especially if graphs \
        are used dramatically.
        """
        graphFlag = False
        if not os.path.exists(os.path.dirname(filePath)):
            try:
                os.makedirs(os.path.dirname(filePath))
            except:
                print(('Cannot create output file directory ' + os.path.dirname(filePath)))
                sys.exit(0)
               
        tableCount = 0
        initiation_count=0
        with open(filePath, "w", encoding='utf-8') as reportFile:

            if (lazy_reload == False):
                reportFile.write(self._template.CreateHeader(self._title, False, filePath, self.project_name))
            else:
                reportFile.write(self._template.CreateHeader(self._title, True, filePath, self.project_name))
            reportFile.write(self._template.CreateInitButtons())
            if self._toc:
                reportFile.write(self._template.CreateSectionBegin("Table of Contents"))
                reportFile.write(self._template.CreateTOC(self._content))
                reportFile.write(self._template.CreateSectionEnd())



            # Process Content List
            for currContentItem in self._content:
                if currContentItem["type"] == "SectionBegin":
                    reportFile.write(self._template.CreateSectionBegin(currContentItem["Label"],anchor=currContentItem["anchor"],classStr = currContentItem["classStr"]))
                elif currContentItem["type"] == "SectionEnd":
                    reportFile.write(self._template.CreateSectionEnd())
                elif currContentItem["type"] == "Text":
                    reportFile.write(self._template.CreateTextSection(currContentItem["TextContent"]))
                elif currContentItem["type"] == "Table":
                    # Table Begin
                    table = currContentItem["TableObject"]
                    table.propagateStyle()

                    reportFile.write(self._template.CreateTableBegin(table.label, tableCount=tableCount, sortable=table.sortable, width=table.width, table_num=initiation_count))

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
                        reportFile.write(self._template.CreateTableHeaderEnd(tableCount=tableCount))
                    is_last_row = 0
                    row_Count = 0
                    # Table Rows
                    for currRow in table.rows:
                        row_Count = row_Count + 1
                        if row_Count == len(table.rows):
                            is_last_row = 1
                        c = 0
                        for currCell in currRow.cells:
                            c = c+1
                            if c ==1:
                                reportFile.write(self._template.CreateTableRowBegin(currRow.classStr))
                            reportFile.write(self._template.CreateTableRowCell(text            = currCell.text,
                                                                               textColor       = currCell.textColor,
                                                                               textAlign       = currCell.textAlign,
                                                                               textStyle       = currCell.textStyle,
                                                                               backgroundColor = currCell.backgroundColor,
                                                                               tooltip         = currCell.tooltip,
                                                                               tableCount       =tableCount,
                                                                               is_last_row  = is_last_row))
                        reportFile.write(self._template.CreateTableRowEnd())
                    # Table End
                    reportFile.write(self._template.CreateTableEnd(tableCount=tableCount))

                    if filePath[9:14] == "GIDAS" and initiation_count == 0:  #if test for GIDAS then it is having onlt 2 table so we setting the table count as 3 for scroller.
                        tableCount = 7
                        initiation_count = 1
                    elif filePath[9:12] == "AMP" and initiation_count == 0:  #if test for GIDAS then it is having onlt 2 table so we setting the table count as 3 for scroller.
                        tableCount = 7
                        initiation_count = 1

                    # elif filePath.split("\\")[2] == "Pre_analysis" \
                    #         and initiation_count == 0:  #Pre-Analysis report has 1 table so we setting the table count as 1 for scroller.
                    #     tableCount = 7
                    #     initiation_count = 1

                    else:                                                    #for other tests table count will increase one by one.
                        tableCount += 1
                        initiation_count += 1                               #initiation count for the tabel data setting table count in $(this(1))

                elif currContentItem["type"] =="ColorScale":
                    Scale= currContentItem["ColorScale"]
                    reportFile.write(self._template.generateScale(Scale))
                elif currContentItem["type"] == "Graph":
                    g = currContentItem["GraphObject"]
                    yData = []
                    try:
                        for currYDataSeries in g.yData:
                            #print("currYDataSeries.name is ", currYDataSeries.name)
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
                        #print("graphData is ", graphData)
                        reportFile.write(self._template.CreateGraph(graphData, filePath))
                    except:
                        #print("currYDataSeries.name is in exception is ", currYDataSeries.name)
                        pass
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
        """
        Save the html report to the given path.

        :param filePath: Absolute directory path where the html report shall be saved.
        """

        # Create graphData directory
        if not os.path.exists(os.path.join(os.path.dirname(filePath),'graphData')):
            try:
                os.makedirs(os.path.join(os.path.dirname(filePath),'graphData'))
            except:
                print(('Cannot create output file directory ' + os.path.dirname(filePath)))
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
            
    def SectionBegin(self, sectionLabel,anchor=[], additional_html=[], classStr = ""):
        """
        Add the start of a section in the html report.

        :param sectionLabel: Name of the section
        :param anchor: Anchor of the section
        """

        self._content.append({"type": "SectionBegin", "Label": sectionLabel,"anchor": anchor,"additional_html": additional_html, "classStr": classStr})

    def SectionEnd(self):
        """
        Add the end of a section in the html report.
        """

        self._content.append({"type": "SectionEnd"})

    def AddText(self, text):
        """
        Add a text to the html report.

        :param text: Test string that shall be added
        """

        if not type(text) is str:
            logger.exception("Wrong argument type: " + str(type(text)))
            
        self._content.append({"type": "Text", "TextContent": text})

    def AddTable(self, table):
        """
        Add a table to the html report.

        :param table: Object of class cHTMLReportTable
        """

        if not type(table) is cHTMLReportTable:
            logger.exception("Wrong argument type: " + str(type(table)))
            
        self._content.append({"type": "Table", "TableObject": table})


    def AddGraph(self, graph):
        """
        Add a graph to the html report.

        :param graph: Object of class cHTMLReportGraph
        """

        if not type(graph) is cHTMLReportGraph:
            logger.exception("Wrong argument type: " + str(type(graph)))
                    
        self._content.append({"type": "Graph", "GraphObject": graph})

    def AddChart(self, chart):
        if not type(chart) is cHTMLReportBarChart:
            logger.exception("Wrong argument type: " + str(type(chart)))
        self._content.append({"type": "Chart", "ChartObject": chart})

    def AddVideo(self, video):
        """
        Add a video to the html report.

        :param video: Object of class cHTMLReportVideo
        """

        if not type(video) is cHTMLReportVideo:
            logger.exception("Wrong argument type: " + str(type(video)))
                    
        self._content.append({"type": "Video", "VideoObject": video})
    
    def AddImage(self, img):
        """
        Add an image to the html report.

        :param video: Object of class cHTMLReportImage
        """

        if not type(img) is cHTMLReportImage:
            logger.exception("Wrong argument type: " + str(type(img)))
                    
        self._content.append({"type": "Image", "ImageObject": img})
        
    def AddLink(self, link):
        """
        Add a link to the html report.

        :param video: Object of class cHTMLReportLink
        """

        if not type(link) is cHTMLReportLink:
            logger.exception("Wrong argument type: " + str(type(link)))
                    
        self._content.append({"type": "Link", "LinkObject": link})

    def GetLink(self, link):
        """
        Return a html string to insert a link.

        :param link: Object of class cHTMLReportLink

        :return: HTML string for the link
        """

        if not type(link) is cHTMLReportLink:
            logger.exception("Wrong argument type: " + str(type(link)))
        linkData = {"path": link.path, "text": link.text,"prefix": link.prefix, "title": link.title}
        return self._template.CreateLink(linkData)

    def AddHTMLCode(self, code):
        """
        Add html code to the html report.

        :param code: String of html code that shall be inserted
        """

        return self._content.append({"type": 'Code', "CodeObject": code})

    def AddColorScale(self, Scale):
        """
        Add a color scale to the html report.

        :param Scale: Object of class cHTMLReportScale
        """
        if not type(Scale) is cHTMLReportScale:
            logger.exception("Wrong argument type: " + str(type(Scale)))
            
        self._content.append({"type": "ColorScale", "ColorScale": Scale})

    def AddImageGraph(self, graph):
        if not type(graph) is cHTMLReportImageGraph:
            logger.exception("Wrong argument type: " + str(type(graph)))

        self._content.append({"type": "ImageGraph", "ImageGraphObject": graph})

    def sectionnr(self):
        contentList = self._content
        countSection = 1
        for currItem in contentList:
            # print currItem
            if currItem['type'] == 'SectionBegin':
                countSection += 1
        return countSection




