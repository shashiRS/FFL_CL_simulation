# ====================================================================
# System Imports
# ====================================================================
import os
import sys
from collections import OrderedDict
from shutil import copy
import re
import numpy as np
import random
import math

# ====================================================================
# Simulation Imports
# ====================================================================
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from pyBase.pyHTMLReport.htmlReport_template_base import cHTMLReportTemplateBase

FILE_NAME_HEADER = "header.txt"
FILE_NAME_FOOTER = "footer.txt"
Tabelid = "Tabel"

try:
    FOLDER_SCRIPTS = os.path.join('./input', 'report', 'scripts')
    FOLDER_GFX = os.path.join('./input', 'report', 'gfx')
    FOLDER_VIDEOS = os.path.join('./input', 'report', 'video')
    open(os.path.join('./input', 'report', FILE_NAME_HEADER), "r")
except:
    FOLDER_SCRIPTS = os.path.join(os.path.dirname(__file__), 'scripts')
    # print(f"FOLDER_SCRIPTS: {FOLDER_SCRIPTS}")
    FOLDER_GFX = os.path.join(os.path.dirname(__file__), 'gfx')
    FOLDER_VIDEOS = os.path.join(os.path.dirname(__file__), 'video')
# print(f"FOLDER_GFX: {FOLDER_GFX}")

MARGIN_FACTOR_LEFT = 100
TOC_MARGIN_FACTOR_LEFT = 30

# CONTI_COLORS = [(191,115,0), (226,135,0), (255,194,102), (255,165,0)]
COLORS = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0)]
HIGHLIGHT_COLOR_RBGA = "rgba(255,255,255, 0.05)"


def GetFilesInFolder(folder):
    result = []
    for root, subFolders, files in os.walk(folder):
        for file in files:
            result.append(os.path.join(root, file))
    return result


class cHTMLReportTemplate(cHTMLReportTemplateBase):
    def __init__(self):
        self._currSectionLevel = 0
        self._currSectionCount = 0
        self._graphCount = 0
        self._maxSectionCount = 1000
        self.Scale_counter = 0
        self._birdEyeData = OrderedDict()
        self._carmaker_initial_image = OrderedDict()
        self._birdeye_initial_image = OrderedDict()
        self._graphId = []
        self.start_time = OrderedDict()
        self.end_time = OrderedDict()

    def GetAdditionalFilePaths(self):
        filePaths = []
        gfxFiles = GetFilesInFolder(FOLDER_GFX)
        for currGfxFilePath in gfxFiles:
            filePaths.append({"folder": "gfx", "path": currGfxFilePath})
        scriptFiles = GetFilesInFolder(FOLDER_SCRIPTS)
        for currScriptFilePath in scriptFiles:
            filePaths.append({"folder": "scripts", "path": currScriptFilePath})
        videoFiles = GetFilesInFolder(FOLDER_VIDEOS)
        for currVideoFilePath in videoFiles:
            filePaths.append({"folder": "video", "path": currVideoFilePath})
        return filePaths

    def GetFoldersInDirectory(self):
        dirPaths = []
        gfxFiles = dirPaths.append(str(FOLDER_GFX))
        scriptFiles = dirPaths.append(str(FOLDER_SCRIPTS))
        videoFiles = dirPaths.append(str(FOLDER_VIDEOS))
        return dirPaths

    def CreateHeader(self, title, lazy_reload=True):
        try:
            headerFile = open(os.path.join('./input', 'report', FILE_NAME_HEADER), "r")
        except:
            headerFile = open(os.path.join(os.path.dirname(__file__), FILE_NAME_HEADER), "r")
        headerFileContent = headerFile.read()
        if (lazy_reload == False):
            headerFileContent = headerFileContent.replace("var lazy_reload = 1;", "var lazy_reload = 0;")
        return headerFileContent.replace("{title}", title)

    def CreateSectionBegin(self, sectionLabel, anchor=[]):
        self._currSectionLevel += 1
        self._currSectionCount += 1

        #   If the user wishes to set an anchor to the Section, an empty section
        #   with the specified anchor is added
        anchorstring = ''
        if anchor is not []:
            anchorstring = '<div id={0}></div>'.format(anchor)

        sectionBeginString = ''
        sectionBeginString += '<dt>'
        sectionBeginString += '<div id="secExpander{sectionID}">{anchorstring}<script>jQuery(function() \
                              {$("#secExpander{sectionID}").click(function() \
                              {$("#divSection{sectionID}").toggle(); \
                              if($("#divSection{sectionID}").is(":visible")){$("#imgExpander{sectionID}").attr("src","gfx/toggle_minus.png"); \
                              }else{$("#imgExpander{sectionID}").attr("src","gfx/toggle_plus.png");}});});</script> \
                              <h2 style="margin-left: {marginLeft}px;"><a name="Section{sectionID}"><img id="imgExpander{sectionID}" style="width: 16px; height: 16px; float: left;" alt="" src="gfx/toggle_minus.png"> &nbsp;{label}</a></h2> \
                              <div id="divSection{sectionID}">'.replace("{sectionID}", str(self._currSectionCount)) \
            .replace("{label}", sectionLabel) \
            .replace("{marginLeft}", str(MARGIN_FACTOR_LEFT * (self._currSectionLevel - 1))) \
            .replace("{anchorstring}", anchorstring)
        sectionBeginString += '</dt></div>'
        sectionBeginString += '<dd><dl>\n'

        return sectionBeginString

    def AddDivision(self, classStr):
        return '<dx class={classStr}>'.replace("{classStr}", classStr)

    def EndDivision(self):
        return '</dx>'

    def CreateSectionEnd(self):
        self._currSectionLevel -= 1
        return '</dl></dd>\n'

    def CreateTOC(self, contentList):
        TOCcontent = ''
        countSection = 1
        countSectLev = 0
        for currItem in contentList:
            if currItem['type'] == 'DivisionBegin':
                TOCcontent += f'<div class={currItem["class"]}>\n'
            if currItem['type'] == 'SectionBegin':
                countSection += 1
                countSectLev += 1
                TOCcontent += '<div style="margin-left: {marginLeft}px;"><a onclick="open_section(\'#divSection{SectionNr}\')" href="#Section{SectionNr}">{textContent}</a></div><br>' \
                    .format(marginLeft=TOC_MARGIN_FACTOR_LEFT * countSectLev, SectionNr=countSection,
                            textContent=currItem['Label'])
            if currItem['type'] == 'SectionEnd':
                countSectLev -= 1
            if currItem['type'] == 'DivisionEnd':
                TOCcontent += '\n</div>'
        return TOCcontent

    def CreateInitButtons(self):
        """
        Adds jQuery to the html report.
        """
        # %copy & paste from html_report.py
        SCRIPT = """
        $(document).ready(function(){
            STATE_HIDDEN_ALL = 0;
            STATE_SHOW_ALL = 1;
            STATE_MIXED = 2;
            state = STATE_MIXED;
            initButtons();
            hide_all_sections();
            ///buildToc()
        });

        function initButtons(){
            $("dt").click(function(){ // trigger 
                $(this).next("dd").slideToggle("slow");
                $(this).find("input").toggleClass("opened closed");
                state = STATE_MIXED;
                /* Invoke scroll event to trigger inview-event of dygraphs */
                window.scrollBy(0,1);
                window.scrollBy(0,-1);
                return false;
            });

            $("#show_all").click(function(){
                show_all_sections();
            });

            $("#hide_all").click(function(){
                hide_all_sections();
            });
        }
        """.replace('{maxSectionCount}', str(self._maxSectionCount))

        return '<script>\n' + SCRIPT + '\n</script>\n' + '<input type="submit" class="Buttons" value="Show all" id="show_all" />' + '\n' + '<input type="submit" class="Buttons" value="Hide all" id="hide_all" />' + '\n'

    def CreateFooter(self):
        try:
            footerFile = open(os.path.join('./input', 'report', FILE_NAME_FOOTER), "r")
        except:
            footerFile = open(os.path.join(os.path.dirname(__file__), FILE_NAME_FOOTER), "r")
        return footerFile.read()

    def CreateTextSection(self, text):
        text = text.replace("\n", "<br>")
        textSection = '<div style="margin-left: {marginLeft}px; color: #FFFFFF; font-size:12px">{textContent}</div><br>'.format(
            marginLeft=MARGIN_FACTOR_LEFT * self._currSectionLevel, textContent=text)
        return textSection

    def CreateTableBegin(self, caption="", tableCount=0, sortable=True, width=None):
        tableBegin = '<script> $(document).ready(function() {$("#table{tableCount}").DataTable({"lengthMenu": [[-1, 10, 25, 50], ["All", 10, 25, 50]], paging: false, "aaSorting": [], '.replace(
            '{tableCount}', str(tableCount))
        if not sortable:
            tableBegin += '"bSort": false, '
        tableBegin += 'bFilter: false, bInfo: false});});</script>'
        tableBegin += '<br><table id="table{tableCount}" class="niceTable" width="{width}" border="0" cellpadding="0" cellspacing="0"><caption align="bottom" style="color: #FFFFFF">{tableCaption}</caption>'.format(
            marginLeft=MARGIN_FACTOR_LEFT * self._currSectionLevel, tableCaption=caption, tableCount=tableCount,
            width=width)
        return tableBegin

    def CreateTableEnd(self):
        return '</tbody></table><br>'

    def CreateTableHeaderBegin(self):
        return '<thead>'

    def CreateTableHeaderCell(self, text=None, textColor=None, textAlign=None, textStyle=None, backgroundColor=None):
        return self.CreateTableHeaderRowCell(text, textColor, textAlign, textStyle, backgroundColor)

    def CreateTableHeaderEnd(self):
        return '</thead><tbody>'

    def CreateTableRowBegin(self, classStr=''):
        return f'<tr class={classStr}>'

    def CreateTableHeaderRowCell(self, text=None, textColor=None, textAlign=None, textStyle=None, backgroundColor=None,
                                 tooltip=None):
        styleString = ""
        if textColor is not None or textAlign is not None or textStyle is not None or backgroundColor is not None:
            styleString = 'style="'
            if textColor is not None: styleString += "color: rgb({red}, {green}, {blue});".format(red=textColor[0],
                                                                                                  green=textColor[1],
                                                                                                  blue=textColor[2])
            if textAlign is not None: styleString += "text-align: {0};".format(textAlign)
            if textStyle is not None: styleString += "font-weight: {0};".format(textStyle)
            if backgroundColor is not None: styleString += "background-color: rgb({red}, {green}, {blue});".format(
                red=backgroundColor[0], green=backgroundColor[1], blue=backgroundColor[2])
            styleString += '"'

        if tooltip is not None:
            titleString = 'title= "%s"' % tooltip
            cellString = '<th {title} {style}>{content}</th>'.format(title=titleString, style=styleString, content=text)
        else:
            cellString = '<th {style}>{content}</th>'.format(style=styleString, content=text)

        return cellString

    def CreateTableRowCell(self, text=None, textColor=None, textAlign=None, textStyle=None, backgroundColor=None,
                           tooltip=None):
        styleString = ""
        if textColor is not None or textAlign is not None or textStyle is not None or backgroundColor is not None:
            styleString = 'style="'
            if textColor is not None: styleString += "color: rgb({red}, {green}, {blue});".format(red=textColor[0],
                                                                                                  green=textColor[1],
                                                                                                  blue=textColor[2])
            if textAlign is not None: styleString += "text-align: {0};".format(textAlign)
            if textStyle is not None: styleString += "font-weight: {0};".format(textStyle)
            if backgroundColor is not None: styleString += "background-color: rgb({red}, {green}, {blue});".format(
                red=backgroundColor[0], green=backgroundColor[1], blue=backgroundColor[2])
            styleString += '"'

        if tooltip is not None:
            titleString = 'title= "%s"' % tooltip
            cellString = '<td {title} {style}><nobr>{content}</nobr></td>'.format(title=titleString, style=styleString,
                                                                                  content=text)
        else:
            cellString = '<td {style}><nobr>{content}</nobr></td>'.format(style=styleString, content=text)

        return cellString

    def CreateTableHeaderColSpanCell(self, colSpanWidth=None, text=None, textColor=None, textAlign=None, textStyle=None,
                                     backgroundColor=None, tooltip=None):
        if colSpanWidth is not None:
            colSpanString = ""
            colSpanString += '<td colspan="{headerColSpan}">{content}</td>'.format(headerColSpan=str(colSpanWidth),
                                                                                   content=text)

        return colSpanString

    def CreateTableRowEnd(self):
        return '</tr>'

    def getRandomColor(self):
        letters = list('0123456789abcdef')
        color = '#'
        for i in range(6):
            color += random.choice(letters)
        return color

    def CreateBarChart(self, chartData):
        htmlFileString = ''
        htmlFileString += '<center><div class="scroll-bar" id="chartContainer" style="overflow-x:auto; position: relative; height: 430px; width: 900px;"></div></center>'
        htmlFileString += '<br>'
        categories = chartData["data"].keys()
        width = eval('250+len(categories)*20+(math.ceil(len(categories)/21)*80)')
        if width < 900:
            width = 900
        htmlFileString += """ <script>
                window.addEventListener("load",function () {
                    var chart = new CanvasJS.Chart("chartContainer", {
                        animationEnabled: true,
                        width: """ + str(width) + """,
                        title:{
                            text:""" + '"' + chartData["title"] + '"' + """
                        },
                        axisX:{
                                title: """ + '"' + chartData["project"] + '"' + """,
                                valueFormatString: " ",
                                tickLength: 0
                        },
                        axisY: {
                                title: "Number of Passed/Total number of test cases",
                                suffix: "%",
                                minimum: 0,
                                maximum: 120,
                                interval: 20,
                        },
                        toolTip:{
                            shared: false
                        },
                        legend: {
                            reversed: false,
                            verticalAlign: "center",
                            horizontalAlign: "right"
                        },
                        dataPointMaxWidth: 20,
                        data:["""
        # categories = ["failed", "passed", "unknown"]
        categories = chartData["data"].keys()
        labels = ['passed']
        quantString = ""
        for category in categories:
            quantString += """{
                            type: "column",
                            color: '{random_color}',
                            indexLabelFontSize: 12,
                            indexLabelFontColor: "black",
                            indexLabelOrientation: "vertical",
                            indexLabelPlacement: "inside",
                            indexLabelWrap: true,
                            toolTipContent: "<b>{name}:</b> {y} % passed",
                            showInLegend: true, """.replace('{random_color}', self.getRandomColor())
            quantString += 'name: "{category}",'.replace("{category}", category)
            quantString += """ dataPoints: ["""
            labelString = ""
            for label in labels:
                passed = chartData["data"][category][label]
                total = chartData["data"][category]['total']
                yData = str(round((passed * 100) / total, 2)) if total else '0'
                labelString += """ { y: {yData}, indexLabel: "{passed}/{total} {category}" },""".replace("{yData}",
                                                                                                         yData).replace(
                    "{passed}", str(passed)).replace("{total}", str(total)).replace("{category}", category)
            quantString += labelString
            quantString += """]
                            },\n"""
        htmlFileString += quantString
        htmlFileString += """]
                    });
                    chart.render();
                    chart.set("dataPointWidth",Math.ceil(chart.axisX[0].bounds.width/chart.data[0].dataPoints.length),true);
                    })
                </script> """
        htmlFileString += '<br>'
        return htmlFileString
    
    def CreateGraph(self, graphData, filePath):
        if graphData["syncGroup"] is None: graphData["syncGroup"] = "noSyncGroup{0}".format(self._currSectionCount)
        graphID = "graph{0:04d}".format(self._graphCount)
        legendID = "legend{0:04d}".format(self._graphCount)
        graphData["legend"] = 'always'
        vertLinesList = graphData["vertLines"]

        # Declare js-file
        jsFileString = ""

        ### Write Graph Data to file
        jsFileString += 'var vertLines{graphID} = new Array();\n'.replace("{graphID}", graphID)
        if vertLinesList is not None:
            for i, obj in enumerate(vertLinesList):
                jsFileString += 'vertLines{graphID}[{i}] = new Object();\n'.replace("{i}", str(i)).replace("{graphID}",
                                                                                                           graphID)
                jsFileString += 'vertLines{graphID}[{i}]["sigName"] = "{sigName}";\n'.replace("{i}", str(i)).replace(
                    "{sigName}", vertLinesList[i].sigName).replace("{graphID}", graphID)
                jsFileString += 'vertLines{graphID}[{i}]["xCoord"] = {xCoord};\n'.replace("{i}", str(i)).replace(
                    "{xCoord}", str(vertLinesList[i].xCoord)).replace("{graphID}", graphID)
                jsFileString += 'vertLines{graphID}[{i}]["color"] = "{color}";\n'.replace("{i}", str(i)).replace(
                    "{color}", str(vertLinesList[i].color)).replace("{graphID}", graphID)

        currEqualNumberCount = []
        for currYData in graphData["yData"]:
            currEqualNumberCount.append(0)

        jsFileString += 'var GraphData{graphID} = '.replace("{graphID}", graphID)
        jsFileString += "[\n"
        for idx, currX in enumerate(graphData["xData"]):
            currDataLineString = "[{0},\t".format(currX)
            for currYData in graphData["yData"]:
                currDataLineString += "{0},\t".format(currYData["dataPoints"][idx])
            currDataLineString += "],\n"

            # build "plains"
            # see whether there is a graph with NoInterpBetwDataPoints option enabled
            noInterpDesired = False
            if idx < len(graphData["xData"]) - 1:
                for currYData in graphData["yData"]:
                    if currYData["NoInterpBetwDataPoints"]:
                        noInterpDesired = True
                        break
                # if NoInterpBetwDataPoints is enabled, build "plains"
                if noInterpDesired:
                    currDataLineString += "[{0},\t".format(graphData["xData"][idx + 1])
                    for currYData in graphData["yData"]:
                        if currYData["NoInterpBetwDataPoints"]:
                            currDataLineString += "{0},\t".format(currYData["dataPoints"][idx])
                        else:
                            currDataLineString += "{0},\t".format(currYData["dataPoints"][idx + 1])
                    currDataLineString += "],\n"

            jsFileString += currDataLineString
        jsFileString += '\n];\n'

        jsFileString += 'function addGraphToHTML() {\n$("#{GraphID}") .html("\\\n'.replace('{GraphID}', graphID)

        ### Graph Prefix
        jsFileString += '<div style=\'margin-left\': {marginLeft}px;\' id=\'{graphName}\'></div>\\\n<script type=\'text/javascript\'>\\\n'.format(
            marginLeft=MARGIN_FACTOR_LEFT * self._currSectionLevel, graphName=graphID)
        jsFileString += 'if (typeof {syncGroupName} == \'undefined\'){{syncGroupName} = [];}\\\n{syncGroupName}.push(\\\nnew Dygraph(\\\ndocument.getElementById(\'{graphName}\'),\\\n'.replace(
            "{syncGroupName}", graphData["syncGroup"]).replace("{graphName}", graphID)

        ### Call function from extern js-file
        jsFileString += 'getData_{graphID}(),\\\n'.replace("{graphID}", graphID)

        ### Graph Options
        jsFileString += "{\\\n"

        # Labels
        if graphData["xDataName"] is None: graphData["xDataName"] = ""

        # Get max num of y-axis-labels
        maxNumYLabels = 0
        for i, currYData in enumerate(graphData["yData"]):
            if (currYData["yAxisLabel"] is not None):
                if len(currYData["yAxisLabel"]) > maxNumYLabels:
                    maxNumYLabels = len(currYData["yAxisLabel"])

        # Adapt graph height to max number of y-axis-labels
        ##if (maxNumYLabels <= 4):
        for i, currYData in enumerate(graphData["yData"]):
            if currYData["axisAssign"] == 'y2':
                additionalWidth = 56
                break
            else:
                additionalWidth = 0
        jsFileString += 'width: ' + str(graphData["width"] + additionalWidth) + ', height: ' + str(
            graphData["height"]) + ',\\\n'
        ##else:
        ##    jsFileString += 'width: '+str(graphData["width"]) +', height: ' + str(graphData["height"]+40*(maxNumYLabels-4)) + ','
        # else:
        #    jsFileString += 'width: '+str(graphData["width"]) +', height: ' + str(graphData["height"]) + ','

        jsFileString += 'labels: [\'{xDataLabel}\''.format(xDataLabel=graphData["xDataName"])
        for currYData in graphData["yData"]:
            if currYData["name"] is None: currYData["name"] = ""
            jsFileString += ',\'{currYDataLabel}\''.format(currYDataLabel=currYData["name"])
        jsFileString += "],\\\n"

        # animated zoom
        jsFileString += "animatedZooms: true,\\\n"

        # range selector
        if graphData["rangeselector"] is True:
            jsFileString += "showRangeSelector: true,\\\n"
            jsFileString += "rangeSelectorHeight: 35,\\\n"
            jsFileString += "rangeSelectorPlotStrokeColor: 'black',\\\n"
            jsFileString += "rangeSelectorPlotFillColor: '',\\\n"
        # Set special y axis labels
        jsFileString += 'connectSeparatedPoints: true,'
        jsFileString += 'axes:{'
        for i, currYData in enumerate(graphData["yData"]):
            if currYData["axisAssign"] is not None:
                jsFileString += '{yAxis}:{'.replace('{yAxis}', str(currYData["axisAssign"]))
            else:
                jsFileString += '{yAxis}:{'.replace('{yAxis}', 'y')
            if currYData["yAxisLabel"] is not None:
                jsFileString += 'axisLabelFormatter: function({yDataLabel}){'.replace("{yDataLabel}", currYData["name"])
                jsFileString += "if({yDataLabel}>=0 && {yDataLabel}<1) return '{STATE}';".replace("{STATE}", currYData[
                    "yAxisLabel"][str(0)]).replace("{yDataLabel}", currYData["name"])
                for j in range(0, len(currYData["yAxisLabel"])):
                    jsFileString += "else if({yDataLabel}>={j} && {yDataLabel}<{j}+1) return '{STATE}';".replace("{j}",
                                                                                                                 str(
                                                                                                                     j)).replace(
                        "{STATE}", currYData["yAxisLabel"][str(j)]).replace("{yDataLabel}", currYData["name"])
                jsFileString += '},'
            if currYData["legendLabel"] is not None:
                #   valueFormatter-Function:
                #   declares aliases for the values: Instead of numbers, the legend will show names returned by this function
                jsFileString += 'valueFormatter: function({legendLabel}){'.replace("{legendLabel}", currYData["name"])
                jsFileString += "if({legendLabel}>=0 && {legendLabel}<1) return '{STATE}';".replace("{STATE}",
                                                                                                    currYData[
                                                                                                        "legendLabel"][
                                                                                                        str(
                                                                                                            0)]).replace(
                    "{legendLabel}", currYData["name"])
                for j in range(0, len(currYData["legendLabel"])):
                    jsFileString += "else if({legendLabel}>={j} && {legendLabel}<{j}+1) return '{STATE}';".replace(
                        "{j}", str(j)).replace("{STATE}", currYData["legendLabel"][str(j)]).replace("{legendLabel}",
                                                                                                    currYData["name"])
                jsFileString += '},'

            if graphData["gridAlignAxis"] is not None:
                if graphData["gridAlignAxis"] == currYData["axisAssign"]:
                    jsFileString += 'drawGrid: true, independentTicks: true'
                else:
                    jsFileString += 'drawGrid: false, independentTicks: true'
            else:
                jsFileString += 'drawGrid: true, independentTicks: true'
            # else:
            #    jsFileString += 'drawGrid: false'
            jsFileString += '},'
        jsFileString += '},\\\n'

        # jsFileString += 'axes:{'
        # for i,currYData in enumerate(graphData["yData"]):
        #    if currYData["yAxisLabel"] is not None:
        #        if currYData["axisAssign"] is not None:
        #            jsFileString += '{yAxis}:{axisLabelFormatter: function({yDataLabel}){'.replace("{yDataLabel}", currYData["name"]).replace("{yAxis}", currYData["axisAssign"])
        #        else:
        #            #jsFileString += 'y:{axisLabelFormatter: function({yDataLabel}){'.replace("{yDataLabel}", graphData["yData"][0]["name"])
        #            jsFileString += 'y:{axisLabelFormatter: function({yDataLabel}){'.replace("{yDataLabel}", currYData["name"])
        #        jsFileString += "if({yDataLabel}>=0 && {yDataLabel}<1) return '{STATE}';".replace("{STATE}", currYData["yAxisLabel"][str(0)]).replace("{yDataLabel}", currYData["name"])
        #        for j in range(0, len(currYData["yAxisLabel"])):
        #            jsFileString += "else if({yDataLabel}>={j} && {yDataLabel}<{j}+1) return '{STATE}';".replace("{j}", str(j)).replace("{STATE}", currYData["yAxisLabel"][str(j)]).replace("{yDataLabel}", currYData["name"])
        #        jsFileString += '},'
        #        #if graphData["gridAlignAxis"] is not None:
        #            #if graphData["gridAlignAxis"] == currYData["axisAssign"]:
        #        jsFileString += 'drawGrid: true, independentTicks: true'
        #            #else:
        #            #    jsFileString += 'drawGrid: false'
        #        jsFileString += '},'
        # jsFileString += '},'

        # Ranges
        if graphData["xLim"] is not None:
            jsFileString += "dateWindow: [{xMin},{xMax}],\\\n".format(xMin=graphData["xLim"][0],
                                                                      xMax=graphData["xLim"][1])

        if graphData["yLim"] is not None:
            jsFileString += "valueRange: [{yMin},{yMax}],\\\n".format(yMin=graphData["yLim"][0],
                                                                      yMax=graphData["yLim"][1])

        if graphData["xLabel"] is not None:
            jsFileString += 'xlabel: \'%s\',\\\n' % graphData["xLabel"]

        # if len(graphData["yData"])>1:
        #    if graphData["yLabel"] is not None:
        #        jsFileString += 'ylabel: \'%s\',' % graphData["yData"][0]["name"]
        #    if len(graphData["yData"])>1:
        #        jsFileString += 'y2label: \'%s\',' % graphData["yLabel"]
        # else:
        #    if graphData["yLabel"] is not None:
        #        jsFileString += 'ylabel: \'%s\',' % graphData["yLabel"]
        if graphData["yLabel"] is not None:
            jsFileString += 'ylabel: \'%s\',\\\n' % graphData[
                "yLabel"]  # <  The label for the first y-axis will always be derived from
            #   the graphs "yLabel"-property
            if currYData["axisAssign"] == "y2":  # <  If however the current line is assigned to a second y-axis,
                jsFileString += 'y2label: \'%s\',\\\n' % currYData[
                    "name"]  # the second y-label is derived from the Name-Property of
            #   the line.

        if graphData["legend"] is not None:
            jsFileString += 'legend: \'%s\',\\\n' % graphData["legend"]

        #   put legend outside, if asked to
        if graphData["legendLocation"] == "inside":
            pass
        elif graphData["legendLocation"] == "below":
            jsFileString += "labelsDiv: document.getElementById('{0}_below'), ".format(legendID)
            jsFileString += "labelsSeparateLines: true,\\\n"
        else:  # < default, put it at the right side of the plot (outside)
            jsFileString += "labelsDiv: document.getElementById('{0}_right'), ".format(legendID)
            jsFileString += "labelsSeparateLines: true,\\\n"

        # Colors
        jsFileString += "colors: ["
        currContiColorID = 0
        for currYData in graphData["yData"]:
            if currYData["lineColor"] is None: currYData["lineColor"] = COLORS[currContiColorID]
            jsFileString += '\'rgb({red},{green},{blue})\','.format(red=currYData["lineColor"][0],
                                                                    green=currYData["lineColor"][1],
                                                                    blue=currYData["lineColor"][2])
            currContiColorID = (currContiColorID + 1) % len(COLORS)
        jsFileString += "],\\\n"

        # Strokes
        i = 0
        for currYData in graphData["yData"]:
            jsFileString += "'{currYDataLabel}':".format(currYDataLabel=currYData["name"])
            jsFileString += "{"
            if currYData[
                "lineType"] is not None:  # default/None: data points will be connected to line, otherwise set lineType=drawPoints
                if currYData["lineType"] == "drawPoints":
                    jsFileString += "strokeWidth: 0, drawPoints: true, pointSize: 2, "
                else:
                    jsFileString += "strokePattern: {strokeType},".format(strokeType=currYData["lineType"])
            if currYData["lineWidth"] is not None:
                jsFileString += "strokeWidth: {strokeWidth},".format(strokeWidth=currYData["lineWidth"])
            # if currYData["yAxisLabel"] is not None and i==len(graphData["yData"])-1 and len(graphData["yData"])>1:
            # Assign
            if currYData["axisAssign"] == "y2":
                jsFileString += "axis:{}"
            jsFileString += "},\\\n"
            i += 1

        #   highlight areas
        hl = graphData["highlightedIntervall"]
        if (hl[0] is None) and (hl[1] is None):  # <  highlight nothing
            hl[0] = 0
            hl[1] = 0
        elif (hl[0] is None) and (hl[1] is not None):  # <  highlight everything up until hl[1]
            hl[0] = graphData["xData"][0]
        elif (hl[1] is None) and (hl[0] is not None):  # <  highlight everything from hl[0] to the end
            hl[1] = graphData["xData"][-1]
        # <  else: hl[0] and hl[1] were given and are used by default

        jsFileString += ("drawCallback:\\\nfunction(me, initial) {\\\n"
                         "if (blockRedraw || initial) return; \\\nblockRedraw = true; \\\n"
                         "var range = me.xAxisRange();\\\n"
                         "for (var j = 0; j < {syncGroupName}.length; j++) { \\\n"
                         "if ({syncGroupName}[j] == me) continue; \\\n"
                         "{syncGroupName}[j].updateOptions( { \\\n"
                         "dateWindow: range \\\n"
                         "} ); \\\n"
                         "} \\\n"
                         "blockRedraw = false; \\\n"
                         "}\\\n"
                         "\\\n, highlightCallback: function(e, x, pts, row) { \\\n"
                         "for (var j = 0; j < {syncGroupName}.length; j++) { \\\n"
                         "{syncGroupName}[j].setSelection(row); \\\n"
                         "}},underlayCallback: function (canvas, area, g)\\\n{\\\n"
                         'var bottom_left = g.toDomCoords({highlight_start}, -20);\\\n'
                         'var top_right = g.toDomCoords({highlight_end}, +20);\\\n'
                         'var left = bottom_left[0];\\\n'
                         'var right = top_right[0];\\\n'
                         "canvas.fillStyle = '{color}';\\\n"
                         'canvas.fillRect(left, area.y, right - left, area.h);\\\n'
                         'var range{graphID} = g.xAxisRange();\\\n'
                         'var xRange{graphID} = range{graphID}[1]-range{graphID}[0];\\\n'
                         'for (var i=0; i < {vertLineCount};i++) {\\\n'
                         "var VertLineXCoord{graphID}=vertLines{graphID}[i]['xCoord'];\\\n"
                         'var line_start{graphID} = VertLineXCoord{graphID}-0.001*xRange{graphID};\\\n'
                         'var line_end{graphID}=VertLineXCoord{graphID}+0.001*xRange{graphID};\\\n'
                         'var bottom_left{graphID} = g.toDomCoords(line_start{graphID}, 0);\\\n'
                         'var top_right{graphID} = g.toDomCoords(line_end{graphID}, 0);\\\n'
                         'var left{graphID}=bottom_left{graphID}[0];\\\n'
                         'var right{graphID} = top_right{graphID}[0]; \\\n'
                         "canvas.fillStyle=vertLines{graphID}[i]['color'];\\\n"
                         'canvas.fillRect(left{graphID}, area.y, right{graphID}-left{graphID}, area.h);\\\n'
                         '}}'
                         '}));\\\n</script>"\n').replace("{syncGroupName}", graphData["syncGroup"]) \
            .replace("{graphID}", graphID) \
            .replace("{highlight_start}", str(hl[0])) \
            .replace("{highlight_end}", str(hl[1])) \
            .replace("{color}", HIGHLIGHT_COLOR_RBGA)

        if vertLinesList is not None:
            jsFileString = jsFileString.replace("{vertLineCount}", str(len(vertLinesList)))
        else:
            jsFileString = jsFileString.replace("{vertLineCount}", "0")

        jsFileString += ');}\n'
        with open(os.path.join(os.path.dirname(filePath),
                               'graphData/graphData_{graphID}.js'.replace("{graphID}", graphID)), 'w') as file:
            file.write(jsFileString)
            file.write('function getData_{graphID}() { return GraphData{graphID};}'.replace("{graphID}", graphID))

        htmlFileString = ''
        if graphData["title"] is not None:
            if vertLinesList is not None:
                for i, obj in enumerate(vertLinesList):
                    htmlFileString += "<div style='width: 150px;left: {0}px; position:absolute; padding: 6px 6px; border: 2px solid #a1a1a1; border-radius: 8px;'>".format(
                        str(int(graphData["width"]) - 175 * (len(vertLinesList) - i)))
                    htmlFileString += '<span align="center" style="color: {color}">&nbsp;<b>{name}</b>: {time}s&nbsp;</span>'.format(
                        color=vertLinesList[i].color, time=str(vertLinesList[i].xCoord), name=vertLinesList[i].sigName)
                    htmlFileString += '</div>'
                htmlFileString += '<br>'
                htmlFileString += '<br>'
                htmlFileString += '<br>'
            htmlFileString += '<div style=" color: #FFFFFF; font-size: 24px"><u>{graphTitle}&nbsp;&nbsp;</u>{help}</div><br>' \
                .replace("{graphTitle}", graphData["title"]) \
                .replace("{help}", '<span class="circ" title="{0}">&nbsp;?&nbsp;</span>'.format(
                graphData["helpstring"].replace('\n', '&#13;')) if graphData["helpstring"] is not None else "")
        # if vertLinesList is not None:
        #    htmlFileString += "<div class='box' style='width: 150px; position:fixed'>"
        #    for i, obj in enumerate(vertLinesList):
        #        htmlFileString += '<p align="center" style="color: {color}">{name}: {time}s</p>'.format(color=vertLinesList[i].color, time=str(vertLinesList[i].xCoord), name=vertLinesList[i].sigName)
        #    htmlFileString += '</div>'

        htmlFileString += '<table><tbody><tr><td valign="top">'

        htmlFileString += '<div id="{graphID}"></div>'.replace('{graphID}', graphID)

        htmlFileString += '</td><td valign="top">&nbsp;&nbsp;</td><td valign="top"><div id="{0}_right"></div></td></tr></tbody></table><table  width = 100% style="table-layout:fixed;"><td style:"width: 33%"></td><td style:"width: 33%"><div id="{0}_below" style="left:600px;"></div></td><td style:"width: 33%"></td></table>'.format(
            legendID)

        htmlFileString += """ <script>
                            var got_{graphID}=false;
                            $(document).ready(function() {
                            if(lazy_reload==1){                                
                            $("#{graphID}").bind("inview", function(event, isVisible) 
                            { if(!isVisible) return;
                              if(got_{graphID}) return;
                            $.getScript('graphData/graphData_{graphID}.js', function() {
                            addGraphToHTML();
                            got_{graphID}=true;
                            });});}
                            else {$.getScript('graphData/graphData_{graphID}.js', function() {
                            addGraphToHTML();
                            });}});
                            </script>""".replace('{graphID}', graphID)

        self._graphCount += 1
        return htmlFileString

    def CreateImage(self, imgData):
        return '\n<IMG SRC="%s" ALT="%s" WIDTH=%d HEIGHT=%d/><br/>' % (
            imgData['path'], imgData['title'], imgData['width'], imgData['height'])

    def MoveFileBtwFolders(self, source, destination):
        try:
            src_files = os.listdir(source)
            for file_name in src_files:
                full_file_name = os.path.join(source, file_name)
                if os.path.isfile(full_file_name):
                    copy(full_file_name, destination)
        except:
            print("copy failed", sys.exc_info())

    def TimeStamp_ImageData(self, basepath, condition, folder_name):
        image_path = os.path.join("./Images", folder_name)
        images = os.listdir(basepath)
        carmaker_images = [s for s in images if condition in s]
        time = []
        image = []
        for i in carmaker_images:
            if i.endswith(".jpeg") or i.endswith(".jpg"):
                time.append(float(re.split('_', i)[-1].strip(".jpeg")))  # getting the time stamp from images
            g = os.path.join(image_path, str(i))
            g = re.sub(r"\\", "/", g)
            image.append(g)  # getting images
        return time, image

    def find_nearest(self, array, value):
        array = np.asarray(array)
        idx = (np.abs(array - value)).argmin()
        return array[idx]

    def birds_eyeview_timestamp(self, timestamps):
        start = end = 0
        for timestamp in timestamps:
            if timestamp.sigName.lower() == 'start':
                start = timestamp.xCoord
            elif timestamp.sigName.lower() == 'end':
                end = timestamp.xCoord
        return start, end

    def CreateDynamicImageGraph(self, graphData, filePath):
        self.xlabel = graphData["xLabel"]
        self.ylabel = graphData["yLabel"]
        graphID = "graph{0:04d}".format(self._graphCount)
        basepath = str(os.path.dirname(filePath))
        # birds eye view image folder
        basepath = os.path.join(basepath, 'Images', graphData["imageFolderName"])
        basepath = re.sub(r"\\", "/", basepath)

        if not os.path.exists(basepath):
            try:
                os.makedirs(basepath)
                src = os.path.join(graphData["cmPrjDir"], "SimOutput", "Images", graphData["imageFolderName"])
                self.MoveFileBtwFolders(src, basepath)
            except:
                print('Cannot create output file directory ' + os.path.dirname(filePath))
                sys.exit(0)

        basepath = re.sub(r"\\", "/", basepath)
        carmaker_image_name = graphData["imageFolderName"].split("_")[0] + "_" + \
                              graphData["imageFolderName"].split("_")[-2]
        carmaker_image_time, carmaker_image_data = self.TimeStamp_ImageData(basepath, carmaker_image_name,
                                                                            graphData["imageFolderName"])
        visualDebugger_image_time, visualDebugger_image_data = self.TimeStamp_ImageData(basepath, "BirdEye",
                                                                                        graphData["imageFolderName"])
        # start and end time to display birds eye images
        start_time, end_time = self.birds_eyeview_timestamp(graphData["vertLines"])
        # print ("carmaker_image_time", carmaker_image_time)
        # print ("graphData[x]", graphData["xData"])
        # print("carmaker_image_data", carmaker_image_data)
        # print("visualDebugger_image_time", visualDebugger_image_time)
        # print("visualDebugger_image_data", visualDebugger_image_data)

        jsFileString = ""

        graphFlag = True
        count = 0
        self._graphId.append(graphID)
        ### Write Graph Data to varaible
        currDataLineString = "[\n"
        if visualDebugger_image_time and visualDebugger_image_data and carmaker_image_data and carmaker_image_time:
            currDataLineString = self.CreateDataString(True, count, graphID, graphData, currDataLineString,
                                                       carmaker_image_time, carmaker_image_data,
                                                       visualDebugger_image_data, visualDebugger_image_time)

        elif carmaker_image_data and carmaker_image_time and (
                not visualDebugger_image_time or not visualDebugger_image_data):
            currDataLineString = self.CreateDataString("NoVisualImage", count, graphID, graphData, currDataLineString,
                                                       carmaker_image_time, carmaker_image_data,
                                                       visualDebugger_image_data, visualDebugger_image_time)

        elif visualDebugger_image_time and visualDebugger_image_data and (
                not carmaker_image_data or not carmaker_image_time):
            currDataLineString = self.CreateDataString("NoIPGImage", count, graphID, graphData, currDataLineString,
                                                       carmaker_image_time, carmaker_image_data,
                                                       visualDebugger_image_data, visualDebugger_image_time)

        else:
            currDataLineString = self.CreateDataString(False, count, graphID, graphData, currDataLineString,
                                                       carmaker_image_time, carmaker_image_data,
                                                       visualDebugger_image_data, visualDebugger_image_time)

        currDataLineString += '\n]'
        self._birdEyeData[graphID] = currDataLineString
        self.start_time[graphID], self.end_time[graphID] = start_time, end_time

        jsFileString += '<script src="scripts//canvasjs.min.js"></script>\n'
        jsFileString += '<div style=\' color: #FFFFFF; font-size: 24px\'><u>{title}</u><pre style="font-size: 12px;color:#00FF41">(Hover between 2 vertical lines to view Images)</pre></div>\n'.replace(
            '{title}', graphData['title'])
        jsFileString += '<table><tr>\n'
        jsFileString += (
            '<td><div id=\'chartContainer{graphID}\' width=\'1000\' height=\'300\' style=\'height:300px;width:500px;position:relative\'></div></td>\n').replace(
            '{graphID}', graphID)
        jsFileString += (
            '<td><div id=\'carmakerImage{graphID}\' width=\'400\' height=\'300\' style=\'width:400px;height:400px;position:relative\'></div></td>\n').replace(
            '{graphID}', graphID)
        jsFileString += (
            '<td><div id=\'birdEyeImage{graphID}\' width=\'400\' height=\'300\' style=\'width:400px;height:400px;position:relative\'></div></td>\n').replace(
            '{graphID}', graphID)
        jsFileString += '</tr></table>\n'

        jsFileString += '<br>\n'
        jsFileString += '<br>\n'

        self._graphCount += 1
        return graphFlag, jsFileString

    def CreateDataString(self, flag, count, graphID, graphData, currDataLineString, carmaker_image_time,
                         carmaker_image_data, visualDebugger_image_data, visualDebugger_image_time):
        for idx, currX in enumerate(graphData["xData"]):
            carmaker_image = ""
            visualDebugger_image = ""
            currDataLineString += '{x: ' + str("{0:.2f}".format(currX)) + ',\t'
            currDataLineString += 'y: ' + str(graphData['yData'][idx]) + ',\t'
            if flag == False:
                visualDebugger_image = os.path.join("./gfx", "visual_extract_issue.png")
                visualDebugger_image = re.sub(r"\\", "/", visualDebugger_image)
                carmaker_image = os.path.join("./gfx", "ipg_extract_issue.png")
                carmaker_image = re.sub(r"\\", "/", carmaker_image)
                if count == 0:
                    self._carmaker_initial_image[graphID] = str(carmaker_image)
                    self._birdeye_initial_image[graphID] = str(visualDebugger_image)
                    count += 1
            else:
                for index, currG in enumerate(
                        carmaker_image_time if flag != "NoIPGImage" else visualDebugger_image_time):
                    # print ("currX {}, currG {}".format(currX, currG))
                    if str(round(currX, 2)) == str(currG):
                        if flag == True:
                            carmaker_image = carmaker_image_data[index]
                            element = self.find_nearest(visualDebugger_image_time, currG)
                            image_index = [index for index, currB in enumerate(visualDebugger_image_time) if
                                           str(currB) == str(round(element, 2))]
                            visualDebugger_image = visualDebugger_image_data[image_index[0]]
                        elif flag == "NoVisualImage":
                            carmaker_image = carmaker_image_data[index]
                            visualDebugger_image = os.path.join("./gfx", "visual_extract_issue.png")
                            visualDebugger_image = re.sub(r"\\", "/", visualDebugger_image)
                        elif flag == "NoIPGImage":
                            visualDebugger_image = visualDebugger_image_data[index]
                            carmaker_image = os.path.join("./gfx", "ipg_extract_issue.png")
                            carmaker_image = re.sub(r"\\", "/", carmaker_image)
                        if count == 0:
                            self._carmaker_initial_image[graphID] = str(carmaker_image)
                            self._birdeye_initial_image[graphID] = str(visualDebugger_image)
                            count += 1
                        break
            currDataLineString += 'G: \"' + str(carmaker_image) + '\",\t'
            currDataLineString += 'B: \"' + str(visualDebugger_image) + '\",\t'
            currDataLineString += "},\n"
        return currDataLineString

    def CreateImageGraphScript(self):
        jsFileString = ""
        jsFileString += ' <script> window.addEventListener("load",function() {\n'
        for Id in self._graphId:
            jsFileString += (
                '{\n'
                'var image1 = "";\n'
                'var image2 = "";\n'
                'var data = [];\n'
                'var dataSeries = { type: \'line\', color: \'#000000\' };\n'
                'let dataPoints = {currDataLineString};\n'
                'var start_time = {start_time};\n'
                'var end_time = {end_time};\n'

                'dataSeries.dataPoints = dataPoints;\n'
                'dataSeries.mouseover = myEventHandler{graphID};\n'
                'dataSeries.click = myEventHandler{graphID};\n'
                'dataSeries.mousemove = myEventHandler{graphID};\n'
                'dataSeries.rangeChanging = myEventHandler{graphID};\n'
                'data.push(dataSeries);\n'

                # //Better to construct options first and then pass it as a parameter
                'var options{graphID} = {\n'
                'backgroundColor: \'#666666\',\n'
                'zoomEnabled: true,\n'
                'animationEnabled: true,\n'
                'name: \'Time\',\n'

                'toolTip: {\n'
                'fontColor: \'black\',\n'
                'content: "{x}",\n'
                'shared: true,\n'
                '},\n'

                'axisY: {\n'
                'includeZero: false,\n'
                'title: \'{ylabel}\',\n'
                'lineThickness: 1,\n'
                'lineColor: \'black\',\n'
                'gridDashType: \'line\',\n'
                'gridThickness: 1,\n'
                'interval: 0,\n'
                'gridColor: \'grey\'\n'
                '},\n'

                'axisX: {\n'
                'valueFormatString: \'#,##0.##\',\n'
                'lineThickness: 1,\n'
                'title: \'{xlabel}\',\n'
                'lineColor: \'black\',\n'
                'interval: 10,\n'
                'gridColor: \'grey\',\n'
                'gridDashType: \'line\',\n'
                'gridThickness: 1,\n'
                'stripLines:[\n'
                '{\n'
                'value:{start_time},\n'
                'color:"#000000"\n'
                '},\n'
                '{\n'
                'value:{end_time},\n'
                'color:"#000000"\n'
                '}\n'
                ']\n'
                '},\n'

                'data: data\n'  # random data
                '};\n'

                'var chart{graphID} = new CanvasJS.Chart(\'chartContainer{graphID}\', options{graphID}	);\n'
                'jQuery("#chartContainer{graphID}").on("inview", function(event, isInView) {\n'
                'if (isInView) {\n'
                'setTimeout(function(){\n'
                'chart{graphID}.render();\n'
                '}, 100);\n'

                #Function to get graph position, x-axis scale value and its respective object.
                '''jQuery("#chartContainer{graphID}").on("mousemove", 
	            function(e){
		        var parentOffset = $(this).parent().offset();
		        var relX = e.pageX - parentOffset.left;
		        var xValue = (chart{graphID}.axisX[0].convertPixelToValue(relX)).toFixed(2);
				let xVal_obj = dataPoints.find(o => o.x === parseFloat(xValue));
		        myEventHandler{graphID}(e, xVal_obj);
	            });\n'''

                'var eventData = \'<div><br><br><br><img src = \' + "{carmaker_initial_image}" + \' position=average caretPadding=6 height=300 width=400/>\' + \'</div>\';\n'
                'var eventData2 = \'<div><br><br><br><img src = \' + "{birdeye_initial_image}" + \' position=average caretPadding=6 height=300 width=400/>\' + \'</div>\';\n'
                '$(\'#carmakerImage{graphID}\').html(eventData);\n'
                '$(\'#birdEyeImage{graphID}\').html(eventData2);\n'
                '}\n'
                'else{\n'
                'var eventData = "";\n'
                '$(\'#carmakerImage{graphID}\').html(eventData);\n'
                '$(\'#birdEyeImage{graphID}\').html(eventData);\n}});\n'

                'function myEventHandler{graphID}(e, Val_obj=null) {\n'
                'if (Val_obj != null) {\n'
                'if (Val_obj.G != "" && Val_obj.x >= {start_time} && Val_obj.x <= {end_time}) {\n'
                'image1 = Val_obj.G;\n'
                'image2 = Val_obj.B;\n'
                'var eventData = \'<div><br><br><br><img src = \' + image1 + \' position=average caretPadding=6 height=300 width=400/>\' + \'</div>\';\nchart{graphID}.render();\n'
                '$(\'#carmakerImage{graphID}\').html(eventData);\n'
                'var eventData2 = \'<div><br><br><br><img src = \' + image2 + \' position=average caretPadding=6 height=300 width=400/>\' + \'</div>\';\nchart{graphID}.render();\n'
                '$(\'#birdEyeImage{graphID}\').html(eventData2);\n'
                '}'
                'else if (Val_obj.x >= {start_time} && Val_obj.x <= {end_time}) {\n'
                'var eventData = \'<div><br><br><br><img src = \' + image1 + \' position=average caretPadding=6 height=300 width=400/>\' + \'</div>\';\nchart{graphID}.render();\n'
                '$(\'#carmakerImage{graphID}\').html(eventData);\n'
                'var eventData2 = \'<div><br><br><br><img src = \' + image2 + \' position=average caretPadding=6 height=300 width=400/>\' + \'</div>\';\nchart{graphID}.render();\n'
                '$(\'#birdEyeImage{graphID}\').html(eventData2);\n'
                '}'
                'else {'
                'var eventData = \'<div><br><br><br><img src = ""\' + \'</div>\';\nchart{graphID}.render();\n'
                '$(\'#carmakerImage{graphID}\').html(eventData);\n'
                '$(\'#birdEyeImage{graphID}\').html(eventData);\n'
                '}'
                '}\n'
                'else {\n'
                'if (e.dataPoint.G != "" && e.dataPoint.x >= {start_time} && e.dataPoint.x <= {end_time}) {\n'
                'image1 = e.dataPoint.G;\n'
                'image2 = e.dataPoint.B;\n'
                'var eventData = \'<div><br><br><br><img src = \' + image1 + \' position=average caretPadding=6 height=300 width=400/>\' + \'</div>\';\nchart{graphID}.render();\n'
                '$(\'#carmakerImage{graphID}\').html(eventData);\n'
                'var eventData2 = \'<div><br><br><br><img src = \' + image2 + \' position=average caretPadding=6 height=300 width=400/>\' + \'</div>\';\nchart{graphID}.render();\n'
                '$(\'#birdEyeImage{graphID}\').html(eventData2);\n'
                '}'
                'else if (e.dataPoint.x >= {start_time} && e.dataPoint.x <= {end_time}) {\n'
                'var eventData = \'<div><br><br><br><img src = \' + image1 + \' position=average caretPadding=6 height=300 width=400/>\' + \'</div>\';\nchart{graphID}.render();\n'
                '$(\'#carmakerImage{graphID}\').html(eventData);\n'
                'var eventData2 = \'<div><br><br><br><img src = \' + image2 + \' position=average caretPadding=6 height=300 width=400/>\' + \'</div>\';\nchart{graphID}.render();\n'
                '$(\'#birdEyeImage{graphID}\').html(eventData2);\n'
                '}'
                'else {'
                'var eventData = \'<div><br><br><br><img src = ""\' + \'</div>\';\nchart{graphID}.render();\n'
                '$(\'#carmakerImage{graphID}\').html(eventData);\n'
                '$(\'#birdEyeImage{graphID}\').html(eventData);\n'
                '}'
                '}}\n'
                '};	\n').replace('{graphID}', Id) \
                .replace('{currDataLineString}', self._birdEyeData[Id]) \
                .replace('{xlabel}', self.xlabel) \
                .replace('{ylabel}', self.ylabel) \
                .replace('{carmaker_initial_image}', self._carmaker_initial_image[Id]) \
                .replace('{birdeye_initial_image}', self._birdeye_initial_image[Id]) \
                .replace('{start_time}', str(self.start_time[Id])).replace('{end_time}', str(self.end_time[Id]))

        jsFileString += '})\n'

        jsFileString += '</script>\n'

        return jsFileString

    def CreateVideo(self, videoData):
        # htmlFileString = """<video controls>
        #                    <source src=""" +videoData["fileName"] +""" type=video/mp4>
        #                    </video>"""
        htmlFileString = """<video id=\"""" + videoData["title"] + """\" class="video-js vjs-default-skin" controls preload="none" width="640" height="300"
                            poster=\"""" + videoData["imgFileName"] + """\" data-setup="{}">
                            <source src=\"""" + videoData["fileName"] + """\" type='video/mp4' />
                            <p class="vjs-no-js">To view this video please enable JavaScript, and consider upgrading to a web browser that <a href="http://videojs.com/html5-video-support/" target="_blank">supports HTML5 video</a></p>
                            </video>

                            <script type='text/javascript'>
                                var video = videojs(\"""" + videoData["title"] + """\", {
                                controls: true,
                                autoplay: false,
                                preload: 'none',
                                plugins: {
                                framebyframe: {
                                fps: 30,
                                steps: [
                                        { text: '<- -', step: -5 },
                                        { text: '<-', step: -1 },
                                        { text: '->', step: 1 },
                                        { text: '- ->', step: 5 },
                                      ]
                                    }
                                  }
                                });
                            </script>"""
        return htmlFileString

    def CreateLink(self, linkData):
        return '%s<a href="%s" %s>%s</a>' % (linkData['prefix'].replace(" ", "&nbsp"),
                                             linkData['path'],
                                             'title="' + linkData['title'] + '"' if linkData[
                                                                                        'title'] is not None else '',
                                             linkData['text'])

    def AddHTMLCode(self, codeData):
        return codeData['code']

    def generateScale(self, Scale):
        TabelId = 'ScaleID' + str(self.Scale_counter)
        check = False
        if Scale.value >= Scale.start and Scale.value <= Scale.end:
            check = True
        if check == False:
            print("value is outside tabel range")
            return -1
        # mode:
        # if Scale.mode=='descending':
        #	temp='Color.push(str.concat("rgb(" + (Math.floor(254/tds{num}.length))*k + "," + (255-(Math.floor(254/tds{num}.length))*k) + "," + 0 +")"));\n'.format(num=self.Scale_counter)
        # elif Scale.mode == 'ascending':
        #	temp='Color.push(str.concat("rgb(" + (255-(Math.floor(254/tds{num}.length))*k) + "," + (Math.floor(254/tds{num}.length))*k + "," + 0 +")"));\n'.format(num=self.Scale_counter)
        if Scale.mode == 'descending':
            temp = 'if (i >= tds0.length/2) {{\
                  Color.push(str.concat("rgb(" + 255 + "," + Math.floor(510-(255/tds{num}.length)*2*i) + "," + 0 +")"));}}\
                  else{{Color.push(str.concat("rgb(" + Math.floor((255/tds{num}.length)*i*2)+ "," + 255 + "," + 0 +")"));}}'.format(
                num=self.Scale_counter)
        elif Scale.mode == 'ascending':
            temp = 'if (i >= tds0.length/2) {{\
                  Color.push(str.concat("rgb(" + Math.floor(510-(255/tds{num}.length)*i*2)+ "," + 255 + "," + 0 +")"));}}\
                  else{{Color.push(str.concat("rgb(" + 255 + "," + (Math.floor(255/tds{num}.length))*2*i + "," + 0 +")"));}}'.format(
                num=self.Scale_counter)

        strOut = """<img id="{Id2}" src="gfx\Arrow1DownOrange.png" style="width:20px;height:25px;position: relative;left:100px;top:10px;" ></img>\n
            <div class="block" >\n
            <table id="{Id}" style="width:100%; height:30px; table-layout: fixed; border: 1px solid black; border-collapse: collapse;">\n
            <tr>\n""".format(Id=TabelId, Id2=TabelId + 'IMG')

        for i in range((Scale.end - Scale.start + 1) / Scale.step):
            strOut = strOut + '<td style="height:7px; border: 1px solid black; border-collapse: collapse;"></td>\n'
        strOut = strOut + '</tr>\n'

        for i in range((Scale.end - Scale.start + 1) / Scale.step):
            strOut = strOut + '<td style="background-color:#333333;border: 1px solid black; border-collapse: collapse;">{}</td>\n'.format(
                Scale.start + i * Scale.step)

        # add these lines under strOut for Mouse Position function
        # '<p id="mousePos"></p>\n'\
        # '<script src="scripts\Mousepositon.js"></script>\n'\
        # and change '<div class="block" >\n'\ to '<div class="block" onmousemove="getMouse(event)>\n'\"

        strOut = strOut + """</tr>\n</table>\n</div>\n
                      <script>\n
                      var tds{num}=document.getElementById("{Id}").getElementsByTagName("tr")[0].getElementsByTagName("td");\n
                      var Color=[];\n
                      var str;\n
                      for (i=0; i <tds{num}.length; i++){{\n
                      str="";\n
                      var k=i+1;\n
                      {mode}
                      tds{num}[i].style.backgroundColor =Color[i];}}\n
                      tds{num}=document.getElementById("{Id}").getElementsByTagName("tr")[1].getElementsByTagName("td");\n
                      var wid{num}=parseFloat(document.getElementById("{Id}").offsetWidth);\n
                      var value{num}= (parseFloat(wid{num}*({value}-parseFloat(document.getElementById("{Id}").getElementsByTagName("tr")[1].getElementsByTagName("td")[0].innerHTML))/(({step}+parseFloat(document.getElementById("{Id}").getElementsByTagName("tr")[1].getElementsByTagName("td")[tds{num}.length-1].innerHTML)- parseFloat(document.getElementById("{Id}").getElementsByTagName("tr")[1].getElementsByTagName("td")[0].innerHTML))/{step}))-10)/{step};\n
                      document.getElementById("{Id2}").style.left=value{num}+"px"\n
                      function setImg{num}(){{\n
                      wid{num}=parseFloat(document.getElementById("{Id}").offsetWidth);\n
                      value{num}= (parseFloat(wid{num}*({value}-parseFloat(document.getElementById("{Id}").getElementsByTagName("tr")[1].getElementsByTagName("td")[0].innerHTML))/(({step}+parseFloat(document.getElementById("{Id}").getElementsByTagName("tr")[1].getElementsByTagName("td")[tds{num}.length-1].innerHTML)- parseFloat(document.getElementById("{Id}").getElementsByTagName("tr")[1].getElementsByTagName("td")[0].innerHTML))/{step}))-10)/{step};\n
                      document.getElementById("{Id2}").style.left=value{num}+"px"}};window.addEventListener("resize",setImg{num});</script>""".format(
            value=Scale.value, step=Scale.step, Id=TabelId, Id2=TabelId + 'IMG', mode=temp, num=self.Scale_counter)
        self.Scale_counter += 1
        return strOut