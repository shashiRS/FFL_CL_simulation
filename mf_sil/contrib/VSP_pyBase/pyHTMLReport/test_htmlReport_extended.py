#====================================================================
# System Imports
#====================================================================
import os
import sys

from htmlReport import cHTMLReport, cHTMLReportTable, cHTMLReportTableRow, cHTMLReportTableCell, cHTMLReportGraph, cHTMLReportGraphYData, cHTMLReportImage, cHTMLReportLink, cHTMLReportVideo, cHTMLReportVerticalLine, cHTMLReportCode

report = cHTMLReport(title="The Title", theme="contimodern", toc=True)


# ###################################################
# ##################### TABLES ######################
# ###################################################
report.SectionBegin("Table section")

# Simple Table
# -------------------------------
report.SectionBegin("Simple Table")
report.AddText("First: A simple table")
simpleTable = cHTMLReportTable("First Table", sortable=False)
simpleTable.setHeaderRow(cHTMLReportTableRow(cells=["Id", "Name", "Description", "Result"]))
for i in range(0,2):
    simpleTable.addRow(cHTMLReportTableRow(cells=[cHTMLReportTableCell("i".replace("i", str(i))), cHTMLReportTableCell("Freddy"), cHTMLReportTableCell("He once won a cow milking contest"), cHTMLReportTableCell("24")]))
    simpleTable.addRow(cHTMLReportTableRow(cells=["i".replace("i", str(i+1)), "Brady",  "Spends his money on used dumpster trucks", "15"]))
    simpleTable.addRow(cHTMLReportTableRow(cells=["i".replace("i", str(i+2)), "Quinn",  "Wants to have 5 children with his ugly wife", "6"]))
    simpleTable.addRow(cHTMLReportTableRow(cells=["i".replace("i", str(i+3)), "Bob",    "Dont talk to him without having a beer first", "221"]))
report.AddTable(simpleTable)
report.SectionEnd()
# -------------------------------


# Extended Table
# -------------------------------
report.SectionBegin('Extended Table')
report.AddText('Second: Table with extra header row and sort function and a width of 800px')
simpleTable = cHTMLReportTable("Second Table", sortable=True, width = 800)
simpleTable.setHeaderRow(cHTMLReportTableRow(cells=["Id", "Name", "Description", "Result"]))
simpleTable.addHeaderDescription(cHTMLReportTableRow(cells=[cHTMLReportTableCell(text="HR",colSpan="2"), cHTMLReportTableCell(text="Test",colSpan="2")]))
for i in range(0,2):
    simpleTable.addRow(cHTMLReportTableRow(cells=[cHTMLReportTableCell("i".replace("i", str(i))), cHTMLReportTableCell("Freddy"), cHTMLReportTableCell("He once won a cow milking contest"), cHTMLReportTableCell("24")]))
    simpleTable.addRow(cHTMLReportTableRow(cells=["i".replace("i", str(i+1)), "Brady",  "Spends his money on used dumpster trucks", "15"]))
    simpleTable.addRow(cHTMLReportTableRow(cells=["i".replace("i", str(i+2)), "Quinn",  "Wants to have 5 children with his ugly wife", "6"]))
    simpleTable.addRow(cHTMLReportTableRow(cells=["i".replace("i", str(i+3)), "Bob",    "Dont talk to him without having a beer first", "221"]))
report.AddTable(simpleTable)
report.SectionEnd()
# -------------------------------

report.SectionEnd()

# ###################################################
# ##################### GRAPHS ######################
# ###################################################
report.SectionBegin("Graph Section")

# Simple Graph
# -------------------------------
report.SectionBegin('Simple Graph')
report.AddText('Simple Plot with two graphs')
yData1 = cHTMLReportGraphYData([3,4,2,3,4,5,6], "SomeData 1")
yData2 = cHTMLReportGraphYData([2,1,1,6,5,5,6], "SomeData 2")
simpleGraph = cHTMLReportGraph([0,1,2,3,4,5,6], [yData1,yData2], xLabel='x', yLabel='y')
report.AddGraph(simpleGraph)
report.SectionEnd()
# -------------------------------

# Graph with range selector
# -------------------------------
report.SectionBegin('Range Selector')
report.AddText('Plot with range selector and legend location can be chosen in code')
yData1 = cHTMLReportGraphYData([3,4,2,3,4,5,6], "SomeData 1")
yData2 = cHTMLReportGraphYData([2,1,1,6,5,5,6], "SomeData 2")
simpleGraph = cHTMLReportGraph([0,1,2,3,4,5,6], [yData1,yData2], xLabel='x', yLabel='y', rangeselector=True, legendLocation='right') # legendLocation: inside, below, right (default)
report.AddGraph(simpleGraph)
report.SectionEnd()
# -------------------------------

# Plots in one sync-group
# -------------------------------
report.SectionBegin('Synchronized Graphs')
report.AddText('Zoom actions are synchronized between plots in the same group.')
report.AddText('Graph 1 and 3 are in the same sync-group, graph 2 is not.')
yData1 = cHTMLReportGraphYData([3,4,2,3,4,5,6], "SomeData 1")
yData2 = cHTMLReportGraphYData([2,1,1,6,5,5,6], "SomeData 2")
simpleGraph = cHTMLReportGraph([0,1,2,3,4,5,6], [yData1,yData2], xLabel='x', yLabel='y', syncGroup='group1')
report.AddGraph(simpleGraph)
yData1 = cHTMLReportGraphYData([3,4,2,3,4,5,6], "SomeData 1")
yData2 = cHTMLReportGraphYData([2,1,1,6,5,5,6], "SomeData 2")
simpleGraph = cHTMLReportGraph([0,1,2,3,4,5,6], [yData1,yData2], xLabel='x', yLabel='y', syncGroup='group2')
report.AddGraph(simpleGraph)
Data1 = cHTMLReportGraphYData([3,4,2,3,4,5,6], "SomeData 1")
yData2 = cHTMLReportGraphYData([2,1,1,6,5,5,6], "SomeData 2")
simpleGraph = cHTMLReportGraph([0,1,2,3,4,5,6], [yData1,yData2], xLabel='x', yLabel='y', syncGroup='group1')
report.AddGraph(simpleGraph)
report.SectionEnd()
# -------------------------------

# Graph with highlighted intervall
# -------------------------------
report.SectionBegin('Highlighted intervall')

report.AddText('Intervall [0.5,1.5] is highlighted and x-range is initially [0,2]')
yData1 = cHTMLReportGraphYData([3,4,2,3,4,5,6], "SomeData 1")
yData2 = cHTMLReportGraphYData([2,1,1,6,5,5,6], "SomeData 2")
simpleGraph = cHTMLReportGraph([0,1,2,3,4,5,6], [yData1,yData2], xLim=[0, 2], xLabel='x', yLabel='y', highlightedIntervall=[0.5,1.5])
report.AddGraph(simpleGraph)

report.AddText('Highlight intervall from beginning to 1 and x-range is initially [0,2]')
yData1 = cHTMLReportGraphYData([3,4,2,3,4,5,6], "SomeData 1")
yData2 = cHTMLReportGraphYData([2,1,1,6,5,5,6], "SomeData 2")
simpleGraph = cHTMLReportGraph([0,1,2,3,4,5,6], [yData1,yData2], xLim=[0, 2], xLabel='x', yLabel='y', highlightedIntervall=[None,1])
report.AddGraph(simpleGraph)

report.AddText('Highlight intervall from 1 to end and x-range is initially [0,2]')
yData1 = cHTMLReportGraphYData([3,4,2,3,4,5,6], "SomeData 1")
yData2 = cHTMLReportGraphYData([2,1,1,6,5,5,6], "SomeData 2")
simpleGraph = cHTMLReportGraph([0,1,2,3,4,5,6], [yData1,yData2], xLim=[0, 2], xLabel='x', yLabel='y', highlightedIntervall=[1,None])
report.AddGraph(simpleGraph)

report.SectionEnd()
# -------------------------------

# Graph with vertical line(s)
# -------------------------------
report.SectionBegin('Vertical lines')

report.AddText('Graph with two vertical lines that may indicate an event')
yData1 = cHTMLReportGraphYData([3,4,2,3,4,5,6], "SomeData 1")
yData2 = cHTMLReportGraphYData([2,1,1,6,5,5,6], "SomeData 2")
simpleGraph = cHTMLReportGraph([0,1,2,3,4,5,6], [yData1,yData2], xLabel='x', yLabel='y', vertLines=[cHTMLReportVerticalLine(sigName='Warning', xCoord=1.2, color='yellow'), 
                                                                                                    cHTMLReportVerticalLine(sigName='Braking', xCoord=1.6, color='red')])
report.AddGraph(simpleGraph)

report.SectionEnd()
# -------------------------------

# Graphs on two different axes
# -------------------------------
report.SectionBegin('Two Y Axes')

report.AddText('The graphs are aligned to different y axes.')
yData1 = cHTMLReportGraphYData([3,4,2,3,4,5,6], "Pressure", axisAssign='y1')      # y1 and y2 are standard labels for left and right y axis in dygraph and cannot be renamed
yData2 = cHTMLReportGraphYData([20,10,10,60,50,50,60], "Probability", axisAssign='y2')
simpleGraph = cHTMLReportGraph([0,1,2,3,4,5,6], [yData1,yData2], xLabel='Time', yLabel='Pressure')
report.AddGraph(simpleGraph)

report.SectionEnd()
# -------------------------------

# Labels for y axes
# -------------------------------
report.SectionBegin('Y Labels on axis')

report.AddText('Y axis has labels instead of numbers.')
yData1 = cHTMLReportGraphYData([3,4,2,3,4,5,6], "Pressure", axisAssign='y1', yAxisLabel={'0':'L0', '1':'L1', '2':'L2', '3':'L3', '4':'L4', '5':'L5', '6':'L6'})      # y1 and y2 are standard indicators for left and right y axis in dygraph and cannot be renamed
yData2 = cHTMLReportGraphYData([20,10,10,60,50,50,60], "Probability", axisAssign='y2')
simpleGraph = cHTMLReportGraph([0,1,2,3,4,5,6], [yData1,yData2], xLabel='Time', yLabel='Pressure')
report.AddGraph(simpleGraph)

report.SectionEnd()
# -------------------------------

# Data reduction
# -------------------------------
report.SectionBegin('Data reduction')

report.AddText('The amount of data that is used by dygraph to generate plots is reduced by smart choices for relevant information.')
report.AddText('Example: If the y value of a graph stays the same for several times, only the first and last point of this value is regarded and the points are connected.')
report.AddText('Note: Do not combine this option with the option "NoInterpBetwDataPoints". If you do so, it causes the creation of much unnecessary data.')
yData1 = cHTMLReportGraphYData([3,4,2,3,4,5,6], "Pressure")      # y1 and y2 are standard indicators for left and right y axis in dygraph and cannot be renamed
simpleGraph = cHTMLReportGraph([0,1,2,3,4,5,6], [yData1], xLabel='Time', yLabel='Pressure', dataReduction=True)
report.AddGraph(simpleGraph)

report.SectionEnd()
# -------------------------------

# Avoid interpolation between data points
# -------------------------------
report.SectionBegin('Avoid interpolation between data points')

report.AddText('Usually the graph between two data points is interpolated. You can switch this function on/off for each graph in the plot.')
report.AddText('You can see two graphs with the same function values. One with and one without interpolation.')
report.AddText('Note: Do not combine this option with the option "datareduction"! If you do so, it causes the creation of much unnecessary data.')
yData1 = cHTMLReportGraphYData([3,4,2,3,4,5,2], "PressureInterp", NoInterpBetwDataPoints=False)
yData2 = cHTMLReportGraphYData([3,4,2,3,4,5,2], "PressureNoInterp", NoInterpBetwDataPoints=True)
simpleGraph = cHTMLReportGraph([0,1,2,3,4,5,6], [yData1, yData2], xLabel='Time', yLabel='Pressure', dataReduction=False)
report.AddGraph(simpleGraph)

report.SectionEnd()
# -------------------------------

## Graphs on two different axes
## -------------------------------
#report.SectionBegin('Legend labels')
#
#report.AddText('Legend label.')
#yData1 = cHTMLReportGraphYData([3,4,2,3,4,5,6], "Pressure", axisAssign='y1', legendLabel='test1')      # y1 and y2 are standard labels for left and right y axis in dygraph and cannot be renamed
#yData2 = cHTMLReportGraphYData([20,10,10,60,50,50,60], "Probability", axisAssign='y2')
#simpleGraph = cHTMLReportGraph([0,1,2,3,4,5,6], [yData1,yData2], xLabel='Time', yLabel='Pressure')
#report.AddGraph(simpleGraph)
#
#report.SectionEnd()
## -------------------------------

report.SectionEnd()


# ###################################################
# ##################### LINKS #######################
# ###################################################
report.SectionBegin('Links')
report.AddLink(cHTMLReportLink('http://www.google.de', 'Link to google'))   # you need to write "http://". Otherwise it will be assumed as a relative link to the html document
report.SectionEnd()

# ###################################################
# ##################### LINKS #######################
# ###################################################
report.SectionBegin('Images')
rp = os.path.relpath(os.path.join(  os.path.dirname(__file__), "template_contimodern", "gfx"), os.path.join(  os.path.dirname(__file__), "test_output") )
report.AddImage(cHTMLReportImage(os.path.join(  rp, "conti_seal.png" ), 'my_image', 200, 200))
report.SectionEnd()

# ###################################################
# ##################### VIDEOS ######################
# ###################################################

report.SectionBegin("Video Section")

tmpVideo = cHTMLReportVideo("Test_RRec",os.path.join(  "video", "Test.rrec.mp4" ),os.path.join(  "video", "Test.rrec.bmp" ))
report.AddVideo(tmpVideo)

report.SectionEnd()

report.SectionBegin("Include own HTML Code")
report.AddText("Given html code is written in the document.")
report.AddHTMLCode(cHTMLReportCode(code='<a href="http://www.google.de">Testlink</a>'))
report.SectionEnd()

# ###################################################
# ##################### SAFE REPORT #################
# ###################################################
report.Save( os.path.join(os.path.dirname(__file__), "test_output", "test_report.html" ))



print(('html created at: ' + os.path.join(os.path.dirname(__file__), "test_output", "test_report.html" )))
