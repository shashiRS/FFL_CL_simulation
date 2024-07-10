#====================================================================
# System Imports
#====================================================================
import os
import sys

from htmlReport import cHTMLReport, cHTMLReportTable, cHTMLReportTableRow, cHTMLReportTableCell, cHTMLReportGraph, cHTMLReportGraphYData, cHTMLReportImage, cHTMLReportLink, cHTMLReportVideo,cHTMLReportScale

report = cHTMLReport(title="The Title", theme="contimodern", toc=True)

report.SectionBegin("This is the First Section")
report.AddText("How nice of you!")
simpleTable = cHTMLReportTable("the best table in the world probably", sortable=False)
simpleTable.setHeaderRow(cHTMLReportTableRow(cells=["Id", "Name", "Description", "Result"]))
simpleTable.addHeaderDescription(cHTMLReportTableRow(cells=[cHTMLReportTableCell(text="HR",colSpan="2"), cHTMLReportTableCell(text="Test",colSpan="2")]))
for i in range(0,200,4):
    simpleTable.addRow(cHTMLReportTableRow(cells=[cHTMLReportTableCell("i".replace("i", str(i))), cHTMLReportTableCell("Freddy"), cHTMLReportTableCell(text="He once won a cow milking contest",textColor=(124,223,20),textAlign="center"), cHTMLReportTableCell("24")]))
    #simpleTable.addRow(cHTMLReportTableRow(cells=["i".replace("i", str(i+1)), "Brady",  "Spends his money on used dumpster trucks", "15"]))
    #simpleTable.addRow(cHTMLReportTableRow(cells=["i".replace("i", str(i+2)), "Quinn",  "Wants to have 5 children with his ugly wife", "6"]))
    #simpleTable.addRow(cHTMLReportTableRow(cells=["i".replace("i", str(i+3)), "Bob",    "Dont talk to him without having a beer first", "221"]))
report.AddTable(simpleTable)
simpleTable = cHTMLReportTable("the best table in the world probably")
simpleTable.setHeaderRow(cHTMLReportTableRow(cells=["Id", "Name", "Description", "Result"]))
#simpleTable.addHeaderDescription(cHTMLReportTableRow(cells=["HR", "Test"], colSpan=["1", "3"]))
#simpleTable.addHeaderDescription(cHTMLReportTableRow(cells=["HR", "Test"], colSpan=["2", "2"]))
simpleTable.addRow(cHTMLReportTableRow(cells=["0", "Freddy", cHTMLReportTableCell(text="He once won a cow milking contest",backgroundColor=(199,200,15),textAlign="center",textColor=(15,74,239) ), "24"]))
simpleTable.addRow(cHTMLReportTableRow(cells=["1", "Brady",  "Spends his money on used dumpster trucks", "15"]))
simpleTable.addRow(cHTMLReportTableRow(cells=["2", "Quinn",  "Wants to have 5 children with his ugly wife", "6"]))
simpleTable.addRow(cHTMLReportTableRow(cells=["3", "Bob",    "Dont talk to him without having a beer first", "221"]))
report.AddTable(simpleTable)
report.SectionEnd()
report.SectionBegin("Second section")
report.AddText("Some text for the second section")
report.SectionBegin("With a nested Section!")
report.AddText("...and some more for the nested section")
report.SectionEnd()
report.SectionBegin("And another nested section")
report.AddText("HOW crazy is that?")
report.SectionEnd()
report.SectionBegin("And now: A Graph Section!")

yData1 = cHTMLReportGraphYData([3,4,2,3,4,5,6,2], "SomeData 1")
yData2 = cHTMLReportGraphYData([2,1,1,6,5,5,6,3], "SomeData 2")
simpleGraph = cHTMLReportGraph([0,1,2,3,4,5,6,8], [yData1,yData2], xLim=[0, 10], xLabel='x1', yLabel='y1',lineWidth=3,title='First graph title', height=300,rangeselector = True, highlightedIntervall=[2,4] )
report.AddGraph(simpleGraph)

yData1 = cHTMLReportGraphYData([3,4,2,3,4,5,6], "SomeData 1")
yData2 = cHTMLReportGraphYData([2,1,1,6,5,5,6], "SomeData 2")
simpleGraph = cHTMLReportGraph([0,1,2,3,4,5,6], [yData1,yData2], xLim=[0, 6], xLabel='x2', yLabel='y2',title='Second graph title', width=550,  height=300 )
report.AddGraph(simpleGraph)

yData1 = cHTMLReportGraphYData([3,4,2,3,4,5,6], "SomeData 1")
yData2 = cHTMLReportGraphYData([2,1,1,6,5,5,6], "SomeData 2")
simpleGraph = cHTMLReportGraph([0,1,2,3,4,5,6], [yData1,yData2], xLim=[0, 2], xLabel='x3', yLabel='y3')
report.AddGraph(simpleGraph)

yData1 = cHTMLReportGraphYData([3,4,2,3,4,5,6], "SomeData 1")
yData2 = cHTMLReportGraphYData([2,1,1,6,5,5,6], "SomeData 2")
simpleGraph = cHTMLReportGraph([0,1,2,3,4,5,6], [yData1,yData2], xLim=[0, 2], xLabel='x4', yLabel='y4')
report.AddGraph(simpleGraph)

rp = os.path.relpath(os.path.join(  os.path.dirname(__file__), "template_contimodern", "gfx"), os.path.join(  os.path.dirname(__file__), "test_output") )
report.AddImage(cHTMLReportImage(os.path.join(  rp, "conti_seal.png" ), 'my_image', 450, 250))

report.AddLink(cHTMLReportLink("http://www.n-tv.de", 'news'))
report.SectionEnd()

report.SectionBegin("Finally: A fancy video section")

tmpVideo = cHTMLReportVideo("Test_RRec",os.path.join(  "video", "Test.rrec.mp4" ),os.path.join(  "video", "Test.rrec.bmp" ))
report.AddVideo(tmpVideo)
report.AddColorScale(cHTMLReportScale(0.55,0,50,1,'ascending'))
report.SectionEnd()
report.Save( os.path.join(os.path.dirname(__file__), "test_output", "test_report.html" ))


print(('html created at: ' + os.path.join(os.path.dirname(__file__), "test_output", "test_report.html" )))
