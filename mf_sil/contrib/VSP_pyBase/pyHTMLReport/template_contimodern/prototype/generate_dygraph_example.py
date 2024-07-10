from random import randint

numberOfGraphs     = 1000
numberOfPlotPoints = 4000

section01 = '<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.01 Transitional//EN"><html><head><script type="text/javascript" src="dygraph-combined.js"></script><title></title></head><body>'

with open("dygraphExample.html","w") as f:
	f.write(section01)
	for currGraphID in range(numberOfGraphs):
		f.write('<div id="graphdiv{0}"></div>'.format(currGraphID))
		f.write('<script type="text/javascript">g = new Dygraph(document.getElementById("graphdiv{0}"),"time,Value1,Value2,Value3\\n" +'.format(currGraphID))
		for currPlotPointID in range(numberOfPlotPoints):
			f.write('"{0},{1},{2},{3}\\n"+'.format(currPlotPointID,randint(-100,100),randint(-100,100),randint(-100,100)))
		f.write('"{0},{1},{2},{3}\\n"'.format(numberOfPlotPoints,randint(-100,100),randint(-100,100),randint(-100,100)))
		f.write(');</script>')
	f.write('</body></html>')