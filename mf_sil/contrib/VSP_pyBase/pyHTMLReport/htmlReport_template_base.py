class cHTMLReportTemplateBase(object):
    def GetAdditionalFilePaths(self):
        pass
    def CreateHeader(self, title):
        pass
    def CreateFooter(self):
        pass
    def CreateSectionBegin(self, sectionLabel):
        pass
    def CreateSectionEnd(self):
        pass
    def CreateTextSection(self, text):
        pass
    def CreateTableBegin(self, caption=""):
        pass
    def CreateTableEnd(self):
        pass
    def CreateTableHeaderBegin(self):
        pass
    def CreateTableHeaderCell(self, text=None, textColor=None, textAlign=None, textStyle=None, backgroundColor=None):
        pass
    def CreateTableHeaderEnd(self):
        pass
    def CreateTableRowBegin(self):
        pass
    def CreateTableRowCell(self, text=None, textColor=None, textAlign=None, textStyle=None, backgroundColor=None):
        pass
    def CreateTableRowEnd(self):
        pass
    def CreateGraph(self, graphData):
        pass
    