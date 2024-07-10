import tkinter, tkinter.filedialog

import wx
import unicodedata

import os
import sys

import threading
import inspect
import configparser

class EventHook(object):
    def __init__(self):
        self.__handlers = []

    def __iadd__(self, handler):
        self.__handlers.append(handler)
        return self

    def __isub__(self, handler):
        self.__handlers.remove(handler)
        return self

    def fire(self, *args, **keywargs):
        for handler in self.__handlers:
            handler(*args, **keywargs)

    def clearObjectHandlers(self, inObject):
        for theHandler in self.__handlers:
            if theHandler.__self__ == inObject:
                self -= theHandler


class StoppableThread(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self):
        super(StoppableThread, self).__init__()
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()


class OpenDialog():
    def __init__(self):
        pass
    
    def GetIniFileName(self, caller):
        fileExtLen=len(os.path.splitext(caller)[1])
        return os.path.join( os.path.dirname(caller), os.path.basename(caller)[:-fileExtLen] + '.ini')
    
    def run(self, *args, **kwargs):
        save_defaults = kwargs.get('save_defaults', True)
        initialdir = kwargs.get('initialdir', '')
        initialfile = kwargs.get('initialfile', '')
        title = kwargs.get('title', '')
        
        root = tkinter.Tk()
        root.withdraw()
        
        # befor option handling
        if save_defaults == True:
            try:
                caller = ''
                call_stack = inspect.stack()
                for elem_tuple in call_stack:
                    for elem in elem_tuple:
                        if inspect.isframe(elem):
                            elem = inspect.getabsfile(elem)
                        if isinstance(elem,str):
                            if not (os.path.splitext(os.path.basename(__file__))[0] in elem):
                                if '_cmpy' in elem:
                                    caller = elem
                
                if caller !='':
                    if os.path.isfile(self.GetIniFileName(caller)):
                        conf = configparser.ConfigParser()
                        conf.read(self.GetIniFileName(caller))
                        try:
                            initialdir = conf.get('General', 'initialdir')
                            initialfile = conf.get('General', 'initialfile')
                        except configparser.NoSectionError:
                            raise                      
            except:
                initialdir=''
                initialfile=''
        
        res_filepath = self.tkOpenHandle(initialdir=initialdir, initialfile=initialfile, title=title)

        # after option handling 
        if save_defaults == True:
            if caller!='':
                conf = configparser.RawConfigParser()
                conf.add_section('General')
                conf.set('General', 'initialdir', str( os.path.dirname(res_filepath) ))
                conf.set('General', 'initialfile', str( os.path.basename(res_filepath) ))
                # Writing the ini file
                with open(self.GetIniFileName(caller), 'w') as inifile:
                    conf.write(inifile)          
        return res_filepath


def fileOpenDialog(*args, **kwargs):
    diag = OpenDialog()
    diag.tkOpenHandle = tkinter.filedialog.askopenfilename
    
    return diag.run(*args, **kwargs)

def directoryOpenDialog(*args, **kwargs):
    return tkinter.filedialog.askdirectory()

#
# ListBoxChoice - 
#
class ListBoxChoice(object):
    """
    Usage: 
    root = Tk()
    oLBC = helpers.ListBoxChoice(root, "Choose Variants", "Choose several variants", ['Option1', 'Option2'])
    variantChoiceIdxTuple = oLBC.returnValue()
    """    
    def __init__(self, master=None, title=None, message=None, list=[]):
        self.master = master
        self.value = None
        self.list = list[:]
        
        self.modalPane = tkinter.Toplevel(self.master)

        self.modalPane.transient(self.master)
        self.modalPane.grab_set()

        self.modalPane.bind("<Return>", self._choose)
        self.modalPane.bind("<Escape>", self._cancel)

        if title:
            self.modalPane.title(title)

        if message:
            tkinter.Label(self.modalPane, text=message).pack(padx=5, pady=5)

        listFrame = tkinter.Frame(self.modalPane)
        listFrame.pack(side=tkinter.TOP, padx=5, pady=5)
        
        scrollBar = tkinter.Scrollbar(listFrame)
        scrollBar.pack(side=tkinter.RIGHT, fill=tkinter.Y)
        self.listBox = tkinter.Listbox(listFrame, width=100, height=20, selectmode=tkinter.MULTIPLE)
        self.listBox.pack(side=tkinter.LEFT, fill=tkinter.Y)
        scrollBar.config(command=self.listBox.yview)
        self.listBox.config(yscrollcommand=scrollBar.set)
        self.list.sort()
        for item in self.list:
            self.listBox.insert(tkinter.END, item)

        buttonFrame = tkinter.Frame(self.modalPane)
        buttonFrame.pack(side=tkinter.BOTTOM)

        chooseButton = tkinter.Button(buttonFrame, text="Choose", command=self._choose)
        chooseButton.pack()

        cancelButton = tkinter.Button(buttonFrame, text="Cancel", command=self._cancel)
        cancelButton.pack(side=tkinter.RIGHT)

    def _choose(self, event=None):
        try:
            self.value = self.listBox.curselection()
        except IndexError:
            self.value = None
        self.modalPane.destroy()

    def _cancel(self, event=None):
        self.modalPane.destroy()
        
    def returnValue(self):
        self.master.wait_window(self.modalPane)
        return self.value

#
# Copy with Progress on Command Line
#
def copyFile(SOURCE_FILENAME, TARGET_FILENAME):
    source_size = os.stat(SOURCE_FILENAME).st_size
    copied = 0
    if not os.path.isdir( os.path.dirname(TARGET_FILENAME) ):
        os.makedirs(os.path.dirname(TARGET_FILENAME))
    with open(SOURCE_FILENAME, 'rb') as source:
        with open(TARGET_FILENAME, 'wb') as target:
    
            while True:
                chunk = source.read(32768)
                if not chunk:
                    break
                target.write(chunk)
                copied += len(chunk)
                #print('\r%02d%%' % (copied * 100 / source_size), end=' ')
                print('\r%02d%%' % (copied * 100 / source_size))
        
    print('\n')

#
# Create wxEntryDialog for Comments
#
class CreateEntryDialog(wx.Dialog):
    def __init__(self, parent, title, caption, defaultText=""):
        style = wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER
        super(CreateEntryDialog, self).__init__(parent, -1, title, style=style)
        text = wx.StaticText(self, -1, caption)
        input = wx.TextCtrl(self, -1, style=wx.TE_MULTILINE)
        input.SetInitialSize((400, 150))
        buttons = self.CreateButtonSizer(wx.OK|wx.CANCEL)
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(text, 0, wx.ALL, 5)
        sizer.Add(input, 1, wx.EXPAND|wx.ALL, 5)
        sizer.Add(buttons, 0, wx.EXPAND|wx.ALL, 5)
        self.SetSizerAndFit(sizer)
        self.input = input
        self.input.SetValue(defaultText)
        self.Center()

    def GetValue(self):
        return self.input.GetValue()

def ShowEntryDialog(title, caption, defaultText=""):
    app = wx.PySimpleApp()
    dialog = CreateEntryDialog(None, title, caption, defaultText)
    userInput = None
    if dialog.ShowModal() == wx.ID_OK:
        userInput = dialog.GetValue()
    dialog.Destroy()
    app.MainLoop()
    userInput = str(unicodedata.normalize('NFKD', userInput).encode('ascii','ignore'))
    return userInput