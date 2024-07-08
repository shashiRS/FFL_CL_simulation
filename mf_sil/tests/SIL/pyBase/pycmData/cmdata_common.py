import re

### Dynamic class ###
class cDynamicProperty(object):
    def setAttribute(self, key_str, value):
        key_split = key_str.split('.')

        if len(key_split) == 1:
            # last key -> set value
            setattr(self, key_split[-1], value)
        else:
            if hasattr(self, key_split[0]):
                getattr(self, key_split[0]).setAttribute('.'.join(key_split[1:]), value)
            else:
                # recursive call
                tmp = cDynamicProperty()
                tmp.setAttribute('.'.join(key_split[1:]), value)
                # set created object
                setattr(self, key_split[0], tmp)

    def _setAttributes(self):
        for currAttribute in self._Attributes:
            self.setAttribute(currAttribute[0], currAttribute[1])

### Simple Float, Integer and String ###
class cSimpleDataType(object):
    def __init__(self, fileDescription, datatype, value=None, valueChecker=None, splitter=" = ", srcTestRunFile=None):
        self._desc      = fileDescription
        self._datatype  = datatype
        self._srcTRFile = srcTestRunFile
        self._valChck   = valueChecker
        self._splt      = splitter
        if self._srcTRFile is not None:
            value = self._readTestRunFile(value)
        self._value     = value
        self._rightDataType(value) 

    def readfromFile(self, srcFile, EndStrNxtLn='##'): ## EndStrNxtLn for Workaround for Text parts
        helpvar = str()
        with open(srcFile, 'r') as f:
            existingvarlist = f.readlines()
            counter = 0
            if self._splt == "": elementTofind = self._desc
            else: elementTofind = self._desc + self._splt[0]
            for row in existingvarlist:
                counter += 1               
                if (row).find(elementTofind) is 0:
                    if self._splt == ":\n":
                        while counter <= len(existingvarlist) and (existingvarlist[counter][0:1] == "\t" or existingvarlist[counter][0:4] == " "*4):
                            helpvar = helpvar + existingvarlist[counter]
                            counter += 1
                    # Workaround for copied parts of Vehicle file as str
                    elif self._value != None and self._datatype == str and self._value.find('\n') > 0:                        
                        helpvar = row[len(self._desc):]
                        while counter < len(existingvarlist) and existingvarlist[counter].find(EndStrNxtLn) is -1:
                            helpvar = helpvar + existingvarlist[counter] 
                            counter += 1
                    else:
                        helpvar = row[len(self._desc+self._splt):]
                    if (helpvar[:-1] is not '') or (self._datatype == str and helpvar[:-1] is ''):
                        helpvar = re.sub(r"(\$[^$=]+=)",r"",helpvar); # Remove testmanager variables from helpvar
                        self._value = self._datatype((helpvar[:-1]))
                        self._rightDataType(self._value)
                    break

    def _rightDataType(self, value):
        valid = True
        if value is not None:
            if (self._datatype != value.__class__):
                print("ERROR in cSimpleDataType: {0} expected! {1} is not allowed!".format(self._datatype, value.__class__))
                valid = False
            if self._valChck is not None and self._valChck(value) is False:
                print("ERROR in cSimpleDataType: One value is {0} and thus out of bounce!".format(value))
                valid = False
        return valid

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        if (True==self._rightDataType(value)):
            self._value = value

    def getConfigFileText(self):
        if self._value is None:
            outputvalue = ""
        else:
            outputvalue = str(self._value)
        return self._desc + self._splt + outputvalue + "\n"

    def __str__(self):
        return 'Value: {0}'.format(self._value)


### 3D Vector as float###
class cSimple3Dfloat(object):
    def __init__(self, fileDescription, value, valueChecker=None):
        self._desc = fileDescription
        self._value = value
        self._x = value[0]
        self._y = value[1]
        self._z = value[2]
        self._valChck  = valueChecker
        self._rightDataType(value)
        self._splt = " ="
        
    def readfromFile(self, srcFile):
        with open(srcFile, 'r') as f:
            existingvarlist = f.readlines()
            found = False
            for row in existingvarlist:
                if row.find(self._desc + self._splt) is 0:
                    dummylist = row[len(self._desc+self._splt):].strip().split(' ')
                    #dummylist = filter(None, dummylist)
                    found = True
                elif row.find(self._desc + self._splt[:-1]) is 0:
                    dummylist = row[len(self._desc+self._splt[:-1]):].strip().split(' ')
                    #dummylist = filter(None, dummylist)
                    found = True
                if found:
                    self._value = []
                    for item in dummylist[:3]:
                        item = re.sub(r"(\$[^$=]+=)",r"",item); # Remove testmanager variables from helpvar
                        self._value.append(float(item))
                    self._rightDataType(self._value)
                    break  

    def _rightDataType(self, value_list):
        valid = True
        if len(value_list) > 3:
            print("ERROR in cSimple3Dfloat: More vector elements than expected!")
            valid = False
        for element in value_list:
            if (float != element.__class__):
                print("ERROR in cSimple3Dfloat: {0} expected! {1} is not allowed!".format(float, element.__class__))
                valid = False
            if self._valChck is not None and self._valChck(element) is False:
                print("ERROR in cSimple3Dfloat: One value is {0} and thus out of bounce!".format(element))
                valid = False
        return valid

    def _singleRightDataType(self, value):
        valid = True
        if value is not None:
            if (float != value.__class__):
                print("ERROR in cSimpleDataType: {0} expected! {1} is not allowed!".format(float, value.__class__))
                valid = False
            if self._valChck is not None and self._valChck(value) is False:
                print("ERROR in cSimpleDataType: One value is {0} and thus out of bounce!".format(value))
                valid = False
        return valid

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        if (True==self._rightDataType(value)):
            self._value = value
            self._x = value[0]
            self._y = value[1]
            self._z = value[2]

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, x):
        if (True==self._singleRightDataType(x)):
            self._x = x
            self._value[0] = x

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, y):
        if (True==self._singleRightDataType(y)):
            self._y = y
            self._value[1] = y

    @property
    def z(self):
        return self._z

    @z.setter
    def z(self, z):
        if (True==self._singleRightDataType(z)):
            self._z = z
            self._value[2] = z

    def getConfigFileText(self):
        return self._desc + " = " + str(self._value[0]) + " " + str(self._value[1]) + " " + str(self._value[2]) + "\n"

    def __str__(self):
        return 'Value: {0}'.format(self._value)

### 6D Vector as float based on cSimple3Dfloat###
class cSimple6Dfloat(cDynamicProperty):
    def __init__(self, fileDescription, detailDescription1, value1, detailDescription2, value2):
        self._desc = fileDescription
        self._Attributes = []
        self._Attributes.append((detailDescription1, cSimple3Dfloat(detailDescription1, value1)))
        self._Attributes.append((detailDescription2, cSimple3Dfloat(detailDescription2, value2)))
        self._setAttributes()
        self._splt = " = "

    def readfromFile(self, srcFile):
        with open(srcFile, 'r') as f:
            existingvarlist = f.readlines()
            found = False
            for row in existingvarlist:
                if row.find(self._desc + self._splt) is 0:
                    dummylist = row[len(self._desc+" = "):].split(' ')
                    found = True
                elif row.find(self._desc + self._splt[:-1]) is 0:
                    dummylist = row[len(self._desc+self._splt[:-1]):].split(' ')
                    found = True
                if found:
                    self._value = []
                    for item in dummylist:
                        if item != '':
                            self._value.append(float(item))
                    self._Attributes[0][1]._value = self._value[0:3]
                    self._Attributes[1][1]._value = self._value[3:6]
                    break

    def getConfigFileText(self):
        return (self._desc + " = " + str(self._Attributes[0][1]._value[0]) + " " + str(self._Attributes[0][1]._value[1]) + " " + str(self._Attributes[0][1]._value[2]) + " "
                + str(self._Attributes[1][1]._value[0]) + " " + str(self._Attributes[1][1]._value[1]) + " " + str(self._Attributes[1][1]._value[2]) + "\n")

    def __getitem__(self,index):
        if index < 3:
            retValue = self._Attributes[0][1]._value[index]
        if index >= 3:
            retValue = self._Attributes[1][1]._value[index-3]
        return retValue

    def __setitem__(self, index, newvalue):
        if index < 3:
            self._Attributes[0][1].value[index] = newvalue
            self._Attributes[0][1].x = self._Attributes[0][1].value[0]
            self._Attributes[0][1].y = self._Attributes[0][1].value[1]
            self._Attributes[0][1].z = self._Attributes[0][1].value[2]
        if index >= 3:
            self._Attributes[1][1].value[index-3] = newvalue
            self._Attributes[1][1].x = self._Attributes[1][1].value[0]
            self._Attributes[1][1].y = self._Attributes[1][1].value[1]
            self._Attributes[1][1].z = self._Attributes[1][1].value[2]


