#
# System Imports
#
import struct
import os
import sys
#sys = reload(sys)
#sys.setdefaultencoding('utf-8')
import logging
logger = logging.getLogger("pyBase")

# ============================================================================ #
# File Format Specifications ERG Type 2
# ============================================================================ #
""" Format Identifier (ERG-File type 2 -> Value= 'CM-ERG' padded with zeros) """
ERG_V2_HEADER_FORMAT_IDENTIFIER_STR_POSITION = 1
ERG_V2_HEADER_FORMAT_IDENTIFIER_STR_BYTE_LENGTH = 8

""" File Format Version Number """
ERG_V2_HEADER_FILE_FORMAT_VERSION_NUMBER_UINT_POSITION = 2
ERG_V2_HEADER_FILE_FORMAT_VERSION_NUMBER_UINT_BYTE_LENGTH = 1

""" Byte Order (Value=0 means little-endian, Value!=0 means big-endian) """
ERG_V2_HEADER_BYTE_ORDER_BOOL_POSITION = 3
ERG_V2_HEADER_BYTE_ORDER_BOOL_BYTE_LENGTH = 1

""" Record Size """
ERG_V2_HEADER_RECORD_SIZE_IN_BYTE_UINT_POSITION = 4
ERG_V2_HEADER_RECORD_SIZE_IN_BYTE_UINT_BYTE_LENGTH = 2

""" Unused space as placeholder for future versions """
ERG_V2_HEADER_PLACEHOLDER_POSITION = 5
ERG_V2_HEADER_PLACEHOLDER_BYTE_LENGTH = 4
# ---------------------------------------------------------------------------- #


# ============================================================================ #
# Datatype Descriptions
# ============================================================================ #
""" List of the byte length of all possible used datatypes  """
DATATYPE_DOUBLE_LENGTH_BYTE    = 8
DATATYPE_FLOAT_LENGTH_BYTE     = 4
DATATYPE_LONGLONG_LENGTH_BYTE  = 8
DATATYPE_ULONGLONG_LENGTH_BYTE = 8
DATATYPE_LONG_LENGTH_BYTE      = 4
DATATYPE_ULONG_LENGTH_BYTE     = 4
DATATYPE_INT_LENGTH_BYTE       = 4
DATATYPE_UINT_LENGTH_BYTE      = 4
DATATYPE_SHORT_LENGTH_BYTE     = 2
DATATYPE_USHORT_LENGTH_BYTE    = 2
DATATYPE_CHAR_LENGTH_BYTE      = 1
DATATYPE_UCHAR_LENGTH_BYTE     = 1

""" Map type names into format characters used by the module struct """
DATATYPE_NAME_TO_STRUCT_REPR = {'Double'   :'d',          \
                                'Float'    :'f',          \
                                'LongLong' :'q',  #int64  \
                                'ULongLong':'Q',  #uint64 \        
                                'Long'     :'i',  #int32  \
                                'ULong'    :'I',  #uint32 \
                                'Int'      :'i',  #int32  \
                                'UInt'     :'I',  #uint32 \ 
                                'Short'    :'h',  #int16  \ 
                                'UShort'   :'H',  #uint16 \
                                'Char'     :'b',          \
                                'UChar'    :'B',          \
                                }
# TODO: 'n Bytes' not handled

""" Byte Order Representations in ERG File Header and for the Module Struct """
DATATYPE_BYTE_ORDER_BIG_ENDIAN_ERG_REPR = 1
DATATYPE_BYTE_ORDER_BIG_ENDIAN_STRUCT_REPR = '>'
DATATYPE_BYTE_ORDER_LITTLE_ENDIAN_ERG_REPR = 0
DATATYPE_BYTE_ORDER_LITTLE_ENDIAN_STRUCT_REPR = '<'

DATATYPE_BYTE_ORDER_MAP_TO_STRUCT_REPR = {'big':DATATYPE_BYTE_ORDER_BIG_ENDIAN_STRUCT_REPR,\
                                          'little':DATATYPE_BYTE_ORDER_LITTLE_ENDIAN_STRUCT_REPR,\
                                          }
# ---------------------------------------------------------------------------- #


# ============================================================================ #
# Info file Configs
# ============================================================================ #
INFO_FILE_DATA_VALUE_PREFIX_STR     = 'File.At.'
INFO_FILE_DATA_NAME_STR             = 'Name'
INFO_FILE_DATA_TYPE_STR             = 'Type'
INFO_FILE_DATA_QUANTITY_PREFIX_STR  = 'Quantity'
INFO_FILE_DATA_QUANTITY_POSTFIX_STR = 'Unit'

""" Example of the first data entry in a info-file """
#File.At.1.Name =	Time
#File.At.1.Type =	Double
#Quantity.Time.Unit =	s

# ---------------------------------------------------------------------------- #


# ============================================================================ #
# Data Structur Layout (used to describe the record-data inside python)
# ============================================================================ #
""" Essential values of each Record """
DATA_RECORD_ESSENTIAL_VALUES   = [INFO_FILE_DATA_NAME_STR, INFO_FILE_DATA_TYPE_STR]
DATA_RECORD_ESSENTIAL_NAME_IDX = 0
DATA_RECORD_ESSENTIAL_TYPE_IDX = 1

DATA_RECORD_NUMBER_OF_VALUES = 4
DATA_RECORD_POSITION_IDX     = 0
DATA_RECORD_NAME_IDX         = 1
DATA_RECORD_TYPE_IDX         = 2
DATA_RECORD_QUANTITY_IDX     = 3
DATA_RECORD_QUANTITY_DEFAULT = ''

""" Example of a dataRecord entry """
#dataStruct = [[1, 'Name', 'Type', 'Quantity'],[1, 'Time', 'Double', 's']]
# ---------------------------------------------------------------------------- #


#
# Class Definition
#
class cErgFile:
    """
    Class to read erg files (Carmaker result files).

    :param ergFilePath: Absolute path to the erg file
    :param cacheWholeFile: If true, reads the whole erg file once. If false, it reads each quantity if it is necessary. This may lead to worse performance.
    """

    # TODO:
    # - handle little and big endian! -> already startet... not finished
    
    # TODO unlikely:
    # - add some status messages (logging)

    # TODO unlikely:
    # - show the timeline (a vector of all timesteps) -> only possible, when a time value is available!
    
    # TODO unlikely:
    # - Usecase: ask for a single variable (like 'time') and give it back as tuple (time, value)
    
    # TODO unlikely:
    # - Usecase: ask for a single variable vector -> return just the values as vector
    
    def __init__(self,ergFilePath, cacheWholeFile=True):
        self._filePath = ergFilePath
        self._readPosition_byte = 0
        self._countRecordsRead = 0
        
        
        # get the data offset by the header (header length in byte)
        self._headerLength_byte = ERG_V2_HEADER_FORMAT_IDENTIFIER_STR_BYTE_LENGTH \
            + ERG_V2_HEADER_FILE_FORMAT_VERSION_NUMBER_UINT_BYTE_LENGTH \
            + ERG_V2_HEADER_BYTE_ORDER_BOOL_BYTE_LENGTH \
            + ERG_V2_HEADER_RECORD_SIZE_IN_BYTE_UINT_BYTE_LENGTH \
            + ERG_V2_HEADER_PLACEHOLDER_BYTE_LENGTH
        
        # parse the info file to get the data structure to know
        self._infoFileHandle = open(ergFilePath+'.info', mode='r')
        self._recordDescription = self._parseInfoFile()
        self._recordLength_byte = self._getRecordLength()
        
        # analyze the data file
        self._recordsTotalCount = self._getTotalRecordsCount()
        
        # open the data file (important to open as (r)ead (b)inary
        self._ergFileHandle = open(ergFilePath, mode='rb', buffering=0)
        
        # read the header data
        self._header_raw = self._readHeader()
        
        # set the byte order
        self._byteOrderStr = self._setByteOrder(self._header_raw['byte_order'])
        
        # check data
        self._checkIntegrety()

        self._wholeContent = {};
        self._cachedWholeFile = False
        self._currentRecord = 0 #is used whenever the whole file is cached to keep track of the current record
        if(cacheWholeFile==True):
            self.readWholeErgFile();
            self._cachedWholeFile = True
            

            
        
    def __del__(self):
        try:
            self._ergFileHandle.close()
        except:
            #if erg file doesn't exist or error occured when reading.
            pass
        
    # to use the withstatement
    def __enter__(self):
        return self
    def __exit__(self, type, value, traceback):
        self.__del__()

    def __iter__(self):
        return self

    def next(self):
        currData = self.readNextTimeStep()
        if currData == '':
            raise StopIteration
        else:
            return currData

    def __next__(self):
        currData = self.readNextTimeStep()
        if currData == '':
            raise StopIteration
        else:
            return currData

    def _setByteOrder(self, byteOrderErgFormat):
        byteOrderStr = ''
        if byteOrderErgFormat == DATATYPE_BYTE_ORDER_LITTLE_ENDIAN_ERG_REPR:
            byteOrderStr = 'little'
        else:
            byteOrderStr = 'big'
        return byteOrderStr
        
    def getArrayOfAvailableValues(self):
        """
        Get array of available signals.
        :return: List of strings with all available values by name
        """

        valuesStr = []
        for valueDescription in self._recordDescription:
            valuesStr.append(valueDescription[DATA_RECORD_NAME_IDX])
        return valuesStr

    def getUnitOfValue(self,valueStr):
        """ 
        Get the unit of a value.

        :return: String representation of the value's unit. If no value found in erg file with the name valueStr, an empty string is returned!
        """

        unitStr = ''
        for valueDescription in self._recordDescription:
            if valueDescription[DATA_RECORD_NAME_IDX] == valueStr:
                unitStr = valueDescription[DATA_RECORD_QUANTITY_IDX]
                break
        return unitStr
    
    def jumpToRecordById(self, recordId):
        """
        Set the current reading position of the erg data file to the package with the id recordId.
        Raise an exception if the recordID exceeds the total record count.
        
        :return: True if succeeded.
        """

        success = False
        
        if recordId <= self._recordsTotalCount:
            if self._cachedWholeFile == False:
                byteToJumpTo = ((recordId*self._recordLength_byte)+self._headerLength_byte)
                self._ergFileHandle.seek(byteToJumpTo , os.SEEK_SET)
                self._readPosition_byte = byteToJumpTo
            else:
                self._currentRecord = recordId
            success = True
        else:
            raise IOError('Error while jumping to data package: ' + str(recordId) +' .There are only ' +str(self._recordsTotalCount) +' data packages available!' )
        return success
    
    def getCurrentRecordId(self):
        """
        Get the current Package id (floor the current byte-position)

        :return: Current package ID
        """

        if(self._cachedWholeFile == False):
            packageId = int( ( self._readPosition_byte - self._headerLength_byte ) / self._recordLength_byte )
            return packageId
        else:
            return self._currentRecord

    def _getTotalRecordsCount(self):
        """
        Get the total record count.
        Raise an exception if filesize is not coherent to package length.

        :return: Number of total records.
        """

        dataFileSize = os.path.getsize(self._filePath) # get the Filesize in bytes
        
        # raise error in case the filesize is not coherent!
        try:
            if (( dataFileSize - self._headerLength_byte ) % self._recordLength_byte) != 0:
                # os.remove(self._filePath)
                # if os.path.exists(str(self._filePath) + ".info"):
                #     os.remove(str(self._filePath) + ".info")
                raise IOError('Filesize is not coherent to package length!')
        except:pass
        
        return int( ( dataFileSize - self._headerLength_byte ) / self._recordLength_byte )


    def _parseInfoFile(self):
        """
        Parse the info file.
        Raise an exception if something goes wrong.

        :return: Data struct with record information.
        """

        # init return value
        dataStruct = []
        
        # init tmp values
        reading_not_ready = True
        read_next_line = False
        
        line = ''
        essentials_read = []
        
        # STATEMACHINE: 
        # - 0 idle (until new variable is found -> reading_essentials)
        STATE_IDLE = 0
        # - 1 reading_essentiales (until must have are completed -> reading)
        STATE_READING_ESSENTIALS = 1
        # - 2 reading optional (until blank line -> idle + save dataSet (consists f.e. of Name,Type,(Unit))
        STATE_READING_OPTIONAL = 2
        # - 3 init
        STATE_INIT = 3
        
        # set the initial state of the statemachine
        act_state = STATE_INIT
        
        # start reading the info-file!
        while reading_not_ready:            
            if read_next_line:
                line = self._infoFileHandle.readline()
                line = line.strip('"')                 # To remove if any double quotes in erg.info file
                if len(line) == 0:
                    reading_not_ready = False
                    break
            
            # process the statemachine:
            if (act_state) == STATE_IDLE:
                # STATE idle (waiting for new variable)
                                
                if line[0:len(INFO_FILE_DATA_VALUE_PREFIX_STR)] == INFO_FILE_DATA_VALUE_PREFIX_STR:
                    # new Variable-Statement in info file found
                    act_dataSet[DATA_RECORD_POSITION_IDX] = int( line[len(INFO_FILE_DATA_VALUE_PREFIX_STR):].split('.')[0] )
                    
                    read_next_line = False
                    act_state = STATE_READING_ESSENTIALS
                    
            elif (act_state) == STATE_READING_ESSENTIALS:
                # STATE reading_essentials (get expected values: Name, Type)
                act_value_name_str = line[(len(INFO_FILE_DATA_VALUE_PREFIX_STR) + len(str(act_dataSet[DATA_RECORD_POSITION_IDX])) + 1):].split(' ')[0].strip()
                act_value_value_str = line.split('=')[1].strip()
                
                if act_value_name_str == INFO_FILE_DATA_NAME_STR:
                    act_dataSet[DATA_RECORD_NAME_IDX] = act_value_value_str
                    essentials_read[DATA_RECORD_ESSENTIAL_NAME_IDX] = True
                    
                elif act_value_name_str == INFO_FILE_DATA_TYPE_STR:
                    act_dataSet[DATA_RECORD_TYPE_IDX] = act_value_value_str
                    essentials_read[DATA_RECORD_ESSENTIAL_TYPE_IDX] = True
                    
                else:
                    raise SyntaxError('The ValueName: ' + act_value_name_str + ' is not expected! Error while reading the INFO-file!')
                
                read_next_line = True
                
                if all( essentials_read ):
                    act_state = STATE_READING_OPTIONAL
                
            elif (act_state) == STATE_READING_OPTIONAL:
                # STATE reading optional (get optional values until \n Nextline statement is detected)
                if line == '\n':
                    # newline found
                    
                    # save act_dataSet to big dataset
                    dataStruct.append(act_dataSet)
                    
                    # switch to STATE_IDLE again!
                    act_state = STATE_INIT
                    
                    read_next_line = False
                    
                elif line[0:len(INFO_FILE_DATA_QUANTITY_PREFIX_STR)] == INFO_FILE_DATA_QUANTITY_PREFIX_STR:
                    # Quantity found
                    act_dataSet[DATA_RECORD_QUANTITY_IDX] = line.split('=')[1].strip()
                else:
                    raise SyntaxError('Error while reading the INFO-file! The Line: "' + line + '" can not be interpreted! ')
                    
            elif act_state == STATE_INIT:
                # STATE init (init and reset data structur)
                act_dataSet = [None]*DATA_RECORD_NUMBER_OF_VALUES
                essentials_read = [False for i in range(len(DATA_RECORD_ESSENTIAL_VALUES))]
                
                # switch to STATE_IDLE again!
                act_state = STATE_IDLE
                
                read_next_line = True
                
            else:
                raise SyntaxError('Error this state does not exists: ' + str( act_state ))
        
        return dataStruct

    def _getRecordLength(self):
        """
        Get the length of a record.

        :return: Total datablock length (bytes)
        """

        totalLength_byte = 0
        
        for valueDescription in self._recordDescription:
            totalLength_byte += self._getDatatypeLength_byte(valueDescription[DATA_RECORD_TYPE_IDX])
        
        return totalLength_byte

    def _getDatatypeLength_byte(self, dataTypeStr):
        """
        Get the length of a datatype.
        Raise an exception if the datatype is unknown.

        :param dataTypeStr: Datatype, e.g. "Double", "Float", etc.

        :return: Data length (bytes)
        """

        if dataTypeStr == 'Double':
            dataLength = DATATYPE_DOUBLE_LENGTH_BYTE
        elif dataTypeStr == 'Float':
            dataLength = DATATYPE_FLOAT_LENGTH_BYTE
        elif dataTypeStr == 'LongLong':
            dataLength = DATATYPE_LONGLONG_LENGTH_BYTE
        elif dataTypeStr == 'ULongLong':
            dataLength = DATATYPE_ULONGLONG_LENGTH_BYTE
        elif dataTypeStr == 'Long':
            dataLength = DATATYPE_LONG_LENGTH_BYTE
        elif dataTypeStr == 'ULong':
            dataLength = DATATYPE_ULONG_LENGTH_BYTE
        elif dataTypeStr == 'Int':
            dataLength = DATATYPE_INT_LENGTH_BYTE
        elif dataTypeStr == 'UInt':
            dataLength = DATATYPE_UINT_LENGTH_BYTE
        elif dataTypeStr == 'Short':
            dataLength = DATATYPE_SHORT_LENGTH_BYTE
        elif dataTypeStr == 'UShort':
            dataLength = DATATYPE_USHORT_LENGTH_BYTE
        elif dataTypeStr == 'Char':
            dataLength = DATATYPE_CHAR_LENGTH_BYTE
        elif dataTypeStr == 'UChar':
            dataLength = DATATYPE_UCHAR_LENGTH_BYTE
        elif dataTypeStr[-5:] == 'Bytes':
            dataLength = int( dataTypeStr.split()[0] )
        else:
            raise SyntaxError('Error! Unknown Datatype: ' + dataTypeStr)        
        return dataLength
    
    def _readHeader(self):
        """
        Read the header.

        :return: Dictionary with header information
        """

        header_raw = {\
            'format_identifier': ''.join( set.decode('ascii') for set in struct.unpack( 'ssssssss', self._ergFileHandle.read(ERG_V2_HEADER_FORMAT_IDENTIFIER_STR_BYTE_LENGTH) ) ),\
            'version_number'   : struct.unpack( 'B', self._ergFileHandle.read(ERG_V2_HEADER_FILE_FORMAT_VERSION_NUMBER_UINT_BYTE_LENGTH) )[0],\
            'byte_order'       : struct.unpack( 'b', self._ergFileHandle.read(ERG_V2_HEADER_BYTE_ORDER_BOOL_BYTE_LENGTH) )[0],\
            'record_size'      : struct.unpack( 'H', self._ergFileHandle.read(ERG_V2_HEADER_RECORD_SIZE_IN_BYTE_UINT_BYTE_LENGTH) )[0],\
            'placeholder'      : self._ergFileHandle.read( ERG_V2_HEADER_PLACEHOLDER_BYTE_LENGTH),\
            }
        self._readPosition_byte += self._headerLength_byte
        
        return header_raw
    
    def readNextTimeStep(self):
        """
        Read the next dataPackage in the erg-file.
        If the end of the erg-file is reached, the function returns an empty string.

        :return: values as a dict={'ValueName':'Value',...} for this time step.
        """

        dataDict = cRobustDict()		
        if self._cachedWholeFile == False:
            for valueDescription in self._recordDescription:
                if not valueDescription[DATA_RECORD_TYPE_IDX][-5:] == 'Bytes':
                    # read data
                    raw_data = self._ergFileHandle.read( self._getDatatypeLength_byte(valueDescription[DATA_RECORD_TYPE_IDX]) )
                    self._readPosition_byte += self._getDatatypeLength_byte(valueDescription[DATA_RECORD_TYPE_IDX])
                    
                    #unpack data
                    if raw_data != b'' or raw_data != '':
                        try:
                            unpacked_data = struct.unpack( DATATYPE_NAME_TO_STRUCT_REPR[valueDescription[DATA_RECORD_TYPE_IDX]], raw_data )[0]
                            dataDict[valueDescription[DATA_RECORD_NAME_IDX]] = unpacked_data
                        except:
                            raise StopIteration

                    else:
                        # no more data to read!
                        dataDict = ''
                        break
                else:
                    # read data and throw it away!
                    dataLength = int( valueDescription[DATA_RECORD_TYPE_IDX].split()[0] )
                    raw_data = self._ergFileHandle.read( dataLength )
            
            self._countRecordsRead += 1
        else:
            if self._currentRecord < self._recordsTotalCount:
                for valueDescription in self._recordDescription:
                    if not valueDescription[DATA_RECORD_TYPE_IDX][-5:] == 'Bytes':
                        dataDict[valueDescription[DATA_RECORD_NAME_IDX]]  = self._wholeContent[valueDescription[DATA_RECORD_NAME_IDX]][self._currentRecord]
                self._currentRecord += 1
            else:
                dataDict = ''
        return dataDict

    def readRecord(self, recordId):
        """
        Read the record for the given ID.

        :param recordId: Record ID

        :return: The record
        """

        self.jumpToRecordById(recordId)
        return self.readNextTimeStep()

    def readSingleSignalValue(self, signalName, recordId):
        """
        Read a single signal value.

        :param signalName: Name of the signal
        :param recordId: Record ID

        :return: Value of the signal for given record ID
        """

        self.jumpToRecordById(recordId)
        return self.readNextTimeStep()[signalName]

    def readAllSignalValues(self, signalName):
        """
        Read all signal values for a given signal.

        :param signalName: Name of the signal

        :return: List of all values for the given signal
        """

        result = []
        if(self._cachedWholeFile == True):
            if(signalName in list(self._wholeContent.keys())):
                result = self._wholeContent[signalName]
        else:        
            self.jumpToRecordById(0)

            for currRecord in self:
                result.append(currRecord[signalName])
        return result

    def readWholeErgFile(self):
        """
        Read the whole erg file.
        """

        self.jumpToRecordById(0)
        for currRecord in self:
            for descr in self._recordDescription:
                self._wholeContent.setdefault(descr[DATA_RECORD_NAME_IDX], [])
                self._wholeContent[descr[DATA_RECORD_NAME_IDX]].append(currRecord[descr[DATA_RECORD_NAME_IDX]])

    def _checkIntegrety(self):
        """
        Check if the data in the erg file are consistent with their header.
        Raise an exception if not plausible.
        """

        if self._recordLength_byte != self._header_raw['record_size']:
            raise IOError('Data Package Length differes between header and reality!')

class cRobustDict(dict):
    def __init__(self):
        dict.__init__(self)
        
    def __getitem__(self,key):
        if key in self:
            res = dict.__getitem__(self,key)
        else:
            # /todo: Change behavior!
            #logging.warning('Key {k} not found!! Returned 0.0 instead!'.format(k=key.upper() ))
            res = 0.0
        return res
        
