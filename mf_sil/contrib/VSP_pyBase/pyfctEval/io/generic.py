#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from abc import ABCMeta, abstractmethod
from typing import List

import numpy as np
import pandas as pd

from datamodel import ObjectDataFrame, SignalDataFrame

__author__ = "Philipp Baust"
__copyright__ = "Copyright 2020, Continental AMS ADAS SYS ADS V&V TES LND"
__credits__ = []
__license__ = "Copyright (c) 2020, Continental AMS ADAS. All rights reserved."
__version__ = "0.2_hotfix1"
__maintainer__ = "Philipp Baust"
__email__ = "philipp.baust@continental-corporation.com"
__status__ = "Development"


class ReaderException(Exception):
    """ Base class for all reader exceptions. """

    pass


class WriterException(Exception):
    """ Base class for add writer exception. """

    pass


class SignalNotFoundException(ReaderException):
    """ Raised when a signal is not available in opened source. """

    pass


class IReader(metaclass=ABCMeta):
    """ Interface for all signal readers.
    All readers are implemented as context managers.
    Signal access is realized as item getters.
    """

    def __init__(self, **kwargs):
        super().__init__()

    # @property
    # def signals(self) -> List[str]:
    #     """ Returns a list of available signals in file.
    #
    #     :return: list of signals
    #     """
    #     return []

    def __getitem__(self, item) -> np.array:
        """ Return the values for the item.

        The implementing class has to decide how to access the values,
        could be either a signal name (for low level readers) or an alias name, or
        even some wildcard.
        """
        raise SignalNotFoundException("")

    def __enter__(self):
        """ Opens the file when used as context manager, which is the preferred
        way of usage.
        """
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """ Closes the reader when used as context manager.

        :param exc_type:
        :param exc_value:
        :param traceback:
        :return:
        """
        self.close()

    @abstractmethod
    def open(self):
        """ Opens the data source and prepares for reading. """
        pass

    @abstractmethod
    def close(self):
        """ Closes the data source. """
        pass


class IObjectReader(metaclass=ABCMeta):
    """ Interface for all data-readers which provide objects.
    In ADAS we consider objects all items which are computed by sensors and
    placed on the object list, depending on the sensor technology this might
    differ drastically:

      * Radar (ARS4xx, ARS5xx, ...): Objects can be traffic participants (car,
        pedestrians, trucks, ...) *and* static objects (guardrails, poles, ...).
      * Camera (MFC4xx, MFC5xx, ...): Usually only traffic participants.
      * Fusion devices (CEM, ...): Also usually traffic participants, although
        they provide a static environment as well. Still in TSF a object list
        is considered to contain traffic participants and optionally other
        objects but not only static environment.
      * Reference sensorics (IBEO, Leica, RT-Range): Objects are traffic
        participants

    The objects are returned as `ObjectDataFrame` which is a subclassed
    pandas dataframe which allows for easy filtering and temporary storage.
    """

    @property
    @abstractmethod
    def objects(self) -> ObjectDataFrame:
        """ Returns a the read objects as subclassed pandas dataframe.

        :return: A object dataframe.
        """
        pass


class ISignalReader(metaclass=ABCMeta):
    """ Interface for all data-readers which provide signals. """

    @property
    @abstractmethod
    def signals(self) -> SignalDataFrame:
        """ Returns a the read objects as subclassed pandas dataframe.

        :return: A object dataframe.
        """
        pass


class ILanesReader(metaclass=ABCMeta):
    """ Interface for all data-readers which provide lanes.

    """

    @property
    @abstractmethod
    def lanes(self) -> pd.DataFrame:
        pass


class IWriter(object):
    """ Interface for all signal writers.
    All writers are implmented as context managers.
    Signal access is realized as item getters.
    """

    def __init__(
        self,
        # filepath="",
        **kwargs
    ):
        """

        :param filepath:
        :param kwargs:
        """
        super(IWriter, self).__init__()
        # self._filepath = filepath

    def __setitem__(self, key, value):
        """ Add a signal to the writer """
        pass

    def __getitem__(self, item):
        raise SignalNotFoundException("")

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def open(self):
        pass

    def close(self):
        pass

    @property
    def signals(self) -> List[str]:
        return []


class GenericObjectDefinition(object):
    """ Base Definition for all object list.

    A object list is guaranteed to have the properties defined in
    :py:class:`io.sensorics_objects.GenericObjectDefinition.DefaultProperties`

    .. hint::
       All reader configs shall inherit from that class. Recomendation is to
       extend the DefaultProperties enum as well. See Objectreading guideline
       for details.

    """

    # TODO: Move to correct module

    class Columns(object):
        """ Enum defining the default columns all object list have in common.
        """

        TIMESTAMP = "timestamp"
        TSF_INDEX = "tsf_index"

        #: Longitudinal Distance to ref point of object.
        #  Note: The exact location is not defined and may be sensor dependent.
        DIST_X = "dist_x"

        #: Lateral Distance to ref point of object.
        #  Note: The exact location is not defined and may be sensor dependent.
        DIST_Y = "dist_y"

        #: Relative longitudinal velocity of object.
        VREL_X = "vrel_x"

        #: Object x-coordinate of corner 0 relative to dist_x.
        POS_X0 = "pos_x0"

        #: Object x-coordinate of corner 1 relative to dist_x.
        POS_X1 = "pos_x1"

        #: Object x-coordinate of corner 2 relative to dist_x.
        POS_X2 = "pos_x2"

        #: Object x-coordinate of corner 3 relative to dist_x.
        POS_X3 = "pos_x3"

        #: Object y-coordinate of corner 0 relative to dist_y.
        POS_Y0 = "pos_y0"

        #: Object y-coordinate of corner 1 relative to dist_y.
        POS_Y1 = "pos_y1"

        #: Object y-coordinate of corner 2 relative to dist_y.
        POS_Y2 = "pos_y2"

        #: Object y-coordinate of corner 3 relative to dist_y.
        POS_Y3 = "pos_y3"

    def __init__(self):
        super().__init__()
        self._root = ""
        self._properties = []
        self._size = -1

    @property
    def object_properties(self):
        return self._properties

    @property
    def object_root(self):
        return self._root

    @property
    def list_size(self):
        return self._size
