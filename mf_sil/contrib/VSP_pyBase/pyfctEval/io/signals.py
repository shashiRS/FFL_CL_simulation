#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sensorics objects
-----------------

The modules provide a specialized reader to work with sensorics objects, as
given by algo reprocessing.

The usual file format is BSIG.
"""
import logging

import numpy as np
import pandas as pd

from io.bsig import , Multi
from io.datamodel import SignalDataFrame
from io.generic import IReader, ISignalReader

__author__ = "Philipp Baust"
__copyright__ = "Copyright 2020, Continental AMS ADAS SYS ADS V&V TES LND"
__credits__ = []
__license__ = "Copyright (c) 2020, Continental AMS ADAS. All rights reserved."
__version__ = "0.2_hotfix1"
__maintainer__ = "Philipp Baust"
__email__ = "philipp.baust@continental-corporation.com"
__status__ = "Development"


_log = logging.getLogger(__name__)


class SignalDefinition(object):
    class Columns(object):
        """ Enum defining the default columns all object list have in common.
        """

        TIMESTAMP = "timestamp"

        #: MTS Timestamp
        MTS_TS = "mts_ts"

        #: Optional UTC Timestamp (if tstp fiel is provided)
        UTC_TS = "utc_ts"

        # #: Object track number
        # TRACK = "track"

    def __init__(self):
        super().__init__()
        self._root = ""
        self._properties = []

    @property
    def signal_root(self):
        return self._root

    @property
    def signal_properties(self):
        return self._properties


class SignalReader(IReader, ISignalReader):
    """ Reader for sensorics object list (e.g ARSxxx, MFCxxx).

    It is assumed that the signal maintenance state ist provided which
    tracks the object lifetime (This assumption holds true for all newer
    generation sensorics algorithms).
    """

    MTS_TS_SIGNAL = "MTS.Package.TimeStamp"
    UTC_TS_SIGNAL = "MTS.Package.UtcTimeStamp"

    def __init__(
        self, filename, signal_definition: SignalDefinition, tstp_filename=None
    ):
        """ Constructor.

        :param filename: full path to the files that should be read
          (e.g. bsig, h5, ...)
        :param object_definition: Object definition object which maps what
          signal shall be read.
        :param tstp_filename:
        """
        super().__init__()
        if isinstance(filename, (tuple, list)) and len(filename) > 1:
            self._rdr = Multi(*filename)
        elif isinstance(filename, (tuple, list)):
            self._rdr = (filename[0])
        else:
            self._rdr = (filename)

        self._defs = signal_definition

        self._df = None

    def open(self):
        self._rdr.open()

        # Read object data one by one
        mts_ts = self._rdr[self.MTS_TS_SIGNAL].astype(np.int64)
        self._df = SignalDataFrame({self._defs.Columns.MTS_TS: mts_ts}, index=mts_ts)

        for alias, signal_name_suffix in self._defs.signal_properties:
            if signal_name_suffix.startswith("."):
                fqn = "{}{}".format(self._defs.signal_root, signal_name_suffix)
            else:
                fqn = signal_name_suffix

            if "%" in fqn:
                p0, p1 = fqn.split("%")
                v = [
                    int(k[len(p0) : -len(p1)])
                    for k in self._rdr.signals
                    if k.startswith(p0) and k.endswith(p1)
                ]

                for k in v:
                    fqn = "{}{}{}".format(p0, k, p1)
                    d = self._rdr[fqn]
                    self._df[(alias, k)] = d

            else:
                sig = self._rdr[fqn]
                if len(sig.shape) == 1:
                    self._df[alias] = self._rdr[fqn]
                else:
                    # Terrible solution
                    s = sig.shape[1]
                    index = pd.MultiIndex.from_product([[alias], np.arange(0, s)])
                    d = pd.DataFrame(sig, columns=index)

                    d[self._defs.Columns.MTS_TS] = mts_ts
                    d.set_index(self._defs.Columns.MTS_TS, drop=False, inplace=True)
                    self._df = self._df.join(d, on=self._defs.Columns.MTS_TS)
                    self._df.drop(columns=[(self._defs.Columns.MTS_TS, "")], inplace=True)

    def close(self):
        self._rdr.close()
        self._df = None

    # def __getitem__(self, item):
    #     return self._df[item].values

    @property
    def signals(self) -> SignalDataFrame:
        # !!! copy by design !!! Although not imutable we can reuse the read data
        # with in the next component without fearing datacorruption by ill behaving
        # components
        # Rationale: Reading is more expensive than processing
        return self._df # .copy()
