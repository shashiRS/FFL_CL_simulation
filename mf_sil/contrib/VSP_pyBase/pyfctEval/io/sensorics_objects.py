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
from io.datamodel import ObjectDataFrame
from io.generic import IObjectReader, IReader, GenericObjectDefinition
from misc.time import TimelineUtils


__author__ = "Philipp Baust"
__copyright__ = "Copyright 2020, Continental AMS ADAS SYS ADS V&V TES LND"
__credits__ = []
__license__ = "Copyright (c) 2020, Continental AMS ADAS. All rights reserved."
__version__ = "0.2_hotfix1"
__maintainer__ = "Philipp Baust"
__email__ = "philipp.baust@continental-corporation.com"
__status__ = "Development"


_log = logging.getLogger(__name__)
_log.setLevel(logging.INFO)


class SensoricsObjectDefinition(GenericObjectDefinition):
    """ Specialized object definition for all sensorics objects.
    All sensorics object readers additionally require the following
    signals for processing:

      * MTS Timestamp as timeline
      * Maintenance state for object admin state

    Additionally it makes sense to also export the UTC timestamp.
    """

    class Columns(GenericObjectDefinition.Columns):
        #: MTS Timestamp
        MTS_TS = "mts_ts"

        #: Optional UTC Timestamp (if tstp fiel is provided)
        UTC_TS = "utc_ts"

        #: Object track number
        TRACK = "track"

        #: Objects Admin state
        MAINTENANCE_STATE = "maintenance_state"

    def __init__(self):
        super().__init__()


class SensoricObjectsReader(IObjectReader, IReader):
    """ Reader for sensorics object list (e.g ARSxxx, MFCxxx).

    It is assumed that the signal maintenance state ist provided which
    tracks the object lifetime (This assumption holds true for all newer
    generation sensorics algorithms).
    """

    MTS_TS_SIGNAL = "MTS.Package.TimeStamp"
    UTC_TS_SIGNAL = "MTS.Package.UtcTimeStamp"

    def __init__(
        self, filename, object_definition: SensoricsObjectDefinition, tstp_filename=None
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
        self._defs = object_definition
        # self.object_list_root = object_definition.object_root
        # self.object_signal_map = object_definition.object_properties
        # self.list_size = object_definition.list_size

        self._tstp_filepath = tstp_filename
        self._odf = ObjectDataFrame()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def open(self):
        self._rdr.open()

        # Start with idx == 1 intentionally to avoid 0 since it can be easily
        # confused with NaN / None
        last_zoo_index = 1

        # Read object data one by one
        mts_ts = self._rdr[self.MTS_TS_SIGNAL].astype(np.int64)

        tracks = []
        for k in range(self._defs.list_size):
            _log.debug("Reading object data for track {}.".format(k))
            track_data = pd.DataFrame(
                {self._defs.Columns.MTS_TS: mts_ts, self._defs.Columns.TRACK: k,}
            )
            for alias, signal_name_suffix in self._defs.object_properties:
                if signal_name_suffix.startswith("."):
                    fqn = "{}[{}]{}".format(
                        self._defs.object_root, k, signal_name_suffix
                    )
                else:
                    fqn = signal_name_suffix.replace("%", str(k))
                track_data[alias] = self._rdr[fqn]

            # # TODO Intermediate hack to allow for TPF object indexing
            # fqn = "{}[{}]{}".format(
            #     self._defs.object_root, k, ".general.uiID"
            # )
            # track_data["__uiid"] = self._rdr[fqn]
            # track_data["__uiid"] = track_data["__uiid"].astype(dtype=np.int16)

            # The enum definition for eMaintenance state, taken from the OD
            # globals. Although the RTE defines two more states (4 and 5) they
            # are not used.
            #
            # /*! Indicate the administration state of object @min: 0 @max:5
            # @values: enum {
            # ODOBJ_MT_STATE_PREDICTED=3,
            # ODOBJ_MT_STATE_MEASURED=2,
            # ODOBJ_MT_STATE_NEW=1,ODOBJ_MT_STATE_DELETED=0
            # } */
            # typedef uint8 ODObj_t_MaintenanceState;

            maintenance_state = self._defs.Columns.MAINTENANCE_STATE

            tsf_indices = (
                 (track_data[maintenance_state].astype(np.int16).diff() < 0 )
                 & (track_data[maintenance_state] > 1)
                 & (track_data[maintenance_state] < 4)
            )


            # tsf_indices = track_data[maintenance_state] == 1
            tsf_indices[
                np.logical_or(
                    track_data[maintenance_state] < 1, track_data[maintenance_state] > 3
                )
            ] = np.nan

            if tsf_indices.isna().all():
                _log.debug("Empty track")
                continue

            tsf_indices = tsf_indices.cumsum() + last_zoo_index
            last_zoo_index = tsf_indices.max()

            track_data[self._defs.Columns.TSF_INDEX] = tsf_indices
            track_data.dropna(inplace=True)

            tracks.append(track_data)

        df = pd.concat(tracks, axis=0)

        if self._tstp_filepath:
            # Optional if needed load the UTC TS from the tstp file, this is
            # needed in case you want to associate objects with reference
            # environments (e.g. IBEO, LEICA, ...)

            with (self._tstp_filepath) as r:
                mts_ts_tstp = r[self.MTS_TS_SIGNAL].astype(np.int64)
                utc_ts_tstp = r[self.UTC_TS_SIGNAL].astype(np.int64)

                s = pd.DataFrame(
                    data={
                        self._defs.Columns.MTS_TS: mts_ts_tstp,
                        self._defs.Columns.UTC_TS: utc_ts_tstp,
                    }
                )

                tu = TimelineUtils()
                r = tu.reassign(s, mts_ts, on_column=self._defs.Columns.MTS_TS)
                r.set_index(self._defs.Columns.MTS_TS, drop=True, inplace=True)
                df.set_index(self._defs.Columns.MTS_TS, drop=False, inplace=True)
                df[self._defs.Columns.UTC_TS] = r[self._defs.Columns.UTC_TS]

            self._odf = ObjectDataFrame(
                df.set_index(
                    [self._defs.Columns.UTC_TS, self._defs.Columns.TSF_INDEX],
                    drop=False,
                ).sort_index()
            )

        else:
            self._odf = ObjectDataFrame(
                df.set_index(
                    [self._defs.Columns.MTS_TS, self._defs.Columns.TSF_INDEX],
                    drop=False,
                ).sort_index()
            )

    @property
    def objects(self) -> ObjectDataFrame:
        # !!! copy by design !!! Although not imutable we can reuse the read data
        # with in the next component without fearing datacorruption by ill behaving
        # components
        # Rationale: Reading is more expensive than processing
        return self._odf # .copy()

    def close(self):
        self._rdr.close()

    def __getitem__(self, item) -> np.array:
        """ Expose the signal reader as well.
        :return Array
        """
        return self._rdr[item]
