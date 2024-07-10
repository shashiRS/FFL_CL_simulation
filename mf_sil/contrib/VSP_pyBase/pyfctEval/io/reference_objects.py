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
from enum import Enum

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


class RtRangeObjectDefinition(GenericObjectDefinition):
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

        TARGET = "target"

        # REL_DIST_X = "rel_dist_x"
        # REL_DIST_Y = "rel_dist_y"
        # REL_VEL_X = "rel_vel_x"
        VREL_Y = "rel_vel_y"
        LONG_RANGE = "long_range"
        LONG_TTC = "long_ttc"
        LONG_TTC_A = "long_ttc_a"
        LAT_RANGE = "lat_range"
        LAT_TTC = "lat_ttc"
        LAT_TTC_A = "lat_ttc_a"

    class EgoColumns(object):

        MTS_TS = "mts_ts"

        #: Optional UTC Timestamp (if tstp fiel is provided)
        UTC_TS = "utc_ts"

        VELOCITY = "ego_velocity"
        ACCELERATION = "ego_acceleration"
        YAWRATE = "ego_yawrate"


    class Dimensions(Enum):
        GVT = (
            (0, 1.712 / 2,),
            (4.023, 1.712 / 2,),
            (4.023, -1.712 / 2,),
            (0, -1.712 / 2,),
        )

        VRU_PED = (
            (-0.25, -0.25,),
            (0.25, -0.25,),
            (0.25, 0.25,),
            (-0.25, 0.25,),
        )

        VRU_CHILD = (
            (-0.25, -0.25,),
            (0.25, -0.25,),
            (0.25, 0.25,),
            (-0.25, 0.25,),
        )

        VRU_BICYLE_LONG = (
            (-0.25, -0.25,),
            (0.25, -0.25,),
            (0.25, 0.25,),
            (-0.25, 0.25,),
        )

        VRU_BICYLE_CROSS = (
            (-0.25, -0.25,),
            (0.25, -0.25,),
            (0.25, 0.25,),
            (-0.25, 0.25,),
        )


    def __init__(self):
        super().__init__()

        # Signal root
        self._root = "RT-Range Processor"

        # Map signal column name to object property signal names.
        # Note : These signals are defined starting after the Target0X part
        # of signal names
        self._properties = [
            # Mappings for the **mandatory** defaults
            (self.Columns.DIST_X, r".RelativeToHunter.RelativeDistanceX"),
            (self.Columns.DIST_Y, r".RelativeToHunter.RelativeDistanceY"),
            (self.Columns.VREL_X , r".RelativeToHunter.RelativeVelocityX_mps"),
            (self.Columns.VREL_Y , r".RelativeToHunter.RelativeVelocityY_mps"),
            (self.Columns.LONG_RANGE, r".Range.LongitudinalRange"),
            (self.Columns.LONG_TTC ,  r".Range.LongitudinalTTC"),
            (self.Columns.LONG_TTC_A, r".Range.LongitudinalTTC_A_corrected"),
            (self.Columns.LAT_RANGE , r".Range.LateralRange"),
            (self.Columns.LAT_TTC  ,  r".Range.LateralTTC"),
            (self.Columns.LAT_TTC_A , r".Range.LateralTTC_A_corrected"),
        ]

        self._ego_properties = [
            (self.EgoColumns.VELOCITY , r".Hunter.VelocityVehicle.LongitudinalVelocity_mps"),
            (self.EgoColumns.ACCELERATION, r".Hunter.AccelLevel.AccelerationForward"),
            (self.EgoColumns.YAWRATE, r".Hunter.AngularRateLevel.YawRate"),
        ]



class RtRangeObjectsReader(IObjectReader, IReader):
    """ Reader for sensorics object list (e.g ARSxxx, MFCxxx).

    It is assumed that the signal maintenance state ist provided which
    tracks the object lifetime (This assumption holds true for all newer
    generation sensorics algorithms).
    """

    MTS_TS_SIGNAL = "MTS.Package.TimeStamp"
    UTC_TS_SIGNAL = "MTS.Package.UtcTimeStamp"

    def __init__(
        self, filename, ref_object_definition=RtRangeObjectDefinition(), tstp_filename=None
    ):
        """ Constructor.

        :param filename: full path to the files that should be read
          (e.g. bsig, h5, ...)
        :param ref_object_definition: Object definition object which maps what
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
        self._defs = ref_object_definition
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
        # Read object data one by one
        mts_ts = self._rdr[self.MTS_TS_SIGNAL].astype(np.int64)

        number_of_targets = self._rdr[self._defs.object_root + ".Extended RT-Range data.Target.TotalNumberOfTargets"][-1]
        target = self._rdr[self._defs.object_root + ".Extended RT-Range data.Target.TargetNumber"][-1]

        tracks = []
        for k in range(number_of_targets):
            _log.debug("Reading object data for target {}.".format(k))
            track_data = pd.DataFrame({
                self._defs.Columns.MTS_TS: mts_ts,
                self._defs.Columns.TARGET: k + 1,  # Avoid TSF IDx == 0
            })
            for alias, signal_name_suffix in self._defs.object_properties:
                # if signal_name_suffix.startswith("."):
                fqn = "{0:}.Target{1:02d}{2:}".format(
                    self._defs.object_root, k + target, signal_name_suffix
                )
                # else:
                #     fqn = signal_name_suffix.replace("%", str(k))
                track_data[alias] = self._rdr[fqn]

            # Drop all defects
            track_data = track_data.loc[
                ~(track_data.sum(axis=1) - track_data[self._defs.Columns.MTS_TS] == 0)
            ]
            tracks.append(track_data)

        df = pd.concat(tracks, axis=0)
        df[self._defs.Columns.TSF_INDEX] = df[self._defs.Columns.TARGET]

        df.set_index(self._defs.Columns.MTS_TS, drop=False, inplace=True)

        for alias, signal_name_suffix in self._defs._ego_properties:
            fqn = "{0:}{1:}".format(
                self._defs.object_root, signal_name_suffix
            )
            s = pd.Series(self._rdr[fqn], index=mts_ts)

            df[alias] = s

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
        return self._odf.copy()

    def close(self):
        self._rdr.close()

    def __getitem__(self, item) -> np.array:
        """ Expose the signal reader as well.
        :return Array
        """
        return self._rdr[item]
