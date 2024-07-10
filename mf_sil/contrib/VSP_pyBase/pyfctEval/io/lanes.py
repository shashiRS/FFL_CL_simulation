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
from typing import Union

import numpy as np
import pandas as pd

from io.bsig import , Multi
from io.datamodel import LaneDataFrame
from io.generic import IReader, ILanesReader

__author__ = "Philipp Baust"
__copyright__ = "Copyright 2020, Continental AMS ADAS SYS ADS V&V TES LND"
__credits__ = []
__license__ = "Copyright (c) 2020, Continental AMS ADAS. All rights reserved."
__version__ = "0.2_hotfix1"
__maintainer__ = "Philipp Baust"
__email__ = "philipp.baust@continental-corporation.com"
__status__ = "Development"

from misc._utilities import log_runtime

_log = logging.getLogger(__name__)



class RmfLaneDefinition(object):
    """ Specialized object definition for all sensorics objects.
    All sensorics object readers additionally require the following
    signals for processing:

      * MTS Timestamp as timeline
      * Maintenance state for object admin state

    Additionally it makes sense to also export the UTC timestamp.
    """

    class Columns(object):
        #: MTS Timestamp
        MTS_TS = "mts_ts"


    def __init__(self):
        super().__init__()

        # Signal self.__root
        self.__root = "SIM VFB ALL.RMF.RMFRoadModel"

        self._properties = [
        ]


class LanesReader(ILanesReader, IReader):
    """ Reader for sensorics object list (e.g ARSxxx, MFCxxx).

    It is assumed that the signal maintenance state ist provided which
    tracks the object lifetime (This assumption holds true for all newer
    generation sensorics algorithms).
    """

    MTS_TS_SIGNAL = "MTS.Package.TimeStamp"
    UTC_TS_SIGNAL = "MTS.Package.UtcTimeStamp"

    def __init__(
        self, filename, lane_definition=RmfLaneDefinition(), tstp_filename=None
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

        self._defs = lane_definition
        self._tstp_filepath = tstp_filename
        self._df = LaneDataFrame()
        
        self.__root = "SIM VFB ALL.RMF.RMFRoadModel."
        self.__signal_length = -1

    @property
    def lanes(self):
        return self._df

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    @log_runtime
    def __read_global_info(self):
        # Read object data one by one
        self.__mts_ts = self._rdr[self.MTS_TS_SIGNAL].astype(np.int64)
        self.__signal_length = self.__mts_ts.shape[0]
        ego_line_idx = self._rdr[self.__root + "aLanesHypotheses.egoLaneIndex"]
        no_Lanes = self._rdr[self.__root + "aLanesHypotheses.numLanes"]

        mi = pd.MultiIndex.from_product([["gbl"],
                                         ["mts_ts", "ego_idx", "no_lanes"]])

        df = pd.DataFrame(np.array([self.__mts_ts, ego_line_idx, no_Lanes, ]).T, columns=mi)
        df.astype({("gbl", "mts_ts"): np.int64, ("gbl", "ego_idx"): np.uint8,
                   ("gbl", "no_lanes"): np.uint8})

        return df

    @log_runtime
    def __read_lane_hypothesis(self):
        s = {
            "leftLane": self.__root + "aLanesHypotheses.lanes[{}].leftLane",
            "rightLane": self.__root + "aLanesHypotheses.lanes[{}].rightLane",
            "numLeftBoundaryParts": self.__root + "aLanesHypotheses.lanes[{}].numLeftBoundaryParts",
            "numRightBoundaryParts": self.__root + "aLanesHypotheses.lanes[{}].numRightBoundaryParts",
            "leftBoundaryParts": self.__root + "aLanesHypotheses.lanes[{}].leftBoundaryParts",
            "rightBoundaryParts": self.__root + "aLanesHypotheses.lanes[{}].rightBoundaryParts",
        }
        lane_signals = list(s.keys())

        # lane boundary parts is an array, but I do not want to add another level
        cols = (
                lane_signals[0:4]
                + ["leftBoundaryParts_{}".format(k) for k in range(4)]
                + ["rightBoundaryParts_{}".format(k) for k in range(4)]
        )
        no_lane_hypothesis = 5
        hypothesis_hypercube = np.zeros((
            self.__signal_length,
            len(cols) * no_lane_hypothesis
        ))

        c = 0
        for lane_idx in range(no_lane_hypothesis):
            for k, key in enumerate(lane_signals):
                # c = lane_idx * len(s) + k
                d = self._rdr[s[key].format(lane_idx)]
                if d.ndim == 1:
                    hypothesis_hypercube[:, c] = d
                    c += 1
                else:
                    hypothesis_hypercube[:, c:c + d.shape[1]] = d
                    c += d.shape[1]

        mi = pd.MultiIndex.from_product(
            [range(no_lane_hypothesis), cols]
        )
        hypothesis = pd.DataFrame(hypothesis_hypercube,
                                  # index=mts_ts,
                                  columns=mi)

        return hypothesis

    @log_runtime
    def __read_linear_objects(self):
        # linear objects
        s = {
            "line": self.__root + "aLinearObjects[{}].geometry.line",
            "from": self.__root + "aLinearObjects[{}].geometry.from",
            "to": self.__root + "aLinearObjects[{}].geometry.to",
            "marking": self.__root + "aLinearObjects[{}].marking",
            "type": self.__root + "aLinearObjects[{}].type",
            "color": self.__root + "aLinearObjects[{}].color",
            "confidence": self.__root + "aLinearObjects[{}].confidence",
        }
        linear_object_signals = list(s.keys())
        no_linear_objects = 20

        linear_objs_hypercube = np.zeros((self.__signal_length, len(s) * no_linear_objects))
        for lobj_idx in range(no_linear_objects):
            for k, key in enumerate(linear_object_signals):
                c = lobj_idx * len(s) + k
                linear_objs_hypercube[:, c] = self._rdr[s[key].format(lobj_idx)]

        mi = pd.MultiIndex.from_product(
            [range(no_linear_objects), linear_object_signals]
        )
        linear_objects = pd.DataFrame(
            linear_objs_hypercube,
            # index=mts_ts,
            columns=mi
        )
        return linear_objects

    @log_runtime
    def __read_line(self, line_idx):
        pt_count = self._rdr[
            "SIM VFB ALL.RMF.RMFRoadModel.sGeometries.lines[{}].numPoints".format(
                line_idx)]
        max_point_count = np.max(pt_count)

        if max_point_count == 0:
            _log.debug("No lines for {}".format(line_idx))

        _log.debug("Line Idx {}: Max Point count {}".format(line_idx, max_point_count))
        lines_hypercube = np.zeros((self.__signal_length * max_point_count, 6))

        for point_idx in range(max_point_count):
            x = self._rdr[
                self.__root + "sGeometries.lines[{}].points[{}].fX".format(line_idx,
                                                                           point_idx)]
            y = self._rdr[
                self.__root + "sGeometries.lines[{}].points[{}].fY".format(line_idx,
                                                                           point_idx)]

            cs = self.__signal_length * point_idx
            ce = self.__signal_length * point_idx + self.__signal_length

            lines_hypercube[cs:ce, 0] = self.__mts_ts
            lines_hypercube[cs:ce, 1] = np.ones(self.__mts_ts.shape) * point_idx
            lines_hypercube[cs:ce, 2] = pt_count
            lines_hypercube[cs:ce, 3] = x
            lines_hypercube[cs:ce, 4] = y
            lines_hypercube[cs:ce, 5] = np.ones(self.__mts_ts.shape) * line_idx

        lines_df = pd.DataFrame(lines_hypercube,
                                columns=["mts_ts", "idx", "l", "x", "y", "line_idx"])
        lines_df = lines_df.astype(
            {"mts_ts": np.int64, "idx": np.uint8, "l": np.uint8, "line_idx": np.uint8})

        lines_df.set_index(["mts_ts", "idx"], drop=False, inplace=True)
        lines_df.sort_index(inplace=True)
        # print(len(lines_df))
        lines_df = lines_df.loc[~lines_df.index.duplicated(keep='first')]
        # print(len(lines_df))
        lines_df = lines_df.loc[(lines_df["l"] != 0)]

        # print(len(lines_df))

        # def _reduce(subdf):
        #     l = int(subdf.iloc[0]["l"])
        #     return subdf.iloc[0:l]
        #
        # out = lines_df.groupby(level=[0, ]).apply(_reduce)


        out = lines_df.loc[lines_df["idx"] < lines_df["l"]]

        if not out.empty:
            out.set_index(["mts_ts", "line_idx", "idx"], drop=False, inplace=True)

        return out

    @log_runtime
    def __read_lines(self):
        # Lines
        no_lines = 17
        no_points = 150

        lines = []
        for line_idx in range(no_lines):
            out = self.__read_line(line_idx)
            if not out.empty:
                lines.append(out)

        if len(lines) == 0:
            return pd.DataFrame() # empty

        all_lines = pd.concat(lines, axis=0)
        all_lines.sort_index(inplace=True)
        all_lines["lane"] = -128
        all_lines["left"] = False
        all_lines["right"] = False
        all_lines.astype({
            "lane": np.int8,
            "left": np.bool,
            "right": np.bool,
        })

        return all_lines


    @log_runtime
    def __filter_lane(self, df, lane_idx, line_idx, boundary : Union["l", "r"]):
        """
        returns a df where the given line is part of the given lane whilst the lane is valid.

        lane hyp: 0..4
        line 0 ..16

        :param df:
        :param lane_idx:
        :param line_idx:
        :return:
        """

        if boundary == "l":
            bp_0 = "leftBoundaryParts_0"
            bp_1 = "leftBoundaryParts_1"
            bp_2 = "leftBoundaryParts_2"
            bp_3 = "leftBoundaryParts_3"
        else:
            bp_0 = "rightBoundaryParts_0"
            bp_1 = "rightBoundaryParts_1"
            bp_2 = "rightBoundaryParts_2"
            bp_3 = "rightBoundaryParts_3"

        # el = df.loc[
        el = (
                (df[(lane_idx, "lane_idx")] != -128)
                & (

                        ((
                                 (df[(lane_idx, bp_0)] == 0)
                                 | (df[(lane_idx, bp_1)] == 0)
                                 | (df[(lane_idx, bp_2)] == 0)
                                 | (df[(lane_idx, bp_3)] == 0)
                         ) & (df[(0, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 1)
                                   | (df[(lane_idx, bp_1)] == 1)
                                   | (df[(lane_idx, bp_2)] == 1)
                                   | (df[(lane_idx, bp_3)] == 1)
                           ) & (df[(1, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 2)
                                   | (df[(lane_idx, bp_1)] == 2)
                                   | (df[(lane_idx, bp_2)] == 2)
                                   | (df[(lane_idx, bp_3)] == 2)
                           ) & (df[(2, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 3)
                                   | (df[(lane_idx, bp_1)] == 3)
                                   | (df[(lane_idx, bp_2)] == 3)
                                   | (df[(lane_idx, bp_3)] == 3)
                           ) & (df[(3, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 4)
                                   | (df[(lane_idx, bp_1)] == 4)
                                   | (df[(lane_idx, bp_2)] == 4)
                                   | (df[(lane_idx, bp_3)] == 4)
                           ) & (df[(4, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 5)
                                   | (df[(lane_idx, bp_1)] == 5)
                                   | (df[(lane_idx, bp_2)] == 5)
                                   | (df[(lane_idx, bp_3)] == 5)
                           ) & (df[(5, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 6)
                                   | (df[(lane_idx, bp_1)] == 6)
                                   | (df[(lane_idx, bp_2)] == 6)
                                   | (df[(lane_idx, bp_3)] == 6)
                           ) & (df[(6, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 7)
                                   | (df[(lane_idx, bp_1)] == 7)
                                   | (df[(lane_idx, bp_2)] == 7)
                                   | (df[(lane_idx, bp_3)] == 7)
                           ) & (df[(7, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 8)
                                   | (df[(lane_idx, bp_1)] == 8)
                                   | (df[(lane_idx, bp_2)] == 8)
                                   | (df[(lane_idx, bp_3)] == 8)
                           ) & (df[(8, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 9)
                                   | (df[(lane_idx, bp_1)] == 9)
                                   | (df[(lane_idx, bp_2)] == 9)
                                   | (df[(lane_idx, bp_3)] == 9)
                           ) & (df[(9, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 10)
                                   | (df[(lane_idx, bp_1)] == 10)
                                   | (df[(lane_idx, bp_2)] == 10)
                                   | (df[(lane_idx, bp_3)] == 10)
                           ) & (df[(10, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 11)
                                   | (df[(lane_idx, bp_1)] == 11)
                                   | (df[(lane_idx, bp_2)] == 11)
                                   | (df[(lane_idx, bp_3)] == 11)
                           ) & (df[(11, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 12)
                                   | (df[(lane_idx, bp_1)] == 12)
                                   | (df[(lane_idx, bp_2)] == 12)
                                   | (df[(lane_idx, bp_3)] == 12)
                           ) & (df[(12, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 13)
                                   | (df[(lane_idx, bp_1)] == 13)
                                   | (df[(lane_idx, bp_2)] == 13)
                                   | (df[(lane_idx, bp_3)] == 13)
                           ) & (df[(13, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 14)
                                   | (df[(lane_idx, bp_1)] == 14)
                                   | (df[(lane_idx, bp_2)] == 14)
                                   | (df[(lane_idx, bp_3)] == 14)
                           ) & (df[(14, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 15)
                                   | (df[(lane_idx, bp_1)] == 15)
                                   | (df[(lane_idx, bp_2)] == 15)
                                   | (df[(lane_idx, bp_3)] == 15)
                           ) & (df[(15, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 16)
                                   | (df[(lane_idx, bp_1)] == 16)
                                   | (df[(lane_idx, bp_2)] == 16)
                                   | (df[(lane_idx, bp_3)] == 16)
                           ) & (df[(16, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 17)
                                   | (df[(lane_idx, bp_1)] == 17)
                                   | (df[(lane_idx, bp_2)] == 17)
                                   | (df[(lane_idx, bp_3)] == 17)
                           ) & (df[(17, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 18)
                                   | (df[(lane_idx, bp_1)] == 18)
                                   | (df[(lane_idx, bp_2)] == 18)
                                   | (df[(lane_idx, bp_3)] == 18)
                           ) & (df[(18, "line")] == line_idx))
                        | ((
                                   (df[(lane_idx, bp_0)] == 19)
                                   | (df[(lane_idx, bp_1)] == 19)
                                   | (df[(lane_idx, bp_2)] == 19)
                                   | (df[(lane_idx, bp_3)] == 19)
                           ) & (df[(19, "line")] == line_idx))
                )
        )

        return el

    def open(self):
        self._rdr.open()

        df = self.__read_global_info()
        hypothesis = self.__read_lane_hypothesis()
        linear_objects = self.__read_linear_objects()

        # Merge the control data frame
        df = df.join(hypothesis.astype(np.uint8))
        df = df.join(linear_objects.astype(np.uint))
        df.set_index(("gbl", "mts_ts"), inplace=True, drop=False)
        df = df.loc[~df.index.duplicated(keep="first")]
        df = df.loc[df[("gbl", "no_lanes")] > 0]

        # figure out ego lane idx to lane hyp
        # start with ego
        df[(0,"lane_idx")] = -128
        df[(1, "lane_idx")] = -128
        df[(2, "lane_idx")] = -128
        df[(3, "lane_idx")] = -128
        df[(4, "lane_idx")] = -128
        df.astype({
            (0,"lane_idx"): np.int8,
            (1, "lane_idx"): np.int8,
            (2, "lane_idx"): np.int8,
            (3, "lane_idx"): np.int8,
            (4, "lane_idx"): np.int8,
                   })

        for lane_idx in range(5):
            df.loc[
                (df[("gbl", "ego_idx")]  == lane_idx)
                & (df[("gbl", "no_lanes")] > 0),
                (lane_idx, "lane_idx")
            ] = 0  # Ego lane is 0!

        # df.loc[(df[("gbl", "ego_idx")] == 1 ) & (df[("gbl", "no_lanes")] > 0), (1, "lane_idx")] = 0  # Ego lane is 0!
        # df.loc[(df[("gbl", "ego_idx")] == 2 ) & (df[("gbl", "no_lanes")] > 0), (2, "lane_idx")] = 0  # Ego lane is 0!
        # df.loc[(df[("gbl", "ego_idx")] == 3 ) & (df[("gbl", "no_lanes")] > 0), (3, "lane_idx")] = 0  # Ego lane is 0!
        # df.loc[(df[("gbl", "ego_idx")] == 4 ) & (df[("gbl", "no_lanes")] > 0), (4, "lane_idx")] = 0  # Ego lane is 0!

        # first left / right
        for lane_idx_1 in range(5):
            for lane_idx_2 in range(5):
                if lane_idx_1 == lane_idx_2:
                    continue

                df.loc[
                    (df[(lane_idx_1, "lane_idx")] == 0)
                    & (df[(lane_idx_1, "leftLane")] == lane_idx_2),
                    (lane_idx_2, "lane_idx")
                ] = 1

                df.loc[
                    (df[(lane_idx_1, "lane_idx")] == 0)
                    & (df[(lane_idx_1, "rightLane")] == lane_idx_2),
                    (lane_idx_2, "lane_idx")
                ] = -1

        # second left / right
        for lane_idx_1 in range(5):
            for lane_idx_2 in range(5):
                if lane_idx_1 == lane_idx_2:
                    continue

                df.loc[
                    (df[(lane_idx_1, "lane_idx")] == 1)
                    & (df[(lane_idx_1, "leftLane")] == lane_idx_2),
                    (lane_idx_2, "lane_idx")
                ] = 2

                df.loc[
                    (df[(lane_idx_1, "lane_idx")] == -1)
                    & (df[(lane_idx_1, "rightLane")] == lane_idx_2),
                    (lane_idx_2, "lane_idx")
                ] = -2

        # Prepare for final assignement
        for line in range(17):
            df[("lane_map", line)] = -128
            df[("is_left", line)] = False
            df[("is_right", line)] = False
            df.astype({
                ("lane_map", line): np.int8,
                ("is_left", line): np.bool,
                ("is_right", line): np.bool,
            })

        # map lane to line
        lane_idx = 3
        line_idx = 11

        for lane_idx in range(5):
            for line_idx in range(17):
                flt = self.__filter_lane(df, lane_idx, line_idx, "l")
                df.loc[flt, ("lane_map", line_idx)] = df.loc[flt, (lane_idx, "lane_idx")]
                df.loc[flt, ("is_left", line_idx)] = True

                flt = self.__filter_lane(df, lane_idx, line_idx, "r")
                df.loc[flt, ("lane_map", line_idx)] = df.loc[flt, (lane_idx, "lane_idx")]
                df.loc[flt, ("is_right", line_idx)] = True

        # attach to all_lines


        # Process
        all_lines = self.__read_lines()
        if all_lines.empty:
            _log.info("Input does not contain any lanes.")
            return

        all_lines.set_index("mts_ts", drop=False, inplace=True)
        for line in range(17):
            all_lines["line_lane_{}".format(line)] = df[("lane_map", line)]
            all_lines["is_left_{}".format(line)] = df[("is_left", line)]
            all_lines["is_right_{}".format(line)] = df[("is_right", line)]

        for line in range(17):
            li = all_lines.loc[(all_lines["line_idx"] == line), "line_lane_{}".format(line)]
            all_lines.loc[
                (all_lines["line_idx"] == line)
                , "lane"
            ] = li

            il = all_lines.loc[(all_lines["line_idx"] == line), "is_left_{}".format(line)]
            all_lines.loc[
                (all_lines["line_idx"] == line)
                , "left"
            ] = il

            ir = all_lines.loc[(all_lines["line_idx"] == line), "is_right_{}".format(line)]
            all_lines.loc[
                (all_lines["line_idx"] == line)
                , "right"
            ] = ir

        all_lines["number_of_lanes"] = df[("gbl", "no_lanes")]
        all_lines = all_lines.loc[~(all_lines["lane"] == -128)]

        # now drop the superflous columns
        c = ["line_lane_{}".format(line) for line in range(17)]
        c.extend(["is_left_{}".format(line) for line in range(17)])
        c.extend(["is_right_{}".format(line) for line in range(17)])
        all_lines.drop(columns=c, inplace=True)

        self._df = LaneDataFrame(all_lines)


    def close(self):
        self._rdr.close()

    def __getitem__(self, item) -> np.array:
        """ Expose the signal reader as well.
        :return Array
        """
        return self._df[item]
