#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BSIG reader and writer.
"""
import logging

import pandas as pd


__author__ = "Philipp Baust"
__copyright__ = "Copyright 2020, Continental AMS ADAS SYS ADS V&V TES LND"
__credits__ = []
__license__ = "Copyright (c) 2020, Continental AMS ADAS. All rights reserved."
__version__ = "0.2_hotfix1"
__maintainer__ = "Philipp Baust"
__email__ = "philipp.baust@continental-corporation.com"
__status__ = "Development"


_log = logging.getLogger(__name__)


class SignalDataSeries(pd.Series):
    """ Describes column data out of a object data frame. """

    @property
    def _constructor(self):
        return SignalDataSeries

    @property
    def _constructor_expanddim(self):
        return SignalDataFrame

    @property
    def as_plain_df(self) -> pd.Series:
        """ Debug support, allow for table visualization pyCharm and other IDEs

        :return: a plain dataframe of this object dataframe.
        """
        return pd.Series(self)


class SignalDataFrame(pd.DataFrame):
    """ Data-frame representation of an signal list.

    Signals are indexed to either

        * MTS or
        * UTC timestamps

    Currently this class does not add any specialized functionality.
    """

    @property
    def as_plain_df(self) -> pd.DataFrame:
        """ Debug support, allow for table visualization pyCharm and other IDEs

        :return: a plain dataframe of this object dataframe.
        """
        return pd.DataFrame(self)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @property
    def _constructor(self):
        return SignalDataFrame

    @property
    def _constructor_sliced(self):
        return SignalDataSeries


    @property
    def signals(self) -> "SignalDataFrame":
        return self

class ObjectDataSeries(pd.Series):
    """ Describes column data out of a object data frame. """

    @property
    def _constructor(self):
        return ObjectDataSeries

    @property
    def _constructor_expanddim(self):
        return ObjectDataFrame

    @property
    def as_plain_df(self) -> pd.Series:
        """ Debug support, allow for table visualization pyCharm and other IDEs

        :return: a plain dataframe of this object dataframe.
        """
        return pd.Series(self).copy(deep=True)


class LaneDataSeries(pd.Series):
    """ Describes column data out of a object data frame. """

    @property
    def _constructor(self):
        return LaneDataSeries

    @property
    def _constructor_expanddim(self):
        return LaneDataFrame

    @property
    def as_plain_df(self) -> pd.Series:
        """ Debug support, allow for table visualization pyCharm and other IDEs

        :return: a plain dataframe of this object dataframe.
        """
        return pd.Series(self).copy(deep=True)


class ObjectDataFrame(pd.DataFrame):
    """ Data-frame representation of an object list.

    Objects are guaranteed to have the following columns:

     * a recording unique object index
     * dist_x, dist_y
     * shape points 0..4 indicating the vortex points of the objects

    Some notes on the index:
    The index is only unique in the observed time interval, reading
    two or more time intervals may reset the index.
    But reading the same stride (of the same input) twice will produce the
    same indices.
    """

    @property
    def as_plain_df(self):
        """ Debug support, allow for table visualization pyCharm and other IDEs

        :return: a plain dataframe of this object dataframe.
        """
        return pd.DataFrame(self).copy(deep=True)

    _geometry_column_name = "_geometry"

    def __init__(self, *args, **kwargs):
        geometry = kwargs.pop("geometry", None)
        super().__init__(*args, **kwargs)

        # TODO: The geometry stuff is taken from geopandas and might be usefull
        #  for objects as well.
        #  Right now the *only* purppose of an object dataframe is to allow
        #  for later extension.

    #     # # set_geometry ensures the geometry data have the proper dtype,
    #     # # but is not called if `geometry=None` ('geometry' column present
    #     # # in the data), so therefore need to ensure it here manually
    #     # # but within a try/except because currently non-geometries are
    #     # # allowed in that case
    #     # # TODO do we want to raise / return normal DataFrame in this case?
    #     # if geometry is None and "geometry" in self.columns:
    #     #     # only if we have actual geometry values -> call set_geometry
    #     #     index = self.index
    #     #     try:
    #     #         self["geometry"] = _ensure_geometry(self["geometry"].values)
    #     #     except TypeError:
    #     #         pass
    #     #     else:
    #     #         if self.index is not index:
    #     #             # With pandas < 1.0 and an empty frame (no rows), the index
    #     #             # gets reset to a default RangeIndex -> set back the original
    #     #             # index if needed
    #     #             self.index = index
    #     #         geometry = "geometry"
    #
    #     if geometry is not None:
    #         self.set_geometry(geometry, inplace=True)
    #
    # #     else:
    # #         self[self._geometry_column_name] = self._compute_polygons()
    # #
    # # def _compute_polygons(self):
    # #     # TODO: Shit, on large dfs
    # #     # TODO: Use groupby / apply!
    # #     r = []
    # #     for idx, row in self.iterrows():
    # #         if not set(["dist_x", "dist_y", "x0", "y0", "x1", "y1", "x2", "y2", "x3", "y3"]).issubset(set(self.columns)):
    # #             r.append(None)
    # #             continue
    # #
    # #         p = Polygon([
    # #             [row["dist_x"] + row["x0"],  row["dist_y"] + row["y0"], ],
    # #             [row["dist_x"] + row["x1"], row["dist_y"] + row["y1"], ],
    # #             [row["dist_x"] + row["x2"], row["dist_y"] + row["y2"], ],
    # #             [row["dist_x"] + row["x3"], row["dist_y"] + row["y3"], ],
    # #             [row["dist_x"] + row["x0"], row["dist_y"] + row["y0"], ],
    # #         ])
    # #         r.append(p)
    # #     return r
    #
    #
    #
    # def __setattr__(self, attr, val):
    #     # have to special case geometry b/c pandas tries to use as column...
    #     if attr == "geometry":
    #         object.__setattr__(self, attr, val)
    #     else:
    #         super(ObjectDataFrame, self).__setattr__(attr, val)
    #
    # def _get_geometry(self):
    #     if self._geometry_column_name not in self:
    #         raise AttributeError(
    #             "No geometry data set yet (expected in"
    #             " column '%s'." % self._geometry_column_name
    #         )
    #     return self[self._geometry_column_name]
    #
    # def _set_geometry(self, col):
    #     if not pd.api.types.is_list_like(col):
    #         raise ValueError("Must use a list-like to set the geometry property")
    #     self.set_geometry(col, inplace=True)
    #
    # geometry = property(
    #     fget=_get_geometry, fset=_set_geometry, doc="Geometry data for GeoDataFrame"
    # )

    @property
    def _constructor(self):
        return ObjectDataFrame

    @property
    def _constructor_sliced(self):
        return ObjectDataSeries

    @property
    def objects(self) -> "ObjectDataFrame":
        return self

    # @property
    # def birdseye(self):
    #     return BirdsEyePlotter(self)


class BirdsEyePlotter(object):
    """ Plotter for debugging
    """

    def __init__(self, df: ObjectDataFrame):
        self.df = df

    def __getitem__(self, item):
        # TODO

        sub_df = self.df.set_index("ts", drop=False).loc[item]

        for idx, row in sub_df.iterrows():
            print(row)

        # verts = [
        #     (0., 0.),  # left, bottom
        #     (0., 1.),  # left, top
        #     (1., 1.),  # right, top
        #     (1., 0.),  # right, bottom
        #     (0., 0.),  # ignored
        # ]
        #
        # codes = [
        #     Path.MOVETO,
        #     Path.LINETO,
        #     Path.LINETO,
        #     Path.LINETO,
        #     Path.CLOSEPOLY,
        # ]
        #
        # path = Path(verts, codes)
        #
        # fig, ax = plt.subplots()
        # patch = patches.PathPatch(path, facecolor='orange', lw=2)
        # ax.add_patch(patch)
        # ax.set_xlim(-2, 2)
        # ax.set_ylim(-2, 2)
        # plt.show()




class LaneDataFrame(pd.DataFrame):
    """ Data-frame representation of lanes.

    """

    @property
    def as_plain_df(self):
        """ Debug support, allow for table visualization pyCharm and other IDEs

        :return: a plain dataframe of this object dataframe.
        """
        return pd.DataFrame(self).copy(deep=True)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @property
    def _constructor(self):
        return LaneDataFrame

    @property
    def _constructor_sliced(self):
        return LaneDataSeries

