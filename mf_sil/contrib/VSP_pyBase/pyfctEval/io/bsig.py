#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BSIG reader and writer.
"""
import logging
import os
import struct
import zlib
from typing import List

import numpy as np
import pandas as pd

from generic import (
    ReaderException,
    WriterException,
    IReader,
    IWriter,
    SignalNotFoundException,
)

__author__ = "Philipp Baust"
__copyright__ = "Copyright 2020, Continental AMS ADAS SYS ADS V&V TES LND"
__credits__ = ["Robert Hecker", "Sven Mertens", "Günther Rädler"]
__license__ = "Copyright (c) 2020, Continental AMS ADAS. All rights reserved."
__version__ = "0.2_hotfix1"
__maintainer__ = "Philipp Baust"
__email__ = "philipp.baust@continental-corporation.com"
__status__ = "Development"


_log = logging.getLogger(__name__)


SIG_NAME = "SignalName"
SIG_TYPE = "SignalType"
SIG_ARRAYLEN = "ArrayLength"
SIG_OFFSET = "Offsets"
SIG_SAMPLES = "SampleCount"


ARR_FRMT = {
    0x0008: "B",
    0x8008: "b",
    0x0010: "H",
    0x8010: "h",
    0x0020: "L",
    0x8020: "l",
    0x0040: "Q",
    0x8040: "q",
    0x9010: "f",
    0x9020: "d",
}


SIG_FRMT_RDR = {
    "c": 1,
    "b": 1,
    "B": 1,
    "h": 2,
    "H": 2,
    "I": 4,
    "l": 4,
    "L": 4,
    "q": 8,
    "Q": 8,
    "f": 4,
    "d": 8,
}


SIG_FRMT_WRT = {
    "b": 32776,
    "h": 32784,
    "l": 32800,
    "B": 8,
    "H": 16,
    "L": 32,
    "q": 32832,
    "Q": 64,
    "f": 36880,
    "d": 36896,
}


class BsigReader(IReader):
    """ Reader implementation for BISG file format.
    The reader has a context manager, signal access can be done in an dict like manner.

    Usage examples:

    .. code-block:: python

        with (filename) as bsig:
           ts = bsig["MTS.Package.TimeStamp"]

            # do something here



    """

    def __init__(self, filepath, **kwargs):
        """ Constructor.
        :param filepath: full path to the BSIG
        """
        # super(, self).__init__(filepath, **kwargs)
        super().__init__(**kwargs)
        self._filepath = filepath

        self.__version = None
        self.__fh = None  # the file handle
        self.__sd = {}
        self.__offset_data_type = "Q"
        self.__file_size = -1
        self.__compression = False
        self.__block_size = -1
        self.__hdr_size = -1

    def open(self):
        """ Opens the BSIG and reads the signal list.
        **Prefer using the BSIG reader as a context instead of opening / closing it explicitly**
        """
        file_header = 24
        self.__fh = open(self._filepath, "rb")

        # read global header
        if self.__read_sig("c" * 4) != (b"B", b"S", b"I", b"G"):
            raise ReaderException("Given file is not of type BSIG!")
        version = self.__read_sig("B" * 3)
        if version[0] not in (2, 3):  # we support version 2 and 3 by now
            raise ReaderException(
                "Unsupported version: %d.%d.%d, supporting only V2 & V3!" % version
            )
        self.__version = version[0]
        # changed from unsigned int to unsigned long long
        if version[0] == 2:
            self.__offset_data_type = "I"

        # get total size of file
        self.__fh.seek(0, os.SEEK_END)
        self.__file_size = self.__fh.tell()
        self.__fh.seek(-file_header, os.SEEK_CUR)

        # read file header
        signal_count, self.__block_size, self.__hdr_size, offset_size = self.__read_sig(
            "IIII"
        )
        self.__read_sig("B" * 3)  # internal version is unused, read over
        self.__compression = self.__read_sig("B")[0] == 1
        if self.__read_sig("c" * 4) != (b"B", b"I", b"N", b"\x00"):  # bin signature
            raise ReaderException("BSIG signature wrong!")

        # read signal description
        self.__fh.seek(self.__file_size - file_header - self.__hdr_size)

        # temp list to add offsets and signal length from offset data block
        signal_list = []
        for _ in range(signal_count):
            sig_name_len = self.__read_sig("H")[0]
            signal_name = (b"".join(self.__read_sig("c" * sig_name_len))).decode(
                "utf-8"
            )
            array_len, stype = self.__read_sig("II")
            signal_list.append(signal_name)
            self.__sd[signal_name] = {SIG_TYPE: stype, SIG_ARRAYLEN: array_len}

        # read offsets data
        self.__fh.seek(self.__file_size - file_header - self.__hdr_size - offset_size)
        for sig in signal_list:
            offset_count, self.__sd[sig][SIG_SAMPLES] = self.__read_sig("II")
            if offset_count:
                self.__sd[sig][SIG_OFFSET] = self.__read_sig(
                    self.__offset_data_type * offset_count
                )
            else:
                self.__sd[sig][SIG_OFFSET] = []

    def close(self):
        """ Closes the BSIG file.
        **Prefer using the BSIG reader as a context instead of opening / closing it explicitly**
        """
        if self.__fh is None:
            _log.debug("Attempting to close already closed BSIG.")
            return

        self.__fh.close()
        self.__fh = None

    def __str__(self):
        return "<bsig{}: '{}', signals: {}>".format(
            self.__version, self.__fh.name, len(self)
        )

    def __len__(self):
        """ Returns the number of signals in the currently open BSIG

        :return: number of signals in the binary file.
        """
        return len(self.__sd)

    def __getitem__(self, item) -> np.array:
        """ Return the signal data for the given signal name.

        :return Array
        """
        return self.__signal(item)

    def __signal(self, signal_name) -> np.array:
        """ Retrieves the signal from the file.

        :param signal_name: case sensitive signal name
        :return: signal data
        """
        if self.__fh is None:
            raise ReaderException("Attempt to read on closed filehandle.")

        try:
            signal_info = self.__sd[signal_name]
        except KeyError as ex:
            raise ReaderException("signal not found: %s" % signal_name) from ex

        # align offset and count, count is initially the length,
        # but we use it as stop point and offset as start point
        frmt = ARR_FRMT[signal_info[SIG_TYPE]]  # data format
        dlen = SIG_FRMT_RDR[frmt]  # length of one data point
        alen = signal_info[SIG_ARRAYLEN]  # array length of signal
        sig = []  # extracted signal

        # increment with array length
        offset = 0
        count = signal_info[SIG_SAMPLES] * alen

        # read data blocks
        for offs in signal_info[SIG_OFFSET]:
            self.__fh.seek(offs)
            if self.__compression:
                data = self.__fh.read(self.__read_sig("I")[0])
                data = zlib.decompress(data)
            else:
                data = self.__fh.read(self.__block_size)

            data = struct.unpack(frmt * (len(data) // dlen), data)
            sig.extend(data)

        if alen in [0, 1]:
            # Flat lists shape (N) or signal arrays of shape (N,1)
            return np.array(sig[offset:count], dtype=frmt)
        else:
            # Arrays of shape (N,M) with M > 1
            return np.array(sig[offset:count], dtype=frmt).reshape(
                ((count - offset) // alen, alen)
            )

    @property
    def signals(self) -> List[str]:
        """ Returns a list of signal names available in the BSIG.

        :return: all signal names in file
        """
        return self.__sd.keys()

    def __read_sig(self, stype):
        """ Reads and unpacks the specified data nibble from the BSIG

        :param stype: type definition * length
        """
        return struct.unpack(stype, self.__fh.read(SIG_FRMT_RDR[stype[0]] * len(stype)))


class Multi(IReader):
    def __init__(self, *filenames, sync_on="MTS.Package.TimeStamp"):
        super().__init__()

        self.filenames = [f for f in filenames if f is not None]
        self.sync_on = sync_on

        self._rdrs = [(f) for f in self.filenames]

        self.rdr_ts = {}
        self._signals = []

    @property
    def signals(self):
        return self._signals

    def __getitem__(self, item) -> np.array:
        """ Return the values for the item.

        The implementing class has to decide how to access the values,
        could be either a signal name (for low level readers) or an alias name, or
        even some wildcard.
        """
        if item == self.sync_on:
            return self._merged

        r = None
        for r in self._rdrs:
            if item in r.signals:
                break

        if r is None:
            raise SignalNotFoundException(
                "Signal '{}' is in none of the provided BSIGs.".format(item)
            )

        s = pd.Series(r[item], self.rdr_ts[r])
        s = s[~s.index.duplicated(keep="first")]
        d = s.reindex(self._merged, method="ffill")
        return d.values

    def open(self):
        """ Opens the data source and prepares for reading. """
        for r in self._rdrs:
            r.open()
            self.rdr_ts[r] = r[self.sync_on]
            self._signals.extend(r.signals)

        self._merged = np.sort(np.unique(np.concatenate(list(self.rdr_ts.values()))))

    def close(self):
        """ Closes the data source. """
        for r in self._rdrs:
            r.close()


class BsigWriter(IWriter):
    """ Writer implementation for BISG file format.
    The writer has a context manager, signal can be add in an dict like manner.

    Usage examples:

    .. code-block:: python

        with BsigWriter(filename) as bsig:
            bsig["MTS.Package.TimeStamp"] = np.array([0,1,2], dtype=np.uint8)

            # do something here



    """

    def __init__(self, filepath, v2_format=False, block_size=4096, compress_data=True):
        """ Constructor.

        :param filepath: file path to the BSIG
        :param v2_format: Switch to enable v2 data format
        :param block_size: signal data blocksize, has to be power of 2 and between 2 ** 8 and 2 ** 17
        :param compress_data: Switch to enable/disable compression
        """
        super().__init__()
        self._filepath = filepath
        self.__v2 = v2_format
        self.__compress_data = compress_data
        self.__block_size = block_size
        assert self.__block_size in (2 ** i for i in range(8, 17)), "block_size wrong!"

        self._signal_data = []
        self.__sd = {}

        self.__fp = None

    def open(self):
        """ Opens the target file and writes the file format header data.

        **Consider to use the BsigWriter as a context manager instead of opening and closing files explcitly**
        """
        self.__fp = open(self._filepath, "wb")
        # Global header
        self._write_sig("c", "BSIG")
        self._write_sig("B", [2 if self.__v2 else 3, 0, 0, 0])

        try:
            self.__fp.flush()
        except Exception as ex:
            raise WriterException(
                "Cannot write to '{}'. File may be used by other process".format(
                    self._filepath
                )
            ) from ex

    def close(self):
        """ Closes the BSIG file.
        """
        if self.__fp is None:
            _log.debug("Writer already closed")
            return

        # write offsets
        offset = self.__fp.tell()
        for name in self.__sd.keys():
            signal = self.__sd[name]
            self._write_sig("I", [len(signal[SIG_OFFSET]), signal[SIG_SAMPLES]])
            self._write_sig("L" if self.__v2 else "Q", signal[SIG_OFFSET])
        offset = self.__fp.tell() - offset

        # write signal desc
        header = self.__fp.tell()
        for name in self.__sd.keys():
            signal = self.__sd[name]
            self._write_sig("H", len(name))
            self._write_sig("c", name)
            self._write_sig(
                "I", [signal[SIG_ARRAYLEN], SIG_FRMT_WRT[signal[SIG_TYPE]]]
            )  # array length & type

        header = self.__fp.tell() - header

        # write file header
        self._write_sig("I", [len(self.__sd), self.__block_size, header, offset])
        c = 1 if self.__compress_data else 0
        self._write_sig(
            "B", [0, 0, 0, c]
        )  # write internal version (unused) & compression flag
        self._write_sig("c", "BIN\x00")

        self.__fp.close()
        self.__fp = None

    def __str__(self):
        return "<bsig{}: '{}', signals: {}>".format(
            2 if self.__v2 else 3, self.__fp.name, len(self.__sd)
        )

    def _write_sig(self, stype, data):
        """ Writes packed signal data of given type to file.

        :param stype: data type description
        :param data: content
        """
        if type(data) == str:
            data = [d.encode("ascii", errors="strict") for d in data]
            for d in data:
                self.__fp.write(struct.pack(stype, d))
        elif type(data) in (list, tuple):
            for d in data:
                self.__fp.write(struct.pack(stype, d))
        else:
            self.__fp.write(struct.pack(stype, data))

    @property
    def signals(self) -> List[str]:
        """ Returns all signal names which are known in a list.

        :return: Signal list
        """
        return self.__sd.keys()

    def __setitem__(self, key, value):
        """ Adds a signal

        :type key: name of signal
        :param value: signal to be added
        """
        self.__write_signal(key, value)

    def __write_signal(self, name: str, signal: np.array):
        if self.__fp is None:
            raise WriterException("Attempt to write on closed file handle.")

        signal_len = len(signal)
        array_len = signal[0].size if signal_len > 0 else 0
        if array_len > 1:
            signal = signal.flatten()

        offsets = []
        i = 0
        block_sz = self.__block_size // signal.dtype.itemsize

        if not self.__compress_data and len(signal) % block_sz != 0:
            # We need to fill to the next block length.
            padded_length = int(np.ceil(len(signal) / block_sz) * block_sz)
            padded_signal = np.zeros(padded_length, dtype=signal.dtype)
            padded_signal[0 : len(signal)] = signal
            signal = padded_signal

        while i < len(signal):
            offsets.append(self.__fp.tell())
            if self.__compress_data:
                data = zlib.compress(
                    b"".join(
                        [
                            struct.pack(signal.dtype.char, d)
                            for d in signal[i : i + block_sz]
                        ]
                    )
                )
                self._write_sig("I", len(data))
            else:
                data = b"".join(
                    [
                        struct.pack(signal.dtype.char, d)
                        for d in signal[i : i + block_sz]
                    ]
                )

            self.__fp.write(data)
            i += block_sz

        self.__sd[name] = {
            SIG_SAMPLES: signal_len,
            SIG_ARRAYLEN: array_len,
            SIG_OFFSET: offsets,
            SIG_TYPE: signal.dtype.char,
        }
