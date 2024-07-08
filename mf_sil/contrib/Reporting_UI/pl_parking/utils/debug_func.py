import logging
import tempfile
from pathlib import Path

from tsf.core.utilities import debug

_log = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

import os
import sys

# import test case to be debugged - example
from pl_parking.PLP.CEM.PFS.SWKPI_CNC_PFS_ParkingMarkerAccuracy import FtPCLAccuracy


def tc_debug_func(test_case, data_folder: Path, temp_dir: Path = None, open_explorer=True):
    """Optional, call to debug to set up debugging in the simplest possible way.
    When calling the test case you need to provide a valid input to
    execute the test (e.g. a BSIG file) and report the result.
    This is only meant to jump start testcase debugging.
    """

    test_bsigs = []  # absolute path to measurements used for debug
    os.makedirs(data_folder, exist_ok=True)

    debug(
        test_case,
        *test_bsigs,
        temp_dir=temp_dir,
        open_explorer=open_explorer,
        kpi_report=False,
        dev_report=True,
    )
    _log.debug("All done.")


if __name__ == "__main__":
    working_directory = Path(tempfile.mkdtemp("_tsf"))

    data_folder = working_directory / "data"
    out_folder = working_directory / "out"

    # Input imported test case
    tc_debug_func(FtPCLAccuracy, data_folder=data_folder, temp_dir=out_folder, open_explorer=True)
