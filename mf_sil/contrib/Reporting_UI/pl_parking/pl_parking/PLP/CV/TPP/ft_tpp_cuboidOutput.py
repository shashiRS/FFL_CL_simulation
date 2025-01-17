"""TestCases performing all necessary checks for cuboid output."""

import logging
import os

import pandas as pd
import plotly.graph_objects as go

_log = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

from tsf.core.results import FALSE, TRUE, BooleanResult
from tsf.core.testcase import (
    TestCase,
    TestStep,
    register_inputs,
    register_signals,
    testcase_definition,
    teststep_definition,
    verifies,
)

import pl_parking.common_constants as fc
import pl_parking.common_ft_helper as fh
import pl_parking.PLP.CV.TPP.constants as ct
import pl_parking.PLP.CV.TPP.ft_helper as fh_tpp
from pl_parking.common_ft_helper import rep

FT_TPP = "FT_TPP_CUBOID_OUTPUT"

# TODO: Check the required names for teststep definition


@teststep_definition(
    step_number=1,
    name="TPP_ValidateCuboidSizesOutput",
    description="This test will verify the sizes for each DynamicObject_t with cuboid class",
    expected_result=BooleanResult(TRUE),
)
@register_signals(FT_TPP, fh_tpp.TPPSignals)
class TestCuboidSizesOutputTestStep(TestStep):
    """Test step definition."""

    custom_report = fh_tpp.TPPCustomTeststepReport

    def __init__(self):
        """Initialize the teststep."""
        super().__init__()

    def process(self, **kwargs):
        """Process the test result."""
        _log.debug("Starting processing...")
        reader = self.readers[FT_TPP]
        self.result.details.update(
            {"Plots": [], "Plot_titles": [], "Remarks": [], "file_name": os.path.basename(self.artifacts[0].file_path)}
        )
        test_result = fc.INPUT_MISSING  # Result
        plot_titles, plots, remarks = rep([], 3)

        invalid_frames_list = []  # list of frames with errors

        # List signals to be read
        object_members = [
            fh_tpp.TPPSignals.Columns.CLASSTYPE,
            fh_tpp.TPPSignals.Columns.LENGTH,
            fh_tpp.TPPSignals.Columns.WIDTH,
            fh_tpp.TPPSignals.Columns.HEIGHT,
        ]

        # Geenerate a DF
        cols = reader.columns.values
        class_cols = [col for col in cols if "classType" in col]
        length_cols = [col for col in cols if "length" in col]
        width_cols = [col for col in cols if "width" in col]
        height_cols = [col for col in cols if "height" in col]

        unique_ts = reader.ts.values

        for ts in unique_ts:
            res = pd.DataFrame(columns=object_members)
            detections_ts = reader.loc[reader.ts == ts]
            numObjects = detections_ts["numObjects"].values[0]
            savedColums = 7 + numObjects * 9
            currentCols = cols[0:savedColums]
            real_detections_ts = detections_ts[currentCols]

            res["classType"] = real_detections_ts[class_cols[0:numObjects]].values.reshape(
                -1,
            )
            res["length"] = real_detections_ts[length_cols[0:numObjects]].values.reshape(
                -1,
            )
            res["width"] = real_detections_ts[width_cols[0:numObjects]].values.reshape(
                -1,
            )
            res["height"] = real_detections_ts[height_cols[0:numObjects]].values.reshape(
                -1,
            )

            # Test sizes for each cuboid
            # TODO: Edit after GRAPPA interface changes
            # Flag set to false when invalid objects are found in the timestamp
            valid_timeframe = True

            for className in ct.TppCuboids:
                classType = ct.ClassTypes[className]
                df = res[res["classType"] == classType]

                valid_timeframe = valid_timeframe & fh_tpp.check_size(df, "width", className)
                valid_timeframe = valid_timeframe & fh_tpp.check_size(df, "length", className)
                valid_timeframe = valid_timeframe & fh_tpp.check_size(df, "height", className)

            if not valid_timeframe:
                invalid_frames_list.append(ts)

        invalid_no_frames = len(invalid_frames_list)
        valid_no_frames = len(unique_ts) - invalid_no_frames

        fig = go.Figure(
            go.Pie(
                values=[valid_no_frames, invalid_no_frames],
                labels=["ok", "not ok"],
                marker={"colors": ["green", "red"]},
            )
        )

        plot_titles.insert(0, "Ratio of invalid frames")
        plots.insert(0, fig)
        remarks.insert(0, "Ratio of invalid frames to number of frames")

        if len(invalid_frames_list) == 0:
            test_result = fc.PASS
        else:
            test_result = fc.FAIL

        result_df = {
            "Verdict": {"value": test_result.title(), "color": fh.get_color(test_result)},
            fc.REQ_ID: ["TODO"],
            fc.TESTCASE_ID: ["L3_TPP_41582"],
            fc.TEST_SAFETY_RELEVANT: ["TODO"],
            fc.TEST_DESCRIPTION: ["TODO"],
            fc.TEST_RESULT: [test_result],
        }

        if test_result == fc.PASS:
            self.result.measured_result = TRUE
        else:
            self.result.measured_result = FALSE
        for plot in plots:
            self.result.details["Plots"].append(plot.to_html(full_html=False, include_plotlyjs=False))
        for plot_title in plot_titles:
            self.result.details["Plot_titles"].append(plot_title)
        for remark in remarks:
            self.result.details["Remarks"].append(remark)

        self.result.details["Additional_results"] = result_df


@verifies("L3_TPP_41582")
@testcase_definition(
    name="TPP TPP_ValidateCuboidSizesOutput",
    description="This test will verify the sizes for each DynamicObject_t with cuboid class",
)
@register_inputs("/Playground_2/TSF-Debug")
# @register_inputs("/TSF_DEBUG/")
class TestCuboidSizesOutput(TestCase):
    """Output of cuboid sizes test case."""

    custom_report = fh_tpp.TPPCustomTestcaseReport

    @property
    def test_steps(self):
        """Define the test steps."""
        return [
            TestCuboidSizesOutputTestStep,
        ]


@teststep_definition(
    step_number=1,
    name="TPP_ValidateCuboidYawAngleOutput",
    description="This test will verify the yaw angle for each TPP Cuboid",
    expected_result=BooleanResult(TRUE),
)
@register_signals(FT_TPP, fh_tpp.TPPSignals)
class TestCuboidYawOutputTestStep(TestStep):
    """Test Step definition."""

    custom_report = fh_tpp.TPPCustomTeststepReport

    def __init__(self):
        """Initialize the teststep."""
        super().__init__()

    def process(self, **kwargs):
        """Process the test result."""
        _log.debug("Starting processing...")
        reader = self.readers[FT_TPP]
        self.result.details.update(
            {"Plots": [], "Plot_titles": [], "Remarks": [], "file_name": os.path.basename(self.artifacts[0].file_path)}
        )
        test_result = fc.INPUT_MISSING  # Result
        plot_titles, plots, remarks = rep([], 3)

        invalid_frames_list = []  # list of frames with errors

        # List signals to be read
        object_members = [fh_tpp.TPPSignals.Columns.CLASSTYPE, fh_tpp.TPPSignals.Columns.YAW]

        # Geenerate a DF
        cols = reader.columns.values

        class_cols = [col for col in cols if "classType" in col]
        yaw_cols = [col for col in cols if "yaw" in col]
        unique_ts = reader.ts.values

        for ts in unique_ts:
            res = pd.DataFrame(columns=object_members)
            detections_ts = reader.loc[reader.ts == ts]
            numObjects = detections_ts["numObjects"].values[0]
            savedColums = 7 + numObjects * 9
            currentCols = cols[0:savedColums]
            real_detections_ts = detections_ts[currentCols]

            res["classType"] = real_detections_ts[class_cols[0:numObjects]].values.reshape(
                -1,
            )
            res["yaw"] = real_detections_ts[yaw_cols[0:numObjects]].values.reshape(
                -1,
            )

            # Test sizes for each cuboid
            # TODO: Edit after GRAPPA interface changes
            # Flag set to false when invalid objects are found in the timestamp
            valid_timeframe = True

            for className in ct.TppCuboids:
                classType = ct.ClassTypes[className]
                df = res[res["classType"] == classType]

                valid_timeframe = valid_timeframe & fh_tpp.check_yaw(df)

            if not valid_timeframe:
                invalid_frames_list.append(ts)

        invalid_no_frames = len(invalid_frames_list)
        valid_no_frames = len(unique_ts) - invalid_no_frames

        fig = go.Figure(
            go.Pie(
                values=[valid_no_frames, invalid_no_frames],
                labels=["ok", "not ok"],
                marker={"colors": ["green", "red"]},
            )
        )

        plot_titles.insert(0, "Ratio of invalid frames")
        plots.insert(0, fig)
        remarks.insert(0, "Ratio of invalid frames to number of frames")

        if invalid_no_frames == 0:
            test_result = fc.PASS
        else:
            test_result = fc.FAIL

        result_df = {
            "Verdict": {"value": test_result.title(), "color": fh.get_color(test_result)},
            fc.REQ_ID: ["TODO"],
            fc.TESTCASE_ID: ["TODO"],
            fc.TEST_SAFETY_RELEVANT: ["TODO"],
            fc.TEST_DESCRIPTION: ["TODO"],
            fc.TEST_RESULT: [test_result],
        }

        if test_result == fc.PASS:
            self.result.measured_result = TRUE
        else:
            self.result.measured_result = FALSE

        for plot, plot_title, remark in zip(plots, plot_titles, remarks):
            self.result.details["Plots"].append(
                f"<h2>{plot_title}</h2>{plot.to_html(full_html=False, include_plotlyjs=False)}<p>{remark}</p>"
            )

        self.result.details["Additional_results"] = result_df


@verifies("TODO")
@testcase_definition(
    name="TPP TPP_ValidateCuboidYawOutput",
    description="This test will verify the value of the yaw angle for each TPP Cuboid",
)
@register_inputs("/Playground_2/TSF-Debug")
# @register_inputs("/TSF_DEBUG/")
class TestCuboidYawOutput(TestCase):
    """Cuboid output yaw test case."""

    custom_report = fh_tpp.TPPCustomTestcaseReport

    @property
    def test_steps(self):
        """Define the test steps."""
        return [
            TestCuboidYawOutputTestStep,
        ]


@teststep_definition(
    step_number=1,
    name="TPP_ValidateCuboidConfidenceAngleOutput",
    description="This test will verify the confidence for each TPP Cuboid",
    expected_result=BooleanResult(TRUE),
)
@register_signals(FT_TPP, fh_tpp.TPPSignals)
class TestCuboidConfidenceOutputTestStep(TestStep):
    """Test Step definition."""

    custom_report = fh_tpp.TPPCustomTeststepReport

    def __init__(self):
        """Initialize the teststep."""
        super().__init__()

    def process(self, **kwargs):
        """Process the test result."""
        _log.debug("Starting processing...")
        reader = self.readers[FT_TPP]
        self.result.details.update(
            {"Plots": [], "Plot_titles": [], "Remarks": [], "file_name": os.path.basename(self.artifacts[0].file_path)}
        )
        test_result = fc.INPUT_MISSING  # Result
        plot_titles, plots, remarks = rep([], 3)

        invalid_frames_list = []  # list of frames with errors

        # List signals to be read
        object_members = [fh_tpp.TPPSignals.Columns.CLASSTYPE, fh_tpp.TPPSignals.Columns.CONFIDENCE]

        # Geenerate a DF
        cols = reader.columns.values

        class_cols = [col for col in cols if "classType" in col]
        confidence_cols = [col for col in cols if "confidence" in col]
        unique_ts = reader.ts.values

        for ts in unique_ts:
            res = pd.DataFrame(columns=object_members)
            detections_ts = reader.loc[reader.ts == ts]
            numObjects = detections_ts["numObjects"].values[0]
            savedColums = 7 + numObjects * 9
            currentCols = cols[0:savedColums]
            real_detections_ts = detections_ts[currentCols]

            res["classType"] = real_detections_ts[class_cols[0:numObjects]].values.reshape(
                -1,
            )
            res["confidence"] = real_detections_ts[confidence_cols[0:numObjects]].values.reshape(
                -1,
            )

            # Test sizes for each cuboid
            # TODO: Edit after GRAPPA interface changes
            # Flag set to false when invalid objects are found in the timestamp
            valid_timeframe = True

            for className in ct.TppCuboids:
                classType = ct.ClassTypes[className]
                df = res[res["classType"] == classType]
                valid_timeframe = valid_timeframe & fh_tpp.check_confidence(df)

            if not valid_timeframe:
                invalid_frames_list.append(ts)

        invalid_no_frames = len(invalid_frames_list)
        valid_no_frames = len(unique_ts) - invalid_no_frames

        fig = go.Figure(
            go.Pie(
                values=[valid_no_frames, invalid_no_frames],
                labels=["ok", "not ok"],
                marker={"colors": ["green", "red"]},
            )
        )

        plot_titles.insert(0, "Ratio of invalid frames")
        plots.insert(0, fig)
        remarks.insert(0, "Ratio of invalid frames to number of frames")

        if invalid_no_frames == 0:
            test_result = fc.PASS
        else:
            test_result = fc.FAIL

        result_df = {
            "Verdict": {"value": test_result.title(), "color": fh.get_color(test_result)},
            fc.REQ_ID: ["TODO"],
            fc.TESTCASE_ID: ["TODO"],
            fc.TEST_SAFETY_RELEVANT: ["TODO"],
            fc.TEST_DESCRIPTION: ["TODO"],
            fc.TEST_RESULT: [test_result],
        }

        if test_result == fc.PASS:
            self.result.measured_result = TRUE
        else:
            self.result.measured_result = FALSE

        for plot, plot_title, remark in zip(plots, plot_titles, remarks):
            self.result.details["Plots"].append(
                f"<h2>{plot_title}</h2>{plot.to_html(full_html=False, include_plotlyjs=False)}<p>{remark}</p>"
            )

        self.result.details["Additional_results"] = result_df


@verifies("TODO")
@testcase_definition(
    name="TPP TPP_ValidateCuboidConfidenceOutput",
    description="This test will verify the value of the confidence for each TPP Cuboid",
)
@register_inputs("/Playground_2/TSF-Debug")
# @register_inputs("/TSF_DEBUG/")
class TestCuboidConfidenceOutput(TestCase):
    """Cuboid output confidence test case."""

    custom_report = fh_tpp.TPPCustomTestcaseReport

    @property
    def test_steps(self):
        """Define the test steps."""
        return [
            TestCuboidConfidenceOutputTestStep,
        ]


@teststep_definition(
    step_number=1,
    name="TPP_ValidateCuboidCenterPointOutput",
    description="This test will verify the center point for each TPP Cuboid",
    expected_result=BooleanResult(TRUE),
)
@register_signals(FT_TPP, fh_tpp.TPPSignals)
class TestCuboidCenterPointOutputTestStep(TestStep):
    """Test Step definition."""

    custom_report = fh_tpp.TPPCustomTeststepReport

    def __init__(self):
        """Initialize the teststep."""
        super().__init__()

    def process(self, **kwargs):
        """Process the test result."""
        _log.debug("Starting processing...")
        reader = self.readers[FT_TPP]
        self.result.details.update(
            {"Plots": [], "Plot_titles": [], "Remarks": [], "file_name": os.path.basename(self.artifacts[0].file_path)}
        )
        test_result = fc.INPUT_MISSING  # Result
        plot_titles, plots, remarks = rep([], 3)

        invalid_frames_list = []  # list of frames with errors

        # List signals to be read
        object_members = [
            fh_tpp.TPPSignals.Columns.CLASSTYPE,
            fh_tpp.TPPSignals.Columns.CENTERX,
            fh_tpp.TPPSignals.Columns.CENTERY,
            fh_tpp.TPPSignals.Columns.CENTERZ,
        ]

        # Geenerate a DF
        cols = reader.columns.values
        class_cols = [col for col in cols if "classType" in col]
        cx_cols = [col for col in cols if "center_x" in col]
        cy_cols = [col for col in cols if "center_y" in col]
        cz_cols = [col for col in cols if "center_z" in col]

        unique_ts = reader.ts.values

        for ts in unique_ts:
            res = pd.DataFrame(columns=object_members)
            detections_ts = reader.loc[reader.ts == ts]
            numObjects = detections_ts["numObjects"].values[0]
            savedColums = 7 + numObjects * 9
            currentCols = cols[0:savedColums]
            real_detections_ts = detections_ts[currentCols]

            res["classType"] = real_detections_ts[class_cols[0:numObjects]].values.reshape(
                -1,
            )
            res["center_x"] = real_detections_ts[cx_cols[0:numObjects]].values.reshape(
                -1,
            )
            res["center_y"] = real_detections_ts[cy_cols[0:numObjects]].values.reshape(
                -1,
            )
            res["center_z"] = real_detections_ts[cz_cols[0:numObjects]].values.reshape(
                -1,
            )

            # Test sizes for each cuboid
            # TODO: Edit after GRAPPA interface changes
            # Flag set to false when invalid objects are found in the timestamp
            valid_timeframe = True

            for className in ct.TppCuboids:
                classType = ct.ClassTypes[className]
                df = res[res["classType"] == classType]

                valid_timeframe = valid_timeframe & fh_tpp.check_center_point(df)

            if not valid_timeframe:
                invalid_frames_list.append(ts)

        invalid_no_frames = len(invalid_frames_list)
        valid_no_frames = len(unique_ts) - invalid_no_frames

        fig = go.Figure(
            go.Pie(
                values=[valid_no_frames, invalid_no_frames],
                labels=["ok", "not ok"],
                marker={"colors": ["green", "red"]},
            )
        )

        plot_titles.insert(0, "Ratio of invalid frames")
        plots.insert(0, fig)
        remarks.insert(0, "Ratio of invalid frames to number of frames")

        if invalid_no_frames == 0:
            test_result = fc.PASS
        else:
            test_result = fc.FAIL

        result_df = {
            "Verdict": {"value": test_result.title(), "color": fh.get_color(test_result)},
            fc.REQ_ID: ["TODO"],
            fc.TESTCASE_ID: ["TODO"],
            fc.TEST_SAFETY_RELEVANT: ["TODO"],
            fc.TEST_DESCRIPTION: ["TODO"],
            fc.TEST_RESULT: [test_result],
        }

        if test_result == fc.PASS:
            self.result.measured_result = TRUE
        else:
            self.result.measured_result = FALSE

        for plot, plot_title, remark in zip(plots, plot_titles, remarks):
            self.result.details["Plots"].append(
                f"<h2>{plot_title}</h2>{plot.to_html(full_html=False, include_plotlyjs=False)}<p>{remark}</p>"
            )

        self.result.details["Additional_results"] = result_df


@verifies("TODO")
@testcase_definition(
    name="TPP TPP_ValidateCuboidCenterPointOutput",
    description="This test will verify the value of the center point for each TPP Cuboid",
)
@register_inputs("/Playground_2/TSF-Debug")
# @register_inputs("/TSF_DEBUG/")
class TestCuboidCenterPointOutput(TestCase):
    """Cuboid output center point test case."""

    custom_report = fh_tpp.TPPCustomTestcaseReport

    @property
    def test_steps(self):
        """Define the test steps."""
        return [
            TestCuboidCenterPointOutputTestStep,
        ]


@teststep_definition(
    step_number=1,
    name="TPP_ValidateCuboidClassTypeOutput",
    description="This test will verify the class type for each TPP Cuboid",
    expected_result=BooleanResult(TRUE),
)
@register_signals(FT_TPP, fh_tpp.TPPSignals)
class TestCuboidClassTypeOutputTestStep(TestStep):
    """Test Step definition."""

    custom_report = fh_tpp.TPPCustomTeststepReport

    def __init__(self):
        """Initialize the teststep."""
        super().__init__()

    def process(self, **kwargs):
        """Process the test result."""
        _log.debug("Starting processing...")
        reader = self.readers[FT_TPP]
        self.result.details.update(
            {"Plots": [], "Plot_titles": [], "Remarks": [], "file_name": os.path.basename(self.artifacts[0].file_path)}
        )
        test_result = fc.INPUT_MISSING  # Result
        plot_titles, plots, remarks = rep([], 3)

        invalid_frames_list = []  # list of frames with errors

        # List signals to be read
        object_members = [fh_tpp.TPPSignals.Columns.CLASSTYPE]

        # Geenerate a DF
        cols = reader.columns.values

        class_cols = [col for col in cols if "classType" in col]
        unique_ts = reader.ts.values

        for ts in unique_ts:
            res = pd.DataFrame(columns=object_members)
            detections_ts = reader.loc[reader.ts == ts]
            numObjects = detections_ts["numObjects"].values[0]
            savedColums = 7 + numObjects * 9
            currentCols = cols[0:savedColums]
            real_detections_ts = detections_ts[currentCols]

            res["classType"] = real_detections_ts[class_cols[0:numObjects]].values.reshape(
                -1,
            )

            # Test sizes for each cuboid
            # TODO: Edit after GRAPPA interface changes
            # Flag set to false when invalid objects are found in the timestamp
            valid_timeframe = True

            for className in ct.TppCuboids:
                classType = ct.ClassTypes[className]
                df = res[res["classType"] == classType]

                valid_timeframe = valid_timeframe & fh_tpp.check_class_type(df, ct.TppCuboids)

            if not valid_timeframe:
                invalid_frames_list.append(ts)

        invalid_no_frames = len(invalid_frames_list)
        valid_no_frames = len(unique_ts) - invalid_no_frames

        fig = go.Figure(
            go.Pie(
                values=[valid_no_frames, invalid_no_frames],
                labels=["ok", "not ok"],
                marker={"colors": ["green", "red"]},
            )
        )

        plot_titles.insert(0, "Ratio of invalid frames")
        plots.insert(0, fig)
        remarks.insert(0, "Ratio of invalid frames to number of frames")

        if invalid_no_frames == 0:
            test_result = fc.PASS
        else:
            test_result = fc.FAIL

        result_df = {
            "Verdict": {"value": test_result.title(), "color": fh.get_color(test_result)},
            fc.REQ_ID: ["TODO"],
            fc.TESTCASE_ID: ["TODO"],
            fc.TEST_SAFETY_RELEVANT: ["TODO"],
            fc.TEST_DESCRIPTION: ["TODO"],
            fc.TEST_RESULT: [test_result],
        }

        if test_result == fc.PASS:
            self.result.measured_result = TRUE
        else:
            self.result.measured_result = FALSE

        for plot, plot_title, remark in zip(plots, plot_titles, remarks):
            self.result.details["Plots"].append(
                f"<h2>{plot_title}</h2>{plot.to_html(full_html=False, include_plotlyjs=False)}<p>{remark}</p>"
            )

        self.result.details["Additional_results"] = result_df


@verifies("TODO")
@testcase_definition(
    name="TPP TPP_ValidateCuboidClassTypeOutput",
    description="This test will verify the value of the class types for each TPP Cuboid",
)
@register_inputs("/Playground_2/TSF-Debug")
# @register_inputs("/TSF_DEBUG/")
class TestCuboidClassTypeOutput(TestCase):
    """Cuboid output class type test case."""

    custom_report = fh_tpp.TPPCustomTestcaseReport

    @property
    def test_steps(self):
        """Define the test steps."""
        return [
            TestCuboidClassTypeOutputTestStep,
        ]
