#! /usr/bin/env python3
import os
import sys
import subprocess
import logging
from xml.etree import ElementTree
from enum import Enum
from optparse import OptionParser
from shutil import copy, move


class Variant(Enum):
    Base = 1
    Entry = 2
    Performance = 3


class TestCatalogue(Enum):
    """Enumeration of maneuvering functions test catalogues"""
    AP_RegressionTests = 1       # mid regression test catalogue for AP
    AP_AutoTestExecution = 2     # full regression test catalogue for AP
    AP_SmallRegressionTests = 3  # small regression test catalogue for AP
    SI_RegressionTests = 4       # small regression test catalogue for SI
    MP_SmallRegressionTests = 5  # small regression test catalogue for memory parking


class TestCase:
    """
    A class to represent a test case consisting of test that belongs to a report.

    Attributes:
        report (str): the report name
        test (str): th test name
    """
    def __init__(self, report, test):
        self.report = report
        self.test = test

    def __hash__(self):
        return hash((self.report, self.test))

    def __eq__(self, other):
        return (self.report, self.test) == (other.report, other.test)

    def __ne__(self, other):
        return not self == other

    def __repr__(self):
        return repr((self.report, self.test))


class TestResult(Enum):
    """Enumeration of possible test results (FAILED, NOT_ASSESSED, PASSED)"""
    FAILED = 1
    NOT_ASSESSED = 2
    PASSED = 3

    def is_better_than(self, other):
        """Returns whether this TestResult is PASSED and the other is not."""
        return self == TestResult.PASSED and other != TestResult.PASSED

    def is_minor_change(self, other):
        """Returns whether this and the other TestResult are not PASSED
        and differ (FAILED vs. NOT_ASSESSED)."""
        return {self, other} == {TestResult.FAILED, TestResult.NOT_ASSESSED}

#Error Code for test evaluation dropped (see ErrorCodes_cfg.py)
EVALUATION_DROPPED = pow(2, 11)
logging.basicConfig(level=logging.DEBUG, style = "{", format = "[{levelname}] {filename}({lineno}): {message}")

THRESHOLD_XML_PATH = os.path.abspath(os.path.dirname(__file__))
PATH_selectedVariant = os.path.abspath(os.path.join(os.path.dirname(__file__),'..','..', '..','..', 'scripts', 'selected_variant.txt'))

def parseRegressionTestResults(summaryFilePath):
    if not os.path.isfile(summaryFilePath):
        logging.error("Test summary file does not exist: " + summaryFilePath)
        sys.exit(EVALUATION_DROPPED)
    summaryXML = ElementTree.parse(summaryFilePath)
    report2passedPercentage = {}
    for report in summaryXML.getroot():
        reportName = report.tag
        passedTests = float(report.find("Passed").text.split("=")[0])
        totalTests = float(report.find("Total").text)
        report2passedPercentage.setdefault(reportName, passedTests/totalTests * 100.0)
    return report2passedPercentage

def parseRegressionTestThresholds(thresholdsFilePath):
    if not os.path.isfile(thresholdsFilePath):
        logging.error("Test thresholds file does not exist: " + thresholdsFilePath)
        sys.exit(EVALUATION_DROPPED)
    thresholdXML = ElementTree.parse(thresholdsFilePath)
    report2Threshold = {}
    for report in thresholdXML.getroot():
        reportName = report.find("Name").text
        threshold = float(report.find("Threshold").text)
        report2Threshold.setdefault(reportName, threshold)
    return report2Threshold


def parse_test_results(test_results_file_path):
    """
    Parses the test results file and creates a dictionary
    that maps the TestCase(report, test) to the TestResult.

    Arguments:
        test_results_file_path (str): Path to the test results file
                                      (e.g. 'Summary_Test_Results.txt')

    Returns:
        test results mapping {(report, test) : result} (dict{TestCase : TestResult})
    """
    test_results_mapping = {}
    summaryXML = ElementTree.parse(test_results_file_path)

    for kpi in summaryXML.getroot():
        for child in kpi:
            # Temporary workaround: ignore MF_SLOT_PERSISTENCE test results for now since they are not mature yet.
            if kpi.tag == "MF_SLOT_PERSISTENCE":
                continue
            test_case = TestCase(kpi.tag, child.tag)
            try:
                test_result = TestResult[child.text]
            except KeyError as error:
                print(error)
                test_result = TestResult["NOT_ASSESSED"]
            test_results_mapping[test_case] = test_result

    return test_results_mapping

def diff_test_results(new_test_results_path, old_test_results_path, selectedTestCatalogue):
    """
   Compares the new test results to the checked in test results and
   writes the test result changes to the file 'Changed_Test_Results.txt'.

    Arguments:
        new_test_results_path (str): Path to the new test results file
        old_test_results_path (str): Path to the old test results file
    """
    # backup old test results and revert it to checked in version
    copy(old_test_results_path, old_test_results_path + "-backup")
    subprocess.run("git checkout -- " + os.path.basename(old_test_results_path),
                   cwd=test_report_path, check=True, shell=True, stdout=subprocess.PIPE)
    old_test_results = parse_test_results(old_test_results_path)
    new_test_results = parse_test_results(new_test_results_path)
    # restore old test results file
    move(old_test_results_path + "-backup", old_test_results_path)

    old_test_cases = set(old_test_results.keys())
    new_test_cases = set(new_test_results.keys())
    additional_tests = new_test_cases.difference(old_test_cases)
    missing_tests = old_test_cases.difference(new_test_cases)
    worsened_tests = [test for test in old_test_cases.intersection(
        new_test_cases) if old_test_results[test].is_better_than(new_test_results[test])]
    improved_tests = [test for test in old_test_cases.intersection(
        new_test_cases) if new_test_results[test].is_better_than(old_test_results[test])]
    other_changed_tests = [test for test in old_test_cases.intersection(
        new_test_cases) if new_test_results[test].is_minor_change(old_test_results[test])]

    to_str = lambda test_case: "\n\t(%s) %s: %s -> %s" % \
        (test_case.report, test_case.test,
         old_test_results[test_case].name, new_test_results[test_case].name)
    diff_text = "WORSENED TESTS:" + \
        "".join(sorted([to_str(test_case) for test_case in worsened_tests]))
    diff_text += "\n\nIMPROVED TESTS:" + \
        "".join(sorted([to_str(test_case) for test_case in improved_tests]))
    diff_text += "\n\nOTHER CHANGED TESTS:" + \
        "".join(sorted([to_str(test_case) for test_case in other_changed_tests]))
    to_str = lambda test_case: "\n\t(%s) %s: %s" % \
        (test_case.report, test_case.test, old_test_results[test_case].name)
    diff_text += "\n\nMISSING TESTS:" + \
        "".join(sorted([to_str(test_case) for test_case in missing_tests]))
    to_str = lambda test_case: "\n\t(%s) %s: %s" % \
        (test_case.report, test_case.test, new_test_results[test_case].name)
    diff_text += "\n\nADDITIONAL TESTS:" + \
        "".join(sorted([to_str(test_case) for test_case in additional_tests]))
    diff_text += "\n"
    logging.info("Test result changes:\n%s", diff_text)
    new_file_name = "Changed_" + os.path.splitext(os.path.basename(new_test_results_path))[0] + ".txt"
    changed_results_path = os.path.join(os.path.dirname(new_test_results_path), new_file_name)
    with open(changed_results_path, 'w') as changed_results_file:
        changed_results_file.write(diff_text)

    # Error cases
    ## All Regression tests: Missing tests
    if len(missing_tests) > 0:
        logging.error("There are %d missing test results!", len(missing_tests))
        sys.exit(EVALUATION_DROPPED)
    ## AP or SI Small Regression: Worsened, Other changed (needed because some KPI thresholds below 100 percent)
    elif selectedTestCatalogue in (TestCatalogue.AP_SmallRegressionTests, TestCatalogue.SI_RegressionTests):
        if len(worsened_tests) > 0:
            logging.error("There are %d worsened test results!", len(worsened_tests))
            sys.exit(EVALUATION_DROPPED)
        elif len(other_changed_tests) > 0:
            logging.error("There are %d other changed test results!", len(other_changed_tests))
            sys.exit(EVALUATION_DROPPED)

if __name__ == '__main__':
     ### Parse Command Line Options
    optparser = OptionParser(usage="usage: %prog [options]")
    catalogueChoices = [catalogue.name for catalogue in TestCatalogue]
    catalogueDefault = TestCatalogue.AP_RegressionTests.name
    optparser.add_option("-c", "--catalogue", dest='testCatalogue',
                        type="choice", choices=catalogueChoices, default=catalogueDefault,
                        help="Test catalogue to evaluate. Valid options: %s, Default: %s" % (catalogueChoices, catalogueDefault))
    options = optparser.parse_args()[0]
    selectedTestCatalogue = TestCatalogue[options.testCatalogue]
    logging.info("START evaluate test results")
    logging.info("selected test catalogue: " + selectedTestCatalogue.name)

    # Check, if selected_variant.txt file exists. In case file is not available, use base as default.
    selectedVariant = Variant.Base
    if os.path.isfile(PATH_selectedVariant):
        with open(PATH_selectedVariant, 'r') as variantFile:
            variantFileContent = variantFile.readline().strip()
            if variantFileContent == 'entry':
                selectedVariant = Variant.Entry
            elif variantFileContent == 'performance':
                selectedVariant = Variant.Performance
            else:
                selectedVariant = Variant.Base
        logging.info("selected variant: " + selectedVariant.name)
    else:
        logging.warn("Missing variant file '%s', using default variant '%s'.", PATH_selectedVariant, selectedVariant.name)

    if TestCatalogue.AP_RegressionTests == selectedTestCatalogue:
        summarySubfolder = 'Regr_Reports_AUP'
        if selectedVariant == Variant.Base:
            thresholdsFileName = 'Test_Thresholds_Mid.xml'
        elif selectedVariant == Variant.Performance:
            thresholdsFileName = 'Test_Thresholds_Mid_Performance.xml'
        else:
            thresholdsFileName = 'Test_Thresholds_Mid_CUS.xml'
    elif TestCatalogue.AP_SmallRegressionTests == selectedTestCatalogue:
        summarySubfolder = 'Small_Regr_AUP'
        if selectedVariant == Variant.Base:
            thresholdsFileName = 'Test_Thresholds_Small.xml'
        elif selectedVariant == Variant.Performance:
            thresholdsFileName = 'Test_Thresholds_Small_Performance.xml'
        else:
            thresholdsFileName = 'Test_Thresholds_Small_CUS.xml'
    elif TestCatalogue.AP_AutoTestExecution == selectedTestCatalogue:
        summarySubfolder = 'Reports_AUP'
        thresholdsFileName = 'Test_Thresholds_Full.xml'
    elif TestCatalogue.SI_RegressionTests == selectedTestCatalogue:
        summarySubfolder = 'Output_Regr_Reports_SI'
        thresholdsFileName = ('SI_Test_Thresholds.xml' if (Variant.Base == selectedVariant) else 'SI_Test_Thresholds_CUS.xml')
    elif TestCatalogue.MP_SmallRegressionTests == selectedTestCatalogue:
        summarySubfolder = 'Small_Regr_MP'
        if selectedVariant == Variant.Base:
            thresholdsFileName = 'Test_Thresholds_Small_MP.xml'
        elif selectedVariant == Variant.Performance:
            thresholdsFileName = 'Test_Thresholds_Small_MP.xml'
        else:
            thresholdsFileName = 'Test_Thresholds_Small_MP.xml'
    else:
        raise Exception("The selected test catalogue '{}' is not supported!".format(selectedTestCatalogue.name))
        
    summaryFileName = 'Summary_per_KPI.xml'    
    test_report_path = os.path.abspath(os.path.join(os.path.dirname(
        __file__), '..', '..', 'Test_Results', summarySubfolder))
    summaryFilePath = os.path.join(test_report_path, 'LastReport', summaryFileName)

    report2passedTestPerc = parseRegressionTestResults(summaryFilePath)
    report2passedTestThreshold = parseRegressionTestThresholds(os.path.join(THRESHOLD_XML_PATH, thresholdsFileName))
    
    # Check whether the passed test percentage is greater than or equal to the threshold for all test reports
    regressionTestsFailed = False
    for reportName in report2passedTestThreshold:
        if reportName in report2passedTestPerc:
            if report2passedTestPerc[reportName] < report2passedTestThreshold[reportName]:
                regressionTestsFailed = True
                logging.error("Report '%s': Passed test percentage %.1f%% is below the threshold %.1f%%.", reportName, report2passedTestPerc[reportName], report2passedTestThreshold[reportName])
        else:
            logging.warn("No report corresponding to the threshold for '%s' found in the test summary file: %s", reportName, summaryFilePath)

    if selectedVariant == Variant.Base:
        RESULTS_FILE_NAME = 'Verdict_per_KPI.xml'
    elif selectedVariant == Variant.Performance:
        RESULTS_FILE_NAME = 'Verdict_per_KPI_Performance.xml'
    else:
        RESULTS_FILE_NAME = 'Verdict_per_KPI_CUS.xml'
    
    diff_test_results(os.path.join(test_report_path, 'LastReport', 'Verdict_per_KPI.xml'),
                      os.path.join(test_report_path, RESULTS_FILE_NAME),
                      selectedTestCatalogue)

    logging.info("END evaluate regression test results")
    logging.shutdown()
    sys.exit(EVALUATION_DROPPED if regressionTestsFailed else 0)
