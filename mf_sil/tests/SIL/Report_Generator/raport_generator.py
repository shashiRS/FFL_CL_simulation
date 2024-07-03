import json
import logging
import os
import yaml
import shutil
import subprocess
import sys
import time
import webbrowser
from collections import defaultdict
from distutils.dir_util import copy_tree
from os.path import exists, isdir, isfile, join
from xml.dom import minidom
from xml.etree import ElementTree

import jinja2
import pandas as pd
import plotly.graph_objects as go
from lxml import etree as ET
from plotly.subplots import make_subplots

class MissingCfgFile(Exception):
    
    def __init__(self, message):
        self.message = message
    def __str__(self):
        return self.message

EVALUATION_DROPPED = pow(2, 11)



def reset_logger(logger):
    """Clear all handlers of the logger."""
    if logger.hasHandlers():
        for handler in logger.handlers[:]:
            logger.removeHandler(handler)
            handler.close()
def delete_old_contents(folder):
    """Delete old data from specified folder"""
    for filename in os.listdir(folder):
        file_path = join(folder, filename)
        try:
            if isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            logger.error("Failed to delete %s. Reason: %s" % (file_path, e))

def save_xml(f_verdict, f_summary):
    """Save the xml data in specified folders"""
    # Cleaning the Last Report folder
    delete_old_contents(last_report_folder_path_v2)

    # Save the files to specified path
    file = open(join(current_report_path, "Verdict_per_KPI.xml"), "w")
    file.write(f_verdict)
    file.close()

    file = open(join(current_report_path, "Summary_per_KPI.xml"), "w")
    file.write(f_summary)
    file.close()


def find_jsons(folder_path, text_to_match: str):
    """Search for all functional test results in the Reports path"""
    json_file_list = []

    # Use os.walk to go into subfolders
    for folder, subfolders, files in os.walk(folder_path):

        if folder != folder_path:
            for file in files:
                if (
                    text_to_match.lower() in file
                ):  
                    json_file_list.append(join(folder, file))

    return json_file_list


def unpack_json(json_list):
    """Look into the found json file and unpack all the neccesary information"""
    # Initialize dictionaries to hold data for Summary and Verdict xmls
    verdict_dict = {}
    summary_dict_temp = {}
    summary_dict = {}
    # List and dict to be used for summary xml
    filename_dict = {}

    parking_type_dict = {
        "AUPSim_UC_ParRight": [],
        "AUPSim_UC_ParLeft": [],
        "AUPSim_UC_PerpRight": [],
        "AUPSim_UC_PerpLeft": [],
        "AUPSim_NUC_ParRight": [],
        "AUPSim_NUC_ParLeft": [],
        "AUPSim_NUC_PerpLeft": [],
        "AUPSim_NUC_PerpRight": [],
        "AUPSim_UC_AngLeft": [],
        "AUPSim_UC_AngRight": [],
    }

    if IS_REGRESSION_TEST:
        for json_path in json_list:
            # After each iteration clean the data from summary_dict_temp
            summary_dict_temp.clear()

            try:
                # Open and read json file
                file = open(json_path, "r")
                file_data = json.loads(file.read())

                # Make a list with all the filenames found in the json file
                filename_list = list(file_data.keys())

                # Make a list with test_files found
                # [0] was used to get tests. Reason: all .erg files will have the same tests
                func_tests_list = file_data.get(filename_list[0])

                # Populating the dict with filenames
                # This dict will be used for the summary xml
                for file_name in filename_list:
                    filename_dict[file_name] = []

                # Add the test_names to the verdict dictionary
                for test_name in func_tests_list.keys():
                    if test_name not in verdict_dict.keys():
                        verdict_dict[test_name] = []
                # Go through each file and then through each test for that file
                for file in file_data:
                    for tests in file_data[file].keys():
                        if tests != "Overall result":
                            # Get result from each test for that file
                            result = file_data[file][tests]["Result"]
                            result = str.upper(result)
                            if result == "NOT ASSESSED":
                                result = "NOT_ASSESSED"

                            # Filename and pass/fail result will be stored as a tuple
                            verdict_dict[tests].append((file, result))
                            # Test and pass/fail result will be stored as a tuple
                            filename_dict[file].append((tests, result))

            except IndexError:
                logger.error(f"Json empty:{json_path}")
            except KeyError:
                logger.error(f"KEY error empty:{json_path}")
            except json.decoder.JSONDecodeError:
                logger.error(f"JSON file {json_path} is corrupted")

            """ The logic behind the below for loops:
            Input:
                dict = {'F_park_type_filename_1.erg:[
                                                (Test_tyoe_1, result),
                                                (Test_tyoe_2, result),
                                                ...],
                        F_park_type_filename_1.erg:[
                                                (Test_tyoe_1, result),
                dict_2 = {ParkType_1:(Test_tyoe_1, result), ***Test type selected for that parking type
                                        (Test_tyoe_2, result),
                                        .....],
                        ParkType_2:(Test_tyoe_1, result), 
                                        (Test_tyoe_2, result),
                                        .....],
                        ....}
                dict_3 = {ParkType_1:{Test_tyoe_1:[ result1, result2, result3, result4, ... *** Pass, fail, pass, fail fail, ...
                                    Test_tyoe_2:[ result1, result2, result3, result4, ...
                                        .....},
                        ParkType_2:{Test_tyoe_1:[ result1, result2, result3, result4, ... *** Pass, fail, pass, fail fail, ...
                                    Test_tyoe_2:[ result1, result2, result3, result4, ...
                                        .....},
                                        ...}
                dict_4 = {ParkType_1_Test_type_1:[result1,result2, result3, ....],
                        ParkType_2_Test_type_2:[result1,result2, result3, ....],
                        }
                final_dict = take each key from dict_4 "_UC_ParLeft_VEDODO" and change it to "ParLeft_VEDODO"

            """

        # We use the keys in filename_dict to add them to the parking_type_dict
        # Example: all ParLeft files to Parleft key in the dictionary
        for parking_tyoe in parking_type_dict:
            for filename in filename_dict:

                if parking_tyoe in filename:
                    parking_type_dict[parking_tyoe].append(filename_dict[filename])

        # Unpack that tuple we stored above (test_name, result) and add to a temporary dictionary for later clean up
        # The key:value is  Test name : [list]
        for park_type in parking_type_dict:
            for idx in range(len(parking_type_dict[park_type])):
                for idx2 in range(len(parking_type_dict[park_type][idx])):
                    # [idx2][0] returns the Test name
                    summary_dict_temp[
                        f"{park_type}_{parking_type_dict[park_type][idx][idx2][0]}"
                    ] = []

        # Use the second tuple item - result - to append to that list
        # Example: _UC_PerpLeft_VEDODO = [failed,passed, passed, fail, ... ]
        for park_type in parking_type_dict:
            for idx in range(len(parking_type_dict[park_type])):
                for idx2 in range(len(parking_type_dict[park_type][idx])):

                    # [park_type][idx][idx2][1] returns the result (Pass or Fail)
                    summary_dict_temp[
                        f"{park_type}_{parking_type_dict[park_type][idx][idx2][0]}"
                    ].append(parking_type_dict[park_type][idx][idx2][1])

        try:
            # Pop Overall result, we don't need it
            # verdict_dict.pop('Overall result')

            # Clean key names: _UC_ParLeft --->  ParLeft
            summary_dict = {}
            for name in summary_dict_temp:
                # check for each parking type
                if "Par" in name:
                    index = name.index("Par")
                elif "Perp" in name:
                    index = name.index("Perp")
                elif "Ang" in name:
                    index = name.index("Ang")
                summary_dict[name[index:]] = summary_dict_temp[name]
            summary_dict = {x.replace(" ", "_"): v for x, v in summary_dict.items()}

        except KeyError:
            logger.error("Overall result not found")
            
        except ValueError as e:
            logger.error(str(e))
        logger.info("Saving xml file...")

    else:

        non_regr_keys = {}
        verdict_dict = {}
        summary_dict = {}
        test_cases_names = {}
        for json_path in json_list:
            # After each iteration clean the data from summary_dict_temp
            summary_dict_temp.clear()
            # Open and read json file
            file = open(json_path, "r")
            file_data = json.loads(file.read())
            # Make a list with all the filenames found in the json file
            filename_list = list(file_data.keys())
            testcases = [value for key,value in file_data.items()]
            testcases = [key for data in testcases for key in  data.keys()]
            cleaned_testcases = list(dict.fromkeys(testcases))
            for tc in cleaned_testcases:
                if tc not in verdict_dict:
                    verdict_dict.setdefault(tc,[])
                if tc not in summary_dict:
                    summary_dict.setdefault(tc,[])

            for file in filename_list:
                for key in verdict_dict.keys():
                    meas_result = file_data[file][key]['Result'].upper()
                    verdict_dict[key].append((file,meas_result))
                    summary_dict[key].append(meas_result)
        #test_cases_names.update(({key: 0} for key in file_data[filez]) for filez in filename_list)

    f_verdict, f_summary = build_Mid_Small_Regressionxml(verdict_dict, summary_dict)

    # Save files
    save_xml(f_verdict, f_summary)


def build_Mid_Small_Regressionxml(verdict_dict, summary_dict):
    """Building the xml file using a json file"""
    # Replacing white spaces with _ to comply with XML naming standards
    verdict_dict = {x.replace(" ", "_"): v for x, v in verdict_dict.items()}
    summary_dict = {x.replace(" ", "_"): v for x, v in summary_dict.items()}

    # Creating a root element for each xml
    verdict_root = ET.Element("Summary")
    summary_root = ET.Element("Summary")

    # For each test, add a new child to Summary root
    # Eample: <Summary> <NUC>...</NUC>......</Summary>
    for test_name in verdict_dict.keys():

        verdict_element = ET.SubElement(verdict_root, test_name)
        try:

            # Unpack the tuple we stored earlier (filename,result)
            for files, result in verdict_dict[test_name]:
                ET.SubElement(verdict_element, files).text = result

        except KeyError as err:
            logger.error(f"KEY error empty {str(err)}")

    # For each park type, add new chuld to root
    # Example: <Summary> <ParRight_NUC>...</ParRight_NUC>......</Summary>
    for park_type in summary_dict.keys():
        if "MF_SLOT_DETECTION" in park_type or "MF_SLOT_PERSISTENCE" in park_type:
            pass
        else:
            summary_element = ET.SubElement(summary_root, park_type)

            # Initialize/reset counters for each for loop
            pass_cnt = 0
            pass_prc = 0
            fail_cnt = 0
            fail_prc = 0
            nan_cnt = 0
            nan_prc = 0
            total_cnt = 0

            try:
                # Unpack tuple we stored earlier (test_name, result) and increment counter accordingly
                for result in summary_dict[park_type]:
                    if result == "PASSED":
                        pass_cnt += 1
                    elif result == "FAILED":
                        fail_cnt += 1
                    elif result == "NOT_ASSESSED":
                        nan_cnt += 1

                # Get total count and percentages
                total_cnt = pass_cnt + fail_cnt + nan_cnt
                if total_cnt == 0:
                    pass_prc = 0
                    fail_prc = 0
                else:

                    pass_prc = pass_cnt / total_cnt * 100
                    fail_prc = fail_cnt / total_cnt * 100
                nan_prc = nan_cnt / (pass_cnt + fail_cnt + nan_cnt) * 100

                # Add subelements to child element
                # Example: <ParRight_NUC><Passed>value</Passed>....</ParRight_NUC>
                ET.SubElement(summary_element, "Passed").text = (
                    f"{pass_cnt} = {pass_prc:.2f}%"
                )
                ET.SubElement(summary_element, "Failed").text = (
                    f"{fail_cnt} = {fail_prc:.2f}%"
                )
                ET.SubElement(summary_element, "NotAssessed").text = (
                    f"{nan_cnt} = {nan_prc:.2f}%"
                )
                ET.SubElement(summary_element, "Total").text = f"{total_cnt}"

            except IndexError:
                logger.error("Json empty")
            except KeyError:
                logger.error("KEY error empty")

    for key in verdict_dict.keys():
        if "MF_SLOT_DETECTION" in key or "MF_SLOT_PERSISTENCE" in key:

            # Initialize/reset counters for each for loop
            summary_element = ET.SubElement(summary_root, key)
            pass_cnt = 0
            pass_prc = 0
            fail_cnt = 0
            fail_prc = 0
            nan_cnt = 0
            nan_prc = 0
            total_cnt = 0

            try:

                # Unpack tuple we stored earlier (test_name, result) and increment counter accordingly
                for files, result in verdict_dict[key]:
                    if result == "PASSED":
                        pass_cnt += 1
                    elif result == "FAILED":
                        fail_cnt += 1
                    elif result == "NOT_ASSESSED":
                        nan_cnt += 1

            except IndexError:
                logger.error("Json empty")
            except KeyError:
                logger.error("KEY error empty")
    

            # Get total count and percentages
            total_cnt = pass_cnt + fail_cnt + nan_cnt
            if total_cnt == 0:
                pass_prc = 0
                fail_prc = 0
            else:

                pass_prc = pass_cnt / total_cnt * 100
                fail_prc = fail_cnt / total_cnt * 100
            nan_prc = nan_cnt / (pass_cnt + fail_cnt + nan_cnt) * 100
            # Add subelements to child element
            # Example: <SLOTDETECTION><Passed>value</Passed>....</SLOTDETECTION>
            ET.SubElement(summary_element, "Passed").text = (
                f"{pass_cnt} = {pass_prc:.2f}%"
            )
            ET.SubElement(summary_element, "Failed").text = (
                f"{fail_cnt} = {fail_prc:.2f}%"
            )
            ET.SubElement(summary_element, "NotAssessed").text = (
                f"{nan_cnt} = {nan_prc:.2f}%"
            )
            ET.SubElement(summary_element, "Total").text = f"{total_cnt}"
        else:
            pass
    # Process xml file, add indent
    verdict_xml = ET.tostring(verdict_root)
    summary_xml = ET.tostring(summary_root)
    verdict_xml_indented = minidom.parseString(verdict_xml).toprettyxml(indent="    ")
    summary_xml_indented = minidom.parseString(summary_xml).toprettyxml(indent="    ")

    return verdict_xml_indented, summary_xml_indented

def check_directories(folder_list):
    """Check is specified folders exist. If not, create them"""
    try:

        for path in folder_list:
            if not exists(path):
                os.makedirs(path)
    except OSError as e:
        logger.error(f"Error creating folders {e}")


def delete_old_data(folder):
    """Delete old data from specified folder"""
    for filename in os.listdir(folder):
        file_path = join(folder, filename)
        try:
            if isfile(file_path) or os.path.islink(file_path):
                if file_path.endswith(".json"):
                    os.unlink(file_path)

        except Exception as e:
            logger.error("Failed to delete %s. Reason: %s" % (file_path, e))


def search_files(
    regression_type, variant, json_path_v2, input_folder_path_v2, test_collection=None
):
    """Search for files with file extension specified in a constant
    and then initialise a dictionary with measurements based on their type
    """
    # Cleaning the evaluation_json folder
    delete_old_data(json_path_v2)
    
    # Make a list with all files found
    file_name_list = os.listdir(input_folder_path_v2)
    file_pathways = []
    paths_to_eval_jsons = []
    # Add file to list only if file extension matches and if
    # there is a .info file also
    for file in file_name_list:
        if file.endswith(FILE_EXTENSION):
            file_temp = file + ".info"
            if file_temp in file_name_list:
                file_pathways.append(join(input_folder_path_v2, file))
            else:
                logger.info(f"File {file} has no .info file")

    if IS_REGRESSION_TEST:

        # Iterate the list and if a file matches with a dictionary key
        # add the file to a list, which is paired with that specific key
        for file in file_pathways:
            for park_type in MEASUREMENTS_BY_PARKING_TYPE:
                if park_type in file:
                    MEASUREMENTS_BY_PARKING_TYPE[park_type].append(file)

        # Iterate over every PARKING type and call method with arguments:
        # (list of files,
        # list of tests,
        # PARKING type)
        # (in the above order)
        for key in MEASUREMENTS_BY_PARKING_TYPE:

            paths = create_json(
                MEASUREMENTS_BY_PARKING_TYPE[key][1:],
                MEASUREMENTS_BY_PARKING_TYPE[key][0],
                key,
                regression_type,
                variant,
            )
            if len(paths) == 1:
                paths_to_eval_jsons.append(paths[0])
            else:
                for path in paths:
                    paths_to_eval_jsons.append(path)
        return paths_to_eval_jsons
    else:

        paths = create_json(file_pathways, test_collection, "", "", variant)
        return paths


def next_index_filename(f):
    """Find out if file exists. If so, make new file with an index at the end"""
    fnew = f
    r, ext = os.path.splitext(f)
    i = 0
    while os.path.exists(fnew):
        i += 1
        fnew = "%s_%i%s" % (r, i, ext)
    return fnew


def create_json(file_list, fun_tests, measurement, regression_type, variant):
    """Creating json using the list of items"""
    statistics_report_name = ""
    paths_to_eval_jsons = []
    # is_small is used with MAX_FILES
    is_small = False
    if measurement == ".erg":
        measurement = "TSF_Report"
    if not eval_json_cfg:
        logger.critical("Error while reading evaluation json configuration file.Aborting evaluation ...")
        reset_logger(logger)
        sys.exit()
    eval_json_cfg["fun"]["tableData"].clear()
    json_fun_cfg = eval_json_cfg["fun"]
    json_general_cfg = eval_json_cfg["general"]
    if len(file_list) <= MAX_FILES:
        # Make subfolders (0,1,2,..)
        is_small = True

    # Counter for folder creation
    subfolder_cnt = 0
    try:
        while file_list:
            # If True -> do not make suboflders
            if IS_REGRESSION_TEST:
                statistics_report_name = "common_ft_helper.MF_SIL_StatisticsReport"
                if is_small:
                    path = join(output_folder_path_v2, time_string, measurement)
                else:
                    path = join(
                        output_folder_path_v2,
                        time_string,
                        measurement,
                        str(subfolder_cnt),
                    )
                    subfolder_cnt += 1
            else:
                statistics_report_name = "common_ft_helper.Non_MF_SIL_StatisticsReport"

                if is_small:
                    path = join(output_folder_path_v2, time_string)
                else:
                    path = join(
                        output_folder_path_v2, time_string, str(subfolder_cnt)
                    )
                    subfolder_cnt += 1
            json_file = {}

            # tabledata will contain each file details
            
            json_fun_cfg["reportPath"] = path
            json_fun_cfg["platform"] = PLATFORM
            json_fun_cfg["project"] = PROJECT
            json_fun_cfg["reportPath"] = path
            json_fun_cfg["selectedPlugins"]["statistics"] = [f"pl_parking.{statistics_report_name}"]
            json_fun_cfg["selectedPlugins"]["statistics"] = [f"pl_parking.{statistics_report_name}"]
            json_fun_cfg["checked"]= [f"pl_parking.statistics.{statistics_report_name}"] 

            if IS_REGRESSION_TEST:
                json_fun_cfg["regressiontype"] = REGRESSION_TYPE_DICT.get(
                    regression_type, ""
                )
                json_fun_cfg["variant"] = variant
                json_fun_cfg["selectedPlugins"]["overviews"] = [
                    "pl_parking.common_ft_helper.BackButtonOverview"
                ]
            else:
                json_fun_cfg["selectedPlugins"]["overviews"] = [
                    "pl_parking.common_ft_helper.BackButtonOverviewNonRegression"
                ]
       
            # Fun is an object that contains all the files to be tested and the necessary settings
            json_file["fun"] = json_fun_cfg

            # General is an object that contains only one setting

            json_file["General"] = json_general_cfg

            # This is used if the list of files is greater than the max_files
            # Use a temporary list to pass a range of files(0,MAX_FILES)
            # then store remaining files in same list to be added in another eval_file json
            temp_list = file_list[:MAX_FILES]
            file_list = file_list[MAX_FILES:]

            # Add files to table_data dict
            for file in temp_list:
                tabe_data = {
                    "bsig": file,
                    "functionalTest": fun_tests,
                    "test_scenario": "",
                }
                json_fun_cfg["tableData"].append(tabe_data)

            # If files exceed MAX_FILES, this will prevent from overwriting

            filename = next_index_filename(rf"{json_path_v2}\eval_{measurement}.json")
            paths_to_eval_jsons.append(filename)
            # Save json file
            with open(filename, "w", encoding="utf-8") as f:
                json.dump(json_file, f, ensure_ascii=False, indent=1)

    except IndexError as e:
        logger.error(f"Error when creating json file. Path {path} , error : {e}")

    return paths_to_eval_jsons


def parseRegressionTestResults(summaryFilePath):
    if not os.path.isfile(summaryFilePath):
        logger.error("Test summary file does not exist: " + summaryFilePath)
        sys.exit(EVALUATION_DROPPED)
    summaryXML = ElementTree.parse(summaryFilePath)
    report2passedPercentage = {}
    for report in summaryXML.getroot():
        try:
            reportName = report.tag
            passedTests = int(report.find("Passed").text.split("=")[0])
            totalTests = int(report.find("Total").text)
            report2passedPercentage.setdefault(
                reportName, passedTests / totalTests * 100.0
            )
        except ZeroDivisionError:
            report2passedPercentage.setdefault(reportName, 0)
    return report2passedPercentage


def parseRegressionTestThresholds(thresholdsFilePath):
    if not os.path.isfile(thresholdsFilePath):
        logger.error("Test thresholds file does not exist: " + thresholdsFilePath)
        sys.exit(EVALUATION_DROPPED)
    thresholdXML = ElementTree.parse(thresholdsFilePath)
    report2Threshold = {}
    for report in thresholdXML.getroot():
        reportName = report.find("Name").text
        threshold = float(report.find("Threshold").text)

        report2Threshold.setdefault(reportName, threshold)
    return report2Threshold


def clean_key(key):
    return (
        key.replace("__", " ").replace("_", " ").replace("AUP MF CORE", "MF AUP CORE")
    )


def chart_result_threshold(time_stamp, regression_type, meas_count, selectedVariant):

    threshold_list = []
    time_stamp = time_stamp.replace("S_", "")
    result_list = []
    plotly_html_strings = (
        r"No thresholds provided. Check Raport_generator\Brics_CI folder."
    )
    threshold_list = []
    common_keys = None
    park_type_test_case = []  # Eg. Parleft MF AUP CORE

    color_dict = {0: "red", 1: "green"}
    if isfile(
        join(
            threshold_xml_path,
            REGRESSION_THRESHOLD_DICT.get(
                f"{regression_type}_{selectedVariant}", "no_file.xml"
            ),
        )
    ) and isfile(join(current_report_path, "Summary_per_KPI.xml")):
        try:
            
            threshold = parseRegressionTestThresholds(
                join(
                    threshold_xml_path,
                    REGRESSION_THRESHOLD_DICT[f"{regression_type}_{selectedVariant}"],
                )
            )
            result_date = parseRegressionTestResults(
                join(current_report_path, "Summary_per_KPI.xml")
            )
            threshold = {clean_key(k): v for k, v in threshold.items() if v is not None}
            result_date = {
                clean_key(k): v for k, v in result_date.items() if v is not None
            }

            nuc_key = ""
            for key in result_date.keys():
                if "nuc" in key.lower():
                    nuc_key = key
                    break
            if nuc_key:
                del result_date[nuc_key]

            # Check for missing data ( Eg. No Parleft teetcases were performed, so add key with 0 value)
            for key, value in threshold.items():
                if key not in result_date:
                    result_date.update({key: 0.0})
            common_keys = set(key for key in threshold.keys()) and set(
                key for key in result_date.keys()
            )
            threshold = dict(sorted(threshold.items()))
            result_date = dict(sorted(result_date.items()))

        except Exception as err:
            
            logger.error(str(err))
    else:
        logger.error("Graph plots creation failed.")
        logger.error("No threshold and/or Summary_per_KPI.xml files provided.")

    plotly_html_strings = ""
    if common_keys:

        if input_folder_v2 in [MID_REG,SMALL_REG]:

            parking_categories = CATEGORIES_FOR_XML_CHARTS[
                f"{regression_type}_{selectedVariant}"
            ]
            extracted_values = {}
            referinta = {}
            for i in parking_categories:
                for keie in result_date.keys():
                    if i in keie:
                        if i not in extracted_values:
                            extracted_values[i] = {}
                        extracted_values[i][keie] = result_date[keie]
            for i in parking_categories:
                for keie in threshold.keys():
                    if i in keie:
                        if i not in referinta:
                            referinta[i] = {}
                        referinta[i][keie] = threshold[keie]
            extr_keys = list(extracted_values.keys())
            found_report_paths = find_additional_info("mf_sil_graph_links.txt")
            test_case_links = []
            for f in found_report_paths:
                infile = open(f, "r")
                for line in infile:
                    a = line.replace("[", "").replace("]", "").replace("'", "")
                    test_case_links.append(a)
                infile.close()

            # version with 2 subplots, suitable for Mid and Small Regression
            ###########################
            test_collection_links = {
                item: [link for link in test_case_links if item in link]
                for item in extr_keys
            }
            for item in test_case_links:
                if "nuc" in item.lower():
                    new_key = item.split("/")[0]
                    test_collection_links.update({new_key: [item]})
                if 'aupsim_uc_parright' in item.lower():
                    key_to_update =  'ParRight'
                    test_collection_links.update({key_to_update: [item]})
            plotly_html_strings += "<h1>Report links</h1>"
            plotly_html_strings += "<br>"
            for name, link in test_collection_links.items():
                if link:
                    plotly_html_strings += (
                        f'<h5><a href="../{link[0]}/index.html">{name}</a></h5>'
                    )
            for i in range(0, len(extr_keys), 2):
                # Create subplot with 1 row and 2 columns
                fig = make_subplots(
                    rows=1,
                    cols=2,
                    subplot_titles=[f"{extr_keys[i]}", f"{extr_keys[i + 1]}"],
                )

                # Iterate through the pairs of extr_keys
                for j, extr_key in enumerate([extr_keys[i], extr_keys[i + 1]], start=1):
                    result_list = []
                    park_type_test_case = []
                    threshold_list = []
                    # not_assessed_list= []
                    for key, val in extracted_values[extr_key].items():
                        result_list.append(val)
                        park_type_test_case.append(key)

                    for key, val in referinta[extr_key].items():
                        threshold_list.append(val)

                    measured_values_color = [
                        color_dict[x >= y] for x, y in zip(result_list, threshold_list)
                    ]

                    threshold_color = ["blue"] * len(threshold_list)

                    # Add both bars to the subplot
                    fig.add_trace(
                        go.Bar(
                            x=park_type_test_case,
                            y=result_list,
                            text=[f"{x:.2f}" for x in result_list],
                            name="Pass rate",
                            marker_color=measured_values_color,
                        ),
                        row=1,
                        col=j,
                    )

                    fig.add_trace(
                        go.Bar(
                            x=park_type_test_case,
                            y=threshold_list,
                            text=[f"{x:.2f}" for x in threshold_list],
                            name="Threshold",
                            marker_color=threshold_color,
                        ),
                        row=1,
                        col=j,
                    )

                    fig.update_layout(
                        showlegend=False,
                        height=500,

                    )

                # Append the HTML string to the result
                plotly_html_strings += (
                    fig.to_html()
                )  # full_html=False, include_plotlyjs=False)
        else:
            found_report_paths = find_additional_info("mf_sil_graph_links.txt")
            test_case_links = []
            for f in found_report_paths:
                infile = open(f, "r")
                for line in infile:
                    a = line.replace("[", "").replace("]", "").replace("'", "")
                    test_case_links.append(a)
                infile.close()
            test_collection_links = {
                'TSF Raport': [link for link in test_case_links ]
            
            }

            plotly_html_strings += "<h1>Report links</h1>"
            plotly_html_strings += "<br>"
            for name, link in test_collection_links.items():
                if link:
                    plotly_html_strings += (
                        f'<h5><a href="../{link[0]}/index.html">{name}</a></h5>'
                    )
            extracted_values = {}
            referinta = {}

            for test in result_date.keys():
                if test not in extracted_values:
                    extracted_values[test] = {}
                extracted_values[test] = result_date[test]
            
            for test in threshold.keys():
                if test not in referinta:
                    referinta[test] = {}
                referinta[test] = threshold[test]
            
            # Version for separate plots, suitable for generic simulations
            ########################
            result_list = []
            park_type_test_case = []
            threshold_list = []
            for extr_key in extracted_values.keys():
                fig = go.Figure()
                result_list.clear()
                park_type_test_case.clear()
                threshold_list.clear()
                result_list.append(extracted_values[extr_key])
                park_type_test_case.append(extr_key)
                threshold_list.append(referinta[extr_key])

                valori_culoare = ['green']* 6
                valori_culoare = [color_dict[x>=y] for x,y in zip(result_list,threshold_list)]
                threshold_culoare = ['blue'] * 6
                fig.add_trace(go.Bar(x=park_type_test_case, y=result_list, name=f'Pass rate', text =[f'{x:.2f}' for x in result_list],marker_color=valori_culoare))
                fig.add_trace(go.Bar(x=park_type_test_case, y=threshold_list, name=f'Threshold', text =[f'{x:.2f}' for x in threshold_list],marker_color=threshold_culoare))

                fig.update_layout(title=f'{extr_key}', height=500,showlegend=True)

                plotly_html_strings += fig.to_html()

    root = os.path.dirname(__file__)
    templaete_folder = os.path.abspath(
        os.path.join(root, "Evaluation_raport_scripts", "templates")
    )

    jinja_env = jinja2.Environment(loader=jinja2.FileSystemLoader(templaete_folder))
    template = jinja_env.get_template("graph_page.html")
    html = template.render(block_content=plotly_html_strings)

    with open(
        os.path.join(current_report_path, "static", "graphs.html"),
        "w",
        encoding="utf-8",
    ) as f:
        f.write(html)

def find_additional_info(file_name: str):
    """Search for all mf sil txts in the Reports path"""
    additional_info = []

    # Use os.walk to go into subfolders
    for folder, subfolders, files in os.walk(current_report_path):
        if folder != current_report_path:
            for file in files:
                if file.endswith(file_name):
                    additional_info.append(join(folder, file))

    return additional_info

def remove_duplicate_lists(lst):
    unique_lists = []
    seen = set()
    for sublist in lst:
        # Convert the sublist to a tuple to make it hashable
        sublist_tuple = tuple(sublist)
        if sublist_tuple not in seen:
            unique_lists.append(sublist)
            seen.add(sublist_tuple)
    return unique_lists

def build_common_report():
    ceva = defaultdict(dict)

    file_paths = find_additional_info("mf_sil_data.txt")

    header_paths = find_additional_info("mf_sil_data_headers.txt")

    header_list = []
    temp_header_list = []
    last_item = []
    for f in header_paths:

        mainlist = []
        infile = open(f, "r")
        for line in infile:

            a = line.replace("[", "").replace("]", "").replace("'", "")
            a = a.replace(" ", "").replace("MF", "MF ").replace("AUP", "AUP ")

            mainlist.append(a.strip().split(","))

        infile.close()
        for tests in mainlist:
            temp_header_list.append(tests)

    temp_header_list = remove_duplicate_lists(temp_header_list)

    for nested_list in temp_header_list:
        for item in nested_list:
            header_list.append(item)
    for file in file_paths:

        with open(file) as f:
            # this data would be a dicitonay which can be easily managed.
            dataz = json.load(f)

        for step_key, step_files in dataz.items():
            ceva[step_key].update(step_files)
    html_new_str = ""
    html_new_str_nuc = ""
    if ceva.get("NUC", False):
        header_list = [x for x in header_list if "NUC" not in x]
        nuc_dict = {"NUC": ceva["NUC"]}
        s = pd.DataFrame(nuc_dict)
        del ceva["NUC"]
        total_meas = 0

        try:
            import numpy as np

            for i, j in s.iterrows():
                total_meas += 1
                pass_counter = 0
                for col_name, item in j.items():  # Iterate over items in each row
                    if not isinstance(item, str) and np.isnan(item):
                        # Replace NaN values with a custom string
                        s.at[i, col_name] = "n/a"
                        continue
                    if "passed" in item:
                        pass_counter += 1
                if pass_counter == len(j):
                    s.drop(i, inplace=True)

        except Exception as err:

            logger.error(str(err))
        if not s.empty:

            s.columns = pd.MultiIndex.from_tuples(zip(["NUC"], s.columns))

            pd.set_option("display.max_colwidth", None)
            html = s.to_html(classes="table table-hover", escape=False, col_space=100)


            html = html.split("\n")
            html_new = []

            for i in range(len(html)):
                if "<td>" not in html[i]:
                    html_new.append(html[i])

                elif "<td>" in html[i]:
                    bg_col = (html[i].split(";background-color: ")[-1]).split(
                        "; color:"
                    )[0]

                    html_new.append(html[i].replace("<td>", f"<td bgcolor={bg_col}>"))
            html_new_str_nuc = "\n".join(html_new)
            html_new_str_nuc = (
                html_new_str_nuc.replace(
                    "<thead>", '<thead class= "sticky-top" id = "myHeader">'
                )
                .replace(
                    '<th style="min-width: 100px;"',
                    '<th halign="left"style="text-align:center; min-width: 100px;"',
                )
                .replace(
                    'halign="left"style="min-width: 100px;"',
                    'halign="left"style="text-align:center; min-width: 100px;"',
                )
            )

        else:
            html_new_str_nuc = "<h4>No failed NUC measurements found</h4>"

    s = pd.DataFrame(ceva)
    total_meas = 0

    try:
        import numpy as np

        for i, j in s.iterrows():
            total_meas += 1
            pass_counter = 0
            for col_name, item in j.items():  # Iterate over items in each row
                if not isinstance(item, str) and np.isnan(item):
                    # Replace NaN values with a custom string
                    s.at[i, col_name] = "n/a"
                    continue
                if "passed" in item:
                    pass_counter += 1
            if pass_counter == len(j):
                s.drop(i, inplace=True)

    except Exception as err:
        logger.error(str(err))
    if not s.empty:

        s.columns = pd.MultiIndex.from_tuples(zip(header_list, s.columns))

        pd.set_option("display.max_colwidth", None)
        html = s.to_html(
            classes="table table-hover height:500px", escape=False, col_space=100
        )

        html = html.split("\n")
        html_new = []

        for i in range(len(html)):
            if "<td>" not in html[i]:
                html_new.append(html[i])

            elif "<td>" in html[i]:
                bg_col = (html[i].split(";background-color: ")[-1]).split("; color:")[0]

                html_new.append(html[i].replace("<td>", f"<td bgcolor={bg_col}>"))
        html_new_str = "\n".join(html_new)
        html_new_str = (
            html_new_str.replace(
                "<thead>", '<thead class= "sticky-top" id = "myHeader">'
            )
            .replace(
                'halign="left">',
                'halign="left"style="text-align:center; min-width: 100px;">',
            )
            .replace(
                'halign="left"style="min-width: 100px;"',
                'halign="left"style="text-align:center; min-width: 100px;"',
            )
        )
    else:
        html_new_str = "<h4>No failed measurements found</h4>"
    root = os.path.dirname(__file__)
    templaete_folder = os.path.abspath(
        os.path.join(root, "Evaluation_raport_scripts", "templates")
    )

    jinja_env = jinja2.Environment(loader=jinja2.FileSystemLoader(templaete_folder))
    template = jinja_env.get_template("failed_meas_page.html")
    html = template.render(
        block_content=html_new_str, nuc_block_content=html_new_str_nuc
    )

    with open(
        os.path.join(current_report_path, "static", "failed_meas.html"), "w"
    ) as file:
        file.write(html)
    return total_meas


def build_overview_page(time_stamp, regression_type, meas_count, selectedVariant):

    time_stamp = time_stamp.replace("S_", "")

    commit_id = ""
    branch_name = ""

    # Get commit id and branch name
    try:
        commit_id_result = subprocess.run(
            ["git", "log", "--pretty=format:'%H'", "-n1", "."],
            capture_output=True,
            text=True,
            check=True,
        )
        commit_id = commit_id_result.stdout.replace("'", "").replace('"', "")

        branch_name_result = subprocess.run(
            ["git", "branch", "--contains", commit_id],
            capture_output=True,
            text=True,
            check=True,
        )
        branch_name = branch_name_result.stdout.strip().replace("*", "")

    except subprocess.CalledProcessError as e:
        # If the git log command fails, you can handle the exception here
        logger.error(f"Error running 'git log': {e.stderr}")

    overview_df = pd.DataFrame(
        {
            "1": {
                "1": "Date",
                "2": "Project",
                "3": "Commit ID",
                "4": "Branch name",
                "5": "Regression type",
                "6": "Variant",
                "7": "Measurements evaluated",
            },
            "2": {
                "1": time_stamp,
                "2": PROJECT,
                "3": commit_id,
                "4": branch_name,
                "5": REGRESSION_TYPE_DICT.get(regression_type, "Not provided"),
                "6": selectedVariant.title(),
                "7": meas_count,
            },
        }
    )

    overview_html = overview_df.to_html(
        classes="table table-hover height:500px",
        header=False,
        justify="center",
        border=0,
        index=False,
        table_id="my_table",
        max_cols=2,
        escape=False,
    )

    root = os.path.dirname(__file__)
    templaete_folder = os.path.abspath(
        os.path.join(root, "Evaluation_raport_scripts", "templates")
    )

    jinja_env = jinja2.Environment(loader=jinja2.FileSystemLoader(templaete_folder))
    template = jinja_env.get_template("overview_page.html")
    html = template.render(block_content=overview_html)

    with open(
        os.path.join(current_report_path, "Overview.html"), "w", encoding="utf-8"
    ) as f:
        f.write(html)

    webbrowser.open("file://" + os.path.join(current_report_path, "Overview.html"))

def load_configuration():

    # Modify global variables according to the desired testrun configuration
    global default_cfg
    global eval_json_cfg
    global testrun_cfg
    global MAX_FILES
    global IS_REGRESSION_TEST
    global FILE_EXTENSION
    global PLATFORM
    global PROJECT
    global CATEGORIES_FOR_XML_CHARTS

    global MAX_FILES

    global REGRESSION_THRESHOLD_DICT
    try:
        if input_folder_v2 == SMALL_REG:
            with open(os.path.join(sp,*default_cfg["SMALL_REG_CFG"] ), 'r') as file:
                testrun_cfg = yaml.safe_load(file)
        elif input_folder_v2 == MID_REG:
            with open(os.path.join(sp,*default_cfg["MID_REG_CFG"] ), 'r') as file:
                testrun_cfg = yaml.safe_load(file)
        elif input_folder_v2 == MP_REG:
            with open(os.path.join(sp,*default_cfg["MP_REG_CFG"] ), 'r') as file:
                testrun_cfg = yaml.safe_load(file)
        else:
            raise MissingCfgFile("Configuration file not provided for this testrun!")
        with open(os.path.join(sp,*default_cfg["JSON_EVAL_CFG"] ), 'r') as file:
            eval_json_cfg = yaml.safe_load(file)
        
        MAX_FILES = testrun_cfg['MAX_FILES']
        IS_REGRESSION_TEST = testrun_cfg['IS_REGRESSION_TEST']
        FILE_EXTENSION = testrun_cfg['FILE_EXTENSION']
        PLATFORM = testrun_cfg['PLATFORM']
        PROJECT = testrun_cfg['PROJECT']
        REGRESSION_THRESHOLD_DICT = testrun_cfg["THRESHOLD"]
        CATEGORIES_FOR_XML_CHARTS = testrun_cfg["CATEGORIES_FOR_XML_CHARTS"]

    except MissingCfgFile as error:
        logger.critical('Invalid configuration provided! Aborting evaluation...')
        logger.critical(f'Error :{str(error)}')
        reset_logger(logger)
        sys.exit()
    except KeyError as error:
        logger.critical('Invalid configuration provided! Aborting evaluation...')
        logger.critical(f'Can not find key {str(error)} in cfg file')
        reset_logger(logger)
        sys.exit()
    except FileNotFoundError as error:
        logger.critical('Invalid configuration provided! Aborting evaluation...')
        logger.critical(f'{str(error)}')
        reset_logger(logger)
        sys.exit()
    except Exception as error:
        logger.critical('Invalid configuration provided! Aborting evaluation...')
        logger.critical(f'Error :{str(error)}')
        reset_logger(logger)
        sys.exit()
    

def load_default_settings():
    global REGRESSION_TYPE_DICT
    global SMALL_REG
    global MID_REG
    global MP_REG
    try:
        SMALL_REG = default_cfg['SMALL_REG']
        MID_REG = default_cfg['MID_REG']
        MP_REG = default_cfg['MP_REG']
        REGRESSION_TYPE_DICT = default_cfg['REGRESSION_TYPE_DICT']
    except MissingCfgFile as error:
        logger.critical('Invalid configuration provided! Aborting evaluation...')
        logger.critical(f'Error :{str(error)}')
        reset_logger(logger)
        sys.exit()
    except KeyError as error:
        logger.critical('Invalid configuration provided! Aborting evaluation...')
        logger.critical(f'Can not find key {str(error)} in cfg file')
        reset_logger(logger)
        sys.exit()
    except FileNotFoundError as error:
        logger.critical('Invalid configuration provided! Aborting evaluation...')
        logger.critical(f'{str(error)}')
        reset_logger(logger)
        sys.exit()
    except Exception as error:
        logger.critical('Invalid configuration provided! Aborting evaluation...')
        logger.critical(f'Error :{str(error)}')
        reset_logger(logger)
        sys.exit()    

if __name__ == "__main__":

    testrun_cfg : yaml = None
    default_cfg : yaml = None
    eval_json_cfg : yaml = None
    sp = os.path.dirname(__file__)  # script path
    
    # Use default settings to setup paths and constants
    with open(os.path.join(sp,"Evaluation_raport_scripts","default_settings.yml" ), 'r') as file:
        default_cfg = yaml.safe_load(file)

    MAX_FILES = None
    IS_REGRESSION_TEST = None
    FILE_EXTENSION = None
    PLATFORM = None
    PROJECT = None
    REGRESSION_TYPE_DICT = None
    test_collection = None
    CATEGORIES_FOR_XML_CHARTS = None
    REGRESSION_THRESHOLD_DICT = None

    load_default_settings()

    input_folder_v2 = sys.argv[2]
    output_folder_v2 = sys.argv[3]

    # Check build variant
    selectedVariant = ""
    PATH_selectedVariant = os.path.abspath(
        os.path.join(sp, *default_cfg["mf_sil_selected_variant"]))

    # Check, if selected_variant.txt file exists. In case file is not available, use base.
    if os.path.isfile(PATH_selectedVariant):
        variant = open(PATH_selectedVariant, "r")
        selectedVariant = variant.readline().strip().lower()
        variant.close()
    else:
        selectedVariant = "base"

    # Define paths
    named_tuple = time.localtime()
    time_string = time.strftime("S_20%y-%m-%d_%H-%M", named_tuple)

    json_path_v2 = os.path.abspath(os.path.join(sp, default_cfg["eval_jsons_folder"]))
    threshold_xml_path = os.path.abspath(os.path.join(sp, default_cfg["brics_folder"]))
    tsf_runner_path_v2 = os.path.abspath(
        os.path.join(sp,*default_cfg["tsf_runner_path"]))
    venv_scan_script_path = os.path.abspath(
        os.path.join(sp,*default_cfg["venv_scan_script"]))
    workspace_path = os.path.abspath(
        os.path.join(sp, *default_cfg["workspace_path"]))
    input_folder_path_v2 = os.path.abspath(
        os.path.join(sp, *default_cfg['input_folder'] ,input_folder_v2))
    output_folder_path_v2 = os.path.abspath(
        os.path.join(sp, *default_cfg['output_folder'], output_folder_v2))
    last_report_folder_path_v2 = os.path.abspath(
        os.path.join(sp, *default_cfg['output_folder'], output_folder_v2, "LastReport"))
    current_report_path = os.path.abspath(
        os.path.join(output_folder_path_v2, time_string))
    tsf_python_path_v2 = os.path.abspath(
        os.path.join(sp,*default_cfg['tsf_python']))
    
    check_directories([input_folder_path_v2,output_folder_path_v2,
                json_path_v2,last_report_folder_path_v2,current_report_path])

    logging.basicConfig(
    level=logging.INFO,
    format='[REPORT GENERATOR] %(asctime)s - [%(levelname)s] - %(message)s',
    handlers=[
        logging.FileHandler(os.path.join(output_folder_path_v2, time_string,f'{time_string}_log.log')), # output to file
        logging.StreamHandler() # output to console
    ]
)
    logger = logging.getLogger()
    load_configuration()
    
    if "regresession" in sys.argv[1].lower():

        if input_folder_v2 == MID_REG:

            MEASUREMENTS_BY_PARKING_TYPE = {
            "AUPSim_NUC_ParRight": [testrun_cfg["NUC_Tests"]],
            "AUPSim_UC_ParLeft": [testrun_cfg[f"TestCollection_{selectedVariant}"]["parLeft"] ],
            "AUPSim_UC_PerpLeft": [testrun_cfg[f"TestCollection_{selectedVariant}"]["perpLeft"] ],
            "AUPSim_UC_ParRight": [testrun_cfg[f"TestCollection_{selectedVariant}"]["parRight"] ],
            "AUPSim_UC_PerpRight": [testrun_cfg[f"TestCollection_{selectedVariant}"]["perpRight"] ],
            "AUPSim_UC_AngLeft": [testrun_cfg[f"TestCollection_{selectedVariant}"]["angLeft"]  ],
            "AUPSim_UC_AngRight": [testrun_cfg[f"TestCollection_{selectedVariant}"]["angRight"] ],
        }
        elif input_folder_v2 == SMALL_REG:
            MEASUREMENTS_BY_PARKING_TYPE = {
            "AUPSim_NUC_ParRight": [testrun_cfg["NUC_Tests"]],
            "AUPSim_UC_ParLeft": [testrun_cfg[f"TestCollection_{selectedVariant}"]["parLeft"] ],
            "AUPSim_UC_PerpLeft": [testrun_cfg[f"TestCollection_{selectedVariant}"]["perpLeft"] ],
            "AUPSim_UC_ParRight": [testrun_cfg[f"TestCollection_{selectedVariant}"]["parRight"] ],
            "AUPSim_UC_PerpRight": [testrun_cfg[f"TestCollection_{selectedVariant}"]["perpRight"] ],
            "AUPSim_UC_AngLeft": [testrun_cfg[f"TestCollection_{selectedVariant}"]["angLeft"]  ],
            "AUPSim_UC_AngRight": [testrun_cfg[f"TestCollection_{selectedVariant}"]["angRight"] ],
        }

        elif input_folder_v2 == MP_REG:
            test_collection= testrun_cfg[f"TestCollection_{selectedVariant}"]

        eval_jsons = search_files(
                output_folder_v2,
                selectedVariant.title(),
                json_path_v2,
                input_folder_path_v2,
                test_collection=test_collection
            )


        if eval_jsons:

            for path in eval_jsons:
                err_list = []
                try:
                    command = " ".join(
                        [
                            tsf_python_path_v2,
                            tsf_runner_path_v2,
                            path,
                            "--check-inputs",
                            "--clean-dir",
                            "--sqlite",
                            json_path_v2,
                            "-r",
                        ]
                    )
                    logger.info(f"Evaluating {os.path.basename(path)}")
                
                    result = subprocess.run(
                        command,
                        shell=True,
                        check=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        universal_newlines=True,
                    )
                
                    logger.info(result.stdout)


                except subprocess.CalledProcessError as e:
                    logger.error(f'Error :{str(e.stderr)}')

            t0 = time.time()
            text_to_search = "functional_test_results.json"
            # Find all functional test reports and return a list
            json_file_list = find_jsons(current_report_path, text_to_search)
            
            if json_file_list:

                # Unpack each json and then build xmls
                unpack_json(json_file_list)
                try:
                    logger.info(f'Creating overview report ...')
                    # Copy the last report in the Last Report folder
                    copy_tree(
                        os.path.join(
                            sp, "Evaluation_raport_scripts", "static"
                        ),
                        os.path.join(current_report_path, "static"),
                    )
                    meas_count = build_common_report()
                    chart_result_threshold(
                        time_string, output_folder_v2, meas_count, selectedVariant
                    )
                    build_overview_page(
                        time_string, output_folder_v2, meas_count, selectedVariant
                    )
                except Exception as e:
                    logger.error(f'****************************************************')
                    logger.error(f'Overall report creation failed!')
                    logger.error(str(e))
                    logger.error(f'****************************************************')
                    exc_type, exc_obj, exc_tb = sys.exc_info()
                    fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                    logger.error(f"{exc_type}, {fname}, {exc_tb.tb_lineno}")
                logger.info("Moving files to Last Report ...")
                
                copy_tree(current_report_path, last_report_folder_path_v2)
                # Get time after executing all scripts and calculate time passed
                t1 = time.time()
                total = t1 - t0
                logger.info(f"Total time elapsed for xml generator: {total:.5f} seconds")
            else:
                logger.error("\n")
                logger.error("* No functional_test_results.json found")
                logger.error(f"* Possible cause 1: no erg files have been generated, check input folder {input_folder_path_v2}")
                logger.error(f"* Possible cause 2: no report generated, check results folder {current_report_path}")
                logger.error("\n")
        else:
            logger.error(f"* No measurements found in {input_folder_path_v2}")