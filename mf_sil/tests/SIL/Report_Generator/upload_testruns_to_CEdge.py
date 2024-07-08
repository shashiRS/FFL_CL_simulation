"""
Script for uploading tesruns to CAEdge platform.
The test lists from eval_AUP_testcases.py are used to identify witch testruns belong to entry, performance, small and mid regression catalogues.
Already existing testruns and catalogues on the platform are deleted to prevent uploading testruns twice and removing unused testruns.
The script uploads tesrtuns and creates test catalogues for entry/performance small/mid regression tests

Author: Andrei Ghiran (uic65783)
"""

import os
import requests
import json
import imp

scenario_service_url = "https://conti.prk.prod.pl-parking.aws-global.continental.cloud/scenario-db"

EVALPATH = os.path.dirname(__file__)
PATH_selectedVariant = os.path.abspath(os.path.join(EVALPATH,'..', '..','..', 'scripts', 'selected_variant.txt'))

file = open(PATH_selectedVariant,"r")
original_variant = file.read().strip()
file.close()

file = open(PATH_selectedVariant,"w")
file.write("entry")
file.close()

import eval_AUP_testcases
# Geting entry regression lists from eval_AUP_testcases
Entry_Small_Reg_Folders = eval_AUP_testcases.TestCaseFolders_SmallRegres
Entry_Mid_Reg_Folders = eval_AUP_testcases.TestCaseFolders_Regression

file = open(PATH_selectedVariant,"w")
file.write("performance")
file.close()
imp.reload(eval_AUP_testcases)
# Geting performance regression lists from eval_AUP_testcases
Perf_Small_Reg_Folders = eval_AUP_testcases.TestCaseFolders_SmallRegres
Perf_Mid_Reg_Folders = eval_AUP_testcases.TestCaseFolders_Regression

file = open(PATH_selectedVariant,"w")
file.write(original_variant)
file.close()

TESTRUNS_DIRS_PATH = eval_AUP_testcases.TestCaseFolder_Regression

def delete_all_test_catalogues():
    headers = {
        'accept': '*/*',
    } 
    response = requests.get(scenario_service_url+"/api/catalogue")
    for catalogue in response.json()["catalogues"]:
        catalogue_id = catalogue["url"].split('/')[-1]
        delete_responce = requests.delete(scenario_service_url+"/api/catalogue/"+catalogue_id, headers=headers)

def delete_all_test_runs():
    headers = {
        'accept': '*/*',
    } 
    response = requests.get(scenario_service_url+"/api/scenario")
    for scenario in response.json()["scenarios"]:
        scenario_id = scenario["url"].split('/')[-1]
        delete_responce = requests.delete(scenario_service_url+"/api/scenario/"+scenario_id, headers=headers)

def upload_testruns(testrun_folders):
    headers = {
        'accept': '*/*',
    } 
    for dir in testrun_folders:
        path = os.path.join(TESTRUNS_DIRS_PATH,dir)
        for file in os.listdir(path):
            properties = f"folder={dir};"
            if (dir in Perf_Small_Reg_Folders):
                properties += f"test_catalogue=SmallRegressionPerformance;"
            if (dir in Entry_Small_Reg_Folders):
                properties += f"test_catalogue=SmallRegressionEntry;"
            if (dir in Perf_Mid_Reg_Folders):
                properties += f"test_catalogue=MidRegressionPerformance;"
            if (dir in Entry_Mid_Reg_Folders):
                properties += f"test_catalogue=MidRegressionEntry;"

            testrun_file = open(os.path.join(path,file), "rb")
            data = {
                "carmaker_version": "8.1",
                "creator": "uic65783",
                "description": f"AUP regression testrun",
                "maturity": "DRAFT",
                "name": file,
                "properties": properties,
                "vehicle": 'VW_PassatVariant_20TSI_DSG_LLVW_AP_C_GenSteerAng.car'
            }
            response = requests.post(scenario_service_url+"/api/scenario", headers=headers, data=data, files={"scenario_file":testrun_file})
            if not response:
                print(response.text)

def create_test_catalogues():
    for size in ["small", "mid"]:
        for variant in ["entry", "performance"]:
            response = requests.get(scenario_service_url+"/api/scenario", params={"properties":f"test_catalogue={size.capitalize()}Regression{variant.capitalize()};"})
            scenarios = response.json()["scenarios"]
            scenario_list = []
            for scenario in scenarios:
                scenario_list.append(scenario["url"])

            headers = {
                'accept': '*/*',
                'Content-Type': 'application/json'
            }

            data = {
                "carmaker_version":"8.1",
                "creator":"uic65783",
                "description":variant.capitalize()+" "+size.capitalize()+" Regression test catalogue",
                "maturity": "DRAFT",
                "name": f"{variant.capitalize()} {size.capitalize()} Regression",
                "scenario_urls": scenario_list,
                "short_description": f"{size.capitalize()}_Regression_{variant.capitalize()}"
            } 

            response = requests.post(scenario_service_url+"/api/catalogue", headers=headers, data=json.dumps(data))

            if response:
                print(f"Catalogue {variant.capitalize()} {size.capitalize()} Regression, successfully created at "+response.headers["location"])
            else:
                print(response.text) 

if __name__ == '__main__':
    all_testrun_dirs = list(set(Entry_Mid_Reg_Folders+Entry_Small_Reg_Folders+Perf_Mid_Reg_Folders+Perf_Small_Reg_Folders))
    delete_all_test_catalogues()
    delete_all_test_runs()
    upload_testruns(all_testrun_dirs)
    create_test_catalogues()


