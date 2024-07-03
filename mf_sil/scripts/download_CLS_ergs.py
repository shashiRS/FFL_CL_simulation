'''
This is a script for downloading erg result files from the CLS/CAEdge platform
Author Andrei Ghiran - (uic65783)

Command line call example: 
python download_CLS_ergs.py <campaign_results_id> <folder_name>

Parameters:
campaign_results_id: The simulation campaign result ID found on the "Campaign Results" section in the platform UI
folder_name: name of the folder in SimOutput where the erg files should be placed

'''

import os
import requests
import sys
from zipfile import ZipFile

results_bd_url = "https://conti.prk.prod.pl-parking.aws-global.continental.cloud/result-db"
simoutput_folder_path = os.path.join("..","tests","SIL","CarMaker","SimOutput")


def extract_files(folder):
    print("-----------Extracting ergs to SimOutput folder-----------")
    for zip in os.listdir(os.path.join(".","temp")):
        with ZipFile(os.path.join(".","temp",zip), "r") as zip_file:
            zip_file.extractall(os.path.join(simoutput_folder_path,folder))


def download_ergs(campaign_results_id, folder):
    print("-----------Downloading zips to temp folder-----------")
    response = requests.get(results_bd_url+f"/api/campaign/{campaign_results_id}")
    testrun_result_urls = response.json()["result_urls"]

    os.mkdir(os.path.join(".","temp"))

    for test_url in testrun_result_urls:
        id = test_url.split("/")[-1]
        zip_name = requests.get(test_url).json()["scenario"]["scenario_url"].split("/")[-1]
        file = requests.get(results_bd_url+"/api/result/"+id+"/cm-erg")

        new_file = open(os.path.join(".","temp",zip_name)+".zip","wb")
        new_file.write(file.content)
        new_file.close()

    extract_files(folder)

    print("-----------Removing temp files-----------")
    for zip in os.listdir(os.path.join(".","temp")):
        os.remove(os.path.join(".","temp",zip))

    os.rmdir(os.path.join(".","temp"))


def rename_ergs(campaign_results_id, folder):
    print("-----------Renaming erg files-----------")
    test_results_map = {}
    response = requests.get(results_bd_url+f"/api/campaign/{campaign_results_id}")
    testrun_result_urls = response.json()["result_urls"]

    for test_url in testrun_result_urls:
        response = requests.get(test_url)
        scenario_name = response.json()["scenario"]["name"]
        id = response.json()["scenario"]["scenario_url"].split("/")[-1]
        test_results_map[id] = scenario_name

    for erg in os.listdir(os.path.join(simoutput_folder_path, folder)):
        new_name = test_results_map[erg.split(".")[0]]
        for bit in erg.split('.')[1:]:
            new_name += "." + bit
        os.rename(os.path.join(simoutput_folder_path,folder,erg),os.path.join(simoutput_folder_path,folder,new_name))


if __name__ == '__main__':
    print("-----------START-----------")
    campaign_results_id = sys.argv[1]
    folder_name = sys.argv[2]
    download_ergs(campaign_results_id,folder_name)
    rename_ergs(campaign_results_id, folder_name)
    print("-----------DONE-----------")

