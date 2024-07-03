import requests
import datetime
import sys
import urllib3
urllib3.disable_warnings()


# This python file is designed for to trigger FTP through jenkins
# imp note remove all print statements in jenkins
# This function taking input container name
# containername has been generated automatically by taking build number+branch name as arguments

# creating container  naming and adding metadata for container

container_service_url = "https://conti.prk.prod.pl-parking.aws-global.continental.cloud/container-db"

def create_container(tag_name="default.6.0.0-fallback"):
    headers = {
        'accept': '*/*',
    } 
    build_ver = tag_name.split(".", 1)[-1]
    project_name = "mf_sil"
    container_name = project_name + " " + tag_name.split(".", 1)[0]
    print("container name: " + container_name)
    print("build_version: " + build_ver)
    data = {
        "build_time": datetime.datetime.now(),
        "build_version": build_ver,
        "carmaker_version": "8.1.1",
        "description": "mf_sil container uploaded by Jenkins Release Pipeline",
        "name": container_name,
        "vehicle_list": [
            'VW_PassatVariant_20TSI_DSG_LLVW_AP_C_GenSteerAng.car'
        ],
        "creator": "Jenkins",
        "maturity": "DRAFT",
        "operating_system": "WINDOWS",
        "container_structure_version": "0.2",
    }
    
    response = requests.post(container_service_url + '/api/container', headers=headers, data=data)
    if (response):
        container_location = response.headers['location']
        container_id = container_location.split('/')[-1]
    else:
        print(response.text)

    print("Container Location: " + container_location)
    print("Container ID: " + container_id)
    if container_binaries_upload(container_id, tag_name) == 1:
        print("Container Upload Successfull.")
    else:
        print("Container binaries upload failed")


# uploading binaries which generated in folder(package.SCPREM.container.2.0.0-fallback.vs2017.zip) into container
def container_binaries_upload(container_id, tag_name):
    headers = {
        'accept': '*/*',
    }
    package_path = "./CLS_containers/package.mf_sil.container." + tag_name + ".vs2017.zip"

    file = open(package_path, 'rb')
    
    upload_url = requests.post(container_service_url + '/api/container/' + container_id + '/uploadurl',
                              headers=headers,
                              )
    
    response = requests.put(upload_url.json()["presigned_url"], data=file)

    if response:
        print(
            'Container updated with zip file at location ' + container_service_url + '/api/container/' + container_id + '/archive')
        return 1
    else:
        print(response.text)
        return 0


if __name__ == '__main__':
    create_container(sys.argv[1])