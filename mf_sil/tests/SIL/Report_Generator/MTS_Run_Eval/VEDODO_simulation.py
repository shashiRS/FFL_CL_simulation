

# Create a bpl file for every rrec file found in the given folder

#---------------------------------------------------------------------------------------------------------
# Python imports 

import os
from shutil import copyfile
import stat
import argparse

#---------------------------------------------------------------------------------------------------------

# Set full path to the input folder ( where recordings are located )

parser = argparse.ArgumentParser()
parser.add_argument("--rec_path", help="Path to NCAP recordings", required=True)
parser.add_argument("--mts_config", help="Path to MTS Configuration", required=True)
parser.add_argument("--config_name", help="Path to MTS Configuration", required=True)
args = parser.parse_args()

#print vars(args)["rec_path"]
Input_folder = args.rec_path #vars(args)["rec_path"]
MTS_configuration_path = args.mts_config
Config_name = args.config_name

#---------------------------------------------------------------------------------------------------------

# Create MTS structure and bpl files in MTS configuration folder

Current_folder = os.getcwd()

# Path to configuration file what will be used
src_cfg_file = Config_name

os.chdir(MTS_configuration_path)
#-------------------------------------------------------------------------------------------------------------------------

# Modify mts.ini file in mts_system in order to allow multiple instances of MTS

src = os.path.join(os.getcwd(), "mts", "mts.ini")
dst = src

mts_ini_file = open(src,'r')
filedata = mts_ini_file.read()
mts_ini_file.close()

if "AllowMultipleInstances" not in filedata:
    newdata = filedata.replace("[System]","[System]\nAllowMultipleInstances=1")
elif "AllowMultipleInstances=0" in filedata:
    newdata = filedata.replace("AllowMultipleInstances=0","AllowMultipleInstances=1")
else:
    newdata = filedata
    
    
os.chmod(dst, stat.S_IWRITE)
mts_ini_file_modified = open(dst,'w')
mts_ini_file_modified.write(newdata)
mts_ini_file_modified.close()


#-------------------------------------------------------------------------------------------------------------------------

# Path where configuration file will be copied

#dst_cfg_file = os.path.abspath(os.path.join(MTS_configuration_path, "..", "..", "joint", os.path.basename(Config_name)))
#print("dst_cfg_file din joint... ", dst_cfg_file)


# Copy the cfg file in Toyota Project
#if src_cfg_file != dst_cfg_file:
#    copyfile(src_cfg_file,dst_cfg_file)
#    print("config copied")

Output_path = os.path.join(os.getcwd(), "mts_measurement")
# Create MTS folder structure
if not os.path.exists(os.path.join(Output_path, 'data')):
    os.makedirs(os.path.join(Output_path,'data'))

if not os.path.exists(os.path.join(Output_path, 'ini')):
    os.makedirs(os.path.join(Output_path,'ini'))

if not os.path.exists(os.path.join(Output_path, 'log')):
    os.makedirs(os.path.join(Output_path,'log'))

if not os.path.exists(os.path.join(Output_path, 'res')):
    os.makedirs(os.path.join(Output_path,'res'))
    
if not os.path.exists(os.path.join(Output_path, 'www')):
    os.makedirs(os.path.join(Output_path,'www'))

# Output folder when the bpl files will be created
Output_folder = "ini"

#---------------------------------------------------------------------------------------------------------

section = False
section_text = 0

# Some manipulation below....

Record_dict = {}

endings = [".bpl"] 

path_ini =  os.path.join(Output_path, Output_folder)

for file in os.listdir(path_ini):
    if os.path.isfile(os.path.join(path_ini, file)):
        for ending in endings:
            if ending in file:
                full_path = os.path.join(Output_path, Output_folder, file)
                os.chmod(full_path, stat.S_IWRITE)
                os.remove(full_path)
                print ("The file {} from ini file was deleted.".format(file))
            else:
                print("Skip file {}".format(file))
    else:
        print("Skip folder {}".format(file))

print ("\n")

for root, dirs, files in os.walk(Input_folder):
    if (len(dirs) !=0):
        for input_dir in dirs:
            for input_content in os.listdir(os.path.join(root, input_dir)):
                if input_content.endswith(".rrec") or input_content.endswith(".rec"):
                    Record_dict[input_content] = os.path.join(root, input_dir, input_content)
    
    elif (len(Record_dict) == 0) and (len(files) != 0):
        for input_file in os.listdir(os.path.join(Input_folder)):
            if input_file.endswith(".rrec") or input_file.endswith(".rec"):
                Record_dict[input_file] = os.path.join(Input_folder, input_file)
           
index = 1

for record in Record_dict:
    BPL_content = ''
    BPL_content += '<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n'
    BPL_content += '<BatchList>\n'
    BPL_content += "    <BatchEntry fileName=\"{}\">\n".format(Record_dict[record])

    if section == True:
        BPL_content += "        <SectionList>\n"
        BPL_content += section_text
        BPL_content += "        </SectionList>\n"
    
    BPL_content += "    </BatchEntry>\n"
    BPL_content += "</BatchList>\n"
    
    BPL_file_Path = os.path.join(Output_path, Output_folder, 'BPL_file_{}.bpl')
    BPL_file_Path = BPL_file_Path.format(index)
    
#    if (os.path.isfile(BPL_file_Path) == False):
    BPL_file = open(BPL_file_Path,'w')
    
    print ("Created {} in ini folder.".format(BPL_file_Path.split('\\')[-1]))
        
    BPL_file.write(BPL_content)
    BPL_file.close()
        
    index += 1   

#print("\n\nTask Finished !!")
