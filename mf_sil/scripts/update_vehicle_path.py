
##################################################################################################################################
#                                   Imports and global variables                                                                 #
##################################################################################################################################

from os import pardir, path, listdir
from optparse import OptionParser

script_path = path.abspath(__file__)

vehicle_prefix = "Vehicle = "
suffix_start = len(vehicle_prefix)
scenario_file_exts = ["", ".testrun"] # Specifies which files should be changed (either no file extension or ".testrun")
new_vehicle_subfolder = ""

default_scenario_path = path.normpath(path.join(script_path, pardir, pardir, "tests", "SIL", "CarMaker", "Data", "TestRun"))
default_vehicle_path = path.join(default_scenario_path, pardir, "Vehicle")

# Return values (error codes):
CHANGED = 0
UP_TO_DATE = 1
COULD_NOT_FIND = 2

##################################################################################################################################
#                                    Function defintions                                                                         #
##################################################################################################################################

def update_vehicle_path_in_file(file_path:str, new_vehicle_subdir:str = new_vehicle_subfolder, new_vehicle_dict:dict = None):
    """! Update the relative path of the vehicle data for a given CarMaker scenario file.

        @param file_path             File path (str) to the CarMaker scenario
        @param new_vehicle_subdir    New relative subdirectory (str) for the vehicle data (default removes all subfolders)
        @param new_vehicle_dict      Optional additonal dictionary (dict) with separate subdirectories for all vehicle data files

        @return     One of the global constants CHANGED, UP_TO_DATE, or COULD_NOT_FIND depending on whether the new file has changed and the reason why it has not changed (if applicable) 
    """
    result = UP_TO_DATE

    lines = []
    with open(file_path, 'r') as file:
        lines = file.readlines()
    # Generate new lines to replace old ones
    newLines = []
    for line in lines:
        # Check if line concerns the vehicle path
        if line.startswith(vehicle_prefix):
            # Determine file name (without subdirectories)
            filename = line[suffix_start:].split("/")[-1].strip()
            # Determine new relative path - try using dictionary first
            if(new_vehicle_dict != None):
                if(filename in new_vehicle_dict):
                    new_vehicle_path = new_vehicle_dict[filename]
                    newLine = vehicle_prefix + new_vehicle_path.replace("\\", "/") + "\n"
                else:
                    result = COULD_NOT_FIND
                    newLines.append(line)
                    continue
            else:
                if(new_vehicle_subdir == ""):
                    new_vehicle_path = filename
                else:
                    new_vehicle_path = path.join(new_vehicle_subdir, filename)            
                newLine = vehicle_prefix + new_vehicle_path.replace("\\", "/") + "\n"
            if(newLine != line):
                result = CHANGED
            newLines.append(newLine)
        else:
            newLines.append(line)
    # Update file
    with open(file_path, 'w') as file:
        file.writelines(newLines)

    return result

def check_exts(filename:str, file_exts, check_empty_ext:bool = True):
    """! Check if a filename has a certain extension

    @param filename      The name or full path of the file (str)
    @param file_exts     The desired file extensions (either a single str or a list of str)
    @param check_empty_ext  An optional boolean flag. If True, an input file_ext = "" will only return true if the entire filename contains no "."

    @return     True if the filename ends with the desired extension and False otherwise 
    """
    result = False

    if(type(file_exts) != list):
        file_exts = [file_exts]

    for ext in file_exts:
        # Special case: Check if file has "." (thus no extension at all) if check_empty_ext is True. Otherwise, accept any file (since all files formally end with "").
        if(ext == ""):
            if(not check_empty_ext):
                result = True
                break
            elif(not filename.__contains__(".")):
                result =True
                break
        # Regular case: Check for the specified file-ending
        else:
            if(filename.endswith(ext)):
                result = True
                break
    return result
        
def find_rel_vehicle_paths(vehicle_folder_path:str, current_subdir:str=""):
    """! Generate a dictionary containing the relative paths of all vehicle data files.

    @param vehicle_folder_path   The absolute path (str) to the 'Vehicle' folder (<cmprjdir>/Data/Vehicle)
    @param current_subdir        Only used for recursive calls, do not change default input

    @return     A dictionary (dict) using filenames as keys and the relative paths to the files as arguments
    """
    result = dict()

    found_files =listdir(vehicle_folder_path)

    for filename in found_files:

        filepath = path.join(vehicle_folder_path, filename)

        if path.isdir(filepath):
            # Recursive function call and merge (update) result
            temp = find_rel_vehicle_paths(filepath, current_subdir=path.join(current_subdir, filename))
            result.update(temp)
        else:
            result[filename] = path.join(current_subdir, filename).replace("\\", "/")

    return result

def update_vehicle_path_in_folder(folder_path:str, file_exts = scenario_file_exts, apply_to_subfolders:bool = True, auto_find:bool = True, 
                vehicle_folder_path:str = default_vehicle_path, new_vehicle_subdir:str=new_vehicle_subfolder,  silent:bool = False, summaryOnly:bool = True):
    """! Update all CarMaker scenarios in a given folder regarding their vehicle data path.

    @param folder_path          The absolute path of the folder (str) containing all CarMaker scenarios (folder is named "TestRun" in mf_sil project)
    @param file_exts            The extension (str or list of str) of the CarMaker scenario files
    @param apply_to_subfolders  Option (bool) to recursively apply the function to all subfolders found at folder_path
    @param auto_find            Option (bool) to automatically determine the paths of vehicle data files by scanning the "Vehicle" folder
    @param vehicle_folder_path  The path of the "Vehicle" folder (str) containing vehicle data files (only used if auto_find is True)
    @param new_vehicle_subdir   The new subdirectory (str) for the vehicle data (only used if auto_find is False)    
    @param silent               Option (bool) to turn off all console outputs
    @param summaryOnly          Option (bool) to turn off all outputs from recursive calls and only print a final summary

    @return     Summary list: numbers of changed, up-to-date, and ignored files and a list of filenames that could not be updated (vehicle path not found) 
    """
    found_files = listdir(folder_path)

    # Find relative paths of all vehicle files
    if(auto_find):
        vehicle_dict = find_rel_vehicle_paths(vehicle_folder_path)
    else:
        vehicle_dict = None

    # Initialize Summary 
    summaryList = [0, 0, 0, [] ]

    # Update scenario files
    for filename in found_files:
        filepath = path.join(folder_path, filename)
        # Found a subfolder
        if path.isdir(filepath):
            if(apply_to_subfolders):
                silent_rec = True
                if((not silent) and (not summaryOnly)):
                    print("Recursively updating folder: " + filepath)
                    silent_rec = False
                tempSummary = update_vehicle_path_in_folder(filepath, file_exts, apply_to_subfolders, auto_find, vehicle_folder_path, new_vehicle_subdir, silent = silent_rec)
                for i in range(0,4):
                    summaryList[i] += tempSummary[i]
        # Found a valid scenario file
        elif check_exts(filename, file_exts):
            change_result = update_vehicle_path_in_file(filepath, new_vehicle_subdir, vehicle_dict)
            # Update summary information
            if change_result == CHANGED:
                summaryList[0] += 1
            elif change_result == COULD_NOT_FIND:
                summaryList[3].append(filepath)
            elif change_result == UP_TO_DATE:
                summaryList[1] += 1
        # Found an invalid file
        else:
            summaryList[2] += 1
    # Print summary
    if not silent:
        print("\n" + str(summaryList[0]) + " files changed \n" + str(summaryList[1]) + " files already up-to-date \n" + str(summaryList[2]) + " files with mismatching extensions ignored")
        if( len(summaryList[3]) > 0):
            print("\n" + "Failed to update " + str(len(summaryList[3])) + " file(s): \n")
            for filename in summaryList[3]:
                print("Could not find vehicle path for file: " + filename)

    return summaryList

##################################################################################################################################
#                                    Run vehicle path update                                                                     #
##################################################################################################################################

if __name__ == '__main__':
    optparser = OptionParser(usage ="usage: %prog [options]")
    optparser.add_option("-p", "--path", dest="folder_path",  default=default_scenario_path, help="Path to CarMaker scenarios." )
    optparser.add_option("-q", "--quiet", action = "store_true", dest="silent", default=False, help = "Turns off console outputs.")
    optparser.add_option("-r", "--recursionoff", action = "store_false", dest="recursive", default =True, help = "Disables changes to files in subfolders.")
    optparser.add_option("-a", "--autooff", action = "store_false", dest="auto", default = True, help = "Disables automatic determination of vehicle data paths inside project folder.")
    optparser.add_option("-s", "--recursiveSummary", action = "store_false", dest="summaryOnly", default = True, help = "Enables summary console outputs for recursive calls.")
    args = optparser.parse_args()[0]

    vehicle_folder_path = path.normpath(path.join(args.folder_path, pardir, "Vehicle"))

    print("Starting file updates...")
    update_vehicle_path_in_folder(args.folder_path, file_exts = scenario_file_exts,  apply_to_subfolders = args.recursive, auto_find = args.auto, 
                                    vehicle_folder_path = vehicle_folder_path, new_vehicle_subdir= new_vehicle_subfolder, silent = args.silent, summaryOnly= args.summaryOnly)