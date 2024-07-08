#! /usr/bin/env python3
import os
from optparse import OptionParser

executablePathDefault      = None
carmakerProjectPathDefault = None

def update_cm_exec_path(cm_exec_path, cm_prj_path):
    """
    Update the path to the Carmaker executable in the Carmaker project.

    Arguments:
        cm_exec_path (str): File path to Carmaker executable
        cm_prj_path (str): Directory path Carmaker project

    Returns:
        None
    """

    # normalize path to Carmaker executable
    cm_exec_path = os.path.normpath(cm_exec_path)

    # Set path to Carmaker executable by updating GUI file in Carmaker project
    gui_file_path = os.path.join(cm_prj_path, 'Data', 'Config', 'GUI')
    lines = []
    with open(gui_file_path, 'r') as file:
        lines = file.readlines()

    newLines = []
    for line in lines:
        if line.startswith('CM.Exe ='):
            newLines.append('CM.Exe = ' + cm_exec_path.replace('\\', '/') + '\n')
        else:
            newLines.append(line)

    with open(gui_file_path, 'w') as file:
        file.writelines(newLines)


if __name__ == '__main__':
    ### Parse Command Line Options
    optparser = OptionParser(usage="usage: %prog [options]")
    optparser.add_option("-e", "--exe", dest='executablePath',  default=executablePathDefault, help="Path to the Carmaker executable." )
    optparser.add_option("-p", "--project", dest='carmakerProjectPath',  default=carmakerProjectPathDefault, help="Path to Carmaker project." )
    args = optparser.parse_args()[0]

    update_cm_exec_path(args.executablePath, args.carmakerProjectPath)

