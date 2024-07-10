"""
@organization:    AMS ADAS SYS DF MF VSP FFM
@author:          Daniel Bremenkamp, modified by Matthias Hillmann
@date:            Created on 26.06.2020
@name:            doc_gen.py

@copyright: (c) Continental Teves AG & Co. oHG
 ALL RIGHTS RESERVED
 The reproduction, transmission, or use of this document or its content is
 not permitted without express written authority. Offenders will be liable
 for damages.
 All rights, including rights created by patent grant or registration of a
 utility model or design, are reserved.
"""

__all__ = ['create_documentation']

__version__ = '1.0.2'

__author__ = 'Daniel Bremenkamp' \
             '<Daniel.Bremenkamp@Continental-Corporation.com>'

import os
import shutil
import glob

def moveAllFilesinDir(srcDir, dstDir):
    # Check if both the are directories
    if os.path.isdir(srcDir) and os.path.isdir(dstDir) :
        # Iterate over all the files in source directory
        for filePath in glob.glob(srcDir + '\*'):
            # Move each file to destination Directory
            shutil.move(filePath, dstDir);
    else:
        print("srcDir & dstDir should be Directories")

def create_rst(root_path):
    """
    create sphinx documentation for plugins
    """
    rename_list = ['static', 'sources']

    # remove old docs folder
    shutil.rmtree(root_path + '/../../docs', ignore_errors=True)
    # run sphinx form console
    os.system(root_path + '/../../doc/make html')
    # move files into correct directories
    moveAllFilesinDir(root_path + '/../../docs/html', root_path + '/../../docs')
    # remove obsolete directories
    shutil.rmtree(root_path + '/../../docs/html', ignore_errors=True)
    #shutil.rmtree(root_path + '/../../docs/doctrees', ignore_errors=True)
    # rename _underscored folders
    abspath = os.path.abspath('root_path' + '/../../../docs')
    os.chdir(abspath)
    for entry in rename_list:
        os.rename(abspath + '/_' + entry, abspath + '/' + entry)
    # get files from input_parser folder
    for root, _, files in os.walk(abspath):
        for name in files:
            if ('.html' in name) or ('.js' in name):
                ifile = open(os.path.join(root, name), 'r', encoding='utf-8')
                data = ifile.readlines()
                ifile.close()
                for idx in range(len(data)):
                    for entry in rename_list:
                        if entry in data[idx]:
                            data[idx] = data[idx].replace('_' + entry, entry)
                ofile = open(os.path.join(root, name), 'w', encoding='utf-8')
                ofile.writelines(data)
                ofile.close()


def create_documentation():
    """
    create documentation of plugins
    """
    # get path of the main file to determine the tool location
    root_path = os.path.dirname(os.path.realpath(__file__))
    # create documentation
    # create_readme(root_path)
    create_rst(root_path)

if __name__ == "__main__":
    create_documentation()