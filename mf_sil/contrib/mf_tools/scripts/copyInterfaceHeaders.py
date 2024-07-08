import os
import shutil
import errno
import stat
import sys

#script to copy all interface includes to a common target include directory
curDir = os.path.dirname(__file__)
rootDir = os.path.abspath(os.path.join(curDir, '..', '..'))

# check args
arguments = len(sys.argv) - 1
position = 1
variant = ""
while (arguments >= position):
    arg = sys.argv[position]
    print ("parameter %i: %s" % (position, arg))
    if(arg == '--generic'):
        variant = 'generic'
        print ("Selected variant: generic")
    if(arg == '--cus_only'):
        variant = 'cus_only'
        print ("Selected variant: cus_only")
    position = position + 1

#allow to remove read only files
def del_rw(action, name, exc):
    os.chmod(name, stat.S_IWRITE)
    os.remove(name)

foldersInRoot = os.listdir(rootDir)
destFolder = os.path.join(os.path.abspath(rootDir), r'CopiedIncludes')
filesToCopy = {}

#remove old header files if exist
if os.path.isdir(destFolder):
  print('Remove old header files from: ' + destFolder)
  shutil.rmtree(destFolder, onerror=del_rw)
  
# remove all _types repos
for k in foldersInRoot:
    if "_types" in k:
        foldersInRoot.remove(k)

#copy new header files
print('Copy new header files to ' + destFolder)
for curFolder in foldersInRoot:
  componentDir = os.path.join(os.path.abspath(rootDir), curFolder);  
  if os.path.isdir(componentDir):
    exclude = set()
    if(curFolder == 'jsoncpp'):
      interfaceDir = os.path.join(os.path.abspath(componentDir), 'include')
      tgtFolder = 'json'      
    elif (curFolder == 'si_core'):
      interfaceDir = os.path.abspath(componentDir)
      tgtFolder = 'si_core'
      exclude.update(["conan_workarea", "venv"])
    #for mf_taposd copy also src folder
    elif (curFolder == 'mf_taposd'):
      interfaceDir = os.path.abspath(componentDir)
      tgtFolder = 'mf_taposd'
      exclude.update(["conan_workarea", "venv"])
    else:
      interfaceDir = os.path.join(os.path.abspath(componentDir), 'interface')
      tgtFolder = curFolder;

    if (os.path.isdir(interfaceDir)):
      for root, dirs, files in os.walk(interfaceDir):
        dirs[:] = [d for d in dirs if d not in exclude]
        for file in files:
          if '.h' in file:
              filesToCopy[(os.path.join(root, file))] = os.path.join(os.path.abspath(destFolder), os.path.join(tgtFolder, file))

    if (curFolder != 'mf_sil'): #ignore mf_sil, as this folder contains so many .h files in the tests folder, this could crash this script
        # add all .h-files located in comonentDir/tests/platform to the filesToCopy-List
        # these files will be copied to a folder "componentDir_test".
        testsDir = os.path.join(os.path.abspath(componentDir), 'tests/platform')
        if (os.path.isdir(testsDir)):
            for r, d, f in os.walk(testsDir):
                for file in f:
                    if '.h' in file:
                        if file == 'SI_OUTPUT_TestDataHandler.h':
                            filesToCopy[(os.path.join(r, file))] = os.path.join(os.path.abspath(destFolder), os.path.join(curFolder + "_JsonReader", file))
                        else:
                            filesToCopy[(os.path.join(r, file))] = os.path.join(os.path.abspath(destFolder), os.path.join(curFolder + "_test", file))

if os.path.isdir(rootDir): 
  for src, dst in filesToCopy.items():
    try:
      shutil.copy(src, dst)
    except IOError as e:
      # EACCESS: another file with same name already exists, don't copy the second one
      if e.errno == errno.EACCES:
        continue
      # ENOENT(2): file does not exist, raised also on missing dst parent dir
      if e.errno != errno.ENOENT:
        raise
      # try creating parent directories
      os.makedirs(os.path.dirname(dst))
      shutil.copy(src, dst)
    #Set copied files to read only to avoid cahning these files afterwards
    os.chmod(dst, stat.S_IREAD)

