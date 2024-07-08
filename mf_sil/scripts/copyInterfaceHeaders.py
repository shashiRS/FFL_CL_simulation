import os
import shutil
import errno
import stat
import sys

#script to copy all interface includes to a common target include directory
curDir = os.path.dirname(__file__)
rootDir = os.path.abspath(os.path.join(curDir, '..', '..'))

# check args
copyForCarMaker = True
arguments = len(sys.argv) - 1
position = 1
while (arguments >= position):
    arg = sys.argv[position]
    print ("parameter %i: %s" % (position, arg))
    if(arg == '--MTS'):
        copyForCarMaker = False
    position = position + 1

#Copy variant specific content
selectedVariantFile = "selected_variant.txt"
if os.path.exists(os.path.join(curDir,selectedVariantFile)):
    f=open(os.path.join(curDir,selectedVariantFile), "r")
    if f.mode == 'r':
        variant =f.read()
        f.close()
        print('Selected variant from ' + selectedVariantFile + ': ' + variant)
else:
    print('ERROR in copyInterfaceHeaders.py: ' + selectedVariantFile + ' not existing. Therefore a variant can not be evaluated. Please run getComponents.py first to define your variant!')
    exit()

#allow to remove read only files
def del_rw(action, name, exc):
    os.chmod(name, stat.S_IWRITE)
    os.remove(name)

# param input for path selection
if copyForCarMaker:
    print ("copy to CarMaker dir")
    dstPath = r'mf_sil\tests\SIL\CarMaker\include'
else:
    print ("copy to MTS dir")
    dstPath = r'mf_sil\tests\SIL\MTS\include'

foldersInRoot = os.listdir(rootDir)
destFolder = os.path.join(os.path.abspath(rootDir), dstPath +'\CopiedIncludes')
filesToCopy = {}

#remove old header files if exist
if os.path.isdir(destFolder):
  print('Remove old header files from: ' + destFolder)
  shutil.rmtree(destFolder, onerror=del_rw)

#create destination folder if not existing
if not os.path.isdir(destFolder):
  print('Creating folder: ' + destFolder)
  os.makedirs(destFolder)

#copy new header files
print('Copy new header files to ' + destFolder)
for curFolder in foldersInRoot:
  componentDir = os.path.join(os.path.abspath(rootDir), curFolder);  
  if os.path.isdir(componentDir):
    exclude = ["conan_workarea"]
    exclude.append("prqa_reports")
    exclude.append("venv")
    exclude.append(".git")
    if(curFolder == 'jsoncpp'):
      interfaceDir = os.path.join(os.path.abspath(componentDir), 'include')
      tgtFolder = 'json'
    elif (curFolder == 'adas_platforms'):
      interfaceDir = os.path.join(os.path.abspath(componentDir), 'src\\x86')
      tgtFolder = 'adas_platforms'
    elif (curFolder == 'si_core'):
      interfaceDir = os.path.abspath(componentDir)
      tgtFolder = 'si_core'
    #for mf_taposd coppy also src folder
    elif (curFolder == 'mf_taposd'):
      interfaceDir = os.path.abspath(componentDir)
      tgtFolder = 'mf_taposd'
    elif (curFolder == 'mf_plot'):
      interfaceDir = os.path.abspath(componentDir)
      tgtFolder = 'mf_plot'
    elif (curFolder == 'TRATCO'):
      interfaceDir = os.path.join(os.path.abspath(componentDir), 'src\\project\\PLP\\platform\\vsp')
      tgtFolder = 'TRATCO'
    elif (curFolder == 'VECONA'):
      interfaceDir = os.path.join(os.path.abspath(componentDir), 'src\\project\\PLP\\platform\\vsp')
      tgtFolder = 'VECONA'
    elif (curFolder == 'plp_log'):
      interfaceDir = os.path.abspath(componentDir)
      tgtFolder = 'plp_log'
    else:
      interfaceDir = os.path.join(os.path.abspath(componentDir), 'interface')
      tgtFolder = curFolder;

    if(variant == 'entry'):
      #print("excluding base and performance" + variant)
      exclude.append("premium_parking")
      exclude.append("performance_parking")
    elif(variant == 'base'):
      #print("excluding entry and performance" + variant)
      exclude.append("ultrasonic_parking")
      exclude.append("performance_parking")
    elif(variant == 'base_moco'):
      #print("excluding entry and performance" + variant)
      exclude.append("ultrasonic_parking")
      exclude.append("performance_parking")
    else:
      #print("excluding ultrasound" + variant)
      #This will be rolled out later. Currently for most components it should be equal: exclude.append("premium_parking")
      exclude.append("ultrasonic_parking")

    if (os.path.isdir(interfaceDir)):
      for root, dirs, files in os.walk(interfaceDir):
        dirs[:] = [d for d in dirs if d not in exclude]
        for file in files:
          if '.h' in file:
            # si_core exports the header 'cem_iodata_pod.h' to the subfolder 'cem_dbm_adapter_if'
            if file == 'cem_iodata_pod.h':
              fileTgtFolder = os.path.join(tgtFolder, 'cem_dbm_adapter_if')
            else:
              fileTgtFolder = tgtFolder
            filesToCopy[(os.path.join(root, file))] = os.path.join(os.path.abspath(destFolder), os.path.join(fileTgtFolder, file))

if os.path.isdir(os.path.join(os.path.abspath(rootDir), dstPath)): 
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
else:
  print('ERROR in copyInterfaceHeaders.py: ' + destFolder + ' not existing.')
  exit()

