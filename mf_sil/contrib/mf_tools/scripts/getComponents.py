import yaml
import os
import subprocess
import sys
import six

curDir = os.path.dirname(__file__)
baseRepoDir = os.path.abspath(os.path.join(curDir, '..', '..'))
gitHubDefaultPath = "git@github-am.geo.conti.de:ADAS/"
selectedVariantFile = "selected_variant.txt"

okColor = '\033[94m' 
failColor = '\033[91m'
endColor = '\033[0m'

forceCheckout = False
askForProjectVariant = True

# check args
arguments = len(sys.argv) - 1
position = 1
while (arguments >= position):
    arg = sys.argv[position]
    print ("parameter %i: %s" % (position, arg))
    if(arg == '-f'):
        forceCheckout = True
        print ("forceCheckout")
    if(arg == '--generic'):
        askForProjectVariant = False
        project_variant = 'generic'
        print ("Selected variant generic")
    if(arg == '--cus_only'):
        askForProjectVariant = False
        project_variant = 'cus_only'
        print ("Selected variant generic")
    position = position + 1
    
# user input to select premium or cus only variant
if askForProjectVariant == True:
    project_variant = six.moves.input('Please enter a variant: generic or cus_only ?\n')

# check for available project variants: generic, cus_only and update/remove selectedVariantFile
if (project_variant != 'generic') and (project_variant !='cus_only'):
    print('ERROR in getComponents.py: ' + project_variant +' is no valid project variant! Please enter one of the following variants:')
    print('generic')
    print('cus_only')
    print('Removed selected variant file ' + selectedVariantFile)
    if os.path.exists(selectedVariantFile):
        os.remove(selectedVariantFile)
    exit()
else:
    print('Valid project variant selected: ' + project_variant)
    file = open(selectedVariantFile,"w") 
    file.write(project_variant) 
    file.close()
    print('Updated selected variant file ' + selectedVariantFile)

# read yaml file
with open("../conf/build.yml", 'r') as stream:
  data_loaded = yaml.safe_load(stream)  
  dependencies = data_loaded.get('variants').get(project_variant).get('uses')
  
  # remove all _types repos
  for k, v in list(dependencies.items()):
    if "_types" in k:
        del dependencies[k]
  
  subprocess.call('', shell=True)
  print(endColor)
  print(' checking out revisions for mf_tools')
  for key in dependencies:
    repo = key
    release = dependencies[key].get('release')
    if repo == 'jsoncpp':
        # hack for jsoncpp (other default path)
        row = [repo, "http://github-am.geo.conti.de/ADAS-3RDPARTY/"+repo, ""]
    elif repo == 'us_psd':
        # hack for us_psd (branch sandbox/devBranch)
        row = [repo, "http://github-am.geo.conti.de/ADAS/"+repo, ""]
    else:
        row = [repo, gitHubDefaultPath+repo, "release/"+release]
    os.chdir(baseRepoDir)
    #clone if folder does not exist      
    if not os.path.isdir(os.path.join(baseRepoDir, row[0])):
      print(okColor + 'cloning ' + row[0] + ' from ' + row[1] + ' version:' + row[2] + endColor)
      cloneCmd = 'git clone ' + row[1]
      os.system(cloneCmd)      
      if(row[2]): #non-empty revision info
        os.chdir(os.path.join(baseRepoDir, row[0]))
        os.system('git fetch --tags')      
        checkoutCmd = 'git checkout tags/' + row[2] + ' -b ' + row[2]
        print(okColor + checkoutCmd + endColor)
        os.system(checkoutCmd)
    else:
      #if already existing, check out required revision if no local changes exist, otherwise warn
      os.chdir(os.path.join(baseRepoDir, row[0]))
      statusProc = subprocess.Popen('git status', stdout=subprocess.PIPE, shell=True)
      statusRes = statusProc.stdout.read();
      numLinesInStatus = statusRes.count(b'\n')
      if numLinesInStatus > 4 and not forceCheckout:
        print (okColor +  ' local changes in ' + row[0] + '... skipping! Please stash local changes if you want to check out the revision needed for mf_tools!' +' ' + endColor)
        continue
      statusLines = statusRes.splitlines()
      currentBranch = statusLines[0].split()[2]
      branchStatus = statusLines[1].split()[3]
      if(row[2]): #specific revision required
        if(row[2] != currentBranch.decode()):
          os.system('git fetch --tags')      
          checkoutCmd = 'git checkout tags/' + row[2] + ' -b ' + row[2]
          print(okColor + 'Switching to other revision of ' + row[0] + ': ' + row[2] + endColor)
          print(checkoutCmd)
          switchProc = subprocess.Popen(checkoutCmd, stderr=subprocess.PIPE, shell=True)
          switchRes = switchProc.stderr.read();  
          print(switchRes.decode())
          if((b'fatal' in switchRes )and (b'exists' in switchRes)):#if the branch already exists, just check it out
            print('Checking out existing branch ' + row[2])
            os.system('git checkout ' + row[2])  
        else:
          print(okColor + row[0] +' already in revision ' + row[2] + endColor)
      else:#latest and greatest required, pull if no local changes
        if(branchStatus != 'ahead'):
          print(okColor + ' pulling ' + row[0] +' ' + endColor)
          os.system('git pull')         
        else:
          print(failColor +  ' local commits in ' + row[0] +'... skipping!'  +' ' + endColor)
input("Press Enter to continue with copying interface headers and fetching needed headers from artifactory...")