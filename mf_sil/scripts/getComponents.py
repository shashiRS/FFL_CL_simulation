import yaml
import os
import subprocess
import sys
import six

if sys.version_info[0] < 3:
    sys.exit("Unsupported python version. Python v3.6.x is supported.")

curDir = os.path.dirname(__file__)
thisRepoDir = os.path.abspath(os.path.join(curDir, '..'))
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
    if (arg == '--base'):
        askForProjectVariant = False
        project_variant = 'base'
        print ("Selected variant base")
    if (arg == '--base_moco'):
        askForProjectVariant = False
        project_variant = 'base_moco'
        print ("Selected variant base_moco")
    if(arg == '--entry'):
        askForProjectVariant = False
        project_variant = 'entry'
        print ("Selected variant entry")
    if(arg == '--performance'):
        askForProjectVariant = False
        project_variant = 'performance'
        print ("Selected variant performance")
    position = position + 1

# user input to select premium or cus only variant
if askForProjectVariant == True:
    project_variant = six.moves.input('Please enter a variant: entry or base or performance ?\n')

# check for available project variants: base, base_moco, entry, performance and update/remove selectedVariantFile
if (project_variant != 'base') and (project_variant !='base_moco') and (project_variant !='entry') and (project_variant !='performance'):
    print('ERROR in getComponents.py: ' + project_variant +' is no valid project variant! Please enter one of the following variants:')
    print('base')
    print('base_moco')
    print('entry')
    print('performance')
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
  # HACK to add mf_tools release 2.13.0 used for HTML report generation in CarMaker evaluation (e.g. regression tests)
  dependencies.update(mf_tools = {'release': '2.13.5', 'variant': 'generic'})
  dependencies.update(hmiaup = {'release': '1.6.1', 'variant': 'generic'})
  # HACK to get SWFW as long as not provided by appl_srv_layer
  dependencies.update(swfw_base = {'release': '6.0.3', 'variant': 'generic'})
  if (project_variant !='entry'):
    # HACK to also get svc_model_lib for the CarMaker.sln
    dependencies.update(svc_model_lib = {'release': '1.2.0', 'variant': 'generic'})
  # skip carmaker_build_dep since no repository exists for it
  dependencies.pop('carmaker_build_dep', None)
  subprocess.call('', shell=True)
  print(endColor)
  print('Checking out revisions for CarMaker')
  for key in dependencies:
    repo = key
    release = dependencies[key].get('release')
    if repo == 'jsoncpp':
        # hack for jsoncpp (other default path)
        row = [repo, "http://github-am.geo.conti.de/ADAS-3RDPARTY/"+repo, ""]
    elif repo == 'hmiaup' or repo == 'adas_platforms' or repo == 'cml':
        # hack for hmiaup, adas_platforms, cml (release naming v1.x.x)
        row = [repo, gitHubDefaultPath+repo, "v"+release]
    elif repo == 'swfw_base' or repo == 'swfw_if':
        # hack for swfw (release naming releases/1.x.x)
        row = [repo, gitHubDefaultPath+repo, "releases/"+release]
    elif (repo == 'mf_lsca' and release == '1.1.45-dev2') or repo == 'svc_model_lib':
        # hack for mf_lsca release 1.1.45-dev2 (naming without 'release/' prefix)
        row = [repo, gitHubDefaultPath+repo, release]
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
        print (failColor +  'Local changes in ' + row[0] + '... skipping! Please stash local changes if you want to check out the revision needed for CarMaker!' +' ' + endColor)
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
        if(branchStatus.decode() != 'ahead'):
          print(okColor + 'Pulling ' + row[0] +' ' + endColor)
          os.system('git pull')         
        else:
          print(failColor +  ' local commits in ' + row[0] +'... skipping!'  +' ' + endColor)

# Fetch the --projects required variants from build.yml for the standalone solution
os.chdir(thisRepoDir)
os.system("python ./scripts/fetch.py")

print("Success")
if sys.version_info[0] < 3:
    raw_input("Press Enter to continue...")
else:
    input("Press Enter to continue...")