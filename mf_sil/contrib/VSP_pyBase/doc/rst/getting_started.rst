Getting Started
======================================

This chapter describes how to use pyBase in a project.

Preconditions
---------------
Please ensure to use the following software in the correct version:

* Python 3.7
* Carmaker 8.1.x or later

Carmaker shall be installed in its default path, which is

.. code-block:: bash

  C:\LegacyApp\IPG

In Bricks build system Carmaker is usually installed in 

.. code-block:: bash

  C:\cip_tools\carmaker\<version>\IPG
  
Both paths are supported by pyBase. If your Carmaker is installed in a different location, pyBase may produce errors when executing a simulation while it tries to find the GUI executable in the installation path. This problem can occur when the office license mode is used.

.. note::
  If you installed a newer version of Carmaker on your machine in the standard installation directory, pyBase will detect and start the newer version automatically. As there is an example repository existing (introduced in a section below) that is set up with the version mentioned above, the example execution may fail since the example project is not compatible with newer Carmaker versions.
  To avoid the behavior starting always the newest version or starting Carmaker from a customized installation path, see the example below or the documentation of cm_eval_runner to specify a fixed version to be used.


Important information
----------------------
pyBase can be treated like a library, but in comparison to a real library all the source code is visible.
pyBase shall not be adapted or changed according to projects needs, but it is a common base for all projects. |br|

.. note::
  If you want to change something in the source code of pyBase, please be aware that the VSP department cannot support in case of issues. There is no dedicated team and there are no dedicated people assigned to give support for customized versions of pyBase. |br|
  If you face an issue in a released version, you can contact team members from VSP.
  
Integrate pyBase to a project
------------------------------
There are two options to integrate pyBase to a project.

Integrate pyBase as Conan package via build.yml in CIP Bricks build system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
For each release of pyBase >=2.0.3 there is a Conan package available in the CIP Artifactory.
The package can be referenced in the build.yml file of the project's repository.

.. note::
  Please regard that the content of the folder pyfctEval is NOT part of the Conan package. This ensures the most relevant parts of pyBase for the CarMaker test automatization to be present with a minimum of memory consumption.

Integrate pyBase as git submodule
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
As pyBase is treated like a library, no project specific changes are made within pyBase.
To use the classes and methods pyBase provides, you can link a specific released commit to your project in Git as submodule.
An example how your testing repository may look like, can be found here in the **VSP_cmEval_Example** repository: https://github-am.geo.conti.de/ADAS/VSP_cmEval_Example |br|

Run an example simulation
--------------------------
There is an example repository that executes a simulation and generates a report: https://github-am.geo.conti.de/ADAS/VSP_cmEval_Example |br|
It can be cloned and executed if the preconditions mentioned above are fulfilled.

.. note::
  Remember to use the recursive option when cloning the repository as it contains submodules: |br|
  git clone https://github-am.geo.conti.de/ADAS/VSP_cmEval_Example --recursive (two dashes before recursive option!)
  
As pyBase uses python site-packages that do not come along with the standard python installation, there is a *requirements.txt* file on the top level in pyBase's repository. |br|
To not harm your global python installation on your machine, it is recommended to use python virtual environments to resolve missing dependencies. This approach is compatible with CIP Bricks build system. If you are not familiar with python virtual environments, see the example batch file in the example repository: https://github-am.geo.conti.de/ADAS/VSP_cmEval_Example/blob/master/run/runEvaluation.bat |br|
This batch file creates a virtual environment, installs the necessary site-packages, runs an example simulation and generates a simulation report. After the simulation ends the report will show automatically. |br|

Create your own simulation execution
-------------------------------------
To create your own execution script you can use the example mentioned above as a template. As the example consists of large script files, here is some guidance. |br|
The most important parts of the execution scripts are following:

.. code-block:: python

  # Retrieve all TestCases and TestRuns
  global testCasesBase
  testCasesBase = testcases.GetTestCases(testCaseSetName, CMPrjDir, False)
  testCasesAE   = testcases.GetTestCases(testCaseSetName, CMPrjDir, True)
  global testRunsBase
  testRunsBase = [currTestCase["cmTestRun"] for currTestCase in testCasesBase]
  testRunsAE   = [currTestCase["cmTestRun"] for currTestCase in testCasesAE]
	
In above mentioned code a list of testruns is created. The lists testRunsBase and testRunsAE contain objects of class cmTestRun. All testruns in these lists shall be simulated. |br|
In the following code you can see the execution of the simulation:

.. code-block:: python

  # Run all TestRuns
  testRunner   = cCMEvalTestRunner(CMPrjDir, export_quantities, numberOfCarMakerInstances, sampleRate='default', cmGuiPath = r"C:\LegacyApp\IPG\carmaker\win64-8.1.1\bin\CM.exe")
  global ergFilePaths
  ergFilePaths = testRunner.runEvaluations(testCaseSetName, testRunsBase + testRunsAE, maxTimeInSimulationSeconds, 
                                             headlessMode = True, useCoreLicense = True, nRetries = N_RETRIES, executable = os.path.join(CMPrjDir, "src", "CarMaker.win64.exe"), 
                                             ignoreExistingResFile = True)
									 
After the method runEvaluations(..) finished, the simulation happened. All the execution logic is "hidden" behind this method. |br|
The example files are quite longer as it shows that we parallel the erg file parsing for a better performance.
After the simulation finished, a report can be generated as shown in the example. |br|
As pyBase uses python's logging class the output status or error messages, please ensure you initialize the logging library in your scripts. If you do not, you cannot see messages coming from pyBase.
For initialization you can use your own output format or use the example:

.. code-block:: python

  import logging
  ### Enable Logging
  logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(message)s', datefmt='%d-%m-%Y %H:%M:%S')
  
For further details about the execution parameters or report generation, please have a look in the corresponding documentation section.

Typical issues
-------------------------
**When I clone the example repository, the pyBase folder is empty.** |br|
Reason: You did not clone recursively as pyBase is shared as a submodule. |br|
Solution: See the example section above. |br|
|br|
**When I try to start the simulation pyBase outputs the error message "CarMaker Telnet Connection Timeout."** |br|
Reason: This may happen on machines with a slow system performance or on machines with high work load. Carmaker does not respond within a given time. |br|
Solution: The time is configured in *cmpyglue.py* in variable *TELNET_CONNECTION_TIMEOUT*. Adapt it and try it again. |br|
Note: Please report this issue to a VSP member as the timeout may be adapted in the pyBase in general if this issue occurs frequently for many users.