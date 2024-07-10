Simulation modes
=================

This chapter describes which simulation modes are available for CarMaker simulations. |br|
It will show how a CarMaker simulation can be started with pyBase using a specific mode.


Preconditions
---------------
In order to use a specific simulation mode, please be aware that it may be necessary to activate your user account for the specific license. In the following explanations this hint will be given for each license model individually. |br|
Before the testrun starts, pyBase will display the chosen simulation mode as a log message. |br|
**If you are not sure which simulation mode you shall choose, please contact a member of VSP.**

.. note::
  It is recommended to NOT use the GUI / Office mode if you want to run more than one CarMaker instance in parallel as the overall simulation performance decreases. In this case, choose a different mode.


GUI / Office mode
------------------
**Running a CarMaker simulation in GUI mode requires a usual office license which can be requested via SSP7. Once a license is assigned to your user account, you can simulate with any version of CarMaker Office.**
  
By default pyBase executes a CarMaker simulation in the GUI mode. Therefore the communication between pyBase and CarMaker is controlled via telnet. CarMaker will simulate one TestRun after another. If you choose a higher number of CarMaker instances in the cmEvalRunner, multiple instances of CarMaker will run in parallel. Each instance blocks one CarMaker office license. |br| |br|
This mode is activated by passing a list of TestRuns (of type cmTestRun) to the runEvaluations function of the cmEvalRunner. pyBase will ensure a proper execution of all TestRuns. |br|
This is the classical simulation mode that has been used for years. |br|
As the GUI needs to be started and remote-controlled, this mode consumes a few more processing time for the startup. |br|
Choose a higher number of parallel simulations in the cmEvalRunner:

.. code-block:: python

  testRunner = cCMEvalTestRunner(..., numberOfCarMakerInstances = 2, ...)


Headless mode
---------------
**Running a CarMaker simulation in headless mode requires a usual office license which can be requested via SSP7. Once a license is assigned to your user account, you can simulate with any version of CarMaker Office >=8.1.**

.. note::
  Do not mix up this mode with the continuous testing / core license mode!
  
This mode is similar to the GUI mode. More than one simulation can be started as described in the GUI mode. The license usage is the same as in GUI mode.  In this mode no GUI is started for the execution of TestRuns. The simulation is performed headless. This feature is available for CarMaker >=8.1. |br|
Choose the headless mode in the runEvalutions function of the cmEvalRunner:

.. code-block:: python

  ergFilePaths = testRunner.runEvaluations(..., headlessMode = True, ...)


HPC light mode
----------------
**Running a CarMaker simulation in HPC light mode mode requires a usual office license which can be requested via SSP7. Once a license is assigned to your user account, you can simulate with any version of CarMaker Office >=8.1.**

This mode is similar to the GUI mode. Only one instance of CarMaker per machine can be started when using this mode. This mode enables to execute four CarMaker TestRuns in parallel by blocking only one office license. The parallel execution of the TestRuns is handled by CarMaker itself. pyBase cannot control the execution and CarMaker introduces its own naming convention for result files. Thus pyBase cannot distribute TestRuns to different instances and does not support running more than one instance of CarMaker when using this mode. |br|
HPC light mode is chosen automatically, if a testscript file (\*.ts) of the CarMaker simulation manager is passed in the runEvaluations function of the cmEvalRunner instead of a list of TestRuns:

.. code-block:: python

  ergFilePaths = testRunner.runEvaluations(..., 'smokeTests.ts', ...)
  
The ts file has to be placed in the TestRun folder of the chosen CarMaker project.

.. note::
  Only to use the HPC light mode a testscript file is passed to the cmEvalRunner. In all other modes, a list of objects of type cmTestRun is required to give pyBase the execution control!

Continuous testing / Core license mode
----------------------------------------
**Running a CarMaker simulation in Continuous testing / Core license mode requires a core license which is assigned to your user account whenever you request an usual office license via SSP7. Once a license is assigned to your user account, you can simulate with any version of CarMaker Office >=8.1.**

This mode is used for continuous testing and high parallelization purposes. At the moment it is used in the ADAS CIP Bricks build system for continuous testing activities.
The simulation will be performed in a headless mode using a core license. Multiple instances can be started per machine. Each instance blocks one core license. |br|
Choose the continuous testing / core license mode and the number of instances in the constructor of cmEvalRunner class and its runEvaluations method:

.. code-block:: python

  testRunner = cCMEvalTestRunner(..., numberOfCarMakerInstances = 2, ...)
  ergFilePaths = testRunner.runEvaluations(..., useCoreLicense = True, ...)