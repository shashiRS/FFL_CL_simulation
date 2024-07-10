.. cm_eval_runner documentation file

cm_eval_runner
======================================

cCMEvalTestRunner
------------------
.. hint::
  As this class is used by a developer for executing a Carmaker simulation, see the example repository linked in the "Getting Started" section for more details.
  
.. note::
  In runEvaluations function the parameter returnErgPaths will be removed in the next versions. That means in next versions of pyBase the function will not \
  return a list of absolute erg file paths anymore but a list of cmTestRun objects as it is passed in parameter testRuns. In this list the execution status and absolute path \
  to the erg file is contained. |br|
  This is necessary as up to now TestRuns which are passed to runEvaluations and get an execution error, are not documented.
  Only in case of using the HPC light mode, it still returns a list of absolute paths to erg files.
  
.. autoclass:: pycmEval.cm_eval_runner.cCMEvalTestRunner
   :members:
   :exclude-members: SimulationRunningThread, SimulationRunningThreadHeadless, TestSeriesRunningThread