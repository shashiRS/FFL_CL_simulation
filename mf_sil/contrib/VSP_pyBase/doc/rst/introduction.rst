Introduction
======================================

Scope of pyBase
----------------
The scope of pyBase is to automatize the execution and evaluation of regression tests. This shall enable Carmaker to be used in CI/CT systems for nightly tests or tests that are executed for every commit. Additionally pyBase shall enable developer to perform regression tests of their software, which is integrated in Carmaker, locally and fast on their local machines. |br|
|br|
Usecases of pyBase:

* Regression tests and nightly tests locally on a developers machine and in CI/CT systems (like CIP Bricks)
* Common scripts for all driving function projects to execute their simulations in Carmaker
* Enable developers to perform testing-driven development with fast feedback loops on their own machines
* Support of latest Carmaker versions and its interfaces
* Provide a straight forward approach for executing Carmaker simulation and report generation

Out of scope for pyBase:

* KPI testing framework with the help of pre-defined classes to define KPIs (as this is covered by TSF framework)
* Connection to tools like IMS, Github, Doors, Rhapsody or test databases (as this is covered by TSF framework)
* Generation of a large number of testcase/testrun with statistical methods like classification tree method (as this is covered by the VSP ScenarioGenerator)

pycmCommon
-----------
In pycmCommon there is cmpyglue. cmpyglue is the **C**\ ar\ **M**\ aker **Py**\ thon **glue**. It enables a developer to control the Carmaker via python.

.. note::
  Please ensure your user account has a valid Carmaker license assigned to use these methods. Especially when you try to use core licenses instead of usual office licenses there are restricted as they are intended to be used for CI/CT activities. |br|
  If you did not request a license but you need one, for an usual office license please request a Carmaker installation in SSP7.
  After your request is closed, a license is assigned to your Windows user account by the IT. |br|
  If you need a core license, please contact Thomas Meissner (thomas.meissner@continental-corporation.com) from VSP (Virtual Simulation Platform) team.
  

pycmEval
-----------
pycmEval is **py**\ thon **C**\ ar\ **M**\ aker **Eval**\ uation and is an abstraction layer that provides classes and functions that can be used to control Carmaker simulations without caring about a telnet connection or the erg file format. |br|
Additionally it provides most commonly used evaluation functions for the evaluation of the results after the simulation. |br|
**cm_eval_runner** is a class to configure and run a Carmaker simulation in an automated manner. |br|
**cm_eval_erg_parser** is a class providing methods to read data from erg files. |br|
**cm_eval_erg_calculator** is a class providing commonly used methods for the result evaluation like calculating the minimum distance between the ego and a traffic vehicle or calculate the time of collision.

pyHTMLReport
--------------
pyHTMLReport is **py**\ thon **HTML Report** and provides classes and methods to write an evaluation report in html format without writing html code. |br|
It includes javascript applets for creating tables, drawing graphs with zoom options and a lazy load mechanism to create a performant web page to visualize simulation results.

.. note::
  It is recommended to use Chrome or Firefox to open the web page. There may be issues with Internet Explorer or Edge in regards to the javascript applets.