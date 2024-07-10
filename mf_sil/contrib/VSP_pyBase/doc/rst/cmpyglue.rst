.. cmpyglue documentation file

cmpyglue
======================================

CMRC
-----------
CMRC is the **C**\ ar\ **M**\ aker **R**\ emote **C**\ ontrol.
There is a class (CMRC) to control the Carmaker GUI via a telnet connection and so control the execution of testruns. |br|
In this case an usual office license per instance is needed.

.. autoclass:: pycmCommon.cmpyglue.CMRC
   :members:
   
CMRC_HeadlessInstance
----------------------
CMRC_HeadlessInstance means **C**\ ar\ **M**\ aker **R**\ emote **C**\ ontrol **HeadlessInstance** and is a class to control the Carmaker using a headless mode. |br|
Optionally it allows to use a core license for the simulation. If you want to use a core license it has to be assigned to your user account. |br|
See introduction page for further details.


.. note::
  Sometimes the core license is named by its old term "runtime license". The license was renamed by IPG to "core license".

.. autoclass:: pycmCommon.cmpyglue.CMRC_HeadlessInstance
	:members:
	