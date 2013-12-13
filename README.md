cpNode
======

Use Arduino platforms for CMRI interface input/output controls.

This software developed by Chuck Catania and Seth Neumann, for use
with bare Arduino boards and the cpNode I2C based I/O extenders.

Discussions about this software or the cpNode boards takes place on
the Arduini Yahoogroup.  More details on group membership is at
http://groups.yahoo.com/neo/groups/Arduini/info



CMRI
====

CMRI is a long standing protocol originally designed to provide
computer control for model railroads.  Most CMRI installations are a
set of boards connected to a single host via an RS-485 serial
interface.  This cpNode software sets up an Arduino to be a completely
CMRI compatible device on that RS-485 bus.

Extensions to the CMRI protocol are proposed to augment the
capabilities of the network, by allowing for arbitrary data to be sent
to or from the various devices.  The initial driver for this
capability is to allow a remote RFID reader to send information about
the tags read by that reader.


Licensing
=========

cpNode is licensed under the Creative Commons Attribution-ShareAlike
3.0 Unported License.  For more information, please visit visit
http://creativecommons.org/licenses/by-sa/3.0/deed.en_US

