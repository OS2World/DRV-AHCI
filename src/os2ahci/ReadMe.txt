AHCI Driver for OS/2 v2.06

Introduction
============

OS2AHCI is an AHCI driver for OS/2. It supports both ATA and
ATAPI devices in a single driver. An ATAPI/CDROM filter driver is
only required if you want to read/write CD-DA (audio) format CDs.


Copyrights and License
======================

(c) Copyright IBM Corporation 1990,2000.
All rights reserved.
Copyright (c) 2011 thi.guten Software Development
Copyright (c) 2011 Mensys B.V.
Copyright (c) 2013-2018 David Azarewicz

Authors: Christian Mueller, Markus Thielen

Parts copied from/inspired by the Linux AHCI driver;
those parts are (c) Linux AHCI/ATA maintainers.

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 MA  02111-1307  USA

The OS2AHCI.ADD Driver Software is a derivative work of the IBM DDK.
Binary programs and documentation for the OS2AHCI.ADD Driver
Software are licensed to and distributed by Arca Noae, LLC.

The source code can be retrieved from http://svn.netlabs.org.
In compliance to the GNU General Public License, the source code
can of course be modified/compiled to run on other products as long
as modifications will also be published as outlined in the GNU GPL.

Please note that builds other than the official binary delivered by
the Arca Noae web site are not officially supported.


Getting Support and Reporting Problems
======================================

For more information and to report problems please visit:
  https://www.arcanoae.com
and click on SUPPORT.

Or go directly to the AHCI support wiki at:
  https://www.arcanoae.com/wiki/ahci/


Driver Command Line Options
===========================

Global Options

Option                 Description
------------------------------------------------------------------------------
/B:<baud>              Initialize the COM port to the specified baud rate. Allowable
                       baud values are: 300, 600, 1200, 2400, 4800, 9600, 19200,
                       38400, 57600, and 115200. /B has no effect if /C is not also
                       specified. If /B is not specified, the COM port is not
                       initialized. For example, if you are using the kernel debugger,
                       the kernel debugger initializes the COM port so you should not
                       use this switch.

/C:<n>                 Set debug COM port base address. Values for n can be:
                         1 = COM1
                         2 = COM2
                         a hex value (COM port base address) COM1=3f8, COM2=2f8
                       The default is 0. If set to 0 then no output goes to the COM port.

/D[:n]                 Debug output to COM port/debug buffer. Values for n can be:
                         1 = requests
                         2 = detailed
                         3 = verbose
                       If :n is not specified the debug level is incremented for
                       each /D specified.

/W                     Allows the debug buffer to wrap when full.

/V[:n]                 Display informational messages during boot. Values for n can be:
                         1 = Display sign on banner
                         2 = Display adapter information
                       If :n is not specified the verbosity level is incremented for
                       each /V specified.

/G:<vendor>:<device>   Add generic PCI ID to list of supported AHCI adapters
                       (e.g. /G:8086:2829)

/T                     Perform thorough PCI ID scan; default = on, can be
                       turned off with /!T to perform only a PCI class scan

/F                     Force the use of the HW write cache when using NCQ
                       commands; see "Native Command Queuing" below for
                       further explanation (default = off)

/R                     Reset ports during initialization (default = on)
                       Can be turned off with /!R, however, when the
                       [Intel] AHCI controller was found to be
                       initialized by the BIOS in SATA mode, ports will
                       always be reset even when /!R is specified

/A:n                   Set adapter to n for adapter-specific options
                       (default = -1, all adapters)

/P:n                   Set port to n for port-specific options
                       (default = -1, all ports)

/I                     Ignore current adapter if no port has been specified.
                       Otherwise, ignore the current port on the current adapter.

/U                     Check for usable disks and ignore disks that are not
                       usable. To be usable a disk must be an MBR disk or wiped.
                       (default = on) Can be turned off with /!U.

Port-specific Options

Option                 Description
------------------------------------------------------------------------------
/S                     Enable SCSI emulation for ATAPI units (default = on)
                       SCSI emulation is required for tools like cdrecord.

/N                     Enable NCQ (Native Command Queuing) for hard disks
                       (default = off)

/LS                    Set link speed (default = 0):
                         0 = maximum,
                         1 = limit to generation 1
                         2 = limit to generation 2
                         3 = limit to generation 3

/LP                    Set link power management (default = 0):
                         0 = full power management,
                         1 = transitions to "partial slumber state" disabled,
                         2 = transitions to "slumber state" disabled,
                         3 = transitions to both partial and slumber states disabled

/4                     Force track size to be 56 sectors regardless of the
                       reported disk geometry to optimize partition boundaries
                       for hard disks with 4096 byte sectors.

Port-specific options depend on the currently active adapter
and port selector (/A and /P). Those selectors are -1 per default
which means "all" adapters/ports. The scope can be reduced by limiting
it to an adapter (/A) or an adapter and a port (/A and /P). The scope
can be reset by setting the corresponding option back to -1.

For example:

  BASEDEV=OS2AHCI.ADD /N /A:0 /P:5 /!N /A:1 /P:-1 /!N

This has the following effect:

  - Enable NCQ for all hard disks
  - Disable NCQ for hard disk on adapter #0, port #5
  - Disable NCQ for all hard disks on adapter #1

Another example:

  BASEDEV=OS2AHCI.ADD /A:0 /P:0 /I

Means to ingore the disk plugged into port 0 on adapeter 0.


Native Command Queuing
======================

Native Command Queuing (NCQ) is a feature which allows sending multiple I/O
requests to hard disks before waiting for any of the requests to complete,
much like Tagged Command Queuing for SCSI devices. This allows the disks
to reorder I/O requests to minimize head movements, resulting in improved
performance when executing random I/Os. In practice, this will be most
noticable when multiple programs request I/O services to different parts
of the disk -- a single program typically won't queue up I/O's but instead
will wait for each I/O to complete (with the exception of programs like
database servers).

While we believe NCQ will work with the majority of controllers and hard
disks, it's currently turned off by default until we have more feedback
from OS/2 users. In order to turn on NCQ, just add the command line
option "/N" to OS2AHCI.ADD.

NCQ and HW Caches
-----------------

In NCQ mode, OS2AHCI supports a request flag which allows upstream code
(e.g. file systems) to force writes to go directly to the disk instead
of being buffered in the HW disk cache. However, at least JFS doesn't
support this flag properly which effectively disables the HW disk cache
for write operations across the board, resulting in a substantial
performance loss. In order to prevent OS2AHCI from disabling the HW
cache when so requested by upstream code, please use the command line
option "/F".

This may, of course, result in data loss in case of power failures but
apparently this was the situation with previous IDE drivers as well thus
shouldn't make much difference in the field. The JFS code also seems to
imply that this flag has never been widely supported by [IDE] drivers;
otherwise, the JFS developers should have stumbled over the performance
loss a long time ago and fixed the code.

NOTES:

 - Without NCQ, OS2AHCI behaves like former IDE drivers, i.e. the HW
   cache will always be enabled (on modern disks).

 - When suspending, rebooting or shutting down, OS2AHCI always flushes
   the HW disk cache regardless of the "/F" or "/N" command line options.


Interoperability With IDE Drivers
=================================

There are three kinds of IDE/ATA/SATA controllers:

 1. Older controllers (IDE or SATA) without AHCI support
    This kind of controller will only be recognized by IDE drivers
    (IBM1S506.ADD or DANIS506.ADD).

 2. AHCI-capable controllers which supports IDE/SATA interfaces
    This kind of controller will work with IDE or AHCI drivers and it's
    up to the user to decide which driver to use.

 3. AHCI-only controllers
    This kind of controller will only be recognized by OS2AHCI.

If there's a mix of controllers of types 1 and 3, both an IDE and an AHCI
driver will be required.

If type 2 controllers are involved, it's up to the user to decide which
driver to use. Both DANIS506.ADD and OS2AHCI.ADD will verify whether another
driver has already allocated the corresponding adapter, thus the only
decision to take for mixed configurations is whether type-2 controllers
should be handled by DANIS506.ADD or OS2AHCI.ADD and this can be done by
having the desired driver's BASEDEV statement coming first in CONFIG.SYS.

NOTE: Older versions of DANIS506.ADD did not verify whether the resources
      of a particular adapter were already allocated by another driver.
      DANIS506.ADD 1.8.8 or later is required for this to work.

      When using earlier versions of DANIS506.ADD, the options "/A:x /I"
      will be required to tell DANIS506.ADD to ignore adapters to be
      driven by OS2AHCI.ADD. The same applies to IBM1S506.ADD

Mixed Controller Example
------------------------

Assume a DELL D630 or a Thinkpad T60. The hard disk is attached to the
SATA/AHCI controller of the ICH-7 hub while the CDROM is attached to the
PATA IDE controller. This allows two different configurations:

 1. Drive HDD and CDROM via DANIS506.ADD
 2. Drive HDD via OS2AHCI.ADD and CDROM via DANIS506.ADD

OS2AHCI.ADD can't drive the CDROM because it's attached to a PATA
IDE controller which doesn't support AHCI.

 - If OS2AHCI.ADD comes first in CONFIG.SYS, it will take over the SATA/AHCI
   controller and drive the HDD. DANIS506.ADD will take care of the PATA/IDE
   controller for the CDROM.
 - If DANIS506.ADD comes first in CONFIG.SYS, it will take over both the
   SATA/AHCI and the PATA/IDE controller and OS2AHCI.ADD will silently exit.

Advantages of AHCI
------------------

The interfaces provided by the various [Intel] controllers could be
summarized like this (the term ATA as driver interface being a bit of our
own invention):

 - Intel PIIX: IDE (I/O registers) and ATA (taskfile)
 - Intel ICH6: IDE (I/O registers), ATA (taskfile) and SATA
   (FIS, vendor-specific)
 - Intel ICH7: IDE (I/O registers), ATA (taskfile), SATA
   (FIS, vendor-specific) and AHCI (FIS)
 - Intel PCH: AHCI (FIS)

Taskfiles are regions in memory with ATA commands which the IDE/ATA
controller can read and process autonomously. FIS (Frame Information
Structures) are pretty much the same but they are specific to the SATA
communication protocol on the serial link. The most important FIS type
for AHCI drivers is the H2D (host to device) FIS which basically contains
the ATA command to be executed.

The big advantage of AHCI controllers, apart from being vendor-neutral,
is that they take care of a lot of things which previous-generation
drivers like DANIS506 would have to do step by step. For example, in
order to send an ATAPI command, DANIS506 would have to do the following:

  * Send ATA "PACKET" command to device (via IDE registers, ATA taskfiles
    or SATA FIS)
  * Wait until device signals via interrupt it's ready for the ATAPI command
  * Send ATAPI command to device via PIO
  * Wait until device signals via interrupt it's ready to transfer data
  * Send/Receive any data that might come along with the ATAPI command via
    PIO, or wait for DMA transfer to complete
  * Wait until device signals via interrupt that command and data transfer
    have completed

For OS2AHCI, the same operation looks like this:

  * Fill in AHCI command header, FIS with ATA "PACKET" command and the ATAPI
    command
  * Tell port engine to process the command
  * Wait until controller signals via interrupt that command and data
    transfer have completed

The AHCI controller automatically takes care of all underlying bits and
pieces. OS2AHCI doesn't even have to know whether a particular message is
sent via PIO or DMA because this is handled by the AHCI controller, too.
And the whole concept of PIO and DMA is only relevant between AHCI controller
and the device -- all transfers between OS2AHCI and the AHCI controller are
always done via DMA.


SMART Support
=============

Starting with version 1.22, OS2AHCI supports the IOCTL interface required by
existing SMART monitoring tools. Beware that the IBM version of smartctl.exe
is hard-coded to open the character device named "IBMS506$" so it will not
work with OS2AHCI. You must use a version that allows specifying ahci devices.

NOTES:

 - The IOCTL interface for SMART is based on the idea of IDE controllers
   with a master and a slave drive. OS2AHCI maps all devices (ATA or ATAPI)
   sequentially to this pattern. If, for example, you have 4 hard disks and
   one CDROM attached to a single controller on ports 1, 2, 5, 7, and 11,
   SMART tools will see 3 controllers as follows:

    - controller 0, master: HD on port 1
    - controller 0, slave:  HD on port 2
    - controller 1, master: HD on port 5
    - controller 1, slave:  HD on port 7
    - controller 2, master: CDROM on port 11

 - The DSKSP_GEN_GET_COUNTERS interface is currently unsupported; calls to
   the corresponding IOCTL will return 0 for all counters. SMART counters
   are not affected by this limitation, i.e. SMART tools will be able to
   report counters from the physical disk; this limitation only affects
   the software counters maintained by ADD drivers which do support the
   DSKSP_GEN_GET_COUNTERS IOCTL request.

Known Issues and Limitations
============================

Hot swap is not supported.
Port expanders are not supported. Only one drive per port is allowed.


Manual Installation
===================

- Copy the driver file, OS2AHCI.ADD, to \OS2\BOOT on your boot disk.

- Add the following line to CONFIG.SYS:
  BASEDEV=OS2AHCI.ADD


Building The Driver
-------------------

The toolchain required for compilation consists of:

 - The MiniDDK or an updated DDK (You must have a DDK license
   to build this driver.)
 - Open Watcom version 1.9 or later
 - The Drv32 kit. (You must have a DDK license to use the Drv32 kit.)

Define DDK, WATCOM, and DRV32KIT in the environment.
Use "wmake" or "wmake -a" to build the driver. See _build.cmd.


Change Log
==========

v.2.06 19-Sep-2018 - David Azarewicz
  Added Usable Disk check to ignore non-usable (e.g. non-MBR) disks.
  Added /U switch to enable/disable Usable Disk check.

v.2.05 05-Apr-2018 - David Azarewicz
  Changes to debug output for debug versions.
  Removed interrupt reqirement on init.

v.2.04 06-Dec-2017 - David Azarewicz
  Cosmetic changes to user display.
  Fixed the ioctl pass-thru interface.
  Removed the old IBM smartctl.exe from the distribution.

v.2.03 15-Jul-2017 - David Azarewicz
  Added MSI support. PSD 3.23.06 or higher is required for MSI.

v.2.02 07-Jun-2017 - David Azarewicz
  Fixed interrupt handler issue for multiple adapters.

v.2.01 01-Oct-2016 - David Azarewicz
  Major reorganization of the entire driver.
  Enhanced debugging support.

v.1.32 09-Nov-2013 - David Azarewicz
  Fix for some hardware that reports incorrect status
  Report real device in addition to fake SCSI device when SCSI emulation
    is enabled.

v.1.31 21-Aug-2013 - David Azarewicz
  Enhanced debug output.
  Added code to check for bad geometries reported by the BIOS and fix them.
  Fixed a PCI ID coding error that has been there since version 1.01.

v.1.30 29-Jun-2013 - David Azarewicz
  Enhanced debug log output
  Removed the IBMS506 header that was causing problems and shouldn't
    be there anyway.
  Fixed a defect in the SMART IOCtl.
  Added ability to ignore individual ports.

v.1.29 12-Jun-2013 - David Azarewicz
  Changed scsi emulation to be on by default.

v.1.28 01-Jun-2013 - David Azarewicz
  Reworked trap dump kernel exit
  Removed unused IDC entry point.
  Reworked suspend/resume routines.
  Implemented a temporary hack to make resume work reasonably well.
  Suspend/resume is only supported on OS/2 systems with ACPI.
  Suspend/resume is known to not work reliably and cannot be further
    addressed in this driver.

v.1.27 23-Apr-2013 - David Azarewicz
  Added LVM aware disk geometry reporting.
  Begin to add disk information report - not finished yet.
  Removed undocumented /Q switch and made the driver quiet by default.
  Debug output improvements.
  Added /B switch for setting debug baud rate.
  Fixed up time delay functions

v.1.26 26-Mar-2013 - David Azarewicz
  Fix spin-up / power-up issue on some hardware
  Reorganized and improved debug output.

v.1.26 21-Feb-2013 - rousseau
  Virtual box fix
  Some SMP fixes
  Changed default for port reset to always

v.1.25 02-Oct-2012 - markus.thi
  Added support for trap dumps

v.1.24 21-May-2012 - markus.thi
  Fixed JFS long format hang (ticket 16)

V.1.23 16-May-2012 - markus.thi
  added IDC entry point to allow switching back to BIOS mode

v.1.22 17-Oct-2011 - markus.thi
  Added "IBMS506" header to accomodate broken SMART tools.
