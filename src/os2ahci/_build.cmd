@echo off
rem ROOT must be set the the base of the AHCI sources.
rem  ie. Where you checked out the trunk, not the os2ahci directory.
rem set ROOT=f:\src\ahci
rem set VENDOR=Your Name
rem set BLD_MAJOR=2
rem set BLD_MINOR=01
rem set WATCOM=e:\Watcom
rem set BLD_DATE=20161008
rem set DDK=f:\ddk
rem set DRV32KIT=f:\Drv32
wmake -a

