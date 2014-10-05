###############################################################################
# Makefile - makefile for IDCTEST driver, c600 tool chain
#
# Copyright (c) 2011 thi.guten Software Development
# Copyright (c) 2011 Mensys B.V.
#
# Authors: Christian Mueller, Markus Thielen
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#

###############################################################################
# Tool Chain
#

AS         = $(DDK)\tools\alp.exe
CC         = cl
LD         = $(DDK)\base\tools\link.exe
MAPSYM     = $(DDK)\base\tools\mapsym.exe
BLDLEVEL   = cmd.exe /c ..\..\tools\bldlvl.cmd
BLDDATE    = cmd.exe /c ..\..\tools\blddate.cmd

AFLAGS     = -Mb
CFLAGS     = -c -nologo -Asnw -W2 -G2ms -Zlpd -Fc -Zi -DECS_BUILD
LFLAGS     = /noe /nod /packd /a:16 /batch /map /line

###############################################################################
# Environment

# main path to OS/2 DDK; this needs to be set before this makefile will work
!ifndef DDK
DDK          = i:\ddk
!endif

# main path to 16 bit C compiler
!ifndef CC16
CC16         = i:\c600
!endif

CC_INCLUDE   = -I..\include \
               -I$(DDK)\base\h \
               -I$(DDK)\base\ibmh \
	       -I$(DDK)\base\src\dev\dasd\diskh \
               -I$(DDK)\base\src\dev\thinkpad\dockii\apmcalls \
               -I$(CC16)\include

AS_INCLUDE   = -I:$(DDK)\base\inc \
               -I:$(DDK)\base\src\dev\dasd\diskinc

LIB_DIRS     = $(DDK)\base\lib\ \
	       $(DDK)\base\src\dev\dasd\devhlp\ \
               $(DDK)\base\src\dev\thinkpad\dockii\apmcalls\ \
               $(CC16)\lib\ \


###############################################################################
# Main dependencies

TARGET   = idctest.sys

LIBS     = dhcalls doscalls slibcep

SRCS     = init.asm idctest.c

OBJS     = init.obj idctest.obj

INCS     = ..\include\ahci-idc.h


all: $(TARGET)

clean:
	rm -f $(OBJS) $(TARGET) *.cod *.lst


###############################################################################
# Object/source dependencies

init.obj:     init.asm   Makefile

idctest.obj:  idctest.c  Makefile $(INCS)


###############################################################################
# Action definitions (compile/link commands)

# emacs TAGS file creation
# NOTE: OS/2 emacs etags.exe expects an empty file named c:\dev\null...
tags:	$(SRCS) $(INCS) Makefile
	etags.exe $(SRCS) $(INCS) $(DDK)\base\src\dev\dasd\diskh\dhcalls.h \
	$(DDK)\base\src\dev\dasd\diskh\iorb.h \
	$(DDK)\base\h\bsedos16.h \
	$(DDK)\base\h\devcmd.h \
	$(DDK)\base\h\reqpkt.h

.asm.obj:
	$(AS) $(AFLAGS) $(AS_INCLUDE) $*.asm

.c.obj:
	$(CC) $(CFLAGS) $(CC_INCLUDE) $*.c

$(TARGET): $(OBJS) idctest.def Makefile
	$(LD) $(LFLAGS) $(OBJS),$(TARGET),$*.map,$(LIB_DIRS) $(LIBS),$*.def
	$(MAPSYM) idctest


