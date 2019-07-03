###############################################################################
# Makefile - makefile for os2ahci test program, gcc tool chain
#
# Copyright (c) 2010 Christian Mueller, Markus Thielen.
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

CC         = gcc
LD         = gcc

CFLAGS     = -c
LFLAGS     =

###############################################################################
# Environment

CC_INCLUDE   =

AS_INCLUDE   =

LIB_DIRS     =


###############################################################################
# Main dependencies

TARGET   = ioctl_test.exe

LIBS     = -los2

SRCS     = ioctl_test.c

OBJS     = ioctl_test.o

INCS     =


all: $(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)


###############################################################################
# Object/source dependencies

ioctl_test.o: ioctl_test.c Makefile

###############################################################################
# Action definitions (compile/link commands)

.c.o:
	$(CC) $(CFLAGS) $(CC_INCLUDE) $*.c

$(TARGET): $(OBJS) Makefile
	$(LD) -o $(TARGET) $(OBJS) $(LIB_DIRS) $(LIBS)


