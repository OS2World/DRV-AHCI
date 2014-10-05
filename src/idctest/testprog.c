/*************************************************************************
 *
 *  testprog.c - ring 3 test program for AHCI IDC function.
 *  This program opens the IDCTEST.SYS driver, which will call OS2AHCI's
 *  IDC entry point with a TEST IOCTL that does nothing but beep.
 * 
 *  Author: Markus Thielen, thi.guten Software Development.
 *
 *  Compilation (Watcom): wcl386 -bt=os2 test.c
 *
 * Copyright (c) 2010 by thi.guten Software development, www.thiguten.de
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*---- includes --------------------------------------------------------------*/

#define INCL_DOSFILEMGR
#define INCL_DOSPROCESS
#define INCL_DOSERRORS
#include <os2.h>
#include <stdio.h>
#include <string.h>


/******************************************************************************
 * main()
 */
void main(void)
{
  ULONG rc;
  ULONG action;
  HFILE hf;
  char *drv_name = "IDCTEST$";

  printf("OS2AHCI IDC Test Program. Do you hear a beep??\n");
  rc = DosOpen(drv_name, &hf, &action, 0,
               FILE_NORMAL, FILE_OPEN, OPEN_SHARE_DENYNONE, NULL);
  if (rc) {
    /* open failed */
    fprintf(stderr, "DosOpen failed with code %d ", rc);
    fprintf(stderr, "Is IDCTEST.SYS really loaded?\n");
    return;
  }

  DosClose(hf);

}
