/******************************************************************************
 * idctest.c - main and only source file for the OS2AHCI IDC test driver
 *
 * This driver is used to test the ring 0 IDC interface of OS2AHCI.
 * When this driver is opened by a Ring 3 process, it uses OS2AHCI's
 * IDC interface to send it a test IOCTL that does nothing but beep.
 *
 * Authors: Christian Mueller, Markus Thielen
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

#define INCL_NOPMAPI
#define INCL_DOSINFOSEG
#define INCL_NO_SCB
#define INCL_DOSERRORS
#include <os2.h>
#include <dos.h>
#include <bseerr.h>
#include <dskinit.h>
#include <scb.h>

#include <devhdr.h>
#include <iorb.h>
#include <strat2.h>
#include <reqpkt.h>

#include <dhcalls.h>

#include <addcalls.h>
#include <rmcalls.h>
#include <devclass.h>
#include <devcmd.h>
#include <rmbase.h>
#include "ahci-idc.h"


/*---- function proEtotypes ---------------------------------------------------*/

extern APIRET APIENTRY DosPutMessage           (USHORT hf,
                                                USHORT cbMsg,
                                                PCHAR pchMsg);

static unsigned int    strlen                  (PSZ s);

static void            memset                  (void _far *p,
                                                unsigned int cb,
                                                unsigned char byte);

int                    init                    (RPINITIN _far *req);

int                    c_strat                 (RPH _far *req);

void                   drv_open                (void);


/*---- global data -----------------------------------------------------------*/

char init_msg[] = "\r\nOS2AHCI IDC Test Driver\r\n";
char init_fail[] = "Failed to attach to OS2AHCI$";
char init_ok[] = "up and running...\r\n";

PFN Device_Help = 0;

AHCI_ATTACH_AREA attach_data;
int last_cmd;

extern char _cdecl       end_of_data;   /* label at the end of all data segments */
extern void _cdecl _near end_of_code(); /* label at the end of all code segments */

/******************************************************************************
 * c_strat_idc - IDC test driver entry point
 */
int c_strat_idc(RPH _far *req)

{
  USHORT result = STDON;
  last_cmd = req->Cmd;

  switch (last_cmd) {

  case CMDInit:
    return (init((RPINITIN _far *) req));

  case CMDOpen:
    drv_open();
    return(STDON);

  case CMDGenIOCTL:
    drv_open();
    return(STDON);

  case CMDClose:
    return(STDON);

  default:
    break;

  }

  return  STDON | STERR | ERROR_BAD_COMMAND;
}

/******************************************************************************
 * init
 */
int init(RPINITIN _far *req)
{

  char *ahci_name = "OS2AHCI$ ";
  RPINITOUT _far *rsp = (RPINITOUT _far *) req;
  int ret = STDON;

  /* store DevHelp entry point */
  Device_Help = req->DevHlpEP;

  DosPutMessage(1, strlen(init_msg), init_msg);

  /* attach to OS2AHCI.ADD */
  if (DevHelp_AttachDD(ahci_name, (NPBYTE) &attach_data)) {
    /* failed to attach */
    DosPutMessage(1, strlen(init_fail), init_fail);
    rsp->CodeEnd = 0;
    rsp->DataEnd = 0;
    ret = STDON | ERROR_I24_QUIET_INIT_FAIL; 

  } else {
    /* successfully attached */
    DosPutMessage(1, strlen(init_ok), init_ok);
    rsp->CodeEnd = (USHORT) end_of_code;
    rsp->DataEnd = (USHORT) &end_of_data;
    ret = STDON;
  }

  return ret;
}

/******************************************************************************
 * drv_open() - this is called when a ring 3 process opens us with DosOpen.
 * We then call into OS2AHCI's IDC interface with a test IOCTL that causes
 * OS2AHCI to beep.
 */
void drv_open(void)
{
  RP_GENIOCTL rp_ioctl;
  unsigned short target_ds = attach_data.prot_ds;
  PFN_AHCI_IDC_ENTRY func = attach_data.prot_entry_point;
  
  memset((void _far*) &rp_ioctl, sizeof(rp_ioctl), 0x00);
  
  /* prepare request packet */
  rp_ioctl.rph.Len  = (UCHAR) sizeof(rp_ioctl);
  rp_ioctl.rph.Cmd  = CMDGenIOCTL;
  rp_ioctl.Category = OS2AHCI_IDC_CATEGORY;
  rp_ioctl.Function = OS2AHCI_IDC_BEEP;

  /* call IDC entry point in Assembler;
   * IDC calling convention is to set DS to the data segment of
   * the target driver and ES to the segment of the request packet.
   */
  _asm {
    push ds
    push es
    push bx
    push si
    push di

    push ss
    pop  es
    lea  bx, rp_ioctl
    mov  ds, target_ds
    call dword ptr [func]

    pop  di
    pop  si
    pop  bx
    pop  es
    pop  ds
  }

  
}


/******************************************************************************
 * strlen
 */
static unsigned int strlen(PSZ s)
{
  int i = 0;
  while (*s++) {
    i++;
  }
  
  return (i);
}

/******************************************************************************
 * memset
 */
static void memset(void _far *d, unsigned int cb, unsigned char byte)
{
  unsigned int i;
  unsigned char _far *p = (unsigned char _far*) d;

  for (i = 0; i < cb; i++) {
    p[i] = byte;
  }

}
