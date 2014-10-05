/******************************************************************************
 * ahciidc.h - definitions for the OS2AHCI IDC interface
 *
 * Copyright (c) 2011 thi.guten Software Development
 * Copyright (c) 2011 Mensys B.V.
 *
 * Authors: Christian Mueller, Markus Thielen
 *
 * Parts copied from/inspired by the Linux AHCI driver;
 * those parts are (c) Linux AHCI/ATA maintainers
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


/* -------------------------- macros and constants ------------------------- */

/* IDC category */
#define OS2AHCI_IDC_CATEGORY           0x88

/* enter BIOS mode (e.g. to generate trap dumps)
 * Request packet parameters: None
 */
#define OS2AHCI_IDC_BIOSMODE           0x66

/* test IOCTL; does nothing but BEEP
 * Request packet parameters: None
 */
#define OS2AHCI_IDC_BEEP               0x67


/* ------------------------ typedefs and structures ------------------------ */

/* AHCI_IDC_ENTRY defines a function pointer to the OS2AHCI IDC entry point */
typedef void (_cdecl _far *PFN_AHCI_IDC_ENTRY) (unsigned short real_ds, 
                                                RP_GENIOCTL _far *ioctl);

#pragma pack(1)

/*
 * AHCI_ATTACH_AREA defines the device driver attach table as required
 * to attach to os2ahci.add using DevHelp_AttachDD
 */
typedef struct {
  PFN_AHCI_IDC_ENTRY  real_entry_point;   /* real mode entry point address */
  unsigned short      real_ds;            /* real mode DS */

  PFN_AHCI_IDC_ENTRY  prot_entry_point;   /* protected mode entry point address */
  unsigned short      prot_ds;            /* protected mode DS */
  } AHCI_ATTACH_AREA;

#pragma pack()

/* -------------------------- function prototypes -------------------------- */

/* ------------------------ global/static variables ------------------------ */


