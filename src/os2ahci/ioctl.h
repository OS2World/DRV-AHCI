/******************************************************************************
 * ioctl.h - IOCTL structures and constants for os2ahci driver
 *
 * Copyright (c) 2011 thi.guten Software Development
 * Copyright (c) 2011 Mensys B.V.
 * Copyright (c) 2013-2018 David Azarewicz
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

/* IOCTL categories and functions */
#define OS2AHCI_IOCTL_CATEGORY         0x87
#define OS2AHCI_IOCTL_GET_DEVLIST      0x01
#define OS2AHCI_IOCTL_PASSTHROUGH      0x02

/* IOCTL definitions from s506oem.h (primarily required for SMART calls) */
#define DSKSP_CAT_SMART             0x80  /* SMART IOCTL category */
#define DSKSP_SMART_ONOFF           0x20  /* turn SMART on or off */
#define DSKSP_SMART_AUTOSAVE_ONOFF  0x21  /* turn SMART autosave on or off */
#define DSKSP_SMART_SAVE            0x22  /* force save of SMART data */
#define DSKSP_SMART_GETSTATUS       0x23  /* get SMART status (pass/fail) */
#define DSKSP_SMART_GET_ATTRIBUTES  0x24  /* get SMART attributes table */
#define DSKSP_SMART_GET_THRESHOLDS  0x25  /* get SMART thresholds table */
#define DSKSP_SMART_GET_LOG         0x26  /* get SMART log        table */
#define DSKSP_SMART_AUTO_OFFLINE    0x27  /* set SMART offline autosave timer */
#define DSKSP_SMART_EXEC_OFFLINE    0x28  /* execute SMART immediate offline */

#define SMART_CMD_ON                   1  /* on value for related SMART functions */
#define SMART_CMD_OFF                  0  /* off value for related SMART functions */

#define DSKSP_CAT_GENERIC           0x90  /* generic IOCTL category */
#define DSKSP_GEN_GET_COUNTERS      0x40  /* get general counter values table */
#define DSKSP_GET_UNIT_INFORMATION  0x41  /* get unit configuration and BM DMA counters */
#define DSKSP_GET_INQUIRY_DATA      0x42  /* get ATA/ATAPI inquiry data */

/* unit information structure flags from s506oem.h */
#define UIF_VALID           0x8000U       /* unit information valid */
#define UIF_TIMINGS_VALID   0x4000U       /* timing information valid */
#define UIF_RUNNING_BMDMA   0x2000U       /* running Bus Master DMA on unit */
#define UIF_RUNNING_DMA     0x1000U       /* running slave DMA on unit */
#define UIF_SATA            0x0004U       /* SATA             */
#define UIF_SLAVE           0x0002U       /* slave on channel */
#define UIF_ATAPI           0x0001U       /* ATAPI device if 1, ATA otherwise */

/* device flags */
#define DF_LBA48            0x0001U
#define DF_ATAPI            0x0002U
#define DF_ATAPI_16         0x0004U
#define DF_REMOVABLE        0x0008U

/* passthrough flags */
#define PT_WRITE            0x0001U    /* transfer direction host -> device */
#define PT_ATAPI            0x0002U    /* ATAPI command (ATA if not set) */

/* ------------------------ typedefs and structures ------------------------ */

#ifndef VMDHL_WRITE
typedef void *LIN;
#endif

#pragma pack(1)

/******************************************************************************
 * Generic ATA-8 command structure. This is loosely based on ATA-8, i.e. we
 * don't have to deal with shadow registers and map them to the low bytes of
 * adjecent words, etc. but there are still some oddities which need to be
 * taken into consideration. For example, ATA-8 says LBA-28 sector addresses
 * are simply the lower 28 bits in the LBA field. However, the underlying
 * transport (SATA in our case) only looks at the three lower bytes of the
 * LBA field, much like an IDE device would do. This means that only 3 bytes
 * of the LBA field are processed and this yields only 24 bits. The remaining
 * 4 bits need to be stored in the "device" register....
 *
 * Everything else seems to behave normally, as far as this term can be
 * applied to IDE/ATA. Further details can be found in section 7.1.5.1 of the
 * ATA-8 spec (Inputs for 28-bit Read/Write Commands).
 */
typedef struct {
  USHORT  features;       /* feature bits */
  USHORT  count;          /* count register (e.g. number of sectors) */
  ULONG   lba_l;          /* low 24 bits of LBA-28 (32 bits for 48-bit devices) */
  USHORT  lba_h;          /* high 16 bits of LBA for 48-bit devices */
  UCHAR   cmd;            /* ATA command field */
  UCHAR   device;         /* ATA device field and upper 4 bits of LBA-28 */
} ATACMD;

/******************************************************************************
 * Data structure for OS2AHCI_IOCTL_GET_DEVLIST; the parameter for this IOCTL
 * is a USHORT with the number of entries in 'devs'.
 */
typedef struct {
  USHORT  cnt;            /* number of entries in 'devs' */
  struct {
    USHORT  adapter;      /* adapter */
    USHORT  port;         /* port */
    USHORT  device;       /* device (port multiplier) */
    USHORT  type;         /* device type; see UIB_TYPE_* in iorb.h */
    USHORT  ncq_max;      /* maximum number of queued commands */
    USHORT  flags;        /* device flags; see DF_* above */
  } devs[1];
} OS2AHCI_DEVLIST;

/******************************************************************************
 * Parameter structure for OS2AHCI_IOCTL_PASSTHROUGH; the data structure for
 * this IOCTL is a buffer for sense data in case of errors.
 */
typedef struct {
  USHORT  adapter;        /* adapter */
  USHORT  port;           /* port */
  USHORT  device;         /* device (port multiplier) */
  USHORT  flags;          /* request flags; see PT_* above */
  ULONG   timeout;        /* timeout in seconds (0 = default of 30s) */

  USHORT  cmdlen;         /* length of ATA or ATAPI command */
  union {
    ATACMD ata;           /* ATA command */
    UCHAR  cdb[20];       /* ATAPI command */
  } cmd;

  ULONG   buflen;         /* length of buffer for data transfers */
  void *buf;              /* buffer for data transfers (32-bit linear address) */
  USHORT  sense_len;      /* length of sense data in IOCTL DataPacket */
} OS2AHCI_PASSTHROUGH;

/******************************************************************************
 * DSKSP command parameters; copied from s506oem.h
 */
typedef struct _DSKSP_CommandParameters
{
  BYTE        byPhysicalUnit;             /* physical unit number 0-n */
                                          /* 0 = Pri/Mas, 1=Pri/Sla, 2=Sec/Mas, etc. */
} DSKSP_CommandParameters;

/******************************************************************************
 * DSKSP device counters data; copied from s506oem.h
 */
typedef struct _DeviceCountersData
{
  USHORT      wRevisionNumber;            /* counter structure revision */
  ULONG       TotalReadOperations;        /* total read operations performed */
  ULONG       TotalWriteOperations;       /* total write operations performed */
  ULONG       TotalWriteErrors;           /* total write errors encountered */
  ULONG       TotalReadErrors;            /* total read errors encountered */
  ULONG       TotalSeekErrors;            /* total seek errors encountered */
  ULONG       TotalSectorsRead;           /* total number of sectors read */
  ULONG       TotalSectorsWritten;        /* total number of sectors written */

  ULONG       TotalBMReadOperations;      /* total bus master DMA read operations */
  ULONG       TotalBMWriteOperations;     /* total bus master DMA write operations */
  ULONG       ByteMisalignedBuffers;      /* total buffers on odd byte boundary */
  ULONG       TransfersAcross64K;         /* total buffers crossing a 64K page boundary */
  USHORT      TotalBMStatus;              /* total bad busmaster status */
  USHORT      TotalBMErrors;              /* total bad busmaster error */
  ULONG       TotalIRQsLost;              /* total lost interrupts */
  USHORT      TotalDRQsLost;              /* total lost data transfer requests */
  USHORT      TotalBusyErrors;            /* total device busy timeouts        */
  USHORT      TotalBMStatus2;             /* total bad busmaster status */
  USHORT      TotalChipStatus;            /* total bad chip status */
  USHORT      ReadErrors[4];
  USHORT      WriteErrors[2];
  USHORT      SeekErrors[2];
  USHORT      SATAErrors;
} DeviceCountersData;

/******************************************************************************
 * DSKSP unit information data; copied from s506oem.h
 */
typedef struct _UnitInformationData
{
  USHORT      wRevisionNumber;            /* structure revision number */
  union {
    struct {
      USHORT  wTFBase;                    /* task file register base addr */
      USHORT  wDevCtl;                    /* device control register addr */
    } rev0;
    ULONG     dTFBase;                    /* task file register base addr */
  };
  USHORT      wIRQ;                       /* interrupt request level */
  USHORT      wFlags;                     /* flags */
  UCHAR       byPIO_Mode;                 /* PIO transfer mode programmed */
  UCHAR       byDMA_Mode;                 /* DMA transfer mode programmed */
  ULONG       UnitFlags1;
  USHORT      UnitFlags2;
} UnitInformationData;


#pragma pack()

