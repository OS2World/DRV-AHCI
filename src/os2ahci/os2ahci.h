/******************************************************************************
 * os2ahci.h - main header file for os2ahci driver
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

/* ----------------------------- include files ----------------------------- */

/* IMPORTANT NOTE: The DDK headers require tight structure packing and this
 * is controlled via compiler parameters. Thus, all stuctures in os2ahci.add
 * are expected to be byte-aligned without the need of explicit pragma pack()
 * directives. Where possible, the structures are laid out such that words
 * and dwords are aligned at least on 2-byte boundaries.
 */

/* Global feature defines
 * DEBUG = enable debug logging routines to be compled in.
 * LEGACY_APM = enable the legacy APM interface to be compiled in.
 *   Legacy APM support is not needed on eCS systems with ACPI and is more reliable without it enabled.
 */
//#define LEGACY_APM
//#define DAZ_NEW_CODE

#include "Dev32lib.h"
#include "Dev32rmcalls.h"
#include <Dev32iorb.h>
#include "ahci.h"
#include "ahci-idc.h"

/* -------------------------- macros and constants ------------------------- */

#define MAX_AD 8 /* maximum number of adapters */

#define TIMER_COUNT 128

/* default command timeout (can be overwritten in the IORB) */
#define DEFAULT_TIMEOUT 30000

/* Maximum number of retries for commands in the restart/reset context hooks.
 *
 * Please note that the corresponding variable in the ADD workspace is a bit
 * field, thus increasing this value means increasing the size of the bit
 * field. At the time of writing this comment the 'retries' variable was 2
 * bits wide (i.e. a maximum number of 3 retries) and there was exactly one
 * bit left before the ADD workspace structure would become too large...
 */
#define MAX_RETRIES 3

/* debug output macros */
#ifdef DEBUG
#define DPRINTF(a,b,...) dprintf(a, b, ##__VA_ARGS__)
#define DHEXDUMP(a,b,c,d,...) dHexDump(a, b, c, d, ##__VA_ARGS__)
#define DUMP_HOST_REGS(l,a,b) {if (D32g_DbgLevel>=l) ahci_dump_host_regs(a,b);}
#define DUMP_PORT_REGS(l,a,b) {if (D32g_DbgLevel>=l) ahci_dump_port_regs(a,b);}
#else
#define DPRINTF(a,b,...)
#define DHEXDUMP(a,b,c,d,...)
#define DUMP_HOST_REGS(l,a,b)
#define DUMP_PORT_REGS(l,a,b)
#endif

/* verbosity console print macros
 * (we use 'i' in ciprintf here to avoid name clash
 * with vprintf-like funcs)
 */
#define ciprintf(a,...)  {if (verbosity > 0) iprintf(a, ##__VA_ARGS__);}
#define ciiprintf(a,...) {if (verbosity > 1) iprintf(a, ##__VA_ARGS__);}

/* adapter number from AD_INFO pointer; mainly for dprintf() purposes */
#define ad_no(ai) ( ( (u32)ai - (u32)ad_infos ) / sizeof(*ai))

#define MakeNear16PtrFromDiff(Base16, Base32, New32) \
  ( ( CastFar16ToULONG(Base16) + ( (ULONG)(New32) - (ULONG)(Base32) ) ) & 0xffff)

#define MakeFar16PtrFromDiff(Base16, Base32, New32) \
  CastULONGToFar16(CastFar16ToULONG(Base16) + ((ULONG)(New32) - (ULONG)(Base32))))

/* Takes the selector from the first parameter, and the offset specified
 * in the second parameter, and returns a flat pointer
 */
extern void *MakeFlatFromNear16(void __far16 *, USHORT);
#pragma aux MakeFlatFromNear16 = \
  "mov ax, bx" \
  "call Far16ToFlat" \
  parm nomemory [eax] [bx] value [eax] modify nomemory exact [eax];

/* stdarg.h macros with explicit far pointers */
typedef char *va_list;
#define va_start(va, last)    va = (va_list) (&last + 1)
#define va_arg(va, type)      ((type *) (va += sizeof(type)))[-1]
#define va_end(va)            va = 0

/* stddef macros */
#define offsetof(s, e) ((u32)&((s *)0)->e)

/* shortcut macros */
#define spin_lock(sl)     KernAcquireSpinLock(&sl)
#define spin_unlock(sl)   KernReleaseSpinLock(&sl)

/* Get AHCI port MMIO base from AD_INFO and port number. */
#define port_base(ai, p) ((u8 *) (ai)->mmio + 0x100 + (p) * 0x80)
#define port_dma_base(ai, p) ((AHCI_PORT_DMA *) ((ai)->ports[(p)].dma_buf))
#define port_dma_base_phys(ai, p) ((ai)->ports[p].dma_buf_phys)

/* Convert an SATA adapter/port/device address into a 16-bit IORB unit handle
 * (and the other way round). The mapping looks like this:
 *
 *  mapping                   comment
 *  -----------------------------------------------------------------------
 *  4 bits for the adapter    current max is 8 adapters
 *  4 bits for the port       AHCI spec defines up to 32 ports
 *  4 bits for the device     SATA spec defines up to 15 devices behind PMP
 */
#define iorb_unit(a, p, d)       ((((u16) (a) & 0x0fU) << 8) | \
                                  (((u16) (p) & 0x0fU) << 4) | \
                                  (((u16) (d) & 0x0fU)))
#define iorb_unit_adapter(iorb)  (((iorb)->UnitHandle >> 8) & 0x07)
#define iorb_unit_port(iorb)     (((iorb)->UnitHandle >> 4) & 0x0f)
#define iorb_unit_device(iorb)   ((iorb)->UnitHandle & 0x0f)

/*******************************************************************************
 * Convenience macros for IORB processing functions
 */
/* is this IORB on driver or port level? */
#define iorb_driver_level(iorb) ((iorb)->CommandCode == IOCC_CONFIGURATION)

/* is this IORB to be inserted at the beginnig of the IORB queue? */
#define iorb_priority(iorb)     ((iorb)->CommandCode == IOCC_DEVICE_CONTROL && \
                                 (iorb)->CommandModifier == IOCM_ABORT))

/* access IORB ADD workspace */
#define add_workspace(iorb)     ((ADD_WORKSPACE *) &(iorb)->ADDWorkSpace)



/******************************************************************************
 * PCI generic IDs and macros
 */
#define PCI_ANY_ID                     0xffffU
#define PCI_VDEVICE(vendor, device)    PCI_VENDOR_ID_##vendor, (device), \
                                       PCI_ANY_ID, PCI_ANY_ID, 0, 0

/******************************************************************************
 * PCI vendor IDs for AHCI adapters known to this driver (copied from Linux
 * pci_ids.h)
 */
#define PCI_VENDOR_ID_AL       0x10b9
#define PCI_VENDOR_ID_AMD      0x1022
#define PCI_VENDOR_ID_AT       0x1259
#define PCI_VENDOR_ID_ATI      0x1002
#define PCI_VENDOR_ID_ATT      0x11c1
#define PCI_VENDOR_ID_CMD      0x1095
#define PCI_VENDOR_ID_CT       0x102c
#define PCI_VENDOR_ID_INTEL    0x8086
#define PCI_VENDOR_ID_INITIO   0x1101
#define PCI_VENDOR_ID_JMICRON  0x197B
#define PCI_VENDOR_ID_MARVELL  0x11ab
#define PCI_VENDOR_ID_NVIDIA   0x10de
#define PCI_VENDOR_ID_PROMISE  0x105a
#define PCI_VENDOR_ID_SI       0x1039
#define PCI_VENDOR_ID_VIA      0x1106

/******************************************************************************
 * PCI class IDs we're interested in (copied from Linux pci_ids.h)
 */
#define PCI_BASE_CLASS_STORAGE          0x01
#define PCI_CLASS_STORAGE_SCSI          0x0100
#define PCI_CLASS_STORAGE_IDE           0x0101
#define PCI_CLASS_STORAGE_FLOPPY        0x0102
#define PCI_CLASS_STORAGE_IPI           0x0103
#define PCI_CLASS_STORAGE_RAID          0x0104
#define PCI_CLASS_STORAGE_SATA          0x0106
#define PCI_CLASS_STORAGE_SATA_AHCI     0x010601
#define PCI_CLASS_STORAGE_SAS           0x0107
#define PCI_CLASS_STORAGE_OTHER         0x0180

/******************************************************************************
 * ANSI color code constants
 */
#define ANSI_CLR_BRIGHT "\x1b[1m"
#define ANSI_CLR_RED    "\x1b[31m"
#define ANSI_CLR_GREEN  "\x1b[32m"
#define ANSI_CLR_BLUE   "\x1b[34m"
#define ANSI_CLR_CYAN   "\x1b[36m"
#define ANSI_CLR_WHITE  "\x1b[37m"
#define ANSI_RESET      "\x1b[0m"


/* ------------------------ typedefs and structures ------------------------ */

/* PCI device information structure; this is used both for scanning and for
 * identification purposes in 'AD_INFO'; based on the Linux pci_device_id
 * structure but hard-wired to use board_* constants for 'driver_data'
 */
typedef struct {
  u16   vendor;          /* PCI device vendor/manufacturer */
  u16   device;          /* PCI device ID inside vendor scope */
  u16   subvendor;       /* subsystem vendor (unused so far) */
  u16   subdevice;       /* subsystem device (unused so far) */
  u32   class;           /* PCI device class */
  u32   class_mask;      /* bits to match when scanning for 'class' */
  u32   board;           /* AHCI controller board type (board_* constants) */
  char  *chipname;       /* human readable chip ID string */
} PCI_ID;

/* IORB queue; since IORB queues are updated at interrupt time, the
 * corresponding pointers (not the data they point to) need to be volatile.
 */
typedef struct {
  IORBH FAR16DATA *volatile vRoot;           /* root of request list */
  IORBH FAR16DATA *volatile vTail;           /* tail of request list */
} IORB_QUEUE;

typedef struct {
    USHORT Cylinders;
    USHORT HeadsPerCylinder;
    USHORT SectorsPerTrack;
    ULONG  TotalSectors;
    char *Method;
} DEV_INFO;

/* port information structure */
typedef struct {
  IORB_QUEUE    iorb_queue;            /* 00 IORB queue for this port */
  unsigned      dev_max     : 4;       /* 08 maximum device number on this port (0..AHCI_MAX_DEVS-1) */
  unsigned      cmd_slot    : 5;       /*    current command slot index (using round-
                                        *    robin indexes to prevent starvation) */

  volatile u32  ncq_cmds;              /* 0c bitmap for NCQ commands issued */
  volatile u32  reg_cmds;              /* 10 bitmap for regular commands issued */
  u32           dma_buf_phys;          /* 14 physical address of DMA scratch buffer */
  u8           *dma_buf;               /* 18 DMA scatch buffers */

  struct {                             /* 1c */
    unsigned allocated :1;        /* if != 0, device is allocated */
    unsigned present   :1;        /* if != 0, device is present */
    unsigned lba48     :1;        /* if != 0, device supports 48-bit LBA */
    unsigned atapi     :1;        /* if != 0, this is an ATAPI device */
    unsigned atapi_16  :1;        /* if != 0, device suports 16-byte cmds */
    unsigned removable :1;        /* if != 0, device has removable media */
    unsigned dev_type  :5;        /* device type (UIB_TYPE_* in iorb.h) */
    unsigned ncq_max   :5;        /* maximum tag number for queued commands */
    unsigned ignored   :1;        /* if != 0, device is not MBR */
    UNITINFO *unit_info;          /* pointer to modified unit info */
    DEV_INFO dev_info;
    char dev_name[AHCI_DEV_NAME_LEN];
  } devs[AHCI_MAX_DEVS];

  u32 unaligned_read_count;
  u32 error_count;
} P_INFO;

/* adapter information structure */
typedef struct {
  PCI_ID       *pci;                   /* 00 pointer to corresponding PCI ID */

  unsigned      port_max : 5;          /* 04 maximum port number (0..AHCI_MAX_PORTS-1) */
  unsigned      cmd_max : 5;           /*    maximum cmd slot number (0-31) */
  unsigned      port_scan_done : 1;    /*    if != 0, port scan already done */
  unsigned      busy : 1;              /*    if != 0, adapter is busy */
  unsigned      hw_ports : 6;          /*    number of ports as reported by the hardware */
  unsigned      int_set:1;             /* interrupt has been set */

  u32           port_map;              /* 08   bitmap of active ports */
  u16           pci_vendor;            /* 0c */
  u16           pci_device;            /* 0e */

  /* initial adapter configuration from BIOS */
  u32           bios_config[HOST_CAP2 / sizeof(u32) + 1];  /* 10  0x24 / 4 + 1 = 0x0a dwords = 0x28 bytes*/

  u32           cap;                   /* 38 working copy of CAP register */
  u32           cap2;                  /* 3c working copy of CAP2 register */
  u32           flags;                 /* 40 adapter flags */

  HRESOURCE     rm_adh;                /* 44 resource handle for adapter */
  HRESOURCE     rm_bars[6];            /* 48 resource handle for MMIO and I/O BARs */

  u16           bus_dev_func;          /* 64 PCI bus number PCI device and function number */
  u8            irq;                   /* 66 interrupt number */
  u8            irq_pin;               /* 67 irq pin */

  u32           mmio_phys;             /* 68 physical address of MMIO region */
  u32           mmio_size;             /* 6c size of MMIO region */
  u8           *mmio;                  /* 70 pointer to this adapter's MMIO region */

  P_INFO        ports[AHCI_MAX_PORTS]; /* 74 SATA ports on this adapter */
} AD_INFO;

/* ADD workspace in IORB (must not exceed 16 bytes) */
typedef struct {
  void (*ppfunc)(IORBH FAR16DATA *vIorb, IORBH *pIorb);         /* 00 post-processing function */
  void         *buf;                   /* 04 response buffer (e.g. for identify cmds) */
  ULONG         timer;                 /* 08 timer for timeout procesing */
  USHORT        blocks;                /* 0c number of blocks to be transferred */
  unsigned short processing    :1;     /* 0e IORB is being processd */
  unsigned short idempotent    :1;     /*    IORB is idempotent (can be retried) */
  unsigned short queued_hw     :1;     /*    IORB has been queued to hardware */
  unsigned short no_ncq        :1;     /*    must not use native command queuing */
  unsigned short is_ncq        :1;     /*    should use native command queueing */
  unsigned short complete      :1;     /*    IORB has completed processing */
  unsigned short unaligned     :1;     /*    unaligned S/G; need to use transfer buffer */
  unsigned short retries       :2;     /*    number of retries for this command */
  unsigned short cmd_slot      :5;     /*    AHCI command slot for this IORB */
} ADD_WORKSPACE;                       /* 10 */

/* sg_memcpy() direction */
typedef enum {
  SG_TO_BUF,                           /* copy from S/G list to buffer */
  BUF_TO_SG                            /* copy from buffer to S/G list */
} SG_MEMCPY_DIRECTION;

/* Define the size of a disk name.  Disk Names are user defined names given to physical disk drives in the system. */
#define DLA_TABLE_SIGNATURE1  0x424D5202L
#define DLA_TABLE_SIGNATURE2  0x44464D50L
#define DISK_NAME_SIZE	  20

typedef struct _DLA_Table_Sector { /* DTS */
     ULONG DLA_Signature1;         /* The magic signature (part 1) of a Drive Letter Assignment Table. */
     ULONG DLA_Signature2;         /* The magic signature (part 2) of a Drive Letter Assignment Table. */
     ULONG DLA_CRC;                /* The 32 bit CRC for this sector.  Calculated assuming that this field and all unused space in the sector is 0. */
     ULONG Disk_Serial_Number;     /* The serial number assigned to this disk. */
     ULONG Boot_Disk_Serial_Number;/* The serial number of the disk used to boot the system.  This is for conflict resolution when multiple volumes
                                      want the same drive letter.  Since LVM.EXE will not let this situation happen, the only way to get this situation
                                      is for the disk to have been altered by something other than LVM.EXE, or if a disk drive has been moved from one
                                      machine to another.  If the drive has been moved, then it should have a different Boot_Disk_Serial_Number.  Thus,
                                      we can tell which disk drive is the "foreign" drive and therefore reject its claim for the drive letter in question.
                                      If we find that all of the claimaints have the same Boot_Disk_Serial_Number, then we must assign drive letters on
                                      a first come, first serve basis.                                          */
     ULONG Install_Flags;          /* Used by the Install program. */
     ULONG Cylinders;
     ULONG Heads_Per_Cylinder;
     ULONG Sectors_Per_Track;
     char  Disk_Name[DISK_NAME_SIZE];  /* The name assigned to the disk containing this sector. */
     UCHAR Reboot;                 /* For use by Install.  Used to keep track of reboots initiated by install. */
     BYTE  Reserved[3];            /* Alignment. */
                                   /* These are the four entries which correspond to the entries in the partition table. */
} DLA_Table_Sector, *PDLA_Table_Sector;

/* -------------------------- function prototypes -------------------------- */

static inline unsigned long readl(void *a)
{
  return *(volatile unsigned long*)a;
}

static inline void writel(void *a, unsigned long v)
{
  *(volatile unsigned long*)a = v;
}

extern void shutdown_driver(void);

/* os2ahci.c */
extern USHORT init_drv(REQPACKET *req);
extern USHORT gen_ioctl(REQPACKET *ioctl);
extern USHORT char_dev_input(REQPACKET *rwrb);
extern USHORT exit_drv(int func);
extern USHORT sr_drv(int func);
extern void add_entry(IORBH FAR16DATA *vIorb);
extern void trigger_engine(void);
extern int trigger_engine_1(void);
extern void send_iorb(IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern void iocc_configuration (IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern void iocc_device_control(IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern void iocc_unit_control(IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern void iocm_device_table(IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern void iocc_geometry(IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern void iocc_execute_io(IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern void iocc_unit_status(IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern void iocc_adapter_passthru(IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern void iorb_queue_add(IORB_QUEUE *queue, IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern int iorb_queue_del(IORB_QUEUE *queue, IORBH FAR16DATA *vIorb);
extern void iorb_seterr(IORBH *pIorb, USHORT error_code);
extern void iorb_done(IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern void iorb_complete(IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern void iorb_requeue(IORBH *pIorb);
extern void aws_free(ADD_WORKSPACE *aws);
extern void lock_adapter(AD_INFO *ai);
extern void unlock_adapter(AD_INFO *ai);
extern void __syscall timeout_callback(ULONG timer_handle, ULONG p1);
extern void __syscall reset_watchdog(ULONG timer_handle, ULONG p1);

/* ahci.c */
extern int ahci_config_caps(AD_INFO *ai);
extern int ahci_save_bios_config(AD_INFO *ai);
extern int ahci_restore_bios_config(AD_INFO *ai);
extern int ahci_restore_initial_config(AD_INFO *ai);
extern AHCI_PORT_CFG *ahci_save_port_config(AD_INFO *ai, int p);
extern void ahci_restore_port_config(AD_INFO *ai, int p, AHCI_PORT_CFG *pc);
extern int ahci_enable_ahci(AD_INFO *ai);
extern int ahci_scan_ports(AD_INFO *ai);
extern int ahci_complete_init(AD_INFO *ai);
extern int ahci_reset_port(AD_INFO *ai, int p, int ei);
extern int ahci_start_port(AD_INFO *ai, int p, int ei);
extern void ahci_start_fis_rx(AD_INFO *ai, int p);
extern void ahci_start_engine(AD_INFO *ai, int p);
extern int ahci_stop_port(AD_INFO *ai, int p);
extern int ahci_stop_fis_rx(AD_INFO *ai, int p);
extern int ahci_stop_engine(AD_INFO *ai, int p);
extern int ahci_port_busy(AD_INFO *ai, int p);
extern void ahci_exec_iorb(IORBH FAR16DATA *vIorb, IORBH *pIorb, int ncq_capable, int (*func)(IORBH FAR16DATA *, IORBH *pIorb, int));
extern void ahci_exec_polled_iorb(IORBH FAR16DATA *vIorb, IORBH *pIorb, int (*func)(IORBH FAR16DATA *, IORBH *pIorb, int), ULONG timeout);
extern int ahci_exec_polled_cmd(AD_INFO *ai, int p, int d, int timeout, int cmd, ...);
extern int ahci_set_dev_idle(AD_INFO *ai, int p, int d, int idle);
extern int ahci_flush_cache(AD_INFO *ai, int p, int d);

extern int ahci_intr(u32 irq);
extern void ahci_port_intr(AD_INFO *ai, int p);
extern void ahci_error_intr(AD_INFO *ai, int p, u32 irq_stat);

extern void ahci_get_geometry(IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern void ahci_unit_ready(IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern void ahci_read(IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern void ahci_verify(IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern void ahci_write(IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern void ahci_execute_cdb(IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern void ahci_execute_ata(IORBH FAR16DATA *vIorb, IORBH *pIorb);
extern void ahci_dump_host_regs(AD_INFO *ai, int bios_regs);
extern void ahci_dump_port_regs(AD_INFO *ai, int p);
extern int ahci_reset_controller(AD_INFO *ai);

extern void sg_memcpy(SCATGATENTRY *sg_list, USHORT sg_cnt, ULONG sg_off, void *buf, USHORT len, SG_MEMCPY_DIRECTION dir);
extern void panic(char *msg);

/* trace.c */
extern void build_user_info(void);

/* pci.c */
extern int add_pci_id(u16 vendor, u16 device);
extern void scan_pci_bus(void);
extern int pci_enable_int(USHORT BusDevFunc);
extern void pci_hack_virtualbox(void);
extern char *vendor_from_id(u16 vendor);
extern char *device_from_id(u16 device);

/* ctxhook.c */
extern void _Syscall restart_ctxhook(ULONG parm);
extern void _Syscall reset_ctxhook(ULONG parm);
extern void _Syscall engine_ctxhook(ULONG parm);

/* apm.c */
extern void apm_init(void);
extern void suspend(void);
extern void resume(void);

/* ioctl.c */
extern USHORT ioctl_get_devlist(REQPACKET *ioctl);
extern USHORT ioctl_passthrough(REQPACKET *ioctl);
extern USHORT ioctl_gen_dsk(REQPACKET *ioctl);
extern USHORT ioctl_smart(REQPACKET *ioctl);


/* ---------------------------- global variables --------------------------- */

extern int thorough_scan; /* if != 0, perform thorough PCI scan */
extern int init_reset; /* if != 0, reset ports during init */
extern int force_write_cache; /* if != 0, force write cache */
extern int verbosity; /* if != 0, show some info during boot */
extern int use_mbr_test;

extern HDRIVER rm_drvh; /* resource manager driver handle */
extern USHORT add_handle; /* adapter device driver handle */
extern char drv_name[]; /* driver name as string ("OS2AHCI") */

extern PCI_ID pci_ids[]; /* SATA adapter PCI IDs */
extern SpinLock_t drv_lock; /* driver-level spinlock */
extern ULONG com_lock; /* debug log spinlock */
extern IORB_QUEUE driver_queue; /* driver-level IORB queue */
extern AD_INFO ad_infos[]; /* adapter information list */
extern int ad_info_cnt; /* number of entries in ad_infos[] */
extern u16 ad_ignore; /* bitmap with adapters to be ignored */
extern int init_complete; /* if != 0, initialization has completed */
extern int suspended; /* indicates if the driver is suspended */
extern int resume_sleep_flag;

/* port restart context hook and input data */
extern ULONG restart_ctxhook_h;
extern volatile u32 ports_to_restart[MAX_AD];

/* port reset context hook and input data */
extern ULONG reset_ctxhook_h;
extern ULONG th_reset_watchdog;
extern volatile u32 ports_to_reset[MAX_AD];
extern IORB_QUEUE abort_queue;

/* trigger engine context hook and input data */
extern ULONG engine_ctxhook_h;

/* apapter/port-specific options saved when parsing the command line */
extern u8 emulate_scsi[MAX_AD][AHCI_MAX_PORTS];
extern u8 enable_ncq[MAX_AD][AHCI_MAX_PORTS];
extern u8 link_speed[MAX_AD][AHCI_MAX_PORTS];
extern u8 link_power[MAX_AD][AHCI_MAX_PORTS];
extern u8 track_size[MAX_AD][AHCI_MAX_PORTS];
extern u8 port_ignore[MAX_AD][AHCI_MAX_PORTS];

#ifdef DEBUG
extern void DumpIorb(IORBH *pIorb);
#endif

