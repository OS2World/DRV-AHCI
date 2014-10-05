/******************************************************************************
 * os2ahci.h - main header file for os2ahci driver
 *
 * Copyright (c) 2011 thi.guten Software Development
 * Copyright (c) 2011 Mensys B.V.
 * Copyright (c) 2013 David Azarewicz
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
#define DEBUG
//#define LEGACY_APM

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

/* NOTE: (Rousseau)
 * The regular dhcalls.h from $(DDK)\base\h also works.
 * The devhelp.h from $(DDK)\base\h produces inline assembler errors.
 * The modified devhelp.h from ..\include works OK and is used because it
 * generates a slightly smaller driver image.
 */
#ifdef __WATCOMC__
/* include WATCOM specific DEVHELP stubs */
#include <devhelp.h>
#else
#include <dhcalls.h>
#endif

#include <addcalls.h>
#include <rmcalls.h>
#include <devclass.h>
#include <devcmd.h>
#include <rmbase.h>

#include "ahci.h"
#include "ahci-idc.h"

/* -------------------------- macros and constants ------------------------- */

#define MAX_AD               8       /* maximum number of adapters */

/* Timer pool size. In theory, we need one timer per outstanding command plus
 * a few miscellaneous timers but it's unlikely we'll ever have outstanding
 * commands on all devices on all ports on all apapters -- this would be
 * 8 * 32 * 32 = 8192 outstanding commands on a maximum of 8 * 32 * 15 = 3840
 * devices and that's a bit of an exaggeration. It should be more than enough
 * to have 128 timers.
 */
#define TIMER_COUNT        128
#define TIMER_POOL_SIZE    (sizeof(ADD_TIMER_POOL) + \
                            TIMER_COUNT * sizeof(ADD_TIMER_DATA))

/* default command timeout (can be overwritten in the IORB) */
#define DEFAULT_TIMEOUT  30000

/* Maximum number of retries for commands in the restart/reset context hooks.
 *
 * Please note that the corresponding variable in the ADD workspace is a bit
 * field, thus increasing this value means increasing the size of the bit
 * field. At the time of writing this comment the 'retries' variable was 2
 * bits wide (i.e. a maximum number of 3 retries) and there was exactly one
 * bit left before the ADD workspace structure would become too large...
 */
#define MAX_RETRIES          3

/* max/min macros */
#define max(a, b)  (a) > (b) ? (a) : (b)
#define min(a, b)  (a) < (b) ? (a) : (b)

/* debug output macros */
#ifdef DEBUG
#define dprintf   if (debug > 0) printf
#define dphex     if (debug > 0) phex
#define ddprintf  if (debug > 1) printf
#define ddphex    if (debug > 1) phex
#define dddprintf if (debug > 2) printf
#define dddphex   if (debug > 2) phex
#define ntprintf printf_nts
#define aprintf printf
#else
#define dprintf(a,...)
#define dphex(a,b,c,...)
#define ddprintf(a,...)
#define ddphex(a,b,c,...)
#define dddprintf(a,...)
#define dddphex(a,b,c,...)
#define ntprintf(a,...)
#define aprintf(a,...)
#endif

/* verbosity console print macros
 * (we use 'i' in ciprintf here to avoid name clash
 * with vprintf-like funcs)
 */
#define ciprintf  if (verbosity > 0) cprintf
#define ciiprintf if (verbosity > 1) cprintf

/* TRACE macros (for our internal ring buffer trace) */
#define AHCI_DEBUG_BUF_SIZE  0x10000UL /* 64k must be a power of 2 */
#define AHCI_INFO_BUF_SIZE  0x1000UL /* 4k must be a power of 2 */

/* adapter number from AD_INFO pointer; mainly for dprintf() purposes */
#define ad_no(ai) (((u16) ai - (u16) ad_infos) / sizeof(*ai))

/* Convert far function address into NPFN (the DDK needs this all over the
 * place and just casting to NPFN will produce a "segment lost in conversion"
 * warning. Since casting to a u32 is a bit nasty for function pointers and
 * might have to be revised for different compilers, we'll use a central
 * macro for this crap.
 */
#define mk_NPFN(func)      (NPFN) (u32) (func)

/* stdarg.h macros with explicit far pointers
 *
 * NOTE: The compiler pushes fixed arguments with 16 bits minimum, thus
 *       the last fixed argument (i.e. the one passed to va_start) must
 *       have at least 16 bits. Otherwise, the address calculation in
 *       va_start() will fail.
 */
typedef char _far *va_list;
#define va_start(va, last)    va = (va_list) (&last + 1)
#define va_arg(va, type)      ((type _far *) (va += sizeof(type)))[-1]
#define va_end(va)            va = 0

/* ctype macros */
#define isupper(ch)           ((ch) >= 'A' && (ch) <= 'Z')
#define tolower(ch)           (isupper(ch) ? (ch) + ('a' - 'A') : (ch))

/* stddef macros */
#define offsetof(s, e)        ((u16) &((s *) 0)->e)

/* SMP spinlock compatibility macros for older DDKs using CLI/STI */
#ifdef SPINLOCK_EMULATION
#define DevHelp_CreateSpinLock(p_sph)  *(p_sph) = 0
#define DevHelp_FreeSpinLock(sph)      0

#define DevHelp_AcquireSpinLock(sph)    if ((sph) != 0)                \
                                         panic("recursive spinlock"); \
                                       (sph) = disable()

#define DevHelp_ReleaseSpinLock(sph)   if (sph) {                     \
                                         (sph) = 0;                   \
                                         enable();                    \
                                       }
#endif

/* shortcut macros */
#define spin_lock(sl)     DevHelp_AcquireSpinLock(sl)
#define spin_unlock(sl)   DevHelp_ReleaseSpinLock(sl)

/* Get AHCI port MMIO base from AD_INFO and port number. For the time being,
 * MMIO addresses are assumed to be valid 16:16 pointers which implies
 * that one GDT selector is allocated per adapter.
 */
#define port_base(ai, p)   ((u8 _far *) (ai)->mmio + 0x100 + (p) * 0x80)

/* Get address of port-specific DMA scratch buffer. The total size of all DMA
 * buffers required for 32 ports exceeds 65536 bytes, thus we need multiple
 * GDT selectors to access all port DMA scratch buffers and some logic to map
 * a port number to the corresponding DMA scratch buffer address.
 */
#define PORT_DMA_BUFS_PER_SEG  ((size_t) (65536UL / AHCI_PORT_PRIV_DMA_SZ))
#define PORT_DMA_BUF_SEGS      ((AHCI_MAX_PORTS + PORT_DMA_BUFS_PER_SEG - 1) \
                                / PORT_DMA_BUFS_PER_SEG)
#define PORT_DMA_SEG_SIZE      ((u32) PORT_DMA_BUFS_PER_SEG * \
                                (u32) AHCI_PORT_PRIV_DMA_SZ)

#define port_dma_base(ai, p) \
  ((AHCI_PORT_DMA _far *) ((ai)->dma_buf[(p) / PORT_DMA_BUFS_PER_SEG] + \
                           ((p) % PORT_DMA_BUFS_PER_SEG) * AHCI_PORT_PRIV_DMA_SZ))

#define port_dma_base_phys(ai, p) \
  ((ai)->dma_buf_phys + (u32) (p) * AHCI_PORT_PRIV_DMA_SZ)

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
#define iorb_unit_adapter(iorb)  (((u16) (iorb)->UnitHandle >> 8) & 0x07U)
#define iorb_unit_port(iorb)     (((u16) (iorb)->UnitHandle >> 4) & 0x0fU)
#define iorb_unit_device(iorb)   ((u16)  (iorb)->UnitHandle       & 0x0fU)

/*******************************************************************************
 * Convenience macros for IORB processing functions
 */
/* is this IORB on driver or port level? */
#define iorb_driver_level(iorb) ((iorb)->CommandCode == IOCC_CONFIGURATION)

/* is this IORB to be inserted at the beginnig of the IORB queue? */
#define iorb_priority(iorb)     ((iorb)->CommandCode == IOCC_DEVICE_CONTROL && \
                                 (iorb)->CommandModifier == IOCM_ABORT))

/* access IORB ADD workspace */
#define add_workspace(iorb)     ((ADD_WORKSPACE _far *) &(iorb)->ADDWorkSpace)



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

typedef unsigned int     size_t;

typedef struct {
  u32 Start;
  u32 End;
} TIMER;

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
  IORBH _far *volatile root;           /* root of request list */
  IORBH _far *volatile tail;           /* tail of request list */
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
  IORB_QUEUE    iorb_queue;            /* IORB queue for this port */
  unsigned      dev_max     : 4;       /* maximum device number on this port (0..AHCI_MAX_DEVS-1) */
  unsigned      cmd_slot    : 5;       /* current command slot index (using round-
                                        * robin indexes to prevent starvation) */

  volatile u32  ncq_cmds;              /* bitmap for NCQ commands issued */
  volatile u32  reg_cmds;              /* bitmap for regular commands issued */

  struct {
    unsigned    allocated : 1;         /* if != 0, device is allocated */
    unsigned    present   : 1;         /* if != 0, device is present */
    unsigned    lba48     : 1;         /* if != 0, device supports 48-bit LBA */
    unsigned    atapi     : 1;         /* if != 0, this is an ATAPI device */
    unsigned    atapi_16  : 1;         /* if != 0, device suports 16-byte cmds */
    unsigned    removable : 1;         /* if != 0, device has removable media */
    unsigned    dev_type  : 5;         /* device type (UIB_TYPE_* in iorb.h) */
    unsigned    ncq_max   : 5;         /* maximum tag number for queued commands */
    UNITINFO _far *unit_info;          /* pointer to modified unit info */
    DEV_INFO    dev_info;
  } devs[AHCI_MAX_DEVS];
} P_INFO;

/* adapter information structure */
typedef struct {
  PCI_ID       *pci;                   /* pointer to corresponding PCI ID */

  unsigned      port_max : 5;          /* maximum port number (0..AHCI_MAX_PORTS-1) */
  unsigned      cmd_max : 5;           /* maximum cmd slot number (0-31) */
  unsigned      port_scan_done : 1;    /* if != 0, port scan already done */
  unsigned      busy : 1;              /* if != 0, adapter is busy */

  unsigned      hw_ports : 6;          /* number of ports as reported by the hardware */

  u32           port_map;              /* bitmap of active ports */
  u16           pci_vendor;
  u16           pci_device;

  /* initial adapter configuration from BIOS */
  u32           bios_config[HOST_CAP2 / sizeof(u32) + 1];

  u32           cap;                   /* working copy of CAP register */
  u32           cap2;                  /* working copy of CAP2 register */
  u32           flags;                 /* adapter flags */

  HRESOURCE     rm_adh;                /* resource handle for adapter */
  HRESOURCE     rm_bars[6];            /* resource handle for MMIO and I/O BARs */
  HRESOURCE     rm_irq;                /* resource handle for IRQ */

  u8            bus;                   /* PCI bus number */
  u8            dev_func;              /* PCI device and function number */
  u16           irq;                   /* interrupt number */

  u32           mmio_phys;             /* physical address of MMIO region */
  u32           mmio_size;             /* size of MMIO region */
  u8      _far *mmio;                  /* pointer to this adapter's MMIO region */

  u32           dma_buf_phys;          /* physical address of DMA scratch buffer */
  u8      _far *dma_buf[PORT_DMA_BUF_SEGS]; /* DMA scatch buffer */

  P_INFO        ports[AHCI_MAX_PORTS]; /* SATA ports on this adapter */
} AD_INFO;

/* ADD workspace in IORB (must not exceed 16 bytes) */
typedef struct {
  void (*ppfunc)(IORBH _far *iorb);    /* post-processing function */
  void         *buf;                   /* response buffer (e.g. for identify cmds) */
  ULONG         timer;                 /* timer for timeout procesing */
  USHORT        blocks;                /* number of blocks to be transferred */
  unsigned      processing    : 1;     /* IORB is being processd */
  unsigned      idempotent    : 1;     /* IORB is idempotent (can be retried) */
  unsigned      queued_hw     : 1;     /* IORB has been queued to hardware */
  unsigned      no_ncq        : 1;     /* must not use native command queuing */
  unsigned      is_ncq        : 1;     /* should use native command queueing */
  unsigned      complete      : 1;     /* IORB has completed processing */
  unsigned      unaligned     : 1;     /* unaligned S/G; need to use transfer buffer */
  unsigned      retries       : 2;     /* number of retries for this command */
  unsigned      cmd_slot      : 5;     /* AHCI command slot for this IORB */
} ADD_WORKSPACE;

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

/* init.asm */
extern u32  _cdecl        readl       (void _far *addr);
extern u32  _cdecl        writel      (void _far *addr, u32 val);
extern void _far * _cdecl memcpy      (void _far *v_dst, void _far *v_src, int len);
extern void _far * _cdecl memset      (void _far *p, int ch, size_t len);
extern void _cdecl _far restart_hook  (void);
extern void _cdecl _far reset_hook    (void);
extern void _cdecl _far engine_hook   (void);
extern void _cdecl _far asm_krnl_exit (void);
extern void _cdecl udelay             (u16 microseconds);

/* os2ahci.c */
extern USHORT  init_drv               (RPINITIN _far *req);
extern USHORT  gen_ioctl              (RP_GENIOCTL _far *ioctl);
extern USHORT  char_dev_input         (RP_RWV _far *rwrb);
extern USHORT  exit_drv               (int func);
extern USHORT  sr_drv                 (int func);
extern void _cdecl _far _loadds add_entry    (IORBH _far *iorb);
extern void    trigger_engine         (void);
extern int     trigger_engine_1       (void);
extern void    send_iorb              (IORBH _far *iorb);
extern void    iocc_configuration     (IORBH _far *iorb);
extern void    iocc_device_control    (IORBH _far *iorb);
extern void    iocc_unit_control      (IORBH _far *iorb);
extern void    iocm_device_table      (IORBH _far *iorb);
extern void    iocc_geometry          (IORBH _far *iorb);
extern void    iocc_execute_io        (IORBH _far *iorb);
extern void    iocc_unit_status       (IORBH _far *iorb);
extern void    iocc_adapter_passthru  (IORBH _far *iorb);
extern void    iorb_queue_add         (IORB_QUEUE _far *queue, IORBH _far *iorb);
extern int     iorb_queue_del         (IORB_QUEUE _far *queue, IORBH _far *iorb);
extern void    iorb_seterr            (IORBH _far *iorb, USHORT error_code);
extern void    iorb_done              (IORBH _far *iorb);
extern void    iorb_complete          (IORBH _far *iorb);
extern void    iorb_requeue           (IORBH _far *iorb);
extern void    aws_free               (ADD_WORKSPACE _far *aws);
extern void    lock_adapter           (AD_INFO *ai);
extern void    unlock_adapter         (AD_INFO *ai);
extern void _cdecl _far timeout_callback    (ULONG timer_handle, ULONG p1, ULONG p2);
extern void _cdecl _far reset_watchdog      (ULONG timer_handle, ULONG p1, ULONG p2);

/* ahci.c */
extern int     ahci_save_bios_config        (AD_INFO *ai);
extern int     ahci_restore_bios_config     (AD_INFO *ai);
extern int     ahci_restore_initial_config  (AD_INFO *ai);
extern AHCI_PORT_CFG *ahci_save_port_config (AD_INFO *ai, int p);
extern void    ahci_restore_port_config     (AD_INFO *ai, int p,
                                             AHCI_PORT_CFG *pc);
extern int     ahci_enable_ahci             (AD_INFO *ai);
extern int     ahci_scan_ports              (AD_INFO *ai);
extern int     ahci_complete_init           (AD_INFO *ai);
extern int     ahci_reset_port              (AD_INFO *ai, int p, int ei);
extern int     ahci_start_port              (AD_INFO *ai, int p, int ei);
extern void    ahci_start_fis_rx            (AD_INFO *ai, int p);
extern void    ahci_start_engine            (AD_INFO *ai, int p);
extern int     ahci_stop_port               (AD_INFO *ai, int p);
extern int     ahci_stop_fis_rx             (AD_INFO *ai, int p);
extern int     ahci_stop_engine             (AD_INFO *ai, int p);
extern int     ahci_port_busy               (AD_INFO *ai, int p);
extern void    ahci_exec_iorb               (IORBH _far *iorb, int ncq_capable,
                                             int (*func)(IORBH _far *, int));
extern void    ahci_exec_polled_iorb        (IORBH _far *iorb,
                                             int (*func)(IORBH _far *, int),
                                             ULONG timeout);
extern int     ahci_exec_polled_cmd         (AD_INFO *ai, int p, int d,
                                             int timeout, int cmd, ...);
extern int     ahci_set_dev_idle            (AD_INFO *ai, int p, int d, int idle);
extern int     ahci_flush_cache             (AD_INFO *ai, int p, int d);

extern int     ahci_intr                    (u16 irq);
extern void    ahci_port_intr               (AD_INFO *ai, int p);
extern void    ahci_error_intr              (AD_INFO *ai, int p, u32 irq_stat);

extern void    ahci_get_geometry            (IORBH _far *iorb);
extern void    ahci_unit_ready              (IORBH _far *iorb);
extern void    ahci_read                    (IORBH _far *iorb);
extern void    ahci_verify                  (IORBH _far *iorb);
extern void    ahci_write                   (IORBH _far *iorb);
extern void    ahci_execute_cdb             (IORBH _far *iorb);
extern void    ahci_execute_ata             (IORBH _far *iorb);
extern void ahci_dump_host_regs(AD_INFO *ai, int bios_regs);
extern void ahci_dump_port_regs(AD_INFO *ai, int p);
extern int ahci_reset_controller(AD_INFO *ai);

/* libc.c */
extern void        init_libc     (void);
extern void        init_com      (long BaudRate);
extern int         vsprintf      (char _far *buf, const char *fmt, va_list va);
extern int         sprintf       (char _far *buf, const char *fmt, ...);
extern void        vfprintf      (const char *fmt, va_list va);
extern void _cdecl printf        (const char *fmt, ...);
extern void _cdecl printf_nts    (const char *fmt, ...);
extern void        cprintf       (const char *fmt, ...);
extern void        phex          (const void _far *p, int len, const char *fmt, ...);
extern size_t      strlen        (const char _far *s);
extern char _far  *strcpy        (char _far *dst, const char _far *src);
extern int         memcmp        (void _far *p1, void _far *p2, size_t len);
extern void        sg_memcpy     (SCATGATENTRY _far *sg_list, USHORT sg_cnt,
                                  ULONG sg_off, void _far *buf, USHORT len,
                                  SG_MEMCPY_DIRECTION dir);
extern long        strtol        (const char _far *buf,
                                  const char _far * _far *ep, int base);
extern void       *malloc        (size_t len);
extern void        free          (void *ptr);
extern ULONG       virt_to_phys  (void _far *ptr);
extern void        msleep        (u32 millies);
extern void        panic         (char *msg);
extern int         disable       (void);
extern void        enable        (void);
extern void timer_init(TIMER far *pTimer, u32 Milliseconds);
extern int timer_check_and_block(TIMER far *pTimer);

/* trace.c */
extern void        trace_init    (u32);
extern void        trace_exit    (void);
extern void        trace_write   (u8 _far *s, int len);
extern u16         trace_read    (u8 _far *buf, u16 cb_buf);
extern u16         trace_char_dev(RP_RWV _far *rwrb);
extern void        build_user_info(int check);

/* pci.c */
extern int       add_pci_id     (u16 vendor, u16 device);
extern void      scan_pci_bus   (void);
extern int       pci_enable_int (UCHAR bus, UCHAR dev_func);
extern void      pci_hack_virtualbox(void);
extern char     *vendor_from_id (u16 vendor);
extern char     *device_from_id (u16 device);
UCHAR pci_read_conf    (UCHAR bus, UCHAR dev_func, UCHAR indx,
                        UCHAR size, ULONG _far *val);
UCHAR pci_write_conf   (UCHAR bus, UCHAR dev_func, UCHAR indx, UCHAR size,
                        ULONG val);

/* ctxhook.c */
extern void _cdecl restart_ctxhook    (ULONG parm);
extern void _cdecl reset_ctxhook      (ULONG parm);
extern void _cdecl engine_ctxhook     (ULONG parm);

/* apm.c */
extern void        apm_init           (void);
extern void        suspend        (void);
extern void        resume         (void);

/* ioctl.c */
extern USHORT      ioctl_get_devlist  (RP_GENIOCTL _far *ioctl);
extern USHORT      ioctl_passthrough  (RP_GENIOCTL _far *ioctl);
extern USHORT      ioctl_gen_dsk      (RP_GENIOCTL _far *ioctl);
extern USHORT      ioctl_smart        (RP_GENIOCTL _far *ioctl);


/* ---------------------------- global variables --------------------------- */

extern char _cdecl       end_of_data;   /* label at the end of all data segments */
extern void _cdecl _near end_of_code(); /* label at the end of all code segments */

extern int           debug;         /* if != 0, print debug messages to COM1 */
extern int           thorough_scan; /* if != 0, perform thorough PCI scan */
extern int           init_reset;    /* if != 0, reset ports during init */
extern int           force_write_cache; /* if != 0, force write cache */
extern int           verbosity;     /* if != 0, show some info during boot */
extern int           use_lvm_info;
extern int           wrap_trace_buffer;

extern HDRIVER       rm_drvh;       /* resource manager driver handle */
extern USHORT        add_handle;    /* adapter device driver handle */
extern UCHAR         timer_pool[];  /* timer pool */
extern char          drv_name[];    /* driver name as string ("OS2AHCI") */

extern PCI_ID        pci_ids[];     /* SATA adapter PCI IDs */
extern ULONG         drv_lock;      /* driver-level spinlock */
extern ULONG         com_lock;      /* debug log spinlock */
extern volatile PGINFOSEG gis;
extern IORB_QUEUE    driver_queue;  /* driver-level IORB queue */
extern AD_INFO       ad_infos[];    /* adapter information list */
extern int           ad_info_cnt;   /* number of entries in ad_infos[] */
extern u16           ad_ignore;     /* bitmap with adapters to be ignored */
extern int           init_complete; /* if != 0, initialization has completed */
extern int           suspended;     /* indicates if the driver is suspended */
extern int           resume_sleep_flag;

extern u16           com_base;      /* debug COM port base address */

/* port restart context hook and input data */
extern ULONG         restart_ctxhook_h;
extern volatile u32  ports_to_restart[MAX_AD];

/* port reset context hook and input data */
extern ULONG         reset_ctxhook_h;
extern ULONG         th_reset_watchdog;
extern volatile u32  ports_to_reset[MAX_AD];
extern IORB_QUEUE    abort_queue;

/* trigger engine context hook and input data */
extern ULONG         engine_ctxhook_h;

/* apapter/port-specific options saved when parsing the command line */
extern u8            emulate_scsi[MAX_AD][AHCI_MAX_PORTS];
extern u8            enable_ncq[MAX_AD][AHCI_MAX_PORTS];
extern u8            link_speed[MAX_AD][AHCI_MAX_PORTS];
extern u8            link_power[MAX_AD][AHCI_MAX_PORTS];
extern u8            track_size[MAX_AD][AHCI_MAX_PORTS];
extern u8            port_ignore[MAX_AD][AHCI_MAX_PORTS];

