/******************************************************************************
 * ahci.h - AHCI-specific constants for os2ahci.h
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

/* ----------------------------- include files ----------------------------- */

/* -------------------------- macros and constants ------------------------- */

/******************************************************************************
 *  device prefix strings for Resource Manager
 */
#define RM_HD_PREFIX         "HD_(%d,%d) "
#define RM_HD_PREFIX_LEN     (sizeof(RM_HD_PREFIX) - 1)
#define RM_CD_PREFIX         "CD_(%d,%d) "
#define RM_CD_PREFIX_LEN     (sizeof(RM_CD_PREFIX) - 1)
#define RM_TAPE_PREFIX       "TAPE_(%d,%d) "
#define RM_TAPE_PREFIX_LEN   (sizeof(RM_TAPE_PREFIX) - 1)
#define RM_MAX_PREFIX_LEN    RM_TAPE_PREFIX_LEN

/******************************************************************************
 * AHCI flags and constants; those were initially copied from the Linux AHCI
 * driver but converted to macros because enums are 16 bits for OS/2 drivers
 * (unless we use KEE and a 32-bit compiler, which we don't)
 *
 * Changes from the Linux source:
 *
 *  - reduced AHCI_MAX_SG from 168 to 48 because the port-specific DMA scratch
 *    buffer needs to be less than 64K to allow mapping the whole DMA area to a
 *    16-bit memory segment
 *
 *  - added AHCI_MAX_SG_ELEMENT_LEN constant
 *
 *  - replaced much of the top-level size/offset math with real structs and
 *    corresponding sizeof() directives.
 */
#define AHCI_PCI_BAR            5
#define AHCI_MAX_PORTS          16 /* Spec says 32, but we only support 16 */
#define AHCI_MAX_DEVS           8
#define AHCI_MAX_SG             48 /* hardware max is 64K */
#define AHCI_MAX_SG_ELEMENT_LEN (1UL << 22)
#define AHCI_MAX_CMDS           32
#define AHCI_RX_FIS_SZ          256

/* port-specific DMA scratch buffer aligned to 1024 bytes */
#define AHCI_PORT_PRIV_DMA_SZ   (((sizeof(AHCI_PORT_DMA) + 1023U) / 1024U) * 1024U)

#define AHCI_IRQ_ON_SG          (1UL << 31)
#define AHCI_CMD_ATAPI          (1UL << 5)
#define AHCI_CMD_WRITE          (1UL << 6)
#define AHCI_CMD_PREFETCH       (1UL << 7)
#define AHCI_CMD_RESET          (1UL << 8)
#define AHCI_CMD_CLR_BUSY       (1UL << 10)

#define RX_FIS_D2H_REG          0x40 /* offset of D2H Register FIS data */
#define RX_FIS_SDB              0x58 /* offset of SDB FIS data */
#define RX_FIS_UNK              0x60 /* offset of Unknown FIS data */

#define board_ahci              0
#define board_ahci_vt8251       1
#define board_ahci_ign_iferr    2
#define board_ahci_sb600        3
#define board_ahci_mv           4
#define board_ahci_sb700        5 /* for SB700 and SB800 */
#define board_ahci_mcp65        6
#define board_ahci_nopmp        7
#define board_ahci_yesncq       8
#define board_ahci_nosntf       9

/* global controller registers */
#define HOST_CAP                0x00 /* host capabilities */
#define HOST_CTL                0x04 /* global host control */
#define HOST_IRQ_STAT           0x08 /* interrupt status */
#define HOST_PORTS_IMPL         0x0c /* bitmap of implemented ports */
#define HOST_VERSION            0x10 /* AHCI spec. version compliancy */
#define HOST_CCC                0x14 /* Command Completion Coalescing Control */
#define HOST_CCC_PORTS          0x18 /* CCC ports */
#define HOST_EM_LOC             0x1c /* Enclosure Management location */
#define HOST_EM_CTL             0x20 /* Enclosure Management Control */
#define HOST_CAP2               0x24 /* host capabilities, extended */
#define HOST_BOHC               0x28 /* BIOS hand off control and status */

/* HOST_CTL bits */
#define HOST_RESET              (1UL << 0)  /* reset controller; self-clear */
#define HOST_IRQ_EN             (1UL << 1)  /* global IRQ enable */
#define HOST_AHCI_EN            (1UL << 31) /* AHCI enabled */

/* HOST_CAP bits */
#define HOST_CAP_SXS            (1UL << 5)  /* Supports External SATA */
#define HOST_CAP_EMS            (1UL << 6)  /* Enclosure Management support */
#define HOST_CAP_CCC            (1UL << 7)  /* Command Completion Coalescing */
#define HOST_CAP_PART           (1UL << 13) /* Partial state capable */
#define HOST_CAP_SSC            (1UL << 14) /* Slumber state capable */
#define HOST_CAP_PIO_MULTI      (1UL << 15) /* PIO multiple DRQ support */
#define HOST_CAP_FBS            (1UL << 16) /* FIS-based switching support */
#define HOST_CAP_PMP            (1UL << 17) /* Port Multiplier support */
#define HOST_CAP_ONLY           (1UL << 18) /* Supports AHCI mode only */
#define HOST_CAP_CLO            (1UL << 24) /* Command List Override support */
#define HOST_CAP_LED            (1UL << 25) /* Supports activity LED */
#define HOST_CAP_ALPM           (1UL << 26) /* Aggressive Link PM support */
#define HOST_CAP_SSS            (1UL << 27) /* Staggered Spin-up */
#define HOST_CAP_MPS            (1UL << 28) /* Mechanical presence switch */
#define HOST_CAP_SNTF           (1UL << 29) /* SNotification register */
#define HOST_CAP_NCQ            (1UL << 30) /* Native Command Queueing */
#define HOST_CAP_64             (1UL << 31) /* PCI DAC (64-bit DMA) support */

/* HOST_CAP2 bits */
#define HOST_CAP2_BOH           (1UL << 0)  /* BIOS/OS handoff supported */
#define HOST_CAP2_NVMHCI        (1UL << 1)  /* NVMHCI supported */
#define HOST_CAP2_APST          (1UL << 2)  /* Automatic partial to slumber */

/* HOST_BOHC bits */
#define HOST_BOHC_BOS           (1UL << 0)  /* BIOS owned (semaphore bit) */
#define HOST_BOHC_OOS           (1UL << 1)  /* OS owned (semaphore bit) */
#define HOST_BOHC_SOOE          (1UL << 2)  /* SMI on ownership change enable */
#define HOST_BOHC_OOC           (1UL << 3)  /* OS ownership change */
#define HOST_BOHC_BB            (1UL << 4)  /* BIOS is busy changing ownership */

/* registers for each SATA port */
#define PORT_LST_ADDR           0x00 /* command list DMA addr */
#define PORT_LST_ADDR_HI        0x04 /* command list DMA addr hi */
#define PORT_FIS_ADDR           0x08 /* FIS rx buf addr */
#define PORT_FIS_ADDR_HI        0x0c /* FIS rx buf addr hi */
#define PORT_IRQ_STAT           0x10 /* interrupt status */
#define PORT_IRQ_MASK           0x14 /* interrupt enable/disable mask */
#define PORT_CMD                0x18 /* port command */
#define PORT_TFDATA             0x20 /* taskfile data */
#define PORT_SIG                0x24 /* device TF signature */
#define PORT_CMD_ISSUE          0x38 /* command issue */
#define PORT_SCR_STAT           0x28 /* SATA phy register: SStatus */
#define PORT_SCR_CTL            0x2c /* SATA phy register: SControl */
#define PORT_SCR_ERR            0x30 /* SATA phy register: SError */
#define PORT_SCR_ACT            0x34 /* SATA phy register: SActive */
#define PORT_SCR_NTF            0x3c /* SATA phy register: SNotification */

/* PORT_IRQ_{STAT,MASK} bits */
#define PORT_IRQ_COLD_PRES      (1UL << 31) /* cold presence detect */
#define PORT_IRQ_TF_ERR         (1UL << 30) /* task file error */
#define PORT_IRQ_HBUS_ERR       (1UL << 29) /* host bus fatal error */
#define PORT_IRQ_HBUS_DATA_ERR  (1UL << 28) /* host bus data error */
#define PORT_IRQ_IF_ERR         (1UL << 27) /* interface fatal error */
#define PORT_IRQ_IF_NONFATAL    (1UL << 26) /* interface non-fatal error */
#define PORT_IRQ_OVERFLOW       (1UL << 24) /* xfer exhausted available S/G */
#define PORT_IRQ_BAD_PMP        (1UL << 23) /* incorrect port multiplier */
#define PORT_IRQ_PHYRDY         (1UL << 22) /* PhyRdy changed */
#define PORT_IRQ_DEV_ILCK       (1UL << 7)  /* device interlock */
#define PORT_IRQ_CONNECT        (1UL << 6)  /* port connect change status */
#define PORT_IRQ_SG_DONE        (1UL << 5)  /* descriptor processed */
#define PORT_IRQ_UNK_FIS        (1UL << 4)  /* unknown FIS rx'd */
#define PORT_IRQ_SDB_FIS        (1UL << 3)  /* Set Device Bits FIS rx'd */
#define PORT_IRQ_DMAS_FIS       (1UL << 2)  /* DMA Setup FIS rx'd */
#define PORT_IRQ_PIOS_FIS       (1UL << 1)  /* PIO Setup FIS rx'd */
#define PORT_IRQ_D2H_REG_FIS    (1UL << 0)  /* D2H Register FIS rx'd */
#define PORT_IRQ_FREEZE         (PORT_IRQ_HBUS_ERR | PORT_IRQ_IF_ERR   | \
                                 PORT_IRQ_CONNECT  | PORT_IRQ_PHYRDY   | \
                                 PORT_IRQ_UNK_FIS  | PORT_IRQ_BAD_PMP)
#define PORT_IRQ_ERROR          (PORT_IRQ_FREEZE   | PORT_IRQ_TF_ERR   | \
                                 PORT_IRQ_OVERFLOW | PORT_IRQ_HBUS_DATA_ERR)
#define DEF_PORT_IRQ            (PORT_IRQ_ERROR | PORT_IRQ_SG_DONE     | \
                                 PORT_IRQ_SDB_FIS | PORT_IRQ_DMAS_FIS  | \
                                 PORT_IRQ_PIOS_FIS | PORT_IRQ_D2H_REG_FIS)

#define PORT_ERR_X              (1UL << 26) /* Exchanged */
#define PORT_ERR_FIS            (1UL << 25) /* Unknown FIS type */
#define PORT_ERR_TP             (1UL << 24) /* Transport State Transition Error */
#define PORT_ERR_S              (1UL << 23) /* Link Sequence Error */
#define PORT_ERR_H              (1UL << 22) /* Handshake Error */
#define PORT_ERR_CRC            (1UL << 21) /* CRC Error */
#define PORT_ERR_D              (1UL << 20) /* Disparity Error */
#define PORT_ERR_B              (1UL << 19) /* 10B to 8B Decode Error */
#define PORT_ERR_W              (1UL << 18) /* Comm Wake */
#define PORT_ERR_PI             (1UL << 17) /* Phy Internal Error */
#define PORT_ERR_N              (1UL << 16) /* PhyRdy Change */
#define PORT_ERR_IE             (1UL << 11) /* Internal Error */
#define PORT_ERR_P              (1UL << 10) /* Protocol Error */
#define PORT_ERR_C              (1UL << 9)  /* Persistent Communication or Data Integrity Error */
#define PORT_ERR_TD             (1UL << 8)  /* Transient Data Integrity Error */
#define PORT_ERR_M              (1UL << 1)  /* Recovered Communications Error */
#define PORT_ERR_DI             (1UL << 0)  /* Recoverred Data Integrity Error */
#define PORT_ERR_FAIL_BITS      (PORT_ERR_X | PORT_ERR_FIS | PORT_ERR_TP | PORT_ERR_S | PORT_ERR_H | PORT_ERR_CRC | \
                                 PORT_ERR_D | PORT_ERR_B | PORT_ERR_W | PORT_ERR_PI | PORT_ERR_IE | PORT_ERR_P | \
                                 PORT_ERR_C | PORT_ERR_TD)

/* PORT_CMD bits */
#define PORT_CMD_ASP            (1UL << 27) /* Aggressive Slumber/Partial */
#define PORT_CMD_ALPE           (1UL << 26) /* Aggressive Link PM enable */
#define PORT_CMD_ATAPI          (1UL << 24) /* Device is ATAPI */
#define PORT_CMD_PMP            (1UL << 17) /* PMP attached */
#define PORT_CMD_LIST_ON        (1UL << 15) /* cmd list DMA engine running */
#define PORT_CMD_FIS_ON         (1UL << 14) /* FIS DMA engine running */
#define PORT_CMD_FIS_RX         (1UL << 4)  /* Enable FIS receive DMA engine */
#define PORT_CMD_CLO            (1UL << 3)  /* Command list override */
#define PORT_CMD_POWER_ON       (1UL << 2)  /* Power up device */
#define PORT_CMD_SPIN_UP        (1UL << 1)  /* Spin up device */
#define PORT_CMD_START          (1UL << 0)  /* Enable port DMA engine */

#define PORT_CMD_ICC_MASK       (0xfUL << 28) /* i/f ICC state mask */
#define PORT_CMD_ICC_ACTIVE     (0x1UL << 28) /* Put i/f in active state */
#define PORT_CMD_ICC_PARTIAL    (0x2UL << 28) /* Put i/f in partial state */
#define PORT_CMD_ICC_SLUMBER    (0x6UL << 28) /* Put i/f in slumber state */

/* driver status bits */
#define AHCI_HFLAG_NO_NCQ               (1UL << 0)  /* no native cmd queuing */
#define AHCI_HFLAG_IGN_IRQ_IF_ERR       (1UL << 1)  /* ignore IRQ_IF_ERR */
#define AHCI_HFLAG_IGN_SERR_INTERNAL    (1UL << 2)  /* ignore SERR_INTERNAL */
#define AHCI_HFLAG_32BIT_ONLY           (1UL << 3)  /* force 32bit */
#define AHCI_HFLAG_MV_PATA              (1UL << 4)  /* PATA port */
#define AHCI_HFLAG_NO_MSI               (1UL << 5)  /* no PCI MSI */
#define AHCI_HFLAG_NO_PMP               (1UL << 6)  /* no PMP */
#define AHCI_HFLAG_NO_HOTPLUG           (1UL << 7)  /* ignore PxSERR.DIAG.N */
#define AHCI_HFLAG_SECT255              (1UL << 8)  /* max 255 sectors */
#define AHCI_HFLAG_YES_NCQ              (1UL << 9)  /* force NCQ cap on */
#define AHCI_HFLAG_NO_SUSPEND           (1UL << 10) /* don't suspend */
#define AHCI_HFLAG_SRST_TOUT_IS_OFFLINE (1UL << 11) /* treat SRST timeout as
                                                      link offline */
#define AHCI_HFLAG_NO_SNTF              (1UL << 12) /* no sntf */

#define ICH_MAP                         0x90 /* ICH MAP register */

/* em constants */
#define EM_MAX_SLOTS                    8
#define EM_MAX_RETRY                    5

/* em_ctl bits */
#define EM_CTL_RST                      (1UL << 9)  /* Reset */
#define EM_CTL_TM                       (1UL << 8)  /* Transmit Message */
#define EM_CTL_ALHD                     (1UL << 26) /* Activity LED */

/* ------------------------ typedefs and structures ------------------------ */

/* Primitive types
 *
 * Note: Since OS/2 is essentially an x86 OS and this driver, as well as the
 *       interface it's developed for, is based on x86 design patterns, we're
 *       not even going to start making a difference between little and big
 *       endian architectures. PCI is little endian, AHCI is little endian,
 *       x86 is little endian, and that's it.
 */
typedef unsigned char    u8;
typedef unsigned short   u16;
typedef unsigned long    u32;

/* AHCI S/G structure */
typedef struct {
  u32   addr;            /* address of S/G element */
  u32   addr_hi;         /* address of S/G element (upper 32 bits) */
  u32   reserved;
  u32   size;            /* size of S/G element - 1; the high 10 bits are flags:
                          *  31    : interrupt on completion of this S/G
                          *  30-22 : reserved */
} AHCI_SG;

/* AHCI command header */
typedef struct {
  u32   options;         /* command options */
  u32   status;          /* command status */
  u32   tbl_addr;        /* command table address */
  u32   tbl_addr_high;   /* command table address (upper 32 bits) */
  u32   reserved[4];
} AHCI_CMD_HDR;

/* AHCI command table */
typedef struct {
  u8           cmd_fis[64];                /* ATA command FIS */
  u8           atapi_cmd[16];              /* ATAPI command */
  u8           reserved[48];
  AHCI_SG      sg_list[AHCI_MAX_SG];       /* AHCI S/G list */
} AHCI_CMD_TBL;

/* AHCI port DMA scratch area */
typedef struct {
  AHCI_CMD_HDR   cmd_hdr[AHCI_MAX_CMDS];   /* command headers */
  u8             rx_fis[AHCI_RX_FIS_SZ];   /* FIS RX area */
  AHCI_CMD_TBL   cmd_tbl[AHCI_MAX_CMDS];   /* command table */
} AHCI_PORT_DMA;

/* AHCI port BIOS configuration save area */
typedef struct {
  u32   cmd_list;                          /* cmd list base address */
  u32   cmd_list_h;                        /* cmd list base address high */
  u32   fis_rx;                            /* FIS receive buffer */
  u32   fis_rx_h;                          /* FIS receive bufffer high */
  u32   irq_mask;                          /* IRQ mask */
  u32   port_cmd;                          /* port engine status */
} AHCI_PORT_CFG;

