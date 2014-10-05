/******************************************************************************
 * ata.h - ATA structures and macros for os2ahci driver
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

/******************************************************************************
 * Right now, os2ahci uses a fixed sector size of 512 bytes for hard disks.
 * This may change in the future...
 */
#define ATA_SECTOR_SIZE    512

/******************************************************************************
 * Macros to access geometry values in the ATA ID buffer
 */
#define ATA_CYLS(id_buf)            *((u16 *) (id_buf + ATA_ID_CYLS))
#define ATA_HEADS(id_buf)           *((u16 *) (id_buf + ATA_ID_HEADS))
#define ATA_SECTORS(id_buf)         *((u16 *) (id_buf + ATA_ID_SECTORS))
#define ATA_CAPACITY(id_buf)        *((u32 *) (id_buf + ATA_ID_LBA_CAPACITY))

#define ATA_CAPACITY48_L(id_buf)    *((u32 *) (id_buf + ATA_ID_LBA_CAPACITY_2))
#define ATA_CAPACITY48_H(id_buf)    *((u32 *) (id_buf + ATA_ID_LBA_CAPACITY_2 + 2))

#define CUR_CYLS(id_buf)            *((u16 *) (id_buf + ATA_ID_CUR_CYLS))
#define CUR_HEADS(id_buf)           *((u16 *) (id_buf + ATA_ID_CUR_HEADS))
#define CUR_SECTORS(id_buf)         *((u16 *) (id_buf + ATA_ID_CUR_SECTORS))
#define CUR_CAPACITY(id_buf)        *((u32 *) (id_buf + ATA_ID_CUR_CAPACITY))


/******************************************************************************
 * ATA IDENTIFY offsets and constants. The ATA IDENTIFY response is a bit of a
 * mess and after struggling to put this into some structure, I discovered the
 * Linux folks simply defined offsets and constants to those fields which are
 * actually needed. Thus, I copied the relevant bits from ata.h, replacing
 * the enum with precompiler definitions because our enums are signed 16 bits.
 *
 * Please note that strings such as the product name are stored using byte-
 * swapped 16-bit words.
 */
#define ATA_ID_WORDS            256
#define ATA_ID_CONFIG           0
#define ATA_ID_CYLS             1
#define ATA_ID_HEADS            3
#define ATA_ID_SECTORS          6
#define ATA_ID_SERNO            10
#define ATA_ID_BUF_SIZE         21
#define ATA_ID_FW_REV           23
#define ATA_ID_PROD             27
#define ATA_ID_MAX_MULTSECT     47
#define ATA_ID_DWORD_IO         48
#define ATA_ID_CAPABILITY       49
#define ATA_ID_OLD_PIO_MODES    51
#define ATA_ID_OLD_DMA_MODES    52
#define ATA_ID_FIELD_VALID      53
#define ATA_ID_CUR_CYLS         54
#define ATA_ID_CUR_HEADS        55
#define ATA_ID_CUR_SECTORS      56
#define ATA_ID_CUR_CAPACITY     57
#define ATA_ID_MULTSECT         59
#define ATA_ID_LBA_CAPACITY     60
#define ATA_ID_SWDMA_MODES      62
#define ATA_ID_MWDMA_MODES      63
#define ATA_ID_PIO_MODES        64
#define ATA_ID_EIDE_DMA_MIN     65
#define ATA_ID_EIDE_DMA_TIME    66
#define ATA_ID_EIDE_PIO         67
#define ATA_ID_EIDE_PIO_IORDY   68
#define ATA_ID_ADDITIONAL_SUPP  69
#define ATA_ID_QUEUE_DEPTH      75
#define ATA_ID_MAJOR_VER        80
#define ATA_ID_COMMAND_SET_1    82
#define ATA_ID_COMMAND_SET_2    83
#define ATA_ID_CFSSE            84
#define ATA_ID_CFSSE            84
#define ATA_ID_CFS_ENABLE_1     85
#define ATA_ID_CFS_ENABLE_2     86
#define ATA_ID_CSF_DEFAULT      87
#define ATA_ID_UDMA_MODES       88
#define ATA_ID_HW_CONFIG        93
#define ATA_ID_SPG              98
#define ATA_ID_LBA_CAPACITY_2   100
#define ATA_ID_SECTOR_SIZE      106
#define ATA_ID_LAST_LUN         126
#define ATA_ID_DLF              128
#define ATA_ID_CSFO             129
#define ATA_ID_CFA_POWER        160
#define ATA_ID_CFA_KEY_MGMT     162
#define ATA_ID_CFA_MODES        163
#define ATA_ID_DATA_SET_MGMT    169
#define ATA_ID_ROT_SPEED        217
#define ATA_ID_PIO4             (1U << 1)

#define ATA_ID_SERNO_LEN        20
#define ATA_ID_FW_REV_LEN       8
#define ATA_ID_PROD_LEN         40

#define ATA_PCI_CTL_OFS         2

#define ATA_PIO0                (1U << 0)
#define ATA_PIO1                ATA_PIO0 | (1U << 1)
#define ATA_PIO2                ATA_PIO1 | (1U << 2)
#define ATA_PIO3                ATA_PIO2 | (1U << 3)
#define ATA_PIO4                ATA_PIO3 | (1U << 4)
#define ATA_PIO5                ATA_PIO4 | (1U << 5)
#define ATA_PIO6                ATA_PIO5 | (1U << 6)

#define ATA_PIO4_ONLY           (1U << 4)

#define ATA_SWDMA0              (1U << 0)
#define ATA_SWDMA1              ATA_SWDMA0 | (1U << 1)
#define ATA_SWDMA2              ATA_SWDMA1 | (1U << 2)

#define ATA_SWDMA2_ONLY         (1U << 2)

#define ATA_MWDMA0              (1U << 0)
#define ATA_MWDMA1              ATA_MWDMA0 | (1U << 1)
#define ATA_MWDMA2              ATA_MWDMA1 | (1U << 2)
#define ATA_MWDMA3              ATA_MWDMA2 | (1U << 3)
#define ATA_MWDMA4              ATA_MWDMA3 | (1U << 4)

#define ATA_MWDMA12_ONLY        (1U << 1) | (1U << 2)
#define ATA_MWDMA2_ONLY         (1U << 2)

#define ATA_UDMA0               (1U << 0)
#define ATA_UDMA1               ATA_UDMA0 | (1U << 1)
#define ATA_UDMA2               ATA_UDMA1 | (1U << 2)
#define ATA_UDMA3               ATA_UDMA2 | (1U << 3)
#define ATA_UDMA4               ATA_UDMA3 | (1U << 4)
#define ATA_UDMA5               ATA_UDMA4 | (1U << 5)
#define ATA_UDMA6               ATA_UDMA5 | (1U << 6)

/******************************************************************************
 * Miscellanous constants copied from the Linux LIBATA code. Again, the enums
 * have been converted to preprocessor definitions to avoid problems with
 * signed 16-bit integers.
 *
 * Please note that Linux supports all kinds of IDE/EIDE/ATA/SATA/... devices
 * in LIBATA while we only need the ATA-8 bits, and those only as far as they
 * are relevant to AHCI.
 */

/* bits in ATA command block registers */
#define ATA_HOB                 (1U << 7)     /* LBA48 selector */
#define ATA_NIEN                (1U << 1)     /* disable-irq flag */
#define ATA_LBA                 (1U << 6)     /* LBA28 selector */
#define ATA_DEV1                (1U << 4)     /* Select Device 1 (slave) */
#define ATA_DEVICE_OBS          (1U << 7) | (1U << 5) /* obs bits in dev reg */
#define ATA_DEVCTL_OBS          (1U << 3)     /* obsolete bit in devctl reg */
#define ATA_BUSY                (1U << 7)     /* BSY status bit */
#define ATA_DRDY                (1U << 6)     /* device ready */
#define ATA_DF                  (1U << 5)     /* device fault */
#define ATA_DSC                 (1U << 4)     /* drive seek complete */
#define ATA_DRQ                 (1U << 3)     /* data request i/o */
#define ATA_CORR                (1U << 2)     /* corrected data error */
#define ATA_IDX                 (1U << 1)     /* index */
#define ATA_ERR                 (1U << 0)     /* have an error */
#define ATA_SRST                (1U << 2)     /* software reset */
#define ATA_ICRC                (1U << 7)     /* interface CRC error */
#define ATA_BBK                 ATA_ICRC      /* pre-EIDE: block marked bad */
#define ATA_UNC                 (1U << 6)     /* uncorrectable media error */
#define ATA_MC                  (1U << 5)     /* media changed */
#define ATA_IDNF                (1U << 4)     /* ID not found */
#define ATA_MCR                 (1U << 3)     /* media change requested */
#define ATA_ABORTED             (1U << 2)     /* command aborted */
#define ATA_TRK0NF              (1U << 1)     /* track 0 not found */
#define ATA_AMNF                (1U << 0)     /* address mark not found */
#define ATAPI_LFS               0xF0U         /* last failed sense */
#define ATAPI_EOM               ATA_TRK0NF    /* end of media */
#define ATAPI_ILI               ATA_AMNF      /* illegal length indication */
#define ATAPI_IO                (1U << 1)
#define ATAPI_COD               (1U << 0)

/* ATA device commands */
#define ATA_CMD_DEV_RESET               0x08  /* ATAPI device reset */
#define ATA_CMD_CHK_POWER               0xE5  /* check power mode */
#define ATA_CMD_STANDBY                 0xE2  /* place in standby power mode */
#define ATA_CMD_IDLE                    0xE3  /* place in idle power mode */
#define ATA_CMD_EDD                     0x90  /* execute device diagnostic */
#define ATA_CMD_DOWNLOAD_MICRO          0x92
#define ATA_CMD_NOP                     0x00
#define ATA_CMD_FLUSH                   0xE7
#define ATA_CMD_FLUSH_EXT               0xEA
#define ATA_CMD_ID_ATA                  0xEC
#define ATA_CMD_ID_ATAPI                0xA1
#define ATA_CMD_SERVICE                 0xA2
#define ATA_CMD_READ                    0xC8
#define ATA_CMD_READ_EXT                0x25
#define ATA_CMD_READ_QUEUED             0x26
#define ATA_CMD_READ_STREAM_EXT         0x2B
#define ATA_CMD_READ_STREAM_DMA_EXT     0x2A
#define ATA_CMD_WRITE                   0xCA
#define ATA_CMD_WRITE_EXT               0x35
#define ATA_CMD_WRITE_QUEUED            0x36
#define ATA_CMD_WRITE_STREAM_EXT        0x3B
#define ATA_CMD_WRITE_STREAM_DMA_EXT    0x3A
#define ATA_CMD_WRITE_FUA_EXT           0x3D
#define ATA_CMD_WRITE_QUEUED_FUA_EXT    0x3E
#define ATA_CMD_FPDMA_READ              0x60
#define ATA_CMD_FPDMA_WRITE             0x61
#define ATA_CMD_PIO_READ                0x20
#define ATA_CMD_PIO_READ_EXT            0x24
#define ATA_CMD_PIO_WRITE               0x30
#define ATA_CMD_PIO_WRITE_EXT           0x34
#define ATA_CMD_READ_MULTI              0xC4
#define ATA_CMD_READ_MULTI_EXT          0x29
#define ATA_CMD_WRITE_MULTI             0xC5
#define ATA_CMD_WRITE_MULTI_EXT         0x39
#define ATA_CMD_WRITE_MULTI_FUA_EXT     0xCE
#define ATA_CMD_SET_FEATURES            0xEF
#define ATA_CMD_SET_MULTI               0xC6
#define ATA_CMD_PACKET                  0xA0
#define ATA_CMD_VERIFY                  0x40
#define ATA_CMD_VERIFY_EXT              0x42
#define ATA_CMD_WRITE_UNCORR_EXT        0x45
#define ATA_CMD_STANDBYNOW1             0xE0
#define ATA_CMD_IDLEIMMEDIATE           0xE1
#define ATA_CMD_SLEEP                   0xE6
#define ATA_CMD_INIT_DEV_PARAMS         0x91
#define ATA_CMD_READ_NATIVE_MAX         0xF8
#define ATA_CMD_READ_NATIVE_MAX_EXT     0x27
#define ATA_CMD_SET_MAX                 0xF9
#define ATA_CMD_SET_MAX_EXT             0x37
#define ATA_CMD_READ_LOG_EXT            0x2F
#define ATA_CMD_WRITE_LOG_EXT           0x3F
#define ATA_CMD_READ_LOG_DMA_EXT        0x47
#define ATA_CMD_WRITE_LOG_DMA_EXT       0x57
#define ATA_CMD_TRUSTED_RCV             0x5C
#define ATA_CMD_TRUSTED_RCV_DMA         0x5D
#define ATA_CMD_TRUSTED_SND             0x5E
#define ATA_CMD_TRUSTED_SND_DMA         0x5F
#define ATA_CMD_PMP_READ                0xE4
#define ATA_CMD_PMP_WRITE               0xE8
#define ATA_CMD_CONF_OVERLAY            0xB1
#define ATA_CMD_SEC_SET_PASS            0xF1
#define ATA_CMD_SEC_UNLOCK              0xF2
#define ATA_CMD_SEC_ERASE_PREP          0xF3
#define ATA_CMD_SEC_ERASE_UNIT          0xF4
#define ATA_CMD_SEC_FREEZE_LOCK         0xF5
#define ATA_CMD_SEC_DISABLE_PASS        0xF6
#define ATA_CMD_CONFIG_STREAM           0x51
#define ATA_CMD_SMART                   0xB0
#define ATA_CMD_MEDIA_LOCK              0xDE
#define ATA_CMD_MEDIA_UNLOCK            0xDF
#define ATA_CMD_DSM                     0x06
#define ATA_CMD_CHK_MED_CRD_TYP         0xD1
#define ATA_CMD_CFA_REQ_EXT_ERR         0x03
#define ATA_CMD_CFA_WRITE_NE            0x38
#define ATA_CMD_CFA_TRANS_SECT          0x87
#define ATA_CMD_CFA_ERASE               0xC0
#define ATA_CMD_CFA_WRITE_MULT_NE       0xCD
/* marked obsolete in the ATA/ATAPI-7 spec */
#define ATA_CMD_RESTORE                 0x10

/* READ_LOG_EXT pages */
#define ATA_LOG_SATA_NCQ                0x10

/* READ/WRITE LONG (obsolete) */
#define ATA_CMD_READ_LONG               0x22
#define ATA_CMD_READ_LONG_ONCE          0x23
#define ATA_CMD_WRITE_LONG              0x32
#define ATA_CMD_WRITE_LONG_ONCE         0x33

/* SETFEATURES stuff */
#define SETFEATURES_XFER        0x03
#define XFER_UDMA_7             0x47
#define XFER_UDMA_6             0x46
#define XFER_UDMA_5             0x45
#define XFER_UDMA_4             0x44
#define XFER_UDMA_3             0x43
#define XFER_UDMA_2             0x42
#define XFER_UDMA_1             0x41
#define XFER_UDMA_0             0x40
#define XFER_MW_DMA_4           0x24  /* CFA only */
#define XFER_MW_DMA_3           0x23  /* CFA only */
#define XFER_MW_DMA_2           0x22
#define XFER_MW_DMA_1           0x21
#define XFER_MW_DMA_0           0x20
#define XFER_SW_DMA_2           0x12
#define XFER_SW_DMA_1           0x11
#define XFER_SW_DMA_0           0x10
#define XFER_PIO_6              0x0E  /* CFA only */
#define XFER_PIO_5              0x0D  /* CFA only */
#define XFER_PIO_4              0x0C
#define XFER_PIO_3              0x0B
#define XFER_PIO_2              0x0A
#define XFER_PIO_1              0x09
#define XFER_PIO_0              0x08
#define XFER_PIO_SLOW           0x00

#define SETFEATURES_WC_ON       0x02  /* Enable write cache */
#define SETFEATURES_WC_OFF      0x82  /* Disable write cache */

/* Enable/Disable Automatic Acoustic Management */
#define SETFEATURES_AAM_ON      0x42
#define SETFEATURES_AAM_OFF     0xC2

#define SETFEATURES_SPINUP      0x07  /* Spin-up drive */

#define SETFEATURES_SATA_ENABLE  0x10  /* Enable use of SATA feature */
#define SETFEATURES_SATA_DISABLE 0x90  /* Disable use of SATA feature */

/* SETFEATURE Sector counts for SATA features */
#define SATA_FPDMA_OFFSET       0x01  /* FPDMA non-zero buffer offsets */
#define SATA_FPDMA_AA           0x02  /* FPDMA Setup FIS Auto-Activate */
#define SATA_DIPM               0x03  /* Device Initiated Power Management */
#define SATA_FPDMA_IN_ORDER     0x04  /* FPDMA in-order data delivery */
#define SATA_AN                 0x05  /* Asynchronous Notification */
#define SATA_SSP                0x06  /* Software Settings Preservation */

/* feature values for SET_MAX */
#define ATA_SET_MAX_ADDR        0x00
#define ATA_SET_MAX_PASSWD      0x01
#define ATA_SET_MAX_LOCK        0x02
#define ATA_SET_MAX_UNLOCK      0x03
#define ATA_SET_MAX_FREEZE_LOCK 0x04

/* feature values for DEVICE CONFIGURATION OVERLAY */
#define ATA_DCO_RESTORE         0xC0
#define ATA_DCO_FREEZE_LOCK     0xC1
#define ATA_DCO_IDENTIFY        0xC2
#define ATA_DCO_SET             0xC3

/* feature values for SMART */
#define ATA_SMART_READ_VALUES       0xD0
#define ATA_SMART_READ_THRESHOLDS   0xD1
#define ATA_SMART_AUTOSAVE          0xD2
#define ATA_SMART_SAVE              0xD3
#define ATA_SMART_IMMEDIATE_OFFLINE 0xD4
#define ATA_SMART_READ_LOG          0xD5
#define ATA_SMART_WRITE_LOG         0xD6
#define ATA_SMART_ENABLE            0xD8
#define ATA_SMART_DISABLE           0xD9
#define ATA_SMART_STATUS            0xDA
#define ATA_SMART_AUTO_OFFLINE      0xDB

/* feature values for Data Set Management */
#define ATA_DSM_TRIM            0x01

/* password used in LBA Mid / LBA High for executing SMART commands */
#define ATA_SMART_LBAM_PASS     0x4F
#define ATA_SMART_LBAH_PASS     0xC2

/* ATAPI stuff */
#define ATAPI_PKT_DMA           (1U << 0)
#define ATAPI_DMADIR            (1U << 2)      /* ATAPI data dir:
                                                  0=to device, 1=to host */
#define ATAPI_CDB_LEN           16

/* PMP stuff */
#define SATA_PMP_MAX_PORTS      15
#define SATA_PMP_CTRL_PORT      15

#define SATA_PMP_GSCR_DWORDS    128
#define SATA_PMP_GSCR_PROD_ID   0
#define SATA_PMP_GSCR_REV       1
#define SATA_PMP_GSCR_PORT_INFO 2
#define SATA_PMP_GSCR_ERROR     32
#define SATA_PMP_GSCR_ERROR_EN  33
#define SATA_PMP_GSCR_FEAT      64
#define SATA_PMP_GSCR_FEAT_EN   96

#define SATA_PMP_PSCR_STATUS    0
#define SATA_PMP_PSCR_ERROR     1
#define SATA_PMP_PSCR_CONTROL   2

#define SATA_PMP_FEAT_BIST      (1U << 0)
#define SATA_PMP_FEAT_PMREQ     (1U << 1)
#define SATA_PMP_FEAT_DYNSSC    (1U << 2)
#define SATA_PMP_FEAT_NOTIFY    (1U << 3)

/* SATA Status and Control Registers */
#define SCR_STATUS              0
#define SCR_ERROR               1
#define SCR_CONTROL             2
#define SCR_ACTIVE              3
#define SCR_NOTIFICATION        4

/* SError bits */
#define SERR_DATA_RECOVERED     (1UL << 0)  /* recovered data error */
#define SERR_COMM_RECOVERED     (1UL << 1)  /* recovered comm failure */
#define SERR_DATA               (1UL << 8)  /* unrecovered data error */
#define SERR_PERSISTENT         (1UL << 9)  /* persistent data/comm error */
#define SERR_PROTOCOL           (1UL << 10)  /* protocol violation */
#define SERR_INTERNAL           (1UL << 11)  /* host internal error */
#define SERR_PHYRDY_CHG         (1UL << 16)  /* PHY RDY changed */
#define SERR_PHY_INT_ERR        (1UL << 17)  /* PHY internal error */
#define SERR_COMM_WAKE          (1UL << 18)  /* Comm wake */
#define SERR_10B_8B_ERR         (1UL << 19)  /* 10b to 8b decode error */
#define SERR_DISPARITY          (1UL << 20)  /* Disparity */
#define SERR_CRC                (1UL << 21)  /* CRC error */
#define SERR_HANDSHAKE          (1UL << 22)  /* Handshake error */
#define SERR_LINK_SEQ_ERR       (1UL << 23)  /* Link sequence error */
#define SERR_TRANS_ST_ERROR     (1UL << 24)  /* Transport state trans. error */
#define SERR_UNRECOG_FIS        (1UL << 25)  /* Unrecognized FIS */
#define SERR_DEV_XCHG           (1UL << 26)  /* device exchanged */

/******************************************************************************
 * Parameters for ATA commands. Those parameters are passed in a variable
 * argument list in the format:
 *
 *   AP_xxx, [val, ...], AP_xxx, [val, ...]
 *
 * The values expected by each parameter are indicated within square brackets.
 */
typedef enum {
  AP_FEATURES,     /* [u16]               ATA command features (read: flags) */
  AP_COUNT,        /* [u16]               number of sectors (0 = 65536)      */
  AP_SECTOR_28,    /* [u32]               28-bit sector address              */
  AP_SECTOR_48,    /* [u32, u16]          48-bit sector address              */
  AP_DEVICE,       /* [u16]               ATA cmd "device" field             */
  AP_SGLIST,       /* [void _far *, u16]  buffer S/G (SCATGATENTRY/count)    */
  AP_VADDR,        /* [void _far *, u16]  buffer virtual address (buf/len)   */
  AP_WRITE,        /* [u16]               if != 0, data is written to device */
  AP_AHCI_FLAGS,   /* [u16]               AHCI command header flags          */
  AP_ATAPI_CMD,    /* [void _far *, u16]  ATAPI command (CDB) and length     */
  AP_ATA_CMD,      /* [void _far *]       ATA command (fixed len)            */
  AP_END           /* []                  end of variable argument list      */
} ATA_PARM;

/******************************************************************************
 * Return codes for ata_cmd(); please note that positive return codes indicate
 * that not all S/G elements could be mapped, 0 means success and negative
 * values indicate error conditions.
 */
#define ATA_CMD_SUCCESS             0
#define ATA_CMD_INVALID_PARM       -1
#define ATA_CMD_UNALIGNED_ADDR     -2

/* ------------------------ typedefs and structures ------------------------ */

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
  u16  features;          /* feature bits */
  u16  count;             /* count register (e.g. number of sectors) */
  u32  lba_l;             /* low 24 bits of LBA-28 (32 bits for 48-bit devices) */
  u16  lba_h;             /* high 16 bits of LBA for 48-bit devices */
  u8   cmd;               /* ATA command field */
  u8   device;            /* ATA device field and upper 4 bits of LBA-28 */
} ATA_CMD;

/* generic ATA-8 response structure */
typedef struct {
  u16  error;             /* error code, if any */
  u16  count;             /* count register (e.g. number of sectors) */
  u32  lba_l;             /* low 28 bits of LBA (32 bits for 48-bit devices) */
  u16  lba_h;             /* high 16 bits of LBA for 48-bit devices */
  u16  status;            /* response status bits */
} ATA_RSP;

/******************************************************************************
 * Command-specific ATA-8 structures.
 */

/* -------------------------- function prototypes -------------------------- */

extern int       ata_cmd                  (AD_INFO *ai, int port, int device,
                                           int slot, int cmd, ...);
extern int       v_ata_cmd                (AD_INFO *ai, int port, int device,
                                           int slot, int cmd, va_list va);
extern void      ata_cmd_to_fis           (u8 _far *fis, ATA_CMD _far *cmd,
                                           int device);
extern USHORT    ata_get_sg_indx          (IORB_EXECUTEIO _far *io);
extern void      ata_max_sg_cnt           (IORB_EXECUTEIO _far *io,
                                           USHORT sg_indx, USHORT sg_max,
                                           USHORT _far *sg_cnt,
                                           USHORT _far *sector_cnt);

extern int       ata_get_geometry         (IORBH _far *iorb, int slot);
extern void      ata_get_geometry_pp      (IORBH _far *iorb);
extern int       ata_unit_ready           (IORBH _far *iorb, int slot);
extern int       ata_read                 (IORBH _far *iorb, int slot);
extern int       ata_read_unaligned       (IORBH _far *iorb, int slot);
extern void      ata_read_pp              (IORBH _far *iorb);
extern int       ata_verify               (IORBH _far *iorb, int slot);
extern int       ata_write                (IORBH _far *iorb, int slot);
extern int       ata_write_unaligned      (IORBH _far *iorb, int slot);
extern void      ata_write_pp             (IORBH _far *iorb);
extern int       ata_execute_ata          (IORBH _far *iorb, int slot);
extern void      ata_execute_ata_pp       (IORBH _far *iorb);
extern int       ata_req_sense            (IORBH _far *iorb, int slot);

extern char     *ata_dev_name             (u16 *id_buf);

