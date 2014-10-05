/******************************************************************************
 * atapi.h - ATAPI structures and macros for os2ahci driver
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

#define ATAPI_MIN_CDB_LEN   12   /* minimum ATAPI CDB len acc to AHCI spec */
#define ATAPI_MAX_CDB_LEN   16

/******************************************************************************
 * macros to fill in ATAPI CDB values
 */
#define SET_CDB_16(_t, _v)  (_t)[0] = (u8) ((_v) >> 8); \
                            (_t)[1] = (u8)  (_v)
#define SET_CDB_24(_t, _v)  (_t)[0] = (u8) ((_v) >> 16) \
                            (_t)[1] = (u8) ((_v) >> 8); \
                            (_t)[2] = (u8)  (_v)
#define SET_CDB_32(_t, _v)  (_t)[0] = (u8) ((_v) >> 24);\
                            (_t)[1] = (u8) ((_v) >> 16);\
                            (_t)[2] = (u8) ((_v) >> 8); \
                            (_t)[3] = (u8)  (_v)

#define GET_CDB_16(_f)      ((u16) (_f)[0] << 8  |            \
                             (u16) (_f)[1])
#define GET_CDB_24(_f)      ((u32) (_f)[0] << 16 |            \
                             (u32) (_f)[1] << 8   |           \
                             (u32) (_f)[2])
#define GET_CDB_32(_f)      ((u32) (_f)[0] << 24  |           \
			     (u32) (_f)[1] << 16  |           \
                             (u32) (_f)[2] << 24  |           \
                             (u32) (_f)[3])


/******************************************************************************
 * ATAPI/MMC command codes (as far as relevant for us)
 */
#define ATAPI_CMD_READ_6           0x08
#define ATAPI_CMD_READ_10          0x28
#define ATAPI_CMD_READ_12          0xa8
#define ATAPI_CMD_READ_16          0x88
#define ATAPI_CMD_WRITE_6          0x0a
#define ATAPI_CMD_WRITE_10         0x2a
#define ATAPI_CMD_WRITE_12         0xaa
#define ATAPI_CMD_WRITE_16         0x8a
#define ATAPI_CMD_REQUEST_SENSE    0x03

/******************************************************************************
 * ATAPI command flag bits
 */
#define ATAPI_FLAG_FUA             0x80
#define ATAPI_FLAG_DPO             0x10

#define ATAPI_FEAT_DMA             0x0001
#define ATAPI_FEAT_DMA_TO_HOST     0x0004

/******************************************************************************
 * ATAPI sense data
 */
#define ATAPI_SENSE_LEN            96

#define ASENSE_NO_SENSE            0x00  /* no sense -> success */
#define ASENSE_RECOVERED_ERROR     0x01  /* recovered error -> success */
#define ASENSE_NOT_READY           0x02  /* device not ready */
#define ASENSE_MEDIUM_ERROR        0x03  /* medium/CRC error */
#define ASENSE_HARDWARE_ERROR      0x04  /* device error */
#define ASENSE_ILLEGAL_REQUEST     0x05  /* invalid command/parameters issued */
#define ASENSE_UNIT_ATTENTION      0x06  /* new medium */
#define ASENSE_DATA_PROTECT        0x07  /* protected LBA */
#define ASENSE_BLANK_CHECK         0x08  /* unformatted or write protected */
#define ASENSE_VENDOR_SPECIFIC     0x09  /* vendor specific sense data */
#define ASENSE_COPY_ABORTED        0x0a  /* copy, ...command aborted */
#define ASENSE_ABORTED_COMMAND     0x0b  /* command has been aborted */
#define ASENSE_EQUAL               0x0c
#define ASENSE_VOLUME_OVERFLOW     0x0d  /* out of space */
#define ASENSE_MISCOMPARE          0x0e  /* verification failed */
#define ASENSE_RESERVED            0x0f

/******************************************************************************
 * macro to get sense key from ATAPI_SENSE_DATA pointer
 */
#define ATAPI_GET_SENSE(p_) (u8)(p_->sense.sense_key & 0x0f)

/* ------------------------ typedefs and structures ------------------------ */

/******************************************************************************
 * ATAPI_SENSE_DATA - define layout of ATAPI sense data
 */
typedef union _ATAPI_SENSE_DATA {
  struct {
    u8 valid_respc;      /* valid bit (bit 7), response code (bits 6:0) */
    u8 segment;          /* segment number (obsolete) */
    u8 sense_key;        /* some flags (bits 7:4), sense key (bits 3:0) */
    u8 info[4];          /* information (?) */
    u8 adl_len;          /* additional sense info length */
    u8 cmd_specific[4];  /* command specific stuff (ignored) */
    u8 asc;              /* additional sense code */
    u8 ascq;             /* additional sense code qualifier */
    u8 fruc;             /* field replaceable unit code */
    u8 flags;            /* vendor-specific flags */
    u8 field_off[2];     /* offset to invalid field in parm list */
  } sense;

  u8 padding[ATAPI_SENSE_LEN]; /* pad to 64 bytes */

} ATAPI_SENSE_DATA;

/******************************************************************************Ü
 * ATAPI_CDB_6 - describes 6 byte ATAPI command
 */
typedef struct _ATAPI_CDB_6 {
  u8   cmd;            /* command code */
  u8   lun;            /* SCSI lun (ignored) */
  u8   resvd[2];
  u8   trans_len;      /* transfer length */
  u8   control;
} ATAPI_CDB_6;

/******************************************************************************
 * ATAPI_CDB_10 - describes layout of generic 10 byte ATAPI command
 */
typedef struct _ATAPI_CDB_10 {
  u8   cmd;            /* SCSI/ATAPI command code */
  u8   flags;          /* flags (DPO, FUA) */
  u8   lba[4];         /* logical block address */
  u8   trans_len[2];   /* number of blocks to transfer */
  u8   control;        /* (nothing much) */
} ATAPI_CDB_10;

/******************************************************************************
 * ATAPI_CDB_12 - describes layout of generic 12 byte ATAPI command
 */
typedef struct _ATAPI_CDB_12 {
  u8   cmd;            /* SCSI/ATAPI command code */
  u8   flags;          /* flags (DPO, FUA) */
  u8   lba[4];         /* logical block address */
  u8   trans_len[4];   /* number of blocks to transfer */
  u8   reserved;
  u8   control;        /* (nothing much) */
} ATAPI_CDB_12;


/* ---------------------------- global variables --------------------------- */

/* -------------------------- function prototypes -------------------------- */

extern int     atapi_get_geometry     (IORBH _far *iorb, int slot);
extern int     atapi_unit_ready       (IORBH _far *iorb, int slot);
extern int     atapi_read             (IORBH _far *iorb, int slot);
extern int     atapi_read_unaligned   (IORBH _far *iorb, int slot);
extern int     atapi_verify           (IORBH _far *iorb, int slot);
extern int     atapi_write            (IORBH _far *iorb, int slot);
extern int     atapi_execute_cdb      (IORBH _far *iorb, int slot);
extern int     atapi_req_sense        (IORBH _far *iorb, int slot);

