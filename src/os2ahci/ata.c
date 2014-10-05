/******************************************************************************
 * ata.c - ATA command processing
 *
 * Copyright (c) 2011 thi.guten Software Development
 * Copyright (c) 2011 Mensys B.V.
 * Portions copyright (c) 2013 David Azarewicz
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

#include "os2ahci.h"
#include "ata.h"

/* -------------------------- macros and constants ------------------------- */

/* ------------------------ typedefs and structures ------------------------ */

/* -------------------------- function prototypes -------------------------- */

static int ata_cmd_read (IORBH _far *iorb, AD_INFO *ai, int p, int d, int slot,
                         ULONG sector, ULONG count, SCATGATENTRY _far *sg_list,
                         ULONG sg_cnt);

static int ata_cmd_write(IORBH _far *iorb, AD_INFO *ai, int p, int d, int slot,
                         ULONG sector, ULONG count, SCATGATENTRY _far *sg_list,
                         ULONG sg_cnt, int write_through);

/* ------------------------ global/static variables ------------------------ */

/* ----------------------------- start of code ----------------------------- */

/******************************************************************************
 * Initialize AHCI command slot, FIS and S/G list for the specified ATA
 * command. The command parameters are passed as a variable argument list
 * of type and value(s). The list is terminated by AP_END.
 *
 * Notes:
 *
 *  - The specified command slot is expected to be idle; no checks are
 *    performed to prevent messing with a busy port.
 *
 *  - Port multipliers are not supported, yet, thus 'd' should always
 *    be 0 for the time being.
 *
 *  - 'cmd' is passed as 16-bit integer because the compiler would push
 *    a 'u8' as 16-bit value (it's a fixed argument) and the stdarg
 *    macros would screw up the address of the first variable argument
 *    if the size of the last fixed argument wouldn't match what the
 *    compiler pushed on the stack.
 *
 * Return values:
 *    0 : success
 *  > 0 : could not map all S/G entries; the return value is the number of
 *        S/G entries that could be mapped.
 *  < 0 : other error
 */
int ata_cmd(AD_INFO *ai, int p, int d, int slot, int cmd, ...)
{
  va_list va;
  va_start(va, cmd);
  return(v_ata_cmd(ai, p, d, slot, cmd, va));
}

int v_ata_cmd(AD_INFO *ai, int p, int d, int slot, int cmd, va_list va)
{
  AHCI_PORT_DMA _far *dma_base_virt;
  AHCI_CMD_HDR _far *cmd_hdr;
  AHCI_CMD_TBL _far *cmd_tbl;
  SCATGATENTRY _far *sg_list = NULL;
  SCATGATENTRY sg_single;
  ATA_PARM ap;
  ATA_CMD ata_cmd;
  void _far *atapi_cmd = NULL;
  u32 dma_base_phys;
  u16 atapi_cmd_len = 0;
  u16 ahci_flags = 0;
  u16 sg_cnt = 0;
  int i;
  int n;

  /* --------------------------------------------------------------------------
   * Initialize ATA command. The ATA command is set up with the main command
   * value and a variable list of additional parameters such as the sector
   * address, transfer count, ...
   */
  memset(&ata_cmd, 0x00, sizeof(ata_cmd));
  ata_cmd.cmd = (u8) cmd;

  /* parse variable arguments */
  do {
    switch ((ap = va_arg(va, ATA_PARM))) {

    case AP_AHCI_FLAGS:
      ahci_flags |= va_arg(va, u16);
      break;

    case AP_WRITE:
      if (va_arg(va, u16) != 0) {
        ahci_flags |= AHCI_CMD_WRITE;
      }
      break;

    case AP_FEATURES:
      /* ATA features word */
      ata_cmd.features |= va_arg(va, u16);
      break;

    case AP_COUNT:
      /* transfer count */
      ata_cmd.count = va_arg(va, u16);
      break;

    case AP_SECTOR_28:
      /* 28-bit sector address */
      ata_cmd.lba_l = va_arg(va, u32);
      if (ata_cmd.lba_l & 0xf0000000UL) {
        dprintf("error: LBA-28 address %ld has more than 28 bits\n", ata_cmd.lba_l);
        return(ATA_CMD_INVALID_PARM);
      }
      /* add upper 4 bits to device field */
      ata_cmd.device |= (ata_cmd.lba_l >> 24) & 0x0fU;
      /* only lower 24 bits come into lba_l */
      ata_cmd.lba_l &= 0x00ffffffUL;
      break;

    case AP_SECTOR_48:
      /* 48-bit sector address */
      ata_cmd.lba_l = va_arg(va, u32);
      ata_cmd.lba_h = va_arg(va, u16);
      break;

    case AP_DEVICE:
      /* ATA device byte; note that this byte contains the highest
       * 4 bits of LBA-28 address; we have to leave them alone here. */
      ata_cmd.device |= va_arg(va, u16) & 0xf0U;
      break;

    case AP_SGLIST:
      /* scatter/gather list in SCATGATENTRY/count format */
      sg_list = va_arg(va, void _far *);
      sg_cnt = va_arg(va, u16);
      break;

    case AP_VADDR:
      /* virtual buffer address in addr/len format (up to 4K) */
      sg_single.ppXferBuf = virt_to_phys(va_arg(va, void _far *));
      sg_single.XferBufLen = va_arg(va, u16);
      sg_list = &sg_single;
      sg_cnt = 1;
      break;

    case AP_ATAPI_CMD:
      /* ATAPI command */
      atapi_cmd = va_arg(va, void _far *);
      atapi_cmd_len = va_arg(va, u16);
      ahci_flags |= AHCI_CMD_ATAPI;
      break;

    case AP_ATA_CMD:
      /* ATA command "pass-through" */
      memcpy(&ata_cmd, va_arg(va, void _far *), sizeof(ATA_CMD));
      break;

    case AP_END:
      break;

    default:
      dprintf("error: v_ata_cmd() called with invalid parameter type (%d)\n", (int) ap);
      return(ATA_CMD_INVALID_PARM);
    }

  } while (ap != AP_END);

  /* --------------------------------------------------------------------------
   * Fill in AHCI ATA command information. This includes the port command slot,
   * the corresponding command FIS and the S/G list. The layout of the AHCI
   * port DMA region is based on the Linux AHCI driver and looks like this:
   *
   *  - 32 AHCI command headers (AHCI_CMD_HDR) with 32 bytes, each
   *  -  1 FIS receive area with 256 bytes (AHCI_RX_FIS_SZ)
   *  - 32 AHCI command tables, each consisting of
   *    -  64 bytes for command FIS
   *    -  16 bytes for ATAPI comands
   *    -  48 bytes reserved
   *    -  48 S/G entries (AHCI_SG) with 32 bytes, each
   *
   * Since the whole DMA buffer for all ports is larger than 64KB and we need
   * multiple segments to address all of them, there are no virtual pointers
   * to the individual elements in AD_INFO. Instead, we're relying on macros
   * for getting the base address of a particular port's DMA region, then
   * map a structure on top of that for convenience (AHCI_PORT_DMA).
   */
  dma_base_virt = port_dma_base(ai, p);
  dma_base_phys = port_dma_base_phys(ai, p);

  /* AHCI command header */
  cmd_hdr = dma_base_virt->cmd_hdr + slot;
  memset(cmd_hdr, 0x00, sizeof(*cmd_hdr));
  cmd_hdr->options  = ((d & 0x0f) << 12);
  cmd_hdr->options |= ahci_flags;  /* AHCI command flags */
  cmd_hdr->options |= 5;           /* length of command FIS in 32-bit words */
  cmd_hdr->tbl_addr = dma_base_phys + offsetof(AHCI_PORT_DMA, cmd_tbl[slot]);

  /* AHCI command table */
  cmd_tbl = dma_base_virt->cmd_tbl + slot;
  memset(cmd_tbl, 0x00, sizeof(*cmd_tbl));
  ata_cmd_to_fis(cmd_tbl->cmd_fis, &ata_cmd, d);

  if (atapi_cmd != NULL) {
    /* copy ATAPI command */
    memcpy(cmd_tbl->atapi_cmd, atapi_cmd, atapi_cmd_len);
  }

  /* PRDT (S/G list)
   *
   *  - The S/G list for AHCI adapters is limited to 22 bits for the transfer
   *    size of each element, thus we need to split S/G elements larger than
   *    22 bits into 2 AHCI_SG elements.
   *
   *  - The S/G element size for AHCI is what the spec calls '0'-based
   *    (i.e. 0 means 1 bytes). On top of that, the spec requires S/G transfer
   *    sizes to be even in the context of 16-bit transfers, thus bit '1'
   *    always needs to be set.
   *
   *  - AHCI_MAX_SG_ELEMENT_LEN defines the maximum size of an AHCI S/G
   *    element in bytes, ignoring the '0'-based methodology (i.e. 1 << 22).
   *
   *  - There's a limit on the maximum number of S/G elements in the port DMA
   *    buffer (AHCI_MAX_SG) which is lower than the HW maximum. It's beyond
   *    the control of this function to split commands which require more
   *    than AHCI_MAX_SG entries. In order to help the caller, the return value
   *    of this function will indicate how many OS/2 S/G entries were
   *    successfully mapped.
   */
  for (i = n = 0; i < sg_cnt; i++) {
    u32 sg_addr = sg_list[i].ppXferBuf;
    u32 sg_size = sg_list[i].XferBufLen;

    do {
      u32 chunk = (sg_size > AHCI_MAX_SG_ELEMENT_LEN) ? AHCI_MAX_SG_ELEMENT_LEN
                                                      : sg_size;
      if (n >= AHCI_MAX_SG) {
        /* couldn't store all S/G elements in our DMA buffer */
        ddprintf("ata_cmd(): too many S/G elements\n");
        return(i - 1);
      }
      if ((sg_addr & 1) || (chunk & 1)) {
        ddprintf("error: ata_cmd() called with unaligned S/G element(s)\n");
        return(ATA_CMD_UNALIGNED_ADDR);
      }
      cmd_tbl->sg_list[n].addr = sg_addr;
      cmd_tbl->sg_list[n].size = chunk - 1;
      sg_addr += chunk;
      sg_size -= chunk;
      n++;
    } while (sg_size > 0);
  }

  /* set final S/G count in AHCI command header */
  cmd_hdr->options |= (u32) n << 16;

  if (debug >= 2) {
    aprintf("ATA command for %d.%d.%d, slot %d:\n", ad_no(ai), p, d, slot);
    phex(cmd_hdr, offsetof(AHCI_CMD_HDR, reserved), "cmd_hdr:   ");
    phex(&ata_cmd, sizeof(ata_cmd), "ata_cmd:   ");
    if (atapi_cmd != NULL) {
      phex(atapi_cmd, atapi_cmd_len, "atapi_cmd: ");
    }
    if (n > 0) {
      phex(cmd_tbl->sg_list, sizeof(*cmd_tbl->sg_list) * n, "sg_list:   ");
    }
  }

  return(ATA_CMD_SUCCESS);
}

/******************************************************************************
 * Fill SATA command FIS with values extracted from an ATA command structure.
 * The command FIS buffer (fis) is expected to be initialized to 0s. The
 * structure of the FIS maps to the ATA shadow register block, including
 * registers which can be written twice to store 16 bits (called 'exp').
 *
 * The FIS structure looks like this (using LSB notation):
 *
 *       +----------------+----------------+----------------+----------------+
 *  00   | FIS type (27h) |   C|R|R|R|PMP  |   Command      |  Features      |
 *       +----------------+----------------+----------------+----------------+
 *  04   |   LBA 7:0      |   LBA 15:8     |   LBA 23:16    |  R|R|R|D|Head  |
 *       +----------------+----------------+----------------+----------------+
 *  08   |   LBA 31:24    |   LBA 40:32    |   LBA 47:40    |  Features exp  |
 *       +----------------+----------------+----------------+----------------+
 *  12   |   Count 7:0    |   Count 15:8   |   Reserved     |  Control       |
 *       +----------------+----------------+----------------+----------------+
 *  16   |   Reserved     |   Reserved     |   Reserved     |  Reserved      |
 *       +----------------+----------------+----------------+----------------+
 */
void ata_cmd_to_fis(u8 _far *fis, ATA_CMD _far *ata_cmd, int d)
{
  fis[0]  = 0x27;                  /* register - host to device FIS */
  fis[1]  = (u8) (d & 0xf);        /* port multiplier number */
  fis[1] |= 0x80;                  /* bit 7 indicates Command FIS */
  fis[2]  = (u8) ata_cmd->cmd;
  fis[3]  = (u8) ata_cmd->features;

  fis[4]  = (u8) ata_cmd->lba_l;
  fis[5]  = (u8) (ata_cmd->lba_l >> 8);
  fis[6]  = (u8) (ata_cmd->lba_l >> 16);
  fis[7]  = (u8) ata_cmd->device;

  fis[8]  = (u8) (ata_cmd->lba_l >> 24);
  fis[9]  = (u8) ata_cmd->lba_h;
  fis[10] = (u8) (ata_cmd->lba_h >> 8);
  fis[11] = (u8) (ata_cmd->features >> 8);

  fis[12] = (u8) ata_cmd->count;
  fis[13] = (u8) (ata_cmd->count >> 8);
}

/******************************************************************************
 * Get index in S/G list for the number of transferred sectors in the IORB.
 *
 * Returning io->cSGList indicates an error.
 *
 * NOTE: OS/2 makes sure S/G lists are set up such that entries at the HW
 *       limit will never cross sector boundaries. This means that splitting
 *       S/G lists into multiple commands can be done without editing the S/G
 *       lists.
 */
u16 ata_get_sg_indx(IORB_EXECUTEIO _far *io)
{
  ULONG offset = io->BlocksXferred * io->BlockSize;
  USHORT i;

  for (i = 0; i < io->cSGList && offset > 0; i++) {
    offset -= io->pSGList[i].XferBufLen;
  }

  return(i);
}

/******************************************************************************
 * Get max S/G count which will fit into our HW S/G buffers. This function is
 * called when the S/G list is too long and we need to split the IORB into
 * multiple commands. It returns both the number of sectors and S/G list
 * elements that we can handle in a single command.
 *
 * The parameter 'sg_indx' indicates the current start index in the S/G list
 * (0 if this is the first command iteration).
 *
 * The parameter 'sg_max' is the return value of v_ata_cmd() and indicates
 * how many S/G elements were successfully mapped. Whatever we return needs to
 * be less or equal to this value.
 *
 * Returning 0 in *sg_cnt indicates an error.
 *
 * NOTE: OS/2 makes sure S/G lists are set up such that entries at HW limits
 *       will never cross sector boundaries. This means that splitting S/G
 *       lists into multiple commands can be done without editing S/G list
 *       elements. Since AHCI only allows 22 bits for each S/G element, the
 *       hardware limits are reported as AHCI_MAX_SG / 2 but will vary based
 *       on the actual length of S/G elements. This function looks for the
 *       maximum number of S/G elements that can be mapped on sector
 *       boundaries which will still fit into our HW S/G list.
 */
void ata_max_sg_cnt(IORB_EXECUTEIO _far *io, USHORT sg_indx, USHORT sg_max,
                    USHORT _far *sg_cnt, USHORT _far *sector_cnt)
{
  ULONG max_sector_cnt = 0;
  USHORT max_sg_cnt = 0;
  ULONG offset = 0;
  USHORT i;

  for (i = sg_indx; i < io->cSGList; i++) {
    if (i - sg_indx >= sg_max) {
      /* we're beyond the number of S/G elements we can map */
      break;
    }

    offset += io->pSGList[i].XferBufLen;
    if (offset % io->BlockSize == 0) {
      /* this S/G element ends on a sector boundary */
      max_sector_cnt = offset / io->BlockSize;
      max_sg_cnt = i + 1;
    }
  }

  /* return the best match we found (0 indicating failure) */
  *sector_cnt = max_sector_cnt;
  *sg_cnt = max_sg_cnt;
}


/******************************************************************************
 * Get device or media geometry. Device and media geometry are expected to be
 * the same for non-removable devices, which will always be the case for the
 * ATA devices we're dealing with (hard disks). ATAPI is a different story
 * and handled by atapi_get_geometry().
 */
int ata_get_geometry(IORBH _far *iorb, int slot)
{
  ADD_WORKSPACE _far *aws = add_workspace(iorb);
  int rc;

  /* allocate buffer for ATA identify information */
  if ((aws->buf = malloc(ATA_ID_WORDS * sizeof(u16))) == NULL) {
    iorb_seterr(iorb, IOERR_CMD_SW_RESOURCE);
    return(-1);
  }

  /* request ATA identify information */
  aws->ppfunc = ata_get_geometry_pp;
  rc = ata_cmd(ad_infos + iorb_unit_adapter(iorb),
               iorb_unit_port(iorb),
               iorb_unit_device(iorb),
               slot,
               ATA_CMD_ID_ATA,
               AP_VADDR, (void _far *) aws->buf, ATA_ID_WORDS * sizeof(u16),
               AP_END);

  if (rc != 0) {
    iorb_seterr(iorb, IOERR_CMD_ADD_SOFTWARE_FAILURE);
  }

  return(rc);
}

/* Adjust the cylinder count in the physical
 * geometry to the last full cylinder.
 */
int adjust_cylinders(GEOMETRY _far *geometry, ULONG TotalSectors) {
  USHORT SecPerCyl;
  int rc = FALSE;

  geometry->TotalSectors = TotalSectors;
  SecPerCyl = geometry->SectorsPerTrack * geometry->NumHeads;
  if (SecPerCyl > 0) {
    ULONG TotalCylinders  = TotalSectors / SecPerCyl;

    geometry->TotalSectors   = TotalCylinders * SecPerCyl;
    geometry->TotalCylinders = TotalCylinders;
    if (TotalCylinders >> 16) {
      geometry->TotalCylinders = 65535;
      rc = TRUE;
    }
  }
  return (rc);
}

/* Calculate the logical geometry based on the input physcial geometry
 * using the LBA Assist Translation algorithm.
 */
#define BIOS_MAX_CYLINDERS       1024l
#define BIOS_MAX_NUMHEADS         255
#define BIOS_MAX_SECTORSPERTRACK   63
void log_geom_calculate_LBA_assist(GEOMETRY _far *geometry, ULONG TotalSectors)
{
  UCHAR numSpT   = BIOS_MAX_SECTORSPERTRACK;
  UCHAR numHeads = BIOS_MAX_NUMHEADS;
  ULONG Cylinders;

  if (TotalSectors <= (BIOS_MAX_CYLINDERS * 128 * BIOS_MAX_SECTORSPERTRACK)) {
    USHORT temp = (TotalSectors - 1) / (BIOS_MAX_CYLINDERS * BIOS_MAX_SECTORSPERTRACK);

    if (temp < 16)      numHeads = 16;
    else if (temp < 32) numHeads = 32;
    else if (temp < 64) numHeads = 64;
    else                numHeads = 128;
  }

  do {
    Cylinders = TotalSectors / (USHORT)(numHeads * numSpT);
    if (Cylinders >> 16) {
      if (numSpT < 128)
        numSpT = (numSpT << 1) | 1;
      else
        Cylinders = 65535; // overflow !
    }
  } while (Cylinders >> 16);

  geometry->TotalCylinders  = Cylinders;
  geometry->NumHeads        = numHeads;
  geometry->SectorsPerTrack = numSpT;
}

int check_lvm(IORBH _far *iorb, ULONG sector)
{
  DLA_Table_Sector *pDLA = (DLA_Table_Sector*)add_workspace(iorb)->buf;
  AD_INFO *ai = ad_infos + iorb_unit_adapter(iorb);
  GEOMETRY _far *geometry = ((IORB_GEOMETRY _far *) iorb)->pGeometry;
  int p = iorb_unit_port(iorb);
  int rc;

  rc = ahci_exec_polled_cmd(ai, p, 0, 500, ATA_CMD_READ,
         AP_SECTOR_28, (u32) sector-1,
         AP_COUNT, (u16) 1,
         AP_VADDR, (void _far *) pDLA, 512,
         AP_DEVICE, 0x40,
         AP_END);
  if (rc) return 0;

  ddphex(pDLA, sizeof(DLA_Table_Sector), "DLA sector %d:\n", sector-1);

  if ((pDLA->DLA_Signature1 == DLA_TABLE_SIGNATURE1) && (pDLA->DLA_Signature2 == DLA_TABLE_SIGNATURE2)) {
    ddprintf("is_lvm_geometry found at sector %d\n", sector-1);
    geometry->TotalCylinders = pDLA->Cylinders;
    geometry->NumHeads = pDLA->Heads_Per_Cylinder;
    geometry->SectorsPerTrack = pDLA->Sectors_Per_Track;
    geometry->TotalSectors = pDLA->Cylinders * pDLA->Heads_Per_Cylinder * pDLA->Sectors_Per_Track;
    return 1;
  }

  return 0;
}

/******************************************************************************
 * Try to read LVM information from the disk. If found, use the LVM geometry.
 * This function will only work at init time. A better strategy would be to
 * calculate the geometry during ahci_scan_ports and save it away and then just
 * return the saved values when ata_get_geometry() is called.
 */
int is_lvm_geometry(IORBH _far *iorb)
{
  GEOMETRY _far *geometry = ((IORB_GEOMETRY _far *) iorb)->pGeometry;
  ULONG sector;

  if (init_complete) return 0; /* We cannot use ahci_exec_polled_cmd() after init_complete */

  if (use_lvm_info) {
    #ifdef DEBUG
    AD_INFO *ai = ad_infos + iorb_unit_adapter(iorb);
    int p = iorb_unit_port(iorb);
    int d = iorb_unit_device(iorb);
    ddprintf("is_lvm_geometry (%d.%d.%d)\n", ad_no(ai), p, d);
    #endif

    /* First check the sector reported by the hardware */
    if (check_lvm(iorb, geometry->SectorsPerTrack)) return 1;

    for (sector = 255; sector >= 63; sector >>= 1) {
      if (sector == geometry->SectorsPerTrack) continue;
      if (check_lvm(iorb, sector)) return 1;
    }
  }

  return 0;
}

/******************************************************************************
 * Post processing function for ata_get_geometry(): convert the ATA identify
 * information to OS/2 IOCC_GEOMETRY information.
 */
void ata_get_geometry_pp(IORBH _far *iorb)
{
  GEOMETRY _far *geometry = ((IORB_GEOMETRY _far *) iorb)->pGeometry;
  USHORT geometry_len =  ((IORB_GEOMETRY _far *) iorb)->GeometryLen;
  u16 *id_buf = add_workspace(iorb)->buf;
  int a = iorb_unit_adapter(iorb);
  int p = iorb_unit_port(iorb);
  char *Method;

  /* Fill-in geometry information; the ATA-8 spec declares the geometry
   * fields in the ATA ID buffer as obsolete but it's still the best
   * guess in most cases. If the information stored in the geometry
   * fields is apparently incorrect, we'll use the algorithm typically
   * used by SCSI adapters and modern PC BIOS versions:
   *
   *  - 512 bytes per sector
   *  - 255 heads
   *  -  63 sectors per track (or 56 with the parameter "/4")
   *  -   x cylinders (calculated)
   *
   * Please note that os2ahci currently does not natively support ATA sectors
   * larger than 512 bytes, therefore relies on the translation logic built
   * into the corresponding ATA disks. In order to prevent file systems that
   * use block sizes larger than 512 bytes (FAT, JFS, ...) from ending up on
   * incorrectly aligned physical sector accesses, hence using more physical
   * I/Os than necessary, the command line parameter "/4" can be used to force
   * a track size of 56 sectors. This way, partitions will start on 4K
   * boundaries.
   *
   * Another limitation is that OS/2 has a 32-bit variable for the total number
   * of sectors, limiting the maximum capacity to roughly 2TB. This is another
   * issue that needs to be addressed sooner or later; large sectors could
   * raise this limit to something like 8TB but this is not really much of a
   * difference. Maybe there's something in later DDKs that allows more than
   * 32 bits?
   */
  memset(geometry, 0x00, geometry_len);
  geometry->BytesPerSector = ATA_SECTOR_SIZE;

  /* extract total number of sectors */
  if (id_buf[ATA_ID_CFS_ENABLE_2] & 0x400) {
    /* 48-bit LBA supported */
    if (ATA_CAPACITY48_H(id_buf) != 0) {
      /* more than 32 bits for number of sectors */
      dprintf("warning: limiting disk %d.%d.%d to 2TB\n",
              iorb_unit_adapter(iorb), iorb_unit_port(iorb),
              iorb_unit_device(iorb));
      geometry->TotalSectors = 0xffffffffUL;
    } else {
      geometry->TotalSectors = ATA_CAPACITY48_L(id_buf);
    }
  } else {
    /* 28-bit LBA */
    geometry->TotalSectors = ATA_CAPACITY(id_buf) & 0x0fffffffUL;
  }

  Method = "None";
  /* fabricate the remaining geometry fields */
  if (track_size[a][p] != 0) {
    /* A specific track size has been requested for this port; this is
     * typically done for disks with 4K sectors to make sure partitions
     * start on 8-sector boundaries (parameter "/4").
     */
    geometry->NumHeads        = 255;
    geometry->SectorsPerTrack = track_size[a][p];
    geometry->TotalCylinders  = geometry->TotalSectors / ((u32) geometry->NumHeads * (u32) geometry->SectorsPerTrack);
    Method = "Custom";
  } else if (CUR_HEADS(id_buf) > 0 && CUR_CYLS(id_buf) > 0 && CUR_SECTORS(id_buf) > 0 &&
             CUR_CAPACITY(id_buf) == (u32) CUR_HEADS(id_buf) * (u32) CUR_CYLS(id_buf) * (u32) CUR_SECTORS(id_buf)) {
    /* BIOS-supplied (aka "current") geometry values look valid */
    geometry->NumHeads        = CUR_HEADS(id_buf);
    geometry->SectorsPerTrack = CUR_SECTORS(id_buf);
    geometry->TotalCylinders  = CUR_CYLS(id_buf);
    Method = "BIOS";
  } else if (ATA_HEADS(id_buf) > 0 && ATA_CYLS(id_buf) > 0 && ATA_SECTORS(id_buf) > 0) {
    /* ATA-supplied values for geometry look valid */
    geometry->NumHeads        = ATA_HEADS(id_buf);
    geometry->SectorsPerTrack = ATA_SECTORS(id_buf);
    geometry->TotalCylinders  = ATA_CYLS(id_buf);
    Method = "ATA";
  } else {
    /* use typical SCSI geometry */
    geometry->NumHeads        = 255;
    geometry->SectorsPerTrack = 63;
    geometry->TotalCylinders  = geometry->TotalSectors / ((u32) geometry->NumHeads * (u32) geometry->SectorsPerTrack);
    Method = "SCSI";
  }

  dprintf("Physical geometry: %ld cylinders, %d heads, %d sectors per track (%ldMB) (%s)\n",
      (u32) geometry->TotalCylinders, (u16) geometry->NumHeads, (u16) geometry->SectorsPerTrack,
      (u32) (geometry->TotalSectors / 2048), Method);

  /* Fixup the geometry in case the geometry reported by the BIOS is bad */
  if (adjust_cylinders(geometry, geometry->TotalSectors)) { // cylinder overflow
    log_geom_calculate_LBA_assist(geometry, geometry->TotalSectors);
    geometry->TotalSectors = (USHORT)(geometry->NumHeads * geometry->SectorsPerTrack) * (ULONG)geometry->TotalCylinders;
  }
  adjust_cylinders(geometry, geometry->TotalSectors);

  dprintf("Logical geometry: %ld cylinders, %d heads, %d sectors per track (%ldMB) (%s)\n",
      (u32) geometry->TotalCylinders, (u16) geometry->NumHeads, (u16) geometry->SectorsPerTrack,
      (u32) (geometry->TotalSectors / 2048), Method);

  if (is_lvm_geometry(iorb)) Method = "LVM";
  ad_infos[a].ports[p].devs[0].dev_info.Cylinders = geometry->TotalCylinders;
  ad_infos[a].ports[p].devs[0].dev_info.HeadsPerCylinder = geometry->NumHeads;
  ad_infos[a].ports[p].devs[0].dev_info.SectorsPerTrack = geometry->SectorsPerTrack;
  ad_infos[a].ports[p].devs[0].dev_info.TotalSectors = geometry->TotalSectors;
  ad_infos[a].ports[p].devs[0].dev_info.Method = Method;

  dprintf("Reported geometry: %ld cylinders, %d heads, %d sectors per track (%ldMB) (%s)\n",
      (u32) geometry->TotalCylinders, (u16) geometry->NumHeads, (u16) geometry->SectorsPerTrack,
      (u32) (geometry->TotalSectors / 2048), Method);

  /* tell interrupt handler that this IORB is complete */
  add_workspace(iorb)->complete = 1;
}

/******************************************************************************
 * Test whether unit is ready.
 */
int ata_unit_ready(IORBH _far *iorb, int slot)
{
  /* This is a NOP for ATA devices (at least right now); returning an error
   * without setting an error code means ahci_exec_iorb() will not queue any
   * HW command and the IORB will complete successfully.
   */
  ((IORB_UNIT_STATUS _far *) iorb)->UnitStatus = US_READY | US_POWER;
  return(-1);
}

/******************************************************************************
 * Read sectors from AHCI device.
 */
int ata_read(IORBH _far *iorb, int slot)
{
  IORB_EXECUTEIO _far *io = (IORB_EXECUTEIO _far *) iorb;
  AD_INFO *ai = ad_infos + iorb_unit_adapter(iorb);
  ULONG sector = io->RBA + io->BlocksXferred;
  USHORT count = io->BlockCount - io->BlocksXferred;
  USHORT sg_indx;
  USHORT sg_cnt;
  int p = iorb_unit_port(iorb);
  int d = iorb_unit_device(iorb);
  int rc;

  if (io->BlockCount == 0) {
    /* NOP; return -1 without error in IORB to indicate success */
    return(-1);
  }

  if (add_workspace(iorb)->unaligned) {
    /* unaligned S/G addresses present; need to use double buffers */
    return(ata_read_unaligned(iorb, slot));
  }

  /* Kludge: some I/O commands during boot use excessive S/G buffer lengths
   * which cause NCQ commands to lock up. If there's only one S/G element
   * and this element is already larger than what we can derive from the sector
   * count, we'll adjust that element.
   */
  if (io->BlocksXferred == 0 && io->cSGList == 1 &&
      io->pSGList[0].XferBufLen > (ULONG) io->BlockCount * io->BlockSize) {
    io->pSGList[0].XferBufLen = (ULONG) io->BlockCount * io->BlockSize;
  }

  /* prepare read command while keeping an eye on S/G count limitations */
  do {
    sg_indx = ata_get_sg_indx(io);
    sg_cnt = io->cSGList - sg_indx;
    if ((rc = ata_cmd_read(iorb, ai, p, d, slot, sector, count,
                           io->pSGList + sg_indx, sg_cnt)) > 0) {
      /* couldn't map all S/G elements */
      ata_max_sg_cnt(io, sg_indx, (USHORT) rc, &sg_cnt, &count);
    }
  } while (rc > 0 && sg_cnt > 0);

  if (rc == 0) {
    add_workspace(iorb)->blocks = count;
    add_workspace(iorb)->ppfunc = ata_read_pp;

  } else if (rc > 0) {
    iorb_seterr(iorb, IOERR_CMD_SGLIST_BAD);

  } else if (rc == ATA_CMD_UNALIGNED_ADDR) {
    /* unaligned S/G addresses detected; need to use double buffers */
    add_workspace(iorb)->unaligned = 1;
    return(ata_read_unaligned(iorb, slot));

  } else {
    iorb_seterr(iorb, IOERR_CMD_ADD_SOFTWARE_FAILURE);
  }

  return(rc);
}

/******************************************************************************
 * Read sectors from AHCI device with unaligned S/G element addresses. AHCI
 * only allows aligned S/G addresses while OS/2 doesn't have these kind of
 * restrictions. This doesn't happen very often but when it does, we need to
 * use a transfer buffer and copy the data manually.
 */
int ata_read_unaligned(IORBH _far *iorb, int slot)
{
  IORB_EXECUTEIO _far *io = (IORB_EXECUTEIO _far *) iorb;
  ADD_WORKSPACE _far *aws = add_workspace(iorb);
  AD_INFO *ai = ad_infos + iorb_unit_adapter(iorb);
  ULONG sector = io->RBA + io->BlocksXferred;
  SCATGATENTRY sg_single;
  int p = iorb_unit_port(iorb);
  int d = iorb_unit_device(iorb);
  int rc;

  ddprintf("ata_read_unaligned(%d.%d.%d, %ld)\n", ad_no(ai), p, d, sector);

  /* allocate transfer buffer */
  if ((aws->buf = malloc(io->BlockSize)) == NULL) {
    iorb_seterr(iorb, IOERR_CMD_SW_RESOURCE);
    return(-1);
  }

  /* prepare read command using transfer buffer */
  sg_single.ppXferBuf = virt_to_phys(aws->buf);
  sg_single.XferBufLen = io->BlockSize;
  rc = ata_cmd_read(iorb, ai, p, d, slot, sector, 1, &sg_single, 1);

  if (rc == 0) {
    add_workspace(iorb)->blocks = 1;
    add_workspace(iorb)->ppfunc = ata_read_pp;

  } else if (rc > 0) {
    iorb_seterr(iorb, IOERR_CMD_SGLIST_BAD);

  } else {
    iorb_seterr(iorb, IOERR_CMD_ADD_SOFTWARE_FAILURE);
  }

  return(rc);
}

/******************************************************************************
 * Post processing function for ata_read(); this function updates the
 * BlocksXferred counter in the IORB and, if not all blocks have been
 * transferred, requeues the IORB to process the remaining sectors. It also
 * takes care of copying data from the transfer buffer for unaligned reads.
 */
void ata_read_pp(IORBH _far *iorb)
{
  IORB_EXECUTEIO _far *io = (IORB_EXECUTEIO _far *) iorb;
  ADD_WORKSPACE _far *aws = add_workspace(iorb);

  if (aws->unaligned) {
    /* copy transfer buffer to corresponding physical address in S/G list */
    sg_memcpy(io->pSGList, io->cSGList,
              (ULONG) io->BlocksXferred * (ULONG) io->BlockSize,
              aws->buf, io->BlockSize, BUF_TO_SG);
  }

  io->BlocksXferred += add_workspace(iorb)->blocks;
  ddprintf("ata_read_pp(): blocks transferred = %d\n", (int) io->BlocksXferred);

  if (io->BlocksXferred >= io->BlockCount) {
    /* we're done; tell IRQ handler the IORB is complete */
    add_workspace(iorb)->complete = 1;
  } else {
    /* requeue this IORB for next iteration */
    iorb_requeue(iorb);
  }
}

/******************************************************************************
 * Verify readability of sectors on ATA device.
 */
int ata_verify(IORBH _far *iorb, int slot)
{
  IORB_EXECUTEIO _far *io = (IORB_EXECUTEIO _far *) iorb;
  AD_INFO *ai = ad_infos + iorb_unit_adapter(iorb);
  int p = iorb_unit_port(iorb);
  int d = iorb_unit_device(iorb);
  int rc;

  if (io->BlockCount == 0) {
    /* NOP; return -1 without error in IORB to indicate success */
    return(-1);
  }

  /* prepare verify command */
  if (io->RBA >= (1UL << 28) || io->BlockCount > 256) {
    /* need LBA48 for this command */
    if (!ai->ports[p].devs[d].lba48) {
      iorb_seterr(iorb, IOERR_RBA_LIMIT);
      return(-1);
    }
    rc = ata_cmd(ai, p, d, slot, ATA_CMD_VERIFY_EXT,
                 AP_SECTOR_48, (u32) io->RBA, (u16) 0,
                 AP_COUNT,     (u16) io->BlockCount,
                 AP_DEVICE,    0x40,
                 AP_END);
  } else {
    rc = ata_cmd(ai, p, d, slot, ATA_CMD_VERIFY,
                 AP_SECTOR_28, (u32) io->RBA,
                 AP_COUNT,     (u16) io->BlockCount & 0xffU,
                 AP_DEVICE,    0x40,
                 AP_END);
  }

  return(rc);
}

/******************************************************************************
 * Write sectors to AHCI device.
 */
int ata_write(IORBH _far *iorb, int slot)
{
  IORB_EXECUTEIO _far *io = (IORB_EXECUTEIO _far *) iorb;
  AD_INFO *ai = ad_infos + iorb_unit_adapter(iorb);
  ULONG sector = io->RBA + io->BlocksXferred;
  USHORT count = io->BlockCount - io->BlocksXferred;
  USHORT sg_indx;
  USHORT sg_cnt;
  int p = iorb_unit_port(iorb);
  int d = iorb_unit_device(iorb);
  int rc;

  if (io->BlockCount == 0) {
    /* NOP; return -1 without error in IORB to indicate success */
    return(-1);
  }

  if (add_workspace(iorb)->unaligned) {
    /* unaligned S/G addresses present; need to use double buffers */
    return(ata_write_unaligned(iorb, slot));
  }

  /* prepare write command while keeping an eye on S/G count limitations */
  do {
    sg_indx = ata_get_sg_indx(io);
    sg_cnt = io->cSGList - sg_indx;
    if ((rc = ata_cmd_write(iorb, ai, p, d, slot, sector, count,
                            io->pSGList + sg_indx, sg_cnt,
                            io->Flags & XIO_DISABLE_HW_WRITE_CACHE)) > 0) {
      /* couldn't map all S/G elements */
      ata_max_sg_cnt(io, sg_indx, (USHORT) rc, &sg_cnt, &count);
    }
  } while (rc > 0 && sg_cnt > 0);

  if (rc == 0) {
    add_workspace(iorb)->blocks = count;
    add_workspace(iorb)->ppfunc = ata_write_pp;

  } else if (rc > 0) {
    iorb_seterr(iorb, IOERR_CMD_SGLIST_BAD);

  } else if (rc == ATA_CMD_UNALIGNED_ADDR) {
    /* unaligned S/G addresses detected; need to use double buffers */
    add_workspace(iorb)->unaligned = 1;
    return(ata_write_unaligned(iorb, slot));

  } else {
    iorb_seterr(iorb, IOERR_CMD_ADD_SOFTWARE_FAILURE);
  }

  return(rc);
}

/******************************************************************************
 * Write sectors from AHCI device with unaligned S/G element addresses. AHCI
 * only allows aligned S/G addresses while OS/2 doesn't have these kind of
 * restrictions. This doesn't happen very often but when it does, we need to
 * use a transfer buffer and copy the data manually.
 */
int ata_write_unaligned(IORBH _far *iorb, int slot)
{
  IORB_EXECUTEIO _far *io = (IORB_EXECUTEIO _far *) iorb;
  ADD_WORKSPACE _far *aws = add_workspace(iorb);
  AD_INFO *ai = ad_infos + iorb_unit_adapter(iorb);
  ULONG sector = io->RBA + io->BlocksXferred;
  SCATGATENTRY sg_single;
  int p = iorb_unit_port(iorb);
  int d = iorb_unit_device(iorb);
  int rc;

  ddprintf("ata_write_unaligned(%d.%d.%d, %ld)\n", ad_no(ai), p, d, sector);

  /* allocate transfer buffer */
  if ((aws->buf = malloc(io->BlockSize)) == NULL) {
    iorb_seterr(iorb, IOERR_CMD_SW_RESOURCE);
    return(-1);
  }

  /* copy next sector from S/G list to transfer buffer */
  sg_memcpy(io->pSGList, io->cSGList,
            (ULONG) io->BlocksXferred * (ULONG) io->BlockSize,
            aws->buf, io->BlockSize, SG_TO_BUF);

  /* prepare write command using transfer buffer */
  sg_single.ppXferBuf = virt_to_phys(aws->buf);
  sg_single.XferBufLen = io->BlockSize;
  rc = ata_cmd_write(iorb, ai, p, d, slot, sector, 1, &sg_single, 1,
                     io->Flags & XIO_DISABLE_HW_WRITE_CACHE);

  if (rc == 0) {
    add_workspace(iorb)->blocks = 1;
    add_workspace(iorb)->ppfunc = ata_write_pp;

  } else if (rc > 0) {
    iorb_seterr(iorb, IOERR_CMD_SGLIST_BAD);

  } else {
    iorb_seterr(iorb, IOERR_CMD_ADD_SOFTWARE_FAILURE);
  }

  return(rc);
}


/******************************************************************************
 * Post processing function for ata_write(); this function updates the
 * BlocksXferred counter in the IORB and, if not all blocks have been
 * transferred, requeues the IORB to process the remaining sectors.
 */
void ata_write_pp(IORBH _far *iorb)
{
  IORB_EXECUTEIO _far *io = (IORB_EXECUTEIO _far *) iorb;

  io->BlocksXferred += add_workspace(iorb)->blocks;
  ddprintf("ata_write_pp(): blocks transferred = %d\n", (int) io->BlocksXferred);

  if (io->BlocksXferred >= io->BlockCount) {
    /* we're done; tell IRQ handler the IORB is complete */
    add_workspace(iorb)->complete = 1;
  } else {
    /* requeue this IORB for next iteration */
    iorb_requeue(iorb);
  }
}

/******************************************************************************
 * Execute ATA command.
 */
int ata_execute_ata(IORBH _far *iorb, int slot)
{
  IORB_ADAPTER_PASSTHRU _far *apt = (IORB_ADAPTER_PASSTHRU _far *) iorb;
  AD_INFO *ai = ad_infos + iorb_unit_adapter(iorb);
  int p = iorb_unit_port(iorb);
  int d = iorb_unit_device(iorb);
  int rc;

  if (apt->ControllerCmdLen != sizeof(ATA_CMD)) {
    iorb_seterr(iorb, IOERR_CMD_SYNTAX);
    return(-1);
  }

  rc = ata_cmd(ai, p, d, slot, 0,
               AP_SGLIST,   apt->pSGList, apt->cSGList,
               AP_ATA_CMD,  apt->pControllerCmd,
               AP_WRITE,    !(apt->Flags & PT_DIRECTION_IN),
               AP_END);

  if (rc == 0) {
    add_workspace(iorb)->ppfunc = ata_execute_ata_pp;
  }

  return(rc);
}

/******************************************************************************
 * Post processing function for ata_execute_ata(); the main purpose of this
 * function is to copy the received D2H FIS (i.e. the device registers after
 * command completion) back to the ATA command structure.
 *
 * See ata_cmd_to_fis() for an explanation of the mapping.
 */
void ata_execute_ata_pp(IORBH _far *iorb)
{
  AHCI_PORT_DMA _far *dma_base;
  ATA_CMD _far *cmd;
  AD_INFO *ai;
  u8 _far *fis;
  int p;

  /* get address of D2H FIS */
  ai = ad_infos + iorb_unit_adapter(iorb);
  p = iorb_unit_port(iorb);
  dma_base = port_dma_base(ai, p);
  fis = dma_base->rx_fis + 0x40;

  if (fis[0] != 0x34) {
    /* this is not a D2H FIS - give up silently */
    ddprintf("ata_execute_ata_pp(): D2H FIS type incorrect: %d\n", fis[0]);
    add_workspace(iorb)->complete = 1;
    return;
  }

  /* map D2H FIS to the original ATA controller command structure */
  cmd = (ATA_CMD _far *) ((IORB_ADAPTER_PASSTHRU _far *) iorb)->pControllerCmd;

  cmd->cmd      = fis[2];
  cmd->device   = fis[7];
  cmd->features = ((u16) fis[3])
                | ((u16) fis[11]);
  cmd->lba_l    = ((u32) fis[4])
                | ((u32) fis[5] << 8)
                | ((u32) fis[6] << 16)
                | ((u32) fis[8] << 24);
  cmd->lba_h    = ((u16) fis[9])
                | ((u16) fis[10] << 8);
  cmd->count    = ((u16) fis[12])
                | ((u16) fis[13] << 8);

  dphex(cmd, sizeof(*cmd), "ahci_execute_ata_pp(): cmd after completion:\n");

  /* signal completion to interrupt handler */
  add_workspace(iorb)->complete = 1;
}

/******************************************************************************
 * Request sense information for a failed command. Since there is no "request
 * sense" command for ATA devices, we need to read the current error code from
 * the AHCI task file register and fabricate the sense information.
 *
 * NOTES:
 *
 *   - This function must be called right after an ATA command has failed and
 *     before any other commands are queued on the corresponding port. This
 *     function is typically called in the port restart context hook which is
 *     triggered by an AHCI error interrupt.
 *
 *   - The ATA error bits are a complete mess. We'll try and catch the most
 *     interesting error codes (such as medium errors) and report everything
 *     else with a generic error code.
 */
int ata_req_sense(IORBH _far *iorb, int slot)
{
  AD_INFO *ai = ad_infos + iorb_unit_adapter(iorb);
  u8 _far *port_mmio = port_base(ai, iorb_unit_port(iorb));
  u32 tf_data = readl(port_mmio + PORT_TFDATA);
  u8 err = (u8) (tf_data >> 8);
  u8 sts = (u8) (tf_data);

  if (sts & ATA_ERR) {
    if (sts & ATA_DF) {
      /* there is a device-specific error condition */
      if (err & ATA_ICRC) {
        iorb_seterr(iorb, IOERR_ADAPTER_DEVICEBUSCHECK);
      } else if (err & ATA_UNC) {
        iorb_seterr(iorb, IOERR_MEDIA);
      } else if (err & ATA_IDNF) {
        iorb_seterr(iorb, IOERR_RBA_ADDRESSING_ERROR);
      } else {
        iorb_seterr(iorb, IOERR_DEVICE_NONSPECIFIC);
      }

    } else {
      iorb_seterr(iorb, IOERR_DEVICE_NONSPECIFIC);
    }
  } else {
    /* this function only gets called when we received an error interrupt */
    iorb_seterr(iorb, IOERR_DEVICE_NONSPECIFIC);
  }

  /* Return an error to indicate there's no HW command to be submitted and
   * that the IORB can be completed "as is" (the upstream code expects the
   * IORB error code, if any, to be set when this happens and this is exactly
   * what this function is all about).
   */
  return(-1);
}

/******************************************************************************
 * Extract vendor and device name from an ATA INDENTIFY buffer. Since strings
 * in the indentify buffer are byte-swapped, we need to swap them back.
 */
char *ata_dev_name(u16 *id_buf)
{
  static char dev_name[ATA_ID_PROD_LEN + 1];
  char *t = dev_name;
  char *s = (char *) (id_buf + ATA_ID_PROD);
  int i;

  dev_name[sizeof(dev_name)-1] = '\0';

  for (i = 0; i < ATA_ID_PROD_LEN / 2; i++) {
    *(t++) = s[1];
    *(t++) = s[0];
    s += 2;
  }

  return(dev_name);
}

/******************************************************************************
 * Fabricate ATA READ command based on the capabilities of the corresponding
 * device and the paramters set from above (NCQ, etc).
 */
static int ata_cmd_read(IORBH _far *iorb, AD_INFO *ai, int p, int d, int slot,
                        ULONG sector, ULONG count, SCATGATENTRY _far *sg_list,
                        ULONG sg_cnt)
{
  int rc;

  if (sector >= (1UL << 28) || count > 256 || add_workspace(iorb)->is_ncq) {
    /* need LBA48 for this command */
    if (!ai->ports[p].devs[d].lba48) {
      iorb_seterr(iorb, IOERR_RBA_LIMIT);
      return(-1);
    }
    if (add_workspace(iorb)->is_ncq) {
      /* use NCQ read; count goes into feature register, tag into count! */
      rc = ata_cmd(ai, p, d, slot, ATA_CMD_FPDMA_READ,
                   AP_SECTOR_48, (u32) sector, (u16) 0,
                   AP_FEATURES,  (u16) count,
                   AP_COUNT,     (u16) (slot << 3), /* tag == slot */
                   AP_SGLIST,    sg_list, (u16) sg_cnt,
                   AP_DEVICE,    0x40,
                   AP_END);
    } else {
      rc = ata_cmd(ai, p, d, slot, ATA_CMD_READ_EXT,
                   AP_SECTOR_48, (u32) sector, (u16) 0,
                   AP_COUNT,     (u16) count,
                   AP_SGLIST,    sg_list, (u16) sg_cnt,
                   AP_DEVICE,    0x40,
                   AP_END);
    }

  } else {
    rc = ata_cmd(ai, p, d, slot, ATA_CMD_READ,
                 AP_SECTOR_28, (u32) sector,
                 AP_COUNT,     (u16) count & 0xffU,
                 AP_SGLIST,    sg_list, (u16) sg_cnt,
                 AP_DEVICE,    0x40,
                 AP_END);
  }

  return(rc);
}

/******************************************************************************
 * Fabricate ATA WRITE command based on the capabilities of the corresponding
 * device and the paramters set from above (NCQ, etc)
 */
static int ata_cmd_write(IORBH _far *iorb, AD_INFO *ai, int p, int d, int slot,
                         ULONG sector, ULONG count, SCATGATENTRY _far *sg_list,
                         ULONG sg_cnt, int write_through)
{
  int rc;

  if (sector >= (1UL << 28) || count > 256 || add_workspace(iorb)->is_ncq) {
    /* need LBA48 for this command */
    if (!ai->ports[p].devs[d].lba48) {
      iorb_seterr(iorb, IOERR_RBA_LIMIT);
      return(-1);
    }
    if (add_workspace(iorb)->is_ncq) {
      /* use NCQ write; count goes into feature register, tag into count! */
      rc = ata_cmd(ai, p, d, slot, ATA_CMD_FPDMA_WRITE,
                   AP_SECTOR_48, (u32) sector, (u16) 0,
                   AP_FEATURES,  (u16) count,
                   /* tag = slot */
                   AP_COUNT,     (u16) (slot << 3),
                   AP_SGLIST,    sg_list, (u16) sg_cnt,
                   AP_DEVICE,    0x40,
                   /* force unit access */
                   AP_DEVICE,    (write_through && !force_write_cache) ? 0x80 : 0,
                   AP_WRITE,     1,
                   AP_END);
    } else {
      rc = ata_cmd(ai, p, d, slot, ATA_CMD_WRITE_EXT,
                   AP_SECTOR_48, (u32) sector, (u16) 0,
                   AP_COUNT,     (u16) count,
                   AP_SGLIST,    sg_list, (u16) sg_cnt,
                   AP_DEVICE,    0x40,
                   AP_WRITE,     1,
                   AP_END);
    }

  } else {
    rc = ata_cmd(ai, p, d, slot, ATA_CMD_WRITE,
                 AP_SECTOR_28, (u32) sector,
                 AP_COUNT,     (u16) count & 0xffU,
                 AP_SGLIST,    sg_list, (u16) sg_cnt,
                 AP_DEVICE,    0x40,
                 AP_WRITE,     1,
                 AP_END);
  }

  return(rc);
}
