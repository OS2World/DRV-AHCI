/******************************************************************************
 * atapi.c - ATAPI command processing
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

#include "os2ahci.h"
#include "ata.h"
#include "atapi.h"

/* need this for the SCSI status block */
#include <Dev32scsi.h>

/* -------------------------- macros and constants ------------------------- */

/* ------------------------ typedefs and structures ------------------------ */

/* -------------------------- function prototypes -------------------------- */

static void atapi_req_sense_pp(IORBH FAR16DATA *vIorb, IORBH *pIorb);
static int atapi_pad_cdb(u8 *cmd_in, u16 cmd_in_len, u8 *cmd_out, u16 *cmd_out_len);

/* ------------------------ global/static variables ------------------------ */

/* ----------------------------- start of code ----------------------------- */

/******************************************************************************
 * Get device or media geometry. This function is not expected to be called.
 */
int atapi_get_geometry(IORBH FAR16DATA *vIorb, IORBH *pIorb, int slot)
{
  DPRINTF(4,"atapi_get_geometry called\n");
  iorb_seterr(pIorb, IOERR_CMD_NOT_SUPPORTED);
  return(-1);
}

/******************************************************************************
 * Test whether unit is ready. This function is not expected to be called.
 */
int atapi_unit_ready(IORBH FAR16DATA *vIorb, IORBH *pIorb, int slot)
{
  DPRINTF(4,"atapi_unit_ready called\n");
  iorb_seterr(pIorb, IOERR_CMD_NOT_SUPPORTED);
  return(-1);
}

/******************************************************************************
 * Read sectors from AHCI device.
 */
int atapi_read(IORBH FAR16DATA *vIorb, IORBH *pIorb, int slot)
{
  IORB_EXECUTEIO *io = (IORB_EXECUTEIO *)pIorb;
  SCATGATENTRY *pSGList = (SCATGATENTRY*)Far16ToFlat(io->pSGList);
  ATAPI_CDB_12 cdb;
  AD_INFO *ai = ad_infos + iorb_unit_adapter(pIorb);
  USHORT count = io->BlockCount - io->BlocksXferred;
  USHORT sg_indx;
  USHORT sg_cnt;
  int p = iorb_unit_port(pIorb);
  int d = iorb_unit_device(pIorb);
  int rc;

  if (io->BlockCount == 0)
  {
    /* NOP; return -1 without error in IORB to indicate success */
    return(-1);
  }

  if (add_workspace(pIorb)->unaligned)
  {
    /* unaligned S/G addresses present; need to use double buffers */
    return(atapi_read_unaligned(pIorb, slot));
  }

  /* translate read command to SCSI/ATAPI READ12 command.
   * READ12 seems to be the most supported READ variant - according to MMC,
   * and it's enough even for BluRay.
   */
  memset(&cdb, 0x00, sizeof(cdb));
  cdb.cmd = ATAPI_CMD_READ_12;
  SET_CDB_32(cdb.lba, io->RBA + io->BlocksXferred);

  DPRINTF(4, "atapi_read\n");

  do {
    /* update sector count (might have been updated due to S/G limitations) */
    SET_CDB_32(cdb.trans_len, count);

    /* update S/G count and index */
    sg_indx = ata_get_sg_indx(io);
    sg_cnt = io->cSGList - sg_indx;

    /* issue command */
    rc = ata_cmd(ai, p, d, slot, ATA_CMD_PACKET,
                 AP_ATAPI_CMD, (void *) &cdb, sizeof(cdb),
                 AP_SGLIST,    pSGList + sg_indx, sg_cnt,
                 AP_DEVICE,    0x40,
                 AP_FEATURES,  ATAPI_FEAT_DMA | ATAPI_FEAT_DMA_TO_HOST,
                 AP_END);

    if (rc > 0)
    {
      /* couldn't map all S/G elements */
      ata_max_sg_cnt(io, sg_indx, (USHORT) rc, &sg_cnt, &count);
    }
  } while (rc > 0 && sg_cnt > 0);

  if (rc == 0)
  {
    add_workspace(pIorb)->blocks = count;
    add_workspace(pIorb)->ppfunc = ata_read_pp;
  }
  else if (rc > 0)
  {
    iorb_seterr(pIorb, IOERR_CMD_SGLIST_BAD);
  }
  else if (rc == ATA_CMD_UNALIGNED_ADDR)
  {
    /* unaligned S/G addresses detected; need to use double buffers */
    add_workspace(pIorb)->unaligned = 1;
    return(atapi_read_unaligned(pIorb, slot));
  }
  else
  {
    iorb_seterr(pIorb, IOERR_CMD_ADD_SOFTWARE_FAILURE);
  }

  return(rc);
}

/******************************************************************************
 * Read sectors from AHCI device with unaligned S/G element addresses. AHCI
 * only allows aligned S/G addresses while OS/2 doesn't have these kind of
 * restrictions. This doesn't happen very often but when it does, we need to
 * use a transfer buffer and copy the data manually.
 */
int atapi_read_unaligned(IORBH *pIorb, int slot)
{
  IORB_EXECUTEIO *io = (IORB_EXECUTEIO *)pIorb;
  ADD_WORKSPACE *aws = add_workspace(pIorb);
  ATAPI_CDB_12 cdb;
  AD_INFO *ai = ad_infos + iorb_unit_adapter(pIorb);
  int p = iorb_unit_port(pIorb);
  int d = iorb_unit_device(pIorb);
  int rc;

  /* translate read command to SCSI/ATAPI READ12 command.
   * READ12 seems to be the most supported READ variant - according to MMC,
   * and it's enough even for BluRay.
   */
  memset(&cdb, 0x00, sizeof(cdb));
  cdb.cmd = ATAPI_CMD_READ_12;
  SET_CDB_32(cdb.lba, io->RBA + io->BlocksXferred);
  SET_CDB_32(cdb.trans_len, 1UL);

  ai->ports[p].unaligned_read_count++;
  DPRINTF(4, "atapi_read_unaligned\n");

  /* allocate transfer buffer */
  if ((aws->buf = MemAlloc(io->BlockSize)) == NULL)
  {
    iorb_seterr(pIorb, IOERR_CMD_SW_RESOURCE);
    return(-1);
  }

  rc = ata_cmd(ai, p, d, slot, ATA_CMD_PACKET,
               AP_ATAPI_CMD, (void *) &cdb, sizeof(cdb),
               AP_VADDR,     (void *) aws->buf, io->BlockSize,
               AP_DEVICE,    0x40,
               AP_FEATURES,  ATAPI_FEAT_DMA | ATAPI_FEAT_DMA_TO_HOST,
               AP_END);

  if (rc == 0)
  {
    add_workspace(pIorb)->blocks = 1;
    add_workspace(pIorb)->ppfunc = ata_read_pp;

  }
  else if (rc > 0)
  {
    iorb_seterr(pIorb, IOERR_CMD_SGLIST_BAD);

  }
  else
  {
    iorb_seterr(pIorb, IOERR_CMD_ADD_SOFTWARE_FAILURE);
  }

  return(rc);
}

/******************************************************************************
 * Verify readability of sectors on AHCI device. This function is not expected
 * to be called.
 */
int atapi_verify(IORBH FAR16DATA *vIorb, IORBH *pIorb, int slot)
{
  DPRINTF(4,"atapi_verify called\n");
  iorb_seterr(pIorb, IOERR_CMD_NOT_SUPPORTED);
  return(-1);
}

/******************************************************************************
 * Write sectors to AHCI device. This function is not expected to be called.
 */
int atapi_write(IORBH FAR16DATA *vIorb, IORBH *pIorb, int slot)
{
  DPRINTF(4,"atapi_write called\n");
  iorb_seterr(pIorb, IOERR_CMD_NOT_SUPPORTED);
  return(-1);
}

/******************************************************************************
 * Execute ATAPI command.
 */
int atapi_execute_cdb(IORBH FAR16DATA *vIorb, IORBH *pIorb, int slot)
{
  IORB_ADAPTER_PASSTHRU *pt = (IORB_ADAPTER_PASSTHRU *)pIorb;
  SCATGATENTRY *pSGList = (SCATGATENTRY*)Far16ToFlat(pt->pSGList);
  int rc;
  u8 cdb[ATAPI_MAX_CDB_LEN];
  u16 cdb_len;

  if (pt->ControllerCmdLen > ATAPI_MAX_CDB_LEN)
  {
    iorb_seterr(pIorb, IOERR_CMD_SYNTAX);
    return -1;
  }
  /* AHCI requires 12 or 16 byte commands */
  atapi_pad_cdb(Far16ToFlat(pt->pControllerCmd), pt->ControllerCmdLen, cdb, &cdb_len);

  if (cdb[0] == 0x12 || cdb[0] == 0x5a)
  {
    /* somebody sets the direction flag incorrectly for those commands */
    pt->Flags |= PT_DIRECTION_IN;
  }

  /* we do not perform the S/G limitation recovery loop here:
   * "ADDs are not required to iterate commands through the CDB PassThru
   * mechanism:" -- Storage Device Driver Reference, Scatter/Gather Lists
   */
  rc = ata_cmd(ad_infos + iorb_unit_adapter(pIorb), iorb_unit_port(pIorb),
               iorb_unit_device(pIorb), slot, ATA_CMD_PACKET,
               AP_ATAPI_CMD, (void *)cdb, cdb_len,
               AP_SGLIST, pSGList, pt->cSGList,
               AP_WRITE, !(pt->Flags & PT_DIRECTION_IN),
               AP_FEATURES, ATAPI_FEAT_DMA,
               AP_FEATURES, (pt->Flags & PT_DIRECTION_IN) ? ATAPI_FEAT_DMA_TO_HOST : 0,
               AP_END);

  if (rc)
  {
    iorb_seterr(pIorb, IOERR_DEVICE_NONSPECIFIC);
  }

  return(rc);
}


/******************************************************************************
 * Request sense information for a failed command.
 *
 * NOTE: This function must be called right after an ATAPI command has failed
 *       and before any other commands are queued on the corresponding device.
 *       This function is typically called in the port restart context hook
 *       which is triggered by an AHCI error interrupt.
 *
 */
int atapi_req_sense(IORBH FAR16DATA *vIorb, IORBH *pIorb, int slot)
{
  SCSI_STATUS_BLOCK *ssb;
  ADD_WORKSPACE *aws = add_workspace(pIorb);
  int rc;
  u8 cdb[ATAPI_MIN_CDB_LEN];
  ATAPI_CDB_6 *pcdb = (ATAPI_CDB_6 *) cdb;
  size_t sense_buf_len = ATAPI_SENSE_LEN;

  DPRINTF(4,"atapi_req_sense\n");

  if ((pIorb->RequestControl & IORB_REQ_STATUSBLOCK) &&
      pIorb->StatusBlockLen >= sizeof(*ssb) && pIorb->pStatusBlock != 0)
  {
    ULONG ulTmp;

    ulTmp = (CastFar16ToULONG(vIorb) & 0xffff0000) + pIorb->pStatusBlock;
    ssb = (SCSI_STATUS_BLOCK *)Far16ToFlat(CastULONGToFar16(ulTmp));

    /* don't request sense data if caller asked us not to; the flag
     * STATUS_DISABLE_REQEST_SENSE is not defined in the old DDK we've been
     * using so we'll use the hard-coded value (0x0008) */
    if (ssb->Flags & 0x0008U)
    {
      iorb_seterr(pIorb, IOERR_DEVICE_NONSPECIFIC);
      return(-1);
    }

    /* if the sense buffer requested is larger than our default, adjust
     * the length accordingly to satisfy the caller's requirements. */
    if (ssb->SenseData != NULL && ssb->ReqSenseLen > sense_buf_len)
    {
      sense_buf_len = ssb->ReqSenseLen;
    }
  }

  /* allocate sense buffer in ADD workspace */
  if ((aws->buf = MemAlloc(sense_buf_len)) == NULL)
  {
    iorb_seterr(pIorb, IOERR_CMD_SW_RESOURCE);
    return(-1);
  }
  memset(aws->buf, 0x00, sense_buf_len);

  /* prepare request sense command */
  memset(cdb, 0x00, sizeof(cdb));
  pcdb->cmd = ATAPI_CMD_REQUEST_SENSE;
  pcdb->trans_len = (u8) sense_buf_len;

  aws->ppfunc = atapi_req_sense_pp;
  rc = ata_cmd(ad_infos + iorb_unit_adapter(pIorb),
               iorb_unit_port(pIorb),
               iorb_unit_device(pIorb),
               slot,
               ATA_CMD_PACKET,
               AP_ATAPI_CMD, (void *)cdb, sizeof(cdb),
               AP_VADDR, (void *)aws->buf, sense_buf_len,
               AP_FEATURES,  ATAPI_FEAT_DMA,
               AP_END);

  if (rc > 0)
  {
    iorb_seterr(pIorb, IOERR_CMD_SGLIST_BAD);
  }
  else if (rc < 0)
  {
    /* we failed to get info about an error -> return
     * non specific device error
     */
    iorb_seterr(pIorb, IOERR_DEVICE_NONSPECIFIC);
  }

  return(rc);
}

/******************************************************************************
 * Post processing function for ATAPI request sense; examines the sense
 * data returned and maps sense info to IORB error info.
 */
static void atapi_req_sense_pp(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  SCSI_STATUS_BLOCK *ssb;
  ADD_WORKSPACE *aws = add_workspace(pIorb);
  ATAPI_SENSE_DATA *psd = (ATAPI_SENSE_DATA *) aws->buf;

  DHEXDUMP(5,psd, sizeof(*psd), "sense buffer:\n");

  if ((pIorb->RequestControl & IORB_REQ_STATUSBLOCK) &&
      pIorb->StatusBlockLen >= sizeof(*ssb) && pIorb->pStatusBlock != 0)
  {
    ULONG ulTmp;

    /* copy sense data to IORB */
    ulTmp = (CastFar16ToULONG(vIorb) & 0xffff0000) + pIorb->pStatusBlock;
    ssb = (SCSI_STATUS_BLOCK *)Far16ToFlat(CastULONGToFar16(ulTmp));
    ssb->AdapterErrorCode = 0;
    ssb->TargetStatus = SCSI_STAT_CHECKCOND;
    ssb->ResidualLength = 0;
    memset(ssb->AdapterDiagInfo, 0x00, sizeof(ssb->AdapterDiagInfo));

    if (ssb->SenseData != NULL)
    {
      memcpy(Far16ToFlat(ssb->SenseData), psd, ssb->ReqSenseLen);
      ssb->Flags |= STATUS_SENSEDATA_VALID;
    }
    pIorb->Status |= IORB_STATUSBLOCK_AVAIL;
  }

  /* map sense data to some IOERR_ value */
  switch (ATAPI_GET_SENSE(psd))
  {
  case ASENSE_NO_SENSE:
  case ASENSE_RECOVERED_ERROR:
    /* no error; this shouldn't happen because we'll only call
     * atapi_req_sense() if we received an error interrupt */
    iorb_seterr(pIorb, IOERR_DEVICE_NONSPECIFIC);
    break;

  case ASENSE_NOT_READY:
    iorb_seterr(pIorb, IOERR_UNIT_NOT_READY);
    break;

  case ASENSE_UNIT_ATTENTION:
    iorb_seterr(pIorb, IOERR_MEDIA_CHANGED);
    break;

  case ASENSE_MEDIUM_ERROR:
    iorb_seterr(pIorb, IOERR_MEDIA);
    break;

  case ASENSE_ILLEGAL_REQUEST:
    iorb_seterr(pIorb, IOERR_CMD_SYNTAX);
    break;

  case ASENSE_DATA_PROTECT:
    iorb_seterr(pIorb, IOERR_MEDIA_WRITE_PROTECT);
    break;

  case ASENSE_BLANK_CHECK:
    iorb_seterr(pIorb, IOERR_MEDIA_NOT_FORMATTED);
    break;

  case ASENSE_ABORTED_COMMAND:
  case ASENSE_COPY_ABORTED:
    iorb_seterr(pIorb, IOERR_CMD_ABORTED);
    break;

  default:
    iorb_seterr(pIorb, IOERR_DEVICE_NONSPECIFIC);
    break;
  }

  /* mark IORB as complete */
  aws->complete = 1;
}

/******************************************************************************
 * Pad ATAPI commands; AHCI requires ATAPI commands to be either 12 or
 * 16 bytes in length. This func converts commands that have a 12 byte
 * equivalent, and pads the others to 12 bytes.
 * cmd_out buffer is expected to be ATAPI_MAX_CDB_LEN in size.
 * returns 0 on success, != 0 if the command can't be converted.
 */
int atapi_pad_cdb(u8 *cmd_in, u16 cmd_in_len, u8 *cmd_out, u16 *cmd_out_len)
{
  ATAPI_CDB_12 *p12;
  u32 tmp;

  if (cmd_in_len == ATAPI_MIN_CDB_LEN || cmd_in_len == ATAPI_MAX_CDB_LEN)
  {
    /* command does not need to be converted */
    memcpy(cmd_out, cmd_in, cmd_in_len);
    *cmd_out_len = cmd_in_len;
    return 0;
  }

  memset(cmd_out, 0x00, ATAPI_MAX_CDB_LEN);
  p12 = (ATAPI_CDB_12 *) cmd_out;
  /* we always convert to 12 byte CDBs */
  *cmd_out_len = ATAPI_MIN_CDB_LEN;

  /* check if command can be converted */
  switch (cmd_in[0])
  {
  case ATAPI_CMD_READ_6:
  case ATAPI_CMD_WRITE_6:
    /* convert from 6 to 12 byte equivalent */
    p12->cmd = 0xa0 | (cmd_in[0] & 0x0f);
    p12->flags = cmd_in[1] & 0xc0; /* 6byte cmds have no flags (FUA etc.) */
    tmp = GET_CDB_24(cmd_in + 1) & 0x1fffffUL;
    SET_CDB_32(p12->lba, tmp);
    SET_CDB_32(p12->trans_len, (cmd_in[4]));
    p12->control = cmd_in[5];
    break;

  case ATAPI_CMD_READ_10:
  case ATAPI_CMD_WRITE_10:
    /* convert from 10 byte to 12 byte equivalent */
    p12->cmd = 0xa0 | (cmd_in[0] & 0x0f);
    p12->flags = cmd_in[1];
    p12->control = cmd_in[9];
    memcpy(p12->lba, cmd_in + 2, 4);
    tmp = GET_CDB_16(cmd_in + 7);
    SET_CDB_32(p12->trans_len, tmp);
    break;

  default:
    /* pad with zeroes to 12 bytes */
    memset(cmd_out, 0x00, ATAPI_MIN_CDB_LEN);
    memcpy(cmd_out, cmd_in, cmd_in_len);
    break;
  }

  return 0;
}

