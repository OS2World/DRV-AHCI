/******************************************************************************
 * ioctl.c - Generic IOCTL command processing
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

#include "os2ahci.h"
#include "atapi.h"
#include "ata.h"
#include "ioctl.h"

#include <scsi.h>

#pragma pack(1)

/* -------------------------- macros and constants ------------------------- */

/* ------------------------ typedefs and structures ------------------------ */

/* Memory area for IOCTLs which send IORBs downstream; currently only
 * OS2AHCI_IOCTL_PASSTHROUGH falls into this category, thus we're simply
 * reusing the IORB_ADAPTER_PASSTHRU structure for now. If this ever changes,
 * we'll need to define a union to cover all IORB types in question.
 */
#define CMDLEN sizeof(((OS2AHCI_PASSTHROUGH *) 0)->cmd)
typedef struct {
  IORB_ADAPTER_PASSTHRU iorb;                  /* IORB */
  SCSI_STATUS_BLOCK   ssb;                     /* SCSI status block */
  UCHAR               cmd[CMDLEN];             /* copy of passthrough cmd
                                                * (need to fill ATA return
                                                * registers at interrupt time)
                                                */
  UCHAR               sense[ATAPI_SENSE_LEN];  /* sense buffer */
  SCATGATENTRY        sg_lst[AHCI_MAX_SG / 2]; /* scatter/gather list */
  ULONG               sg_cnt;                  /* number of S/G elements */
  UCHAR               lh[16];                  /* lock handle for VMLock() */
} IOCTL_CONTEXT;

/* -------------------------- function prototypes -------------------------- */

static USHORT do_smart  (BYTE unit, BYTE sub_func, BYTE cnt, BYTE lba_l,
                         void _far *buf);
static int    map_unit  (BYTE unit, USHORT _far *a, USHORT _far *p,
                         USHORT _far *d);
static LIN    lin       (void _far *p);

IORBH _far * _far _cdecl ioctl_wakeup(IORBH _far *iorb);

/* ------------------------ global/static variables ------------------------ */

/* ----------------------------- start of code ----------------------------- */

/******************************************************************************
 * Return device list to allow the ring 3 application to figure out which
 * adapter/port/device combinations are available.
 */
USHORT ioctl_get_devlist(RP_GENIOCTL _far *ioctl)
{
  OS2AHCI_DEVLIST _far *devlst = (OS2AHCI_DEVLIST _far *) ioctl->DataPacket;
  USHORT maxcnt = 0;
  USHORT cnt = 0;
  USHORT a;
  USHORT p;
  USHORT d;

  /* verify addressability of parm buffer (number of devlst elements) */
  if (DevHelp_VerifyAccess((SEL) ((ULONG) ioctl->ParmPacket >> 16),
                            sizeof(USHORT),
                            (USHORT) (ULONG) ioctl->ParmPacket,
                            VERIFY_READONLY) != 0) {
    return(STDON | STERR | 0x05);
  }

  maxcnt = *((USHORT _far *) ioctl->ParmPacket);

  /* verify addressability of return buffer (OS2AHCI_DEVLIST) */
  if (DevHelp_VerifyAccess((SEL) ((ULONG) devlst >> 16),
                            offsetof(OS2AHCI_DEVLIST, devs) +
                              sizeof(devlst->devs) * maxcnt,
                            (USHORT) (ULONG) devlst,
                            VERIFY_READWRITE) != 0) {
    return(STDON | STERR | 0x05);
  }

  /* fill-in device list */
  for (a = 0; a < ad_info_cnt; a++) {
    AD_INFO *ai = ad_infos + a;

    for (p = 0; p <= ai->port_max; p++) {
      P_INFO *pi = ai->ports + p;

      for (d = 0; d <= pi->dev_max; d++) {
        if (pi->devs[d].present) {
          /* add this device to the device list */
          if (cnt >= maxcnt) {
            /* not enough room in devlst */
            goto ioctl_get_device_done;
          }

          devlst->devs[cnt].adapter  = a;
          devlst->devs[cnt].port     = p;
          devlst->devs[cnt].device   = d;
          devlst->devs[cnt].type     = pi->devs[d].dev_type;
          devlst->devs[cnt].ncq_max  = pi->devs[d].ncq_max;

          if (pi->devs[d].lba48)      devlst->devs[cnt].flags |= DF_LBA48;
          if (pi->devs[d].atapi)      devlst->devs[cnt].flags |= DF_ATAPI;
          if (pi->devs[d].atapi_16)   devlst->devs[cnt].flags |= DF_ATAPI_16;
          if (pi->devs[d].removable)  devlst->devs[cnt].flags |= DF_REMOVABLE;
          cnt++;
        }
      }
    }
  }

ioctl_get_device_done:
  devlst->cnt = cnt;
  return(STDON);
}

/******************************************************************************
 * Adapter passthrough IOCTL. This IOCTL covers both ATA and ATAPI passthrough
 * requests.
 */
USHORT ioctl_passthrough(RP_GENIOCTL _far *ioctl)
{
  OS2AHCI_PASSTHROUGH _far *req = (OS2AHCI_PASSTHROUGH _far *) ioctl->ParmPacket;
  char _far *sense_buf = (char _far *) ioctl->DataPacket;
  IOCTL_CONTEXT *ic;
  USHORT ret;
  USHORT a;
  USHORT p;
  USHORT d;

  /* verify addressability of parm buffer (OS2AHCI_PASSTHROUGH) */
  if (DevHelp_VerifyAccess((SEL) ((ULONG) req >> 16),
                            sizeof(OS2AHCI_PASSTHROUGH),
                            (USHORT) (ULONG) req,
                            VERIFY_READWRITE) != 0) {
    return(STDON | STERR | 0x05);
  }

  /* verify addressability of data buffer (sense data) */
  if (req->sense_len > 0) {
    if (DevHelp_VerifyAccess((SEL) ((ULONG) sense_buf >> 16),
                              req->sense_len,
                              (USHORT) (ULONG) sense_buf,
                              VERIFY_READWRITE) != 0) {
      return(STDON | STERR | 0x05);
    }
  }

  /* Verify basic request parameters such as adapter/port/device, size of
   * DMA buffer (the S/G list can't have more than AHCI_MAX_SG / 2 entries), ...
   */
  a = req->adapter;
  p = req->port;
  d = req->device;
  if (a >= ad_info_cnt || p > ad_infos[a].port_max ||
      d > ad_infos[a].ports[p].dev_max || !ad_infos[a].ports[p].devs[d].present) {
    return(STDON | STERR | ERROR_I24_BAD_UNIT);
  }
  if ((req->buflen + 4095) / 4096 + 1 > AHCI_MAX_SG / 2 ||
      req->cmdlen < 6 || req->cmdlen > sizeof(req->cmd)) {
    return(STDON | STERR | ERROR_I24_INVALID_PARAMETER);
  }

  /* allocate IOCTL context data */
  if ((ic = malloc(sizeof(*ic))) == NULL) {
    return(STDON | STERR | ERROR_I24_GEN_FAILURE);
  }
  memset(ic, 0x00, sizeof(*ic));

  /* lock DMA transfer buffer into memory and construct S/G list */
  if (req->buflen > 0) {
    if (DevHelp_VMLock(VMDHL_LONG | !((req->flags & PT_WRITE) ? VMDHL_WRITE : 0),
                       req->buf, req->buflen, lin(ic->sg_lst), lin(&ic->lh),
                       &ic->sg_cnt) != 0) {
      /* couldn't lock buffer and/or produce a S/G list */
      free(ic);
      return(STDON | STERR | ERROR_I24_INVALID_PARAMETER);
    }
  }

  /* fill in adapter passthrough fields */
  ic->iorb.iorbh.Length           = sizeof(ic->iorb);
  ic->iorb.iorbh.UnitHandle       = iorb_unit(a, p, d);
  ic->iorb.iorbh.CommandCode      = IOCC_ADAPTER_PASSTHRU;
  ic->iorb.iorbh.CommandModifier  = (req->flags & PT_ATAPI) ? IOCM_EXECUTE_CDB : IOCM_EXECUTE_ATA;
  ic->iorb.iorbh.RequestControl   = IORB_ASYNC_POST;
  ic->iorb.iorbh.Timeout          = req->timeout;
  ic->iorb.iorbh.NotifyAddress    = ioctl_wakeup;

  ic->iorb.cSGList          = ic->sg_cnt;
  ic->iorb.pSGList          = ic->sg_lst;
  ic->iorb.ppSGLIST         = virt_to_phys(ic->sg_lst);

  memcpy(ic->cmd, req->cmd.cdb, sizeof(ic->cmd));
  ic->iorb.ControllerCmdLen = req->cmdlen;
  ic->iorb.pControllerCmd   = ic->cmd;
  ic->iorb.Flags            = (req->flags & PT_WRITE) ? 0 : PT_DIRECTION_IN;

  if (req->sense_len > 0) {
    /* initialize SCSI status block to allow getting sense data */
    ic->iorb.iorbh.pStatusBlock     = (BYTE *) &ic->ssb;
    ic->iorb.iorbh.StatusBlockLen   = sizeof(ic->ssb);
    ic->ssb.SenseData               = (SCSI_REQSENSE_DATA _far *) ic->sense;
    ic->ssb.ReqSenseLen             = sizeof(ic->sense);
    ic->iorb.iorbh.RequestControl  |= IORB_REQ_STATUSBLOCK;
  }

  /* send IORB on its way */
  add_entry(&ic->iorb.iorbh);

  /* Wait for IORB completion. */
  spin_lock(drv_lock);
  while (!(ic->iorb.iorbh.Status & IORB_DONE)) {
    DevHelp_ProcBlock((ULONG) (void _far *) &ic->iorb.iorbh, 30000, 1);
  }
  spin_unlock(drv_lock);

  ret = STDON;

  /* map IORB error codes to device driver error codes */
  if (ic->iorb.iorbh.Status & IORB_ERROR) {
    ret |= STERR;

    switch (ic->iorb.iorbh.ErrorCode) {

    case IOERR_UNIT_NOT_READY:
      ret |= ERROR_I24_NOT_READY;
      break;

    case IOERR_MEDIA_CHANGED:
      ret |= ERROR_I24_DISK_CHANGE;
      break;

    case IOERR_MEDIA:
    case IOERR_MEDIA_NOT_FORMATTED:
      ret |= ERROR_I24_CRC;
      break;

    case IOERR_CMD_SYNTAX:
    case IOERR_CMD_NOT_SUPPORTED:
      ret |= ERROR_I24_BAD_COMMAND;
      break;

    case IOERR_MEDIA_WRITE_PROTECT:
      ret |= ERROR_I24_WRITE_PROTECT;
      break;

    case IOERR_CMD_ABORTED:
      ret |= ERROR_I24_CHAR_CALL_INTERRUPTED;
      break;

    case IOERR_RBA_ADDRESSING_ERROR:
      ret |= ERROR_I24_SEEK;
      break;

    case IOERR_RBA_LIMIT:
      ret |= ERROR_I24_SECTOR_NOT_FOUND;
      break;

    case IOERR_CMD_SGLIST_BAD:
      ret |= ERROR_I24_INVALID_PARAMETER;
      break;

    case IOERR_DEVICE_NONSPECIFIC:
    case IOERR_ADAPTER_TIMEOUT:
    case IOERR_ADAPTER_DEVICEBUSCHECK:
    case IOERR_CMD_ADD_SOFTWARE_FAILURE:
    case IOERR_CMD_SW_RESOURCE:
    default:
      ret |= ERROR_I24_GEN_FAILURE;
      break;
    }

    /* copy sense information, if there is any */
    if ((ic->iorb.iorbh.Status & IORB_STATUSBLOCK_AVAIL) &&
        (ic->ssb.Flags | STATUS_SENSEDATA_VALID)) {
      memcpy(sense_buf, ic->ssb.SenseData,
             min(ic->ssb.ReqSenseLen, req->sense_len));
    }

  } else if ((req->flags & PT_ATAPI) == 0) {
    /* Copy ATA cmd back to IOCTL request (ATA commands are effectively
     * registers which are sometimes used to indicate return conditions,
     * e.g. when requesting the smart status)
     */
    memcpy(&req->cmd.ata, ic->cmd, sizeof(req->cmd.ata));
  }

  free(ic);
  if (req->buflen > 0) {
    DevHelp_VMUnLock(lin(ic->lh));
  }
  return(ret);
}

/******************************************************************************
 * Generic disk IOCTL handler; this IOCTL category has originally been defined
 * in IBM1S506; the code has been more or less copied from DANIS506.
 *
 * NOTE: Only a subset of the IOCTL calls are implemented in OS2AHCI at this
 *       point, basically those calls required to get HDMON working.
 */
USHORT ioctl_gen_dsk(RP_GENIOCTL _far *ioctl)
{
  DSKSP_CommandParameters _far *cp = (DSKSP_CommandParameters _far *) ioctl->ParmPacket;
  UnitInformationData _far *ui;
  OS2AHCI_PASSTHROUGH pt;
  RP_GENIOCTL tmp_ioctl;
  USHORT size = 0;
  USHORT ret;
  USHORT a;
  USHORT p;
  USHORT d;
  UCHAR unit;

  /* verify addressability of parm buffer (DSKSP_CommandParameters) */
  if (DevHelp_VerifyAccess((SEL) ((ULONG) cp >> 16),
                            sizeof(DSKSP_CommandParameters),
                            (USHORT) (ULONG) cp,
                            VERIFY_READONLY) != 0) {
    return(STDON | STERR | 0x05);
  }
  unit = cp->byPhysicalUnit;

  /* verify addressability of data buffer (depends on function code) */
  switch (ioctl->Function) {

  case DSKSP_GEN_GET_COUNTERS:
    size = sizeof(DeviceCountersData);
    break;

  case DSKSP_GET_UNIT_INFORMATION:
    size = sizeof(UnitInformationData);
    break;

  case DSKSP_GET_INQUIRY_DATA:
    size = ATA_ID_WORDS * sizeof(u16);
    break;
  }

  if (size > 0) {
    if (DevHelp_VerifyAccess((SEL) ((ULONG) ioctl->DataPacket >> 16),
                             size, (USHORT) (ULONG) ioctl->DataPacket,
                             VERIFY_READWRITE) != 0) {
      return(STDON | STERR | 0x05);
    }
  }

  if (map_unit(unit, &a, &p, &d)) {
    return(STDON | STERR | ERROR_I24_BAD_UNIT);
  }

  /* execute generic disk request */
  switch (ioctl->Function) {

  case DSKSP_GEN_GET_COUNTERS:
    /* Not supported, yet; we would need dynamically allocated device
     * structures to cope with the memory requirements of the corresponding
     * statistics buffer. For the time being, we'll return an empty buffer.
     */
    memset(ioctl->DataPacket, 0x00, sizeof(DeviceCountersData));
    ret = STDON;
    break;

  case DSKSP_GET_UNIT_INFORMATION:
    /* get unit information; things like port addresses won't fit so we don't
     * even bother returning those.
     */
    ui = (UnitInformationData _far *) ioctl->DataPacket;
    memset(ui, 0x00, sizeof(*ui));

    ui->wRevisionNumber = 1;
    ui->wIRQ            = ad_infos[a].irq;
    ui->wFlags          = UIF_VALID;
    ui->wFlags         |= UIF_RUNNING_BMDMA;
    ui->wFlags         |= (unit & 0x0001) ? UIF_SLAVE : 0;
    ui->wFlags         |= (ad_infos[a].ports[p].devs[d].atapi) ? UIF_ATAPI : 0;
    ui->wFlags         |= UIF_SATA;

    ret = STDON;
    break;

  case DSKSP_GET_INQUIRY_DATA:
    /* return ATA ID buffer */
    memset(&tmp_ioctl, 0x00, sizeof(tmp_ioctl));
    tmp_ioctl.Category   = OS2AHCI_IOCTL_CATEGORY;
    tmp_ioctl.Function   = OS2AHCI_IOCTL_PASSTHROUGH;
    tmp_ioctl.ParmPacket = (void _far *) &pt;

    memset(&pt, 0x00, sizeof(pt));
    pt.adapter          = a;
    pt.port             = p;
    pt.device           = d;
    pt.cmdlen           = sizeof(pt.cmd.ata);
    pt.cmd.ata.cmd      = (ad_infos[a].ports[p].devs[d].atapi) ?
                           ATA_CMD_ID_ATAPI : ATA_CMD_ID_ATA;
    pt.buflen           = size;
    pt.buf              = lin(ioctl->DataPacket);

    ret = gen_ioctl(&tmp_ioctl);
    break;

  default:
    ret = STDON | STATUS_ERR_UNKCMD;
    break;
  }

  return(ret);
}

/******************************************************************************
 * SMART IOCTL handler; this IOCTL category has originally been defined in
 * IBM1S506; the code has been more or less copied from DANIS506.
 */
USHORT ioctl_smart(RP_GENIOCTL _far *ioctl)
{
  DSKSP_CommandParameters _far *cp = (DSKSP_CommandParameters _far *) ioctl->ParmPacket;
  USHORT size = 0;
  USHORT ret;
  UCHAR unit;
  UCHAR parm;

  /* verify addressability of parm buffer (DSKSP_CommandParameters) */
  if (DevHelp_VerifyAccess((SEL) ((ULONG) cp >> 16),
                            sizeof(DSKSP_CommandParameters),
                            (USHORT) (ULONG) cp,
                            VERIFY_READONLY) != 0) {
    return(STDON | STERR | 0x05);
  }
  unit = cp->byPhysicalUnit;

  /* verify addressability of data buffer (depends on SMART function) */
  switch (ioctl->Function) {

  case DSKSP_SMART_GETSTATUS:
    size = sizeof(ULONG);
    break;

  case DSKSP_SMART_GET_ATTRIBUTES:
  case DSKSP_SMART_GET_THRESHOLDS:
  case DSKSP_SMART_GET_LOG:
    size = 512;
    break;
  }

  if (size > 0) {
    if (DevHelp_VerifyAccess((SEL) ((ULONG) ioctl->DataPacket >> 16),
                             size, (USHORT) (ULONG) ioctl->DataPacket,
                             VERIFY_READWRITE) != 0) {
      return(STDON | STERR | 0x05);
    }
    parm = ioctl->DataPacket[0];
  }

  /* execute SMART request */
  switch (ioctl->Function) {

  case DSKSP_SMART_ONOFF:
    ret = do_smart(unit, (BYTE) ((parm) ? ATA_SMART_ENABLE : ATA_SMART_DISABLE), 0, 0, NULL);
    break;

  case DSKSP_SMART_AUTOSAVE_ONOFF:
    ret = do_smart(unit, ATA_SMART_AUTOSAVE, (BYTE) ((parm) ? (BYTE) 0xf1 : 0), 0, NULL);
    break;

  case DSKSP_SMART_AUTO_OFFLINE:
    ret = do_smart(unit, ATA_SMART_AUTO_OFFLINE, parm, 0, NULL);
    break;

  case DSKSP_SMART_EXEC_OFFLINE:
    ret = do_smart(unit, ATA_SMART_IMMEDIATE_OFFLINE, 0, parm, NULL);
    break;

  case DSKSP_SMART_SAVE:
    ret = do_smart(unit, ATA_SMART_SAVE, 0, 0, NULL);
    break;

  case DSKSP_SMART_GETSTATUS:
    ret = do_smart(unit, ATA_SMART_STATUS, 0, 0, ioctl->DataPacket);
    break;

  case DSKSP_SMART_GET_ATTRIBUTES:
    ret = do_smart(unit, ATA_SMART_READ_VALUES, 0, 0, ioctl->DataPacket);
    break;

  case DSKSP_SMART_GET_THRESHOLDS:
    ret = do_smart(unit, ATA_SMART_READ_THRESHOLDS, 0, 0, ioctl->DataPacket);
    break;

  case DSKSP_SMART_GET_LOG:
    ret = do_smart(unit, ATA_SMART_READ_LOG, 1, parm, ioctl->DataPacket);
    break;

  default:
    ret = STDON | STATUS_ERR_UNKCMD;
  }

  return(ret);
}

/******************************************************************************
 * Perform SMART request. The code has been more or less copied from DANIS506.
 */
static USHORT do_smart(BYTE unit, BYTE sub_func, BYTE cnt, BYTE lba_l, void _far *buf)
{
  OS2AHCI_PASSTHROUGH pt;
  RP_GENIOCTL ioctl;
  USHORT ret;
  USHORT a;
  USHORT p;
  USHORT d;

  if (map_unit(unit, &a, &p, &d)) {
    return(STDON | STERR | ERROR_I24_BAD_UNIT);
  }

  /* Perform SMART request using the existing OS2AHCI_IOTCL_PASSTHROUGH IOCTL
   * interface which already takes care of allocating an IORB, s/g lists, etc.
   */
  memset(&ioctl, 0x00, sizeof(ioctl));
  ioctl.Category   = OS2AHCI_IOCTL_CATEGORY;
  ioctl.Function   = OS2AHCI_IOCTL_PASSTHROUGH;
  ioctl.ParmPacket = (void _far *) &pt;

  memset(&pt, 0x00, sizeof(pt));
  pt.adapter          = a;
  pt.port             = p;
  pt.device           = d;
  pt.cmdlen           = sizeof(pt.cmd.ata);
  pt.cmd.ata.features = sub_func;
  pt.cmd.ata.count    = cnt;
  pt.cmd.ata.lba_l    = (0xc24fL << 8) | lba_l;
  pt.cmd.ata.cmd      = ATA_CMD_SMART;

  if (buf != NULL && sub_func != ATA_SMART_STATUS) {
    pt.buflen         = 512;
    pt.buf            = lin(buf);
  }

  if (((ret = gen_ioctl(&ioctl)) & STERR) == 0 && sub_func == ATA_SMART_STATUS) {

    /* ATA_SMART_STATUS doesn't transfer anything but instead relies on the
     * returned D2H FIS, mapped to the ATA CMD, to have a certain value
     * (0xf42c); the IOCTL result is expected to be returned as a ULONG in
     * the data buffer.
     */
    if (((pt.cmd.ata.lba_l >> 8) & 0xffff) == 0xf42c) {
      *((ULONG _far *) buf) = 1;
    } else {
      *((ULONG _far *) buf) = 0;
    }
  }

  return(ret);
}

/******************************************************************************
 * Map DSKSP unit number to corresponding adapter/port/device number. Units
 * are identified by an 8-bit adapter/device number with the lowest bit
 * selecting between master (0) and slave (1). This number is mapped to our
 * ATA/ATAPI units sequentially.
 */
static int map_unit(BYTE unit, USHORT _far *a, USHORT _far *p, USHORT _far *d)
{
  USHORT _a;
  USHORT _p;
  USHORT _d;

  /* map unit to adapter/port/device */
  for (_a = 0; _a < ad_info_cnt; _a++) {
    AD_INFO *ai = ad_infos + _a;

    for (_p = 0; _p <= ai->port_max; _p++) {
      P_INFO *pi = ai->ports + _p;

      for (_d = 0; _d <= pi->dev_max; _d++) {
        if (pi->devs[_d].present) {
          if (unit-- == 0) {
            /* found the device */
            *a = _a;
            *p = _p;
            *d = _d;
            return(0);
          }
        }
      }
    }
  }

  /* unit not found */
  return(-1);
}

/******************************************************************************
 * Get linear address for specified virtual address.
 */
static LIN lin(void _far *p)
{
  LIN l;

  if (DevHelp_VirtToLin((SEL) ((ULONG) p >> 16), (USHORT) (ULONG) p, &l) != 0) {
    return(0);
  }

  return(l);
}

/******************************************************************************
 * IORB notification routine; used to wake up the sleeping application thread
 * when the IOCTL IORB is complete.
 */
IORBH _far * _far _cdecl ioctl_wakeup(IORBH _far *iorb)
{
  USHORT awake_count;

  DevHelp_ProcRun((ULONG) iorb, &awake_count);

  return(NULL);
}

