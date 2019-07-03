/******************************************************************************
 *
 * Copyright (c) 2013-2018 David Azarewicz
 *
 */

#include "os2ahci.h"

/* -------------------------- macros and constants ------------------------- */

/* ------------------------ typedefs and structures ------------------------ */

/* -------------------------- function prototypes -------------------------- */

/* ------------------------ global/static variables ------------------------ */

/* ----------------------------- start of code ----------------------------- */

/******************************************************************************
 * Create adapter/port/device list for user output.
 */
void build_user_info(void)
{
  int a;
  int p;
  int d;
  int iFlag;

  for (a = 0; a < ad_info_cnt; a++)
  {
    AD_INFO *ai = ad_infos + a;

    dprintf(0,"Adapter %d: PCI=%d:%d:%d ID=%04x:%04x %s %s irq=%d addr=0x%x version=%x\n", a,
      PCI_BUS_FROM_BDF(ai->bus_dev_func), PCI_DEV_FROM_BDF(ai->bus_dev_func),
      PCI_FUNC_FROM_BDF(ai->bus_dev_func),
      ai->pci_vendor, ai->pci_device, vendor_from_id(ai->pci_vendor), ai->pci->chipname,
      ai->irq, ai->mmio_phys,
      ai->bios_config[HOST_VERSION / sizeof(u32)]);

    for (p = 0; p <= ai->port_max; p++)
    {
      P_INFO *pi = &ai->ports[p];
      iFlag = 1;

      for (d = 0; d <= pi->dev_max; d++)
      {
        if (pi->devs[d].present)
        {
          if (iFlag) dprintf(0,"  Port %d:\n", p);
          iFlag = 0;
          dprintf(0,"    Drive %d:", d);
          if (pi->devs[d].atapi) dprintf(0," atapi");
          if (pi->devs[d].removable) dprintf(0," removable");
          if (pi->devs[d].dev_info.Method != NULL)
          {
            dprintf(0," %d cylinders, %d heads, %d sectors per track (%dMB) (%s)",
              pi->devs[d].dev_info.Cylinders, pi->devs[d].dev_info.HeadsPerCylinder, pi->devs[d].dev_info.SectorsPerTrack,
              pi->devs[d].dev_info.TotalSectors/2048, pi->devs[d].dev_info.Method);
          }
          if (pi->devs[d].ignored) dprintf(0," Not a usable disk");
          dprintf(0,"\n");
          dprintf(0,"             Model: %s\n", pi->devs[d].dev_name);
        }
        else if (verbosity > 0)
        {
          if (iFlag) dprintf(0,"  Port %d:\n", p);
          iFlag = 0;
          dprintf(0,"    No drive present\n");
        } /* if */
      } /* for d */
    } /* for p */
  } /* for a */
}

#ifdef DEBUG
void DumpIorb(IORBH *pIorb)
{
  if (D32g_DbgLevel < 2) return;
  if (!ad_infos[iorb_unit_adapter(pIorb)].ports[iorb_unit_port(pIorb)].devs[iorb_unit_device(pIorb)].atapi) return;

  dprintf(0,"IORB %x: Size=%x Len=%x Handle=%x CmdCode=%x\n",
      pIorb, sizeof(IORBH), pIorb->Length, pIorb->UnitHandle, pIorb->CommandCode);
  dprintf(0,"  CmdMod=%x ReqCtrl=%x Status=%x ErrorCode=%x\n",
      pIorb->CommandModifier, pIorb->RequestControl, pIorb->Status, pIorb->ErrorCode);
  dprintf(0,"  Timeout=%x StatusBlkLen=%x pStatusBlk=%x Res=%x pNxtIORB=%x\n",
      pIorb->Timeout, pIorb->StatusBlockLen, pIorb->pStatusBlock, pIorb->Reserved_1,
      pIorb->pNxtIORB);
}
#endif

