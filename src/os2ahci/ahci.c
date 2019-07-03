/******************************************************************************
 * ahci.c - ahci hardware access functions
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

/* -------------------------- macros and constants ------------------------- */

/* produce ata/atapi function pointer with the given func name */
#define cmd_func(iorb, func)  ad_infos[iorb_unit_adapter(iorb)].     \
                                 ports[iorb_unit_port(iorb)].        \
                                  devs[iorb_unit_device(iorb)].atapi \
                              ? atapi_##func : ata_##func


/* ------------------------ global/static variables ------------------------ */

/* Initial driver status flags indexed by the board_* constants in os2ahci.h
 *
 * NOTE: The Linux AHCI driver uses a combination of board-specific quirk
 *       flags and overriding certain libata service functions to handle
 *       adapter flaws. However, there were only three overrides at the time
 *       os2ahci was written, one for hard adapter resets and two for port
 *       resets, and we can easily implement those within the corresponding
 *       reset handlers. If this becomes more complex, this array of flags
 *       should be converted into a structure array which contains function
 *       pointers to all handler functions which may need to be overridden.
 */
u16 initial_flags[] =
{
  0,                                        /* board_ahci */
  AHCI_HFLAG_NO_NCQ |                       /* board_ahci_vt8251 */
    AHCI_HFLAG_NO_PMP,
  AHCI_HFLAG_IGN_IRQ_IF_ERR,                /* board_ahci_ign_iferr */
  AHCI_HFLAG_IGN_SERR_INTERNAL |            /* board_ahci_sb600 */
    AHCI_HFLAG_NO_MSI |
    AHCI_HFLAG_SECT255 |
    AHCI_HFLAG_32BIT_ONLY,
  AHCI_HFLAG_NO_NCQ |                       /* board_ahci_mv */
    AHCI_HFLAG_NO_MSI |
    AHCI_HFLAG_MV_PATA |
    AHCI_HFLAG_NO_PMP,
  AHCI_HFLAG_IGN_SERR_INTERNAL,             /* board_ahci_sb700 */
  AHCI_HFLAG_YES_NCQ,                       /* board_ahci_mcp65 */
  AHCI_HFLAG_NO_PMP,                        /* board_ahci_nopmp */
  AHCI_HFLAG_YES_NCQ,                       /* board_ahci_yesncq */
  AHCI_HFLAG_NO_SNTF,                       /* board_ahci_nosntf */
};

#define MAX_IRQ_HANDLERS 8 /* This is the maximum number of handlers that Dev32Help_SetIRQ can register */
static u16 irq_used[MAX_IRQ_HANDLERS]; /* IRQ level for each used IRQ */
static int irq_used_cnt; /* number of IRQs used */

/* ----------------------------- start of code ----------------------------- */

#ifdef DEBUG
void ahci_dump_host_regs(AD_INFO *ai, int bios_regs)
{
  int i;
  u32 version;

  DPRINTF(2,"AHCI global registers for adapter %d %d:%d:%d irq=%d addr=0x%x\n",
      ad_no(ai),
      PCI_BUS_FROM_BDF(ai->bus_dev_func), PCI_DEV_FROM_BDF(ai->bus_dev_func),
      PCI_FUNC_FROM_BDF(ai->bus_dev_func), ai->irq, ai->mmio_phys);

  for (i = 0; i <= HOST_CAP2; i += sizeof(u32)) {
    u32 val;

    if (bios_regs) val = ai->bios_config[i/sizeof(u32)];
    else
    {
      /* HOST_CAP2 only exists for AHCI V1.2 and later */
      if ((i == HOST_CAP2) && (version < 0x00010200L)) val = 0;
      else val = readl(ai->mmio + i);
    }
    if (i == HOST_VERSION) version = val;

    dprintf(0," %02x: %08lx", i, val);

    if (i == HOST_CAP) {
      dprintf(0," -");
      if (val & HOST_CAP_64)         dprintf(0," 64bit");
      if (val & HOST_CAP_NCQ)        dprintf(0," ncq");
      if (val & HOST_CAP_SNTF)       dprintf(0," sntf");
      if (val & HOST_CAP_MPS)        dprintf(0," mps");
      if (val & HOST_CAP_SSS)        dprintf(0," sss");
      if (val & HOST_CAP_ALPM)       dprintf(0," alpm");
      if (val & HOST_CAP_LED)        dprintf(0," led");
      if (val & HOST_CAP_CLO)        dprintf(0," clo");
      if (val & HOST_CAP_ONLY)       dprintf(0," ahci_only");
      if (val & HOST_CAP_PMP)        dprintf(0," pmp");
      if (val & HOST_CAP_FBS)        dprintf(0," fbs");
      if (val & HOST_CAP_PIO_MULTI)  dprintf(0," pio_multi");
      if (val & HOST_CAP_SSC)        dprintf(0," ssc");
      if (val & HOST_CAP_PART)       dprintf(0," part");
      if (val & HOST_CAP_CCC)        dprintf(0," ccc");
      if (val & HOST_CAP_EMS)        dprintf(0," ems");
      if (val & HOST_CAP_SXS)        dprintf(0," sxs");
      dprintf(0," cmd_slots:%d", ((val >> 8) & 0x1f) + 1);
      dprintf(0," ports:%d",     (val & 0x1f) + 1);
    } else if (i == HOST_CTL) {
      dprintf(0," -");
      if (val & HOST_AHCI_EN)        dprintf(0," ahci_enabled");
      if (val & HOST_IRQ_EN)         dprintf(0," irq_enabled");
      if (val & HOST_RESET)          dprintf(0," resetting");
    } else if (i == HOST_CAP2) {
      dprintf(0," -");
      if (val & HOST_CAP2_BOH)       dprintf(0," boh");
      if (val & HOST_CAP2_NVMHCI)    dprintf(0," nvmhci");
      if (val & HOST_CAP2_APST)      dprintf(0," apst");
    }
    dprintf(0,"\n");
  }
}

void ahci_dump_port_regs(AD_INFO *ai, int p)
{
  u8 *port_mmio = port_base(ai, p);

  dprintf(0,"AHCI port %d registers:\n", p);
  dprintf(0,"  PORT_CMD       = 0x%x\n", readl(port_mmio + PORT_CMD));
  dprintf(0," command engine status:\n");
  dprintf(0,"  PORT_SCR_ACT   = 0x%x\n", readl(port_mmio + PORT_SCR_ACT));
  dprintf(0,"  PORT_CMD_ISSUE = 0x%x\n", readl(port_mmio + PORT_CMD_ISSUE));
  dprintf(0," link/device status:\n");
  dprintf(0,"  PORT_SCR_STAT  = 0x%x\n", readl(port_mmio + PORT_SCR_STAT));
  dprintf(0,"  PORT_SCR_CTL   = 0x%x\n", readl(port_mmio + PORT_SCR_CTL));
  dprintf(0,"  PORT_SCR_ERR   = 0x%x\n", readl(port_mmio + PORT_SCR_ERR));
  dprintf(0,"  PORT_TFDATA    = 0x%x\n", readl(port_mmio + PORT_TFDATA));
  dprintf(0," interrupt status:\n");
  dprintf(0,"  PORT_IRQ_STAT  = 0x%x\n", readl(port_mmio + PORT_IRQ_STAT));
  dprintf(0,"  PORT_IRQ_MASK  = 0x%x\n", readl(port_mmio + PORT_IRQ_MASK));
  dprintf(0,"  HOST_IRQ_STAT  = 0x%x\n", readl(ai->mmio + HOST_IRQ_STAT));
}
#endif

/******************************************************************************
 * setup the CAPS and other adapter information for this adapter.
 *
 * This function saves working copies of the CAP and CAP2 registers
 * as well as the initial port map in the AD_INFO structure after
 * removing features which are known to cause trouble on this specific
 * piece of hardware.
 */
int ahci_config_caps(AD_INFO *ai)
{
  int ports;
  int i;

  ai->cap = readl(ai->mmio + HOST_CAP);
  ai->port_map = readl(ai->mmio + HOST_PORTS_IMPL);

  /* HOST_CAP2 only exists for AHCI V1.2 and later */
  if (readl(ai->mmio + HOST_VERSION) >= 0x00010200L) ai->cap2 = readl(ai->mmio + HOST_CAP2);

  if (ai->pci->board >= sizeof(initial_flags) / sizeof(*initial_flags))
  {
    dprintf(0,"error: invalid board index in PCI info\n");
    return(-1);
  }
  ai->flags = initial_flags[ai->pci->board];
  ai->hw_ports = (ai->cap & 0x1f) + 1;

  if ((ai->cap & HOST_CAP_64) && (ai->flags & AHCI_HFLAG_32BIT_ONLY))
  {
    /* disable 64-bit support for faulty controllers; OS/2 can't do 64 bits at
     * this point, of course, but who knows where all this will be in a few
     * years...
     */
    ai->cap &= ~HOST_CAP_64;
  }

  /* Remove broken feature bits. This is largely copied from the Linux AHCI driver -- the wisdom
   * around quirks and faulty hardware is hard to come by...
   */
  if ((ai->cap & HOST_CAP_NCQ) && (ai->flags & AHCI_HFLAG_NO_NCQ))
  {
    DPRINTF(1,"controller can't do NCQ, turning off CAP_NCQ\n");
    ai->cap &= ~HOST_CAP_NCQ;
  }

  if (!(ai->cap & HOST_CAP_NCQ) && (ai->flags & AHCI_HFLAG_YES_NCQ))
  {
    DPRINTF(1,"controller can do NCQ, turning on CAP_NCQ\n");
    ai->cap |= HOST_CAP_NCQ;
  }

  if ((ai->cap & HOST_CAP_PMP) && (ai->flags & AHCI_HFLAG_NO_PMP))
  {
    DPRINTF(1,"controller can't do PMP, turning off CAP_PMP\n");
    ai->cap |= HOST_CAP_PMP;
  }

  if ((ai->cap & HOST_CAP_SNTF) && (ai->flags & AHCI_HFLAG_NO_SNTF))
  {
    DPRINTF(1,"controller can't do SNTF, turning off CAP_SNTF\n");
    ai->cap &= ~HOST_CAP_SNTF;
  }

  if (ai->pci_vendor == PCI_VENDOR_ID_JMICRON && ai->pci_device == 0x2361 && ai->port_map != 1)
  {
    DPRINTF(1,"JMB361 has only one port, port_map 0x%x -> 0x%x\n", ai->port_map, 1);
    ai->port_map = 1;
    ai->hw_ports = 1;
  }

  /* Correlate port map to number of ports reported in HOST_CAP
   *
   * NOTE: Port map and number of ports handling differs a bit from the
   *       Linux AHCI driver because we're storing both in AI_INFO. As in the
   *       Linux driver, the port map is the main driver for port scanning but
   *       we're also saving a maximum port number in AI_INFO to reduce the
   *       number of IORB queues to look at in trigger_engine(). This is done
   *       in ahci_scan_ports().
   */
  ports = ai->hw_ports;
  for (i = 0; i < AHCI_MAX_PORTS; i++)
  {
    if (ai->port_map & (1UL << i)) ports--;
  }
  if (ports < 0)
  {
    /* more ports in port_map than in HOST_CAP & 0x1f */
    ports = ai->hw_ports;
    DPRINTF(0,"implemented port map (0x%x) contains more ports than hw_ports (%d), using hw_ports\n", ai->port_map, ports);
    ai->port_map = (1UL << ports) - 1UL;
  }

  /* set maximum command slot number */
  ai->cmd_max = ((ai->cap >> 8) & 0x1f);

  return(0);
}

/******************************************************************************
 * Save BIOS configuration of AHCI adapter. As a side effect, this also saves
 * generic configuration information which we may have to restore after an
 * adapter reset.
 */
int ahci_save_bios_config(AD_INFO *ai)
{
  int i;

  /* save BIOS configuration */
  for (i = 0; i < HOST_CAP2; i += sizeof(u32))
  {
    ai->bios_config[i / sizeof(u32)] = readl(ai->mmio + i);
  }

  DPRINTF(3,"ahci_save_bios_config: BIOS AHCI mode is %d\n", ai->bios_config[HOST_CTL / sizeof(u32)] & HOST_AHCI_EN);

  if ((ai->bios_config[HOST_CTL / sizeof(u32)] & HOST_AHCI_EN) == 0 && ai->pci_vendor == PCI_VENDOR_ID_INTEL)
  {
    /* Adapter is not in AHCI mode and the spec says a COMRESET is
     * required when switching from SATA to AHCI mode and vice versa.
     */
    init_reset = 1;
  }

  DUMP_HOST_REGS(2,ai,1);

  return(0);
}

/******************************************************************************
 * Restore BIOS configuration of AHCI adapter. This is needed after scanning
 * for devices because we still need the BIOS until the initial boot sequence
 * has completed.
 */
int ahci_restore_bios_config(AD_INFO *ai)
{
  DPRINTF(3,"ahci_restore_bios_config: restoring AHCI BIOS configuration on adapter %d\n", ad_no(ai));

  /* Restore saved BIOS configuration; please note that HOST_CTL is restored
   * last because it may cause AHCI mode to be turned off again.
   */
  writel(ai->mmio + HOST_CCC,       ai->bios_config[HOST_CCC / sizeof(u32)]);
  writel(ai->mmio + HOST_CCC_PORTS, ai->bios_config[HOST_CCC_PORTS / sizeof(u32)]);
  writel(ai->mmio + HOST_EM_CTL,    ai->bios_config[HOST_EM_CTL / sizeof(u32)]);
  writel(ai->mmio + HOST_CTL,       ai->bios_config[HOST_CTL / sizeof(u32)]);

  /* flush PCI MMIO delayed write buffers */
  readl(ai->mmio + HOST_CTL);

  if ((ai->bios_config[HOST_CTL / sizeof(u32)] & HOST_AHCI_EN) == 0 && ai->pci_vendor == PCI_VENDOR_ID_INTEL)
  {
    /* This BIOS apparently accesses the controller via SATA registers and
     * the AHCI spec says that we should issue a COMRESET on each port after
     * disabling AHCI mode to allow the SATA controller to re-recognize attached
     * devices. How to do this depends on the controller, of course, but so
     * far I've only seen Dell notebook BIOSs with Intel chipsets to behave
     * like this; all other BIOS implementations I've seen so far seem to take
     * AHCI mode literally and operate the controller in AHCI mode from the
     * beginning.
     *
     * We'll use a feature on Intel ICH7/8 controllers which provides MMIO
     * mappings for the AHCI SCR registers even when not in AHCI mode.
     */
    int p;

    for (p = 0; p <= ai->port_max; p++)
    {
      if (ai->port_map & (1UL << p))
      {
        u8 *port_mmio = port_base(ai, p);
        u32 tmp;

        tmp = readl(port_mmio + PORT_SCR_CTL) & ~0x0000000fUL;
        writel(port_mmio + PORT_SCR_CTL, tmp | 1);
        readl(port_mmio + PORT_SCR_CTL);  /* flush */

        /* spec says "leave reset bit on for at least 1ms"; make it 2ms */
        udelay(2000);

        writel(port_mmio + PORT_SCR_CTL, tmp);
        readl(port_mmio + PORT_SCR_CTL);  /* flush */
      }
    }
  }

  return(0);
}

/******************************************************************************
 * Restore initial configuration (e.g. after an adapter reset). This relies
 * on information saved by 'ahci_save_bios_config()'.
 */
int ahci_restore_initial_config(AD_INFO *ai)
{
  DPRINTF(3,"ahci_restore_initial_config: restoring initial configuration on adapter %d\n", ad_no(ai));

  /* restore saved BIOS configuration */
  //writel(ai->mmio + HOST_CCC,       ai->bios_config[HOST_CCC / sizeof(u32)]);
  //writel(ai->mmio + HOST_CCC_PORTS, ai->bios_config[HOST_CCC_PORTS / sizeof(u32)]);
  //writel(ai->mmio + HOST_EM_CTL,    ai->bios_config[HOST_EM_CTL / sizeof(u32)]);
  //writel(ai->mmio + HOST_CTL,       ai->bios_config[HOST_CTL / sizeof(u32)]);

  writel(ai->mmio + HOST_CAP, ai->bios_config[HOST_CAP / sizeof(u32)]);
  if (ai->bios_config[HOST_CAP2 / sizeof(u32)])
    writel(ai->mmio + HOST_CAP2, ai->bios_config[HOST_CAP2 / sizeof(u32)]);
  writel(ai->mmio + HOST_PORTS_IMPL, ai->bios_config[HOST_PORTS_IMPL / sizeof(u32)]);

  /* flush PCI MMIO delayed write buffers */
  readl(ai->mmio + HOST_PORTS_IMPL);

  return(0);
}

#ifdef NOT_USED
int ahci_reset_controller(AD_INFO *ai)
{
    u32 tmp;
    TIMER Timer;

    DPRINTF(2,"controller reset starting on adapter %d\n", ad_no(ai));

    /* we must be in AHCI mode, before using anything AHCI-specific, such as HOST_RESET. */
    ahci_enable_ahci(ai);

    /* global controller reset */
    tmp = readl(ai->mmio + HOST_CTL);
    if ((tmp & HOST_RESET) == 0) {
        writel(ai->mmio + HOST_CTL, tmp | HOST_RESET);
        readl(ai->mmio + HOST_CTL); /* flush */
    }

    /*
     * to perform host reset, OS should set HOST_RESET
     * and poll until this bit is read to be "0".
     * reset must complete within 1 second, or
     * the hardware should be considered fried.
     */
    TimerInit(&Timer, 1000);
    while (((tmp = readl(ai->mmio + HOST_CTL)) & HOST_RESET) != 0) {
      if (TimerCheckAndBlock(&Timer)) {
        dprintf(0,"controller reset failed (0x%x)\n", tmp);
        return(-1);
      }
    }

    /* turn on AHCI mode */
    ahci_enable_ahci(ai);

    /* Some registers might be cleared on reset.  Restore initial values. */
    ahci_restore_initial_config(ai);

    if (ai->pci_vendor == PCI_VENDOR_ID_INTEL) {
      u32 tmp16 = 0;

      DPRINTF(1,"ahci_reset_controller: intel detected\n");
      /* configure PCS */
      PciReadConfig(ai->bus, ai->dev_func, 0x92, sizeof(u16), &tmp16);
      if ((tmp16 & ai->port_map) != ai->port_map) {
        DPRINTF(3,"ahci_reset_controller: updating PCS %x/%x\n", tmp16, ai->port_map);
        tmp16 |= ai->port_map;
        PciWriteConfig(ai->bus, ai->dev_func, 0x92, sizeof(u16), tmp16);
      }
    }

    return 0;
}
#endif

/******************************************************************************
 * Save port configuration. This is primarily used to save the BIOS port
 * configuration (command list and FIS buffers and the IRQ mask).
 *
 * The port configuration returned by this function is dynamically allocated
 * and automatically freed when calling ahci_restore_port_config().
 */
AHCI_PORT_CFG *ahci_save_port_config(AD_INFO *ai, int p)
{
  AHCI_PORT_CFG *pc;
  u8 *port_mmio = port_base(ai, p);

  if ((pc = MemAlloc(sizeof(*pc))) == NULL) return(NULL);

  pc->cmd_list   = readl(port_mmio + PORT_LST_ADDR);
  pc->cmd_list_h = readl(port_mmio + PORT_LST_ADDR_HI);
  pc->fis_rx     = readl(port_mmio + PORT_FIS_ADDR);
  pc->fis_rx_h   = readl(port_mmio + PORT_FIS_ADDR_HI);
  pc->irq_mask   = readl(port_mmio + PORT_IRQ_MASK);
  pc->port_cmd   = readl(port_mmio + PORT_CMD);

  return(pc);
}

/******************************************************************************
 * Restore port configuration. This is primarily used to restore the BIOS port
 * configuration (command list and FIS buffers and the IRQ mask).
 *
 * The port configuration is automatically freed.
 */
void ahci_restore_port_config(AD_INFO *ai, int p, AHCI_PORT_CFG *pc)
{
  u8 *port_mmio = port_base(ai, p);

  /* stop the port, first */
  ahci_stop_port(ai, p);

  if (ai->bios_config[HOST_CTL / sizeof(u32)] & HOST_AHCI_EN)
  {
    /* BIOS uses AHCI, too, so we need to restore the port settings;
     * restoring PORT_CMD may well start the port again but that's what
     * this function is all about.
     */
    writel(port_mmio + PORT_LST_ADDR,    pc->cmd_list);
    writel(port_mmio + PORT_LST_ADDR_HI, pc->cmd_list_h);
    writel(port_mmio + PORT_FIS_ADDR,    pc->fis_rx);
    writel(port_mmio + PORT_FIS_ADDR_HI, pc->fis_rx_h);
    writel(port_mmio + PORT_IRQ_MASK,    pc->irq_mask);
    writel(port_mmio + PORT_CMD,         pc->port_cmd);

    readl(port_base(ai, p) + PORT_IRQ_MASK); /* flush */
  }

  MemFree(pc);
}

/******************************************************************************
 * Enable AHCI mode on this controller.
 */
int ahci_enable_ahci(AD_INFO *ai)
{
  u32 ctl = readl(ai->mmio + HOST_CTL);
  int i;

  if (ctl & HOST_AHCI_EN)
  {
    /* AHCI mode already enabled */
    return(0);
  }

  if (ai->pci_vendor == PCI_VENDOR_ID_INTEL)
  {
    /* Adapter is not in AHCI mode and the spec says a COMRESET is
     * required when switching from SATA to AHCI mode and vice versa.
     */
    init_reset = 1;
  }

  /* some controllers need AHCI_EN to be written multiple times */
  for (i = 0; i < 5; i++)
  {
    ctl |= HOST_AHCI_EN;
    writel(ai->mmio + HOST_CTL, ctl);
    ctl = readl(ai->mmio + HOST_CTL);   /* flush && sanity check */
    if (ctl & HOST_AHCI_EN)
    {
      return(0);
    }
    msleep(10);
  }

  /* couldn't enable AHCI mode */
  dprintf(0,"failed to enable AHCI mode on adapter %d\n", ad_no(ai));
  return(1);
}

/******************************************************************************
 * Complete initialization of adapter. This includes restarting all active
 * ports and initializing interrupt processing. This is called when receiving
 * the IOCM_COMPLETE_INIT request.
 */
int ahci_complete_init(AD_INFO *ai)
{
  int rc;
  u32 p;
  int i;

  DPRINTF(1,"ahci_complete_init: completing initialization of adapter #%d\n", ad_no(ai));

  if (!ai->int_set)
  {
    /* register IRQ handler; each IRQ level is registered only once */
    p = 1; /* int count */
    if (!(ai->flags & AHCI_HFLAG_NO_MSI))
    {
      if (PsdMsiAlloc(ai->bus_dev_func, &p, &ai->irq)) p = 1; /* shared flag */
      else
      {
        /* we have an msi interrupt */
        ai->irq_pin = 0;
        p = 0; /* exclusive flag */
      }
    }
    for (i = 0; i < irq_used_cnt; i++)
    {
      if (irq_used[i] == ai->irq) break; /* we already have this IRQ registered */
    }
    if (i >= irq_used_cnt)
    {
      if (i >= MAX_IRQ_HANDLERS) return -1; /* no more handlers available */

      DPRINTF(2,"registering interrupt %d pin=%d\n", ai->irq, ai->irq_pin);

      rc = Dev32Help_SetIRQ(ahci_intr, ai->irq, p, ai->irq);
      if (rc && p) /* if failed and was shared */
      {
        p = 0; /* try exclusive */
        rc = Dev32Help_SetIRQ(ahci_intr, ai->irq, p, ai->irq);
      }
      if (rc)
      {
        dprintf(0,"failed to register interrupt %d\n", ai->irq);
        return(-1);
      }
      irq_used[irq_used_cnt++] = ai->irq;
      ai->int_set = 1;
      RmUpdateAddIrq(rm_drvh, ai->rm_adh, ai->irq, ai->irq_pin, p?RS_IRQ_SHARED:RS_IRQ_EXCLUSIVE);
    }
  }

  /* enable AHCI mode */
  if ((rc = ahci_enable_ahci(ai)) != 0) return(rc);

  /* Start all ports. The main purpose is to set the command list and FIS
   * receive area addresses properly and to enable port-level interrupts; we
   * don't really care about the return status because we'll find out soon
   * enough if a previously detected device has problems.
   */
  for (p = 0; p <= ai->port_max; p++)
  {
    if (ai->port_map & (1UL << p))
    {
      if (init_reset)
      {
        DPRINTF(3,"ahci_complete_init: resetting port %d\n", p);
        ahci_reset_port(ai, p, 1);
      }
      else
      {
        DPRINTF(3,"ahci_complete_init: restarting port #%d\n", p);
        ahci_stop_port(ai, p);
        ahci_start_port(ai, p, 1);
      }
    }
  }

  /* clear pending interrupt status */
  writel(ai->mmio + HOST_IRQ_STAT, readl(ai->mmio + HOST_IRQ_STAT));
  readl(ai->mmio + HOST_IRQ_STAT); /* flush */

  /* enable adapter-level interrupts */
  writel(ai->mmio + HOST_CTL, readl(ai->mmio + HOST_CTL) | HOST_IRQ_EN);
  readl(ai->mmio + HOST_CTL); /* flush */

  /* enable interrupts on PCI-level (PCI 2.3 added a feature to disable INTs) */
  /* pci_enable_int(ai->bus, ai->dev_func); */

  DPRINTF(1,"ahci_complete_init: done\n");
  return(0);
}

static int IsUsableDisk(AD_INFO *ai, int p, int d)
{
  union {
    u8 b[512];
    u16 w[256];
    u32 l[128];
  } *pSector0;
  int iRetVal;

  if (!use_mbr_test) return 1;
  if (ai->ports[p].devs[d].removable) return 1;

  do
  {
    iRetVal = 0;
    pSector0 = MemAlloc(512);
    if (!pSector0) break;

    if (ahci_exec_polled_cmd(ai, p, 0, 500, ATA_CMD_READ,
         AP_SECTOR_28, 0,
         AP_COUNT, 1,
         AP_VADDR, (void *)pSector0, 512,
         AP_DEVICE, 0x40,
         AP_END)) break;

    DHEXDUMP(5, pSector0, 512, "Sector0:\n");

    /* check for wiped disk */
    if ((pSector0->l[0] == 0) && (pSector0->l[127] == 0))
    {
      iRetVal = 1;
      break;
    }

    /* check for a valid MBR */
    if (pSector0->w[255] != 0xaa55) break; /* Not valid if no MBR signature */
    if (pSector0->b[0x1c2] == 0xee) break; /* Not valid if guard partition */
    if (pSector0->b[0x1d2] == 0xee) break; /* Not valid if guard partition */
    if (pSector0->b[0x1e2] == 0xee) break; /* Not valid if guard partition */
    if (pSector0->b[0x1f2] == 0xee) break; /* Not valid if guard partition */
    iRetVal = 1;
  } while (0);

  if (pSector0) MemFree(pSector0);
  return iRetVal;
}

/******************************************************************************
 * Set up device attached to the specified port based on ATA_IDENTFY_DEVICE or
 * ATA_IDENTFY_PACKET_DEVICE data.
 *
 * NOTE: Port multipliers are not supported, yet, thus the device number is
 *       expected to be 0 for the time being.
 */
static void ahci_setup_device(AD_INFO *ai, int p, int d, u16 *id_buf)
{
  DEVICESTRUCT ds;
  ADJUNCT adj;
  HDEVICE dh;
  char dev_name[RM_MAX_PREFIX_LEN+ATA_ID_PROD_LEN+1];
  char *pDevName;
  static u8 total_dev_cnt;

  if (p >= AHCI_MAX_PORTS) return;
  if (d >= AHCI_MAX_DEVS) return;

  if (ai->port_max < p) ai->port_max = p;
  if (ai->ports[p].dev_max < d) ai->ports[p].dev_max = d;
  memset(ai->ports[p].devs + d, 0x00, sizeof(*ai->ports[p].devs));

  /* set generic device information (assuming an ATA disk device for now) */
  ai->ports[p].devs[d].present   = 1;
  ai->ports[p].devs[d].removable = (id_buf[ATA_ID_CONFIG] & 0x0080U) != 0;
  ai->ports[p].devs[d].dev_type  = UIB_TYPE_DISK;
  pDevName = ai->ports[p].devs[d].dev_name;
  strlcpy(pDevName, ata_dev_name(id_buf), sizeof(ai->ports[0].devs[0].dev_name));

  if (id_buf[ATA_ID_CONFIG] & 0x8000U)
  {
    /* this is an ATAPI device; augment device information */
    ai->ports[p].devs[d].atapi     = 1;
    ai->ports[p].devs[d].atapi_16  = (id_buf[ATA_ID_CONFIG] & 0x0001U) != 0;
    ai->ports[p].devs[d].dev_type  = (id_buf[ATA_ID_CONFIG] & 0x1f00U) >> 8;
    ai->ports[p].devs[d].ncq_max   = 1;

  }
  else
  {
    /* complete ATA-specific device information */
    if (enable_ncq[ad_no(ai)][p])
    {
      ai->ports[p].devs[d].ncq_max = id_buf[ATA_ID_QUEUE_DEPTH] & 0x001fU;
    }
    if (ai->ports[p].devs[d].ncq_max < 1)
    {
      /* NCQ not enabled for this device, or device doesn't support NCQ */
      ai->ports[p].devs[d].ncq_max = 1;
    }
    if (id_buf[ATA_ID_CFS_ENABLE_2] & 0x0400U)
    {
      ai->ports[p].devs[d].lba48   = 1;
    }
  }

  DPRINTF(2,"found device %d.%d.%d: removable=%d dev_type=%d atapi=%d ncq_max=%d\n",
          ad_no(ai), p, d,
          ai->ports[p].devs[d].removable,
          ai->ports[p].devs[d].dev_type,
          ai->ports[p].devs[d].atapi,
          ai->ports[p].devs[d].ncq_max);

  /* add device to resource manager; we don't really care about errors here */
  memset(&ds, 0x00, sizeof(ds));
  memset(&adj, 0x00, sizeof(adj));

  adj.pNextAdj            = NULL;
  adj.AdjLength           = sizeof(adj);
  adj.AdjType             = ADJ_ADD_UNIT;
  adj.Add_Unit.ADDHandle  = rm_drvh;
  adj.Add_Unit.UnitHandle = (USHORT)total_dev_cnt;

  /* create Resource Manager device key string;
   * we distinguish only HDs and CD drives for now
   */
  if (ai->ports[p].devs[d].removable)
  {
    snprintf(dev_name, sizeof(dev_name), RM_CD_PREFIX "%s", p, d, pDevName);
  }
  else
  {
    snprintf(dev_name, sizeof(dev_name), RM_HD_PREFIX "%s", p, d, pDevName);
  }

  ds.DevDescriptName = dev_name;
  ds.DevFlags        = (ai->ports[p].devs[d].removable) ? DS_REMOVEABLE_MEDIA
                                                        : DS_FIXED_LOGICALNAME;
  ds.DevType         = ai->ports[p].devs[d].dev_type;
  ds.pAdjunctList    = &adj;

  RMCreateDevice(rm_drvh, &dh, &ds, ai->rm_adh, NULL);

  total_dev_cnt++;

  /* try to detect virtualbox environment to enable a hack for IRQ routing */
  if (ai == ad_infos && ai->pci_vendor == 0x8086 && ai->pci_device == 0x2829 &&
      !memcmp(pDevName, "VBOX HARDDISK", 13))
  {
    /* running inside virtualbox */
    pci_hack_virtualbox();
  }
}

/******************************************************************************
 * Scan all ports for connected devices and fill in the corresponding device
 * information.
 *
 * NOTES:
 *
 *  - The adapter is temporarily configured for os2ahci but the original BIOS
 *    configuration will be restored when done. This happens only until we
 *    have received the IOCC_COMPLETE_INIT command.
 *
 *  - Subsequent calls are currently not planned but may be required for
 *    suspend/resume handling, hot swap functionality, etc.
 *
 *  - This function is expected to be called with the spinlock released but
 *    the corresponding adapter's busy flag set. It will aquire the spinlock
 *    temporarily to allocate/free memory for the ATA identify buffer.
 *
 * Called from iocm_device_table()
 */
int ahci_scan_ports(AD_INFO *ai)
{
  AHCI_PORT_CFG *pc = NULL;
  u16 *id_buf;
  int is_ata;
  int rc;
  int p;
  int i;
  TIMER Timer;

  if ((id_buf = MemAlloc(ATA_ID_WORDS * sizeof(u16))) == NULL) return(-1);

  if (ai->bios_config[0] == 0) ahci_save_bios_config(ai); /* first call */

  if (ahci_enable_ahci(ai)) goto exit_port_scan;

  /* perform port scan */
  DPRINTF(1,__func__": scanning ports on adapter %d\n", ad_no(ai));
  for (p = 0; p < AHCI_MAX_PORTS; p++)
  {
    if (!(ai->port_map & (1UL << p))) continue;
    if (port_ignore[ad_no(ai)][p]) continue;

    // DAZ allocate port structure here

    DPRINTF(3,__func__": Wait till not busy on port %d\n", p);
    /* wait until all active commands have completed on this port */
    TimerInit(&Timer, 250);
    while (ahci_port_busy(ai, p))
    {
      if (TimerCheckAndBlock(&Timer)) break;
    }

    if (!init_complete)
    {
      if ((pc = ahci_save_port_config(ai, p)) == NULL) goto exit_port_scan;
    }

    /* start/reset port; if no device is attached, this is expected to fail */
    if (init_reset)
    {
      rc = ahci_reset_port(ai, p, 0);
    }
    else
    {
      DPRINTF(3,__func__": (re)starting port %d\n", p);
      ahci_stop_port(ai, p);
      rc = ahci_start_port(ai, p, 0);
    }

    if (rc == 0)
    {
      /* this port seems to have a device attached and ready for commands */
      DPRINTF(1,__func__": port %d seems to be attached to a device; probing...\n", p);

      #ifdef DAZ_NEW_CODE
      ai->ports[p].dma_buf = MemAllocAlign(AHCI_PORT_PRIV_DMA_SZ, 1024);
      ai->ports[p].dma_buf_phys = MemPhysAdr(ai->ports[p].dma_buf);
      #endif

      /* Get ATA(PI) identity. The so-called signature gives us a hint whether
       * this is an ATA or an ATAPI device but we'll try both in either case;
       * the signature will merely determine whether we're going to probe for
       * an ATA or ATAPI device, first, in order to reduce the chance of sending
       * the wrong command (which would result in a port reset given the way
       * ahci_exec_polled_cmd() was implemented).
       */
      is_ata = readl(port_base(ai, p) + PORT_SIG) == 0x00000101UL;
      for (i = 0; i < 2; i++)
      {
        rc = ahci_exec_polled_cmd(ai, p, 0, 500,
                                  (is_ata) ? ATA_CMD_ID_ATA : ATA_CMD_ID_ATAPI,
                                  AP_VADDR, (void *) id_buf, ATA_ID_WORDS * sizeof(u16),
                                  AP_END);
        if (rc == 0) break;

        /* try again with ATA/ATAPI swapped */
        is_ata = !is_ata;
      }
    }

    if (rc == 0)
    {
      /* we have a valid IDENTIFY or IDENTIFY_PACKET response */
      DHEXDUMP(5,id_buf, ATA_ID_WORDS * sizeof(u16), "ATA_IDENTIFY%s results:\n", (is_ata) ? "" : "_PACKET");
      ahci_setup_device(ai, p, 0, id_buf);
      if (!IsUsableDisk(ai, p, 0)) ai->ports[p].devs[0].ignored = 1;
    }
    else
    {
      /* no device attached to this port */
      ai->port_map &= ~(1UL << p);
      #ifdef DAZ_NEW_CODE
      if (ai->ports[p].dma_buf) MemFree(ai->ports[p].dma_buf);
      ai->ports[p].dma_buf = NULL;
      #endif
    }

    if (pc != NULL) ahci_restore_port_config(ai, p, pc);
  }

exit_port_scan:
  if (!init_complete)
  {
    ahci_restore_bios_config(ai);
  }
  MemFree(id_buf);
  return(0);
}

/******************************************************************************
 * Reset specified port. This function is typically called during adapter
 * initialization and first gets the port into a defined status, then resets
 * the port by sending a COMRESET signal.
 *
 * This function is also the location of the link speed initialization (link
 * needs to be restablished after changing link speed, anyway).
 *
 * NOTE: This function uses a busy loop to wait for DMA engines to stop and
 *       the COMRESET to complete. It should only be called at task time
 *       during initialization or in a context hook.
 */
int ahci_reset_port(AD_INFO *ai, int p, int ei)
{
  u8 *port_mmio = port_base(ai, p);
  u32 tmp;
  TIMER Timer;

  DPRINTF(2,"ahci_reset_port: resetting port %d.%d\n", ad_no(ai), p);
  DUMP_PORT_REGS(2,ai,p);

  /* stop port engines (we don't care whether there is an error doing so) */
  ahci_stop_port(ai, p);

  /* clear SError */
  tmp = readl(port_mmio + PORT_SCR_ERR);
  writel(port_mmio + PORT_SCR_ERR, tmp);

  /* Some hardware reports incorrect status so just set these bits unconditionally */
  tmp = readl(port_mmio + PORT_CMD);
  tmp &= ~PORT_CMD_ALPE; /* turn off agressive power management */
  tmp |= (PORT_CMD_SPIN_UP | PORT_CMD_POWER_ON); /* power up and spin up the drive */
  writel(port_mmio + PORT_CMD, tmp);

  /* set link speed and power management options */
  DPRINTF(3,"ahci_reset_port: setting link speed and power management options\n");
  tmp = readl(port_mmio + PORT_SCR_CTL) & ~0x00000fffUL;
  tmp |= (link_speed[ad_no(ai)][p] & 0x0f) << 4;
  tmp |= (link_power[ad_no(ai)][p] & 0x0f) << 8;
  writel(port_mmio + PORT_SCR_CTL, tmp);

  /* issue COMRESET on the port */
  DPRINTF(3,"ahci_reset_port: issuing COMRESET on port %d\n", p);
  writel(port_mmio + PORT_SCR_CTL, tmp | 1);
  readl(port_mmio + PORT_SCR_CTL);  /* flush */

  /* spec says "leave reset bit on for at least 1ms"; make it 2ms */
  udelay(2000);

  writel(port_mmio + PORT_SCR_CTL, tmp);
  readl(port_mmio + PORT_SCR_CTL);  /* flush */

  /* wait for communication to be re-established after port reset */
  DPRINTF(2,"Wait for communication...\n");
  TimerInit(&Timer, 500);
  while (((tmp = readl(port_mmio + PORT_SCR_STAT)) & 3) != 3)
  {
    if (TimerCheckAndBlock(&Timer))
    {
      DPRINTF(0,"no device present after resetting port #%d (PORT_SCR_STAT = 0x%x)\n", p, tmp);
      return(-1);
    }
  }

  /* clear SError again (recommended by AHCI spec) */
  tmp = readl(port_mmio + PORT_SCR_ERR);
  writel(port_mmio + PORT_SCR_ERR, tmp);

  /* start port so we can receive the COMRESET FIS */
  DPRINTF(2,"ahci_reset_port: starting port %d again\n", p);
  ahci_start_port(ai, p, ei);

  /* wait for device to be ready ((PxTFD & (BSY | DRQ | ERR)) == 0) */
  TimerInit(&Timer, 1000);
  while (((tmp = readl(port_mmio + PORT_TFDATA)) & 0x89) != 0)
  {
    if (TimerCheckAndBlock(&Timer))
    {
      DPRINTF(0,"device not ready on port #%d (PORT_TFDATA = 0x%x)\n", p, tmp);
      ahci_stop_port(ai, p);
      return(-1);
    }
  }
  DPRINTF(3,"ahci_reset_port: PORT_TFDATA = 0x%x\n", readl(port_mmio + PORT_TFDATA));

  return(0);
}

/******************************************************************************
 * Start specified port.
 */
int ahci_start_port(AD_INFO *ai, int p, int ei)
{
  u8 *port_mmio = port_base(ai, p);
  u32 status;

  DPRINTF(3,"ahci_start_port %d.%d\n", ad_no(ai), p);
  /* check whether device presence is detected and link established */

  status = readl(port_mmio + PORT_SCR_STAT);
  DPRINTF(3,"ahci_start_port: PORT_SCR_STAT = 0x%x\n", status);
  if ((status & 0xf) != 3) return(-1);

  /* clear SError, if any */
  status = readl(port_mmio + PORT_SCR_ERR);
  DPRINTF(3,"ahci_start_port: PORT_SCR_ERR  = 0x%x\n", status);
  writel(port_mmio + PORT_SCR_ERR, status);

  /* enable FIS reception */
  ahci_start_fis_rx(ai, p);

  /* enable command engine */
  ahci_start_engine(ai, p);

  if (ei)
  {
    /* clear any pending interrupts on this port */
    if ((status = readl(port_mmio + PORT_IRQ_STAT)) != 0)
    {
      writel(port_mmio + PORT_IRQ_STAT, status);
    }

    /* enable port interrupts */
    writel(port_mmio + PORT_IRQ_MASK, PORT_IRQ_TF_ERR |
                                      PORT_IRQ_HBUS_ERR |
                                      PORT_IRQ_HBUS_DATA_ERR |
                                      PORT_IRQ_IF_ERR |
                                      PORT_IRQ_OVERFLOW |
                                      PORT_IRQ_BAD_PMP |
                                      PORT_IRQ_UNK_FIS |
                                      PORT_IRQ_SDB_FIS |
                                      PORT_IRQ_DMAS_FIS |
                                      PORT_IRQ_PIOS_FIS |
                                      PORT_IRQ_D2H_REG_FIS);
  }
  else
  {
    writel(port_mmio + PORT_IRQ_MASK, 0);
  }
  readl(port_mmio + PORT_IRQ_MASK);  /* flush */

  return(0);
}

/******************************************************************************
 * Start port FIS reception. Copied from Linux AHCI driver and adopted to
 * OS2AHCI.
 */
void ahci_start_fis_rx(AD_INFO *ai, int p)
{
  u8 *port_mmio = port_base(ai, p);
  u32 port_dma = port_dma_base_phys(ai, p);
  u32 tmp;

  /* set command header and FIS address registers */
  writel(port_mmio + PORT_LST_ADDR, port_dma + offsetof(AHCI_PORT_DMA, cmd_hdr));
  writel(port_mmio + PORT_LST_ADDR_HI, 0);
  writel(port_mmio + PORT_FIS_ADDR, port_dma + offsetof(AHCI_PORT_DMA, rx_fis));
  writel(port_mmio + PORT_FIS_ADDR_HI, 0);

  /* enable FIS reception */
  tmp = readl(port_mmio + PORT_CMD);
  tmp |= PORT_CMD_FIS_RX;
  writel(port_mmio + PORT_CMD, tmp);

  /* flush */
  readl(port_mmio + PORT_CMD);
}

/******************************************************************************
 * Start port HW engine. Copied from Linux AHCI driver and adopted to OS2AHCI.
 */
void ahci_start_engine(AD_INFO *ai, int p)
{
  u8 *port_mmio = port_base(ai, p);
  u32 tmp;

  /* start DMA */
  tmp = readl(port_mmio + PORT_CMD);
  tmp |= PORT_CMD_START;
  writel(port_mmio + PORT_CMD, tmp);
  readl(port_mmio + PORT_CMD); /* flush */
}

/******************************************************************************
 * Stop specified port
 */
int ahci_stop_port(AD_INFO *ai, int p)
{
  u8 *port_mmio = port_base(ai, p);
  u32 tmp;
  int rc;

  DPRINTF(3,"ahci_stop_port %d.%d\n", ad_no(ai), p);

  /* disable port interrupts */
  writel(port_mmio + PORT_IRQ_MASK, 0);

  /* disable FIS reception */
  if ((rc = ahci_stop_fis_rx(ai, p)) != 0)
  {
    dprintf(0,"error: failed to stop FIS receive (%d)\n", rc);
    return(rc);
  }

  /* disable command engine */
  if ((rc = ahci_stop_engine(ai, p)) != 0)
  {
    dprintf(0,"error: failed to stop port HW engine (%d)\n", rc);
    return(rc);
  }

  /* clear any pending port IRQs */
  tmp = readl(port_mmio + PORT_IRQ_STAT);
  if (tmp) writel(port_mmio + PORT_IRQ_STAT, tmp);
  writel(ai->mmio + HOST_IRQ_STAT, 1UL << p);

  /* reset PxSACT register (tagged command queues, not reset by COMRESET) */
  writel(port_mmio + PORT_SCR_ACT, 0);
  readl(port_mmio + PORT_SCR_ACT);  /* flush */

  return(0);
}

/******************************************************************************
 * Stop port FIS reception. Copied from Linux AHCI driver and adopted to
 * OS2AHCI.
 *
 * NOTE: This function uses a busy loop to wait for the DMA engine to stop. It
 *       should only be called at task time during initialization or in a
 *       context hook (e.g. when resetting a port).
 */
int ahci_stop_fis_rx(AD_INFO *ai, int p)
{
  u8 *port_mmio = port_base(ai, p);
  TIMER Timer;
  u32 tmp;
  int status;

  /* disable FIS reception */
  tmp = readl(port_mmio + PORT_CMD);
  tmp &= ~PORT_CMD_FIS_RX;
  writel(port_mmio + PORT_CMD, tmp);

  /* wait for completion, spec says 500ms, give it 1000ms */
  status = 0;
  TimerInit(&Timer, 1000);
  while (readl(port_mmio + PORT_CMD) & PORT_CMD_FIS_ON)
  {
    status = TimerCheckAndBlock(&Timer);
    if (status) break;
  }

  return(status ? -1 : 0);
}

/******************************************************************************
 * Stop port HW engine. Copied from Linux AHCI driver and adopted to OS2AHCI.
 *
 * NOTE: This function uses a busy loop to wait for the DMA engine to stop. It
 *       should only be called at task time during initialization or in a
 *       context hook (e.g. when resetting a port).
 */
int ahci_stop_engine(AD_INFO *ai, int p)
{
  u8 *port_mmio = port_base(ai, p);
  TIMER Timer;
  int status;
  u32 tmp;

  tmp = readl(port_mmio + PORT_CMD);

  /* check if the port is already stopped */
  if ((tmp & (PORT_CMD_START | PORT_CMD_LIST_ON)) == 0) return 0;

  /* set port to idle */
  tmp &= ~PORT_CMD_START;
  writel(port_mmio + PORT_CMD, tmp);

  /* wait for engine to stop. This could be as long as 500 msec */
  status = 0;
  TimerInit(&Timer, 500);
  while (readl(port_mmio + PORT_CMD) & PORT_CMD_LIST_ON)
  {
    status = TimerCheckAndBlock(&Timer);
    if (status) break;
  }

  return(status ? -1 : 0);
}

/******************************************************************************
 * Determine whether a port is busy executing commands.
 */
int ahci_port_busy(AD_INFO *ai, int p)
{
  u8 *port_mmio = port_base(ai, p);

  return(readl(port_mmio + PORT_SCR_ACT) != 0 || readl(port_mmio + PORT_CMD_ISSUE) != 0);
}

/******************************************************************************
 * Execute AHCI command for given IORB. This includes all steps typically
 * required by any of the ahci_*() IORB processing functions.
 *
 * NOTE: In order to prevent race conditions with port restart and reset
 *       handlers, we either need to keep the spinlock during the whole
 *       operation or set the adapter's busy flag. Since the expectation
 *       is that command preparation will be quick (it certainly doesn't
 *       involve delays), we're going with the spinlock for the time being.
 */
void ahci_exec_iorb(IORBH FAR16DATA *vIorb, IORBH *pIorb, int ncq_capable, int (*func)(IORBH FAR16DATA *, IORBH *pIorb, int))
{
  volatile u32 *cmds;
  ADD_WORKSPACE *aws = add_workspace(pIorb);
  AD_INFO *ai = &ad_infos[iorb_unit_adapter(pIorb)];
  P_INFO *port = &ai->ports[iorb_unit_port(pIorb)];
  ULONG timeout;
  u8 *port_mmio = port_base(ai, iorb_unit_port(pIorb));
  u16 cmd_max = ai->cmd_max;
  int i;

  /* determine timeout in milliseconds */
  switch (pIorb->Timeout)
  {
  case 0:
    timeout = DEFAULT_TIMEOUT;
    break;
  case 0xffffffffUL:
    timeout = 0xffffffffUL;
    break;
  default:
    timeout = pIorb->Timeout * 1000;
    break;
  }

  DPRINTF(7,"---------- ahci_exec_iorb: iorb=%x\n", vIorb);

  /* Enable AHCI mode; apparently, the AHCI mode may end up becoming
   * disabled, either during the boot sequence (by the BIOS) or by
   * something else. The Linux AHCI drivers have this call in the
   * command processing chain, and apparently for a good reason because
   * without this, commands won't be executed.
   */
  ahci_enable_ahci(ai);

  /* determine whether this will be an NCQ request */
  aws->is_ncq = 0;
  if (ncq_capable && port->devs[iorb_unit_device(pIorb)].ncq_max > 1 &&
      (ai->cap & HOST_CAP_NCQ) && !aws->no_ncq && init_complete)
  {

    /* We can make this an NCQ request; limit command slots to the maximum
     * NCQ tag number reported by the device - 1. Why "minus one"? I seem to
     * recall an issue related to using all 32 tag numbers but can't quite
     * pinpoint it right now. One less won't make much of a difference...
     */
    aws->is_ncq = 1;
    if ((cmd_max = port->devs[iorb_unit_device(pIorb)].ncq_max - 1) > ai->cmd_max)
    {
      cmd_max = ai->cmd_max;
    }
    DPRINTF(8,"NCQ command; cmd_max = %d->%d\n", ai->cmd_max, cmd_max);
  }

  /* make sure adapter is available */
  spin_lock(drv_lock);
  if (!ai->busy)
  {

    if (!init_complete)
    {
      /* no IRQ handlers or context hooks availabe at this point */
      ai->busy = 1;
      spin_unlock(drv_lock);
      ahci_exec_polled_iorb(vIorb, pIorb, func, timeout);
      ai->busy = 0;
      return;
    }

    /* make sure we don't mix NCQ and regular commands */
    if (aws->is_ncq && port->reg_cmds == 0 || !aws->is_ncq && port->ncq_cmds == 0)
    {
      /* Find next available command slot. We use a simple round-robin
       * algorithm for this to prevent commands with higher slot indexes
       * from stalling when new commands are coming in frequently.
       */
      cmds = (aws->is_ncq) ? &port->ncq_cmds : &port->reg_cmds;
      for (i = 0; i <= cmd_max; i++)
      {
        if (++(port->cmd_slot) > cmd_max) port->cmd_slot = 0;
        if ((*cmds & (1UL << port->cmd_slot)) == 0) break;
      }

      if ((*cmds & (1UL << port->cmd_slot)) == 0)
      {
        /* found idle command slot; prepare command */
        if (func(vIorb, pIorb, port->cmd_slot))
        {
          /* Command preparation failed, or no HW command required; IORB
           * will already have the error code if there was an error.
           */
          spin_unlock(drv_lock);
          iorb_done(vIorb, pIorb);
          return;
        }

        /* start timer for this IORB */
        Timer_StartTimerMS(&aws->timer, timeout, timeout_callback, CastFar16ToULONG(vIorb));

        /* issue command to hardware */
        *cmds |= (1UL << port->cmd_slot);
        aws->queued_hw = 1;
        aws->cmd_slot = port->cmd_slot;

        DPRINTF(7,"Issuing command Slot=%d cmds=%x\n", port->cmd_slot, *cmds);
        if (aws->is_ncq)
        {
          writel(port_mmio + PORT_SCR_ACT, (1UL << port->cmd_slot));
          readl(port_mmio + PORT_SCR_ACT); /* flush */
        }
        writel(port_mmio + PORT_CMD_ISSUE, (1UL << port->cmd_slot));
        readl(port_mmio + PORT_CMD_ISSUE); /* flush */

        spin_unlock(drv_lock);
        return;
      }
    }
  }

  /* requeue this IORB; it will be picked up again in trigger_engine() */
  aws->processing = 0;
  spin_unlock(drv_lock);
}

/******************************************************************************
 * Execute polled IORB command. This function is called by ahci_exec_iorb()
 * when the initialization has not yet completed. The reasons for polling until
 * initialization has completed are:
 *
 *  - We need to restore the BIOS configuration after we're done with this
 *    command because someone might still call int 13h routines; sending
 *    asynchronous commands and waiting for interrupts to indicate completion
 *    won't work in such a scenario.
 *  - Our context hooks won't work while the device managers are initializing
 *    (they can't yield at init time).
 *  - The device managers typically poll for command completion during
 *    initialization so it won't make much of a difference, anyway.
 *
 * NOTE: This function must be called with the adapter-level busy flag set but
 *       without the driver-level spinlock held.
 */
void ahci_exec_polled_iorb(IORBH FAR16DATA *vIorb, IORBH *pIorb, int (*func)(IORBH FAR16DATA *, IORBH *pIorb, int), ULONG timeout)
{
  AHCI_PORT_CFG *pc = NULL;
  AD_INFO *ai = ad_infos + iorb_unit_adapter(vIorb);
  int p = iorb_unit_port(pIorb);
  u8 *port_mmio = port_base(ai, p);
  TIMER Timer;
  int rc;

  /* enable AHCI mode */
  if (ahci_enable_ahci(ai) != 0)
  {
    iorb_seterr(pIorb, IOERR_ADAPTER_NONSPECIFIC);
    goto restore_bios_config;
  }

  /* check whether command slot 0 is available */
  if ((readl(port_mmio + PORT_CMD_ISSUE) & 1) != 0)
  {
    iorb_seterr(pIorb, IOERR_DEVICE_BUSY);
    goto restore_bios_config;
  }

  /* save port configuration */
  if ((pc = ahci_save_port_config(ai, p)) == NULL)
  {
    iorb_seterr(pIorb, IOERR_CMD_SW_RESOURCE);
    goto restore_bios_config;
  }

  /* restart/reset port (includes the necessary port configuration) */
  if (init_reset)
  {
    /* As outlined in ahci_restore_bios_config(), switching back and
     * forth between SATA and AHCI mode requires a COMRESET to force
     * the corresponding controller subsystem to rediscover attached
     * devices. Thus, we'll reset the port instead of stopping and
     * starting it.
     */
    if (ahci_reset_port(ai, p, 0))
    {
      iorb_seterr(pIorb, IOERR_ADAPTER_NONSPECIFIC);
      goto restore_bios_config;
    }

  }
  else if (ahci_stop_port(ai, p) || ahci_start_port(ai, p, 0))
  {
    iorb_seterr(pIorb, IOERR_ADAPTER_NONSPECIFIC);
    goto restore_bios_config;
  }

  /* prepare command */
  if (func(vIorb, pIorb, 0) == 0)
  {
    /* successfully prepared cmd; issue cmd and wait for completion */
    DPRINTF(3,"---------- executing polled cmd on slot 0...");
    writel(port_mmio + PORT_CMD_ISSUE, 1);
    TimerInit(&Timer, timeout);
    while (readl(port_mmio + PORT_CMD_ISSUE) & 1)
    {
      rc = TimerCheckAndBlock(&Timer);
      if (rc) break;
    }

    /* 0x89 = BSY(0x80) | DRQ(0x08) | ERR(0x01) */
    if (rc)
    {
      DPRINTF(3," timeout for IORB %x", vIorb);
      iorb_seterr(pIorb, IOERR_ADAPTER_TIMEOUT);
    }
    else if (readl(port_mmio + PORT_SCR_ERR) != 0 || readl(port_mmio + PORT_TFDATA) & 0x89)
    {
      DPRINTF(3," polled cmd error for IORB %x", vIorb);
      iorb_seterr(pIorb, IOERR_DEVICE_NONSPECIFIC);
      ahci_reset_port(ai, iorb_unit_port(pIorb), 0);
    }
    else
    {
      /* successfully executed command */
      if (add_workspace(pIorb)->ppfunc != NULL)
      {
        add_workspace(pIorb)->ppfunc(vIorb, pIorb);
      }
      else
      {
        add_workspace(pIorb)->complete = 1;
      }
    }
    DPRINTF(3,"\n");
  }

restore_bios_config:
  /* restore BIOS configuration */
  if (pc != NULL)
  {
    ahci_restore_port_config(ai, p, pc);
  }
  ahci_restore_bios_config(ai);

  if (add_workspace(pIorb)->complete | (pIorb->Status | IORB_ERROR))
  {
    iorb_done(vIorb, pIorb);
  }
  return;
}

/******************************************************************************
 * Execute polled ATA/ATAPI command. This function will block until the command
 * has completed or the timeout has expired, thus it should only be used during
 * initialization. Furthermore, it will always use command slot zero.
 *
 * The difference to ahci_exec_polled_iorb() is that this function executes
 * arbitrary ATA/ATAPI commands outside the context of an IORB. It's typically
 * used when scanning for devices during initialization.
 */
int ahci_exec_polled_cmd(AD_INFO *ai, int p, int d, int timeout, int cmd, ...)
{
  va_list va;
  u8 *port_mmio = port_base(ai, p);
  u32 tmp;
  int rc;
  TIMER Timer;

  /* verify that command slot 0 is idle */
  if (readl(port_mmio + PORT_CMD_ISSUE) & 1)
  {
    DPRINTF(3,"port %d slot 0 is not idle; not executing polled cmd\n", p);
    return(-1);
  }

  /* fill in command slot 0 */
  va_start(va, cmd);
  if ((rc = v_ata_cmd(ai, p, d, 0, cmd, va)) != 0) return(rc);

  /* start command execution for slot 0 */
  DPRINTF(3,"---------- executing polled cmd...");
  writel(port_mmio + PORT_CMD_ISSUE, 1);

  /* wait until command has completed */
  TimerInit(&Timer, timeout);
  rc = 0;
  while (readl(port_mmio + PORT_CMD_ISSUE) & 1)
  {
    rc = TimerCheckAndBlock(&Timer);
    if (rc)
    {
      DPRINTF(2," Timeout");
      break;
    }
  }

  tmp = readl(port_mmio + PORT_SCR_ERR);
  if (tmp & PORT_ERR_FAIL_BITS)
  {
    DPRINTF(2," SERR = 0x%08lx", tmp);
    rc = 1;
  }
  /* 0x89 = BSY(0x80) | DRQ(0x08) | ERR(0x01) */
  if (((tmp = readl(port_mmio + PORT_TFDATA)) & 0x89) != 0)
  {
    DPRINTF(2," TFDATA = 0x%08lx", tmp);
    rc = 1;
  }

  if (rc)
  {
    DPRINTF(3,"failed\n");
    ahci_reset_port(ai, p, 0);
    return(-1);
  }
  DPRINTF(3,"success\n");
  return(0);
}

/******************************************************************************
 * Flush write cache of the specified device. Since there's no equivalent IORB
 * command, we'll execute this command directly using polling. Otherwise, we
 * would have to create a fake IORB, add it to the port's IORB queue, ...
 *
 * Besides, this function is only called when shutting down and the code there
 * would have to wait for the flush cache command to complete as well, using
 * polling just the same...
 */
int ahci_flush_cache(AD_INFO *ai, int p, int d)
{
  if (!ai->ports[p].devs[d].atapi)
  {
    DPRINTF(2,"flushing cache on %d.%d.%d\n", ad_no(ai), p, d);
    return(ahci_exec_polled_cmd(ai, p, d, 30000,
           ai->ports[p].devs[d].lba48 ? ATA_CMD_FLUSH_EXT : ATA_CMD_FLUSH, AP_END));
  }
  return 0;
}

/******************************************************************************
 * Set device into IDLE mode (spin down); this was used during
 * debugging/testing and is now unused; it's still there in case we need it
 * again...
 *
 * If 'idle' is != 0, the idle timeout is set to 5 seconds, otherwise it
 * is turned off.
 */
int ahci_set_dev_idle(AD_INFO *ai, int p, int d, int idle)
{
  DPRINTF(3,"sending IDLE=%d command to port %d\n", idle, p);
  return ahci_exec_polled_cmd(ai, p, d, 500, ATA_CMD_IDLE, AP_COUNT, idle ? 1 : 0, AP_END);
}

/******************************************************************************
 * AHCI top-level hardware interrupt handler. This handler finds the adapters
 * and ports which have issued the interrupt and calls the corresponding
 * port interrupt handler.
 *
 * On entry, OS/2 will have processor interrupts enabled because we're using
 * shared IRQs but we won't be preempted by another interrupt on the same
 * IRQ level until we indicated EOI. We'll keep it this way, only requesting
 * the driver-level spinlock when actually changing the driver state (IORB
 * queues, ...)
 */
#pragma aux ahci_intr parm [eax]
int ahci_intr(u32 irq)
{
  u32 irq_stat;
  int handled = 0;
  int a;
  int p;

  /* find adapter(s) with pending interrupts */
  for (a = 0; a < ad_info_cnt; a++)
  {
    AD_INFO *ai = ad_infos + a;

    if (ai->irq == irq && (irq_stat = readl(ai->mmio + HOST_IRQ_STAT)) != 0)
    {
      /* this adapter has interrupts pending */
      u32 irq_masked = irq_stat & ai->port_map;

      for (p = 0; p <= ai->port_max; p++)
      {
        if (irq_masked & (1UL << p))
        {
          ahci_port_intr(ai, p);
        }
      }

      /* clear interrupt condition on the adapter */
      writel(ai->mmio + HOST_IRQ_STAT, irq_stat);
      readl(ai->mmio + HOST_IRQ_STAT); /* flush */
      handled = 1;
    }
  }

  if (handled)
  {
    /* Trigger state machine to process next IORBs, if any. Due to excessive
     * IORB requeue operations (e.g. when processing large unaligned reads or
     * writes), we may be stacking interrupts on top of each other. If we
     * detect this, we'll pass this on to the engine context hook.
     */
    #if 0
    if ((u32)&irq_stat < 0xf000)
    {
      DPRINTF(0,"IRQ stack running low; arming engine context hook\n");
      /* Rousseau:
       * A context hook cannot be re-armed before it has completed.
       * (?:\IBMDDK\DOCS\PDDREF.INF->Device Helper (DevHlp) Services)->ArmCtxHook)
       * Also, it is executed at task-time, thus in the context of some
       * application thread. Stacked interrupts with a stack below the
       * threshold specified above, (0xf000), will repeatly try to arm the
       * context hook, but since we are in an interrupted interrupt handler,
       * it's highly unlikely the hook has completed.
       * So, possibly only the first arming is succesful and subsequent armings
       * will fail because no task-time thread has run between the stacked
       * interrupts. One hint would be that if the dispatching truely worked,
       * excessive stacked interrupts in VBox would not be a problem.
       * This needs some more investigation.
       */
      KernArmHook(engine_ctxhook_h, 0, 0);
    }
    else
    #endif
    {
      spin_lock(drv_lock);
      trigger_engine();
      spin_unlock(drv_lock);
    }
    DevCli();
    Dev32Help_EOI(irq);
    return(1); /* handled */
  }

  return(0); /* not handled */
}

/******************************************************************************
 * AHCI port-level interrupt handler. As described above, processor interrupts
 * are enabled on entry thus we have to protect shared resources with a
 * spinlock.
 */
void ahci_port_intr(AD_INFO *ai, int p)
{
  IORB_QUEUE done_queue;
  IORBH FAR16DATA *vIorb;
  IORBH FAR16DATA *vNext = FAR16NULL;
  u8 *port_mmio = port_base(ai, p);
  u32 irq_stat;
  u32 active_cmds;
  u32 done_mask;

  /* get interrupt status and clear it right away */
  irq_stat = readl(port_mmio + PORT_IRQ_STAT);
  writel(port_mmio + PORT_IRQ_STAT, irq_stat);
  readl(port_mmio + PORT_IRQ_STAT); /* flush */

  memset(&done_queue, 0x00, sizeof(done_queue));

  if (irq_stat & PORT_IRQ_ERROR)
  {
    /* this is an error interrupt;
     * disable port interrupts to avoid IRQ storm until error condition
     * has been cleared by the restart handler
     */
    writel(port_mmio + PORT_IRQ_MASK, 0);
    ahci_error_intr(ai, p, irq_stat);
    return;
  }

  spin_lock(drv_lock);

  /* Find out which command slots have completed. Since error recovery for
   * NCQ commands interfers with non-NCQ commands, the upper layers will
   * make sure there's never a mixture of NCQ and non-NCQ commands active
   * on any port at any given time. This makes it easier to find out which
   * commands have completed, too.
   */
  if (ai->ports[p].ncq_cmds != 0)
  {
    active_cmds = readl(port_mmio + PORT_SCR_ACT);
    done_mask = ai->ports[p].ncq_cmds ^ active_cmds;
    DPRINTF(7,"[ncq_cmds]: active_cmds=0x%08x done_mask=0x%08x\n", active_cmds, done_mask);
  }
  else
  {
    active_cmds = readl(port_mmio + PORT_CMD_ISSUE);
    done_mask = ai->ports[p].reg_cmds ^ active_cmds;
    DPRINTF(7,"[reg_cmds]: active_cmds=0x%08x  done_mask=0x%08x\n", active_cmds, done_mask);
  }

  /* Find the IORBs related to the completed commands and complete them.
   *
   * NOTES: The spinlock must not be released while in this loop to prevent
   *        race conditions with timeout handlers or other threads in SMP
   *        systems.
   *
   *        Since we hold the spinlock when IORBs complete, we can't call the
   *        IORB notification routine right away because this routine might
   *        schedule another IORB which could cause a deadlock. Thus, we'll
   *        add all IORBs to be completed to a temporary queue which will be
   *        processed after releasing the spinlock.
   */
  for (vIorb = ai->ports[p].iorb_queue.vRoot; vIorb != FAR16NULL; vIorb = vNext)
  {
    IORBH *pIorb = Far16ToFlat(vIorb);
    ADD_WORKSPACE *aws = (ADD_WORKSPACE *) &pIorb->ADDWorkSpace;

    vNext = pIorb->pNxtIORB;
    if (aws->queued_hw && (done_mask & (1UL << aws->cmd_slot)))
    {
      /* this hardware command has completed */
      ai->ports[p].ncq_cmds &= ~(1UL << aws->cmd_slot);
      ai->ports[p].reg_cmds &= ~(1UL << aws->cmd_slot);

      /* call post-processing function, if any */
      if (aws->ppfunc != NULL) aws->ppfunc(vIorb, pIorb);
      else aws->complete = 1;

      if (aws->complete)
      {
        /* this IORB is complete; move IORB to our temporary done queue */
        iorb_queue_del(&ai->ports[p].iorb_queue, vIorb);
        iorb_queue_add(&done_queue, vIorb, pIorb);
        aws_free(add_workspace(pIorb));
      }
    }
  }

  spin_unlock(drv_lock);

  /* complete all IORBs in the done queue */
  for (vIorb = done_queue.vRoot; vIorb != FAR16NULL; vIorb = vNext)
  {
    IORBH *pIorb = Far16ToFlat(vIorb);

    vNext = pIorb->pNxtIORB;

    iorb_complete(vIorb, pIorb);
  }
}

/******************************************************************************
 * AHCI error interrupt handler. Errors include interface errors and device
 * errors (usually triggered by the error bit in the AHCI task file register).
 *
 * Since this involves long-running operations such as restarting or even
 * resetting a port, this function is invoked at task time via a context
 * hook.
 *
 * NOTE: AHCI controllers stop all processing when encountering an error
 *       condition in order to give the driver time to find out what exactly
 *       went wrong. This means no new commands will be processed until we
 *       clear the error register and restore the "commands issued" register.
 */
void ahci_error_intr(AD_INFO *ai, int p, u32 irq_stat)
{
  int reset_port = 0;

  /* Handle adapter and interface errors. Those typically require a port
   * reset, or worse.
   */
  ai->ports[p].error_count++;

  if (irq_stat & PORT_IRQ_UNK_FIS)
  {
    #ifdef DEBUG
    u32 *unk = (u32 *) (port_dma_base(ai, p)->rx_fis + RX_FIS_UNK);
    DPRINTF(0,"warning: unknown FIS %08lx %08lx %08lx %08lx\n", unk[0], unk[1], unk[2], unk[3]);
    #endif
    reset_port = 1;
  }
  if (irq_stat & (PORT_IRQ_HBUS_ERR | PORT_IRQ_HBUS_DATA_ERR))
  {
    dprintf(0,"warning: host bus [data] error for port #%d\n", p);
    reset_port = 1;
  }
  if (irq_stat & PORT_IRQ_IF_ERR && !(ai->flags & AHCI_HFLAG_IGN_IRQ_IF_ERR))
  {
    dprintf(0,"warning: interface fatal error for port #%d\n", p);
    reset_port = 1;
  }
  if (reset_port)
  {
    /* need to reset the port; leave this to the reset context hook */

    ports_to_reset[ad_no(ai)] |= 1UL << p;
    KernArmHook(reset_ctxhook_h, 0, 0);

    /* no point analyzing device errors after a reset... */
    return;
  }

  dprintf(0,"port #%d interrupt error status: 0x%08lx; restarting port\n", p, irq_stat);

  /* Handle device-specific errors. Those errors typically involve restarting
   * the corresponding port to resume operations which can take some time,
   * thus we need to offload this functionality to the restart context hook.
   */
  ports_to_restart[ad_no(ai)] |= 1UL << p;
  KernArmHook(restart_ctxhook_h, 0, 0);
}

/******************************************************************************
 * Get device or media geometry. Device and media geometry are expected to be
 * the same for non-removable devices.
 */
void ahci_get_geometry(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  DPRINTF(7,"ahci_get_geometry(%d.%d.%d)\n", iorb_unit_adapter(pIorb),
          iorb_unit_port(pIorb), iorb_unit_device(pIorb));

  ahci_exec_iorb(vIorb, pIorb, 0, cmd_func(pIorb, get_geometry));
}

/******************************************************************************
 * Test whether unit is ready.
 */
void ahci_unit_ready(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  DPRINTF(7,"ahci_unit_ready(%d.%d.%d)\n", iorb_unit_adapter(pIorb),
          iorb_unit_port(pIorb), iorb_unit_device(pIorb));

  ahci_exec_iorb(vIorb, pIorb, 0, cmd_func(pIorb, unit_ready));
}

/******************************************************************************
 * Read sectors from AHCI device.
 */
void ahci_read(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  DPRINTF(7,"ahci_read(%d.%d.%d, %d, %d)\n", iorb_unit_adapter(vIorb),
          iorb_unit_port(pIorb), iorb_unit_device(pIorb),
          ((IORB_EXECUTEIO *) pIorb)->RBA,
          ((IORB_EXECUTEIO *) pIorb)->BlockCount);

  ahci_exec_iorb(vIorb, pIorb, 1, cmd_func(pIorb, read));
}

/******************************************************************************
 * Verify readability of sectors on AHCI device.
 */
void ahci_verify(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  DPRINTF(7,"ahci_verify(%d.%d.%d, %d, %d)\n", iorb_unit_adapter(pIorb),
          iorb_unit_port(pIorb), iorb_unit_device(pIorb),
          ((IORB_EXECUTEIO *)pIorb)->RBA,
          ((IORB_EXECUTEIO *)pIorb)->BlockCount);

  ahci_exec_iorb(vIorb, pIorb, 0, cmd_func(pIorb, verify));
}

/******************************************************************************
 * Write sectors to AHCI device.
 */
void ahci_write(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  DPRINTF(7,"ahci_write(%d.%d.%d, %d, %d)\n", iorb_unit_adapter(pIorb),
          iorb_unit_port(pIorb), iorb_unit_device(pIorb),
          ((IORB_EXECUTEIO *)pIorb)->RBA,
          ((IORB_EXECUTEIO *)pIorb)->BlockCount);

  ahci_exec_iorb(vIorb, pIorb, 1, cmd_func(pIorb, write));
}

/******************************************************************************
 * Execute SCSI (ATAPI) command.
 */
void ahci_execute_cdb(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  int a = iorb_unit_adapter(pIorb);
  int p = iorb_unit_port(pIorb);
  int d = iorb_unit_device(pIorb);

  DHEXDUMP(5,Far16ToFlat(((IORB_ADAPTER_PASSTHRU *)pIorb)->pControllerCmd),
        ((IORB_ADAPTER_PASSTHRU *)pIorb)->ControllerCmdLen,
        "ahci_execute_cdb(%d.%d.%d): ", a, p, d);

  if (ad_infos[a].ports[p].devs[d].atapi)
  {
    ahci_exec_iorb(vIorb, pIorb, 0, atapi_execute_cdb);
  }
  else
  {
    iorb_seterr(pIorb, IOERR_CMD_NOT_SUPPORTED);
    iorb_done(vIorb, pIorb);
  }
}

/******************************************************************************
 * Execute ATA command. Please note that this is allowed for both ATA and
 * ATAPI devices because ATAPI devices will process some ATA commands as well.
 */
void ahci_execute_ata(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  #ifdef DEBUG
  int a = iorb_unit_adapter(pIorb);
  int p = iorb_unit_port(pIorb);
  int d = iorb_unit_device(pIorb);

  DHEXDUMP(5,Far16ToFlat(((IORB_ADAPTER_PASSTHRU *)pIorb)->pControllerCmd),
        ((IORB_ADAPTER_PASSTHRU *)pIorb)->ControllerCmdLen,
        "ahci_execute_ata(%d.%d.%d): ", a, p, d);
  #endif

  ahci_exec_iorb(vIorb, pIorb, 0, ata_execute_ata);
}

