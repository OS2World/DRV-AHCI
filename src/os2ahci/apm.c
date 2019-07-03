/******************************************************************************
 * apm.c - Functions to interface with the legacy APM driver, and suspend / resume functions.
 *
 * Copyright (c) 2011 thi.guten Software Development
 * Copyright (c) 2011 Mensys B.V.
 * Portions copyright (c) 2013-2018 David Azarewicz
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

/* Legacy APM support is not needed on eCS systems with ACPI and is more
 * reliable without it enabled.
 */
#ifdef LEGACY_APM

#include <apmcalls.h>
USHORT _cdecl apm_event    (APMEVENT *evt);

/******************************************************************************
 * Connect to APM driver and register for power state change events.
 */
void apm_init(void)
{
  USHORT rc;

  /* connect to APM driver */
  if ((rc = APMAttach()) != 0) {
    DPRINTF(2,"couldn't connect to APM driver (rc = %d)\n", rc);
    return;
  }

  /* register for suspend/resume events */
  if ((rc = APMRegister(apm_event, APM_NOTIFYSETPWR |
                                   APM_NOTIFYNORMRESUME |
                                   APM_NOTIFYCRITRESUME, 0)) != 0) {
    DPRINTF(2,"couldn't register for power event notificatins (rc = %d)\n", rc);
    return;
  }
}

/******************************************************************************
 * APM event handler
 */
USHORT _cdecl apm_event(APMEVENT *evt)
{
  USHORT msg = (USHORT) evt->ulParm1;

  DPRINTF(2,"received APM event: 0x%x/0x%x\n");

  switch (msg) {

  case APM_SETPWRSTATE:
    if (evt->ulParm2 >> 16 != APM_PWRSTATEREADY) {
      /* we're suspending */
      suspend();
    }
    break;

  case APM_NORMRESUMEEVENT:
  case APM_CRITRESUMEEVENT:
    /* we're resuming */
    resume();
    break;

  default:
    DPRINTF(2,"unknown APM event; ignoring...\n");
    break;
  }

  return(0);
}
#endif /* LEGACY_APM */

/******************************************************************************
 * Suspend handler. In a nutshell, it'll turn off interrupts and flush all
 * write caches.
 */
void suspend(void)
{
  int a;
  int p;
  int d;
  TIMER Timer;

  if (suspended) return;
  DPRINTF(2,"suspend()\n");

  /* restart all ports with interrupts disabled */
  for (a = 0; a < ad_info_cnt; a++) {
    AD_INFO *ai = ad_infos + a;

    lock_adapter(ai);
    for (p = 0; p <= ai->port_max; p++) {
      /* wait until all active commands have completed on this port */
      TimerInit(&Timer, 250);
      while (ahci_port_busy(ai, p)) {
        if (TimerCheckAndBlock(&Timer)) break;
      }

      /* restart port with interrupts disabled */
      ahci_stop_port(ai, p);
      ahci_start_port(ai, p, 0);

      /* flush cache on all attached devices */
      for (d = 0; d <= ai->ports[p].dev_max; d++)
      {
        if (ai->ports[p].devs[d].present && !ai->ports[p].devs[d].ignored)
        {
          ahci_flush_cache(ai, p, d);
        }
      }
    }

    /* AHCI spec rev1.1 section 8.3.3:
     * Software must disable interrupts prior to requesting a transition of the HBA to D3 state.
     */
    writel(ai->mmio + HOST_CTL, readl(ai->mmio + HOST_CTL) & ~HOST_IRQ_EN);
    readl(ai->mmio + HOST_CTL); /* flush */

    /* TODO: put the device into the D3 state */
  }

  /* reset init_complete so that we can process IORBs without interrupts */
  init_complete = 0;

  suspended = 1;
  DPRINTF(2,"suspend() finished\n");
}

/******************************************************************************
 * Resume handler. All ports are restarted with interrupts enabled using
 * the same function as the IOCM_COMPLETE_INIT handler does.
 */
void resume(void)
{
  int a;

  if (!suspended) return;
  DPRINTF(2,"resume()\n");

  for (a = 0; a < ad_info_cnt; a++) {
    AD_INFO *ai = ad_infos + a;

    /* TODO: put the device into the D0 state */

    //ahci_reset_controller(ai);

    /* Complete initialization of this adapter; this will restart the ports
     * with interrupts enabled and take care of whatever else needs to be
     * done to get the adapter and its ports up and running.
     */
    ahci_complete_init(ai);
  }

  /* tell the driver we're again fully operational */
  init_complete = 1;

  /* unlock all adapters now that we have set the init_complete flag */
  for (a = 0; a < ad_info_cnt; a++) {
    AD_INFO *ai = ad_infos + a;
    unlock_adapter(ai);
  }

  suspended = 0;

  /* restart engine to resume IORB processing */
  /* The resume_sleep_flag and probably rearming the ctx hook is a temporary hack
   * to make resume kind of work when I/O operations are outstanding or started
   * during the suspend operation. This behavior may change with future versions
   * of the ACPI software which will make this hack unnecessary.
   */
  resume_sleep_flag = 5000;
  KernArmHook(engine_ctxhook_h, 0, 0);

  DPRINTF(2,"resume() finished\n");
}

/******************************************************************************
 * This is the kernel exit handler for panics and traps.
 * Assume the system is trashed and do the absolute minimum necessary
 * to put the adapters into a state so that the BIOS can operate the
 * adapters. We never need to recover from this as the system will be rebooted.
 */
void shutdown_driver(void)
{
  int a;
  int p;
  u16 i;
  u32 tmp;
  //int d;

  DPRINTF(1,"shutdown_driver() enter\n");

  for (a = 0; a < ad_info_cnt; a++)
  {
    AD_INFO *ai = ad_infos + a;

    /* Try to be nice. Wait 50ms for adapter to go not busy.
     * If it doesn't go not busy in that time, too bad. Stop it anyway.
     */
    for (i=0; i<50000 && ai->busy; i++) udelay(1000);

    for (p = 0; p <= ai->port_max; p++)
    {
      u8 *port_mmio = port_base(ai, p);

      /* Wait up to 50ms for port to go not busy. Again stop it
       * anyway if it doesn't go not busy in that time.
       */
      for (i=0; i<50000 && ahci_port_busy(ai, p); i++) udelay(1000);

      /* stop port */
      writel(port_mmio + PORT_IRQ_MASK, 0); /* disable port interrupts */
      writel(port_mmio + PORT_CMD, readl(port_mmio + PORT_CMD) & ~PORT_CMD_FIS_RX); /* disable FIS reception */
      while (readl(port_mmio + PORT_CMD) & PORT_CMD_FIS_ON); /* wait for it to stop */
      writel(port_mmio + PORT_CMD, readl(port_mmio + PORT_CMD) & ~PORT_CMD_START); /* set port to idle */
      while (readl(port_mmio + PORT_CMD) & PORT_CMD_LIST_ON); /* wait for it to stop */

      /* clear any pending port IRQs */
      tmp = readl(port_mmio + PORT_IRQ_STAT);
      if (tmp) writel(port_mmio + PORT_IRQ_STAT, tmp);
      writel(ai->mmio + HOST_IRQ_STAT, 1UL << p);

      /* reset PxSACT register (tagged command queues, not reset by COMRESET) */
      writel(port_mmio + PORT_SCR_ACT, 0);
      readl(port_mmio + PORT_SCR_ACT);  /* flush */

      #if 0
      /* cannot flush caches this way */
      ahci_start_port(ai, p, 0);

      /* flush cache on all attached devices */
      for (d = 0; d <= ai->ports[p].dev_max; d++)
      {
        if (ai->ports[p].devs[d].present && !ai->ports[p].devs[d].ignored)
        {
          ahci_flush_cache(ai, p, d);
        }
      }
      #endif
    }
  }

  init_complete = 0;

  /* restore BIOS configuration for each adapter */
  for (a = 0; a < ad_info_cnt; a++)
  {
    ahci_restore_bios_config(ad_infos + a);
  }

  DPRINTF(1,"shutdown_driver() finished\n");
}

