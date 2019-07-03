/******************************************************************************
 * ctxhook.c - context hooks (kernel thread functions) for os2ahci
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

/* ------------------------ typedefs and structures ------------------------ */

/* -------------------------- function prototypes -------------------------- */

/* ------------------------ global/static variables ------------------------ */

/* port restart context hook and input data */
ULONG           restart_ctxhook_h;
volatile u32    ports_to_restart[MAX_AD];

/* port reset context hook and input data */
ULONG           reset_ctxhook_h;
ULONG           th_reset_watchdog;
volatile u32    ports_to_reset[MAX_AD];
IORB_QUEUE      abort_queue;

/* trigger engine context hook and input data */
ULONG           engine_ctxhook_h;

/* ----------------------------- start of code ----------------------------- */

/******************************************************************************
 * Port restart context hook. This context hook is executed at task time and
 * will handle ports which are stopped due to a device error condition.
 *
 * The following conditions may exist:
 *
 *  - Only a single non-NCQ command is executed by the AHCI adapter at any
 *    given time (even if more are outstanding). This is the case for single
 *    devices or port multipliers without FIS-based command switching. Error
 *    recovery is simple because we know which command has failed and that
 *    all other commands have not yet started executing. Thus, we can requeue
 *    all of them, replacing the failing command with a "request sense"
 *    command to get error details.
 *
 *  - Multiple non-NCQ commands are executed on different devices behind a
 *    port multiplier which supports FIS-based command switching. This is
 *    more difficult to recover from but currently not an issue because we
 *    don't yet support FIS-based command switching (the FIS receive areas
 *    would become too large for the current data model).
 *
 *  - One or more NCQ commands were active at the time of the error, with or
 *    without FIS-based command switching. We would have to interrogate the
 *    corresponding devices to find out which command has failed but if this
 *    is combined with FIS-based command switching, even the AHCI spec
 *    recommends to reset the port. This leads to a much simpler approach:
 *    requeue all NCQ commands (they are idempotent per definition, otherwise
 *    they couldn't be reordered by the device) with the 'no_ncq' flag set
 *    in the IORB and reset the port. Then those comands will be executed as
 *    regular commands. The error, if it reoccurs, can then be handled by
 *    one of the above cases.
 *
 * The upstream code will guarantee that we will never have a mix of NCQ and
 * non-NCQ commands active at the same time in order to reduce complexity
 * in the interrupt and error handlers.
 */
void _Syscall restart_ctxhook(ULONG parm)
{
  IORB_QUEUE done_queue;
  AD_INFO *ai;
  IORBH FAR16DATA *vProblemIorb;
  IORBH FAR16DATA *vIorb;
  IORBH FAR16DATA *vNext;
  u8 *port_mmio;
  int rearm_ctx_hook;
  int need_reset;
  int ccs;
  int a;
  int p;

  D32ThunkStackTo32();

  vNext = FAR16NULL;
  rearm_ctx_hook = 0;

  DPRINTF(8,"restart_ctxhook() started\n");
  memset(&done_queue, 0x00, sizeof(done_queue));

  spin_lock(drv_lock);

  for (a = 0; a < ad_info_cnt; a++)
  {
    ai = ad_infos + a;

    if (ai->busy)
    {
      /* this adapter is busy; leave it alone for now */
      rearm_ctx_hook = 1;
      continue;
    }

    for (p = 0; p <= ai->port_max; p++)
    {
      if (ports_to_restart[a] & (1UL << p))
      {
        ports_to_restart[a] &= ~(1UL << p);

        /* restart this port */
        port_mmio = port_base(ai, p);
        vProblemIorb = FAR16NULL;
        need_reset = 0;

        DPRINTF(8,"port %d, TF_DATA: 0x%x\n", p, readl(port_mmio + PORT_TFDATA));

        /* get "current command slot"; only valid if there are no NCQ cmds */
        ccs = (int) ((readl(port_mmio + PORT_CMD) >> 8) & 0x1f);
        DPRINTF(8," PORT_CMD      = 0x%x\n", ccs);

        for (vIorb = ai->ports[p].iorb_queue.vRoot; vIorb != FAR16NULL; vIorb = vNext)
        {
          IORBH *pIorb = Far16ToFlat(vIorb);
          ADD_WORKSPACE *aws = add_workspace(pIorb);
          vNext = pIorb->pNxtIORB;

          if (aws->queued_hw)
          {
            if (ai->ports[p].ncq_cmds & (1UL << aws->cmd_slot))
            {
              /* NCQ command; force non-NCQ mode and trigger port reset */
              ai->ports[p].ncq_cmds &= ~(1UL << aws->cmd_slot);
              aws->no_ncq = 1;
              need_reset = 1;
            }
            else
            {
              /* regular command; clear cmd bit and identify problem IORB */
              ai->ports[p].reg_cmds &= ~(1UL << aws->cmd_slot);
              if (aws->cmd_slot == ccs)
              {
                /* this is the non-NCQ command that failed */
                DPRINTF(0,"failing IORB: %x\n", vIorb);
                vProblemIorb = vIorb;
              }
            }
            /* we can requeue all IORBs unconditionally (see function comment) */
            if (aws->retries++ < MAX_RETRIES)
            {
              iorb_requeue(pIorb);
            }
            else
            {
              /* retry count exceeded; consider IORB aborted */
              iorb_seterr(pIorb, IOERR_CMD_ABORTED);
              iorb_queue_del(&ai->ports[p].iorb_queue, vIorb);
              iorb_queue_add(&done_queue, vIorb, pIorb);
              if (vIorb == vProblemIorb)
              {
                /* no further analysis -- we're done with this one */
                vProblemIorb = FAR16NULL;
              }
            }
          }
        }

        /* sanity check: issued command bitmaps should be 0 now */
        if (ai->ports[p].ncq_cmds != 0 || ai->ports[p].reg_cmds != 0)
        {
          DPRINTF(0,"warning: commands issued not 0 (%08lx/%08lx); resetting...\n",
                  ai->ports[p].ncq_cmds, ai->ports[p].reg_cmds);
          need_reset = 1;
        }

        if (!need_reset)
        {
          if ((readl(port_mmio + PORT_TFDATA) & 0x88) != 0)
          {
            /* device is not in an idle state */
            need_reset = 1;
          }
        }

        /* restart/reset port */
        ai->busy = 1;
        spin_unlock(drv_lock);
        if (need_reset)
        {
          ahci_reset_port(ai, p, 1);
        }
        else
        {
          ahci_stop_port(ai, p);
          ahci_start_port(ai, p, 1);
        }
        spin_lock(drv_lock);
        ai->busy = 0;

        /* reset internal port status */
        ai->ports[p].ncq_cmds = 0;
        ai->ports[p].reg_cmds = 0;
        ai->ports[p].cmd_slot = 0;

        if (vProblemIorb != FAR16NULL)
        {
          IORBH *pProblemIorb = Far16ToFlat(vProblemIorb);
          /* get details about the error that caused this IORB to fail */
          if (need_reset)
          {
            /* no way to retrieve error details after a reset */
            iorb_seterr(pProblemIorb, IOERR_DEVICE_NONSPECIFIC);
            iorb_queue_del(&ai->ports[p].iorb_queue, vProblemIorb);
            iorb_queue_add(&done_queue, vProblemIorb, pProblemIorb);

          }
          else
          {
            /* get sense information */
            ADD_WORKSPACE *aws = add_workspace(pProblemIorb);
            int d = iorb_unit_device(pProblemIorb);
            int (*req_sense)(IORBH FAR16DATA *, IORBH *, int) = (ai->ports[p].devs[d].atapi) ?
                                                   atapi_req_sense : ata_req_sense;

            aws->processing = 1;
            aws->queued_hw = 1;

            if (req_sense(vProblemIorb, pProblemIorb, 0) == 0)
            {
              /* execute request sense on slot #0 before anything else comes along */
              Timer_StartTimerMS(&aws->timer, 5000, timeout_callback, CastFar16ToULONG(vProblemIorb));
              aws->cmd_slot = 0;
              ai->ports[p].reg_cmds = 1;
              writel(port_mmio + PORT_CMD_ISSUE, 1);
              readl(port_mmio); /* flush */

            }
            else
            {
              /* IORB is expected to contain the error code; just move to done queue */
              iorb_queue_del(&ai->ports[p].iorb_queue, vProblemIorb);
              iorb_queue_add(&done_queue, vProblemIorb, pProblemIorb);
            }
          }
        }
      }
    }
  }

  spin_unlock(drv_lock);

  /* call notification routine on all IORBs which have completed */
  for (vIorb = done_queue.vRoot; vIorb != FAR16NULL; vIorb = vNext)
  {
    IORBH *pIorb = Far16ToFlat(vIorb);
    vNext = pIorb->pNxtIORB;

    spin_lock(drv_lock);
    aws_free(add_workspace(pIorb));
    spin_unlock(drv_lock);

    iorb_complete(vIorb, pIorb);
  }

  /* restart engine to resume IORB processing */
  spin_lock(drv_lock);
  trigger_engine();
  spin_unlock(drv_lock);

  DPRINTF(8,"restart_ctxhook() completed\n");

  /* Check whether we have to rearm ourselves because some adapters were busy
   * when we wanted to restart ports on them.
   */
  if (rearm_ctx_hook)
  {
    msleep(250);
    KernArmHook(restart_ctxhook_h, 0, 0);
  }
  KernThunkStackTo16();
}

/******************************************************************************
 * Reset and abort context hook. This function runs at task time and takes
 * care of port resets and their side effects. Input to this function are:
 *
 *  ports_to_reset[]   - array of port bitmaps, each bit indicating which port
 *                       should be reset unconditionally. This is primarily
 *                       used by the error interrupt handler.
 *
 *  abort_queue        - queue with IORBs to be arborted (timed-out, ...) If
 *                       any of these commands have reached the hardware, the
 *                       corresponding port is reset to interrupt command
 *                       execution. This is primarily used for timeout
 *                       handling and when IORBs are requested to be aborted.
 *
 * After resetting the requested ports, all remaining active IORBs on those
 * ports have to be retried or aborted. Whether a retry is attempted depends
 * on the kind of IORB -- those which are idempotent are retried, all others
 * are aborted. This is different from the port restart hook because the
 * restart hook can assume it is called with the port in error state, thus
 * the controller will have stopped executing commands. The reset handler can
 * be called at any time and we can't tell what's going on in the controller.
 *
 * The IORBs in the global abort_queue are expected to have their error code
 * set (aborted, timeout, ...) but must not be marked as 'done'; otherwise,
 * the upstream code might reuse the IORBs before we're done with them.
 */
void _Syscall reset_ctxhook(ULONG parm)
{
  IORB_QUEUE done_queue;
  AD_INFO *ai;
  IORBH FAR16DATA *vIorb;
  IORBH FAR16DATA *vNext;
  int rearm_ctx_hook;
  int a;
  int p;

  D32ThunkStackTo32();

  vNext = FAR16NULL;
  rearm_ctx_hook = 0;

  DPRINTF(8,"reset_ctxhook() started\n");
  memset(&done_queue, 0x00, sizeof(done_queue));

  spin_lock(drv_lock);

  if (th_reset_watchdog != 0)
  {
    /* watchdog timer still active -- just reset it */
    Timer_CancelTimer(th_reset_watchdog);
    th_reset_watchdog = 0;
  }

  /* add ports of active IORBs from the abort queue to ports_to_reset[] */
  for (vIorb = abort_queue.vRoot; vIorb != FAR16NULL; vIorb = vNext)
  {
    IORBH *pIorb = Far16ToFlat(vIorb);
    vNext = pIorb->pNxtIORB;
    a = iorb_unit_adapter(pIorb);
    p = iorb_unit_port(pIorb);
    ai = ad_infos + a;

    if (ai->busy)
    {
      /* this adapter is busy; leave it alone for now */
      rearm_ctx_hook = 1;
      continue;
    }

    /* move IORB to the local 'done' queue */
    iorb_queue_del(&abort_queue, vIorb);
    iorb_queue_add(&done_queue, vIorb, pIorb);

    /* reset port if the IORB has already been queued to hardware */
    if (add_workspace(pIorb)->queued_hw)
    {
      /* prepare port reset */
      ports_to_reset[a] |= (1UL << p);
    }
  }

  /* reset all ports in 'ports_to_reset[]' */
  for (a = 0; a < ad_info_cnt; a++)
  {
    ai = ad_infos + a;

    if (ai->busy)
    {
      /* this adapter is busy; leave it alone for now */
      rearm_ctx_hook = 1;
      continue;
    }

    for (p = 0; p <= ai->port_max; p++)
    {
      if (ports_to_reset[a] & (1UL << p))
      {
        ports_to_reset[a] &= ~(1UL << p);

        /* Reset this port. Since this is a rather slow operation, we'll
         * release the spinlock while doing so. The adapter is marked as
         * 'busy' to prevent similar routines (e.g. an ahci port scan) from
         * interfering.
         */
        ai->busy = 1;
        spin_unlock(drv_lock);
        ahci_reset_port(ai, p, 1);
        spin_lock(drv_lock);
        ai->busy = 0;

        /* reset port status */
        ai->ports[p].ncq_cmds = 0;
        ai->ports[p].reg_cmds = 0;
        ai->ports[p].cmd_slot = 0;

        /* retry or abort all remaining active commands on this port */
        for (vIorb = ai->ports[p].iorb_queue.vRoot; vIorb != FAR16NULL; vIorb = vNext)
        {
          IORBH *pIorb = Far16ToFlat(vIorb);
          ADD_WORKSPACE *aws = add_workspace(pIorb);
          vNext = pIorb->pNxtIORB;

          if (aws->queued_hw)
          {
            /* this IORB had already been queued to HW when we reset the port */
            if (aws->idempotent && aws->retries++ < MAX_RETRIES)
            {
              /* we can retry this IORB */
              iorb_requeue(pIorb);

            }
            else
            {
              /* we cannot retry this IORB; consider it aborted */
              pIorb->ErrorCode = IOERR_CMD_ABORTED;
              iorb_queue_del(&ai->ports[p].iorb_queue, vIorb);
              iorb_queue_add(&done_queue, vIorb, pIorb);
            }
          }
        }
      }
    }
  }

  spin_unlock(drv_lock);

  /* complete all aborted IORBs */
  for (vIorb = done_queue.vRoot; vIorb != FAR16NULL; vIorb = vNext)
  {
    IORBH *pIorb = Far16ToFlat(vIorb);
    vNext = pIorb->pNxtIORB;

    spin_lock(drv_lock);
    aws_free(add_workspace(pIorb));
    spin_unlock(drv_lock);

    pIorb->Status |= IORB_ERROR;
    iorb_complete(vIorb, pIorb);
  }

  /* restart engine to resume IORB processing */
  spin_lock(drv_lock);
  trigger_engine();
  spin_unlock(drv_lock);

  DPRINTF(8,"reset_ctxhook() completed\n");

  /* Check whether we have to rearm ourselves because some adapters were busy
   * when we wanted to reset ports on them.
   */
  if (rearm_ctx_hook)
  {
    msleep(250);
    KernArmHook(reset_ctxhook_h, 0, 0);
  }

  KernThunkStackTo16();
}

/******************************************************************************
 * IORB Engine context hook. This hook is executed if trigger_engine() came
 * to the conclusion that some of the IORBs keep bouncing, most likely due to
 * some condition on the adapter such as being busy. It could also be a very
 * busy system. Either way, this requires some task-time help.
 */
void _Syscall engine_ctxhook(ULONG parm)
{
  int iorbs_sent;
  int i;

  D32ThunkStackTo32();

  DPRINTF(8,"engine_ctxhook() started\n");
  if (resume_sleep_flag)
  {
    msleep(resume_sleep_flag);
    resume_sleep_flag = 0;
  }

  spin_lock(drv_lock);
  for (i = 0; i < 10; i++)
  {
    if ((iorbs_sent = trigger_engine_1()) == 0) break;
  }
  spin_unlock(drv_lock);

  DPRINTF(8,"engine_ctxhook() completed\n");

  if (iorbs_sent != 0)
  {
    /* need to rearm ourselves for another run */
    msleep(250);
    KernArmHook(engine_ctxhook_h, 0, 0);
  }

  KernThunkStackTo16();
}

