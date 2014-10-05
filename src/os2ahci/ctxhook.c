/******************************************************************************
 * ctxhook.c - context hooks (kernel thread functions) for os2ahci
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
void restart_ctxhook(ULONG parm)
{
  IORB_QUEUE done_queue;
  AD_INFO *ai;
  IORBH _far *problem_iorb;
  IORBH _far *iorb;
  IORBH _far *next = NULL;
  u8 _far *port_mmio;
  int rearm_ctx_hook = 0;
  int need_reset;
  int ccs;
  int a;
  int p;

  dprintf("restart_ctxhook() started\n");
  memset(&done_queue, 0x00, sizeof(done_queue));

  spin_lock(drv_lock);

  for (a = 0; a < ad_info_cnt; a++) {
    ai = ad_infos + a;

    if (ai->busy) {
      /* this adapter is busy; leave it alone for now */
      rearm_ctx_hook = 1;
      continue;
    }

    for (p = 0; p <= ai->port_max; p++) {
      if (ports_to_restart[a] & (1UL << p)) {
        ports_to_restart[a] &= ~(1UL << p);

        /* restart this port */
        port_mmio = port_base(ai, p);
        problem_iorb = NULL;
        need_reset = 0;

        dprintf("port %d, TF_DATA: 0x%lx\n", p, readl(port_mmio + PORT_TFDATA));

        /* get "current command slot"; only valid if there are no NCQ cmds */
        ccs = (int) ((readl(port_mmio + PORT_CMD) >> 8) & 0x1f);
        ddprintf(" PORT_CMD      = 0x%x\n", ccs);

        for (iorb = ai->ports[p].iorb_queue.root; iorb != NULL; iorb = next) {
          ADD_WORKSPACE _far *aws = add_workspace(iorb);
          next = iorb->pNxtIORB;

          if (aws->queued_hw) {
            if (ai->ports[p].ncq_cmds & (1UL << aws->cmd_slot)) {
              /* NCQ command; force non-NCQ mode and trigger port reset */
              ai->ports[p].ncq_cmds &= ~(1UL << aws->cmd_slot);
              aws->no_ncq = 1;
              need_reset = 1;
            } else {
              /* regular command; clear cmd bit and identify problem IORB */
              ai->ports[p].reg_cmds &= ~(1UL << aws->cmd_slot);
              if (aws->cmd_slot == ccs) {
                /* this is the non-NCQ command that failed */
                ddprintf("failing IORB: %Fp\n", iorb);
                problem_iorb = iorb;
              }
            }
            /* we can requeue all IORBs unconditionally (see function comment) */
            if (aws->retries++ < MAX_RETRIES) {
              iorb_requeue(iorb);

            } else {
              /* retry count exceeded; consider IORB aborted */
              iorb_seterr(iorb, IOERR_CMD_ABORTED);
              iorb_queue_del(&ai->ports[p].iorb_queue, iorb);
              iorb_queue_add(&done_queue, iorb);
              if (iorb == problem_iorb) {
                /* no further analysis -- we're done with this one */
                problem_iorb = NULL;
              }
            }
          }
        }

        /* sanity check: issued command bitmaps should be 0 now */
        if (ai->ports[p].ncq_cmds != 0 || ai->ports[p].reg_cmds != 0) {
          dprintf("warning: commands issued not 0 (%08lx/%08lx); resetting...\n",
                  ai->ports[p].ncq_cmds, ai->ports[p].reg_cmds);
          need_reset = 1;
        }

        if (!need_reset) {
          if ((readl(port_mmio + PORT_TFDATA) & 0x88) != 0) {
            /* device is not in an idle state */
            need_reset = 1;
          }
        }

        /* restart/reset port */
        ai->busy = 1;
        spin_unlock(drv_lock);
        if (need_reset) {
          ahci_reset_port(ai, p, 1);
        } else {
          ahci_stop_port(ai, p);
          ahci_start_port(ai, p, 1);
        }
        spin_lock(drv_lock);
        ai->busy = 0;

        /* reset internal port status */
        ai->ports[p].ncq_cmds = 0;
        ai->ports[p].reg_cmds = 0;
        ai->ports[p].cmd_slot = 0;

        if (problem_iorb != NULL) {
          /* get details about the error that caused this IORB to fail */
          if (need_reset) {
            /* no way to retrieve error details after a reset */
            iorb_seterr(problem_iorb, IOERR_DEVICE_NONSPECIFIC);
            iorb_queue_del(&ai->ports[p].iorb_queue, problem_iorb);
            iorb_queue_add(&done_queue, problem_iorb);

          } else {
            /* get sense information */
            ADD_WORKSPACE _far *aws = add_workspace(problem_iorb);
            int d = iorb_unit_device(problem_iorb);
            int (*req_sense)(IORBH _far *, int) = (ai->ports[p].devs[d].atapi) ?
                                                   atapi_req_sense : ata_req_sense;

            aws->processing = 1;
            aws->queued_hw = 1;

            if (req_sense(problem_iorb, 0) == 0) {
              /* execute request sense on slot #0 before anything else comes along */
              ADD_StartTimerMS(&aws->timer, 5000, (PFN) timeout_callback,
                               problem_iorb, 0);
              aws->cmd_slot = 0;
              ai->ports[p].reg_cmds = 1;
              writel(port_mmio + PORT_CMD_ISSUE, 1);
              readl(port_mmio); /* flush */

            } else {
              /* IORB is expected to contain the error code; just move to done queue */
              iorb_queue_del(&ai->ports[p].iorb_queue, problem_iorb);
              iorb_queue_add(&done_queue, problem_iorb);
            }
          }
        }
      }
    }
  }

  spin_unlock(drv_lock);

  /* call notification routine on all IORBs which have completed */
  for (iorb = done_queue.root; iorb != NULL; iorb = next) {
    next = iorb->pNxtIORB;

    spin_lock(drv_lock);
    aws_free(add_workspace(iorb));
    spin_unlock(drv_lock);

    iorb_complete(iorb);
  }

  /* restart engine to resume IORB processing */
  spin_lock(drv_lock);
  trigger_engine();
  spin_unlock(drv_lock);

  dprintf("restart_ctxhook() completed\n");

  /* Check whether we have to rearm ourselves because some adapters were busy
   * when we wanted to restart ports on them.
   */
  if (rearm_ctx_hook) {
    msleep(250);
    DevHelp_ArmCtxHook(0, restart_ctxhook_h);
  }
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
void reset_ctxhook(ULONG parm)
{
  IORB_QUEUE done_queue;
  AD_INFO *ai;
  IORBH _far *iorb;
  IORBH _far *next = NULL;
  int rearm_ctx_hook = 0;
  int a;
  int p;

  dprintf("reset_ctxhook() started\n");
  memset(&done_queue, 0x00, sizeof(done_queue));

  spin_lock(drv_lock);

  if (th_reset_watchdog != 0) {
    /* watchdog timer still active -- just reset it */
    ADD_CancelTimer(th_reset_watchdog);
    th_reset_watchdog = 0;
  }

  /* add ports of active IORBs from the abort queue to ports_to_reset[] */
  for (iorb = abort_queue.root; iorb != NULL; iorb = next) {
    next = iorb->pNxtIORB;
    a = iorb_unit_adapter(iorb);
    p = iorb_unit_port(iorb);
    ai = ad_infos + a;

    if (ai->busy) {
      /* this adapter is busy; leave it alone for now */
      rearm_ctx_hook = 1;
      continue;
    }

    /* move IORB to the local 'done' queue */
    iorb_queue_del(&abort_queue, iorb);
    iorb_queue_add(&done_queue, iorb);

    /* reset port if the IORB has already been queued to hardware */
    if (add_workspace(iorb)->queued_hw) {
      /* prepare port reset */
      ports_to_reset[a] |= (1UL << p);
    }
  }

  /* reset all ports in 'ports_to_reset[]' */
  for (a = 0; a < ad_info_cnt; a++) {
    ai = ad_infos + a;

    if (ai->busy) {
      /* this adapter is busy; leave it alone for now */
      rearm_ctx_hook = 1;
      continue;
    }

    for (p = 0; p <= ai->port_max; p++) {
      if (ports_to_reset[a] & (1UL << p)) {
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
        for (iorb = ai->ports[p].iorb_queue.root; iorb != NULL; iorb = next) {
          ADD_WORKSPACE _far *aws = add_workspace(iorb);
          next = iorb->pNxtIORB;

          if (aws->queued_hw) {
            /* this IORB had already been queued to HW when we reset the port */
            if (aws->idempotent && aws->retries++ < MAX_RETRIES) {
              /* we can retry this IORB */
              iorb_requeue(iorb);

            } else {
              /* we cannot retry this IORB; consider it aborted */
              iorb->ErrorCode = IOERR_CMD_ABORTED;
              iorb_queue_del(&ai->ports[p].iorb_queue, iorb);
              iorb_queue_add(&done_queue, iorb);
            }
          }
        }
      }
    }
  }

  spin_unlock(drv_lock);

  /* complete all aborted IORBs */
  for (iorb = done_queue.root; iorb != NULL; iorb = next) {
    next = iorb->pNxtIORB;

    spin_lock(drv_lock);
    aws_free(add_workspace(iorb));
    spin_unlock(drv_lock);

    iorb->Status |= IORB_ERROR;
    iorb_complete(iorb);
  }

  /* restart engine to resume IORB processing */
  spin_lock(drv_lock);
  trigger_engine();
  spin_unlock(drv_lock);

  dprintf("reset_ctxhook() completed\n");

  /* Check whether we have to rearm ourselves because some adapters were busy
   * when we wanted to reset ports on them.
   */
  if (rearm_ctx_hook) {
    msleep(250);
    DevHelp_ArmCtxHook(0, reset_ctxhook_h);
  }
}

/******************************************************************************
 * IORB Engine context hook. This hook is executed if trigger_engine() came
 * to the conclusion that some of the IORBs keep bouncing, most likely due to
 * some condition on the adapter such as being busy. It could also be a very
 * busy system. Either way, this requires some task-time help.
 */
void engine_ctxhook(ULONG parm)
{
  int iorbs_sent;
  int i;

  dprintf("engine_ctxhook() started\n");
  if (resume_sleep_flag) {
    msleep(resume_sleep_flag);
    resume_sleep_flag = 0;
  }

  spin_lock(drv_lock);
  for (i = 0; i < 10; i++) {
    if ((iorbs_sent = trigger_engine_1()) == 0) {
      break;
    }
  }
  spin_unlock(drv_lock);

  dprintf("engine_ctxhook() completed\n");

  if (iorbs_sent != 0) {
    /* need to rearm ourselves for another run */
    msleep(250);
    DevHelp_ArmCtxHook(0, engine_ctxhook_h);
  }
}

