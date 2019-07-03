/******************************************************************************
 * os2ahci.c - main file for os2ahci driver
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
#include "ioctl.h"
#include "version.h"
#include "devhdr.h"

/* -------------------------- macros and constants ------------------------- */

/* set two-dimensional array of port options */
#define set_port_option(opt, val)                         \
  if (adapter_index == -1) {                              \
    /* set option for all adapters and ports */           \
    memset(opt, val, sizeof(opt));                        \
  } else if (port_index == -1) {                          \
    /* set option for all ports on current adapter */     \
    memset(opt[adapter_index], val, sizeof(*opt));        \
  } else {                                                \
    /* set option for specific port */                    \
    opt[adapter_index][port_index] = val;                 \
  }

#define FLAG_KRNL_EXIT_ADD        0x1000
#define FLAG_KRNL_EXIT_REMOVE     0x2000

#define TYPE_KRNL_EXIT_NMI        0x0000  /* non masked interrupts */
#define TYPE_KRNL_EXIT_SFF        0x0001  /* system fatal faults */
#define TYPE_KRNL_EXIT_PROCDUMP   0x0002
#define TYPE_KRNL_EXIT_DYN        0x0003
#define TYPE_KRNL_EXIT_INT13      0x0004  /* enable int13 IO */

/* ------------------------ typedefs and structures ------------------------ */

/* -------------------------- function prototypes -------------------------- */

extern int SetPsdPutc(void);
static int add_unit_info(IORB_CONFIGURATION *iorb_conf, int dt_ai, int a, int p, int d, int scsi_id);

/* ------------------------ global/static variables ------------------------ */
int thorough_scan = 1; /* if != 0, perform thorough PCI scan */
int init_reset = 1;    /* if != 0, reset ports during init */
int force_write_cache; /* if != 0, force write cache */
int verbosity = 0;     /* default is quiet. 1=show sign on banner, >1=show adapter info during boot */
int use_mbr_test = 1;
long com_baud = 0;

HDRIVER rm_drvh;           /* resource manager driver handle */
USHORT add_handle;        /* driver handle (RegisterDeviceClass) */
char drv_name[] = "OS2AHCI"; /* driver name as string */

/* resource manager driver information structure */
static DRIVERSTRUCT rm_drvinfo =
{
  NULL, /* We cannot do Flat to Far16 conversion at compile time */
  NULL, /* so we put NULLs in all the Far16 fields and then fill */
  NULL, /* them in at run time                                   */
  DMAJOR,
  DMINOR,
  BLD_YEAR, BLD_MONTH, BLD_DAY,
  0,
  DRT_ADDDM,
  DRS_ADD,
  NULL
};

SpinLock_t drv_lock;          /* driver-level spinlock */
IORB_QUEUE driver_queue;      /* driver-level IORB queue */
AD_INFO ad_infos[MAX_AD];  /* adapter information list */
int ad_info_cnt;       /* number of entries in ad_infos[] */
u16 ad_ignore;         /* bitmap with adapter indexes to ignore */
int init_complete;     /* if != 0, initialization has completed */
int suspended;
int resume_sleep_flag;

/* apapter/port-specific options saved when parsing the command line */
u8 emulate_scsi[MAX_AD][AHCI_MAX_PORTS];
u8 enable_ncq[MAX_AD][AHCI_MAX_PORTS];
u8 link_speed[MAX_AD][AHCI_MAX_PORTS];
u8 link_power[MAX_AD][AHCI_MAX_PORTS];
u8 track_size[MAX_AD][AHCI_MAX_PORTS];
u8 port_ignore[MAX_AD][AHCI_MAX_PORTS];

char BldLevel[] = BLDLEVEL;

/* ----------------------------- start of code ----------------------------- */

/******************************************************************************
 * OS/2 device driver main strategy function.
 *
 * NOTE: this is also used as the IDC entry point. We expect an IOCTL request
 *       packet for IDC calls, so they can be handled by gen_ioctl.
 */
void StrategyHandler(REQPACKET *prp)
{
  u16 rc;

  switch (prp->bCommand)
  {
  case STRATEGY_BASEDEVINIT:
    rc = init_drv(prp);
    break;

  case STRATEGY_SHUTDOWN:
    rc = exit_drv(prp->save_restore.Function);
    break;

  case STRATEGY_GENIOCTL:
    rc = gen_ioctl(prp);
    break;

  case STRATEGY_OPEN:
    build_user_info();
    rc = RPDONE;
    break;

  case STRATEGY_READ:
    rc = char_dev_input(prp);
    break;

  case STRATEGY_SAVERESTORE:
    rc = sr_drv(prp->save_restore.Function);
    break;

  case STRATEGY_INITCOMPLETE:
  case STRATEGY_CLOSE:
  case STRATEGY_INPUTSTATUS:
  case STRATEGY_FLUSHINPUT:
    /* noop */
    rc = RPDONE;
    break;

  default:
    rc = RPDONE | RPERR_BADCOMMAND;
    break;
  }

  prp->usStatus = rc;
}

void IdcHandler(REQPACKET *prp)
{
  StrategyHandler(prp);
}

/******************************************************************************
 * Intialize the os2ahci driver. This includes command line parsing, scanning
 * the PCI bus for supported AHCI adapters, etc.
 */
USHORT init_drv(REQPACKET *req)
{
  static int init_drv_called;
  static int init_drv_failed;
  APIRET rmrc;
  const char *pszCmdLine, *cmd_line;
  int adapter_index = -1;
  int port_index = -1;
  int iInvertOption;
  int iStatus;

  if (init_drv_called)
  {
    /* This is the init call for the second (IBMS506$) character
     * device driver. If the main driver failed initialization, fail this
     * one as well.
     */
    return(RPDONE | ((init_drv_failed) ? RPERR_INITFAIL : 0));
  }
  D32g_DbgLevel = 0;
  init_drv_called = 1;
  suspended = 0;
  resume_sleep_flag = 0;
  memset(ad_infos, 0, sizeof(ad_infos));
  memset(emulate_scsi, 1, sizeof(emulate_scsi)); /* set default enabled */
  UtSetDriverName("OS2AHCI$");
  Header.ulCaps |= DEV_ADAPTER_DD; /* DAZ This flag is not really needed. */

  /* create driver-level spinlock */
  KernAllocSpinLock(&drv_lock);

  /* register driver with resource manager */
  rm_drvinfo.DrvrName = drv_name;
  rm_drvinfo.DrvrDescript = "AHCI SATA Driver";
  rm_drvinfo.VendorName = DVENDOR;
  if ((rmrc = RMCreateDriver(&rm_drvinfo, &rm_drvh)) != RMRC_SUCCESS)
  {
    iprintf("%s: failed to register driver with resource manager (rc = %d)", drv_name, rmrc);
    goto init_fail;
  }

  pszCmdLine = cmd_line = req->init_in.szArgs;
  iStatus = 0;
  while (*pszCmdLine)
  {
    if (*pszCmdLine++ != '/') continue; /* Ignore anything that doesn't start with '/' */
    /* pszCmdLine now points to first char of argument */

    if ((iInvertOption = (*pszCmdLine == '!')) != 0) pszCmdLine++;

    if (ArgCmp(pszCmdLine, "B:"))
    {
      pszCmdLine += 2;
      com_baud = strtol(pszCmdLine, &pszCmdLine, 0);
      continue;
    }

    if (ArgCmp(pszCmdLine, "C:"))
    {
      pszCmdLine += 2;
      /* set COM port base address for debug messages */
      D32g_ComBase = strtol(pszCmdLine, &pszCmdLine, 0);
      if (D32g_ComBase == 1) D32g_ComBase = 0x3f8;
      if (D32g_ComBase == 2) D32g_ComBase = 0x2f8;
      continue;
    }

    if (ArgCmp(pszCmdLine, "D"))
    {
      pszCmdLine++;
      if (*pszCmdLine == ':')
      {
        pszCmdLine++;
        D32g_DbgLevel = strtol(pszCmdLine, &pszCmdLine, 0);
      }
      else D32g_DbgLevel++; /* increase debug level */
      continue;
    }

    if (ArgCmp(pszCmdLine, "G:"))
    {
      u16 usVendor;
      u16 usDevice;

      pszCmdLine += 2;
      /* add specfied PCI ID as a supported generic AHCI adapter  */
      usVendor = strtol(pszCmdLine, &pszCmdLine, 16);
      if (*pszCmdLine != ':') break;
      pszCmdLine++;
      usDevice = strtol(pszCmdLine, &pszCmdLine, 16);
      if (add_pci_id(usVendor, usDevice))
      {
        iprintf("%s: failed to add PCI ID %04x:%04x", drv_name, usVendor, usDevice);
        iStatus = 1;
      }
      thorough_scan = 1;
      continue;
    }

    if (ArgCmp(pszCmdLine, "T"))
    {
      pszCmdLine++;
      /* perform thorough PCI scan (i.e. look for individual supported PCI IDs) */
      thorough_scan = !iInvertOption;
      continue;
    }

    if (ArgCmp(pszCmdLine, "R"))
    {
      pszCmdLine++;
      /* reset ports during initialization */
      init_reset = !iInvertOption;
      continue;
    }

    if (ArgCmp(pszCmdLine, "F"))
    {
      pszCmdLine++;
      /* force write cache regardless of IORB flags */
      force_write_cache = 1;
      continue;
    }

    if (ArgCmp(pszCmdLine, "A:"))
    {
      pszCmdLine += 2;
      /* set adapter index for adapter and port-related options */
      adapter_index = strtol(pszCmdLine, &pszCmdLine, 0);
      if (adapter_index < 0 || adapter_index >= MAX_AD)
      {
        iprintf("%s: invalid adapter index (%d)", drv_name, adapter_index);
        iStatus = 1;
      }
      continue;
    }

    if (ArgCmp(pszCmdLine, "P:"))
    {
      pszCmdLine += 2;
      /* set port index for port-related options */
      port_index = strtol(pszCmdLine, &pszCmdLine, 0);
      if (port_index < 0 || port_index >= AHCI_MAX_PORTS)
      {
        iprintf("%s: invalid port index (%d)", drv_name, port_index);
        iStatus = 1;
      }
      continue;
    }

    if (ArgCmp(pszCmdLine, "I"))
    {
      pszCmdLine++;
      /* ignore current adapter index */
      if (adapter_index >= 0)
      {
        if (port_index >= 0) port_ignore[adapter_index][port_index] = !iInvertOption;
        else ad_ignore |= 1U << adapter_index;
      }
      continue;
    }

    if (ArgCmp(pszCmdLine, "S"))
    {
      pszCmdLine++;
      /* enable SCSI emulation for ATAPI devices */
      set_port_option(emulate_scsi, !iInvertOption);
      continue;
    }

    if (ArgCmp(pszCmdLine, "N"))
    {
      pszCmdLine++;
      /* enable NCQ */
      set_port_option(enable_ncq, !iInvertOption);
      continue;
    }

    if (ArgCmp(pszCmdLine, "LS:"))
    {
      int optval;

      pszCmdLine += 3;
      /* set link speed */
      optval = strtol(pszCmdLine, &pszCmdLine, 0);
      set_port_option(link_speed, optval);
      /* need to reset the port in order to establish link settings */
      init_reset = 1;
      continue;
    }

    if (ArgCmp(pszCmdLine, "LP:"))
    {
      int optval;

      pszCmdLine += 3;
      /* set power management */
      optval = strtol(pszCmdLine, &pszCmdLine, 0);
      set_port_option(link_power, optval);
      /* need to reset the port in order to establish link settings */
      init_reset = 1;
      continue;
    }

    if (ArgCmp(pszCmdLine, "4"))
    {
      pszCmdLine++;
      /* enable 4K sector geometry enhancement (track size = 56) */
      if (!iInvertOption) set_port_option(track_size, 56);
      continue;
    }

    if (ArgCmp(pszCmdLine, "U"))
    {
      pszCmdLine++;
      /* Specify to use the MBR test to ignore non-MBR disks.
       * Default is on.
       */
      use_mbr_test = !iInvertOption;
      continue;
    }

    if (ArgCmp(pszCmdLine, "V"))
    {
      pszCmdLine++;
      if (*pszCmdLine == ':')
      {
        pszCmdLine++;
        verbosity = strtol(pszCmdLine, &pszCmdLine, 0);
      }
      else verbosity++; /* increase verbosity level */
      continue;
    }

    if (ArgCmp(pszCmdLine, "W"))
    {
      pszCmdLine++;
      /* Specify to allow the trace buffer to wrap when full. */
      D32g_DbgBufWrap = !iInvertOption;
      continue;
    }

    iprintf("Unrecognized switch: %s", pszCmdLine-1);
    iStatus = 1; /* unrecognized argument */
  }

  if (iStatus) goto init_fail;

  if (com_baud) InitComPort(com_baud);

  dprintf(0,"BldLevel: %s\n", BldLevel);
  dprintf(0,"CmdLine: %s\n", cmd_line);
  /*
  if (sizeof(ADD_WORKSPACE) > ADD_WORKSPACE_SIZE)
  {
    dprintf(0,"ADD_WORKSPACE size is too big! %d>16\n", sizeof(ADD_WORKSPACE));
    goto init_fail;
  }
  */

  /* print initialization message */
  ciprintf("%s driver version %d.%02d", drv_name, DMAJOR, DMINOR);

  #ifdef TESTVER
  #include "testver.c"
  #endif

  /* scan PCI bus for supported devices */
  scan_pci_bus();

  if (ad_info_cnt > 0)
  {
    /* initialization succeeded and we found at least one AHCI adapter */

    if (Dev32Help_RegisterDeviceClass(drv_name, add_entry, 0, 1, &add_handle))
    {
      iprintf("%s: couldn't register device class", drv_name);
      goto init_fail;
    }

    Timer_InitTimer(TIMER_COUNT);

    /* allocate context hooks */
    KernAllocateContextHook(restart_ctxhook, 0, &restart_ctxhook_h);
    KernAllocateContextHook(reset_ctxhook, 0, &reset_ctxhook_h);
    KernAllocateContextHook(engine_ctxhook, 0, &engine_ctxhook_h);

    /* register kernel exit routine for trap dumps */
    Dev32Help_RegisterKrnlExit(shutdown_driver, FLAG_KRNL_EXIT_ADD, TYPE_KRNL_EXIT_INT13);

    return(RPDONE);
  }
  else
  {
    /* no adapters found */
    ciprintf("%s: No adapters found.", drv_name);
  }

init_fail:
  /* initialization failed; set segment sizes to 0 and return error */
  init_drv_failed = 1;

  if (rm_drvh != 0)
  {
    /* remove driver from resource manager */
    RMDestroyDriver(rm_drvh);
  }

  ciprintf("%s driver *not* installed", drv_name);
  return(RPDONE | RPERR_INITFAIL);
}

/******************************************************************************
 * Generic IOCTL via character device driver. IOCTLs are used to control the
 * driver operation and to execute native ATA and ATAPI (SCSI) commands from
 * ring 3 applications. On top of that, some predefined IOCTLs (e.g. SMART
 * commands for ATA disks) are implemented here.
 */
USHORT gen_ioctl(REQPACKET *ioctl)
{
  DPRINTF(2,"IOCTL 0x%x/0x%x\n", ioctl->ioctl.bCategory, ioctl->ioctl.bFunction);

  switch (ioctl->ioctl.bCategory)
  {
  case OS2AHCI_IOCTL_CATEGORY:
    switch (ioctl->ioctl.bFunction)
    {
    case OS2AHCI_IOCTL_GET_DEVLIST:
      return(ioctl_get_devlist(ioctl));

    case OS2AHCI_IOCTL_PASSTHROUGH:
      return(ioctl_passthrough(ioctl));
    }
    break;

  case DSKSP_CAT_GENERIC:
    return(ioctl_gen_dsk(ioctl));

  case DSKSP_CAT_SMART:
    return(ioctl_smart(ioctl));
  }

  return(RPDONE | RPERR_BADCOMMAND);
}

/******************************************************************************
 * Read from character device. If tracing is on (internal ring buffer trace),
 * we return data from the trace buffer; if not, we might return a device
 * dump similar to IBM1S506.ADD/DANIS506.ADD (TODO).
 */
USHORT char_dev_input(REQPACKET *pPacket)
{
  void *LinAdr;

  if (Dev32Help_PhysToLin(pPacket->io.ulAddress, pPacket->io.usCount, &LinAdr))
  {
    pPacket->io.usCount = 0;
    return RPDONE | RPERR_GENERAL;
  }

  pPacket->io.usCount = dCopyToUser(LinAdr, pPacket->io.usCount);

  return RPDONE;
}

/******************************************************************************
 * Device driver exit handler. This handler is called when OS/2 shuts down and
 * flushes the write caches of all attached devices. Since this is effectively
 * the same we do when suspending, we'll call out to the corresponding suspend
 * function.
 *
 * NOTE: Errors are ignored because there's no way we could stop the shutdown
 *       or do something about the error, unless retrying endlessly is
 *       considered an option.
 */
USHORT exit_drv(int func)
{
  DPRINTF(2,"exit_drv(%d) called\n", func);

  if (func == 0)
  {
    /* we're only interested in the second phase of the shutdown */
    return(RPDONE);
  }

  suspend();
  return(RPDONE);
}

/******************************************************************************
 * Device driver suspend/resume handler. This handler is called when ACPI is
 * executing a suspend or resume.
 */
USHORT sr_drv(int func)
{
  DPRINTF(2,"sr_drv(%d) called\n", func);

  if (func) resume();
  else suspend();

  return(RPDONE);
}

/******************************************************************************
 * ADD entry point. This is the main entry point for all ADD requests. Due to
 * the asynchronous nature of ADD drivers, this function primarily queues the
 * IORB(s) to the corresponding adapter or port queues, then triggers the
 * state machine to initiate processing queued IORBs.
 *
 * NOTE: In order to prevent race conditions or engine stalls, certain rules
 *       around locking, unlocking and IORB handling in general have been
 *       established. Refer to the comments in "trigger_engine()" for
 *       details.
 */
void add_entry(IORBH FAR16DATA *vFirstIorb)
{
  IORBH FAR16DATA *vIorb;
  IORBH FAR16DATA *vNext = FAR16NULL;

  spin_lock(drv_lock);

  for (vIorb=vFirstIorb; vIorb!=FAR16NULL; vIorb=vNext)
  {
    IORBH *pIorb = Far16ToFlat(vIorb);

    /* Queue this IORB. Queues primarily exist on port level but there are
     * some requests which affect the whole driver, most notably
     * IOCC_CONFIGURATION. In either case, adding the IORB to the driver or
     * port queue will change the links, thus we need to save the original
     * link in 'vNext'.
     */
    if (pIorb->RequestControl & IORB_CHAIN) vNext = pIorb->pNxtIORB;
    else vNext = (IORBH FAR16DATA *)0;

    pIorb->Status = 0;
    pIorb->ErrorCode = 0;
    memset(&pIorb->ADDWorkSpace, 0x00, sizeof(ADD_WORKSPACE));

    #ifdef DEBUG
    DumpIorb(pIorb); /* DAZ TESTING */
    #endif

    if (iorb_driver_level(pIorb))
    {
      /* driver-level IORB */
      pIorb->UnitHandle = 0;
      iorb_queue_add(&driver_queue, vIorb, pIorb);
    }
    else
    {
      /* port-level IORB */
      int a = iorb_unit_adapter(pIorb);
      int p = iorb_unit_port(pIorb);
      int d = iorb_unit_device(pIorb);

      if (a >= ad_info_cnt ||
          p > ad_infos[a].port_max ||
          d > ad_infos[a].ports[p].dev_max ||
          (ad_infos[a].port_map & (1UL << p)) == 0)
      {
        /* unit handle outside of the allowed range */
        dprintf(0,"warning: IORB for %d.%d.%d out of range\n", a, p, d);
        pIorb->Status = IORB_ERROR;
        pIorb->ErrorCode = IOERR_CMD_SYNTAX;
        iorb_complete(vIorb, pIorb);
        continue;
      }

      iorb_queue_add(&ad_infos[a].ports[p].iorb_queue, vIorb, pIorb);
    }
  }

  /* trigger state machine */
  trigger_engine();

  spin_unlock(drv_lock);
}

/******************************************************************************
 * Trigger IORB queue engine. This is a wrapper function for trigger_engine_1()
 * which will try to get all IORBs sent on their way a couple of times. If
 * there are still IORBs ready for processing after this, this function will
 * hand off to a context hook which will continue to trigger the engine until
 * all IORBs have been sent.
 *
 * NOTE: While initialization has not completed (or during suspend/resume
 *       operations), this function will loop indefinitely because we can't
 *       rely on interrupt handlers or context hooks and complex IORBs
 *       requiring multiple requeues would eventually hang and time out if
 *       we stopped triggering here.
 */
void trigger_engine(void)
{
  int i;

  for (i = 0; i < 3 || !init_complete; i++)
  {
    if (trigger_engine_1() == 0)
    {
      /* done -- all IORBs have been sent on their way */
      return;
    }
  }

  /* Something keeps bouncing; hand off to the engine context hook which will
   * keep trying in the background.
   */
  KernArmHook(engine_ctxhook_h, 0, 0);
}

/******************************************************************************
 * Trigger IORB queue engine in order to send commands in the driver/port IORB
 * queues to the AHCI hardware. This function will return the number of IORBs
 * sent. Keep in mind that IORBs might "bounce" if the adapter/port is not in
 * a state to accept the command, thus it might take quite a few calls to get
 * all IORBs on their way. This is why there's a wrapper function which tries
 * it a few times, then hands off to a context hook which will keep trying in
 * the background.
 *
 * IORBs might complete before send_iorb() has returned, at any time during
 * interrupt processing or on another CPU on SMP systems. IORB completion
 * means modifications to the corresponding IORB queue (the completed IORB
 * is removed from the queue) thus we need to protect the IORB queues from
 * race conditions. The safest approach short of keeping the driver-level
 * spinlock aquired permanently is to keep it throughout this function and
 * release it temporarily in send_iorb().
 *
 * This implies that the handler functions are fully responsible for aquiring
 * the driver-level spinlock when they need it, and for releasing it again.
 *
 * As a rule of thumb, get the driver-level spinlock whenever accessing
 * volatile variables (IORB queues, values in ad_info[], ...).
 *
 * Additional Notes:
 *
 * - This function is expected to be called with the spinlock aquired
 *
 * - Adapters can be flagged as 'busy' which means no new IORBs are sent (they
 *   just remain in the queue). This can be used to release the driver-level
 *   spinlock while making sure no new IORBs are going to hit the hardware.
 *   In order to prevent engine stalls, all handlers using this functionality
 *   need to invoke trigger_engine() after resetting the busy flag.
 *
 * - Driver-level IORBs are not synchronized by adapter-level 'busy' flags.
 *   However, the driver-level queue is worked "one entry at a time" which
 *   means that no new IORBs will be queued on the driver-level queue until
 *   the head element has completed processing. This means that driver-
 *   level IORB handlers don't need to protect against each other. But they
 *   they do need to keep in mind interference with port-level IORBs:
 *
 *   - Driver-level IORB handlers must obtain the spinlock and/or flag all
 *     adapters as 'busy' which are affected by the driver-level IORB
 *
 *   - Driver-level IORB handlers must not access the hardware of a
 *     particular adapter if it's flagged as 'busy' by another IORB.
 */
int trigger_engine_1(void)
{
  IORBH FAR16DATA *vIorb;
  IORBH *pIorb;
  IORBH FAR16DATA *vNext;
  int iorbs_sent = 0;
  int a;
  int p;

  iorbs_sent = 0;

  /* process driver-level IORBs */
  if ((vIorb = driver_queue.vRoot) != FAR16NULL)
  {
    pIorb = Far16ToFlat(vIorb);

    if (!add_workspace(pIorb)->processing)
    {
      send_iorb(vIorb, pIorb);
      iorbs_sent++;
    }
  }

  /* process port-level IORBs */
  for (a = 0; a < ad_info_cnt; a++)
  {
    AD_INFO *ai = ad_infos + a;
    if (ai->busy)
    {
      /* adapter is busy; don't process any IORBs */
      continue;
    }
    for (p = 0; p <= ai->port_max; p++)
    {
      /* send all queued IORBs on this port */
      vNext = FAR16NULL;
      for (vIorb = ai->ports[p].iorb_queue.vRoot; vIorb != FAR16NULL; vIorb = vNext)
      {
        pIorb = Far16ToFlat(vIorb);

        vNext = pIorb->pNxtIORB;
        if (!add_workspace(pIorb)->processing)
        {
          send_iorb(vIorb, pIorb);
          iorbs_sent++;
        }
      }
    }
  }

  return(iorbs_sent);
}

/******************************************************************************
 * Send a single IORB to the corresponding AHCI adapter/port. This is just a
 * switch board for calling the corresponding iocc_*() handler function.
 *
 * NOTE: This function is expected to be called with the driver-level spinlock
 *       aquired. It will release it before calling any of the handler
 *       functions and re-aquire it when done.
 */
void send_iorb(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  /* Mark IORB as "processing" before doing anything else. Once the IORB is
   * marked as "processing", we can release the spinlock because subsequent
   * invocations of trigger_engine() (e.g. at interrupt time) will ignore this
   * IORB.
   */
  add_workspace(pIorb)->processing = 1;
  spin_unlock(drv_lock);

  switch (pIorb->CommandCode)
  {
  case IOCC_CONFIGURATION:
    iocc_configuration(vIorb, pIorb);
    break;

  case IOCC_DEVICE_CONTROL:
    iocc_device_control(vIorb, pIorb);
    break;

  case IOCC_UNIT_CONTROL:
    iocc_unit_control(vIorb, pIorb);
    break;

  case IOCC_GEOMETRY:
    iocc_geometry(vIorb, pIorb);
    break;

  case IOCC_EXECUTE_IO:
    iocc_execute_io(vIorb, pIorb);
    break;

  case IOCC_UNIT_STATUS:
    iocc_unit_status(vIorb, pIorb);
    break;

  case IOCC_ADAPTER_PASSTHRU:
    iocc_adapter_passthru(vIorb, pIorb);
    break;

  default:
    /* unsupported call */
    iorb_seterr(pIorb, IOERR_CMD_NOT_SUPPORTED);
    iorb_done(vIorb, pIorb);
    break;
  }

  /* re-aquire spinlock before returning to trigger_engine() */
  spin_lock(drv_lock);
}

/******************************************************************************
 * Handle IOCC_CONFIGURATION requests.
 */
void iocc_configuration(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  int a;

  switch (pIorb->CommandModifier)
  {

  case IOCM_COMPLETE_INIT:
    /* Complete initialization. From now on, we won't have to restore the BIOS
     * configuration after each command and we're fully operational (i.e. will
     * use interrupts, timers and context hooks instead of polling).
     */
    if (!init_complete)
    {
      DPRINTF(1,"leaving initialization mode\n");
      for (a = 0; a < ad_info_cnt; a++)
      {
        lock_adapter(ad_infos + a);
        ahci_complete_init(ad_infos + a);
      }
      init_complete = 1;

      /* release all adapters */
      for (a = 0; a < ad_info_cnt; a++)
      {
        unlock_adapter(ad_infos + a);
      }
      DPRINTF(1,"leaving initialization mode 2\n");

      #ifdef LEGACY_APM
      /* register APM hook */
      apm_init();
      #endif
    }
    iorb_done(vIorb, pIorb);
    break;

  case IOCM_GET_DEVICE_TABLE:
    /* construct a device table */
    iocm_device_table(vIorb, pIorb);
    break;

  default:
    iorb_seterr(pIorb, IOERR_CMD_NOT_SUPPORTED);
    iorb_done(vIorb, pIorb);
    break;
  }
}

/******************************************************************************
 * Handle IOCC_DEVICE_CONTROL requests.
 */
void iocc_device_control(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  AD_INFO *ai = ad_infos + iorb_unit_adapter(pIorb);
  IORBH FAR16DATA *vPtr;
  IORBH FAR16DATA *vNext = FAR16NULL;
  int p = iorb_unit_port(pIorb);
  int d = iorb_unit_device(pIorb);

  switch (pIorb->CommandModifier)
  {
  case IOCM_ABORT:
    /* abort all pending commands on specified port and device */
    spin_lock(drv_lock);
    for (vPtr = ai->ports[p].iorb_queue.vRoot; vPtr != FAR16NULL; vPtr = vNext)
    {
      IORBH *pPtr = Far16ToFlat(vPtr);

      vNext = pPtr->pNxtIORB;
      /* move all matching IORBs to the abort queue */
      if (vPtr != vIorb && iorb_unit_device(pPtr) == d)
      {
        iorb_queue_del(&ai->ports[p].iorb_queue, vPtr);
        iorb_queue_add(&abort_queue, vPtr, pPtr);
        pPtr->ErrorCode = IOERR_CMD_ABORTED;
      }
    }
    spin_unlock(drv_lock);

    /* trigger reset context hook which will finish the abort processing */
    KernArmHook(reset_ctxhook_h, 0, 0);
    break;

  case IOCM_SUSPEND:
  case IOCM_RESUME:
  case IOCM_GET_QUEUE_STATUS:
    /* Suspend/resume operations allow access to the hardware for other
     * entities such as IBMIDECD.FLT. Since os2ahci implements both ATA
     * and ATAPI in the same driver, this won't be required.
     */
    iorb_seterr(pIorb, IOERR_CMD_NOT_SUPPORTED);
    break;

  case IOCM_LOCK_MEDIA:
  case IOCM_UNLOCK_MEDIA:
  case IOCM_EJECT_MEDIA:
    /* unit control commands to lock, unlock and eject media */
    /* will be supported later... */
    iorb_seterr(pIorb, IOERR_CMD_NOT_SUPPORTED);
    break;

  default:
    iorb_seterr(pIorb, IOERR_CMD_NOT_SUPPORTED);
    break;
  }

  iorb_done(vIorb, pIorb);
}

/******************************************************************************
 * Handle IOCC_UNIT_CONTROL requests.
 */
void iocc_unit_control(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  IORB_UNIT_CONTROL *pIorb_uc = (IORB_UNIT_CONTROL *)pIorb;
  int a = iorb_unit_adapter(pIorb);
  int p = iorb_unit_port(pIorb);
  int d = iorb_unit_device(pIorb);

  spin_lock(drv_lock);
  switch (pIorb->CommandModifier)
  {
  case IOCM_ALLOCATE_UNIT:
    /* allocate unit for exclusive access */
    if (ad_infos[a].ports[p].devs[d].allocated)
    {
      iorb_seterr(pIorb, IOERR_UNIT_ALLOCATED);
    }
    else
    {
      ad_infos[a].ports[p].devs[d].allocated = 1;
    }
    break;

  case IOCM_DEALLOCATE_UNIT:
    /* deallocate exclusive access to unit */
    if (!ad_infos[a].ports[p].devs[d].allocated)
    {
      iorb_seterr(pIorb, IOERR_UNIT_NOT_ALLOCATED);
    }
    else
    {
      ad_infos[a].ports[p].devs[d].allocated = 0;
    }
    break;

  case IOCM_CHANGE_UNITINFO:
    /* Change unit (device) information. One reason for this IOCM is the
     * interface for filter device drivers: a filter device driver can
     * either change existing UNITINFOs or permanently allocate units
     * and fabricate new [logical] units; the former is the reason why we
     * must store the pointer to the updated UNITNIFO for subsequent
     * IOCC_CONFIGURATION/IOCM_GET_DEVICE_TABLE calls.
     */
    if (!ad_infos[a].ports[p].devs[d].allocated)
    {
      iorb_seterr(pIorb, IOERR_UNIT_NOT_ALLOCATED);
      break;
    }
    ad_infos[a].ports[p].devs[d].unit_info = pIorb_uc->pUnitInfo;
    break;

  default:
    iorb_seterr(pIorb, IOERR_CMD_NOT_SUPPORTED);
    break;
  }

  spin_unlock(drv_lock);
  iorb_done(vIorb, pIorb);
}

/******************************************************************************
 * Scan all ports for AHCI devices and construct a DASD device table.
 *
 * NOTES: This function may be called multiple times. Only the first
 *        invocation will actually scan for devices; all subsequent calls will
 *        merely return the results of the initial scan, potentially augmented
 *        by modified unit infos after IOCC_CONFIGURATION/IOCM_CHANGE_UNITINFO
 *        requests.
 *
 *        In order to support applications that can't deal with ATAPI devices
 *        (i.e. need a SCSI adapter) os2ahci will optionally report ATAPI
 *        dvices as SCSI devices. The corresponding SCSI adapter doesn't
 *        really exist and is only reported here for the IOCM_GET_DEVICETABLE
 *        request. The units attached to this adapter will use the real HW
 *        unit IDs, thus we'll never receive a command specific to the
 *        emulated SCSI adapter and won't need to set up any sort of entity
 *        for it; the only purpose of the emulated SCSI adapter is to pass the
 *        bus type "AI_DEVBUS_SCSI_2" upstream, and the emulated units, of
 *        course. The emulated SCSI target IDs are allocated as follows:
 *
 *         0     the virtual adapter
 *         1..n  emulated devices; SCSI target ID increments sequentially
 */
void iocm_device_table(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  IORB_CONFIGURATION *pIorb_conf;
  DEVICETABLE FAR16DATA *vDt;
  DEVICETABLE *pDt;
  char *pPos;
  int scsi_units = 0;
  int scsi_id = 1;
  int rc;
  int dta;
  int a;
  int p;
  int d;

  pIorb_conf = (IORB_CONFIGURATION *)pIorb;
  vDt = pIorb_conf->pDeviceTable;
  pDt = Far16ToFlat(vDt);

  spin_lock(drv_lock);

  /* initialize device table header */
  pDt->ADDLevelMajor = ADD_LEVEL_MAJOR;
  pDt->ADDLevelMinor = ADD_LEVEL_MINOR;
  pDt->ADDHandle     = add_handle;
  pDt->TotalAdapters = ad_info_cnt + 1;

  /* set start of adapter and device information tables */
  pPos = (char*)&pDt->pAdapter[pDt->TotalAdapters];

  /* go through all adapters, including the virtual SCSI adapter */
  for (dta = 0; dta < pDt->TotalAdapters; dta++)
  {
    ADAPTERINFO *pPtr = (ADAPTERINFO *)pPos;

    /* sanity check for sufficient space in device table */
    if ((u32)(pPtr + 1) - (u32)pDt > pIorb_conf->DeviceTableLen)
    {
      dprintf(0,"error: device table provided by DASD too small\n");
      iorb_seterr(pIorb, IOERR_CMD_SW_RESOURCE);
      goto iocm_device_table_done;
    }

    pDt->pAdapter[dta] = MakeNear16PtrFromDiff(pIorb_conf->pDeviceTable, pDt, pPtr);

    //DPRINTF(2,"iocm_device_table: ptr=%x dta=%x pAdapter[dta]=%x pDeviceTable=%x\n",
    //  ptr, dta, dt->pAdapter[dta], iorb_conf->pDeviceTable);
    memset(pPtr, 0x00, sizeof(*pPtr));

    pPtr->AdapterIOAccess = AI_IOACCESS_BUS_MASTER;
    pPtr->AdapterHostBus  = AI_HOSTBUS_OTHER | AI_BUSWIDTH_32BIT;
    pPtr->AdapterFlags    = AF_16M | AF_HW_SCATGAT;
    pPtr->MaxHWSGList     = AHCI_MAX_SG / 2;   /* AHCI S/G elements are 22 bits */

    if (dta < ad_info_cnt)
    {
      /* this is a physical AHCI adapter */
      AD_INFO *ad_info = ad_infos + dta;

      pPtr->AdapterDevBus = AI_DEVBUS_ST506 | AI_DEVBUS_32BIT;
      snprintf(pPtr->AdapterName, sizeof(pPtr->AdapterName), "AHCI_%d", dta);

      if (!ad_info->port_scan_done)
      {
        /* first call; need to scan AHCI hardware for devices */
        if (ad_info->busy)
        {
          dprintf(0,"error: port scan requested while adapter was busy\n");
          iorb_seterr(pIorb, IOERR_CMD_SW_RESOURCE);
          goto iocm_device_table_done;
        }
        ad_info->busy = 1;
        spin_unlock(drv_lock);
        rc = ahci_scan_ports(ad_info);
        spin_lock(drv_lock);
        ad_info->busy = 0;

        if (rc != 0)
        {
          dprintf(0,"error: port scan failed on adapter #%d\n", dta);
          iorb_seterr(pIorb, IOERR_CMD_SW_RESOURCE);
          goto iocm_device_table_done;
        }
        ad_info->port_scan_done = 1;
      }

      /* insert physical (i.e. AHCI) devices into the device table */
      for (p = 0; p <= ad_info->port_max; p++)
      {
        for (d = 0; d <= ad_info->ports[p].dev_max; d++)
        {
          if (ad_info->ports[p].devs[d].present && !ad_info->ports[p].devs[d].ignored)
          {
            if (ad_info->ports[p].devs[d].atapi && emulate_scsi[dta][p])
            {
              /* report this unit as SCSI unit */
              scsi_units++;
              //continue;
            }
            if (add_unit_info(pIorb_conf, dta, dta, p, d, 0))
            {
              goto iocm_device_table_done;
            }
          }
        }
      }
    }
    else
    {
      /* this is the virtual SCSI adapter */
      if (scsi_units == 0)
      {
        /* not a single unit to be emulated via SCSI */
        pDt->TotalAdapters--;
        break;
      }

      /* set adapter name and bus type to mimic a SCSI controller */
      pPtr->AdapterDevBus = AI_DEVBUS_SCSI_2 | AI_DEVBUS_16BIT;
      snprintf(pPtr->AdapterName, sizeof(pPtr->AdapterName), "AHCI_SCSI_0");

      /* add all ATAPI units to be emulated by this virtual adaper */
      for (a = 0; a < ad_info_cnt; a++)
      {
        AD_INFO *ad_info = ad_infos + a;

        for (p = 0; p <= ad_info->port_max; p++)
        {
          for (d = 0; d <= ad_info->ports[p].dev_max; d++)
          {
            if (ad_info->ports[p].devs[d].present && !ad_info->ports[p].devs[d].ignored
                && ad_info->ports[p].devs[d].atapi && emulate_scsi[a][p])
            {
              if (add_unit_info(pIorb_conf, dta, a, p, d, scsi_id++))
              {
                goto iocm_device_table_done;
              }
            }
          }
        }
      }
    }

    /* calculate offset for next adapter */
    pPos = (char *)(pPtr->UnitInfo + pPtr->AdapterUnits);
  }

iocm_device_table_done:
  spin_unlock(drv_lock);
  iorb_done(vIorb, pIorb);
}

/******************************************************************************
 * Handle IOCC_GEOMETRY requests.
 */
void iocc_geometry(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  switch (pIorb->CommandModifier)
  {
  case IOCM_GET_MEDIA_GEOMETRY:
  case IOCM_GET_DEVICE_GEOMETRY:
    add_workspace(pIorb)->idempotent = 1;
    ahci_get_geometry(vIorb, pIorb);
    break;

  default:
    iorb_seterr(pIorb, IOERR_CMD_NOT_SUPPORTED);
    iorb_done(vIorb, pIorb);
  }
}

/******************************************************************************
 * Handle IOCC_EXECUTE_IO requests.
 */
void iocc_execute_io(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  switch (pIorb->CommandModifier)
  {
  case IOCM_READ:
    add_workspace(pIorb)->idempotent = 1;
    ahci_read(vIorb, pIorb);
    break;

  case IOCM_READ_VERIFY:
    add_workspace(pIorb)->idempotent = 1;
    ahci_verify(vIorb, pIorb);
    break;

  case IOCM_WRITE:
    add_workspace(pIorb)->idempotent = 1;
    ahci_write(vIorb, pIorb);
    break;

  case IOCM_WRITE_VERIFY:
    add_workspace(pIorb)->idempotent = 1;
    ahci_write(vIorb, pIorb);
    break;

  default:
    iorb_seterr(pIorb, IOERR_CMD_NOT_SUPPORTED);
    iorb_done(vIorb, pIorb);
  }
}

/******************************************************************************
 * Handle IOCC_UNIT_STATUS requests.
 */
void iocc_unit_status(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  switch (pIorb->CommandModifier)
  {
  case IOCM_GET_UNIT_STATUS:
    add_workspace(pIorb)->idempotent = 1;
    ahci_unit_ready(vIorb, pIorb);
    break;

  default:
    iorb_seterr(pIorb, IOERR_CMD_NOT_SUPPORTED);
    iorb_done(vIorb, pIorb);
  }
}

/******************************************************************************
 * Handle IOCC_ADAPTER_PASSTHROUGH requests.
 */
void iocc_adapter_passthru(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  switch (pIorb->CommandModifier)
  {
  case IOCM_EXECUTE_CDB:
    add_workspace(pIorb)->idempotent = 0;
    ahci_execute_cdb(vIorb, pIorb);
    break;

  case IOCM_EXECUTE_ATA:
    add_workspace(pIorb)->idempotent = 0;
    ahci_execute_ata(vIorb, pIorb);
    break;

  default:
    iorb_seterr(pIorb, IOERR_CMD_NOT_SUPPORTED);
    iorb_done(vIorb, pIorb);
  }
}

/******************************************************************************
 * Add an IORB to the specified queue. This function must be called with the
 * adapter-level spinlock aquired.
 */
void iorb_queue_add(IORB_QUEUE *queue, IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  if (iorb_priority(pIorb)
  {
    /* priority IORB; insert at first position */
    pIorb->pNxtIORB = queue->vRoot;
    queue->vRoot = vIorb;
  }
  else
  {
    /* append IORB to end of queue */
    pIorb->pNxtIORB = FAR16NULL;

    if (queue->vRoot == FAR16NULL)
    {
      queue->vRoot = vIorb;
    }
    else
    {
      ((IORBH *)Far16ToFlat(queue->vTail))->pNxtIORB = vIorb;
    }
    queue->vTail = vIorb;
  }

  #ifdef DEBUG
  if (D32g_DbgLevel)
  {
    /* determine queue type (local, driver, abort or port) and minimum debug
     * level; otherwise, queue debug prints can become really confusing.
     */
    char *queue_type;
    int min_debug = 7;

    if ((u32)queue >> 16 == (u32)&queue >> 16) /* DAZ this is bogus */
    {
      /* this queue is on the stack */
      queue_type = "local";
      min_debug = 8;
    }
    else if (queue == &driver_queue)
    {
      queue_type = "driver";
    }
    else if (queue == &abort_queue)
    {
      queue_type = "abort";
      min_debug = 8;
    }
    else
    {
      queue_type = "port";
    }

    DPRINTF(min_debug,"IORB %x queued (cmd=%d/%d queue=%x [%s], timeout=%d)\n",
           vIorb, pIorb->CommandCode, pIorb->CommandModifier, queue, queue_type,
           pIorb->Timeout);
  }
  #endif
}

/******************************************************************************
 * Remove an IORB from the specified queue. This function must be called with
 * the adapter-level spinlock aquired.
 */
int iorb_queue_del(IORB_QUEUE *queue, IORBH FAR16DATA *vIorb)
{
  IORBH FAR16DATA *_vIorb;
  IORBH FAR16DATA *_vPrev = FAR16NULL;
  int found = 0;

  for (_vIorb = queue->vRoot; _vIorb != FAR16NULL; )
  {
    IORBH *_pIorb = Far16ToFlat(_vIorb);
    if (_vIorb == vIorb)
    {
      /* found the IORB to be removed */
      if (_vPrev != FAR16NULL)
      {
        ((IORBH*)Far16ToFlat(_vPrev))->pNxtIORB = _pIorb->pNxtIORB;
      }
      else
      {
        queue->vRoot = _pIorb->pNxtIORB;
      }
      if (_vIorb == queue->vTail)
      {
        queue->vTail = _vPrev;
      }
      found = 1;
      break;
    }
    _vPrev = _vIorb;
    _vIorb = _pIorb->pNxtIORB;
  }

  #ifdef DEBUG
  if (found)
  {
    DPRINTF(8,"IORB %x removed (queue = %x)\n", vIorb, queue);
  }
  else
  {
    DPRINTF(2,"IORB %x not found in queue %x\n", vIorb, queue);
  }
  #endif

  return(!found);
}

/******************************************************************************
 * Set the error code in the specified IORB
 *
 * NOTE: This function does *not* call iorb_done(). It merely sets the IORB
 *       status to the specified error code.
 */
void iorb_seterr(IORBH *pIorb, USHORT error_code)
{
  pIorb->ErrorCode = error_code;
  pIorb->Status |= IORB_ERROR;
}

/******************************************************************************
 * Mark the specified IORB as done and notify the asynchronous post function,
 * if any. The IORB is also removed from the corresponding IORB queue.
 *
 * NOTES: This function does not clear the Status field; it merely adds the
 *        IORB_DONE flag.
 *
 *        This function is expected to be called *without* the corresponding
 *        driver-level drv_lock aquired. It will aquire the spinlock before
 *        updating the IORB queue and release it before notifying the upstream
 *        code in order to prevent deadlocks.
 *
 *        Due to this logic, this function is only good for simple task-time
 *        completions. Functions working on lists of IORBs (such as interrupt
 *        handlers or context hooks) should call iorb_complete() directly and
 *        implement their own logic for removing the IORB from the port queue.
 *        See abort_ctxhook() for an example.
 */
void iorb_done(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  int a = iorb_unit_adapter(pIorb);
  int p = iorb_unit_port(pIorb);

  /* remove IORB from corresponding queue */
  spin_lock(drv_lock);
  if (iorb_driver_level(pIorb))
  {
    iorb_queue_del(&driver_queue, vIorb);
  }
  else
  {
    iorb_queue_del(&ad_infos[a].ports[p].iorb_queue, vIorb);
  }
  aws_free(add_workspace(pIorb));
  spin_unlock(drv_lock);

  iorb_complete(vIorb, pIorb);
}

/******************************************************************************
 * Complete an IORB. This should be called without the adapter-level spinlock
 * to allow the IORB completion routine to perform whatever processing it
 * requires. This implies that the IORB should no longer be in any global
 * queue because the IORB completion routine may well reuse the IORB and send
 * the next request to us before even returning from this function.
 */
void iorb_complete(IORBH FAR16DATA *vIorb, IORBH *pIorb)
{
  pIorb->Status |= IORB_DONE;

  DPRINTF(7,"IORB %x complete status=0x%04x error=0x%04x\n",
          vIorb, pIorb->Status, pIorb->ErrorCode);

  if (pIorb->RequestControl & IORB_ASYNC_POST)
  {
    Dev32Help_CallFar16((PFNFAR16)pIorb->NotifyAddress, vIorb);
  }
}

/******************************************************************************
 * Requeue the specified IORB such that it will be sent downstream for
 * processing again. This includes freeing all resources currently allocated
 * (timer, buffer, ...) and resetting the flags to 0. The driver-level
 * spinlock must be aquired when calling this function.
 *
 * The following flags are preserved:
 *  - no_ncq
 */
void iorb_requeue(IORBH *pIorb)
{
  ADD_WORKSPACE *aws = add_workspace(pIorb);
  u16 no_ncq = aws->no_ncq;
  u16 unaligned = aws->unaligned;
  u16 retries = aws->retries;

  aws_free(aws);
  memset(aws, 0x00, sizeof(*aws));

  aws->no_ncq = no_ncq;
  aws->unaligned = unaligned;
  aws->retries = retries;
}

/******************************************************************************
 * Free resources in ADD workspace (timer, buffer, ...). This function should
 * be called with the spinlock held to prevent race conditions.
 */
void aws_free(ADD_WORKSPACE *aws)
{
  if (aws->timer != 0)
  {
    Timer_CancelTimer(aws->timer);
    aws->timer = 0;
  }

  if (aws->buf != NULL)
  {
    MemFree(aws->buf);
    aws->buf = NULL;
  }
}

/******************************************************************************
 * Lock the adapter, waiting for availability if necessary. This is expected
 * to be called at task/request time without the driver-level spinlock
 * aquired. Don't call at interrupt time.
 */
void lock_adapter(AD_INFO *ai)
{
  TIMER Timer;

  spin_lock(drv_lock);
  while (ai->busy)
  {
    spin_unlock(drv_lock);
    TimerInit(&Timer, 250);
    while (!TimerCheckAndBlock(&Timer));
    spin_lock(drv_lock);
  }
  ai->busy = 1;
  spin_unlock(drv_lock);
}

/******************************************************************************
 * Unlock adapter (i.e. reset busy flag)
 */
void unlock_adapter(AD_INFO *ai)
{
  ai->busy = 0;
}

/******************************************************************************
 * Timeout handler for I/O commands. Since timeout handling can involve
 * lengthy operations like port resets, the main code is located in a
 * separate function which is invoked via a context hook.
 */
void __syscall timeout_callback(ULONG timer_handle, ULONG p1)
{
  IORBH FAR16DATA *vIorb = (IORBH FAR16DATA *)CastULONGToFar16(p1);
  IORBH *pIorb = Far16ToFlat(vIorb);
  int a = iorb_unit_adapter(pIorb);
  int p = iorb_unit_port(pIorb);

  Timer_CancelTimer(timer_handle);
  dprintf(0,"timeout for IORB %x\n", vIorb);

  /* Move the timed-out IORB to the abort queue. Since it's possible that the
   * IORB has completed after the timeout has expired but before we got to
   * this line of code, we'll check the return code of iorb_queue_del(): If it
   * returns an error, the IORB must have completed a few microseconds ago and
   * there is no timeout.
   */
  spin_lock(drv_lock);
  if (iorb_queue_del(&ad_infos[a].ports[p].iorb_queue, vIorb) == 0)
  {
    iorb_queue_add(&abort_queue, vIorb, pIorb);
    pIorb->ErrorCode = IOERR_ADAPTER_TIMEOUT;
  }
  spin_unlock(drv_lock);

  /* Trigger abort processing function. We don't really care whether this
   * succeeds because the only reason why it would fail should be multiple
   * calls to DevHelp_ArmCtxHook() before the context hook had a chance to
   * start executing, which leaves two scenarios:
   *
   *  - We succeded in arming the context hook. Fine.
   *
   *  - We armed the context hook a second time before it had a chance to
   *    start executing. In this case, the already scheduled context hook
   *    will process our IORB as well.
   */
  KernArmHook(reset_ctxhook_h, 0, 0);

  /* Set up a watchdog timer which calls the context hook manually in case
   * some kernel thread is looping around the IORB_COMPLETE status bit
   * without yielding the CPU (kernel threads don't preempt). This shouldn't
   * happen per design because kernel threads are supposed to yield but it
   * does in the early boot phase.
   */
  Timer_StartTimerMS(&th_reset_watchdog, 5000, reset_watchdog, 0);
}

/******************************************************************************
 * Reset handler watchdog. If a timeout occurs, a context hook is armed which
 * will execute as soon as a kernel thread yields the CPU. However, some
 * kernel components won't yield the CPU during the early boot phase and the
 * only way to kick some sense into those components is to run the context
 * hook right inside this timer callback. Not exactly pretty, especially
 * considering the fact that context hooks were implemented to prevent running
 * lengthy operations like a port reset at interrupt time, but without this
 * watchdog mechanism we run the risk of getting completely stalled by device
 * problems during the early boot phase.
 */
void __syscall reset_watchdog(ULONG timer_handle, ULONG p1)
{
  /* reset watchdog timer */
  Timer_CancelTimer(timer_handle);
  dprintf(0,"reset watchdog invoked\n");

  /* call context hook manually */
  reset_ctxhook(0);
}

/******************************************************************************
 * Add unit info to ADAPTERINFO array (IOCC_GET_DEVICE_TABLE requests). The
 * adapter info array in the device table, dt->pAdapter[], is expected to be
 * initialized for the specified index (dt_ai).
 *
 * Please note that the device table adapter index, dta, is not always equal
 * to the physical adapter index, a: if SCSI emulation has been activated, the
 * last reported adapter is a virtual SCSI adapter and the physical adapter
 * indexes for those units are, of course, different from the device table
 * index of the virtual SCSI adapter.
 */
static int add_unit_info(IORB_CONFIGURATION *pIorb_conf, int dta,
                         int a, int p, int d, int scsi_id)
{
  DEVICETABLE *pDt = Far16ToFlat(pIorb_conf->pDeviceTable);
  ADAPTERINFO *pPtr;
  UNITINFO *pUi;
  AD_INFO *ai = ad_infos + a;

  pPtr = (ADAPTERINFO *)MakeFlatFromNear16(pIorb_conf->pDeviceTable, pDt->pAdapter[dta]);
  //DPRINTF(2,"add_unit_info: ptr=%x dta=%x pAdapter[dta]=%x pDeviceTable=%x\n",
  //    ptr, dta, dt->pAdapter[dta], iorb_conf->pDeviceTable);

  pUi = &pPtr->UnitInfo[pPtr->AdapterUnits];

  if ((u32)(pUi + 1) - (u32)pDt > pIorb_conf->DeviceTableLen)
  {
    dprintf(0,"error: device table provided by DASD too small\n");
    iorb_seterr(&pIorb_conf->iorbh, IOERR_CMD_SW_RESOURCE);
    return(-1);
  }

  if (ai->ports[p].devs[d].unit_info == NULL)
  {
    /* provide original information about this device (unit) */
    memset(pUi, 0x00, sizeof(*pUi));
    pUi->AdapterIndex = dta;                 /* device table adapter index */
    pUi->UnitHandle   = iorb_unit(a, p, d);  /* physical adapter index */
    pUi->UnitIndex    = pPtr->AdapterUnits;
    pUi->UnitType     = ai->ports[p].devs[d].dev_type;
    pUi->QueuingCount = ai->ports[p].devs[d].ncq_max;
    if (ai->ports[p].devs[d].removable)
    {
      pUi->UnitFlags |= UF_REMOVABLE;
    }
    if (scsi_id > 0) {
      /* set fake SCSI ID for this unit */
      pUi->UnitSCSITargetID = scsi_id;
    }
  }
  else
  {
    /* copy updated device (unit) information (IOCM_CHANGE_UNITINFO) */
    memcpy(pUi, ai->ports[p].devs[d].unit_info, sizeof(*pUi));
  }

  pPtr->AdapterUnits++;
  return(0);
}

