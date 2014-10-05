/******************************************************************************
 * os2ahci.c - main file for os2ahci driver
 *
 * Copyright (c) 2011 thi.guten Software Development
 * Copyright (c) 2011 Mensys B.V.
 * Copyright (c) 2013 David Azarewicz
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

/* -------------------------- macros and constants ------------------------- */

/* parse integer command line parameter */
#define drv_parm_int(s, value, type, radix)                         \
  {                                                                 \
    char _far *_ep;                                                 \
    if ((s)[1] != ':') {                                            \
      cprintf("%s: missing colon (:) after /%c\n", drv_name, *(s)); \
      goto init_fail;                                               \
    }                                                               \
    value = (type) strtol((s) + 2,                                  \
                          (const char _far* _far*) &_ep,            \
                          radix);                                   \
    s = _ep;                                                        \
  }

#define drv_parm_int_optional(s, value, type, radix) \
  { \
    char _far *_ep; \
    if ((s)[1] == ':') { \
      value = (type) strtol((s) + 2, (const char _far* _far*) &_ep, radix); \
      s = _ep; \
    } else { \
      value++; \
    } \
  }

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

/* constants for undefined kernel exit routine;
 * see register_krnl_exit() func */
#define DevHlp_RegisterKrnlExit   0x006f

#define FLAG_KRNL_EXIT_ADD        0x1000
#define FLAG_KRNL_EXIT_REMOVE     0x2000

#define TYPE_KRNL_EXIT_NMI        0x0000  /* non masked interrupts */
#define TYPE_KRNL_EXIT_SFF        0x0001  /* system fatal faults */
#define TYPE_KRNL_EXIT_PROCDUMP   0x0002
#define TYPE_KRNL_EXIT_DYN        0x0003
#define TYPE_KRNL_EXIT_INT13      0x0004  /* enable int13 IO */

/* ------------------------ typedefs and structures ------------------------ */

/* -------------------------- function prototypes -------------------------- */

void _cdecl  small_code_        (void);

static int   add_unit_info      (IORB_CONFIGURATION _far *iorb_conf, int dt_ai,
                                 int a, int p, int d, int scsi_id);

static void  register_krnl_exit (void);

/* ------------------------ global/static variables ------------------------ */

int             debug = 0;         /* if > 0, print debug messages to COM1 */
int             thorough_scan = 1; /* if != 0, perform thorough PCI scan */
int             init_reset = 1;    /* if != 0, reset ports during init */
int             force_write_cache; /* if != 0, force write cache */
int             verbosity = 0;     /* default is quiet. 1=show sign on banner, >1=show adapter info during boot */
int             use_lvm_info = 1;
int             wrap_trace_buffer = 0;
long            com_baud = 0;

PFN             Device_Help = 0;   /* pointer to device helper entry point */
ULONG           RMFlags = 0;       /* required by resource manager library */
PFN             RM_Help0 = NULL;   /* required by resource manager library */
PFN             RM_Help3 = NULL;   /* required by resource manager library */
HDRIVER         rm_drvh;           /* resource manager driver handle */
char            rm_drvname[80];    /* driver name as returned by RM */
USHORT          add_handle;        /* driver handle (RegisterDeviceClass) */
UCHAR           timer_pool[TIMER_POOL_SIZE]; /* timer pool */
char            drv_name[] = "OS2AHCI"; /* driver name as string */

/* resource manager driver information structure */
DRIVERSTRUCT rm_drvinfo = {
  drv_name,                        /* driver name */
  "AHCI SATA Driver",              /* driver description */
  DVENDOR,                         /* vendor name */
  CMVERSION_MAJOR,                 /* RM interface version major */
  CMVERSION_MINOR,                 /* RM interface version minor */
  BLD_YEAR, BLD_MONTH, BLD_DAY,    /* date */
  0,                               /* driver flags */
  DRT_ADDDM,                       /* driver type */
  DRS_ADD,                         /* driver sub type */
  NULL                             /* driver callback */
};

ULONG           drv_lock;          /* driver-level spinlock */
IORB_QUEUE      driver_queue;      /* driver-level IORB queue */
AD_INFO         ad_infos[MAX_AD];  /* adapter information list */
int             ad_info_cnt;       /* number of entries in ad_infos[] */
u16             ad_ignore;         /* bitmap with adapter indexes to ignore */
int             init_complete;     /* if != 0, initialization has completed */
int             suspended;
int             resume_sleep_flag;

/* apapter/port-specific options saved when parsing the command line */
u8              emulate_scsi[MAX_AD][AHCI_MAX_PORTS];
u8              enable_ncq[MAX_AD][AHCI_MAX_PORTS];
u8              link_speed[MAX_AD][AHCI_MAX_PORTS];
u8              link_power[MAX_AD][AHCI_MAX_PORTS];
u8              track_size[MAX_AD][AHCI_MAX_PORTS];
u8              port_ignore[MAX_AD][AHCI_MAX_PORTS];

static char     init_msg[] = "%s driver version %d.%02d\n";
static char     exit_msg[] = "%s driver *not* installed\n";
char BldLevel[] = BLDLEVEL;

/* ----------------------------- start of code ----------------------------- */

/******************************************************************************
 * OS/2 device driver main strategy function.
 *
 * NOTE: this is also used as the IDC entry point. We expect an IOCTL request
 *       packet for IDC calls, so they can be handled by gen_ioctl.
 */
USHORT _cdecl c_strat(RPH _far *req)
{
  u16 rc;

  switch (req->Cmd) {

  case CMDInitBase:
    rc = init_drv((RPINITIN _far *) req);
    break;

  case CMDShutdown:
    rc = exit_drv(((RPSAVERESTORE _far *) req)->FuncCode);
    break;

  case CMDGenIOCTL:
    rc = gen_ioctl((RP_GENIOCTL _far *) req);
    break;

  case CMDOpen:
    build_user_info(1);
    rc = STDON;
    break;

  case CMDINPUT:
    rc = char_dev_input((RP_RWV _far *) req);
    break;

  case CMDSaveRestore:
    rc = sr_drv(((RPSAVERESTORE _far *) req)->FuncCode);
    break;

  case CMDClose:
  case CMDInputS:
  case CMDInputF:
    /* noop */
    rc = STDON;
    break;

  default:
    rc = STDON | STATUS_ERR_UNKCMD;
    break;
  }

  return(rc);
}

/******************************************************************************
 * Intialize the os2ahci driver. This includes command line parsing, scanning
 * the PCI bus for supported AHCI adapters, etc.
 */
USHORT init_drv(RPINITIN _far *req)
{
  static int init_drv_called;
  static int init_drv_failed;
  RPINITOUT _far *rsp = (RPINITOUT _far *) req;
  DDD_PARM_LIST _far *ddd_pl = (DDD_PARM_LIST _far *) req->InitArgs;
  APIRET rmrc;
  char _far *cmd_line;
  char _far *s;
  int adapter_index = -1;
  int port_index = -1;
  int invert_option;
  int optval;
  u16 vendor;
  u16 device;

  if (init_drv_called) {
    /* This is the init call for the second (legacy IBMS506$) character
     * device driver. If the main driver failed initialization, fail this
     * one as well.
     */
    rsp->CodeEnd = (u16) end_of_code;
    rsp->DataEnd = (u16) &end_of_data;
    return(STDON | ((init_drv_failed) ? ERROR_I24_QUIET_INIT_FAIL : 0));
  }
  init_drv_called = 1;
  suspended = 0;
  resume_sleep_flag = 0;
  memset(ad_infos, 0, sizeof(ad_infos));
  memset(emulate_scsi, 1, sizeof(emulate_scsi)); /* set default enabled */

  /* set device helper entry point */
  Device_Help = req->DevHlpEP;

  /* create driver-level spinlock */
  DevHelp_CreateSpinLock(&drv_lock);

  /* initialize libc code */
  init_libc();

  /* register driver with resource manager */
  if ((rmrc = RMCreateDriver(&rm_drvinfo, &rm_drvh)) != RMRC_SUCCESS) {
    cprintf("%s: failed to register driver with resource manager (rc = %d)\n",
            drv_name, rmrc);
    goto init_fail;
  }

  /* parse command line parameters */
  cmd_line = (char _far *) ((u32) ddd_pl & 0xffff0000l) + ddd_pl->cmd_line_args;

  for (s = cmd_line; *s != 0; s++) {
    if (*s == '/') {
      if ((invert_option = (s[1] == '!')) != 0) {
        s++;
      }
      s++;
      switch (tolower(*s)) {

      case '\0':
        /* end of command line; can only happen if command line is incorrect */
        cprintf("%s: incomplete command line option\n", drv_name);
        goto init_fail;

      case 'b':
        drv_parm_int(s, com_baud, u32, 10);
        break;

      case 'c':
        /* set COM port base address for debug messages */
        drv_parm_int(s, com_base, u16, 16);
        if (com_base == 1) com_base = 0x3f8;
        if (com_base == 2) com_base = 0x2f8;
        break;

      case 'd':
        /* increase debug level */
        drv_parm_int_optional(s, debug, int, 10);
        break;

      case 'g':
        /* add specfied PCI ID as a supported generic AHCI adapter  */
        drv_parm_int(s, vendor, u16, 16);
        s--;
        drv_parm_int(s, device, u16, 16);
        if (add_pci_id(vendor, device)) {
          cprintf("%s: failed to add PCI ID %04x:%04x\n", drv_name, vendor, device);
          goto init_fail;
        }
        thorough_scan = 1;
        break;

      case 't':
        /* perform thorough PCI scan (i.e. look for individual supported PCI IDs) */
        thorough_scan = !invert_option;
        break;

      case 'r':
        /* reset ports during initialization */
        init_reset = !invert_option;
        break;

      case 'f':
        /* force write cache regardless of IORB flags */
        force_write_cache = 1;
        break;

      case 'a':
        /* set adapter index for adapter and port-related options */
        drv_parm_int(s, adapter_index, int, 10);
        if (adapter_index < 0 || adapter_index >= MAX_AD) {
          cprintf("%s: invalid adapter index (%d)\n", drv_name, adapter_index);
          goto init_fail;
        }
        break;

      case 'p':
        /* set port index for port-related options */
        drv_parm_int(s, port_index, int, 10);
        if (port_index < 0 || port_index >= AHCI_MAX_PORTS) {
          cprintf("%s: invalid port index (%d)\n", drv_name, port_index);
          goto init_fail;
        }
        break;

      case 'i':
        /* ignore current adapter index */
        if (adapter_index >= 0) {
          if (port_index >= 0) port_ignore[adapter_index][port_index] = !invert_option;
          else ad_ignore |= 1U << adapter_index;
        }
        break;

      case 's':
        /* enable SCSI emulation for ATAPI devices */
        set_port_option(emulate_scsi, !invert_option);
        break;

      case 'n':
        /* enable NCQ */
        set_port_option(enable_ncq, !invert_option);
        break;

      case 'l':
        /* set link speed or power savings */
        s++;
        switch (tolower(*s)) {
        case 's':
          /* set link speed */
          drv_parm_int(s, optval, int, 10);
          set_port_option(link_speed, optval);
          break;
        case 'p':
          /* set power management */
          drv_parm_int(s, optval, int, 10);
          set_port_option(link_power, optval);
          break;
        default:
          cprintf("%s: invalid link parameter (%c)\n", drv_name, *s);
          goto init_fail;
        }
        /* need to reset the port in order to establish link settings */
        init_reset = 1;
        break;

      case '4':
        /* enable 4K sector geometry enhancement (track size = 56) */
        if (!invert_option) {
          set_port_option(track_size, 56);
        }
        break;

      case 'z':
        /* Specify to not use the LVM information. There is no reason why anyone would
         * want to do this, but previous versions of this driver did not have LVM capability,
         * so this switch is here temporarily just in case.
         */
        use_lvm_info = !invert_option;
        break;

      case 'v':
        /* be verbose during boot */
        drv_parm_int_optional(s, verbosity, int, 10);
        break;

      case 'w':
        /* Specify to allow the trace buffer to wrap when full. */
        wrap_trace_buffer = !invert_option;
        break;

      case 'q':
        /* Temporarily output a non-fatal message to get anyone using this
         * undocumented switch to stop using it. This will be removed soon
         * and the error will become fatal.
         */
        cprintf("%s: unknown option: /%c\n", drv_name, *s);
        break;

      default:
        cprintf("%s: unknown option: /%c\n", drv_name, *s);
        goto init_fail;
      }
    }
  }

  if (com_baud) init_com(com_baud); /* initialize com port for debug output */

  /* initialize trace buffer if applicable */
  if (debug > 0 && com_base == 0) {
    /* debug is on, but COM port is off -> use our trace buffer */
    trace_init(AHCI_DEBUG_BUF_SIZE);
  } else {
    trace_init(AHCI_INFO_BUF_SIZE);
  }

  ntprintf("BldLevel: %s\n", BldLevel);
  ntprintf("CmdLine: %Fs\n", cmd_line);

  /* print initialization message */
  ciprintf(init_msg, drv_name, VERSION / 100, VERSION % 100);

  #ifdef TESTVER
  #include "testver.c"
  #endif

  /* scan PCI bus for supported devices */
  scan_pci_bus();

  if (ad_info_cnt > 0) {
    /* initialization succeeded and we found at least one AHCI adapter */
    ADD_InitTimer(timer_pool, sizeof(timer_pool));

    if (DevHelp_RegisterDeviceClass(drv_name, (PFN) add_entry, 0, 1, &add_handle)) {
      cprintf("%s: couldn't register device class\n", drv_name);
      goto init_fail;
    }

    /* allocate context hooks */
    if (DevHelp_AllocateCtxHook(mk_NPFN(restart_hook), &restart_ctxhook_h) != 0 ||
        DevHelp_AllocateCtxHook(mk_NPFN(reset_hook), &reset_ctxhook_h) != 0 ||
        DevHelp_AllocateCtxHook(mk_NPFN(engine_hook), &engine_ctxhook_h)) {
      cprintf("%s: failed to allocate task-time context hooks\n", drv_name);
      goto init_fail;
    }

    rsp->CodeEnd = (u16) end_of_code;
    rsp->DataEnd = (u16) &end_of_data;

    /* register kernel exit routine for trap dumps */
    register_krnl_exit();

    return(STDON);

  } else {
    /* no adapters found */
    ciprintf(" No adapters found.\n");
  }

init_fail:
  /* initialization failed; set segment sizes to 0 and return error */
  rsp->CodeEnd = 0;
  rsp->DataEnd = 0;
  init_drv_failed = 1;

  /* free context hooks */
  if (engine_ctxhook_h != 0)  DevHelp_FreeCtxHook(engine_ctxhook_h);
  if (reset_ctxhook_h != 0)   DevHelp_FreeCtxHook(reset_ctxhook_h);
  if (restart_ctxhook_h != 0) DevHelp_FreeCtxHook(restart_ctxhook_h);

  if (rm_drvh != 0) {
    /* remove driver from resource manager */
    RMDestroyDriver(rm_drvh);
  }

  ciprintf(exit_msg, drv_name);
  return(STDON | ERROR_I24_QUIET_INIT_FAIL);
}

/******************************************************************************
 * Generic IOCTL via character device driver. IOCTLs are used to control the
 * driver operation and to execute native ATA and ATAPI (SCSI) commands from
 * ring 3 applications. On top of that, some predefined IOCTLs (e.g. SMART
 * commands for ATA disks) are implemented here.
 */
USHORT gen_ioctl(RP_GENIOCTL _far *ioctl)
{
  dprintf("IOCTL 0x%x/0x%x\n", (u16) ioctl->Category, (u16) ioctl->Function);

  switch (ioctl->Category) {

  case OS2AHCI_IOCTL_CATEGORY:
    switch (ioctl->Function) {

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

  return(STDON | STATUS_ERR_UNKCMD);
}

/******************************************************************************
 * Read from character device. If tracing is on (internal ring buffer trace),
 * we return data from the trace buffer; if not, we might return a device
 * dump similar to IBM1S506.ADD/DANIS506.ADD (TODO).
 */
USHORT char_dev_input(RP_RWV _far *rwrb)
{
  return(trace_char_dev(rwrb));
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
  dprintf("exit_drv(%d) called\n", func);

  if (func == 0) {
    /* we're only interested in the second phase of the shutdown */
    return(STDON);
  }

  suspend();
  return(STDON);
}

/******************************************************************************
 * Device driver suspend/resume handler. This handler is called when ACPI is
 * executing a suspend or resume.
 */
USHORT sr_drv(int func)
{
  dprintf("sr_drv(%d) called\n", func);

  if (func) resume();
  else suspend();

  return(STDON);
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
void _cdecl _far _loadds add_entry(IORBH _far *first_iorb)
{
  IORBH _far *iorb;
  IORBH _far *next = NULL;

  spin_lock(drv_lock);

  for (iorb = first_iorb; iorb != NULL; iorb = next) {
    /* Queue this IORB. Queues primarily exist on port level but there are
     * some requests which affect the whole driver, most notably
     * IOCC_CONFIGURATION. In either case, adding the IORB to the driver or
     * port queue will change the links, thus we need to save the original
     * link in 'next'.
     */
    next = (iorb->RequestControl | IORB_CHAIN) ? iorb->pNxtIORB : 0;

    iorb->Status = 0;
    iorb->ErrorCode = 0;
    memset(&iorb->ADDWorkSpace, 0x00, sizeof(ADD_WORKSPACE));

    if (iorb_driver_level(iorb)) {
      /* driver-level IORB */
      iorb->UnitHandle = 0;
      iorb_queue_add(&driver_queue, iorb);

    } else {
      /* port-level IORB */
      int a = iorb_unit_adapter(iorb);
      int p = iorb_unit_port(iorb);
      int d = iorb_unit_device(iorb);

      if (a >= ad_info_cnt ||
          p > ad_infos[a].port_max ||
          d > ad_infos[a].ports[p].dev_max ||
          (ad_infos[a].port_map & (1UL << p)) == 0) {

        /* unit handle outside of the allowed range */
        dprintf("warning: IORB for %d.%d.%d out of range\n", a, p, d);
        iorb->Status = IORB_ERROR;
        iorb->ErrorCode = IOERR_CMD_SYNTAX;
        iorb_complete(iorb);
        continue;
      }

      iorb_queue_add(&ad_infos[a].ports[p].iorb_queue, iorb);
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

  for (i = 0; i < 3 || !init_complete; i++) {
    if (trigger_engine_1() == 0) {
      /* done -- all IORBs have been sent on their way */
      return;
    }
  }

  /* Something keeps bouncing; hand off to the engine context hook which will
   * keep trying in the background.
   */
  DevHelp_ArmCtxHook(0, engine_ctxhook_h);
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
  IORBH _far *iorb;
  IORBH _far *next;
  int iorbs_sent = 0;
  int a;
  int p;

  iorbs_sent = 0;

  /* process driver-level IORBs */
  if ((iorb = driver_queue.root) != NULL && !add_workspace(iorb)->processing) {
    send_iorb(iorb);
    iorbs_sent++;
  }

  /* process port-level IORBs */
  for (a = 0; a < ad_info_cnt; a++) {
    AD_INFO *ai = ad_infos + a;
    if (ai->busy) {
      /* adapter is busy; don't process any IORBs */
      continue;
    }
    for (p = 0; p <= ai->port_max; p++) {
      /* send all queued IORBs on this port */
      next = NULL;
      for (iorb = ai->ports[p].iorb_queue.root; iorb != NULL; iorb = next) {
        next = iorb->pNxtIORB;
        if (!add_workspace(iorb)->processing) {
          send_iorb(iorb);
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
void send_iorb(IORBH _far *iorb)
{
  /* Mark IORB as "processing" before doing anything else. Once the IORB is
   * marked as "processing", we can release the spinlock because subsequent
   * invocations of trigger_engine() (e.g. at interrupt time) will ignore this
   * IORB.
   */
  add_workspace(iorb)->processing = 1;
  spin_unlock(drv_lock);

  switch (iorb->CommandCode) {

  case IOCC_CONFIGURATION:
    iocc_configuration(iorb);
    break;

  case IOCC_DEVICE_CONTROL:
    iocc_device_control(iorb);
    break;

  case IOCC_UNIT_CONTROL:
    iocc_unit_control(iorb);
    break;

  case IOCC_GEOMETRY:
    iocc_geometry(iorb);
    break;

  case IOCC_EXECUTE_IO:
    iocc_execute_io(iorb);
    break;

  case IOCC_UNIT_STATUS:
    iocc_unit_status(iorb);
    break;

  case IOCC_ADAPTER_PASSTHRU:
    iocc_adapter_passthru(iorb);
    break;

  default:
    /* unsupported call */
    iorb_seterr(iorb, IOERR_CMD_NOT_SUPPORTED);
    iorb_done(iorb);
    break;
  }

  /* re-aquire spinlock before returning to trigger_engine() */
  spin_lock(drv_lock);
}

/******************************************************************************
 * Handle IOCC_CONFIGURATION requests.
 */
void iocc_configuration(IORBH _far *iorb)
{
  int a;

  switch (iorb->CommandModifier) {

  case IOCM_COMPLETE_INIT:
    /* Complete initialization. From now on, we won't have to restore the BIOS
     * configuration after each command and we're fully operational (i.e. will
     * use interrupts, timers and context hooks instead of polling).
     */
    if (!init_complete) {
      dprintf("leaving initialization mode\n");
      for (a = 0; a < ad_info_cnt; a++) {
        lock_adapter(ad_infos + a);
        ahci_complete_init(ad_infos + a);
      }
      init_complete = 1;

      /* DAZ turn off COM port output if on */
      //com_base = 0;

      /* release all adapters */
      for (a = 0; a < ad_info_cnt; a++) {
        unlock_adapter(ad_infos + a);
      }

      #ifdef LEGACY_APM
      /* register APM hook */
      apm_init();
      #endif

      build_user_info(0);
    }
    iorb_done(iorb);
    break;

  case IOCM_GET_DEVICE_TABLE:
    /* construct a device table */
    iocm_device_table(iorb);
    break;

  default:
    iorb_seterr(iorb, IOERR_CMD_NOT_SUPPORTED);
    iorb_done(iorb);
    break;
  }
}

/******************************************************************************
 * Handle IOCC_DEVICE_CONTROL requests.
 */
void iocc_device_control(IORBH _far *iorb)
{
  AD_INFO *ai = ad_infos + iorb_unit_adapter(iorb);
  IORBH _far *ptr;
  IORBH _far *next = NULL;
  int p = iorb_unit_port(iorb);
  int d = iorb_unit_device(iorb);

  switch (iorb->CommandModifier) {

  case IOCM_ABORT:
    /* abort all pending commands on specified port and device */
    spin_lock(drv_lock);
    for (ptr = ai->ports[p].iorb_queue.root; ptr != NULL; ptr = next) {
      next = ptr->pNxtIORB;
      /* move all matching IORBs to the abort queue */
      if (ptr != iorb && iorb_unit_device(ptr) == d) {
        iorb_queue_del(&ai->ports[p].iorb_queue, ptr);
        iorb_queue_add(&abort_queue, ptr);
        ptr->ErrorCode = IOERR_CMD_ABORTED;
      }
    }
    spin_unlock(drv_lock);

    /* trigger reset context hook which will finish the abort processing */
    DevHelp_ArmCtxHook(0, reset_ctxhook_h);
    break;

  case IOCM_SUSPEND:
  case IOCM_RESUME:
  case IOCM_GET_QUEUE_STATUS:
    /* Suspend/resume operations allow access to the hardware for other
     * entities such as IBMIDECD.FLT. Since os2ahci implements both ATA
     * and ATAPI in the same driver, this won't be required.
     */
    iorb_seterr(iorb, IOERR_CMD_NOT_SUPPORTED);
    break;

  case IOCM_LOCK_MEDIA:
  case IOCM_UNLOCK_MEDIA:
  case IOCM_EJECT_MEDIA:
    /* unit control commands to lock, unlock and eject media */
    /* will be supported later... */
    iorb_seterr(iorb, IOERR_CMD_NOT_SUPPORTED);
    break;

  default:
    iorb_seterr(iorb, IOERR_CMD_NOT_SUPPORTED);
    break;
  }

  iorb_done(iorb);
}

/******************************************************************************
 * Handle IOCC_UNIT_CONTROL requests.
 */
void iocc_unit_control(IORBH _far *iorb)
{
  IORB_UNIT_CONTROL _far *iorb_uc = (IORB_UNIT_CONTROL _far *) iorb;
  int a = iorb_unit_adapter(iorb);
  int p = iorb_unit_port(iorb);
  int d = iorb_unit_device(iorb);

  spin_lock(drv_lock);
  switch (iorb->CommandModifier) {

  case IOCM_ALLOCATE_UNIT:
    /* allocate unit for exclusive access */
    if (ad_infos[a].ports[p].devs[d].allocated) {
      iorb_seterr(iorb, IOERR_UNIT_ALLOCATED);
    } else {
      ad_infos[a].ports[p].devs[d].allocated = 1;
    }
    break;

  case IOCM_DEALLOCATE_UNIT:
    /* deallocate exclusive access to unit */
    if (!ad_infos[a].ports[p].devs[d].allocated) {
      iorb_seterr(iorb, IOERR_UNIT_NOT_ALLOCATED);
    } else {
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
    if (!ad_infos[a].ports[p].devs[d].allocated) {
      iorb_seterr(iorb, IOERR_UNIT_NOT_ALLOCATED);
      break;
    }
    ad_infos[a].ports[p].devs[d].unit_info = iorb_uc->pUnitInfo;
    break;

  default:
    iorb_seterr(iorb, IOERR_CMD_NOT_SUPPORTED);
    break;
  }

  spin_unlock(drv_lock);
  iorb_done(iorb);
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
void iocm_device_table(IORBH _far *iorb)
{
  IORB_CONFIGURATION _far *iorb_conf;
  DEVICETABLE _far *dt;
  char _far *pos;
  int scsi_units = 0;
  int scsi_id = 1;
  int rc;
  int dta;
  int a;
  int p;
  int d;

  iorb_conf = (IORB_CONFIGURATION _far *) iorb;
  dt = iorb_conf->pDeviceTable;

  spin_lock(drv_lock);

  /* initialize device table header */
  dt->ADDLevelMajor = ADD_LEVEL_MAJOR;
  dt->ADDLevelMinor = ADD_LEVEL_MINOR;
  dt->ADDHandle     = add_handle;
  dt->TotalAdapters = ad_info_cnt + 1;

  /* set start of adapter and device information tables */
  pos = (char _far *) (dt->pAdapter + dt->TotalAdapters);

  /* go through all adapters, including the virtual SCSI adapter */
  for (dta = 0; dta < dt->TotalAdapters; dta++) {
    ADAPTERINFO _far *ptr = (ADAPTERINFO _far *) pos;

    /* sanity check for sufficient space in device table */
    if ((u32) (ptr + 1) - (u32) dt > iorb_conf->DeviceTableLen) {
      dprintf("error: device table provided by DASD too small\n");
      iorb_seterr(iorb, IOERR_CMD_SW_RESOURCE);
      goto iocm_device_table_done;
    }

    dt->pAdapter[dta] = (ADAPTERINFO _near *) ((u32) ptr & 0xffff);
    memset(ptr, 0x00, sizeof(*ptr));

    ptr->AdapterIOAccess = AI_IOACCESS_BUS_MASTER;
    ptr->AdapterHostBus  = AI_HOSTBUS_OTHER | AI_BUSWIDTH_32BIT;
    ptr->AdapterFlags    = AF_16M | AF_HW_SCATGAT;
    ptr->MaxHWSGList     = AHCI_MAX_SG / 2;   /* AHCI S/G elements are 22 bits */

    if (dta < ad_info_cnt) {
      /* this is a physical AHCI adapter */
      AD_INFO *ad_info = ad_infos + dta;

      ptr->AdapterDevBus = AI_DEVBUS_ST506 | AI_DEVBUS_32BIT;
      sprintf(ptr->AdapterName, "AHCI_%d", dta);

      if (!ad_info->port_scan_done) {
        /* first call; need to scan AHCI hardware for devices */
        if (ad_info->busy) {
          dprintf("error: port scan requested while adapter was busy\n");
          iorb_seterr(iorb, IOERR_CMD_SW_RESOURCE);
          goto iocm_device_table_done;
        }
        ad_info->busy = 1;
        spin_unlock(drv_lock);
        rc = ahci_scan_ports(ad_info);
        spin_lock(drv_lock);
        ad_info->busy = 0;

        if (rc != 0) {
          dprintf("error: port scan failed on adapter #%d\n", dta);
          iorb_seterr(iorb, IOERR_CMD_SW_RESOURCE);
          goto iocm_device_table_done;
        }
        ad_info->port_scan_done = 1;
      }

      /* insert physical (i.e. AHCI) devices into the device table */
      for (p = 0; p <= ad_info->port_max; p++) {
        for (d = 0; d <= ad_info->ports[p].dev_max; d++) {
          if (ad_info->ports[p].devs[d].present) {
            if (ad_info->ports[p].devs[d].atapi && emulate_scsi[dta][p]) {
              /* report this unit as SCSI unit */
              scsi_units++;
              //continue;
            }
            if (add_unit_info(iorb_conf, dta, dta, p, d, 0)) {
              goto iocm_device_table_done;
            }
          }
        }
      }

    } else {
      /* this is the virtual SCSI adapter */
      if (scsi_units == 0) {
        /* not a single unit to be emulated via SCSI */
        dt->TotalAdapters--;
        break;
      }

      /* set adapter name and bus type to mimic a SCSI controller */
      ptr->AdapterDevBus = AI_DEVBUS_SCSI_2 | AI_DEVBUS_16BIT;
      sprintf(ptr->AdapterName, "AHCI_SCSI_0");

      /* add all ATAPI units to be emulated by this virtual adaper */
      for (a = 0; a < ad_info_cnt; a++) {
        AD_INFO *ad_info = ad_infos + a;

        for (p = 0; p <= ad_info->port_max; p++) {
          for (d = 0; d <= ad_info->ports[p].dev_max; d++) {
            if (ad_info->ports[p].devs[d].present && ad_info->ports[p].devs[d].atapi && emulate_scsi[a][p]) {
              if (add_unit_info(iorb_conf, dta, a, p, d, scsi_id++)) {
                goto iocm_device_table_done;
              }
            }
          }
        }
      }
    }

    /* calculate offset for next adapter */
    pos = (char _far *) (ptr->UnitInfo + ptr->AdapterUnits);
  }

iocm_device_table_done:
  spin_unlock(drv_lock);
  iorb_done(iorb);
}

/******************************************************************************
 * Handle IOCC_GEOMETRY requests.
 */
void iocc_geometry(IORBH _far *iorb)
{
  switch (iorb->CommandModifier) {

  case IOCM_GET_MEDIA_GEOMETRY:
  case IOCM_GET_DEVICE_GEOMETRY:
    add_workspace(iorb)->idempotent = 1;
    ahci_get_geometry(iorb);
    break;

  default:
    iorb_seterr(iorb, IOERR_CMD_NOT_SUPPORTED);
    iorb_done(iorb);
  }
}

/******************************************************************************
 * Handle IOCC_EXECUTE_IO requests.
 */
void iocc_execute_io(IORBH _far *iorb)
{
  switch (iorb->CommandModifier) {

  case IOCM_READ:
    add_workspace(iorb)->idempotent = 1;
    ahci_read(iorb);
    break;

  case IOCM_READ_VERIFY:
    add_workspace(iorb)->idempotent = 1;
    ahci_verify(iorb);
    break;

  case IOCM_WRITE:
    add_workspace(iorb)->idempotent = 1;
    ahci_write(iorb);
    break;

  case IOCM_WRITE_VERIFY:
    add_workspace(iorb)->idempotent = 1;
    ahci_write(iorb);
    break;

  default:
    iorb_seterr(iorb, IOERR_CMD_NOT_SUPPORTED);
    iorb_done(iorb);
  }
}

/******************************************************************************
 * Handle IOCC_UNIT_STATUS requests.
 */
void iocc_unit_status(IORBH _far *iorb)
{
  switch (iorb->CommandModifier) {

  case IOCM_GET_UNIT_STATUS:
    add_workspace(iorb)->idempotent = 1;
    ahci_unit_ready(iorb);
    break;

  default:
    iorb_seterr(iorb, IOERR_CMD_NOT_SUPPORTED);
    iorb_done(iorb);
  }
}

/******************************************************************************
 * Handle IOCC_ADAPTER_PASSTHROUGH requests.
 */
void iocc_adapter_passthru(IORBH _far *iorb)
{
  switch (iorb->CommandModifier) {

  case IOCM_EXECUTE_CDB:
    add_workspace(iorb)->idempotent = 0;
    ahci_execute_cdb(iorb);
    break;

  case IOCM_EXECUTE_ATA:
    add_workspace(iorb)->idempotent = 0;
    ahci_execute_ata(iorb);
    break;

  default:
    iorb_seterr(iorb, IOERR_CMD_NOT_SUPPORTED);
    iorb_done(iorb);
  }
}

/******************************************************************************
 * Add an IORB to the specified queue. This function must be called with the
 * adapter-level spinlock aquired.
 */
void iorb_queue_add(IORB_QUEUE _far *queue, IORBH _far *iorb)
{
  if (iorb_priority(iorb) {
    /* priority IORB; insert at first position */
    iorb->pNxtIORB = queue->root;
    queue->root = iorb;

  } else {
    /* append IORB to end of queue */
    iorb->pNxtIORB = NULL;

    if (queue->root == NULL) {
      queue->root = iorb;
    } else {
      queue->tail->pNxtIORB = iorb;
    }
    queue->tail = iorb;
  }

  if (debug) {
    /* determine queue type (local, driver, abort or port) and minimum debug
     * level; otherwise, queue debug prints can become really confusing.
     */
    char *queue_type;
    int min_debug = 1;

    if ((u32) queue >> 16 == (u32) (void _far *) &queue >> 16) {
      /* this queue is on the stack */
      queue_type = "local";
      min_debug = 2;

    } else if (queue == &driver_queue) {
      queue_type = "driver";

    } else if (queue == &abort_queue) {
      queue_type = "abort";
      min_debug = 2;

    } else {
      queue_type = "port";
    }

    if (debug > min_debug) {
      aprintf("IORB %Fp queued (cmd = %d/%d, queue = %Fp [%s], timeout = %ld)\n",
             iorb, iorb->CommandCode, iorb->CommandModifier, queue, queue_type,
             iorb->Timeout);
    }
  }
}

/******************************************************************************
 * Remove an IORB from the specified queue. This function must be called with
 * the adapter-level spinlock aquired.
 */
int iorb_queue_del(IORB_QUEUE _far *queue, IORBH _far *iorb)
{
  IORBH _far *_iorb;
  IORBH _far *_prev = NULL;
  int found = 0;

  for (_iorb = queue->root; _iorb != NULL; _iorb = _iorb->pNxtIORB) {
    if (_iorb == iorb) {
      /* found the IORB to be removed */
      if (_prev != NULL) {
        _prev->pNxtIORB = _iorb->pNxtIORB;
      } else {
        queue->root = _iorb->pNxtIORB;
      }
      if (_iorb == queue->tail) {
        queue->tail = _prev;
      }
      found = 1;
      break;
    }
    _prev = _iorb;
  }

  if (found) {
    ddprintf("IORB %Fp removed (queue = %Fp)\n", iorb, queue);
  } else {
    dprintf("IORB %Fp not found in queue %Fp\n", iorb, queue);
  }

  return(!found);
}

/******************************************************************************
 * Set the error code in the specified IORB
 *
 * NOTE: This function does *not* call iorb_done(). It merely sets the IORB
 *       status to the specified error code.
 */
void iorb_seterr(IORBH _far *iorb, USHORT error_code)
{
  iorb->ErrorCode = error_code;
  iorb->Status |= IORB_ERROR;
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
void iorb_done(IORBH _far *iorb)
{
  int a = iorb_unit_adapter(iorb);
  int p = iorb_unit_port(iorb);

  /* remove IORB from corresponding queue */
  spin_lock(drv_lock);
  if (iorb_driver_level(iorb)) {
    iorb_queue_del(&driver_queue, iorb);
  } else {
    iorb_queue_del(&ad_infos[a].ports[p].iorb_queue, iorb);
  }
  aws_free(add_workspace(iorb));
  spin_unlock(drv_lock);

  iorb_complete(iorb);
}

/******************************************************************************
 * Complete an IORB. This should be called without the adapter-level spinlock
 * to allow the IORB completion routine to perform whatever processing it
 * requires. This implies that the IORB should no longer be in any global
 * queue because the IORB completion routine may well reuse the IORB and send
 * the next request to us before even returning from this function.
 */
void iorb_complete(IORBH _far *iorb)
{
  iorb->Status |= IORB_DONE;

  ddprintf("IORB %Fp complete (status = 0x%04x, error = 0x%04x)\n",
          iorb, iorb->Status, iorb->ErrorCode);

  if (iorb->RequestControl & IORB_ASYNC_POST) {
    iorb->NotifyAddress(iorb);
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
void iorb_requeue(IORBH _far *iorb)
{
  ADD_WORKSPACE _far *aws = add_workspace(iorb);
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
void aws_free(ADD_WORKSPACE _far *aws)
{
  if (aws->timer != 0) {
    ADD_CancelTimer(aws->timer);
    aws->timer = 0;
  }

  if (aws->buf != NULL) {
    free(aws->buf);
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
  while (ai->busy) {
    spin_unlock(drv_lock);
    timer_init(&Timer, 250);
    while (!timer_check_and_block(&Timer));
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
void _cdecl _far timeout_callback(ULONG timer_handle, ULONG p1, ULONG p2)
{
  IORBH _far *iorb = (IORBH _far *) p1;
  int a = iorb_unit_adapter(iorb);
  int p = iorb_unit_port(iorb);

  ADD_CancelTimer(timer_handle);
  dprintf("timeout for IORB %Fp\n", iorb);

  /* Move the timed-out IORB to the abort queue. Since it's possible that the
   * IORB has completed after the timeout has expired but before we got to
   * this line of code, we'll check the return code of iorb_queue_del(): If it
   * returns an error, the IORB must have completed a few microseconds ago and
   * there is no timeout.
   */
  spin_lock(drv_lock);
  if (iorb_queue_del(&ad_infos[a].ports[p].iorb_queue, iorb) == 0) {
    iorb_queue_add(&abort_queue, iorb);
    iorb->ErrorCode = IOERR_ADAPTER_TIMEOUT;
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
  DevHelp_ArmCtxHook(0, reset_ctxhook_h);

  /* Set up a watchdog timer which calls the context hook manually in case
   * some kernel thread is looping around the IORB_COMPLETE status bit
   * without yielding the CPU (kernel threads don't preempt). This shouldn't
   * happen per design because kernel threads are supposed to yield but it
   * does in the early boot phase.
   */
  ADD_StartTimerMS(&th_reset_watchdog, 5000, (PFN) reset_watchdog, 0, 0);
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
void _cdecl _far reset_watchdog(ULONG timer_handle, ULONG p1, ULONG p2)
{
  /* reset watchdog timer */
  ADD_CancelTimer(timer_handle);
  dprintf("reset watchdog invoked\n");

  /* call context hook manually */
  reset_ctxhook(0);
}

/******************************************************************************
 * small_code_ - this dummy func resolves the undefined reference linker
 * error that occurrs when linking WATCOM objects with DDK's link.exe
 */
void _cdecl small_code_(void)
{
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
static int add_unit_info(IORB_CONFIGURATION _far *iorb_conf, int dta,
                         int a, int p, int d, int scsi_id)
{
  DEVICETABLE _far *dt = iorb_conf->pDeviceTable;
  ADAPTERINFO _far *ptr = (ADAPTERINFO _far *) (((u32) dt & 0xffff0000U) +
                                                 (u16) dt->pAdapter[dta]);
  UNITINFO _far *ui = ptr->UnitInfo + ptr->AdapterUnits;
  AD_INFO *ai = ad_infos + a;

  if ((u32) (ui + 1) - (u32) dt > iorb_conf->DeviceTableLen) {
    dprintf("error: device table provided by DASD too small\n");
    iorb_seterr(&iorb_conf->iorbh, IOERR_CMD_SW_RESOURCE);
    return(-1);
  }

  if (ai->ports[p].devs[d].unit_info == NULL) {
    /* provide original information about this device (unit) */
    memset(ui, 0x00, sizeof(*ui));
    ui->AdapterIndex = dta;                 /* device table adapter index */
    ui->UnitHandle   = iorb_unit(a, p, d);  /* physical adapter index */
    ui->UnitIndex    = ptr->AdapterUnits;
    ui->UnitType     = ai->ports[p].devs[d].dev_type;
    ui->QueuingCount = ai->ports[p].devs[d].ncq_max;;
    if (ai->ports[p].devs[d].removable) {
      ui->UnitFlags |= UF_REMOVABLE;
    }
    if (scsi_id > 0) {
      /* set fake SCSI ID for this unit */
      ui->UnitSCSITargetID = scsi_id;
    }
  } else {
    /* copy updated device (unit) information (IOCM_CHANGE_UNITINFO) */
    memcpy(ui, ai->ports[p].devs[d].unit_info, sizeof(*ui));
  }

  ptr->AdapterUnits++;
  return(0);
}

/*******************************************************************************
 * Register kernel exit handler for trap dumps. Our exit handler will be called
 * right before the kernel starts a dump; that's where we reset the controller
 * so it supports BIOS int13 I/O calls.
 */
static void register_krnl_exit(void)
{
  _asm {
    push ds
    push es
    push bx
    push si
    push di

    mov ax, FLAG_KRNL_EXIT_ADD
    mov cx, TYPE_KRNL_EXIT_INT13
    mov bx, SEG asm_krnl_exit
    mov si, OFFSET asm_krnl_exit
    mov dl, DevHlp_RegisterKrnlExit

    call dword ptr [Device_Help]

    pop  di
    pop  si
    pop  bx
    pop  es
    pop  ds
  }

  dprintf("Registered kernel exit routine for INT13 mode\n");
}

