/******************************************************************************
 * trace.c - code for our internal trace ring buffer
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

/* -------------------------- macros and constants ------------------------- */

/* ------------------------ typedefs and structures ------------------------ */

/* -------------------------- function prototypes -------------------------- */

/* ------------------------ global/static variables ------------------------ */

struct {
  u32 phys_addr;   /* physical address of allocated buffer */
  u8 _far *tbuf;   /* mapped address of trace buffer */
  u16 writep;      /* current write offset in buffer */
  u16 readp;       /* current read offset in buffer */
  u16 mask;        /* The mask for wrapping the buffer pointers */
} ahci_trace_buf;

/* ----------------------------- start of code ----------------------------- */

/******************************************************************************
 * initialize AHCI circular trace buffer
 *
 * NOTE: this func must be called during INIT time since it allocates
 *       a GDT selector for the trace ring buffer
 */
void trace_init(u32 ulBufSize)
{
  SEL sel = 0;

  if (ahci_trace_buf.phys_addr) return;

  /* initialize ring buffer logic */
  ahci_trace_buf.writep = 0;
  ahci_trace_buf.readp = 0;
  ahci_trace_buf.mask = ulBufSize - 1;

  if (ahci_trace_buf.phys_addr == 0) {
    /* allocate buffer */
    if (DevHelp_AllocPhys(ulBufSize, MEMTYPE_ABOVE_1M,
                         &(ahci_trace_buf.phys_addr))) {
      /* failed above 1MB, try below */
      if (DevHelp_AllocPhys(ulBufSize, MEMTYPE_BELOW_1M,
                            &(ahci_trace_buf.phys_addr))) {
        /* failed, too. Give up */
        ahci_trace_buf.phys_addr = 0;
        cprintf("%s warning: failed to allocate %dk trace buffer\n",
                drv_name, ulBufSize / 1024);
        return;
      }
    }

    /* allocate GDT selector and map our physical trace buffer to it */
    if (DevHelp_AllocGDTSelector(&sel, 1) ||
        DevHelp_PhysToGDTSelector(ahci_trace_buf.phys_addr,
                                  ulBufSize, sel)) {
      /* failed; free GDT selector and physical memory we allocated before */
      if (sel) {
        DevHelp_FreeGDTSelector(sel);
        sel = 0;
      }
      DevHelp_FreePhys(ahci_trace_buf.phys_addr);
      ahci_trace_buf.phys_addr = 0;
      return;
    }

    /* create ring buffer address */
    ahci_trace_buf.tbuf = (u8 _far *) ((u32) sel << 16);
  }
}

/******************************************************************************
 * cleanup trace buffer
 *
 * NOTE: this function is here for completeness; the trace buffer should not
 *       be deallocated and then reallocated.
 */
void trace_exit(void)
{
  /* free physical address */
  if (ahci_trace_buf.phys_addr) {
    DevHelp_FreePhys(ahci_trace_buf.phys_addr);
    ahci_trace_buf.phys_addr = 0;
  }

  /* free GDT selector */
  if (ahci_trace_buf.tbuf) {
    DevHelp_FreeGDTSelector((SEL) ((u32) (ahci_trace_buf.tbuf) >> 16));
    ahci_trace_buf.tbuf = NULL;
  }
}


/******************************************************************************
 * write a string to the circular trace buffer
 *
 * Note: This func wraps the buffer if necessary, so the caller does not
 *       need to call repeatedly until everything is written.
 *
 */
void trace_write(u8 _far *s, int len)
{
  //NOT USED USHORT awake_cnt;

  if (ahci_trace_buf.phys_addr == 0) {
    /* tracing not active */
    return;
  }

  while (len) {
    if ( !wrap_trace_buffer && (((ahci_trace_buf.writep+1) & ahci_trace_buf.mask) == ahci_trace_buf.readp) ) break; /* buffer is full */

    ahci_trace_buf.tbuf[ahci_trace_buf.writep] = *s++;
    ahci_trace_buf.writep++;
    ahci_trace_buf.writep &= ahci_trace_buf.mask;

    /* keep the latest full buffer of information */
    if (ahci_trace_buf.writep == ahci_trace_buf.readp)
      ahci_trace_buf.readp = (ahci_trace_buf.readp+1) & ahci_trace_buf.mask;

    len--;
  }

  /* wake up processes waiting for data from trace buffer */
  //NOT_USED DevHelp_ProcRun(ahci_trace_buf.phys_addr, &awake_cnt);

}

/******************************************************************************
 * read data from circular trace buffer
 * returns the number of bytes written to the caller's buffer
 *
 * NOTE: the caller is expected to call this func repeatedly
 *       (up to two times) until it returns 0
 */
u16 trace_read(u8 _far *buf, u16 cb_buf)
{
  u16 cb_read;

  if (ahci_trace_buf.phys_addr == NULL) return 0;

  for (cb_read = 0; cb_read < cb_buf && ( ahci_trace_buf.readp != ahci_trace_buf.writep ); cb_read++)
  {
    *buf++ = ahci_trace_buf.tbuf[ahci_trace_buf.readp];
    ahci_trace_buf.readp++;
    ahci_trace_buf.readp &= ahci_trace_buf.mask;
  }

  return cb_read;
}

/******************************************************************************
 * copy trace buffer content to character device reader (request block buffer)
 */
u16 trace_char_dev(RP_RWV _far *rwrb)
{
  u8 _far *to_buf;
  u16 cb_read = 0;
  u16 cb;
  USHORT mode = 0;

  spin_lock(com_lock);

  /* get pointer to caller's buffer */
  if (DevHelp_PhysToVirt(rwrb->XferAddr, rwrb->NumSectors, &to_buf, &mode)) {
    spin_unlock(com_lock);
    return (STATUS_DONE | STERR);
  }

  /* loop until caller's buffer is full or no more data in trace buffer */
  do {
    cb = trace_read(to_buf + cb_read, rwrb->NumSectors - cb_read);
    cb_read += cb;
  } while (cb > 0 && cb_read < rwrb->NumSectors);

  spin_unlock(com_lock);
  rwrb->NumSectors = cb_read;

  return(STDON);
}

/******************************************************************************
 * Create adapter/port/device list for user output.
 */
void build_user_info(int check)
{
  int a;
  int p;
  int d;

  if ( check && (ahci_trace_buf.readp != ahci_trace_buf.writep)) return;

  for (a = 0; a < ad_info_cnt; a++) {
    AD_INFO *ai = ad_infos + a;

    ntprintf("Adapter %d: PCI=%d:%d:%d ID=%04x:%04x %s %s irq=%d addr=0x%lx version=%lx\n", a,
      ai->bus, ai->dev_func>>3, ai->dev_func&7,
      ai->pci_vendor, ai->pci_device, vendor_from_id(ai->pci_vendor), ai->pci->chipname,
      ai->irq, ai->mmio_phys,
      ai->bios_config[HOST_VERSION / sizeof(u32)]);

    for (p = 0; p < ai->hw_ports; p++) {
      P_INFO *pi = &ai->ports[p];

      ntprintf("  Port %d:\n", p);

      for (d = 0; d <= pi->dev_max; d++) {
        if (!pi->devs[d].present) {
          ntprintf("    No drive present\n");
        } else {
          ntprintf("    Drive %d:", d);
          if (pi->devs[d].atapi) ntprintf(" atapi");
          if (pi->devs[d].removable) ntprintf(" removable");
          if (pi->devs[d].dev_info.Method != NULL) {
            ntprintf(" %ld cylinders, %d heads, %d sectors per track (%ldMB) (%s)",
              (u32)pi->devs[d].dev_info.Cylinders, pi->devs[d].dev_info.HeadsPerCylinder, pi->devs[d].dev_info.SectorsPerTrack,
              pi->devs[d].dev_info.TotalSectors/2048, pi->devs[d].dev_info.Method);
          } else ntprintf(" Drive present but no information available. Not queried by OS.");
          ntprintf("\n");
        } /* if */
      } /* for d */
    } /* for p */
  } /* for a */
}

