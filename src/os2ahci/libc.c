/******************************************************************************
 * libc.c - minimal subset of C runtime library for os2ahci
 *
 * Copyright (c) 2011 thi.guten Software Development
 * Copyright (c) 2011 Mensys B.V.
 * Portions copyright (c) 2013 David Azarewicz
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

#define MSG_REPLACEMENT_STRING 1178   /* empty message with a single %; used
                                       * for printing custom messages via
                                       * DevHelp_Save_Message() */

/* heap management constants */
#define HEAP_SIZE             8192
#define HEAP_UNIT              128
#define HEAP_UNIT_CNT         (HEAP_SIZE / HEAP_UNIT)

/* -------------------------- function prototypes -------------------------- */

static void long_to_asc(long val, char _far *buf, int base, int zero, int flen);

/* ------------------------ global/static variables ------------------------ */

/* debug COM port base address */
u16 com_base = 0;

static char  hex_digits[] = "0123456789abcdef";
static ULONG mem_lock;
ULONG com_lock;

/* message table for DosHelp_Save_Message() which prints the first string */
static MSGTABLE init_msgtbl = {
  MSG_REPLACEMENT_STRING,
  1,
  0
};

static struct {
  long Baud;
  u16 Data;
} BaudCodes[] = {
  115200, 0x0001,
  57600, 0x0002,
  38400, 0x0003,
  19200, 0x0006,
  9600, 0x000C,
  4800, 24,
  2400, 48,
  1200, 96,
  600, 192,
  300, 384,
  0, 0 /* end of list */
};

/* very small heap for dynamic memory management */
static u8 heap_buf[HEAP_SIZE];
static u8 heap_units[HEAP_UNIT_CNT];
static ULONG heap_phys_addr;

/* global info segment */
volatile PGINFOSEG gis;


/* ----------------------------- start of code ----------------------------- */

/******************************************************************************
 * Initialize libc components
 */
void init_libc(void)
{
  PSEL p;

  DevHelp_CreateSpinLock(&mem_lock);
  DevHelp_CreateSpinLock(&com_lock);

  DevHelp_VirtToPhys(heap_buf, &heap_phys_addr);

  /* get global info segment */
  if (DevHelp_GetDOSVar(DHGETDOSV_SYSINFOSEG, 0, (PPVOID) &p) == 0) {
    gis = (PGINFOSEG) ((u32) *p << 16);
  }

}

/******************************************************************************
 * Initialize COM port to 115200,n,8,1
 *
 * NOTE: Something is wrong with this code, or the init sequence, but we never
 *       got around to fixing it because it works fine on Virtualbox, and on
 *       physical machines we tend to have the kernel debugger running on the
 *       same port, thus KDB will set port parameters for us. This is going
 *       to be fixed eventually...
 */
void init_com(long BaudRate)
{
  int i;
  u16 RegData;

  if (com_base == 0) return; /* no com port in use */

  /* Find the baud code for the given baud rate */
  for (i = 0; BaudCodes[i].Baud; i++) if (BaudCodes[i].Baud == BaudRate) break;
  if (BaudCodes[i].Baud == 0) i = 0; /* default to 115200 */
  RegData = BaudCodes[i].Data;

  spin_lock(com_lock);

  __asm {
    mov     bx,RegData
    mov     dx,com_base     ; Base address
    cli
    add     dx,3            ; Line Control (+3)
    mov     al,10000000b    ; Set baud flag
    out     dx,al           ; for speed setting

    ; Set Baud
    dec     dx              ; High divisor address
    dec     dx              ; (+1)
    mov     al,bh           ; High divisor value
    out     dx,al           ; set it
    dec     dx              ; Low divisor address (+0)
    mov     al,bl           ; Low divisor value
    out     dx,al           ; set baud rate

    ; Set frame
    mov      al,00000011b   ; Set 8 bit, none, none
    add      dx,3           ; Line Control (+3)
    out      dx,al

    inc      dx
    mov      al,3
    out      dx,al          ; DTR & RTS to High
    sti
  }

  spin_unlock(com_lock);
}

/******************************************************************************
 * Print a formatted message into a string buffer. This is very basic,
 * supporting only strings and integers (16 and 32 bits (l), decimal (d)
 * and hex (x)). Formatting length modifiers are only supported with a single
 * digit -- 32-bit numbers don't need more than 9 characters -- and an
 * optional '0' in front.
 */
int vsprintf(char _far *buf, const char *fmt, va_list va)
{
  char _far *orig = buf;
  char _far *s;
  int lmod;
  int fptr;
  int zero;
  int flen;

  for (; *fmt != '\0'; fmt++) {
    switch (*fmt) {

    case '%':
      fmt++;
      zero = flen = 0;
      if (*fmt >= '0' && *fmt <= '9') {
        /* formatting length modifiers */
        zero = (*fmt == '0') ? 1 : 0;
        fmt += zero;
        if ((flen = *fmt - '0') >= 1 && flen <= 9) {
          fmt++;
        }
      }

      /* data type modifiers */
      lmod = (*fmt == 'l') ? 1 : 0;
      fptr = (*fmt == 'F') ? 1 : 0;
      fmt += lmod + fptr;

      switch (*fmt) {

      case 's':
        if (fptr) {
          char _far *p = va_arg(va, char _far *);
          s = (p == 0) ? "[null]" : p;
        } else {
          char *p = va_arg(va, char *);
          s = (p == 0) ? "[null]" : p;
        }
        while ((*buf = *(s++)) != '\0')
          buf++;
        break;

      case 'c':
        *(buf++) = (char) va_arg(va, int);
        break;

      case 'd':
        long_to_asc((lmod) ? va_arg(va, long) : va_arg(va, int), buf, 10, zero, flen);
        buf += strlen(buf);
        break;

      case 'x':
        long_to_asc((lmod) ? va_arg(va, u32) : va_arg(va, u16), buf, 16, zero, flen);
        buf += strlen(buf);
        break;

      case 'p':
        if (fptr || lmod) {
          u16 off = va_arg(va, u16);
          u16 seg = va_arg(va, u16);
          long_to_asc(seg, buf, 16, 1, 4);
          buf += strlen(buf);
          *(buf++) = ':';
          long_to_asc(off, buf, 16, 1, 4);
          buf += strlen(buf);
        } else {
          long_to_asc(va_arg(va, u16), buf, 16, 1, 4);
          buf += strlen(buf);
        }
        break;

      default:
        *(buf++) = *fmt;
        break;
      }
    break;

    case '\n':
      *(buf++) = '\r';
      *(buf++) = '\n';
      break;

    default:
      *(buf++) = *fmt;
      break;

    }
  }

  *buf = '\0';
  return((int) (buf - orig));
}

/*******************************************************************************
 * Print a formatted message into a string buffer. Relies on vsprintf()
 */
int sprintf(char _far *buf, const char *fmt, ...)
{
  va_list va;

  va_start(va, fmt);
  return(vsprintf(buf, fmt, va));
}

/******************************************************************************
 * Print messages to serial port
 *
 * NOTES: This function uses a 1K buffer for the resulting message. Thus,
 *        messages should never exceed 1024 bytes.
 */
void vprintf(int ts, const char *fmt, va_list va)
{
  static char buf[1024];
  char *s;
  int len = 0;

  spin_lock(com_lock);

  if (ts) {
    /* include timestamp */
    if (gis) {
      len = sprintf(buf, "[%ld] ", gis->msecs);
    } else {
      len = sprintf(buf, "[gis=0] ");
    }
  }

  len += vsprintf(buf + len, fmt, va);

  if (com_base == 0) {
    /* write debug message to trace buffer, not COM port */
    trace_write(buf, len);
    spin_unlock(com_lock);
    return;
  }

  /* write debug message to serial port */
  for (s = buf; *s != '\0'; s++) {

    /* inp() and outp() are redefined by the DDK in an incompatible
     * way (only words). Instead of messing around with those
     * definitions, it's safer and easier to put the whole thing
     * into an _asm block.
     *
     * The C equivalent would look like this:
     *
     *   while (!(inp(com_base + 5) & 0x20));
     *   outp(com_base, *s);
     */

    _asm {
      /* wait until COM transmitter is idle */
      mov  dx, com_base;
      add  dx, 5;
    transmitter_not_idle:
      in   al, dx;
      and  al, 0x20;
      jz   transmitter_not_idle;

      /* output character to be sent */
      mov  dx, com_base;
      mov  bx, s;
      mov  al, [bx];
      out  dx, al;
    };
  }

  spin_unlock(com_lock);
}

/******************************************************************************
 * Print messages to COM port
 */
void printf(const char *fmt, ...)
{
  va_list va;

  va_start(va, fmt);
  vprintf(1, fmt, va);
}

/******************************************************************************
 * Print messages to COM port with no time stamp
 */
void printf_nts(const char *fmt, ...)
{
  va_list va;

  va_start(va, fmt);
  vprintf(0, fmt, va);
}


/******************************************************************************
 * Print a message to the system console. This works only during device driver
 * initialization.
 *
 * NOTE: This function uses a 1K buffer for the resulting message. Thus,
 *       messages should never exceed 1024 bytes...
 */
void cprintf(const char *fmt, ...)
{
  static char buf[1024];
  va_list va;
  size_t len;

  va_start(va, fmt);
  vsprintf(buf, fmt, va);

  if (debug) {
    /* print the same message to COM1/trace as well */
    aprintf("%s", buf);
  }

  /* remove trailing CR/LF (DevHelp_Save_Message() will add it again) */
  if ((len = strlen(buf)) >= 2 && buf[len-1] == '\n' && buf[len-2] == '\r') {
    buf[len-2] = '\0';
  }

  init_msgtbl.MsgStrings[0] = buf;
  DevHelp_Save_Message((NPBYTE) &init_msgtbl);
}

/******************************************************************************
 * Print hex buffer to COM port.
 */
void phex(const void _far *p, int len, const char *fmt, ...)
{
  va_list va;
  const unsigned char _far *buf = p;
  int i;

  if (!debug) {
    return;
  }

  /* print header */
  va_start(va, fmt);
  vprintf(1, fmt, va);

  /* print hex block */
  while (len > 0) {
    ntprintf("%Fp ", buf);

    /* print hex block */
    for (i = 0; i < 16; i++) {
      if (i < len) {
        ntprintf("%c%02x", ((i == 8) ? '-' : ' '), buf[i]);
      } else {
        ntprintf("   ");
      }
    }

    /* print ASCII block */
    ntprintf("   ");
    for (i = 0; i < ((len > 16) ? 16 : len); i++) {
      ntprintf("%c", (buf[i] >= 32 && buf[i] < 128) ? buf[i] : '.');
    }
    ntprintf("\n");

    buf += 16;
    len -= 16;
  }
}

/******************************************************************************
 * Return length of zero-terminated string
 */
size_t strlen(const char _far *s)
{
  int len = 0;

  while (*(s++) != '\0') {
    len++;
  }
  return(len);
}

/******************************************************************************
 * Copy zero-terminated string
 */
char _far *strcpy(char _far *dst, const char _far *src)
{
  char _far *orig = dst;

  while ((*(dst++) = *(src++)) != '\0');
  return(orig);
}

/******************************************************************************
 * Compare blocks of memory
 */
int memcmp(void _far *p1, void _far *p2, size_t len)
{
  register char _far *s1 = p1;
  register char _far *s2 = p2;
  int n = 0;

  while (len > 0) {
    if ((n = *(s1++) - *(s2++)) != 0) {
      return(n);
    }
    len--;
  }
  return(0);
}

/******************************************************************************
 * Copy block from S/G list to virtual address or vice versa.
 */
void sg_memcpy(SCATGATENTRY _far *sg_list, USHORT sg_cnt, ULONG sg_off,
               void _far *buf, USHORT len, SG_MEMCPY_DIRECTION dir)
{
  USHORT mode_flag;
  USHORT i;
  USHORT l;
  ULONG phys_addr;
  ULONG pos = 0;
  char _far *p;

  /* walk through S/G list to find the elements involved in the operation */
  for (i = 0; i < sg_cnt && len > 0; i++) {
    if (pos <= sg_off && pos + sg_list[i].XferBufLen > sg_off) {

      /* this S/G element intersects with the block to be copied */
      phys_addr = sg_list[i].ppXferBuf + (sg_off - pos);
      if ((l = sg_list[i].XferBufLen - (sg_off - pos)) > len) {
        l = len;
      }

      if (DevHelp_PhysToVirt(phys_addr, l, (PVOID) &p, &mode_flag)) {
        panic("sg_memcpy(): DevHelp_PhysToVirt() failed");
      }
      if (dir == SG_TO_BUF) {
        memcpy(buf, p, l);
      } else {
        memcpy(p, buf, l);
      }
      sg_off += l;
      buf = (char _far *) buf + l;
      len -= l;
    }

    pos += sg_list[i].XferBufLen;
  }
}

/******************************************************************************
 * Convert a string to a long value using the specified base
 */
long strtol(const char _far *buf, const char _far * _far *ep, int base)
{
  register const char _far *s = buf;
  long val = 0;
  int negative = 0;

  /* skip leading whitespace */
  while (*s == ' ' || *s == '\t') {
    s++;
  }

  /* positive or negative */
  if (*s == '-') {
    negative = 1;
    s++;
  } else if (*s == '+') {
    s++;
  }

  /* convert string to long integer */
  for (;; s++) {
    int digit = (*s <= '9') ? (*s - '0') : (tolower(*s) - 'a' + 10);
    if (digit < 0 || digit >= base) {
      break;
    }
    val *= base;
    val += digit;
  }

  if (ep != NULL) {
    *ep = s;
  }
  if (negative) {
    val = -val;
  }
  return(val);
}

/******************************************************************************
 * Extremely simple and stupid implementation of malloc(). The heap is very
 * small, only 8K at the moment, and the memory blocks are managed using a
 * simple array of "number of heap units allocated", zero meaning this unit is
 * available. Each heap unit is currently 128 bytes.
 *
 * Dynamic memory is primarily used for things like ATA identify, ATAPI
 * sense buffers, etc. and should be freed as soon as possible, otherwise
 * we'll quickly run out of memory.
 */
void *malloc(size_t len)
{
  u16 units = (len + HEAP_UNIT - 1) / HEAP_UNIT;
  u16 i;
  u16 n;

  spin_lock(mem_lock);

  /* find a sequence of free heap units big enough for the requested length */
  for (i = 0; i < HEAP_UNIT_CNT; i++) {
    if (heap_units[i] == 0) {
      for (n = i; n < i + units && n < HEAP_UNIT_CNT; n++) {
        if (heap_units[n] != 0) {
          break;
        }
      }
      if (n == i + units) {
        /* found a chunk large enough; update 'heap_units[]' */
        for (; i < n; i++) {
          heap_units[i] = (u8) (n - i);
        }
        spin_unlock(mem_lock);
        return(heap_buf + (n - units) * HEAP_UNIT);
      }

      /* keep searching... */
      i = n;
    } else {
      /* skip occupied heap units */
      i += heap_units[i] - 1;
    }
  }

  /* out of memory */
  spin_unlock(mem_lock);
  dprintf("malloc(%d): out of memory\n", len);
  return(NULL);
}

/******************************************************************************
 * Free block of memory allocted by malloc().
 *
 * NOTE: This function is not reentrant, thus must be called with the driver-
 *       level spinlock held. The main reason for this design is that most
 *       functions that need dynamic memory are already holding the spinlock.
 */
void free(void *ptr)
{
  u8 *p = (u8 *) ptr;
  u16 first_unit;
  u16 units;
  u16 i;

  if (p < heap_buf || p >= heap_buf + sizeof(heap_buf) ||
      (u16) (p - heap_buf) % HEAP_UNIT != 0) {
    dprintf("free(0x%p): invalid pointer (heap_buf = 0x%p)\n",
            (u16) p, (u16) heap_buf);
    return;
  }

  /* clear unit allocation counters in heap_units[] */
  spin_lock(mem_lock);

  first_unit = (u16) (p - heap_buf) / HEAP_UNIT;
  units = heap_units[first_unit];
  for (i = first_unit; i < first_unit + units; i++) {
    heap_units[i] = 0;
  }

  spin_unlock(mem_lock);
}

/******************************************************************************
 * Return the physical address of a pointer inside the heap buffer. This is
 * necessary because DevHelp_VirtToPhys() can't be called at interrupt time
 * and we need physical addresses for heap objects when requeueing unaligned
 * IORBs inside ahci_intr -> trigger_engine.
 *
 * If the pointer is not a heap pointer, this function falls back to calling
 * DevHelp_VirtToPhys with all consequences (i.e. a trap when this is done
 * at interrupt time).
 */
ULONG virt_to_phys(void _far *ptr)
{
  if (ptr < heap_buf || ptr > heap_buf + sizeof(heap_buf)) {
    ULONG addr;

    if (DevHelp_VirtToPhys(ptr, &addr) != 0) {
      panic("virt_to_phys(): invalid pointer or execution mode");
    }
    return(addr);
  }

  return(heap_phys_addr + ((char _far *) ptr - (char _far *) heap_buf));
}

/******************************************************************************
 * Setup the millisecond timer. This is implemented by blocking (yielding the
 * CPU) until the system timer value indicates we're done. This function can
 * only be called at task time, or from a context hook.
 *
 * NOTE: The accuracy is limited by the OS/2 timer interrupt frequency which
 *       can lead to intervals up to 55ms (18.2 timer interrupts per second).
 */
void timer_init(TIMER far *pTimer, u32 Milliseconds)
{
  pTimer->Start = gis->msecs;
  pTimer->End = pTimer->Start + Milliseconds;
}

/******************************************************************************
 * Check the millisecond timer. Block if not done.
 */
int timer_check_and_block(TIMER far *pTimer)
{
  u32 current;

  current = gis->msecs;
  if (pTimer->Start <= pTimer->End) {
    if ((current >= pTimer->End) || (current < pTimer->Start)) return 1;
  } else {
    if ((current >= pTimer->End) && (current < pTimer->Start)) return 1;
  }
  DevHelp_ProcBlock((ULONG)&timer_check_and_block, 1, WAIT_IS_INTERRUPTABLE);
  return 0;
}

/******************************************************************************
 * Sleep specified number of milliseonds.
 */
void msleep(u32 millies)
{
  TIMER Timer;

  if (millies == 0) return;
  timer_init(&Timer, millies);
  while (!timer_check_and_block(&Timer));
}

/******************************************************************************
 * Halt processing by submitting an internal error. This is a last resort and
 * should only be called when the system state is corrupt.
 */
void panic(char *msg)
{
  DevHelp_InternalError(msg, strlen(msg));
}

/******************************************************************************
 * Disable interrupts. The reason for using a separate function for this is
 * that the presence of _asm statements will disable compiler optimizations.
 * In order to support nested calls, this function will return 0 if the
 * interrupts were already disabled or != 0, if not.
 *
 * NOTE: SMP systems should use spinlocks.
 */
int disable(void)
{
  int rc = 0;

  _asm {
    pushf
    pop   ax
    and   ax, 0x0200;  /* "interrupts enabled" bit */
    mov   rc, ax;
    cli
  }

  return(rc);
}

/******************************************************************************
 * Enable interrupts. The reason for using a separate function for this is
 * that the presence of _asm statements will disable compiler optimizations.
 *
 * NOTE: SMP systems should use spinlocks.
 */
void enable(void)
{
  _asm sti;
}

/******************************************************************************
 * Convert 'long' to ASCII with the specified base
 */
static void long_to_asc(long val, char _far *buf, int base, int zero, int flen)
{
  register unsigned long abs_val;
  char tmp[80];
  char _far *ptmp = tmp;
  char _far *s;

  if (base > 16) {
    sprintf(buf, "[EVAL]");
    return;
  }

  abs_val = (unsigned long) ((val < 0 && base <= 10) ? -val : val);
  tmp[sizeof(tmp) - 1] = '\0';

  for (s = ptmp + sizeof(tmp) - 2; s > ptmp; s--) {
    *s = hex_digits[abs_val % base];
    flen--;
    if ((abs_val /= base) == 0) {
      break;
    }
  }

  /* left-pad the resulting number with zeros or spaces up to 'flen' */
  while (flen > 0) {
    *(--s) = (zero) ? '0' : ' ';
    flen--;
  }

  /* prepend minus sign if val was negative and base is decimal or less */
  if (val < 0 && base <= 0) {
    *(--s) = '-';
    flen--;
  }

  strcpy(buf, s);
}

