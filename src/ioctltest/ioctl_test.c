#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <memory.h>

#define INCL_DOS
#include <os2.h>
#include "../ioctl.h"

void phex(const void *p, int len);

int main(int arhc, char *argv[])
{
  OS2AHCI_DEVLIST *dl;
  OS2AHCI_PASSTHROUGH pt;
  APIRET rc;
  HFILE fh;
  ULONG at;
  ULONG cnt;
  ULONG plen;
  ULONG dlen;
  char sense_buf[64];
  char id_buf[512];
  int i;
  int n;

  /* open OS2AHCI character device driver */
  rc = DosOpen("\\DEV\\OS2AHCI$", &fh, &at, 0, FILE_SYSTEM,
               OPEN_ACTION_OPEN_IF_EXISTS, OPEN_SHARE_DENYNONE |
               OPEN_FLAGS_NOINHERIT | OPEN_ACCESS_READONLY, NULL);

  if (rc != 0) {
    printf("couldn't open OS2AHCI$; rc = %ld\n", rc);
    return(1);
  }

  /* get device information */
  cnt = 20;
  plen = sizeof(cnt);
  dlen = offsetof(OS2AHCI_DEVLIST, devs) + cnt * sizeof(dl->devs);
  dl = malloc(dlen);
  rc = DosDevIOCtl(fh, OS2AHCI_IOCTL_CATEGORY, OS2AHCI_IOCTL_GET_DEVLIST,
                   &cnt, plen, &plen, dl, dlen, &dlen);

  if (rc != 0) {
    printf("couldn't get device list; rc = %ld\n", rc);
    return(1);
  }

  for (i = 0; i < dl->cnt; i++) {
    printf("connection: %d.%d.%d, type: %d, ncq_max: %d, flags: 0x%04x\n",
           dl->devs[i].adapter, dl->devs[i].port, dl->devs[i].device,
           dl->devs[i].type, dl->devs[i].ncq_max, dl->devs[i].flags);
  }

  printf("\n");

  /* get ATA ID buffer for each device */
  for (i = 0; i < dl->cnt; i++) {
    memset(&pt, 0x00, sizeof(pt));
    memset(sense_buf, 0x00, sizeof(sense_buf));
    pt.adapter   = dl->devs[i].adapter;
    pt.port      = dl->devs[i].port;
    pt.device    = dl->devs[i].device;
    pt.flags     = 0;
    pt.timeout   = 0;

    pt.cmdlen    = sizeof(pt.cmd.ata);
    pt.cmd.ata.cmd = (dl->devs[i].flags & DF_ATAPI) ? 0xa1 : 0xec;

    pt.buflen    = sizeof(id_buf);
    pt.buf       = id_buf;
    pt.sense_len = sizeof(sense_buf);

    plen = sizeof(pt);
    dlen = sizeof(sense_buf);
    rc = DosDevIOCtl(fh, OS2AHCI_IOCTL_CATEGORY, OS2AHCI_IOCTL_PASSTHROUGH,
                     &pt, plen, &plen, sense_buf, dlen, &dlen);

    if (rc == 0) {
      printf("ID for %d.%d.%d: ", dl->devs[i].adapter, dl->devs[i].port,
             dl->devs[i].device);
      for (n = 54; n < 94; n += 2) {
        printf("%c%c", id_buf[n+1], id_buf[n]);
      }
      printf("\n");

    } else {
      printf("\nID command failed (rc = %ld); sense code:\n", rc);
      phex(sense_buf, sizeof(sense_buf));
    }

    if (dl->devs[i].type == 5) {
      /* Test sense buffer handling by ejecting the drive tray, then
       * checking whether the unit is ready.
       */
      printf("ejecting drive tray\n");
      memset(&pt, 0x00, sizeof(pt));
      memset(sense_buf, 0x00, sizeof(sense_buf));
      pt.adapter    = dl->devs[i].adapter;
      pt.port       = dl->devs[i].port;
      pt.device     = dl->devs[i].device;
      pt.flags      = PT_ATAPI;
      pt.timeout    = 0;

      pt.cmdlen     = 6;
      pt.cmd.cdb[0] = 0x1b;
      pt.cmd.cdb[4] = 0x02;

      pt.buflen     = 0;
      pt.buf        = NULL;
      pt.sense_len  = sizeof(sense_buf);

      plen = sizeof(pt);
      dlen = sizeof(sense_buf);
      rc = DosDevIOCtl(fh, OS2AHCI_IOCTL_CATEGORY, OS2AHCI_IOCTL_PASSTHROUGH,
                       &pt, plen, &plen, sense_buf, dlen, &dlen);

      if (rc != 0) {
        printf("\neject command failed (rc = %ld); sense code:\n", rc);
        phex(sense_buf, sizeof(sense_buf));
        break;
      }

      printf("test unit ready\n");
      memset(&pt, 0x00, sizeof(pt));
      memset(sense_buf, 0x00, sizeof(sense_buf));
      pt.adapter    = dl->devs[i].adapter;
      pt.port       = dl->devs[i].port;
      pt.device     = dl->devs[i].device;
      pt.flags      = PT_ATAPI;
      pt.timeout    = 0;

      pt.cmdlen     = 6;
      pt.cmd.cdb[0] = 0x00;

      pt.buflen     = 0;
      pt.buf        = NULL;
      pt.sense_len  = sizeof(sense_buf);

      plen = sizeof(pt);
      dlen = sizeof(sense_buf);
      rc = DosDevIOCtl(fh, OS2AHCI_IOCTL_CATEGORY, OS2AHCI_IOCTL_PASSTHROUGH,
                       &pt, plen, &plen, sense_buf, dlen, &dlen);

      if (rc != 0) {
        printf("\ntest unit ready command failed as expected (rc = %ld); sense code:\n", rc);
        phex(sense_buf, sizeof(sense_buf));
        break;
      }
    }
  }

  return(0);
}

void phex(const void *p, int len)
{
  const unsigned char *buf = p;
  long pos = 0;
  int i;

  /* print hex block */
  while (len > 0) {
    printf("%08X ", pos);

    /* print hex block */
    for (i = 0; i < 16; i++) {
      if (i < len) {
        printf("%c%02x", ((i == 8) ? '-' : ' '), buf[i]);
      } else {
        printf("   ");
      }
    }

    /* print ASCII block */
    printf("   ");
    for (i = 0; i < ((len > 16) ? 16 : len); i++) {
      printf("%c", (buf[i] >= 32 && buf[i] < 128) ? buf[i] : '.');
    }
    printf("\n");

    pos += 16;
    buf += 16;
    len -= 16;
  }
}

