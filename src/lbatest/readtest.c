/******************************************************************************

  readtest.c - simple (dumb) read test program for os2ahci

  Reads each sector of a logical drive under OS/2 and verifies that the
  sector number (LBA) written (by lbatest) in the first 4 bytes of each sector 
  matches its real LBA.

  To run the test, run lbatest under Linux, then attach the drive to an OS/2
  system an create a single primary partition that stretches the entire drive.
  Then run readtest.exe with the drive letter as a parameter.
 

  Author: Markus Thielen

  Compilation (Watcom): wcl386 -bt=os2 readtest.c

  Copyright (c) 2010 by thi.guten Software development, www.thiguten.de

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

******************************************************************************/

/*---- includes -------------------------------------------------------------*/

#define INCL_DOS
#define INCL_ERRORS
#define INCL_DOSDEVICES
#define INCL_DOSDEVIOCTL
#include <os2.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <signal.h>

#define SECTOR_SIZE        512
#define SECTORS_PER_READ   200


/*--- function prototypes ---------------------------------------------------*/

void          usage            (void);

int           read_test        (char *drv);

void          signal_handler   (int sig_no);

/*--- global data -----------------------------------------------------------*/

unsigned long lba_pos;
unsigned long first_wrong_sector = 0xffffffff;


/*--- start of code ---------------------------------------------------------*/

/******************************************************************************
 * main()
 */
int main(int argc, char **argv)
{
  HFILE hf;
  char drv[50];

  if (argc < 2) {
    usage();
    return -1;
  }

  if (isalpha(argv[1][0])) {
    sprintf(drv, "\\\\.\\%c", toupper(argv[1][0]));
  } else if (isdigit(argv[1][0])) {
    sprintf(drv, "\\\\.\\Physical_Disk%c", argv[1][0]);
  } else {
    usage();
    return -1;
  }

  return read_test(drv);

}

/******************************************************************************
 * read_test() - read each sector's first 4 bytes and compare to its LBA
 */
int read_test(char *drv)
{
  unsigned long ret;
  HFILE hf_disk;
  unsigned long action;
  unsigned long cb_read;
  char *buf;
  int rc = 0;
  unsigned long cb_take = SECTOR_SIZE * SECTORS_PER_READ;
  unsigned long err_cnt = 0;
  float gbf = 1024.0 * 1024.0 * 1024.0 / 512.0;
  LONGLONG cbfile = {0};
  int s;
  unsigned long sectors;
  unsigned long val;

  buf = calloc(SECTOR_SIZE, SECTORS_PER_READ);

  /* open drive */
  printf("Getting handle for drive %s\n", drv);
  ret = DosOpen(drv, &hf_disk, &action, 0, 0, 
                OPEN_ACTION_OPEN_IF_EXISTS,
/*                   OPEN_FLAGS_DASD | OPEN_FLAGS_FAIL_ON_ERROR | */
/*                   OPEN_FLAGS_WRITE_THROUGH | OPEN_FLAGS_NO_CACHE | */
/*                   OPEN_FLAGS_SEQUENTIAL | OPEN_SHARE_DENYREADWRITE | */
                OPEN_SHARE_DENYNONE | OPEN_ACCESS_READONLY,
                NULL);
  if (ret != NO_ERROR) {
    fprintf(stderr, "Failed to open disk %s for reading: %d\n", drv, ret);
    return -1;
  }

  /* lock disk */
/*    ret = DosDevIOCtl(hf_disk, IOCTL_DISK, DSK_LOCKDRIVE, NULL, 0, */
/*  		    &action, NULL, 0, &cb_read); */
/*    if (ret != NO_ERROR) { */
/*      fprintf(stderr, "Failed to lock drive, code %d\n", ret); */
/*      rc = -1; */
/*      goto cleanup; */
/*    } */

  /* catch Ctrl+C */
  signal(SIGINT, signal_handler);

  /* go... */
  memset(buf, 0x00, sizeof(buf));
  for (lba_pos = 0; ; lba_pos += SECTORS_PER_READ) {
    ret = DosRead(hf_disk, buf, cb_take, &cb_read);
    if (ret != NO_ERROR) {
      fprintf(stderr, "\nFailed to read from disk %s, code %d\n", drv, ret);
      rc = -1;
      goto cleanup;
    }

    if (cb_read == 0) {
      goto cleanup;
    }

    sectors = cb_read / SECTOR_SIZE;

    for (s = 0; s < sectors; s++) {

      if (lba_pos + s < 64) {
        continue;
      }

      val = *((unsigned long*) (buf + s * SECTOR_SIZE));
      if (val == 0xf6f6f6f6) {
        /* appearantly, this is the funny first partition sector 
         * created by LVM - skip it */
        continue;
      }

      if (val != lba_pos + s) {
        printf("\nWrong sector number: read 0x%08x from sector 0x%08x",
               val, lba_pos + s);
        err_cnt++;
        if (first_wrong_sector == 0xffffffff) {
          first_wrong_sector = lba_pos + s;
        }
      }
    }

    /* progress */
    printf("\r%dk sectors read (%0.02f GB)", lba_pos / 1000, 
           (float) lba_pos / gbf);
  }

cleanup:
  /* unlock drive */
  DosDevIOCtl(hf_disk, IOCTL_DISK, DSK_UNLOCKDRIVE, NULL, 0, &action,
              NULL, 0, &cb_read);
  DosClose(hf_disk);
  free(buf);

  /* print summary */
  printf("Found %d logical errors\n", err_cnt);
  if (first_wrong_sector != 0xffffffff) {
    printf("First wrong sector was %u.\n", err_cnt, first_wrong_sector);
  }

  return rc;
}

/******************************************************************************
 * usage() - print usage summary to STDOUT
 */
void usage(void)
{
  printf("LBA (read sector number) test for os2ahci.add\n"
         "Call with a drive letter (for logical drive) or 1-based disk number (for\n"
         "physical drive)\n\n"
	 "Usage:\n\n"
	 "lbatest <drive letter|drive number>\n\n");
}


/******************************************************************************
 * signal_handler for SIGINT - prints summary to STDOUT
 */
void signal_handler(int sig_no)
{

  if (sig_no == SIGINT) {
    /* read mode interrupted; show summary */
    printf("\n\nLast block read:    0x%08x\n", lba_pos);
    if (first_wrong_sector != 0xffffffff) {
      printf("First wrong sector: 0x%08x\n", first_wrong_sector);
    } else {
      printf("All sectors read ok\n");
    }
  }

  exit(0);

}
