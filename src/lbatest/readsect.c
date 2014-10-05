/******************************************************************************

  readsect.c - simple (dumb) HD sector read and test program

  Author: Markus Thielen

  Compilation (Watcom): wcl386 -bt=os2 readsect.c
                        wcl386 -bt=os2 -d3 readsect.c (for debug)

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
#include <io.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <string.h>
#include <errno.h>

#define SECTOR_SIZE        512
#define SECTORS_PER_READ   4


/*--- function prototypes ---------------------------------------------------*/

void          usage            (void);

int           read_sectors     (char *drv, unsigned long sectors_to_read, 
                                unsigned long sectors_per_read, FILE *fpo);

void          log              (char *fmt, ...);


/*--- global data -----------------------------------------------------------*/

char *log_fn = NULL;
unsigned long buffer_offset = 0;


/*--- start of code ---------------------------------------------------------*/

/******************************************************************************
 * main()
 */
int main(int argc, char **argv)
{
  char drv[50];
  char *output;
  FILE *fpo = NULL; /* output file */
  int ret;
  unsigned long sectors_to_read = 1024;
  unsigned long i;
  unsigned long sectors_per_read = SECTORS_PER_READ;
  unsigned long loop_sectors_start = sectors_per_read;
  int a;

  if (argc < 4) {
    usage();
    return -1;
  }

  /* parse args */
  for (a = 1; a < argc; a++) {

    if (argv[a][0] != '-' && argv[a][0] != '/') {
      fprintf(stderr, "unknown argument %s - aborting\n", argv[a]);
      return (-1);
    }

    switch (tolower(argv[a][1])) {

    case 'd':
      /* drive spec */
      if (isalpha(argv[++a][0])) {
        sprintf(drv, "\\\\.\\%c", toupper(argv[a][0]));
      } else if (isdigit(argv[a][0])) {
        sprintf(drv, "\\\\.\\Physical_Disk%c", argv[a][0]);
      } else {
        usage();
        return -1;
      }
      break;

    case 's':
      /* get sectors to read */
      sectors_to_read = strtoul(argv[++a], NULL, 10);
      break;

    case 'o':
      /* get output file */
      output = argv[++a];
      if (*output == '-' && output[1] == 0) {
        /* use stdout */
        fpo = stdout;
      } else {
        fpo = fopen(output, "wb");
        if (!fpo) {
          perror("Failed to open output file");
          return(-1);
        }
      }
      break;

    case 'b':
      /* optional buffer size */
      if (*argv[++a] == '-') {
        /* use a loop from 1 sectors to number specfied */
        loop_sectors_start = 1;
        sectors_per_read = (unsigned long) atoi(argv[a] + 1);
        log("Looping sector count from 1 to %u; not using output file\n",
            sectors_per_read);
        if (fpo && fpo != stdin) {
          fclose(fpo);
          fpo = NULL;
        }
        
      } else {
        /* just one run */
        sectors_per_read = (unsigned long) atoi(argv[a]);
        loop_sectors_start = sectors_per_read;
      }
      if (sectors_per_read == 0) {
        sectors_per_read = SECTORS_PER_READ;
      }
      break;

    case 'l':
      /* optional log file name */
      log_fn = argv[++a];
      unlink(log_fn);
      break;

    case 'u':
      /* use unaligned read buffer */
      buffer_offset = 1;
      break;
    }
  }

  /* go */
  for (i = loop_sectors_start; i <= sectors_per_read; i++) {
    ret = read_sectors(drv, sectors_to_read, i, fpo);
  }

  if (fpo && fpo != stdout) {
    fclose(fpo);
  }

  log("\nDone\n");
  return ret;

  }

/******************************************************************************
 * read_sectors() - read sectors and dump to output file
 */
int read_sectors(char *drv, unsigned long sectors_to_read, 
                 unsigned long sectors_per_read, FILE *fpo)
{
  unsigned long ret;
  HFILE hf_disk;
  unsigned long action;
  unsigned long cb_read;
  char *buf;
  int rc = 0;
  unsigned long cb_take;
  unsigned long sectors_take;
  unsigned long sectors_read = 0;

  buf = malloc(SECTOR_SIZE * sectors_per_read + buffer_offset);
  log("\nusing %u sectors per read\n", sectors_per_read);
  log("read buffer address: 0x%u %s\n", buf + buffer_offset, 
      buffer_offset ? "(unaligned)" : "");

  /* open drive */
  log("Getting handle for drive %s\n", drv);
  ret = DosOpen(drv, &hf_disk, &action, 0, 0, 
                OPEN_ACTION_OPEN_IF_EXISTS,
                OPEN_SHARE_DENYNONE | OPEN_ACCESS_READONLY,
                NULL);
  if (ret != NO_ERROR) {
    fprintf(stderr, "Failed to open disk %s for reading: %d\n", drv, ret);
    return -1;
  }

  /* go... */
  while (sectors_read < sectors_to_read) {

    sectors_take = min(sectors_to_read - sectors_read, sectors_per_read);
    cb_take = SECTOR_SIZE * sectors_take;

    log("\rReading %u sector(s) this run; (%u sectors read so far)...",
           sectors_take, sectors_read);
    ret = DosRead(hf_disk, buf + buffer_offset, cb_take, &cb_read);
    if (ret != NO_ERROR) {
      fprintf(stderr, "\nFailed to read from disk %s, code %d\n", drv, ret);
      rc = -1;
      break;
    }

    if (cb_read == 0) {
      break;
    }

    if (cb_read != cb_take) {
      log("\n\nRead only %u instead of %u bytes...\n\n", cb_read, cb_take);
      break;
    }

    sectors_read += cb_read / SECTOR_SIZE;
    if (fpo && fwrite(buf + buffer_offset, 1, cb_read, fpo) != cb_read) {
      perror("Failed to write to output file");
      rc = -1;
      break;
    }

  }

  log("\n%u sectors read.\n", sectors_read);

  DosClose(hf_disk);
  free(buf);

  return rc;
}

/******************************************************************************
 * usage() - print usage summary to STDOUT
 */
void usage(void)
{
  printf("readsect.exe - read HD sectors and store them to an output file.\n"
         "Call with a drive letter (for logical drive) or 1-based disk number (for\n"
         "physical drive)\n\n"
	 "Usage:\n\n"
	 "readsect <drive letter|number> <number of sectors> <outfile>\n"
         "         [<read buffer size>] [-lf <log file>] [-unaligned]\n"
         "where:\n"
         "-d drive letter       drive letter of logical drive to read from\n"
         "   drive number       1-based physical disk number to read from\n"
         "-s number of sectors  number of sectors to read (e.g. 1024)\n"
         "-o outfile            path and filename of output file\n"
         "-b read buffer size   optional buffer size in number of sectors (512 byte)\n"
         "                      (default is %d sectors)\n"
         "-l log file           optional log file name\n"
         "-u                    use unaligned buffer\n",
         SECTORS_PER_READ);
}


/*******************************************************************************
 * log() - log a string to the log file (if any)
 */
void log(char *fmt, ...)
{
  va_list arglist;
  FILE *fp;
  char buf[500];

  /* assemble arguments */
  va_start(arglist, fmt);
  vsprintf(buf, fmt, arglist);

  if (log_fn && (fp = fopen(log_fn, "a")) != NULL) {
    /* write string to log file */
    fprintf(fp, buf);
    fflush(fp);
    fclose(fp);
  }

  /* write to stdout */
  printf(buf);

}
