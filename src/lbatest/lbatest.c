/******************************************************************************

  lbatest.c - simple (dumb) test program for os2ahci

  Writes the actual LBA to each sector on a hard disk. The purpose is to
  find out if the sector mapping of os2ahci.add is correct.

  THe program was written to run under Linux, since OS/2 physical disk
  access IOCTLs support CHS addressing, only.

  The os2ahci.add driver provides a special operating mode that does nothing
  but verifying that the tests drive's sector's are addressed in the correct
  manner by verifying the sector numbers written under Linux.

  Author: Markus Thielen

  Compilation: gcc -o lbatest lbatest.c

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

#include <stddef.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#define SECTOR_SIZE        512
#define SECTORS_PER_WRITE  256


/*--- function prototypes ---------------------------------------------------*/

void          usage            (void);

void          write_test       (char *dev);

/*--- global data -----------------------------------------------------------*/


/*--- start of code ---------------------------------------------------------*/

/******************************************************************************
 * main()
 */
int main(int argc, char **argv)
{

  if (argc < 2) {
    usage();
    return -1;
  }

  write_test(argv[1]);

  return 0;
}



/******************************************************************************
 * write_test() - write each sector's address to the sector itself
 */
void write_test(char *dev)
{
  unsigned long lba;
  float gbf = 1024.0 * 1024.0 * 1024.0 / 512.0;
  size_t i;
  unsigned char *wbuf;
  char buf[100];
  size_t cbtake = SECTOR_SIZE * SECTORS_PER_WRITE;
  size_t cb_written;
  FILE *fp;

  /* ask for confirmation, we destroy the drive's contents */
  printf("I'm going to destroy ALL DATA on drive %s now.\n"
	 "type 'DESTROY' to continue, anything else to bail out.\n", dev);
  if (fgets(buf, sizeof(buf), stdin) == NULL || strcmp(buf, "DESTROY\n")) {
    return;
  }

  /* open drive */
  if ((fp = fopen(dev, "wb")) == NULL) {
    perror("Failed to open device");
    return;
  }

  /* allocate big write buffer */
  wbuf = calloc(SECTOR_SIZE, SECTORS_PER_WRITE);

  /* go... */
  for (lba = 0; ; lba += SECTORS_PER_WRITE) {

    /* prepare buffer; set the first 4 bytes of each sector
     * to its LBA */
    for (i = 0; i < SECTORS_PER_WRITE; i++) {
      *((unsigned long *)(wbuf + i * SECTOR_SIZE)) = lba + i;
    }

    /* write buffer to disk */
    cb_written = fwrite(wbuf, 1, cbtake, fp);
    if (cb_written < cbtake) {
      if (!feof(fp)) {
        perror("Failed to write to target device");
      }
      break;
    }

    /* write progress */
    printf("\r%uk sectors written (%0.02f GB)",
           (size_t)(lba / 1000), (float)lba / gbf);

  }

  /* cleanup */
  fclose(fp);
  free(wbuf);

}

/******************************************************************************
 * usage() - print usage summary to STDOUT
 */
void usage(void)
{
  printf("lbatest for os2ahci.add\n"
	 "Usage:\n\n"
	 "lbatest <device>\n\n");
}

