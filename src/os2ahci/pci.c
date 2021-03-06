/******************************************************************************
 * PCI.c - PCI constants and detection code for os2ahci driver
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

/* -------------------------- macros and constants ------------------------- */

/* offset of PCI base address register (BAR) in the PCI config space */
#define PCI_BAR(reg)   (UCHAR) (0x10 + (reg) * sizeof(u32))

/* ------------------------ typedefs and structures ------------------------ */

/* -------------------------- function prototypes -------------------------- */

static void  add_pci_device(PCI_ID *pci_id, USHORT BusDevFunc);
static long  bar_resource(USHORT BusDevFunc, RESOURCESTRUCT *resource, int i);
static char *rmerr(APIRET ret);

/* ------------------------ global/static variables ------------------------ */

/******************************************************************************
 * chipset/controller name strings
 */
static char chip_esb2[]    = "ESB2";
static char chip_ich8[]    = "ICH8";
static char chip_ich8m[]   = "ICH8M";
static char chip_ich9[]    = "ICH9";
static char chip_ich9m[]   = "ICH9M";
static char chip_ich10[]   = "ICH10";
static char chip_pchahci[] = "PCH AHCI";
static char chip_pchraid[] = "PCH RAID";
static char chip_tolapai[] = "Tolapai";
static char chip_sb600[]   = "SB600";
static char chip_sb700[]   = "SB700/800";
static char chip_vt8251[]  = "VT8251";
static char chip_mcp65[]   = "MCP65";
static char chip_mcp67[]   = "MCP67";
static char chip_mcp73[]   = "MCP73";
static char chip_mcp77[]   = "MCP77";
static char chip_mcp79[]   = "MCP79";
static char chip_mcp89[]   = "MCP689";
static char chip_sis968[]  = "968";

static char s_generic[]    = "Generic";


/******************************************************************************
 * PCI vendor and device IDs for known AHCI adapters. Copied from the Linux
 * AHCI driver.
 */

PCI_ID pci_ids[] =
{
  /* Intel
   * NOTE: ICH5 controller does NOT support AHCI, so we do
   *       not add it here! */
  { PCI_VDEVICE(INTEL, 0x2652), board_ahci,            "ICH6"       }, /* ICH6 */
  { PCI_VDEVICE(INTEL, 0x2653), board_ahci,            "ICH6M"      }, /* ICH6M */
  { PCI_VDEVICE(INTEL, 0x27c1), board_ahci,            "ICH7"       }, /* ICH7 */
  { PCI_VDEVICE(INTEL, 0x27c5), board_ahci,            "ICH7M"      }, /* ICH7M */
  { PCI_VDEVICE(INTEL, 0x27c3), board_ahci,            "ICH7R"      }, /* ICH7R */
  { PCI_VDEVICE(AL,    0x5288), board_ahci_ign_iferr,  "ULiM5288"   }, /* ULi M5288 */
  { PCI_VDEVICE(INTEL, 0x2681), board_ahci,            chip_esb2    }, /* ESB2 */
  { PCI_VDEVICE(INTEL, 0x2682), board_ahci,            chip_esb2    }, /* ESB2 */
  { PCI_VDEVICE(INTEL, 0x2683), board_ahci,            chip_esb2    }, /* ESB2 */
  { PCI_VDEVICE(INTEL, 0x27c6), board_ahci,            "ICH7MDH"    }, /* ICH7-M DH */
  { PCI_VDEVICE(INTEL, 0x2821), board_ahci,            chip_ich8    }, /* ICH8 */
  { PCI_VDEVICE(INTEL, 0x2822), board_ahci_nosntf,     chip_ich8    }, /* ICH8 */
  { PCI_VDEVICE(INTEL, 0x2824), board_ahci,            chip_ich8    }, /* ICH8 */
  { PCI_VDEVICE(INTEL, 0x2829), board_ahci,            chip_ich8m   }, /* ICH8M */
  { PCI_VDEVICE(INTEL, 0x282a), board_ahci,            chip_ich8m   }, /* ICH8M */
  { PCI_VDEVICE(INTEL, 0x2922), board_ahci,            chip_ich9    }, /* ICH9 */
  { PCI_VDEVICE(INTEL, 0x2923), board_ahci,            chip_ich9    }, /* ICH9 */
  { PCI_VDEVICE(INTEL, 0x2924), board_ahci,            chip_ich9    }, /* ICH9 */
  { PCI_VDEVICE(INTEL, 0x2925), board_ahci,            chip_ich9    }, /* ICH9 */
  { PCI_VDEVICE(INTEL, 0x2927), board_ahci,            chip_ich9    }, /* ICH9 */
  { PCI_VDEVICE(INTEL, 0x2929), board_ahci,            chip_ich9m   }, /* ICH9M */
  { PCI_VDEVICE(INTEL, 0x292a), board_ahci,            chip_ich9m   }, /* ICH9M */
  { PCI_VDEVICE(INTEL, 0x292b), board_ahci,            chip_ich9m   }, /* ICH9M */
  { PCI_VDEVICE(INTEL, 0x292c), board_ahci,            chip_ich9m   }, /* ICH9M */
  { PCI_VDEVICE(INTEL, 0x292f), board_ahci,            chip_ich9m   }, /* ICH9M */
  { PCI_VDEVICE(INTEL, 0x294d), board_ahci,            chip_ich9    }, /* ICH9 */
  { PCI_VDEVICE(INTEL, 0x294e), board_ahci,            chip_ich9m   }, /* ICH9M */
  { PCI_VDEVICE(INTEL, 0x502a), board_ahci,            chip_tolapai }, /* Tolapai */
  { PCI_VDEVICE(INTEL, 0x502b), board_ahci,            chip_tolapai }, /* Tolapai */
  { PCI_VDEVICE(INTEL, 0x3a05), board_ahci,            chip_ich10   }, /* ICH10 */
  { PCI_VDEVICE(INTEL, 0x3a22), board_ahci,            chip_ich10   }, /* ICH10 */
  { PCI_VDEVICE(INTEL, 0x3a25), board_ahci,            chip_ich10   }, /* ICH10 */
  { PCI_VDEVICE(INTEL, 0x3b22), board_ahci,            chip_pchahci }, /* PCH AHCI */
  { PCI_VDEVICE(INTEL, 0x3b23), board_ahci,            chip_pchahci }, /* PCH AHCI */
  { PCI_VDEVICE(INTEL, 0x3b24), board_ahci,            chip_pchraid }, /* PCH RAID */
  { PCI_VDEVICE(INTEL, 0x3b25), board_ahci,            chip_pchraid }, /* PCH RAID */
  { PCI_VDEVICE(INTEL, 0x3b29), board_ahci,            chip_pchahci }, /* PCH AHCI */
  { PCI_VDEVICE(INTEL, 0x3b2b), board_ahci,            chip_pchraid }, /* PCH RAID */
  { PCI_VDEVICE(INTEL, 0x3b2c), board_ahci,            chip_pchraid }, /* PCH RAID */
  { PCI_VDEVICE(INTEL, 0x3b2f), board_ahci,            chip_pchahci }, /* PCH AHCI */

  /* JMicron 360/1/3/5/6, match class to avoid IDE function */
  { PCI_VENDOR_ID_JMICRON, PCI_ANY_ID, PCI_ANY_ID, PCI_ANY_ID,
    PCI_CLASS_STORAGE_SATA_AHCI, 0xffffffL, board_ahci_ign_iferr, "360" },

  /* ATI */
  { PCI_VDEVICE(ATI, 0x4380), board_ahci_sb600,        chip_sb600   }, /* ATI SB600 */
  { PCI_VDEVICE(ATI, 0x4390), board_ahci_sb700,        chip_sb700   }, /* ATI SB700/800 */
  { PCI_VDEVICE(ATI, 0x4391), board_ahci_sb700,        chip_sb700   }, /* ATI SB700/800 */
  { PCI_VDEVICE(ATI, 0x4392), board_ahci_sb700,        chip_sb700   }, /* ATI SB700/800 */
  { PCI_VDEVICE(ATI, 0x4393), board_ahci_sb700,        chip_sb700   }, /* ATI SB700/800 */
  { PCI_VDEVICE(ATI, 0x4394), board_ahci_sb700,        chip_sb700   }, /* ATI SB700/800 */
  { PCI_VDEVICE(ATI, 0x4395), board_ahci_sb700,        chip_sb700   }, /* ATI SB700/800 */

  /* AMD */
  { PCI_VDEVICE(AMD, 0x7800), board_ahci }, /* AMD Hudson-2 */
  /* AMD is using RAID class only for ahci controllers */
  { PCI_VENDOR_ID_AMD, PCI_ANY_ID, PCI_ANY_ID, PCI_ANY_ID,
    PCI_CLASS_STORAGE_RAID << 8, 0xffffffL, board_ahci, "Hudson2" },

  /* VIA */
  { PCI_VDEVICE(VIA, 0x3349), board_ahci_vt8251,       chip_vt8251  }, /* VIA VT8251 */
  { PCI_VDEVICE(VIA, 0x6287), board_ahci_vt8251,       chip_vt8251  }, /* VIA VT8251 */

  /* NVIDIA */
  { PCI_VDEVICE(NVIDIA, 0x044c), board_ahci_mcp65,     chip_mcp65   }, /* MCP65 */
  { PCI_VDEVICE(NVIDIA, 0x044d), board_ahci_mcp65,     chip_mcp65   }, /* MCP65 */
  { PCI_VDEVICE(NVIDIA, 0x044e), board_ahci_mcp65,     chip_mcp65   }, /* MCP65 */
  { PCI_VDEVICE(NVIDIA, 0x044f), board_ahci_mcp65,     chip_mcp65   }, /* MCP65 */
  { PCI_VDEVICE(NVIDIA, 0x045c), board_ahci_mcp65,     chip_mcp65   }, /* MCP65 */
  { PCI_VDEVICE(NVIDIA, 0x045d), board_ahci_mcp65,     chip_mcp65   }, /* MCP65 */
  { PCI_VDEVICE(NVIDIA, 0x045e), board_ahci_mcp65,     chip_mcp65   }, /* MCP65 */
  { PCI_VDEVICE(NVIDIA, 0x045f), board_ahci_mcp65,     chip_mcp65   }, /* MCP65 */
  { PCI_VDEVICE(NVIDIA, 0x0550), board_ahci_yesncq,    chip_mcp67   }, /* MCP67 */
  { PCI_VDEVICE(NVIDIA, 0x0551), board_ahci_yesncq,    chip_mcp67   }, /* MCP67 */
  { PCI_VDEVICE(NVIDIA, 0x0552), board_ahci_yesncq,    chip_mcp67   }, /* MCP67 */
  { PCI_VDEVICE(NVIDIA, 0x0553), board_ahci_yesncq,    chip_mcp67   }, /* MCP67 */
  { PCI_VDEVICE(NVIDIA, 0x0554), board_ahci_yesncq,    chip_mcp67   }, /* MCP67 */
  { PCI_VDEVICE(NVIDIA, 0x0555), board_ahci_yesncq,    chip_mcp67   }, /* MCP67 */
  { PCI_VDEVICE(NVIDIA, 0x0556), board_ahci_yesncq,    chip_mcp67   }, /* MCP67 */
  { PCI_VDEVICE(NVIDIA, 0x0557), board_ahci_yesncq,    chip_mcp67   }, /* MCP67 */
  { PCI_VDEVICE(NVIDIA, 0x0558), board_ahci_yesncq,    chip_mcp67   }, /* MCP67 */
  { PCI_VDEVICE(NVIDIA, 0x0559), board_ahci_yesncq,    chip_mcp67   }, /* MCP67 */
  { PCI_VDEVICE(NVIDIA, 0x055a), board_ahci_yesncq,    chip_mcp67   }, /* MCP67 */
  { PCI_VDEVICE(NVIDIA, 0x055b), board_ahci_yesncq,    chip_mcp67   }, /* MCP67 */
  { PCI_VDEVICE(NVIDIA, 0x0580), board_ahci_yesncq,    chip_mcp67   }, /* Linux ID */
  { PCI_VDEVICE(NVIDIA, 0x07f0), board_ahci_yesncq,    chip_mcp73   }, /* MCP73 */
  { PCI_VDEVICE(NVIDIA, 0x07f1), board_ahci_yesncq,    chip_mcp73   }, /* MCP73 */
  { PCI_VDEVICE(NVIDIA, 0x07f2), board_ahci_yesncq,    chip_mcp73   }, /* MCP73 */
  { PCI_VDEVICE(NVIDIA, 0x07f3), board_ahci_yesncq,    chip_mcp73   }, /* MCP73 */
  { PCI_VDEVICE(NVIDIA, 0x07f4), board_ahci_yesncq,    chip_mcp73   }, /* MCP73 */
  { PCI_VDEVICE(NVIDIA, 0x07f5), board_ahci_yesncq,    chip_mcp73   }, /* MCP73 */
  { PCI_VDEVICE(NVIDIA, 0x07f6), board_ahci_yesncq,    chip_mcp73   }, /* MCP73 */
  { PCI_VDEVICE(NVIDIA, 0x07f7), board_ahci_yesncq,    chip_mcp73   }, /* MCP73 */
  { PCI_VDEVICE(NVIDIA, 0x07f8), board_ahci_yesncq,    chip_mcp73   }, /* MCP73 */
  { PCI_VDEVICE(NVIDIA, 0x07f9), board_ahci_yesncq,    chip_mcp73   }, /* MCP73 */
  { PCI_VDEVICE(NVIDIA, 0x07fa), board_ahci_yesncq,    chip_mcp73   }, /* MCP73 */
  { PCI_VDEVICE(NVIDIA, 0x07fb), board_ahci_yesncq,    chip_mcp73   }, /* MCP73 */
  { PCI_VDEVICE(NVIDIA, 0x0ad0), board_ahci,           chip_mcp77   }, /* MCP77 */
  { PCI_VDEVICE(NVIDIA, 0x0ad1), board_ahci,           chip_mcp77   }, /* MCP77 */
  { PCI_VDEVICE(NVIDIA, 0x0ad2), board_ahci,           chip_mcp77   }, /* MCP77 */
  { PCI_VDEVICE(NVIDIA, 0x0ad3), board_ahci,           chip_mcp77   }, /* MCP77 */
  { PCI_VDEVICE(NVIDIA, 0x0ad4), board_ahci,           chip_mcp77   }, /* MCP77 */
  { PCI_VDEVICE(NVIDIA, 0x0ad5), board_ahci,           chip_mcp77   }, /* MCP77 */
  { PCI_VDEVICE(NVIDIA, 0x0ad6), board_ahci,           chip_mcp77   }, /* MCP77 */
  { PCI_VDEVICE(NVIDIA, 0x0ad7), board_ahci,           chip_mcp77   }, /* MCP77 */
  { PCI_VDEVICE(NVIDIA, 0x0ad8), board_ahci,           chip_mcp77   }, /* MCP77 */
  { PCI_VDEVICE(NVIDIA, 0x0ad9), board_ahci,           chip_mcp77   }, /* MCP77 */
  { PCI_VDEVICE(NVIDIA, 0x0ada), board_ahci,           chip_mcp77   }, /* MCP77 */
  { PCI_VDEVICE(NVIDIA, 0x0adb), board_ahci,           chip_mcp77   }, /* MCP77 */
  { PCI_VDEVICE(NVIDIA, 0x0ab4), board_ahci,           chip_mcp79   }, /* MCP79 */
  { PCI_VDEVICE(NVIDIA, 0x0ab5), board_ahci,           chip_mcp79   }, /* MCP79 */
  { PCI_VDEVICE(NVIDIA, 0x0ab6), board_ahci,           chip_mcp79   }, /* MCP79 */
  { PCI_VDEVICE(NVIDIA, 0x0ab7), board_ahci,           chip_mcp79   }, /* MCP79 */
  { PCI_VDEVICE(NVIDIA, 0x0ab8), board_ahci,           chip_mcp79   }, /* MCP79 */
  { PCI_VDEVICE(NVIDIA, 0x0ab9), board_ahci,           chip_mcp79   }, /* MCP79 */
  { PCI_VDEVICE(NVIDIA, 0x0aba), board_ahci,           chip_mcp79   }, /* MCP79 */
  { PCI_VDEVICE(NVIDIA, 0x0abb), board_ahci,           chip_mcp79   }, /* MCP79 */
  { PCI_VDEVICE(NVIDIA, 0x0abc), board_ahci,           chip_mcp79   }, /* MCP79 */
  { PCI_VDEVICE(NVIDIA, 0x0abd), board_ahci,           chip_mcp79   }, /* MCP79 */
  { PCI_VDEVICE(NVIDIA, 0x0abe), board_ahci,           chip_mcp79   }, /* MCP79 */
  { PCI_VDEVICE(NVIDIA, 0x0abf), board_ahci,           chip_mcp79   }, /* MCP79 */
  { PCI_VDEVICE(NVIDIA, 0x0d84), board_ahci,           chip_mcp89   }, /* MCP89 */
  { PCI_VDEVICE(NVIDIA, 0x0d85), board_ahci,           chip_mcp89   }, /* MCP89 */
  { PCI_VDEVICE(NVIDIA, 0x0d86), board_ahci,           chip_mcp89   }, /* MCP89 */
  { PCI_VDEVICE(NVIDIA, 0x0d87), board_ahci,           chip_mcp89   }, /* MCP89 */
  { PCI_VDEVICE(NVIDIA, 0x0d88), board_ahci,           chip_mcp89   }, /* MCP89 */
  { PCI_VDEVICE(NVIDIA, 0x0d89), board_ahci,           chip_mcp89   }, /* MCP89 */
  { PCI_VDEVICE(NVIDIA, 0x0d8a), board_ahci,           chip_mcp89   }, /* MCP89 */
  { PCI_VDEVICE(NVIDIA, 0x0d8b), board_ahci,           chip_mcp89   }, /* MCP89 */
  { PCI_VDEVICE(NVIDIA, 0x0d8c), board_ahci,           chip_mcp89   }, /* MCP89 */
  { PCI_VDEVICE(NVIDIA, 0x0d8d), board_ahci,           chip_mcp89   }, /* MCP89 */
  { PCI_VDEVICE(NVIDIA, 0x0d8e), board_ahci,           chip_mcp89   }, /* MCP89 */
  { PCI_VDEVICE(NVIDIA, 0x0d8f), board_ahci,           chip_mcp89   }, /* MCP89 */

  /* SiS */
  { PCI_VDEVICE(SI, 0x1184), board_ahci,               "966"        }, /* SiS 966 */
  { PCI_VDEVICE(SI, 0x1185), board_ahci,               chip_sis968  }, /* SiS 968 */
  { PCI_VDEVICE(SI, 0x0186), board_ahci,               chip_sis968  }, /* SiS 968 */

  /* Marvell */
  { PCI_VDEVICE(MARVELL, 0x6145), board_ahci_mv,       "6145"       }, /* 6145 */
  { PCI_VDEVICE(MARVELL, 0x6121), board_ahci_mv,       "6121"       }, /* 6121 */

  /* Promise */
  { PCI_VDEVICE(PROMISE, 0x3f20), board_ahci,          "PDC42819"   }, /* PDC42819 */

  /* Generic, PCI class code for AHCI */
  { PCI_ANY_ID, PCI_ANY_ID, PCI_ANY_ID, PCI_ANY_ID,
    PCI_CLASS_STORAGE_SATA_AHCI, 0xffffffL, board_ahci, s_generic   },

  /* end of list, including a few slots to define custom adapters (10) */
  { 0, 0, 0, 0, 0, 0, 0, NULL },
  { 0, 0, 0, 0, 0, 0, 0, NULL },
  { 0, 0, 0, 0, 0, 0, 0, NULL },
  { 0, 0, 0, 0, 0, 0, 0, NULL },
  { 0, 0, 0, 0, 0, 0, 0, NULL },
  { 0, 0, 0, 0, 0, 0, 0, NULL },
  { 0, 0, 0, 0, 0, 0, 0, NULL },
  { 0, 0, 0, 0, 0, 0, 0, NULL },
  { 0, 0, 0, 0, 0, 0, 0, NULL },
  { 0, 0, 0, 0, 0, 0, 0, NULL },

  { 0, 0, 0, 0, 0, 0, 0, NULL }
};

/* ----------------------------- start of code ----------------------------- */

/******************************************************************************
 * Add specified PCI vendor and device ID to the list of supported AHCI
 * controllers. Please note that the last slot in pci_ids needs to remain
 * empty because it's used as end marker.
 */
int add_pci_id(u16 vendor, u16 device)
{
  int max_slot = sizeof(pci_ids) / sizeof(*pci_ids) - 2;
  int i;

  /* search for last used slot in 'pci_ids' */
  for (i = max_slot; i >= 0 && pci_ids[i].vendor == 0; i--);
  if (i >= max_slot) return(-1); /* all slots in use */

  /* use slot after the last used slot */
  i++;
  pci_ids[i].vendor = vendor;
  pci_ids[i].device = device;
  pci_ids[i].board = board_ahci;
  pci_ids[i].chipname = s_generic;
  return(0);
}

/******************************************************************************
 * Scan PCI bus using OEMHLP$ IOCTLs and build adapter list.
 */
void scan_pci_bus(void)
{
  UCHAR index;
  int ad_indx = 0;
  int i;
  int n;
  USHORT BusDevFunc;

  DPRINTF(3,"scanning PCI bus...\n");

  /* Go through the list of PCI IDs and search for each device
   *
   * NOTES:
   *
   *  - When searching via class code, the OEMHLP$ interface doesn't allow
   *    setting a bitmask to look for individual portions of class code,
   *    subclass code and programming interface. However, all bitmasks in the
   *    PCI list currently use 0xffffff, thus this should not be a problem at
   *    this point in time.
   *
   *  - Scanning via OEMHLP$ seems rather slow, at least in the virtual
   *    machine I'm currenly using to test this driver. Thus, class code
   *    scans are preferred unless the option "-t" (thorough_scan) has been
   *    specified. The assumption is that most, if not all, modern AHCI
   *    adapters have the correct class code (PCI_CLASS_STORAGE_SATA_AHCI).
   */
  for (i = 0; pci_ids[i].vendor != 0; i++)
  {
    index = 0;
    do
    {
      if (pci_ids[i].device == PCI_ANY_ID || pci_ids[i].vendor == PCI_ANY_ID)
      {
        /* look for class code */
        BusDevFunc = PciFindClass(pci_ids[i].class, index);
      }
      else if (thorough_scan)
      {
        /* look for this specific vendor and device ID */
        BusDevFunc = PciFindDevice( pci_ids[i].vendor, pci_ids[i].device, index);

      }
      else
      {
        BusDevFunc = 0xffff;
      }

      if (BusDevFunc != 0xffff)
      {
        /* found a device */
        int already_found = 0;

        /* increment index for next loop */
        if (++index > 180) return; /* something's wrong here... */

        /* check whether we already found this device */
        for (n = 0; n < ad_info_cnt; n++)
        {
          if (ad_infos[n].bus_dev_func == BusDevFunc)
          {
            /* this device has already been found (e.g. via thorough scan) */
            already_found = 1;
            break;
          }
        }

        if (already_found || (ad_ignore & (1U << ad_indx++)))
        {
          /* ignore this device; it has either already been found via a
           * thorough scan or has been specified to be ignored via command
           * line option */
          continue;
        }

        /* add this PCI device to ad_infos[] */
        add_pci_device(pci_ids + i, BusDevFunc);
      }

    } while (BusDevFunc != 0xffff);
  }
}

/******************************************************************************
 * Enable interrupt generation. PCI 2.3 added a bit which allows disabling
 * interrupt generation for a device. This function clears the corresponding
 * bit in the configuration space command register.
 */
int pci_enable_int(USHORT BusDevFunc)
{
  ULONG tmp;

  if (PciReadConfig(BusDevFunc, 4, sizeof(tmp), &tmp) ||
      PciWriteConfig(BusDevFunc, 4, sizeof(tmp), tmp & ~(1UL << 10)))
  {
    return(-1);
  }
  return(0);
}

/******************************************************************************
 * Hack to set up proper IRQ mappings in the emulated PIIX3 ISA bridge in
 * VirtualBox (for some reason, the first mapped IRQ is 0x80 without this
 * hack).
 */
void pci_hack_virtualbox(void)
{
  UCHAR irq = 0;

  if (!PciReadConfig(0x0008, 0x60, sizeof(irq), &irq) && irq == 0x80)
  {
    /* set IRQ for first device/func to 11 */
    DPRINTF(1,"hacking virtualbox PIIX3 PCI to ISA bridge IRQ mapping\n");
    irq = ad_infos[0].irq;
    PciWriteConfig(0x0008, 0x60, sizeof(irq), irq);
  }
}

/******************************************************************************
 * Add a single PCI device to the list of adapters.
 */
static void add_pci_device(PCI_ID *pci_id, USHORT BusDevFunc)
{
  char rc_list_buf[sizeof(AHRESOURCE) + sizeof(HRESOURCE) * 15];
  AHRESOURCE *rc_list = (AHRESOURCE *) rc_list_buf;
  RESOURCESTRUCT resource;
  ADAPTERSTRUCT adapter;
  ADJUNCT adj;
  AD_INFO *ad_info;
  APIRET ret;
  ULONG val;
  char tmp[40];
  u16 device;
  u16 vendor;
  u32 class;
  int irq;
  int pin;
  int i;

  /*****************************************************************************
   * Part 1: Get further information about the device to be added; PCI ID...
   */
  if (PciReadConfig(BusDevFunc, 0x00, sizeof(ULONG), &val)) return;
  device = (val >> 16);
  vendor = (val & 0xffff);

  /* ... and class code */
  if (PciReadConfig(BusDevFunc, 0x08, sizeof(ULONG), &val)) return;
  class = (val >> 8);

  if (pci_id->device == PCI_ANY_ID)
  {
    /* We found this device in a wildcard search. There are two possible
     * reasons which require a different handling:
     *
     *  1) This device uses a non-standard PCI class and has been identified
     *     with the corresponding class in pci_ids[] (e.g. the entry
     *     PCI_VENDOR_ID_JMICRON), but there is a vendor ID in pci_ids[]. In
     *     this case, we need to verify that the vendor is correct (see
     *     comments regarding OEMHLP limitations in 'scan_pci_bus()')
     *
     *  2) This device was identified using a generic PCI class for AHCI
     *     adapters such as PCI_CLASS_STORAGE_SATA_AHCI and we need to map
     *     the device and vendor ID to the corresponding index in pci_ids[]
     *     if there is such an entry; the index passed to this function will
     *     be the generic class-based index which is fine as long as there's
     *     not special treatment required as indicated by the board_*
     *     constants in pci_ids[]...
     *
     *     The main reason for this kludge is that it seems as if OEMHLP$
     *     is rather slow searching for PCI devices, adding around 30s
     *     to the boot time when scanning for individual AHCI PCI IDs. Thus,
     *     the OS2AHCI driver avoids this kind of scan in favor of a class-
     *     based scan (unless overridden with the "/T" option).
     */
    if (pci_id->vendor != PCI_ANY_ID)
    {
      /* case 1: the vendor is known but we found the PCI device using a class
       * search; verify vendor matches the one in pci_ids[]
       */
      if (pci_id->vendor != vendor) return; /* vendor doesn't match */
    }
    else
    {
      /* case 2: we found this device using a generic class search; if the
       * device/vendor is listed in pci_ids[], use this entry in favor of the
       * one passed in 'pci_id'
       */
      for (i = 0; pci_ids[i].vendor != 0; i++)
      {
        if (pci_ids[i].device == device && pci_ids[i].vendor == vendor)
        {
          pci_id = pci_ids + i;
          break;
        }
      }
    }
  }

  /* found a supported AHCI device */

  if (PciReadConfig(BusDevFunc, 0x3c, sizeof(u32), &val)) return;
  irq = (int) (val & 0xff);
  pin = (int) ((val >> 8) & 0xff);

  #if 0
  i = 1;
  if (irq==0 || irq==255) i = 0;

  if (verbosity > i)
  {
    iprintf("%s AHCI device %s %s (%d:%d:%d %04x:%04x) class:0x%06x", i?"Found":"Ignoring",
     vendor_from_id(vendor), device_from_id(device),
     PCI_BUS_FROM_BDF(BusDevFunc), PCI_DEV_FROM_BDF(BusDevFunc), PCI_FUNC_FROM_BDF(BusDevFunc),
     vendor, device, class);
    if (i==0) iprintf("Invalid interrupt (IRQ=%d).", irq);
  }
  if (i==0) return;
  #endif

  /* make sure we got room in the adapter information array */
  if (ad_info_cnt >= MAX_AD - 1)
  {
    iprintf("%s: too many AHCI devices", drv_name);
    return;
  }

  /****************************************************************************
   * Part 2: Determine resource requirements and allocate resources with the
   *         OS/2 resource manager. While doing so, some of the entries of the
   *         corresponding slot in the AD_INFO array, namely resource manager
   *         handles, are initialized so we need prepare the slot.
   *
   * NOTE: While registering resources with the resource manager, each new
   *       resource is added to the corresponding rc_list.hResource[] slot.
   *       rc_list is used further down to associate resources to adapters
   *       when the adapter itself is registered with the OS/2 resource
   *       manager.
   */
  ad_info = ad_infos + ad_info_cnt;
  memset(ad_info, 0x00, sizeof(*ad_info));
  rc_list->NumResource = 0;

  /* Allocate all I/O and MMIO addresses offered by this device. In theory,
   * we need only BAR #5, the AHCI MMIO BAR, but in order to prevent any
   * other driver from hijacking our device and accessing it via legacy
   * registers we'll reserve anything we can find.
   */

  ciprintf("Adapter %d PCI=%d:%d:%d ID=%04x:%04x\n", ad_info_cnt, PCI_BUS_FROM_BDF(BusDevFunc),
    PCI_DEV_FROM_BDF(BusDevFunc), PCI_FUNC_FROM_BDF(BusDevFunc), vendor, device);
  DPRINTF(1,"Adapter %d PCI=%d:%d:%d ID=%04x:%04x\n", ad_info_cnt, PCI_BUS_FROM_BDF(BusDevFunc),
    PCI_DEV_FROM_BDF(BusDevFunc), PCI_FUNC_FROM_BDF(BusDevFunc), vendor, device);

  for (i = 0; i < sizeof(ad_info->rm_bars) / sizeof(*ad_info->rm_bars); i++)
  {
    long len = bar_resource(BusDevFunc, &resource, i);

    if (len < 0)
    {
      /* something went wrong */
      goto add_pci_fail;
    }
    if (len == 0)
    {
      /* this BAR is unused */
      continue;
    }

    if (i == AHCI_PCI_BAR)
    {
      if (resource.ResourceType != RS_TYPE_MEM)
      {
        iprintf("%s: BAR #5 must be an MMIO region", drv_name);
        goto add_pci_fail;
      }
      /* save this BAR's address as MMIO address */
      ad_info->mmio_phys = resource.MEMResource.MemBase;
      ad_info->mmio_size = resource.MEMResource.MemSize;
    }

    /* register [MM]IO region with resource manager */
    ret = RMAllocResource(rm_drvh, ad_info->rm_bars + i, &resource);
    if (ret != RMRC_SUCCESS)
    {
      if (ret == RMRC_RES_ALREADY_CLAIMED)
      {
        ciiprintf("Device already claimed.");
      }
      else
      {
        iprintf("%s: couldn't register [MM]IO region (rc = %s)", drv_name, rmerr(ret));
      }
      goto add_pci_fail;
    }
    rc_list->hResource[rc_list->NumResource++] = ad_info->rm_bars[i];
  }

  if (ad_info->mmio_phys == 0)
  {
    iprintf("%s: couldn't determine MMIO base address", drv_name);
    goto add_pci_fail;
  }

  /****************************************************************************
   * Part 3: Fill in the remaining fields in the AD_INFO slot and allocate
   * memory and GDT selectors for the adapter. Finally, register the adapter
   * itself with the OS/2 resource manager
   */
  ad_info->pci = pci_id;
  ad_info->pci_vendor = vendor;
  ad_info->pci_device = device;
  ad_info->bus_dev_func = BusDevFunc;
  ad_info->irq = irq;
  ad_info->irq_pin = pin;

  ad_info->mmio = MapPhysToLin(ad_info->mmio_phys, ad_info->mmio_size);
  if (!ad_info->mmio) goto add_pci_fail;

  /* register adapter with resource manager */
  memset(&adj, 0x00, sizeof(adj));
  adj.pNextAdj       = NULL;
  adj.AdjLength      = sizeof(adj);
  adj.AdjType        = ADJ_ADAPTER_NUMBER;
  adj.Adapter_Number = ad_info_cnt;

  memset(&adapter, 0x00, sizeof(adapter));
  snprintf(tmp, sizeof(tmp), "AHCI_%d Controller", ad_info_cnt);
  adapter.AdaptDescriptName = tmp;
  adapter.AdaptFlags        = 0;
  adapter.BaseType          = AS_BASE_MSD;
  adapter.SubType           = AS_SUB_IDE;
  adapter.InterfaceType     = AS_INTF_GENERIC;
  adapter.HostBusType       = AS_HOSTBUS_PCI;
  adapter.HostBusWidth      = AS_BUSWIDTH_32BIT;
  adapter.pAdjunctList      = &adj;

  ret = RMCreateAdapter(rm_drvh, &ad_info->rm_adh, &adapter, NULL, rc_list);
  if (ret != RMRC_SUCCESS)
  {
    iprintf("%s: couldn't register adapter (rc = %s)", drv_name, rmerr(ret));
    goto add_pci_fail;
  }

  if (ahci_config_caps(ad_info)) goto add_pci_fail;

  #ifndef DAZ_NEW_CODE
  /* fill in DMA scratch buffer addresses in adapter info */
  for (i = 0; i < AHCI_MAX_PORTS; i++)
  {
    if (!(ad_info->port_map & (1UL << i))) continue;

    ad_info->ports[i].dma_buf = MemAllocAlign(AHCI_PORT_PRIV_DMA_SZ, 1024);
    ad_info->ports[i].dma_buf_phys = MemPhysAdr(ad_info->ports[i].dma_buf);
  }
  #endif

  /* Successfully added the adapter and reserved its resources; the adapter
   * is still under BIOS control so we're not going to do anything else at
   * this point.
   */

  ad_info_cnt++;
  return;

add_pci_fail:
  /* something went wrong; try to clean up as far as possible */
  for (i = 0; i < sizeof(ad_info->rm_bars) / sizeof(*ad_info->rm_bars); i++)
  {
    if (ad_info->rm_bars[i] != 0) RMDeallocResource(rm_drvh, ad_info->rm_bars[i]);
  }
}

/******************************************************************************
 * Prepare a resource structure for a PCI Base Address Register (BAR). This
 * basically means the type, address and range of the I/O address space. It
 * returns the length of the address range as a signed long to allow the caller
 * to differentiate between error conditions (< 0), unused BARs (0) or valid
 * bars (> 0).
 *
 * NOTE: In order to do this, we need to temporarily write 0xffffffff to
 *       the MMIO base address register (BAR), read back the resulting value
 *       and check the 0 bits from the right end, masking the lower 2 (I/O) or
 *       4 (MMIO) bits. After doing this, we must restore the original value
 *       set up by the BIOS.
 *
 *  31                                                        4 3 2 1 0
 *  -------------------------------------------------------------------
 *                        base address                          P T T I
 *  P = prefetchable
 *  T = type (0 = any 32 bit, 1 = <1M, 2 = 64 bit)
 *  I = I/O (1) or memory (0)
 */
static long bar_resource(USHORT BusDevFunc, RESOURCESTRUCT *resource, int i)
{
  u32 bar_addr = 0;
  u32 bar_size = 0;

  /* temporarily write 1s to this BAR to determine the address range */
  if (PciReadConfig (BusDevFunc, PCI_BAR(i), sizeof(u32), &bar_addr) ||
      PciWriteConfig(BusDevFunc, PCI_BAR(i), sizeof(u32), ~(0UL))    ||
      PciReadConfig (BusDevFunc, PCI_BAR(i), sizeof(u32), &bar_size) ||
      PciWriteConfig(BusDevFunc, PCI_BAR(i), sizeof(u32), bar_addr) )
  {
    iprintf("%s: couldn't determine [MM]IO size", drv_name);
    if (bar_addr != 0)
    {
      PciWriteConfig(BusDevFunc, PCI_BAR(i), sizeof(u32), bar_addr);
    }
    return(-1);
  }

  /* bar not implemented or device not working properly */
  if (bar_size == 0 || bar_size == 0xffffffffUL) return(0);

  /* prepare resource allocation structure */
  memset(resource, 0x00, sizeof(*resource));
  if (bar_addr & 1)
  {
    bar_size = ~(bar_size & 0xfffffffcUL) + 1;
    bar_size &= 0xffffUL;  /* I/O address space is 16 bits on x86 */
    bar_addr &= 0xfffcUL;

    resource->ResourceType              = RS_TYPE_IO;
    resource->IOResource.BaseIOPort     = bar_addr;
    resource->IOResource.NumIOPorts     = bar_size;
    resource->IOResource.IOFlags        = RS_IO_EXCLUSIVE;
    resource->IOResource.IOAddressLines = 16;

  }
  else
  {
    bar_size = ~(bar_size & 0xfffffff0UL) + 1;
    bar_addr &= 0xfffffff0UL;

    resource->ResourceType         = RS_TYPE_MEM;
    resource->MEMResource.MemBase  = bar_addr;
    resource->MEMResource.MemSize  = bar_size;
    resource->MEMResource.MemFlags = RS_MEM_EXCLUSIVE;
  }

  DPRINTF(3,"BAR #%d: type = %s, addr = 0x%08lx, size = %d\n", i,
           (resource->ResourceType == RS_TYPE_IO) ? "I/O" : "MEM",
           bar_addr, bar_size);

  return((long) bar_size);
}

/******************************************************************************
 * return vendor name for PCI vendor ID
 */
char *vendor_from_id(u16 id)
{

  switch(id)
  {
    case PCI_VENDOR_ID_AL:
      return "Ali";
    case PCI_VENDOR_ID_AMD:
    case PCI_VENDOR_ID_ATI:
      return "AMD";
    case PCI_VENDOR_ID_AT:
      return "Allied Telesyn";
    case PCI_VENDOR_ID_ATT:
      return "ATT";
    case PCI_VENDOR_ID_CMD:
      return "CMD";
    case PCI_VENDOR_ID_CT:
      return "CT";
    case PCI_VENDOR_ID_INTEL:
      return "Intel";
    case PCI_VENDOR_ID_INITIO:
      return "Initio";
    case PCI_VENDOR_ID_JMICRON:
      return "JMicron";
    case PCI_VENDOR_ID_MARVELL:
      return "Marvell";
    case PCI_VENDOR_ID_NVIDIA:
      return "NVIDIA";
    case PCI_VENDOR_ID_PROMISE:
      return "PROMISE";
    case PCI_VENDOR_ID_SI:
      return "SiS";
    case PCI_VENDOR_ID_VIA:
      return "VIA";
    default:
      break;
    }

  return "Generic";
}

/******************************************************************************
 * return a device name for a PCI device id
 * NOTE: this is as simple as can be, so don't call it twice in one statement.
 */
char *device_from_id(u16 device)
{
  int i;

  for (i = 0; pci_ids[i].vendor != 0; i++)
  {
    if (pci_ids[i].device == device)
    {
      return pci_ids[i].chipname;
    }
  }

  return s_generic;
}

/******************************************************************************
 * Return textual version of a resource manager error
 */
static char *rmerr(APIRET ret)
{
  switch (ret) {
  case RMRC_SUCCESS:
    return("RMRC_SUCCESS");
  case RMRC_NOTINITIALIZED:
    return("RMRC_NOTINITIALIZED");
  case RMRC_BAD_DRIVERHANDLE:
    return("RMRC_BAD_DRIVERHANDLE");
  case RMRC_BAD_ADAPTERHANDLE:
    return("RMRC_BAD_ADAPTERHANDLE");
  case RMRC_BAD_DEVICEHANDLE:
    return("RMRC_BAD_DEVICEHANDLE");
  case RMRC_BAD_RESOURCEHANDLE:
    return("RMRC_BAD_RESOURCEHANDLE");
  case RMRC_BAD_LDEVHANDLE:
    return("RMRC_BAD_LDEVHANDLE");
  case RMRC_BAD_SYSNAMEHANDLE:
    return("RMRC_BAD_SYSNAMEHANDLE");
  case RMRC_BAD_DEVHELP:
    return("RMRC_BAD_DEVHELP");
  case RMRC_NULL_POINTER:
    return("RMRC_NULL_POINTER");
  case RMRC_NULL_STRINGS:
    return("RMRC_NULL_STRINGS");
  case RMRC_BAD_VERSION:
    return("RMRC_BAD_VERSION");
  case RMRC_RES_ALREADY_CLAIMED:
    return("RMRC_RES_ALREADY_CLAIMED");
  case RMRC_DEV_ALREADY_CLAIMED:
    return("RMRC_DEV_ALREADY_CLAIMED");
  case RMRC_INVALID_PARM_VALUE:
    return("RMRC_INVALID_PARM_VALUE");
  case RMRC_OUT_OF_MEMORY:
    return("RMRC_OUT_OF_MEMORY");
  case RMRC_SEARCH_FAILED:
    return("RMRC_SEARCH_FAILED");
  case RMRC_BUFFER_TOO_SMALL:
    return("RMRC_BUFFER_TOO_SMALL");
  case RMRC_GENERAL_FAILURE:
    return("RMRC_GENERAL_FAILURE");
  case RMRC_IRQ_ENTRY_ILLEGAL:
    return("RMRC_IRQ_ENTRY_ILLEGAL");
  case RMRC_NOT_IMPLEMENTED:
    return("RMRC_NOT_IMPLEMENTED");
  case RMRC_NOT_INSTALLED:
    return("RMRC_NOT_INSTALLED");
  case RMRC_BAD_DETECTHANDLE:
    return("RMRC_BAD_DETECTHANDLE");
  case RMRC_BAD_RMHANDLE:
    return("RMRC_BAD_RMHANDLE");
  case RMRC_BAD_FLAGS:
    return("RMRC_BAD_FLAGS");
  case RMRC_NO_DETECTED_DATA:
    return("RMRC_NO_DETECTED_DATA");
  default:
    return("RMRC_UNKOWN");
  }
}
