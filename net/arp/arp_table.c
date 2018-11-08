/****************************************************************************
 * net/arp/arp_table.c
 * Implementation of the ARP Address Resolution Protocol.
 *
 *   Copyright (C) 2007-2009, 2011, 2014, 2018 Gregory Nutt. All rights
 *     reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based originally on uIP which also has a BSD style license:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2001-2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET

#include <sys/ioctl.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <netinet/in.h>
#include <net/ethernet.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/ip.h>

#include <arp/arp.h>
#include <netdev/netdev.h>

#ifdef CONFIG_NET_ARP

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct arp_table_info_s
{
  in_addr_t              ai_ipaddr;   /* IP address for lookup */
  FAR struct ether_addr *ai_ethaddr;  /* Location to return the MAC address */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The table of known address mappings */

static struct arp_entry_s g_arptable[CONFIG_NET_ARPTAB_SIZE];
static uint8_t g_arptime;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arp_match
 *
 * Description:
 *   This is a callback that checks if the Ethernet network device has the
 *   indicated IPv4 address assigned to it.
 *
 ****************************************************************************/

static int arp_match(FAR struct net_driver_s *dev, FAR void *arg)
{
  FAR struct arp_table_info_s *info = arg;

  /* Make sure that this is an Ethernet device (or an IEEE 802.11 device
   * which is also Ethernet)
   */

  if (dev->d_lltype != NET_LL_ETHERNET &&
      dev->d_lltype != NET_LL_IEEE80211)
    {
      return 0;
    }

  /* Check if the network device has been assigned the IP address of the
   * lookup.
   */

  if (!net_ipv4addr_cmp(dev->d_ipaddr, info->ai_ipaddr))
    {
      return 0;
    }

  /* Yes.. Return the matching Ethernet MAC address if the caller of
   * arp_find() provided a non-NULL location.
   */

  if (info->ai_ethaddr != NULL)
    {
      memcpy(info->ai_ethaddr, &dev->d_mac.ether, ETHER_ADDR_LEN);
    }

  /* Return success in any event */

  return 1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arp_reset
 *
 * Description:
 *   Re-initialize the ARP table.
 *
 ****************************************************************************/

void arp_reset(void)
{
  int i;

  for (i = 0; i < CONFIG_NET_ARPTAB_SIZE; ++i)
    {
      memset(&g_arptable[i].at_ipaddr, 0, sizeof(in_addr_t));
    }
}

/****************************************************************************
 * Name: arp_timer
 *
 * Description:
 *   This function performs periodic timer processing in the ARP module
 *   and should be called at regular intervals. The recommended interval
 *   is 10 seconds between the calls.  It is responsible for flushing old
 *   entries in the ARP table.
 *
 ****************************************************************************/

void arp_timer(void)
{
  FAR struct arp_entry_s *tabptr;
  int i;

  ++g_arptime;
  for (i = 0; i < CONFIG_NET_ARPTAB_SIZE; ++i)
    {
      tabptr = &g_arptable[i];

      if (tabptr->at_ipaddr != 0 &&
          g_arptime - tabptr->at_time >= CONFIG_NET_ARP_MAXAGE)
        {
          tabptr->at_ipaddr = 0;
        }
    }
}

/****************************************************************************
 * Name: arp_update
 *
 * Description:
 *   Add the IP/HW address mapping to the ARP table -OR- change the IP
 *   address of an existing association.
 *
 * Input Parameters:
 *   ipaddr  - The IP address as an inaddr_t
 *   ethaddr - Refers to a HW address uint8_t[IFHWADDRLEN]
 *
 * Returned Value:
 *   Zero (OK) if the ARP table entry was successfully modified.  A negated
 *   errno value is returned on any error.
 *
 * Assumptions
 *   The network is locked to assure exclusive access to the ARP table
 *
 ****************************************************************************/

int arp_update(in_addr_t ipaddr, FAR uint8_t *ethaddr)
{
  struct arp_entry_s *tabptr = NULL;
  int               i;

  /* Walk through the ARP mapping table and try to find an entry to
   * update. If none is found, the IP -> MAC address mapping is
   * inserted in the ARP table.
   */

  for (i = 0; i < CONFIG_NET_ARPTAB_SIZE; ++i)
    {
      tabptr = &g_arptable[i];

      /* Only check those entries that are actually in use. */

      if (tabptr->at_ipaddr != 0)
        {
          /* Check if the source IP address of the incoming packet matches
           * the IP address in this ARP table entry.
           */

          if (net_ipv4addr_cmp(ipaddr, tabptr->at_ipaddr))
            {
              /* An old entry found, update this and return. */

              memcpy(tabptr->at_ethaddr.ether_addr_octet, ethaddr, ETHER_ADDR_LEN);
              tabptr->at_time = g_arptime;
              return OK;
            }
        }
    }

  /* If we get here, no existing ARP table entry was found, so we create one. */
  /* First, we try to find an unused entry in the ARP table. */

  for (i = 0; i < CONFIG_NET_ARPTAB_SIZE; ++i)
    {
      tabptr = &g_arptable[i];
      if (tabptr->at_ipaddr == 0)
        {
          break;
        }
    }

  /* If no unused entry is found, we try to find the oldest entry and
   * throw it away.
   */

  if (i == CONFIG_NET_ARPTAB_SIZE)
    {
      uint8_t tmpage = 0;
      int j = 0;

      for (i = 0; i < CONFIG_NET_ARPTAB_SIZE; ++i)
        {
          tabptr = &g_arptable[i];
          if (g_arptime - tabptr->at_time > tmpage)
            {
              tmpage = g_arptime - tabptr->at_time;
              j = i;
            }
        }

      i = j;
      tabptr = &g_arptable[i];
    }

  /* Now, i is the ARP table entry which we will fill with the new
   * information.
   */

  tabptr->at_ipaddr = ipaddr;
  memcpy(tabptr->at_ethaddr.ether_addr_octet, ethaddr, ETHER_ADDR_LEN);
  tabptr->at_time = g_arptime;
  return OK;
}

/****************************************************************************
 * Name: arp_hdr_update
 *
 * Description:
 *   Add the IP/HW address mapping to the ARP table -OR- change the IP
 *   address of an existing association.
 *
 * Input Parameters:
 *   pipaddr - Refers to an IP address uint16_t[2] in network order
 *   ethaddr - Refers to a HW address uint8_t[IFHWADDRLEN]
 *
 * Returned Value:
 *   Zero (OK) if the ARP table entry was successfully modified.  A negated
 *   errno value is returned on any error.
 *
 * Assumptions
 *   The network is locked to assure exclusive access to the ARP table
 *
 ****************************************************************************/

void arp_hdr_update(FAR uint16_t *pipaddr, FAR uint8_t *ethaddr)
{
  in_addr_t ipaddr = net_ip4addr_conv32(pipaddr);

  /* Update the ARP table */

  (void)arp_update(ipaddr, ethaddr);
}

/****************************************************************************
 * Name: arp_lookup
 *
 * Description:
 *   Find the ARP entry corresponding to this IP address in the ARP table.
 *
 * Input Parameters:
 *   ipaddr - Refers to an IP address in network order
 *
 * Assumptions:
 *   The network is locked to assure exclusive access to the ARP table.
 *   The return value will become unstable when the network is unlocked.
 *
 ****************************************************************************/

FAR struct arp_entry_s *arp_lookup(in_addr_t ipaddr)
{
  FAR struct arp_entry_s *tabptr;
  int i;

  /* Check if the IPv4 address is already in the ARP table. */

  for (i = 0; i < CONFIG_NET_ARPTAB_SIZE; ++i)
    {
      tabptr = &g_arptable[i];
      if (net_ipv4addr_cmp(ipaddr, tabptr->at_ipaddr))
        {
          return tabptr;
        }
    }

  /* Not found */

  return NULL;
}

/****************************************************************************
 * Name: arp_find
 *
 * Description:
 *   Find the ARP entry corresponding to this IP address which may or may
 *   not be in the ARP table (it may, instead, be a local network device).
 *
 * Input Parameters:
 *   ipaddr -  Refers to an IP address in network order
 *   ethaddr - Location to return the corresponding Ethernet MAN address.
 *             This address may be NULL.  In that case, this function may be
 *             used simply to determine if the Ethernet MAC address is
 *             available.
 *
 * Assumptions
 *   The network is locked to assure exclusive access to the ARP table.
 *
 ****************************************************************************/

int arp_find(in_addr_t ipaddr, FAR struct ether_addr *ethaddr)
{
  FAR struct arp_entry_s *tabptr;
  struct arp_table_info_s info;

  /* Check if the IPv4 address is already in the ARP table. */

  tabptr = arp_lookup(ipaddr);
  if (tabptr != NULL)
    {
      /* Yes.. return the Ethernet MAC address if the caller has provided a
       * non-NULL address in 'ethaddr'.
       */

      if (ethaddr != NULL)
        {
          memcpy(ethaddr, &tabptr->at_ethaddr, ETHER_ADDR_LEN);
        }

      /* Return success in any case meaning that a valid Ethernet MAC
       * address mapping is available for the IP address.
       */

      return OK;
    }

  /* No.. check if the IPv4 address is the address assigned to a local
   * Ethernet network device.  If so, return a mapping of that IP address
   * to the Ethernet MAC address assigned to the network device.
   */

  info.ai_ipaddr  = ipaddr;
  info.ai_ethaddr = ethaddr;

  if (netdev_foreach(arp_match, &info) != 0)
    {
      return OK;
    }

  /* Not found */

  return -ENOENT;
}

/****************************************************************************
 * Name: arp_delete
 *
 * Description:
 *   Remove an IP association from the ARP table
 *
 * Input Parameters:
 *   ipaddr - Refers to an IP address in network order
 *
 * Assumptions
 *   The network is locked to assure exclusive access to the ARP table.
 *
 ****************************************************************************/

void arp_delete(in_addr_t ipaddr)
{
  FAR struct arp_entry_s *tabptr;

  /* Check if the IPv4 address is in the ARP table. */

  tabptr = arp_lookup(ipaddr);
  if (tabptr != NULL)
    {
      /* Yes.. Set the IP address to zero to "delete" it */

      tabptr->at_ipaddr = 0;
    }
}

#endif /* CONFIG_NET_ARP */
#endif /* CONFIG_NET */
