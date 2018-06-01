/*-
 * GPL LICENSE SUMMARY
 *
 *   Copyright(c) 2010-2013 Intel Corporation. All rights reserved.
 *   Copyright(c) 2013-2014 Wind River Systems. All rights reserved.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *   The full GNU General Public License is included in this distribution
 *   in the file called LICENSE.GPL.
 *
 *   Contact Information:
 *   Wind River Systems, Inc.
 */

#ifndef _AVP_COMPAT_H_
#define _AVP_COMPAT_H_

#include <linux/version.h>

#ifndef RTE_CACHE_LINE_SIZE
/* not available for kernel builds */
#define RTE_CACHE_LINE_SIZE 64
#endif

/* Custom define for dpdk user-space builds */
typedef phys_addr_t rte_iova_t;

/* Determine whether the netif_trans_update function is available */
#ifdef RHEL_RELEASE_VERSION

#if (RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(7, 4))
#define HAVE_TRANS_START_HELPER
#endif

#if (RHEL_RELEASE_CODE && (RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(7,5)))
#define HAVE_RHEL7_EXTENDED_MIN_MAX_MTU
#define HAVE_VOID_NDO_GET_STATS64
#endif

#else

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0))
#define HAVE_TRANS_START_HELPER
#endif

#endif /* RHEL_RELEASE_VERSION */

#endif /* _AVP_COMPAT_H_ */
