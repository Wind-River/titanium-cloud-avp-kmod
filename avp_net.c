/*-
 * GPL LICENSE SUMMARY
 *
 *   Copyright(c) 2010-2013 Intel Corporation. All rights reserved.
 *   Copyright(c) 2013-2014 Intel Corporation. All rights reserved.
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
 *   Intel Corporation
 */

/*
 * This code is inspired from the book "Linux Device Drivers" by
 * Alessandro Rubini and Jonathan Corbet, published by O'Reilly & Associates
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h> /* eth_type_trans */
#include <linux/skbuff.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/percpu.h>
#include <net/arp.h>

#include <exec-env/wrs_avp_common.h>
#include <avp_fifo.h>
#include "avp_dev.h"
#include "avp_ctrl.h"

/* Defines number of jiffies used to configure the skbuff watchdog */
#define WRS_AVP_WD_TIMEOUT 5

/* Defines the maximum number of packets to be received in one try */
#define WRS_AVP_MBUF_BURST_SZ 32

int avp_net_rx(struct avp_dev *avp, unsigned qnum);

static int
avp_net_open(struct net_device *dev)
{
	struct avp_dev *avp = netdev_priv(dev);

	if (avp->status == WRS_AVP_DEV_STATUS_DETACHED) {
		return -EBUSY;
	}

	if (!is_zero_ether_addr(avp->ethaddr))
		memcpy(dev->dev_addr, avp->ethaddr, ETH_ALEN);
	else
		/*
		 * Generate random mac address. eth_random_addr() is the newer
		 * version of generating mac address in linux kernel.
		 */
		random_ether_addr(dev->dev_addr);

	netif_tx_start_all_queues(dev);

	return avp_ctrl_set_link_state(avp, 1);
}

static int
avp_net_release(struct net_device *dev)
{
	struct avp_dev *avp = netdev_priv(dev);

	if (avp->status == WRS_AVP_DEV_STATUS_DETACHED) {
		return -EBUSY;
	}

	netif_tx_stop_all_queues(dev);

	return avp_ctrl_set_link_state(avp, 0);
}

static int
avp_net_config(struct net_device *dev, struct ifmap *map)
{
	if (dev->flags & IFF_UP)
		return -EBUSY;

	return 0;
}

static inline void *
avp_net_translate_buffer(struct avp_dev *avp, void *addr)
{
	struct avp_mempool_info *pool = &avp->pool[0];

	if (likely((addr >= pool->va) && (addr < (pool->va + pool->length)))) {
		return addr - pool->va + pool->kva;
	} else {
		pool = &avp->pool[1];
		if (likely((addr >= pool->va) && (addr < (pool->va + pool->length)))) {
			return addr - pool->va + pool->kva;
		}
	}

	BUG_ON(0);
	return NULL;
}

int
avp_net_rx(struct avp_dev *avp, unsigned qnum)
{
	unsigned ret;
	unsigned i, num, num_rq, num_fq;
	struct wrs_avp_mbuf *avp_bufs[WRS_AVP_MBUF_BURST_SZ];
	struct wrs_avp_mbuf *pkt_buf;
	void *pkt_data;
	uint32_t pkt_len;
	struct sk_buff *skb;
	struct net_device *dev = avp->net_dev;
	struct avp_stats *stats = this_cpu_ptr(avp->stats);
	struct wrs_avp_fifo *rx_q = avp->rx_q[qnum];
	struct wrs_avp_fifo *free_q = avp->free_q[qnum];

	/* Get the number of entries in rx_q */
	num_rq = avp_fifo_count(rx_q);

	/* Get the number of free entries in free_q */
	num_fq = avp_fifo_free_count(free_q);
	if (num_fq == 0)
		stats->rx_fifo_errors++;

	/* Calculate the number of entries to dequeue in rx_q */
	num = min(num_rq, num_fq);
	num = min(num, (unsigned)WRS_AVP_MBUF_BURST_SZ);

	/* Return if no entry in rx_q and no free entry in free_q */
	if (num == 0)
		return 0;

	/* Burst dequeue from rx_q */
	ret = avp_fifo_get(rx_q, (void **)avp_bufs, num);
	if (ret == 0)
		return 0; /* Failing should not happen */

	/* Transfer received packets to netif */
	for (i = 0; i < num; i++) {

		/* prefetch next entry while process current one */
		if (i < num-1) {
			pkt_buf = avp_net_translate_buffer(avp, (void*)avp_bufs[i+1]);
			prefetch(pkt_buf);
		}

		pkt_buf = avp_net_translate_buffer(avp, (void*)avp_bufs[i]);
		pkt_data = avp_net_translate_buffer(avp, pkt_buf->data);
		pkt_len = pkt_buf->data_len;

		skb = __dev_alloc_skb(pkt_len + NET_IP_ALIGN, GFP_ATOMIC);
		if (unlikely(!skb)) {
			/* Update statistics */
			stats->rx_dropped += (num-i);
			break;
		}

		/* Align IP on 16B boundary */
		skb_reserve(skb, NET_IP_ALIGN);
		memcpy(skb_put(skb, pkt_len), pkt_data, pkt_len);
		skb->dev = dev;
		skb->protocol = eth_type_trans(skb, dev);
		skb->ip_summed = CHECKSUM_UNNECESSARY;

		skb_record_rx_queue(skb, qnum);

		/* Call netif interface */
		netif_rx(skb);

		/* Update statistics */
		u64_stats_update_begin(&stats->rx_syncp);
		stats->rx_bytes += pkt_len;
		stats->rx_packets++;
		u64_stats_update_end(&stats->rx_syncp);
	}

	/* Burst enqueue mbufs into free_q */
	ret = avp_fifo_put(free_q, (void **)avp_bufs, num);
	if (unlikely(ret != num))
		/* Failing should not happen */
		AVP_ERR("Fail to enqueue entries into free_q\n");

	return num;
}

static int
avp_net_tx(struct sk_buff *skb, struct net_device *dev)
{
	int len = 0;
	unsigned ret;
	unsigned i, num, num_aq;
	struct avp_dev *avp = netdev_priv(dev);
	struct avp_stats *stats = this_cpu_ptr(avp->stats);
	struct avp_mbuf_cache *mbuf_cache;
	struct wrs_avp_mbuf *pkt_kva = NULL;
	struct wrs_avp_mbuf *pkt_va = NULL;
	struct wrs_avp_fifo *tx_q;
	struct wrs_avp_fifo *alloc_q;
	void *data_kva;
	unsigned qnum;

	dev->trans_start = jiffies; /* save the timestamp */

	/* Check if the length of skb is less than mbuf size */
	if (skb->len > avp->mbuf_size)
		goto drop;

	qnum = skb_get_queue_mapping(skb);
	BUG_ON(qnum > avp->num_tx_queues);

	tx_q = avp->tx_q[qnum];
	alloc_q = avp->alloc_q[qnum];
	mbuf_cache = &avp->mbuf_cache[qnum];

	/**
	 * Check if it has at least one free entry in tx_q and
	 * one entry in alloc_q.
	 */
	if (avp_fifo_free_count(tx_q) == 0) {
		goto drop;
	}

	if (mbuf_cache->count == 0) {
		num_aq = avp_fifo_count(alloc_q);
		if (num_aq == 0) {
			stats->tx_fifo_errors++;
			goto drop;
		}

		num = min(num_aq, (unsigned)WRS_AVP_QUEUE_MBUF_CACHE_SIZE);

		/* dequeue a mbufs from alloc_q */
		ret = avp_fifo_get(alloc_q, (void **)mbuf_cache->mbufs, num);
		if (ret != num) {
			/* Failing should not happen */
			AVP_ERR("Fail to enqueue mbuf into tx_q\n");
			goto drop;
		}
		mbuf_cache->count = num;
		for (i = 0; i < num; i++) {
			pkt_kva = avp_net_translate_buffer(avp, (void *)mbuf_cache->mbufs[i]);
			prefetch(pkt_kva);
		}
	}

	pkt_va = mbuf_cache->mbufs[--mbuf_cache->count];
	pkt_kva = avp_net_translate_buffer(avp, (void *)pkt_va);
	data_kva = avp_net_translate_buffer(avp, pkt_kva->data);

	len = skb->len;
	memcpy(data_kva, skb->data, len);
	if (unlikely(len < ETH_ZLEN)) {
		memset(data_kva + len, 0, ETH_ZLEN - len);
		len = ETH_ZLEN;
	}
	pkt_kva->pkt_len = len;
	pkt_kva->data_len = len;

	/* enqueue mbuf into tx_q */
	ret = avp_fifo_put(tx_q, (void **)&pkt_va, 1);
	if (unlikely(ret != 1)) {
		/* Failing should not happen */
		AVP_ERR("Fail to enqueue mbuf into tx_q\n");
		goto drop;
	}

	/* Free skb and update statistics */
	dev_kfree_skb(skb);

	u64_stats_update_begin(&stats->tx_syncp);
	stats->tx_bytes += len;
	stats->tx_packets++;
	u64_stats_update_end(&stats->tx_syncp);

	return NETDEV_TX_OK;

drop:
	/* Free skb and update statistics */
	dev_kfree_skb(skb);
	stats->tx_dropped++;

	return NETDEV_TX_OK;
}

#ifdef WRS_AVP_TX_TIMEOUTS
static void
avp_net_tx_timeout (struct net_device *dev)
{
	struct avp_dev *avp = netdev_priv(dev);
	struct avp_stats *stats = this_cpu_ptr(avp->stats);

	AVP_DBG("transmit timeout at %ld, latency %ld\n", jiffies,
			jiffies - dev->trans_start);

	stats->tx_errors++;
	netif_wake_queue(dev);
	return;
}
#endif

static int
avp_net_change_mtu(struct net_device *dev, int new_mtu)
{
	struct avp_dev *avp = netdev_priv(dev);
	int ret;

	AVP_DBG("%s updating mtu to %u\n", new_mtu);

	ret = avp_ctrl_set_mtu(avp, new_mtu);
	if (ret == 0)
		dev->mtu = new_mtu;

	return 0;
}

static struct rtnl_link_stats64 *
avp_net_stats(struct net_device *dev, struct rtnl_link_stats64 *tot)
{
	struct avp_dev *avp = netdev_priv(dev);
	int cpu;
	unsigned int start;

	for_each_possible_cpu(cpu) {
		struct avp_stats *stats = per_cpu_ptr(avp->stats, cpu);
		u64 rx_packets, tx_packets, rx_bytes, tx_bytes;
		u64 rx_errors, tx_errors, rx_dropped, tx_dropped;
		u64 rx_fifo_errors, tx_fifo_errors;

		do {
			start = u64_stats_fetch_begin(&stats->tx_syncp);
			tx_packets = stats->tx_packets;
			tx_bytes = stats->tx_bytes;
			tx_errors = stats->tx_errors;
			tx_dropped = stats->tx_dropped;
			tx_fifo_errors = stats->tx_fifo_errors;
		} while (u64_stats_fetch_retry(&stats->tx_syncp, start));

		do {
			start = u64_stats_fetch_begin(&stats->rx_syncp);
			rx_packets = stats->rx_packets;
			rx_bytes = stats->rx_bytes;
			rx_errors = stats->rx_errors;
			rx_dropped = stats->rx_dropped;
			rx_fifo_errors = stats->rx_fifo_errors;
		} while (u64_stats_fetch_retry(&stats->rx_syncp, start));

		tot->rx_packets += rx_packets;
		tot->tx_packets += tx_packets;
		tot->rx_bytes += rx_bytes;
		tot->tx_bytes += tx_bytes;
		tot->rx_errors += rx_errors;
		tot->tx_errors += tx_errors;
		tot->rx_dropped += rx_dropped;
		tot->tx_dropped += tx_dropped;
		tot->rx_fifo_errors += rx_fifo_errors;
		tot->tx_fifo_errors += tx_fifo_errors;
	}

	return tot;
}

static int
avp_net_header(struct sk_buff *skb, struct net_device *dev,
		unsigned short type, const void *daddr,
		const void *saddr, unsigned int len)
{
	struct ethhdr *eth = (struct ethhdr *) skb_push(skb, ETH_HLEN);

	memcpy(eth->h_source, saddr ? saddr : dev->dev_addr, dev->addr_len);
	memcpy(eth->h_dest,	  daddr ? daddr : dev->dev_addr, dev->addr_len);
	eth->h_proto = htons(type);

	return (dev->hard_header_len);
}


static int
avp_net_rebuild_header(struct sk_buff *skb)
{
	struct net_device *dev = skb->dev;
	struct ethhdr *eth = (struct ethhdr *) skb->data;

	memcpy(eth->h_source, dev->dev_addr, dev->addr_len);
	memcpy(eth->h_dest, dev->dev_addr, dev->addr_len);

	return 0;
}


static const struct header_ops avp_net_header_ops = {
	.create	 = avp_net_header,
	.rebuild = avp_net_rebuild_header,
	.cache	 = NULL,  /* disable caching */
};

static const struct net_device_ops avp_net_netdev_ops = {
	.ndo_open = avp_net_open,
	.ndo_stop = avp_net_release,
	.ndo_set_config = avp_net_config,
	.ndo_start_xmit = avp_net_tx,
	.ndo_change_mtu = avp_net_change_mtu,
	.ndo_get_stats64 = avp_net_stats,
#ifdef WRS_AVP_TX_TIMEOUTS
	.ndo_tx_timeout = avp_net_tx_timeout,
#endif
};

void
avp_net_init(struct net_device *dev)
{
	struct avp_dev *avp = netdev_priv(dev);

	AVP_DBG("avp_net_init\n");

	init_waitqueue_head(&avp->wq);
	mutex_init(&avp->sync_lock);

	ether_setup(dev); /* assign some of the fields */
	dev->netdev_ops		 = &avp_net_netdev_ops;
	dev->header_ops		 = &avp_net_header_ops;
#ifdef WRS_AVP_TX_TIMEOUTS
	dev->watchdog_timeo = WRS_AVP_WD_TIMEOUT;
#endif
}
