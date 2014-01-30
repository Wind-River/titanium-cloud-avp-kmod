/*-
 * GPL LICENSE SUMMARY
 *
 *   Copyright(c) 2010-2013 Intel Corporation. All rights reserved.
 *   Copyright(c) 2014 Wind River Systems, Inc. All rights reserved.
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

#include <linux/device.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/netdevice.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/percpu.h>
#include <linux/jiffies.h>

#include <exec-env/wrs_avp_common.h>
#include <avp_fifo.h>
#include "avp_dev.h"
#include "avp_ctrl.h"

/* Control request response timeout in milliseconds */
#define WRS_AVP_CTRL_RESPONSE_TIMEOUT 500

int
avp_ctrl_set_link_state(struct avp_dev *avp, unsigned state)
{
	struct wrs_avp_request req;
	int ret;

	memset(&req, 0, sizeof(req));
	req.req_id = WRS_AVP_REQ_CFG_NETWORK_IF;
	req.if_up = state;
	ret = avp_ctrl_process_request(avp, &req);

	return (ret == 0 ? req.result : ret);
}

int
avp_ctrl_set_mtu(struct avp_dev *avp, int new_mtu)
{
	struct wrs_avp_request req;
	int ret;

	memset(&req, 0, sizeof(req));
	req.req_id = WRS_AVP_REQ_CHANGE_MTU;
	req.new_mtu = new_mtu;

	ret = avp_ctrl_process_request(avp, &req);

	return (ret == 0 ? req.result : ret);
}

int
avp_ctrl_set_config(struct avp_dev *avp, struct wrs_avp_device_config *config)
{
	struct wrs_avp_request req;
	int ret;

	memset(&req, 0, sizeof(req));
	req.req_id = WRS_AVP_REQ_CFG_DEVICE;
	memcpy(&req.config, config, sizeof(req.config));

	ret = avp_ctrl_process_request(avp, &req);

	return (ret == 0 ? req.result : ret);
}

int
avp_ctrl_shutdown(struct avp_dev *avp)
{
	struct wrs_avp_request req;
	int ret;

	memset(&req, 0, sizeof(req));
	req.req_id = WRS_AVP_REQ_SHUTDOWN_DEVICE;

	ret = avp_ctrl_process_request(avp, &req);

	return (ret == 0 ? req.result : ret);
}

void
avp_ctrl_poll_resp(struct avp_dev *avp)
{
	if (avp_fifo_count(avp->resp_q))
		wake_up_interruptible(&avp->wq);
}

int
avp_ctrl_process_request(struct avp_dev *avp, struct wrs_avp_request *req)
{
	int ret = -1;
	void *resp_va;
	unsigned num;
	unsigned retry;

	if (!avp || !req) {
		AVP_ERR("No AVP instance or request\n");
		return -EINVAL;
	}

	req->result = -ENOTSUPP;
	AVP_DBG("Sending request %u\n", req->req_id);

	mutex_lock(&avp->sync_lock);

	/* Discard any stale responses before starting a new request */
	while (avp_fifo_get(avp->resp_q, (void**)&resp_va, 1)) {
		AVP_DBG("Discarding stale response\n");
	}

	/* Construct data */
	memcpy(avp->sync_kva, req, sizeof(struct wrs_avp_request));
	num = avp_fifo_put(avp->req_q, &avp->sync_va, 1);
	if (num < 1) {
		AVP_ERR("Cannot send to req_q\n");
		ret = -EBUSY;
		goto fail;
	}

	retry = 1;
	do {
		ret = wait_event_interruptible_timeout(
				  avp->wq,
				  avp_fifo_count(avp->resp_q),
				  msecs_to_jiffies(WRS_AVP_CTRL_RESPONSE_TIMEOUT));
	} while (ret <= 0 && retry--);

	if (signal_pending(current) || ret <= 0) {
		AVP_ERR("No response to request %u, ret=%d\n", req->req_id, ret);
		ret = -ETIME;
		goto fail;
	}

	AVP_DBG("Response received for %u after %lu/%lu jiffies\n",
			req->req_id,
			msecs_to_jiffies(WRS_AVP_CTRL_RESPONSE_TIMEOUT) - ret,
			msecs_to_jiffies(WRS_AVP_CTRL_RESPONSE_TIMEOUT));

	num = avp_fifo_get(avp->resp_q, (void **)&resp_va, 1);
	if (num != 1 || resp_va != avp->sync_va) {
		/* This should never happen */
		AVP_ERR("No data in resp_q\n");
		ret = -ENODATA;
		goto fail;
	}

	memcpy(req, avp->sync_kva, sizeof(struct wrs_avp_request));
	ret = 0;

	AVP_DBG("Result %d received for request %u\n",
			req->result, req->req_id);

fail:
	mutex_unlock(&avp->sync_lock);
	return ret;
}
