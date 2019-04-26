/*
 * Copyright (c) 2014-2017 Qualcomm Atheros, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "core.h"
#include "wmi-ops.h"
#include "debug.h"

static void ath10k_sta_update_extd_stats_rx_duration(struct ath10k *ar,
						     struct ath10k_fw_stats *stats)
{
	struct ath10k_fw_extd_stats_peer *peer;
	struct ieee80211_sta *sta;
	struct ath10k_sta *arsta;

	rcu_read_lock();
	list_for_each_entry(peer, &stats->peers_extd, list) {
		sta = ieee80211_find_sta_by_ifaddr(ar->hw, peer->peer_macaddr,
						   NULL);
		if (!sta)
			continue;
		arsta = (struct ath10k_sta *)sta->drv_priv;
		arsta->rx_duration += (u64)peer->rx_duration;
	}
	rcu_read_unlock();
}

static void ath10k_sta_update_stats_rx_duration(struct ath10k *ar,
						struct ath10k_fw_stats *stats)
{
	struct ath10k_fw_stats_peer *peer;
	struct ieee80211_sta *sta;
	struct ath10k_sta *arsta;

	rcu_read_lock();
	list_for_each_entry(peer, &stats->peers, list) {
		sta = ieee80211_find_sta_by_ifaddr(ar->hw, peer->peer_macaddr,
						   NULL);
		if (!sta)
			continue;
		arsta = (struct ath10k_sta *)sta->drv_priv;
		arsta->rx_duration += (u64)peer->rx_duration;
	}
	rcu_read_unlock();
}

void ath10k_sta_update_rx_duration(struct ath10k *ar,
				   struct ath10k_fw_stats *stats)
{
	if (stats->extended)
		ath10k_sta_update_extd_stats_rx_duration(ar, stats);
	else
		ath10k_sta_update_stats_rx_duration(ar, stats);
}

static ssize_t ath10k_dbg_sta_read_aggr_mode(struct file *file,
					     char __user *user_buf,
					     size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	char buf[32];
	int len = 0;

	mutex_lock(&ar->conf_mutex);
	len = scnprintf(buf, sizeof(buf) - len, "aggregation mode: %s\n",
			(arsta->aggr_mode == ATH10K_DBG_AGGR_MODE_AUTO) ?
			"auto" : "manual");
	mutex_unlock(&ar->conf_mutex);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t ath10k_dbg_sta_write_aggr_mode(struct file *file,
					      const char __user *user_buf,
					      size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	u32 aggr_mode;
	int ret;

	if (kstrtouint_from_user(user_buf, count, 0, &aggr_mode))
		return -EINVAL;

	if (aggr_mode >= ATH10K_DBG_AGGR_MODE_MAX)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);
	if ((ar->state != ATH10K_STATE_ON) ||
	    (aggr_mode == arsta->aggr_mode)) {
		ret = count;
		goto out;
	}

	ret = ath10k_wmi_addba_clear_resp(ar, arsta->arvif->vdev_id, sta->addr);
	if (ret) {
		ath10k_warn(ar, "failed to clear addba session ret: %d\n", ret);
		goto out;
	}

	arsta->aggr_mode = aggr_mode;
out:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static const struct file_operations fops_aggr_mode = {
	.read = ath10k_dbg_sta_read_aggr_mode,
	.write = ath10k_dbg_sta_write_aggr_mode,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_write_addba(struct file *file,
					  const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	u32 tid, buf_size;
	int ret;
	char buf[64];

	simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);

	/* make sure that buf is null terminated */
	buf[sizeof(buf) - 1] = '\0';

	ret = sscanf(buf, "%u %u", &tid, &buf_size);
	if (ret != 2)
		return -EINVAL;

	/* Valid TID values are 0 through 15 */
	if (tid > HTT_DATA_TX_EXT_TID_MGMT - 2)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);
	if ((ar->state != ATH10K_STATE_ON) ||
	    (arsta->aggr_mode != ATH10K_DBG_AGGR_MODE_MANUAL)) {
		ret = count;
		goto out;
	}

	ret = ath10k_wmi_addba_send(ar, arsta->arvif->vdev_id, sta->addr,
				    tid, buf_size);
	if (ret) {
		ath10k_warn(ar, "failed to send addba request: vdev_id %u peer %pM tid %u buf_size %u\n",
			    arsta->arvif->vdev_id, sta->addr, tid, buf_size);
	}

	ret = count;
out:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static const struct file_operations fops_addba = {
	.write = ath10k_dbg_sta_write_addba,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_write_addba_resp(struct file *file,
					       const char __user *user_buf,
					       size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	u32 tid, status;
	int ret;
	char buf[64];

	simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);

	/* make sure that buf is null terminated */
	buf[sizeof(buf) - 1] = '\0';

	ret = sscanf(buf, "%u %u", &tid, &status);
	if (ret != 2)
		return -EINVAL;

	/* Valid TID values are 0 through 15 */
	if (tid > HTT_DATA_TX_EXT_TID_MGMT - 2)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);
	if ((ar->state != ATH10K_STATE_ON) ||
	    (arsta->aggr_mode != ATH10K_DBG_AGGR_MODE_MANUAL)) {
		ret = count;
		goto out;
	}

	ret = ath10k_wmi_addba_set_resp(ar, arsta->arvif->vdev_id, sta->addr,
					tid, status);
	if (ret) {
		ath10k_warn(ar, "failed to send addba response: vdev_id %u peer %pM tid %u status%u\n",
			    arsta->arvif->vdev_id, sta->addr, tid, status);
	}
	ret = count;
out:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static const struct file_operations fops_addba_resp = {
	.write = ath10k_dbg_sta_write_addba_resp,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_write_delba(struct file *file,
					  const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	u32 tid, initiator, reason;
	int ret;
	char buf[64];

	simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);

	/* make sure that buf is null terminated */
	buf[sizeof(buf) - 1] = '\0';

	ret = sscanf(buf, "%u %u %u", &tid, &initiator, &reason);
	if (ret != 3)
		return -EINVAL;

	/* Valid TID values are 0 through 15 */
	if (tid > HTT_DATA_TX_EXT_TID_MGMT - 2)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);
	if ((ar->state != ATH10K_STATE_ON) ||
	    (arsta->aggr_mode != ATH10K_DBG_AGGR_MODE_MANUAL)) {
		ret = count;
		goto out;
	}

	ret = ath10k_wmi_delba_send(ar, arsta->arvif->vdev_id, sta->addr,
				    tid, initiator, reason);
	if (ret) {
		ath10k_warn(ar, "failed to send delba: vdev_id %u peer %pM tid %u initiator %u reason %u\n",
			    arsta->arvif->vdev_id, sta->addr, tid, initiator,
			    reason);
	}
	ret = count;
out:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static const struct file_operations fops_delba = {
	.write = ath10k_dbg_sta_write_delba,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_read_rx_duration(struct file *file,
					       char __user *user_buf,
					       size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	char buf[100];
	int len = 0;

	len = scnprintf(buf, sizeof(buf),
			"%llu usecs\n", arsta->rx_duration);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_rx_duration = {
	.read = ath10k_dbg_sta_read_rx_duration,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_read_peer_debug_trigger(struct file *file,
						      char __user *user_buf,
						      size_t count,
						      loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	char buf[8];
	int len = 0;

	mutex_lock(&ar->conf_mutex);
	len = scnprintf(buf, sizeof(buf) - len,
			"Write 1 to once trigger the debug logs\n");
	mutex_unlock(&ar->conf_mutex);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t
ath10k_dbg_sta_write_peer_debug_trigger(struct file *file,
					const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	u8 peer_debug_trigger;
	int ret;

	if (kstrtou8_from_user(user_buf, count, 0, &peer_debug_trigger))
		return -EINVAL;

	if (peer_debug_trigger != 1)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);

	if (ar->state != ATH10K_STATE_ON) {
		ret = -ENETDOWN;
		goto out;
	}

	ret = ath10k_wmi_peer_set_param(ar, arsta->arvif->vdev_id, sta->addr,
					ar->wmi.peer_param->debug, peer_debug_trigger);
	if (ret) {
		ath10k_warn(ar, "failed to set param to trigger peer tid logs for station ret: %d\n",
			    ret);
		goto out;
	}
out:
	mutex_unlock(&ar->conf_mutex);
	return count;
}

static const struct file_operations fops_peer_debug_trigger = {
	.open = simple_open,
	.read = ath10k_dbg_sta_read_peer_debug_trigger,
	.write = ath10k_dbg_sta_write_peer_debug_trigger,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_write_cfr_capture(struct file *file,
						const char __user *user_buf,
						size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	struct wmi_peer_cfr_capture_conf_arg arg;
	u32 per_peer_cfr_status = 0, per_peer_cfr_bw  = 0;
	u32 per_peer_cfr_method = 0, per_peer_cfr_period = 0;
	int ret;
	char buf[64] = {0};

	simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);

	mutex_lock(&ar->conf_mutex);

	if (ar->state != ATH10K_STATE_ON) {
		ret = -ENETDOWN;
		goto out;
	}

	ret = sscanf(buf, "%u %u %u %u", &per_peer_cfr_status, &per_peer_cfr_bw,
		     &per_peer_cfr_period, &per_peer_cfr_method);

	if (ret < 1) {
		ret = -EINVAL;
		goto out;
	}

	if (per_peer_cfr_status && ret != 4) {
		ret = -EINVAL;
		goto out;
	}

	if (per_peer_cfr_status == arsta->cfr_capture.cfr_enable &&
	    (per_peer_cfr_period &&
	    per_peer_cfr_period == arsta->cfr_capture.cfr_period) &&
	    per_peer_cfr_bw == arsta->cfr_capture.cfr_bandwidth &&
	    per_peer_cfr_method == arsta->cfr_capture.cfr_method) {
		ret = count;
		goto out;
	}

	if (per_peer_cfr_status > WMI_PEER_CFR_CAPTURE_ENABLE ||
	    per_peer_cfr_status < WMI_PEER_CFR_CAPTURE_DISABLE ||
	    per_peer_cfr_bw >= ar->hw_params.max_cfr_capture_bw ||
	    per_peer_cfr_bw < WMI_PEER_CFR_CAPTURE_BW_20MHZ ||
	    per_peer_cfr_method < WMI_PEER_CFR_CAPTURE_METHOD_NULL_FRAME ||
	    per_peer_cfr_method >= WMI_PEER_CFR_CAPTURE_METHOD_MAX ||
	    per_peer_cfr_period < WMI_PEER_CFR_PERIODICITY_MIN ||
	    per_peer_cfr_period > WMI_PEER_CFR_PERIODICITY_MAX) {
		ret = -EINVAL;
		goto out;
	}

	/*TODO: Need rework when base time is configurable*/
	if (per_peer_cfr_period % 10) {
		ret = -EINVAL;
		goto out;
	}

	/*TODO:Need correction for 80+80 MHz*/
	if (per_peer_cfr_bw > sta->bandwidth) {
		ret = -EINVAL;
		goto out;
	}

	if (!per_peer_cfr_status) {
		per_peer_cfr_bw = arsta->cfr_capture.cfr_bandwidth;
		per_peer_cfr_period = arsta->cfr_capture.cfr_period;
		per_peer_cfr_method = arsta->cfr_capture.cfr_method;
	}

	arg.request = per_peer_cfr_status;
	arg.periodicity = per_peer_cfr_period;
	arg.bandwidth = per_peer_cfr_bw;
	arg.capture_method = per_peer_cfr_method;

	ret = ath10k_wmi_peer_set_cfr_capture_conf(ar, arsta->arvif->vdev_id,
						   sta->addr, &arg);
	if (ret) {
		ath10k_warn(ar, "failed to send cfr capture info: vdev_id %u peer %pM\n",
			    arsta->arvif->vdev_id, sta->addr);
		goto out;
	}

	ret = count;

	arsta->cfr_capture.cfr_enable = per_peer_cfr_status;
	arsta->cfr_capture.cfr_period = per_peer_cfr_period;
	arsta->cfr_capture.cfr_bandwidth = per_peer_cfr_bw;
	arsta->cfr_capture.cfr_method = per_peer_cfr_method;
out:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static ssize_t ath10k_dbg_sta_read_cfr_capture(struct file *file,
					       char __user *user_buf,
					       size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	char buf[512];
	int len = 0;

	mutex_lock(&ar->conf_mutex);
	len = scnprintf(buf, sizeof(buf) - len, "cfr_status: %s\n"
			"cfr_bandwidth: %dMHz\ncfr_period: %d ms\ncfr_method: %d\n",
			(arsta->cfr_capture.cfr_enable) ? "enabled" :
			"disabled", (arsta->cfr_capture.cfr_bandwidth == 0) ?
			20 : (arsta->cfr_capture.cfr_bandwidth == 1) ?
			40 : 80, arsta->cfr_capture.cfr_period,
			arsta->cfr_capture.cfr_method);
	mutex_unlock(&ar->conf_mutex);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_cfr_capture = {
	.read = ath10k_dbg_sta_read_cfr_capture,
	.write = ath10k_dbg_sta_write_cfr_capture,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_read_peer_ps_state(struct file *file,
						 char __user *user_buf,
						 size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	char buf[20];
	int len = 0;

	spin_lock_bh(&ar->data_lock);

	len = scnprintf(buf, sizeof(buf) - len, "%d\n",
			arsta->peer_ps_state);

	spin_unlock_bh(&ar->data_lock);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_peer_ps_state = {
	.open = simple_open,
	.read = ath10k_dbg_sta_read_peer_ps_state,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

#define STR_PKTS_BYTES  ((strstr(str[j], "pkts")) ? "packets" : "bytes")

static ssize_t ath10k_dbg_sta_dump_tx_stats(struct file *file,
					    char __user *user_buf,
					    size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	struct ath10k_htt_data_stats *stats;
	const char *str_name[ATH10K_STATS_TYPE_MAX] = {"succ", "fail",
						       "retry", "ampdu"};
	const char *str[ATH10K_COUNTER_TYPE_MAX] = {"bytes", "pkts"};
	int len = 0, i, j, k, retval = 0;
	const int size = 16 * 4096;
	char *buf;

	buf = kzalloc(size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&ar->conf_mutex);

	spin_lock_bh(&ar->data_lock);

	if (!arsta->tx_stats) {
		ath10k_warn(ar, "failed to get tx stats");
		spin_unlock_bh(&ar->data_lock);
		mutex_unlock(&ar->conf_mutex);
		kfree(buf);
		return -EINVAL;
	}

	for (k = 0; k < ATH10K_STATS_TYPE_MAX; k++) {
		for (j = 0; j < ATH10K_COUNTER_TYPE_MAX; j++) {
			stats = &arsta->tx_stats->stats[k];
			len += scnprintf(buf + len, size - len, "%s_%s:\n",
					 str_name[k],
					 str[j]);
			len += scnprintf(buf + len, size - len,
					"VHT MCS %s:\n  ",
					STR_PKTS_BYTES);
			for (i = 0; i < ATH10K_VHT_MCS_NUM; i++)
				len += scnprintf(buf + len, size - len,
						 "%llu ",
						 stats->vht[j][i]);
			len += scnprintf(buf + len, size - len, "\n");
			len += scnprintf(buf + len, size - len,
					 "HT MCS %s:\n  ",
					 STR_PKTS_BYTES);
			for (i = 0; i < ATH10K_HT_MCS_NUM; i++)
				len += scnprintf(buf + len, size - len,
						 "%llu ", stats->ht[j][i]);
			len += scnprintf(buf + len, size - len, "\n");
			len += scnprintf(buf + len, size - len,
					 "BW %s:  20Mhz: %llu\t40Mhz: %llu\t",
					 STR_PKTS_BYTES,
					 stats->bw[j][0],
					 stats->bw[j][3]);
			len += scnprintf(buf + len, size - len,
					 "80Mhz: %llu\t160Mhz: %llu\n",
					 stats->bw[j][4],
					 stats->bw[j][5]);
			len += scnprintf(buf + len, size - len,
					 "NSS %s:  1x1: %llu\t2x2: %llu\t",
					 STR_PKTS_BYTES,
					 stats->nss[j][0],
					 stats->nss[j][1]);
			len += scnprintf(buf + len, size - len,
					 "3x3: %llu\t4x4: %llu\n",
					 stats->nss[j][2],
					 stats->nss[j][3]);
			len += scnprintf(buf + len, size - len,
					 "GI %s:  LGI: %llu\t",
					 STR_PKTS_BYTES,
					 stats->gi[j][0]);
			len += scnprintf(buf + len, size - len, "SGI: %llu\n",
					 stats->gi[j][1]);
			len += scnprintf(buf + len, size - len,
					 "legacy rate %s: ",
					 STR_PKTS_BYTES);
			len += scnprintf(buf + len, size - len,
					 "\t1Mbps: %llu\t2Mbps: %llu\t",
					 stats->legacy[j][0],
					 stats->legacy[j][1]);
			len += scnprintf(buf + len, size - len,
					 "5.5Mbps: %llu\t11Mbps: %llu\n",
					 stats->legacy[j][2],
					 stats->legacy[j][3]);
			len += scnprintf(buf + len, size - len,
					 "\t\t\t6Mbps: %llu\t9Mbps: %llu\t",
					 stats->legacy[j][4],
					 stats->legacy[j][5]);
			len += scnprintf(buf + len, size - len,
					 "12Mbps: %llu\t18Mbps: %llu\n",
					 stats->legacy[j][6],
					 stats->legacy[j][7]);
			len += scnprintf(buf + len, size - len,
					 "\t\t\t24Mbps: %llu\t36Mbps: %llu\t",
					 stats->legacy[j][8],
					 stats->legacy[j][9]);
			len += scnprintf(buf + len, size - len,
					 "48Mbps: %llu\t54Mbps: %llu\n",
					 stats->legacy[j][10],
					 stats->legacy[j][11]);
			len += scnprintf(buf + len, size - len,
					 "Rate table %s :\n",
					 STR_PKTS_BYTES);
			for (i = 0; i < ATH10K_RATE_TABLE_NUM; i++) {
				len += scnprintf(buf + len, size - len,
						 "\t%llu",
						 stats->rate_table[j][i]);
				if (!((i + 1) % 8))
					len +=
					scnprintf(buf + len, size - len, "\n");
			}
			len += scnprintf(buf + len, size - len, "\n");
		}
	}

	len += scnprintf(buf + len, size - len,
			 "\nTX duration:\t %llu usecs\n",
			 arsta->tx_stats->tx_duration);
	len += scnprintf(buf + len, size - len,
			"BA fails:\t %llu\n", arsta->tx_stats->ba_fails);
	len += scnprintf(buf + len, size - len,
			"ACK fails\n %llu\n", arsta->tx_stats->ack_fails);
	spin_unlock_bh(&ar->data_lock);

	if (len > size)
		len = size;
	retval = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);

	mutex_unlock(&ar->conf_mutex);
	return retval;
}

#undef STR_PKTS_BYTES

static const struct file_operations fops_tx_stats = {
	.read = ath10k_dbg_sta_dump_tx_stats,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_read_tx_success_bytes(struct file *file,
						    char __user *user_buf,
						    size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	struct ath10k_htt_data_stats *stats;
	char buf[100];
	int len = 0, i, retval = 0;
	u64 total_succ_bytes;

	mutex_lock(&ar->conf_mutex);

	spin_lock_bh(&ar->data_lock);

	if (!arsta->tx_stats) {
		ath10k_warn(ar, "failed to get tx success bytes");
		spin_unlock_bh(&ar->data_lock);
		mutex_unlock(&ar->conf_mutex);
		return -EINVAL;
	}

	stats = &arsta->tx_stats->stats[ATH10K_STATS_TYPE_SUCC];

	total_succ_bytes = stats->gi[ATH10K_COUNTER_TYPE_BYTES][0] +
			   stats->gi[ATH10K_COUNTER_TYPE_BYTES][1];

	for (i = 0; i < ATH10K_LEGACY_NUM; i++)
		total_succ_bytes += stats->legacy[ATH10K_COUNTER_TYPE_BYTES][i];

	len = scnprintf(buf, sizeof(buf),
			"%llu\n", total_succ_bytes);

	spin_unlock_bh(&ar->data_lock);

	retval = simple_read_from_buffer(user_buf, count, ppos, buf, len);

	mutex_unlock(&ar->conf_mutex);

	return retval;
}

static const struct file_operations fops_tx_success_bytes = {
	.read = ath10k_dbg_sta_read_tx_success_bytes,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_read_ampdu_subframe_count(struct file *file,
							char __user *user_buf,
							size_t count,
							loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	char buf[30];
	int len = 0;

	mutex_lock(&ar->conf_mutex);
	len = scnprintf(buf, sizeof(buf) - len,
			"ampdu_subframe_count: %d\n",
			arsta->ampdu_subframe_count);
	mutex_unlock(&ar->conf_mutex);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t ath10k_dbg_sta_write_ampdu_subframe_count(struct file *file,
							 const char __user *user_buf,
							 size_t count,
							 loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	u8 ampdu_subframe_count;
	int ret;

	if (kstrtou8_from_user(user_buf, count, 0, &ampdu_subframe_count))
		return -EINVAL;

	if (ampdu_subframe_count > ATH10K_AMPDU_SUBFRAME_COUNT_MAX ||
	    ampdu_subframe_count < ATH10K_AMPDU_SUBFRAME_COUNT_MIN)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);
	if (ar->state != ATH10K_STATE_ON) {
		ret = -EBUSY;
		goto out;
	}

	ret = ath10k_wmi_peer_set_param(ar, arsta->arvif->vdev_id, sta->addr,
					WMI_PEER_AMPDU, ampdu_subframe_count);
	if (ret) {
		ath10k_warn(ar, "failed to set ampdu subframe count for station"
			    " ret: %d\n", ret);
		goto out;
	}

	ret = count;
	arsta->ampdu_subframe_count = ampdu_subframe_count;
out:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static const struct file_operations fops_set_ampdu_subframe_count = {
	.read = ath10k_dbg_sta_read_ampdu_subframe_count,
	.write = ath10k_dbg_sta_write_ampdu_subframe_count,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

void ath10k_sta_add_debugfs(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			    struct ieee80211_sta *sta, struct dentry *dir)
{
	struct ath10k *ar = hw->priv;

	debugfs_create_file("aggr_mode", 0644, dir, sta, &fops_aggr_mode);
	debugfs_create_file("addba", 0200, dir, sta, &fops_addba);
	debugfs_create_file("addba_resp", 0200, dir, sta, &fops_addba_resp);
	debugfs_create_file("delba", 0200, dir, sta, &fops_delba);
	debugfs_create_file("rx_duration", 0444, dir, sta,
			    &fops_rx_duration);
	debugfs_create_file("peer_debug_trigger", 0600, dir, sta,
			    &fops_peer_debug_trigger);

	if (ath10k_peer_stats_enabled(ar) &&
	    ath10k_debug_is_extd_tx_stats_enabled(ar)){
		debugfs_create_file("tx_stats", 0400, dir, sta,
				    &fops_tx_stats);
		debugfs_create_file("tx_success_bytes", S_IRUGO, dir, sta,
				    &fops_tx_success_bytes);
	}
	debugfs_create_file("peer_ps_state", 0400, dir, sta,
			    &fops_peer_ps_state);
	debugfs_create_file("ampdu_subframe_count", S_IRUGO | S_IWUSR, dir, sta,
			    &fops_set_ampdu_subframe_count);
	if (test_bit(WMI_SERVICE_CFR_CAPTURE_SUPPORT, ar->wmi.svc_map))
		debugfs_create_file("cfr_capture", 0644 , dir,
				    sta, &fops_cfr_capture);
}
