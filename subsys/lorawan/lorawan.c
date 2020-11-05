/*
 * Copyright (c) 2020 Manivannan Sadhasivam <mani@kernel.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <errno.h>
#include <lorawan/lorawan.h>
#include <zephyr.h>

#include "lw_priv.h"

#if CONFIG_LORAWAN_USE_RX_RING_BUFFER
#include <sys/ring_buffer.h>
#endif

#include <LoRaMac.h>

BUILD_ASSERT(!IS_ENABLED(CONFIG_LORAMAC_REGION_UNKNOWN),
	     "Unknown region specified for LoRaWAN in Kconfig");

#ifdef CONFIG_LORAMAC_REGION_AS923
	#define LORAWAN_REGION LORAMAC_REGION_AS923
#elif CONFIG_LORAMAC_REGION_AU915
	#define LORAWAN_REGION LORAMAC_REGION_AU915
#elif CONFIG_LORAMAC_REGION_CN470
	#define LORAWAN_REGION LORAMAC_REGION_CN470
#elif CONFIG_LORAMAC_REGION_CN779
	#define LORAWAN_REGION LORAMAC_REGION_CN779
#elif CONFIG_LORAMAC_REGION_EU433
	#define LORAWAN_REGION LORAMAC_REGION_EU433
#elif CONFIG_LORAMAC_REGION_EU868
	#define LORAWAN_REGION LORAMAC_REGION_EU868
#elif CONFIG_LORAMAC_REGION_KR920
	#define LORAWAN_REGION LORAMAC_REGION_KR920
#elif CONFIG_LORAMAC_REGION_IN865
	#define LORAWAN_REGION LORAMAC_REGION_IN865
#elif CONFIG_LORAMAC_REGION_US915
	#define LORAWAN_REGION LORAMAC_REGION_US915
#elif CONFIG_LORAMAC_REGION_RU864
	#define LORAWAN_REGION LORAMAC_REGION_RU864
#else
	#error "At least one LoRaWAN region should be selected"
#endif

/* Use version 1.0.3.0 for ABP */
#define LORAWAN_ABP_VERSION 0x01000300

#define CEIL_32(x) ((unsigned int)(((((x))+31U)/32U)*32U))
#define SIZE_B_TO_W(x) (CEIL_32(x*8)/32)

#define LORAWAN_PLD_MAX_SIZE_B	242U
#define LORAWAN_PLD_MAX_SIZE_W	SIZE_B_TO_W(LORAWAN_PLD_MAX_SIZE_B)

#define LOG_LEVEL CONFIG_LORAWAN_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(lorawan);

K_SEM_DEFINE(mlme_confirm_sem, 0, 1);
K_SEM_DEFINE(mcps_confirm_sem, 0, 1);

K_MUTEX_DEFINE(lorawan_join_mutex);
K_MUTEX_DEFINE(lorawan_send_mutex);

static enum lorawan_datarate lorawan_datarate = LORAWAN_DR_0;
static uint8_t lorawan_conf_msg_tries = 1;
static bool lorawan_adr_enable;

#if CONFIG_LORAWAN_USE_RX_RING_BUFFER

#if CONFIG_LORAWAN_RX_RING_BUFFER_MAX_SIZE_WORDS < 3
#error The minimum size possible for the ring buffer is 3 x 32bit chunks
#endif

#define RX_RING_BUF_SIZE_WORDS (CONFIG_LORAWAN_RX_RING_BUFFER_MAX_SIZE_WORDS)
struct rx_ring_buf {
	struct ring_buf rb;
	uint32_t buffer[RX_RING_BUF_SIZE_WORDS];
};

struct rx_ring_buf rx_buf;

#else

uint8_t rx_buffer[LORAWAN_PLD_MAX_SIZE_B];
uint8_t rx_buffer_stored_len;
uint8_t rx_buffer_port;

#endif	/* CONFIG_LORAWAN_USE_RX_RING_BUFFER */

static uint16_t rx_buf_avail_elem;
static uint16_t rx_buf_discarded_elem;
#if CONFIG_LORAWAN_USE_RX_RING_BUFFER
static uint32_t payload_tmp[LORAWAN_PLD_MAX_SIZE_W];
static uint8_t payload_tmp_size = LORAWAN_PLD_MAX_SIZE_W;
#endif


static LoRaMacPrimitives_t macPrimitives;
static LoRaMacCallback_t macCallbacks;

static LoRaMacEventInfoStatus_t last_mcps_confirm_status;
static LoRaMacEventInfoStatus_t last_mlme_confirm_status;
static LoRaMacEventInfoStatus_t last_mcps_indication_status;
static LoRaMacEventInfoStatus_t last_mlme_indication_status;

static uint8_t (*getBatteryLevelUser)(void);

static uint8_t getBatteryLevelLocal(void)
{
	if (getBatteryLevelUser != NULL) {
		return getBatteryLevelUser();
	}

	return 255;
}

/**
 * @brief Read a downlink message from the ring buffer.
 *
 * Read a downlink message from the ring buffer and place its payload in the
 * provided container. The message is consumed in the process. If the container
 * is shorter than the available payload any excess data will be discarded and
 * only as much of the payload as possible will be output.
 *
 * @param port Container for the message's port
 * @param payload Container for the message's payload
 * @param len Length in bytes of the \p payload container.
 *
 * @retval non-negative integer indicating the number of read payload bytes
 *         successfully copied to the provided container
 * @retval -EAGAIN No messages in the buffer.
 */
static int rx_buf_get(uint8_t *port, uint8_t *payload, uint16_t len)
{
	int len_to_copy;

#if CONFIG_LORAWAN_USE_RX_RING_BUFFER
	int ret;
	uint16_t rx_buffer_stored_len;

	payload_tmp_size = LORAWAN_PLD_MAX_SIZE_W;
	ret = ring_buf_item_get(&rx_buf.rb, &rx_buffer_stored_len,
				port, payload_tmp, &payload_tmp_size);

	if (ret < 0) {
		return ret;
	}

	rx_buf_avail_elem--;
	len_to_copy = rx_buffer_stored_len < len ? rx_buffer_stored_len : len;
	memcpy(payload, payload_tmp, len_to_copy);
#else
	len_to_copy = rx_buffer_stored_len < len ? rx_buffer_stored_len : len;
	rx_buf_avail_elem = 0;
	*port = rx_buffer_port;
	rx_buffer_port = 0;
	memcpy(payload, rx_buffer, len_to_copy);
#endif
	return len_to_copy;
}

/**
 * @brief Put a downlink message into the ring buffer.
 *
 * Put a downlink message intothe ring buffer. If the buffer doesn not have
 * sufficient free space oldest messages will be discarded for as long as
 * possible until sufficient space is available.
 *
 * @param port Message's port number
 * @param payload Container for the message's payload
 * @param len \p payload size in bytes.
 *
 * @retval 0 On success
 * @retval -EMSGSIZE Message too large for the buffer
 */
static int rx_buf_put(uint8_t port, uint8_t *payload, uint16_t len)
{
#if CONFIG_LORAWAN_USE_RX_RING_BUFFER
	int ret;
	uint16_t type_tmp;
	uint8_t val_tmp;

	memcpy(payload_tmp, payload, len);

	ret = ring_buf_item_put(&rx_buf.rb, len, port, payload_tmp,
				CEIL_32(len*8)/32);
	while (ret == -EMSGSIZE) {
		/* not enough room for the data item -> remove the oldes one */
		payload_tmp_size = LORAWAN_PLD_MAX_SIZE_W;
		ret = ring_buf_item_get(&rx_buf.rb, &type_tmp, &val_tmp,
			payload_tmp, &payload_tmp_size);
		if (ret == -EAGAIN) {
			/* Buffer doesn't have any items -> the one we are
			 * trying to add won't fit any way.
			 */
			return -EMSGSIZE;
		}
		LOG_DBG("Discarded ring buff elemem size %d", payload_tmp_size);
		rx_buf_avail_elem--;
		rx_buf_discarded_elem++;

		memcpy(payload_tmp, payload, len);

		ret = ring_buf_item_put(&rx_buf.rb, len, port, payload_tmp,
					SIZE_B_TO_W(len));
	}

	rx_buf_avail_elem++;
#else
	memcpy(rx_buffer, payload, len);
	rx_buf_avail_elem = 1;
	rx_buffer_port = port;
	rx_buffer_stored_len = len;
#endif

	return 0;
}

/**
 * @brief Return the number of downlink messages available.
 *
 * Return the number of downlink messages available in the buffer
 *
 * @retval number of downlink messages available in the buffer
 */
static int rx_buf_avail(void)
{
	return rx_buf_avail_elem;
}

/**
 * @brief Return the number of discarded downlink messages since last call.
 *
 * Returns the number of discarded downlink messages since this function has
 * been called last.
 *
 * @retval number of discarded characters
 */
static int rx_buf_discarded(void)
{
	int retval = rx_buf_discarded_elem;

	rx_buf_discarded_elem = 0;
	return retval;
}

static void OnMacProcessNotify(void)
{
	LoRaMacProcess();
}

static void McpsConfirm(McpsConfirm_t *mcpsConfirm)
{
	LOG_DBG("Received McpsConfirm (for McpsRequest %d)",
		mcpsConfirm->McpsRequest);

	if (mcpsConfirm->Status != LORAMAC_EVENT_INFO_STATUS_OK) {
		LOG_ERR("McpsRequest failed : %s",
			lorawan_eventinfo2str(mcpsConfirm->Status));
	} else {
		LOG_DBG("McpsRequest success!");
	}

	last_mcps_confirm_status = mcpsConfirm->Status;
	k_sem_give(&mcps_confirm_sem);
}

static void McpsIndication(McpsIndication_t *mcpsIndication)
{
	int ret;
	LOG_DBG("Received McpsIndication %d", mcpsIndication->McpsIndication);

	if (mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK) {
		LOG_ERR("McpsIndication failed : %s",
			lorawan_eventinfo2str(mcpsIndication->Status));
		return;
	}

	/* TODO: Check MCPS Indication type */
	if (mcpsIndication->RxData == true) {
		if (mcpsIndication->BufferSize != 0) {
			LOG_DBG("Rx Data: %s",
				log_strdup(mcpsIndication->Buffer));

			ret = rx_buf_put(mcpsIndication->Port,
				mcpsIndication->Buffer,
				mcpsIndication->BufferSize);
			if (ret == -EMSGSIZE) {
				LOG_WRN("Rx buff too small for DL size %d ",
					mcpsIndication->BufferSize);
			}
		}
	}

	last_mcps_indication_status = mcpsIndication->Status;

	/* TODO: Compliance test based on FPort value*/
}

static void MlmeConfirm(MlmeConfirm_t *mlmeConfirm)
{
	MibRequestConfirm_t mibGet;

	LOG_DBG("Received MlmeConfirm (for MlmeRequest %d)",
		mlmeConfirm->MlmeRequest);

	if (mlmeConfirm->Status != LORAMAC_EVENT_INFO_STATUS_OK) {
		LOG_ERR("MlmeConfirm failed : %s",
			lorawan_eventinfo2str(mlmeConfirm->Status));
		goto out_sem;
	}

	switch (mlmeConfirm->MlmeRequest) {
	case MLME_JOIN:
		mibGet.Type = MIB_DEV_ADDR;
		LoRaMacMibGetRequestConfirm(&mibGet);
		LOG_INF("Joined network! DevAddr: %08x", mibGet.Param.DevAddr);
		break;
	case MLME_LINK_CHECK:
		/* Not implemented */
		LOG_INF("Link check not implemented yet!");
		break;
	default:
		break;
	}

out_sem:
	last_mlme_confirm_status = mlmeConfirm->Status;
	k_sem_give(&mlme_confirm_sem);
}

static void MlmeIndication(MlmeIndication_t *mlmeIndication)
{
	LOG_DBG("Received MlmeIndication %d", mlmeIndication->MlmeIndication);
	last_mlme_indication_status = mlmeIndication->Status;
}

static LoRaMacStatus_t lorawan_join_otaa(
			const struct lorawan_join_config *join_cfg)
{
	MlmeReq_t mlme_req;
	MibRequestConfirm_t mib_req;

	mlme_req.Type = MLME_JOIN;
	mlme_req.Req.Join.Datarate = lorawan_datarate;

	mib_req.Type = MIB_DEV_EUI;
	mib_req.Param.DevEui = join_cfg->dev_eui;
	LoRaMacMibSetRequestConfirm(&mib_req);

	mib_req.Type = MIB_JOIN_EUI;
	mib_req.Param.JoinEui = join_cfg->otaa.join_eui;
	LoRaMacMibSetRequestConfirm(&mib_req);

	mib_req.Type = MIB_NWK_KEY;
	mib_req.Param.NwkKey = join_cfg->otaa.nwk_key;
	LoRaMacMibSetRequestConfirm(&mib_req);

	mib_req.Type = MIB_APP_KEY;
	mib_req.Param.JoinEui = join_cfg->otaa.app_key;
	LoRaMacMibSetRequestConfirm(&mib_req);

	return LoRaMacMlmeRequest(&mlme_req);
}

static LoRaMacStatus_t lorawan_join_abp(
			const struct lorawan_join_config *join_cfg)
{
	MibRequestConfirm_t mib_req;

	mib_req.Type = MIB_ABP_LORAWAN_VERSION;
	mib_req.Param.AbpLrWanVersion.Value = LORAWAN_ABP_VERSION;
	LoRaMacMibSetRequestConfirm(&mib_req);

	mib_req.Type = MIB_NET_ID;
	mib_req.Param.NetID = 0;
	LoRaMacMibSetRequestConfirm(&mib_req);

	mib_req.Type = MIB_DEV_ADDR;
	mib_req.Param.DevAddr = join_cfg->abp.dev_addr;
	LoRaMacMibSetRequestConfirm(&mib_req);

	mib_req.Type = MIB_F_NWK_S_INT_KEY;
	mib_req.Param.FNwkSIntKey = join_cfg->abp.nwk_skey;
	LoRaMacMibSetRequestConfirm(&mib_req);

	mib_req.Type = MIB_S_NWK_S_INT_KEY;
	mib_req.Param.SNwkSIntKey = join_cfg->abp.nwk_skey;
	LoRaMacMibSetRequestConfirm(&mib_req);

	mib_req.Type = MIB_NWK_S_ENC_KEY;
	mib_req.Param.NwkSEncKey = join_cfg->abp.nwk_skey;
	LoRaMacMibSetRequestConfirm(&mib_req);

	mib_req.Type = MIB_APP_S_KEY;
	mib_req.Param.AppSKey = join_cfg->abp.app_skey;
	LoRaMacMibSetRequestConfirm(&mib_req);

	mib_req.Type = MIB_NETWORK_ACTIVATION;
	mib_req.Param.NetworkActivation = ACTIVATION_TYPE_ABP;
	LoRaMacMibSetRequestConfirm(&mib_req);

	return LORAMAC_STATUS_OK;
}

int lorawan_join(const struct lorawan_join_config *join_cfg)
{
	LoRaMacStatus_t status;
	int ret = 0;

	k_mutex_lock(&lorawan_join_mutex, K_FOREVER);

	if (join_cfg->mode == LORAWAN_ACT_OTAA) {
		status = lorawan_join_otaa(join_cfg);
		if (status != LORAMAC_STATUS_OK) {
			LOG_ERR("OTAA join failed: %s",
				lorawan_status2str(status));
			ret = lorawan_status2errno(status);
			goto out;
		}

		LOG_DBG("Network join request sent!");

		/*
		 * We can be sure that the semaphore will be released for
		 * both success and failure cases after a specific time period.
		 * So we can use K_FOREVER and no need to check the return val.
		 */
		k_sem_take(&mlme_confirm_sem, K_FOREVER);
		if (last_mlme_confirm_status != LORAMAC_EVENT_INFO_STATUS_OK) {
			ret = lorawan_eventinfo2errno(last_mlme_confirm_status);
			goto out;
		}
	} else if (join_cfg->mode == LORAWAN_ACT_ABP) {
		status = lorawan_join_abp(join_cfg);
		if (status != LORAMAC_STATUS_OK) {
			LOG_ERR("ABP join failed: %s",
				lorawan_status2str(status));
			ret = lorawan_status2errno(status);
			goto out;
		}
	} else {
		ret = -EINVAL;
	}

out:
	k_mutex_unlock(&lorawan_join_mutex);
	return ret;
}

int lorawan_set_class(enum lorawan_class dev_class)
{
	LoRaMacStatus_t status;
	MibRequestConfirm_t mib_req;

	mib_req.Type = MIB_DEVICE_CLASS;

	switch (dev_class) {
	case LORAWAN_CLASS_A:
		mib_req.Param.Class = CLASS_A;
		break;
	case LORAWAN_CLASS_B:
	case LORAWAN_CLASS_C:
		LOG_ERR("Device class not supported yet!");
		return -ENOTSUP;
	default:
		return -EINVAL;
	};

	status = LoRaMacMibSetRequestConfirm(&mib_req);
	if (status != LORAMAC_STATUS_OK) {
		LOG_ERR("Failed to set device class: %s",
			lorawan_status2str(status));
		return lorawan_status2errno(status);
	}

	return 0;
}

int lorawan_set_datarate(enum lorawan_datarate dr)
{
	/* Bail out if using ADR */
	if (lorawan_adr_enable) {
		return -EINVAL;
	}

	lorawan_datarate = dr;

	return 0;
}

void lorawan_enable_adr(bool enable)
{
	MibRequestConfirm_t mib_req;

	if (enable != lorawan_adr_enable) {
		lorawan_adr_enable = enable;

		mib_req.Type = MIB_ADR;
		mib_req.Param.AdrEnable = lorawan_adr_enable;
		LoRaMacMibSetRequestConfirm(&mib_req);
	}
}

int lorawan_set_conf_msg_tries(uint8_t tries)
{
	lorawan_conf_msg_tries = tries;

	return 0;
}

int lorawan_send(uint8_t port, uint8_t *data, uint8_t len, uint8_t flags)
{
	LoRaMacStatus_t status;
	McpsReq_t mcpsReq;
	LoRaMacTxInfo_t txInfo;
	int ret = 0;
	bool empty_frame = false;

	if (data == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&lorawan_send_mutex, K_FOREVER);

	status = LoRaMacQueryTxPossible(len, &txInfo);
	if (status != LORAMAC_STATUS_OK) {
		/*
		 * If status indicates an error, then most likely the payload
		 * has exceeded the maximum possible length for the current
		 * region and datarate. We can't do much other than sending
		 * empty frame in order to flush MAC commands in stack and
		 * hoping the application to lower the payload size for
		 * next try.
		 */
		LOG_ERR("LoRaWAN Query Tx Possible Failed: %s",
			lorawan_status2str(status));
		empty_frame = true;
		mcpsReq.Type = MCPS_UNCONFIRMED;
		mcpsReq.Req.Unconfirmed.fBuffer = NULL;
		mcpsReq.Req.Unconfirmed.fBufferSize = 0;
		mcpsReq.Req.Unconfirmed.Datarate = DR_0;
	} else {
		if (flags & LORAWAN_MSG_CONFIRMED) {
			mcpsReq.Type = MCPS_CONFIRMED;
			mcpsReq.Req.Confirmed.fPort = port;
			mcpsReq.Req.Confirmed.fBuffer = data;
			mcpsReq.Req.Confirmed.fBufferSize = len;
			mcpsReq.Req.Confirmed.NbTrials = lorawan_conf_msg_tries;
			mcpsReq.Req.Confirmed.Datarate = lorawan_datarate;
		} else {
			/* default message type */
			mcpsReq.Type = MCPS_UNCONFIRMED;
			mcpsReq.Req.Unconfirmed.fPort = port;
			mcpsReq.Req.Unconfirmed.fBuffer = data;
			mcpsReq.Req.Unconfirmed.fBufferSize = len;
			mcpsReq.Req.Unconfirmed.Datarate = lorawan_datarate;
		}
	}

	status = LoRaMacMcpsRequest(&mcpsReq);
	if (status != LORAMAC_STATUS_OK) {
		LOG_ERR("LoRaWAN Send failed: %s", lorawan_status2str(status));
		ret = lorawan_status2errno(status);
		goto out;
	}

	/*
	 * Indicate to the application that the current packet is not sent and
	 * it has to resend the packet.
	 */
	if (empty_frame) {
		ret = -EAGAIN;
		goto out;
	}

	/* Wait for send confirmation */
	if (flags & LORAWAN_MSG_CONFIRMED) {
		/*
		 * We can be sure that the semaphore will be released for
		 * both success and failure cases after a specific time period.
		 * So we can use K_FOREVER and no need to check the return val.
		 */
		k_sem_take(&mcps_confirm_sem, K_FOREVER);

		if (last_mcps_confirm_status != LORAMAC_EVENT_INFO_STATUS_OK) {
			ret = lorawan_eventinfo2errno(last_mcps_confirm_status);
		}
	}

out:
	k_mutex_unlock(&lorawan_send_mutex);
	return ret;
}

int lorawan_set_battery_level_callback(uint8_t (*battery_lvl_clbk)(void))
{
	if (battery_lvl_clbk == NULL) {
		return -EINVAL;
	}

	getBatteryLevelUser = battery_lvl_clbk;

	return 0;
}

int lorawan_receive_available(void)
{
	return rx_buf_avail();
}

int lorawan_receive_read(uint8_t *port, uint8_t *data, uint8_t len)
{
	return rx_buf_get(port, data, len);
}

int lorawan_receive_discarded(void)
{
	return rx_buf_discarded();
}

int lorawan_start(void)
{
	LoRaMacStatus_t status;
	MibRequestConfirm_t mib_req;

	status = LoRaMacStart();
	if (status != LORAMAC_STATUS_OK) {
		LOG_ERR("Failed to start the LoRaMAC stack: %s",
			lorawan_status2str(status));
		return -EINVAL;
	}

	/* TODO: Move these to a proper location */
	mib_req.Type = MIB_SYSTEM_MAX_RX_ERROR;
	mib_req.Param.SystemMaxRxError = CONFIG_LORAWAN_SYSTEM_MAX_RX_ERROR;
	LoRaMacMibSetRequestConfirm(&mib_req);

	mib_req.Type = MIB_PUBLIC_NETWORK;
	mib_req.Param.EnablePublicNetwork = true;
	LoRaMacMibSetRequestConfirm(&mib_req);

	return 0;
}

static int lorawan_init(const struct device *dev)
{
	LoRaMacStatus_t status;

	macPrimitives.MacMcpsConfirm = McpsConfirm;
	macPrimitives.MacMcpsIndication = McpsIndication;
	macPrimitives.MacMlmeConfirm = MlmeConfirm;
	macPrimitives.MacMlmeIndication = MlmeIndication;
	macCallbacks.GetBatteryLevel = getBatteryLevelLocal;
	macCallbacks.GetTemperatureLevel = NULL;
	macCallbacks.NvmContextChange = NULL;
	macCallbacks.MacProcessNotify = OnMacProcessNotify;

	status = LoRaMacInitialization(&macPrimitives, &macCallbacks,
				       LORAWAN_REGION);
	if (status != LORAMAC_STATUS_OK) {
		LOG_ERR("LoRaMacInitialization failed: %s",
			lorawan_status2str(status));
		return -EINVAL;
	}


#if CONFIG_LORAWAN_USE_RX_RING_BUFFER
	ring_buf_init(&rx_buf.rb, RX_RING_BUF_SIZE_WORDS, rx_buf.buffer);
#endif

	rx_buf_avail_elem = 0;
	rx_buf_discarded_elem = 0;

	LOG_DBG("LoRaMAC Initialized");

	return 0;
}

SYS_INIT(lorawan_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
