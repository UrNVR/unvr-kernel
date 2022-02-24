/**
 *  Copyright (c) 2018 MediaTek Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */
#include "btmtk_define.h"
#include "btmtk_main.h"

#define MTKBT_UNSLEEPABLE_LOCK(x, y)	spin_lock_irqsave(x, y)
#define MTKBT_UNSLEEPABLE_UNLOCK(x, y)	spin_unlock_irqsave(x, y)

/**
 * Global parameters(mtkbt_)
 */
static struct btmtk_fops_fwlog *g_fwlog;

const struct file_operations BT_fopsfwlog = {
	.open = btmtk_fops_openfwlog,
	.release = btmtk_fops_closefwlog,
	.read = btmtk_fops_readfwlog,
	.write = btmtk_fops_writefwlog,
	.poll = btmtk_fops_pollfwlog,
	.unlocked_ioctl = btmtk_fops_unlocked_ioctlfwlog
};

static u8 reset_stack_flag;
uint8_t btmtk_log_lvl = BTMTK_LOG_LVL_DEF;
static int stp_enable;
static int uart_baudrate_set;
const u8 READ_ADDRESS_EVENT[] = { 0x0E, 0x0A, 0x01, 0x09, 0x10, 0x00 };
const u8 RESET_EVENT[] = { 0x0E, 0x04, 0x01, 0x03, 0x0c, 0x00 };
const u8 READ_ISO_PACKET_SIZE_CMD[] = {0x01, 0x98, 0xFD, 0x02 };

static void btmtk_load_code_from_bin(u8 **image, char *bin_name,
					 struct device *dev, u32 *code_len);

static int main_init(void)
{
	return 0;
}

static int main_exit(void)
{
	return 0;
}

/* HCI receive mechnism */


static inline struct sk_buff *h4_recv_buf(struct hci_dev *hdev,
					  struct sk_buff *skb,
					  const unsigned char *buffer,
					  int count,
					  const struct h4_recv_pkt *pkts,
					  int pkts_count)
{
	/* Check for error from previous call */
	if (IS_ERR(skb))
		skb = NULL;

	while (count) {
		int i, len;

		if (!count)
			break;

		if (!skb) {
			for (i = 0; i < pkts_count; i++) {
				if (buffer[0] != (&pkts[i])->type)
					continue;

				skb = bt_skb_alloc((&pkts[i])->maxlen,
						   GFP_ATOMIC);
				if (!skb)
					return ERR_PTR(-ENOMEM);

				hci_skb_pkt_type(skb) = (&pkts[i])->type;
				hci_skb_expect(skb) = (&pkts[i])->hlen;
				break;
			}

			/* Check for invalid packet type */
			if (!skb)
				return ERR_PTR(-EILSEQ);

			count -= 1;
			buffer += 1;
		}

		len = min_t(uint, hci_skb_expect(skb) - skb->len, count);
		memcpy(skb_put(skb, len), buffer, len);
		/*
			If kernel version > 4.x
			skb_put_data(skb, buffer, len);
		*/

		count -= len;
		buffer += len;

		/* Check for partial packet */
		if (skb->len < hci_skb_expect(skb))
			continue;

		for (i = 0; i < pkts_count; i++) {
			if (hci_skb_pkt_type(skb) == (&pkts[i])->type)
				break;
		}

		if (i >= pkts_count) {
			kfree_skb(skb);
			return ERR_PTR(-EILSEQ);
		}

		if (skb->len == (&pkts[i])->hlen) {
			u16 dlen;

			switch ((&pkts[i])->lsize) {
			case 0:
				/* No variable data length */
				dlen = 0;
				break;
			case 1:
				/* Single octet variable length */
				dlen = skb->data[(&pkts[i])->loff];
				hci_skb_expect(skb) += dlen;

				if (skb_tailroom(skb) < dlen) {
					kfree_skb(skb);
					return ERR_PTR(-EMSGSIZE);
				}
				break;
			case 2:
				/* Double octet variable length */
				dlen = get_unaligned_le16(skb->data +
							  (&pkts[i])->loff);
				hci_skb_expect(skb) += dlen;

				if (skb_tailroom(skb) < dlen) {
					kfree_skb(skb);
					return ERR_PTR(-EMSGSIZE);
				}
				break;
			default:
				/* Unsupported variable length */
				kfree_skb(skb);
				return ERR_PTR(-EILSEQ);
			}

			if (!dlen) {
				/* No more data, complete frame */
				(&pkts[i])->recv(hdev, skb);
				skb = NULL;
			}
		} else {
			/* Complete frame */
			(&pkts[i])->recv(hdev, skb);
			skb = NULL;
		}
	}

	return skb;
}

static const struct h4_recv_pkt mtk_recv_pkts[] = {
	{ H4_RECV_ACL,      .recv = btmtk_recv_acl },
	{ H4_RECV_SCO,      .recv = hci_recv_frame },
	{ H4_RECV_EVENT,    .recv = btmtk_recv_event },
};

static inline struct sk_buff *mtk_add_stp(struct btmtk_dev *bdev, struct sk_buff *skb)
{
	struct mtk_stp_hdr *shdr;
	int dlen, err = 0, type = 0;
	u8 stp_crc[] = {0x00, 0x00};

	if (unlikely(skb_headroom(skb) < sizeof(*shdr)) ||
		(skb_tailroom(skb) < MTK_STP_TLR_SIZE)) {
		BTMTK_DBG("%s, add pskb_expand_head, headroom = %d, tailroom = %d",
				__func__, skb_headroom(skb), skb_tailroom(skb));

		err = pskb_expand_head(skb, sizeof(*shdr), MTK_STP_TLR_SIZE,
					   GFP_ATOMIC);
	}
	dlen = skb->len;
	shdr = (void *) skb_push(skb, sizeof(*shdr));
	shdr->prefix = 0x80;
	shdr->dlen = cpu_to_be16((dlen & 0x0fff) | (type << 12));
	shdr->cs = 0;
	// Add the STP trailer
	// kernel version > 4.20
	// skb_put_zero(skb, MTK_STP_TLR_SIZE);
	// kernel version < 4.20
	skb_put(skb, sizeof(stp_crc));
	skb->data[skb->len - 1] = 0;
	skb->data[skb->len - 2] = 0;

	return skb;
}

static const unsigned char *
mtk_stp_split(struct btmtk_dev *bdev, const unsigned char *data, int count,
	      int *sz_h4)
{
	struct mtk_stp_hdr *shdr;

	/* The cursor is reset when all the data of STP is consumed out */
	if (!bdev->stp_dlen && bdev->stp_cursor >= 6) {
		bdev->stp_cursor = 0;
		BTMTK_DBG("reset cursor = %d\n", bdev->stp_cursor);
	}

	/* Filling pad until all STP info is obtained */
	while (bdev->stp_cursor < 6 && count > 0) {
		bdev->stp_pad[bdev->stp_cursor] = *data;
		BTMTK_DBG("fill stp format (%02x, %d, %d)\n",
		   bdev->stp_pad[bdev->stp_cursor], bdev->stp_cursor, count);
		bdev->stp_cursor++;
		data++;
		count--;
	}

	/* Retrieve STP info and have a sanity check */
	if (!bdev->stp_dlen && bdev->stp_cursor >= 6) {
		shdr = (struct mtk_stp_hdr *)&bdev->stp_pad[2];
		bdev->stp_dlen = be16_to_cpu(shdr->dlen) & 0x0fff;
		BTMTK_DBG("stp format (%02x, %02x)",
			   shdr->prefix, bdev->stp_dlen);

		/* Resync STP when unexpected data is being read */
		if (shdr->prefix != 0x80 || bdev->stp_dlen > 2048) {
			BTMTK_ERR("stp format unexpect (%02x, %02x)",
				   shdr->prefix, bdev->stp_dlen);
			BTMTK_ERR("reset cursor = %d\n", bdev->stp_cursor);
			bdev->stp_cursor = 2;
			bdev->stp_dlen = 0;
		}
	}

	/* Directly quit when there's no data found for H4 can process */
	if (count <= 0)
		return NULL;

	/* Tranlate to how much the size of data H4 can handle so far */
	*sz_h4 = min_t(int, count, bdev->stp_dlen);

	/* Update the remaining size of STP packet */
	bdev->stp_dlen -= *sz_h4;

	/* Data points to STP payload which can be handled by H4 */
	return data;
}

void mtk_stp_enable(void)
{
	stp_enable = 1;
}

#if LOAD_ROM_PATCH_TO_FLASH
static int btmtk_load_flash_load_fwpatch(struct hci_dev *hdev,
				unsigned char *rom_patch, u32 patch_len)
{
	s32 sent_len = 0;
	u32 cur_len = 0;
	int first_block = 1;
	u8 phase = 0;
	u8 *pos;
	int ret = 0;
	u8 event[] = { 0xE4, 0x05, 0x02, 0x01, 0x01, 0x00, 0x00 };

	pos = kmalloc(UPLOAD_PATCH_PDU + PATCH_HEADER_SIZE, GFP_ATOMIC);
	if (!pos) {
		BTMTK_ERR("%s: kmalloc failed!!!\n", __func__);
		return -1;
	}

	/* loading rom patch */
	while (1) {
		sent_len = (patch_len - cur_len) >= UPLOAD_PATCH_PDU
				? UPLOAD_PATCH_PDU : (patch_len - cur_len);

		if (sent_len > 0) {
			if (first_block == 1) {
				if (sent_len < UPLOAD_PATCH_PDU)
					phase = PATCH_PHASE3;
				else
					phase = PATCH_PHASE1;
				first_block = 0;
			} else if (sent_len == UPLOAD_PATCH_PDU) {
				if (patch_len - cur_len == UPLOAD_PATCH_PDU)
					phase = PATCH_PHASE3;
				else
					phase = PATCH_PHASE2;
			} else {
				phase = PATCH_PHASE3;
			}

			/* prepare HCI header */
			pos[0] = 0x02;
			pos[1] = 0x6F;
			pos[2] = 0xFC;
			pos[3] = (sent_len + 5) & 0xFF;
			pos[4] = ((sent_len + 5) >> 8) & 0xFF;

			/* prepare WMT header */
			pos[5] = 0x01;
			pos[6] = 0x01;
			pos[7] = (sent_len + 1) & 0xFF;
			pos[8] = ((sent_len + 1) >> 8) & 0xFF;

			pos[9] = phase;
			memcpy(&pos[10], rom_patch + cur_len, sent_len);
			ret = btmtk_main_send_cmd(hdev,
				pos, sent_len + PATCH_HEADER_SIZE,
				event, sizeof(event),
				-1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);
			if (ret != event[6]) {
				BTMTK_INFO("%s: send patch failed, terminate",
						__func__);
				ret = -1;
				break;
			}
			cur_len = cur_len + sent_len;
			BTMTK_DBG("%s:sent_len = %d, cur_len = %d, phase = %d",
					__func__, sent_len, cur_len, phase);
		} else {
			BTMTK_INFO("%s: send patch done", __func__);
			ret = 0;
			break;
		}
	}
	kfree(pos);
	return ret;
}

static int btmtk_load_flash_init(struct hci_dev *hdev)
{
	u8 cmd[] = { 0x01, 0x6F, 0xFC, 0x05, 0x01, 0x3F, 0x01, 0x00, 0x00 };
	u8 event[] = {0xE4, 0x06, 0x02, 0x3F, 0x02, 0x00, 0x00, 0x00};
	int ret = 0;

	BTMTK_INFO("%s send", __func__);

	ret = btmtk_main_send_cmd(hdev,
			cmd, sizeof(cmd), event, sizeof(event),
			-1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);
	if (ret != event[6])
		return -1;
	return 0;
}


static int btmtk_load_flash_chech_version(struct hci_dev *hdev, char *version)
{
	u8 cmd[] = { 0x01, 0x6F, 0xFC, 0x05, 0x01, 0x3F, 0x01, 0x00, 0x05 };
	u8 event[] = { 0xE4, 0x14, 0x02, 0x3F, 0x10, 0x00, 0x05, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int ret = 0;

	BTMTK_INFO("%s send", __func__);

	memcpy(&event[GET_FLASH_VERSION_HDR_LEN], version, PATCH_VERSION_LEN);

	ret = btmtk_main_send_cmd(hdev,
			cmd, sizeof(cmd), event, sizeof(event),
			-1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);
	if (ret != event[6])
		return -1;
	return 0;
}

static int btmtk_load_flash_config_section(struct hci_dev *hdev,
			unsigned char *rom_patch)
{
	u8 cmd[] = { 0x01, 0x6F, 0xFC, 0x0F, 0x01, 0x3F, 0x0B, 0x00,
			0x04, 0xFF, 0x07,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 event[] = {0xE4, 0x06, 0x02, 0x3F, 0x02, 0x00, 0x04, 0x00};
	int ret = 0;

	BTMTK_INFO("%s send", __func__);

	cmd[11] = rom_patch[0];
	cmd[12] = rom_patch[1];
	cmd[13] = rom_patch[2];
	cmd[14] = rom_patch[3];
	cmd[15] = rom_patch[4];
	cmd[16] = rom_patch[5];
	cmd[17] = rom_patch[6];
	cmd[18] = rom_patch[7];

	ret = btmtk_main_send_cmd(hdev,
			cmd, sizeof(cmd), event, sizeof(event),
			-1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);
	if (ret != event[6])
		return -1;
	return 0;
}

static int btmtk_load_flash_config_ILM_header(struct hci_dev *hdev)
{
	u8 patch_header_cmd[] = { 0x01, 0x6F, 0xFC, 0x16,
			0x01, 0x3F, 0x12, 0x00,
			0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
			0x00, 0xFF, 0x8F, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};
	u8 patch_header_event[] = {0xE4, 0x06,
			0x02, 0x3F, 0x02, 0x00, 0x04, 0x00};

	u8 fw_header_cmd[] = { 0x01, 0x6F, 0xFC, 0x16,
			0x01, 0x3F, 0x12, 0x00,
			0x04, 0x07, 0x06, 0x00, 0x00, 0x00, 0x00, 0x90, 0x00,
			0x00, 0xFF, 0x7F, 0x0A, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};
	u8 fw_header_event[] = {0xE4, 0x06,
			0x02, 0x3F, 0x02, 0x00, 0x04, 0x00};
	int ret = 0;

	BTMTK_INFO("%s send", __func__);

	ret = btmtk_main_send_cmd(hdev,
			patch_header_cmd, sizeof(patch_header_cmd),
			patch_header_event, sizeof(patch_header_event),
			-1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);
	if (ret != patch_header_event[6])
		return -1;

	ret = btmtk_main_send_cmd(hdev,
			fw_header_cmd, sizeof(fw_header_cmd),
			fw_header_event, sizeof(fw_header_event),
			-1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);
	if (ret != fw_header_event[6])
		return -1;
	return 0;
}

static void btmtk_load_flash_calculate_checksum(struct hci_dev *hdev,
			unsigned char *rom_patch, unsigned int rom_patch_len,
			int *fw_checksum, int *patch_checksum)
{
	int patch_start = 0x1E;
	int patch_end = patch_start + FWPATCH_PATCH_LEN - 1;
	int fw_start = 0x801E;
	int fw_end = rom_patch_len - 1;
	int index = 0;

	*patch_checksum = 0;
	for (index = patch_start; index <= patch_end; index++)
		*patch_checksum += rom_patch[index];

	*fw_checksum = 0;
	for (index = fw_start; index <= fw_end; index++)
		*fw_checksum += rom_patch[index];
}

static int btmtk_load_flash_fw_download(struct hci_dev *hdev,
			int fw_checksum,
			unsigned char *rom_patch, unsigned int rom_patch_len)
{
	u8 cmd[] = { 0x01, 0x6F, 0xFC, 0x11, 0x01, 0x3F, 0x0D, 0x00,
			0x03, 0x06, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xFF, 0xFF,
			0x00, 0x00, 0x00, 0x00};
	u8 event[] = {0xE4, 0x06, 0x02, 0x3F, 0x02, 0x00, 0x03, 0x00};
	unsigned char *fw;
	int ret = 0;

	BTMTK_INFO("%s send", __func__);

	cmd[13] = rom_patch[12];
	cmd[14] = rom_patch[13];
	cmd[17] = fw_checksum & 0xFF;
	cmd[18] = (fw_checksum >> 8) & 0xFF;
	cmd[19] = (fw_checksum >> 16) & 0xFF;
	cmd[20] = (fw_checksum >> 24) & 0xFF;

	ret = btmtk_main_send_cmd(hdev,
			cmd, sizeof(cmd), event, sizeof(event),
			-1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);
	if (ret != event[6])
		return -1;

	fw = rom_patch + FWPATCH_HEADER_LEN + FWPATCH_PATCH_LEN;
	return btmtk_load_flash_load_fwpatch(hdev, fw,
			rom_patch_len - FWPATCH_HEADER_LEN - FWPATCH_PATCH_LEN);
}

static int btmtk_load_flash_patch_download(struct hci_dev *hdev,
			int patch_checksum,
			unsigned char *rom_patch, unsigned int rom_patch_len)
{
	u8 cmd[] = { 0x01, 0x6F, 0xFC, 0x11, 0x01, 0x3F, 0x0D, 0x00,
			0x03, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00};
	u8 event[] = {0xE4, 0x06, 0x02, 0x3F, 0x02, 0x00, 0x03, 0x00};
	unsigned char *patch;
	int ret = 0;

	BTMTK_INFO("%s send", __func__);

	cmd[13] = rom_patch[8];
	cmd[14] = rom_patch[9];
	cmd[15] = rom_patch[10];
	cmd[16] = rom_patch[11];
	cmd[17] = patch_checksum & 0xFF;
	cmd[18] = (patch_checksum >> 8) & 0xFF;
	cmd[19] = (patch_checksum >> 16) & 0xFF;
	cmd[20] = (patch_checksum >> 24) & 0xFF;

	ret = btmtk_main_send_cmd(hdev,
			cmd, sizeof(cmd), event, sizeof(event),
			-1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);
	if (ret != event[6])
		return -1;

	patch = rom_patch + FWPATCH_HEADER_LEN;
	return btmtk_load_flash_load_fwpatch(hdev, patch, FWPATCH_PATCH_LEN);
}

static int btmtk_load_flash_config_signature(struct hci_dev *hdev)
{
	u8 cmd[] = { 0x01, 0x6F, 0xFC, 0x07,
			0x01, 0x3F, 0x03, 0x00, 0x04, 0x00, 0x07 };
	u8 event[] = {0xE4, 0x06, 0x02, 0x3F, 0x02, 0x00, 0x04, 0x00};

	int ret = 0;

	BTMTK_INFO("%s send", __func__);

	ret = btmtk_main_send_cmd(hdev,
			cmd, sizeof(cmd), event, sizeof(event),
			-1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);
	if (ret != event[6])
		return -1;
	return 0;
}

static int btmtk_load_flash_into_external_flash(struct hci_dev *hdev)
{
	u8 cmd[] = { 0x01, 0x6F, 0xFC, 0x05, 0x01, 0x3F, 0x01, 0x00, 0x02 };
	u8 event[] = {0xE4, 0x06, 0x02, 0x3F, 0x02, 0x00, 0x02, 0x00};
	int ret = 0;

	BTMTK_INFO("%s send", __func__);

	ret = btmtk_main_send_cmd(hdev,
			cmd, sizeof(cmd), event, sizeof(event),
			-1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);
	if (ret != event[6])
		return -1;
	return 0;
}

static int btmtk_load_flash_set_efuse(struct hci_dev *hdev)
{
	u8 cmd[] = { 0x01, 0x6F, 0xFC, 0x05, 0x01, 0x3F, 0x01, 0x00, 0x02 };
	u8 event[] = {0xE4, 0x06, 0x02, 0x3F, 0x02, 0x00, 0x02, 0x00};
	int ret = 0;

	BTMTK_INFO("%s send", __func__);

	ret = btmtk_main_send_cmd(hdev,
			cmd, sizeof(cmd), event, sizeof(event),
			-1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);
	if (ret != event[6])
		return -1;
	return 0;
}

int btmtk_load_flash_programing(struct hci_dev *hdev)
{
	unsigned char *rom_patch;
	unsigned char *rom_patch_bin_file_name;
	unsigned int rom_patch_len = 0;
	int fw_checksum = 0;
	int patch_checksum = 0;
	int ret = 0;

	rom_patch_bin_file_name = kzalloc(MAX_BIN_FILE_NAME_LEN, GFP_KERNEL);
	if (!rom_patch_bin_file_name) {
		BTMTK_ERR("%s: kzalloc failed!!!\n", __func__);
		return -1;
	}

	strncpy(rom_patch_bin_file_name, "mt7915_patch_e2_hdr.bin",
		strlen("mt7915_patch_e2_hdr.bin"));

	btmtk_load_code_from_bin(&rom_patch,
			rom_patch_bin_file_name, NULL,
			&rom_patch_len);
	if (rom_patch == NULL) {
		BTMTK_ERR("%s: rom_patch is NULL", __func__);
		ret = -1;
		goto end;
	}

	ret = btmtk_load_flash_init(hdev);
	if (ret < 0) {
		BTMTK_ERR("%s: btmtk_load_flash_init failed", __func__);
		ret = -1;
		goto end;
	}

	ret = btmtk_load_flash_chech_version(hdev, rom_patch);
	if (ret == 0) {
		BTMTK_ERR("%s: btmtk_load_flash_chech_version pass, no need update",
				__func__);
		goto end;
	}

	ret = btmtk_load_flash_config_section(hdev, rom_patch);
	if (ret < 0) {
		BTMTK_ERR("%s: btmtk_load_flash_config_section failed",
				__func__);
		ret = -1;
		goto end;
	}

	ret = btmtk_load_flash_config_ILM_header(hdev);
	if (ret < 0) {
		BTMTK_ERR("%s: btmtk_load_flash_config_ILM_header failed",
				__func__);
		ret = -1;
		goto end;
	}

	btmtk_load_flash_calculate_checksum(hdev,
				rom_patch, rom_patch_len,
				&fw_checksum, &patch_checksum);

	ret = btmtk_load_flash_fw_download(hdev,
				fw_checksum, rom_patch, rom_patch_len);
	if (ret < 0) {
		BTMTK_ERR("%s: btmtk_load_flash_fw_download failed", __func__);
		ret = -1;
		goto end;
	}

	ret = btmtk_load_flash_patch_download(hdev,
				patch_checksum, rom_patch, rom_patch_len);
	if (ret < 0) {
		BTMTK_ERR("%s: btmtk_load_flash_init failed", __func__);
		ret = -1;
		goto end;
	}

	ret = btmtk_load_flash_config_signature(hdev);
	if (ret < 0) {
		BTMTK_ERR("%s: btmtk_load_flash_config_signature failed",
				__func__);
		ret = -1;
		goto end;
	}

	ret = btmtk_load_flash_into_external_flash(hdev);
	if (ret < 0) {
		BTMTK_ERR("%s: btmtk_load_flash_into_external_flash failed",
				__func__);
		ret = -1;
		goto end;
	}

	ret = btmtk_load_flash_set_efuse(hdev);
	if (ret < 0) {
		BTMTK_ERR("%s: btmtk_load_flash_set_efuse failed", __func__);
		ret = -1;
		goto end;
	}
end:
	kfree(rom_patch);
	return ret;
}
#endif

int btmtk_skb_enq_fwlog(void *src, u32 len, u8 type, struct sk_buff_head *queue)
{
	struct sk_buff *skb_tmp = NULL;
	ulong flags = 0;
	int retry = 10;

	do {
		/* If we need hci type, len + 1 */
		skb_tmp = alloc_skb(type ? len + 1 : len, GFP_ATOMIC);
		if (skb_tmp != NULL)
			break;
		else if (retry <= 0) {
			pr_err("%s: alloc_skb return 0, error", __func__);
			return -ENOMEM;
		}
		pr_err("%s: alloc_skb return 0, error, retry = %d",
					__func__, retry);
	} while (retry-- > 0);

	if (type) {
		memcpy(&skb_tmp->data[0], &type, 1);
		memcpy(&skb_tmp->data[1], src, len);
		skb_tmp->len = len + 1;
	} else {
		memcpy(skb_tmp->data, src, len);
		skb_tmp->len = len;
	}

	spin_lock_irqsave(&g_fwlog->fwlog_lock, flags);
	skb_queue_tail(queue, skb_tmp);
	spin_unlock_irqrestore(&g_fwlog->fwlog_lock, flags);
	return 0;
}

ssize_t btmtk_fops_readfwlog(struct file *filp,
					char __user *buf,
					size_t count,
					loff_t *f_pos)
{
	int copyLen = 0;
	ulong flags = 0;
	struct sk_buff *skb = NULL;

	/* picus read a queue, it may occur performace issue */
	spin_lock_irqsave(&g_fwlog->fwlog_lock, flags);
	if (skb_queue_len(&g_fwlog->fwlog_queue))
		skb = skb_dequeue(&g_fwlog->fwlog_queue);

	spin_unlock_irqrestore(&g_fwlog->fwlog_lock, flags);
	if (skb == NULL)
		return 0;

	if (skb->len <= count) {
		if (copy_to_user(buf, skb->data, skb->len))
			BT_ERR("%s: copy_to_user failed!", __func__);

		copyLen = skb->len;
	} else {
		BTMTK_DBG("%s: socket buffer length error(count: %d, skb.len: %d)",
			__func__, (int)count, skb->len);
	}
	kfree_skb(skb);
	return copyLen;
}

ssize_t btmtk_fops_writefwlog(struct file *filp,
					const char __user *buf,
					size_t count,
					loff_t *f_pos)
{
	int i = 0, len = 0, ret = -1;
	struct sk_buff *skb = NULL;

	u8 *i_fwlog_buf = kmalloc(HCI_MAX_COMMAND_BUF_SIZE, GFP_KERNEL);
	u8 *o_fwlog_buf = kmalloc(HCI_MAX_COMMAND_SIZE, GFP_KERNEL);

	skb = alloc_skb(sizeof(buf) + BT_SKB_RESERVE, GFP_ATOMIC);
	if (!skb) {
		BTMTK_ERR("%s allocate skb failed!!", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	if (count > HCI_MAX_COMMAND_BUF_SIZE) {
		BTMTK_ERR("%s: command is larger, count = %zd\n",
				__func__, count);
		ret = -ENOMEM;
		goto exit;
	}

	memset(i_fwlog_buf, 0, HCI_MAX_COMMAND_BUF_SIZE);
	memset(o_fwlog_buf, 0, HCI_MAX_COMMAND_SIZE);
	if (copy_from_user(i_fwlog_buf, buf, count) != 0) {
		BT_ERR("%s: Failed to copy data", __func__);
		ret = -ENODATA;
		goto exit;
	}

	/* hci input command format : echo 01 be fc 01 05 > /dev/stpbtfwlog */
	/* We take the data from index three to end. */
	for (i = 0; i < count; i++) {
		char *pos = i_fwlog_buf + i;
		char temp_str[3] = {'\0'};
		long res = 0;

		if (*pos == ' ' || *pos == '\t'
				|| *pos == '\r' || *pos == '\n') {
			continue;
		} else if (*pos == '0'
				&& (*(pos + 1) == 'x' || *(pos + 1) == 'X')) {
			i++;
			continue;
		} else if (!(*pos >= '0' && *pos <= '9')
				&& !(*pos >= 'A' && *pos <= 'F')
				&& !(*pos >= 'a' && *pos <= 'f')) {
			BTMTK_ERR("%s: There is an invalid input(%c)",
					__func__, *pos);
			ret = -EINVAL;
			goto exit;
		}
		temp_str[0] = *pos;
		temp_str[1] = *(pos + 1);
		i++;
		ret = kstrtol(temp_str, 16, &res);
		if (ret == 0)
			o_fwlog_buf[len++] = (u8)res;
		else
			BT_ERR("%s: Convert %s failed(%d)",
					__func__, temp_str, ret);
	}

	if (o_fwlog_buf[0] != HCI_COMMAND_PKT) {
		BT_ERR("%s: Not support 0x%02X yet", __func__, o_fwlog_buf[0]);
		ret = -EPROTONOSUPPORT;
		goto exit;
	}
	/* check HCI command length */
	if (len > HCI_MAX_COMMAND_SIZE) {
		BT_ERR("%s: command is larger, length = %d", __func__, len);
		ret = -ENOMEM;
		goto exit;
	}

	/* send HCI command */
	bt_cb(skb)->pkt_type = HCI_COMMAND_PKT;
	memcpy(skb->data, o_fwlog_buf, len);
	skb->len = len;

	/* clean fwlog queue before enable picus log */
	if (skb_queue_len(&g_fwlog->fwlog_queue) && skb->data[0] == 0x01
			&& skb->data[1] == 0x5d &&  skb->data[2] == 0xfc) {
		skb_queue_purge(&g_fwlog->fwlog_queue);
		BTMTK_INFO("clean fwlog_queue, skb_queue_len = %d",
				skb_queue_len(&g_fwlog->fwlog_queue));
	}

	ret = btmtk_cif_send_cmd(g_bdev->hdev, skb->data, skb->len, 0, 0, 0);
	if (ret < 0)
		BTMTK_ERR("%s failed!!", __func__);
	else
		BTMTK_INFO("%s: OK", __func__);

	BTMTK_INFO("%s: Write end(len: %d)", __func__, len);
	ret = count;

exit:
	kfree(i_fwlog_buf);
	i_fwlog_buf = NULL;

	kfree(o_fwlog_buf);
	o_fwlog_buf = NULL;

	return ret;	/* If input is correct should return the same length */
}

int btmtk_fops_openfwlog(struct inode *inode, struct file *file)
{
	BTMTK_INFO("%s: Start.", __func__);

	return 0;
}

int btmtk_fops_closefwlog(struct inode *inode, struct file *file)
{
	BTMTK_INFO("%s: Start.", __func__);

	return 0;
}

long btmtk_fops_unlocked_ioctlfwlog(struct file *filp,
					unsigned int cmd,
					unsigned long arg)
{
	BTMTK_INFO("%s: Start.", __func__);

	return 0;
}

unsigned int btmtk_fops_pollfwlog(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(file, &g_fwlog->fw_log_inq, wait);
	if (skb_queue_len(&g_fwlog->fwlog_queue) > 0)
		mask |= POLLIN | POLLRDNORM;			/* readable */

	return mask;
}

int btmtk_recv(struct hci_dev *hdev, const u8 *data, size_t count)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	const unsigned char *p_left = data;
	int sz_left = count;
	int err;
	const unsigned char *p_h4 = NULL;
	int sz_h4 = 0, adv = 0;

	if (bdev == NULL || hdev == NULL || data == NULL) {
		BTMTK_ERR("%s, invalid parameters!", __func__);
		return -EINVAL;
	}

	while (sz_left > 0) {
		/*  The serial data received from MT7622 BT controller is
		 *  at all time padded around with the STP header and tailer.
		 *
		 *  A full STP packet is looking like
		 *   -----------------------------------
		 *  | STP header  |  H:4   | STP tailer |
		 *   -----------------------------------
		 *  but it doesn't guarantee to contain a full H:4 packet which
		 *  means that it's possible for multiple STP packets forms a
		 *  full H:4 packet that means extra STP header + length doesn't
		 *  indicate a full H:4 frame, things can fragment. Whose length
		 *  recorded in STP header just shows up the most length the
		 *  H:4 engine can handle currently.
		 */
		if (stp_enable == 1) {
			p_h4 = mtk_stp_split(bdev, p_left, sz_left, &sz_h4);
			if (!p_h4)
				break;

			adv = (int)(p_h4 - p_left);
			sz_left -= adv;
			p_left += adv;
		} else {
			p_h4 = data;
			sz_h4 = count;
		}

		bdev->rx_skb = h4_recv_buf(hdev, bdev->rx_skb, p_h4,
					   sz_h4, mtk_recv_pkts,
					   ARRAY_SIZE(mtk_recv_pkts));

		if (IS_ERR(bdev->rx_skb)) {
			err = PTR_ERR(bdev->rx_skb);
			pr_err("Frame reassembly failed (%d)", err);
			bdev->rx_skb = NULL;
			return err;
		}

		sz_left -= sz_h4;
		p_left += sz_h4;
	}

	return 0;
}

int btmtk_dispatch_pkt(struct hci_dev *hdev, struct sk_buff *skb)
{
	static u8 fwlog_picus_blocking_warn;
	static u8 fwlog_fwdump_blocking_warn;

	if ((bt_cb(skb)->pkt_type == HCI_ACLDATA_PKT) &&
			skb->data[0] == 0x6f &&
			skb->data[1] == 0xfc) {
		static int dump_data_counter;
		static int dump_data_length;

		dump_data_counter++;
		dump_data_length += skb->len;

		/* picus or syslog */
		/* print dump data to console */
		if (dump_data_counter % 1000 == 0) {
			BTMTK_INFO("%s: FW dump on-going, total_packet = %d, total_length = %d",
					__func__,
					dump_data_counter,
					dump_data_length);
		}

		/* print dump data to console */
		if (dump_data_counter < 20)
			BTMTK_INFO("%s: FW dump data (%d): %s",
					__func__, dump_data_counter,
					&skb->data[4]);

		/* In the new generation,
		 * we will check the keyword of coredump
		 * (; coredump end)
		 * Such as : 79xx
		 */
		if (skb->data[skb->len - 4] == 'e' &&
			skb->data[skb->len - 3] == 'n' &&
			skb->data[skb->len - 2] == 'd') {
			/* This is the latest coredump packet. */
			BTMTK_INFO("%s: FW dump end, dump_data_counter = %d",
				__func__, dump_data_counter);
			/* TODO: Chip reset*/
			reset_stack_flag = HW_ERR_CODE_CORE_DUMP;
		}

		if (skb_queue_len(&g_fwlog->fwlog_queue) < FWLOG_ASSERT_QUEUE_COUNT) {
			/* sent picus data to queue, picus tool will log it */
			if (btmtk_skb_enq_fwlog(skb->data, skb->len,
					0, &g_fwlog->fwlog_queue) == 0) {
				wake_up_interruptible(&g_fwlog->fw_log_inq);
				fwlog_fwdump_blocking_warn = 0;
			}
		} else {
			if (fwlog_fwdump_blocking_warn == 0) {
				fwlog_fwdump_blocking_warn = 1;
				pr_warn("btmtk fwlog queue size is full(coredump)");
			}
		}
		return 1;
	} else if ((bt_cb(skb)->pkt_type == HCI_ACLDATA_PKT) &&
			(skb->data[0] == 0xff || skb->data[0] == 0xfe) &&
			skb->data[1] == 0x05) {
		/* Coredump */
		if (skb_queue_len(&g_fwlog->fwlog_queue) < FWLOG_QUEUE_COUNT) {
			if (btmtk_skb_enq_fwlog(skb->data, skb->len,
					0, &g_fwlog->fwlog_queue) == 0) {
				wake_up_interruptible(&g_fwlog->fw_log_inq);
				fwlog_picus_blocking_warn = 0;
			}
		} else {
			if (fwlog_picus_blocking_warn == 0) {
				fwlog_picus_blocking_warn = 1;
				pr_warn("btmtk fwlog queue size is full(picus)");
			}
		}
		return 1;
	} else if (memcmp(skb->data, RESET_EVENT, sizeof(RESET_EVENT)) == 0) {
		BTMTK_INFO("%s: Get RESET_EVENT", __func__);
		/* Need confirm with Shawn */
		/* if (bdev->bt_cfg.support_auto_picus == true) {
		 * if (btmtk_picus_enable(bdev) < 0) {
		 * BTMTK_ERR("send picus filter param failed");
		 * }
		}
		*/
	}
	return 0;
}

int btmtk_dispatch_acl(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);

	btmtk_dispatch_pkt(hdev, skb);

	if (skb->data[0]== 0x6f && skb->data[1]== 0xfc && skb->len > 12) {
		/* coredump data done
		 * For Example : TotalTimeForDump=0xxxxxxx, (xx secs)
		 */
		if (skb->data[4]== 0x54 && skb->data[5] == 0x6F &&
			skb->data[6]== 0x74 && skb->data[7] == 0x61 &&
			skb->data[8]== 0x6C && skb->data[9] == 0x54 &&
			skb->data[10]== 0x69 && skb->data[11] == 0x6D &&
			skb->data[12]== 0x65) {
			/* coredump end, do reset */
			BTMTK_INFO("%s coredump done", __func__);
			msleep(3000);
			bdev->subsys_reset= 1;
		}
		return 1;
	} else if (skb->data[0]== 0xff && skb->data[1] == 0x05) {
		BTMTK_DBG("%s correct picus log by ACL", __func__);
		return 1;
	}
	return 0;
}

int btmtk_dispatch_event(struct hci_dev *hdev, struct sk_buff *skb)
{

	/* For Picus */
	if (skb->data[0] == 0xff && skb->data[2] == 0x50) {
		btmtk_dispatch_pkt(hdev, skb);
		return 1;
	}
	return 0;
}

int btmtk_recv_acl(struct hci_dev *hdev, struct sk_buff *skb)
{
	int err = 0, skip_pkt = 0;

	skip_pkt = btmtk_dispatch_acl(hdev, skb);
	if(skip_pkt == 0)
		err = hci_recv_frame(hdev, skb);

	return 0;
}


int btmtk_recv_event(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	//struct hci_event_hdr *hdr = (void *)skb->data;
	int err = 0, skip_pkt = 0;

	/* Fix up the vendor event id with 0xff for vendor specific instead
	 * of 0xe4 so that event send via monitoring socket can be parsed
	 * properly.
	 */
	/*if (hdr->evt == 0xe4) {
		BTMTK_DBG("%s hdr->evt is %02x", __func__, hdr->evt);
		hdr->evt = HCI_EV_VENDOR;
	}*/

	/* When someone waits for the WMT event, the skb is being cloned
	 * and being processed the events from there then.
	 */
	if (test_bit(BTMTKUART_TX_WAIT_VND_EVT, &bdev->tx_state)) {
		bdev->evt_skb = skb_clone(skb, GFP_KERNEL);

		if (!bdev->evt_skb) {
			err = -ENOMEM;
			BTMTK_ERR("%s WMT event, clone to evt_skb failed, err = %d", __func__, err);
			goto err_out;
		}

		if (test_and_clear_bit(BTMTKUART_TX_WAIT_VND_EVT, &bdev->tx_state)) {
			BTMTK_DBG("%s clear bit BTMTKUART_TX_WAIT_VND_EVT", __func__);
			wake_up(&bdev->p_wait_event_q);
			BTMTK_DBG("%s wake_up p_wait_event_q", __func__);
		}
		goto err_out;
	}
	BTMTK_DBG_RAW(skb->data, skb->len, "%s, recv evt(hci_recv_frame)", __func__);

	skip_pkt = btmtk_dispatch_event(hdev, skb);
	if(skip_pkt == 0)
		err = hci_recv_frame(hdev, skb);

	if (err < 0) {
		BTMTK_ERR("%s hci_recv_failed, err = %d", __func__, err);
		goto err_free_skb;
	}

	return 0;

err_free_skb:
	kfree_skb(bdev->evt_skb);
	bdev->evt_skb = NULL;

err_out:
	return err;
}

int btmtk_main_send_cmd(struct hci_dev *hdev, const uint8_t *cmd,
		const int cmd_len, const uint8_t *event, const int event_len, int retry,
		int endpoint, const int tx_state, bool wmt_cmd)
{
	struct sk_buff *skb = NULL;
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	int ret = 0;

	skb = alloc_skb(cmd_len + BT_SKB_RESERVE, GFP_ATOMIC);
	if (skb == NULL) {
		BTMTK_ERR("%s allocate skb failed!!", __func__);
		goto err_free_skb;
	}
	/* Reserv for core and drivers use */
	skb_reserve(skb , 7);
	bt_cb(skb)->pkt_type = HCI_COMMAND_PKT;
	memcpy(skb->data, cmd, cmd_len);
	skb->len = cmd_len;

	if (stp_enable == 1)
		skb = mtk_add_stp(bdev, skb);

	set_bit(tx_state, &bdev->tx_state);
	btmtk_cif_send_cmd(hdev, skb->data, skb->len, 5, 0, 0);
	ret = wait_event_timeout(bdev->p_wait_event_q,
		bdev->evt_skb != NULL || tx_state == BTMTKUART_TX_SKIP_VENDOR_EVT,
		5 * HZ);
	if (ret > 0)
		ret = btmtk_cif_receive_evt(hdev,
				event, event_len, 10, -1, tx_state);
	else
		BTMTK_ERR("%s wait event timeout!!", __func__);

err_free_skb:
	kfree_skb(skb);
	if (bdev->evt_skb != NULL) {
		kfree_skb(bdev->evt_skb);
		bdev->evt_skb = NULL;
	}
	return ret;
}

static void btmtk_load_code_from_bin(u8 **image, char *bin_name,
					 struct device *dev, u32 *code_len)
{
	const struct firmware *fw_entry;
	int err = 0;
	int retry = 10;

	do {
		err = request_firmware(&fw_entry, bin_name, dev);
		if (err == 0) {
			break;
		} else if (retry <= 0) {
			*image = NULL;
			pr_err("%s: request_firmware %d times fail!!! err = %d", __func__, 10, err);
			return;
		}
		pr_err("%s: request_firmware fail!!! err = %d, retry = %d", __func__, err, retry);
		msleep(100);
	} while (retry-- > 0);

	*image = kzalloc(fw_entry->size, GFP_KERNEL);
	if (*image == NULL) {
		pr_err("%s: kzalloc failed!! error code = %d", __func__, err);
		return;
	}

	memcpy(*image, fw_entry->data, fw_entry->size);
	*code_len = fw_entry->size;

	release_firmware(fw_entry);
}

int btmtk_load_rom_patch_766x(struct hci_dev *hdev)
{
	char *tmp_str;
	s32 sent_len;
	u32 patch_len = 0;
	u32 cur_len = 0;
	int first_block = 1;
	u8 phase;
	u32 written = 0;
	int ret = 0;

	u8 *pos;
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	/* To-Do, for event check */
	/* u8 event[] = {0x04, 0xE4, 0x05, 0x02, 0x01, 0x01, 0x00, 0x00}; */

	unsigned char		*rom_patch;
	unsigned char		*rom_patch_bin_file_name;
	unsigned int		rom_patch_len = 0;

	struct sk_buff *skb = NULL;

	rom_patch_bin_file_name = kzalloc(MAX_BIN_FILE_NAME_LEN, GFP_KERNEL);

	/* To-Do
	 * Read CHIP ID for bin_file_name
	 */
	memcpy(rom_patch_bin_file_name, "mt7915_patch_e2_hdr.bin", 23);
	//memcpy(rom_patch_bin_file_name, "mt7663_patch_e2_hdr.bin", 23);

	btmtk_load_code_from_bin(&rom_patch,
			rom_patch_bin_file_name, NULL,
			&rom_patch_len);

	ret = btmtk_send_wmt_get_patch_semaphore(hdev);
	if (ret != 2) {
		BTMTK_INFO("%s: skip download\n", __func__);
		clear_bit(BTMTKUART_REQUIRED_DOWNLOAD, &bdev->tx_state);
		goto done;
	}

	tmp_str = rom_patch;

	tmp_str = rom_patch + 16;
	BTMTK_INFO("%s: platform = %c%c%c%c\n", __func__, tmp_str[0], tmp_str[1], tmp_str[2], tmp_str[3]);

	tmp_str = rom_patch + 20;
	BTMTK_INFO("%s: HW/SW version = %c%c%c%c\n", __func__, tmp_str[0], tmp_str[1], tmp_str[2], tmp_str[3]);

	tmp_str = rom_patch + 24;
	BTMTK_INFO("loading rom patch...\n");

	cur_len = 0x00;
	patch_len = rom_patch_len - PATCH_INFO_SIZE;

	BTMTK_INFO("%s: patch_len = %d\n", __func__, patch_len);

	BTMTK_INFO("%s: loading rom patch...\n", __func__);


	pos = kmalloc(UPLOAD_PATCH_UNIT, GFP_ATOMIC);

	/* loading rom patch */
	while (1) {
		s32 sent_len_max = UPLOAD_PATCH_UNIT - PATCH_HEADER_SIZE;

		sent_len = (patch_len - cur_len) >= sent_len_max ? sent_len_max : (patch_len - cur_len);

		if (sent_len > 0) {
			if (first_block == 1) {
				if (sent_len < sent_len_max)
					phase = PATCH_PHASE3;
				else
					phase = PATCH_PHASE1;
				first_block = 0;
			} else if (sent_len == sent_len_max) {
				if (patch_len - cur_len == sent_len_max)
					phase = PATCH_PHASE3;
				else
					phase = PATCH_PHASE2;
			} else {
				phase = PATCH_PHASE3;
			}


			/* prepare HCI header */
			pos[0] = 0x02;
			pos[1] = 0x6F;
			pos[2] = 0xFC;
			pos[3] = (sent_len + 5) & 0xFF;
			pos[4] = ((sent_len + 5) >> 8) & 0xFF;

			/* prepare WMT header */
			pos[5] = 0x01;
			pos[6] = 0x01;
			pos[7] = (sent_len + 1) & 0xFF;
			pos[8] = ((sent_len + 1) >> 8) & 0xFF;

			pos[9] = phase;
			memcpy(&pos[10], rom_patch + PATCH_INFO_SIZE + cur_len,
				sent_len);
			ret = btmtk_main_send_cmd(hdev, pos, sent_len + PATCH_HEADER_SIZE, NULL, -1, -1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);
			if (ret == 0) {
				BTMTK_INFO("%s: send patch failed, terminate", __func__);
				goto done;
			}
			cur_len = cur_len + sent_len;
			BTMTK_INFO("%s: sent_len = %d, cur_len = %d, phase = %d, written = %d", __func__, sent_len, cur_len, phase, written);

			/*if (btmtk_usb_get_rom_patch_result() < 0)
				goto error2;*/

		} else {
			btmtk_send_wmt_reset(hdev);
			BTMTK_INFO("%s: loading rom patch... Done", __func__);
			if (bdev->subsys_reset == 1) {
				skb = alloc_skb(BT_SKB_RESERVE, GFP_ATOMIC);
				if (skb == NULL) {
					BTMTK_ERR("%s allocate skb failed!!", __func__);
				} else {
					BTMTK_INFO("%s: send hw_err!!", __func__);
					hci_skb_pkt_type(skb) = HCI_EVENT_PKT;
					skb->data[0] = 0x10;
					skb->data[1] = 0x01;
					skb->data[2] = 0x01;
					hci_recv_frame(hdev, skb);
				}
				bdev->subsys_reset = 0;
			}
			clear_bit(BTMTKUART_REQUIRED_DOWNLOAD, &bdev->tx_state);
			break;
		}
	}
done:
	ret = btmtk_send_wmt_release_patch_semaphore(hdev);
	if (ret == 3) {
		BTMTK_INFO("%s: release semaphore\n", __func__);
	}
	return ret;
}

static int btmtk_calibration_flow(struct hci_dev *hdev)
{
	btmtk_cif_send_calibration(hdev);
	BTMTK_INFO("%s done", __func__);
	return 0;
}

int btmtk_send_wmt_reset(struct hci_dev *hdev)
{
	/* Support 7668 and 7663 */
	u8 cmd[] = { 0x01, 0x6F, 0xFC, 0x05, 0x01, 0x07, 0x01, 0x00, 0x04 };
	/* To-Do, for event check */
	/* u8 event[] = { 0x04, 0xE4, 0x05, 0x02, 0x07, 0x01, 0x00, 0x00 }; */

	btmtk_main_send_cmd(hdev, cmd, sizeof(cmd), NULL, -1, -1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);

	BTMTK_INFO("%s done", __func__);
	return 0;
}

int btmtk_send_wmt_get_patch_semaphore(struct hci_dev *hdev)
{
	int ret = -1;
	u8 cmd[] = { 0x01, 0x6F, 0xFC, 0x05, 0x01, 0x17, 0x01, 0x00, 0x01 };
	/* To-Do, for event check */
	/* u8 event[] = { 0x04, 0xE4, 0x05, 0x02, 0x17, 0x01, 0x00}; */
	u8 event[] = {0xE4, 0x05, 0x02, 0x17, 0x01, 0x00};
	BTMTK_INFO("%s send", __func__);

	ret = btmtk_main_send_cmd(hdev, cmd, sizeof(cmd), event, sizeof(event), -1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);

	return ret;
}

int btmtk_send_wmt_release_patch_semaphore(struct hci_dev *hdev)
{
	/* Support 7668 and 7663 */
	int ret = -1;
	u8 cmd[] = { 0x01, 0x6F, 0xFC, 0x05, 0x01, 0x17, 0x01, 0x00, 0x00 };
	/* To-Do, for event check */
	/* u8 event[] = { 0x04, 0xE4, 0x05, 0x02, 0x17, 0x01, 0x00}; */
	u8 event[] = {0xE4, 0x05, 0x02, 0x17, 0x01, 0x00};
	BTMTK_INFO("%s send", __func__);

	ret = btmtk_main_send_cmd(hdev, cmd, sizeof(cmd), event, sizeof(event), -1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);

	return ret;
}

int btmtk_send_wmt_power_on_cmd_766x(struct hci_dev *hdev)
{
	/* Support 7668 and 7663 */
	u8 cmd[] = { 0x01, 0x6F, 0xFC, 0x06, 0x01, 0x06, 0x02, 0x00, 0x00, 0x01 };
	/* To-Do, for event check */
	/* u8 event[] = { 0x04, 0xE4, 0x05, 0x02, 0x17, 0x01, 0x00 }; */

	btmtk_main_send_cmd(hdev, cmd, sizeof(cmd), NULL, -1, -1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);

	BTMTK_INFO("%s done", __func__);
	return 0;
}

int btmtk_send_wmt_power_off_cmd_766x(struct hci_dev *hdev)
{
	/* Support 7668 and 7663 */
	u8 cmd[] = { 0x01, 0x6F, 0xFC, 0x06, 0x01, 0x06, 0x02, 0x00, 0x00, 0x00 };
	/* To-Do, for event check */
	/* u8 event[] = { 0x04, 0xE4, 0x05, 0x02, 0x06, 0x01, 0x00 }; */

	btmtk_main_send_cmd(hdev, cmd, sizeof(cmd), NULL, -1, -1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);

	BTMTK_INFO("%s done", __func__);
	return 0;
}

int btmtk_uart_send_wakeup_cmd(struct hci_dev *hdev)
{
	u8 cmd[] = { 0x01, 0x6f, 0xfc, 0x01, 0xFF };
	/* To-Do, for event check */
	/* u8 event[] = { 0x04, 0xE4, 0x06, 0x02, 0x03, 0x02, 0x00, 0x00, 0x03}; */

	if (uart_baudrate_set == 0) {
		BTMTK_INFO("%s uart baudrate is 115200, no need", __func__);
		return 0;
	}
	btmtk_main_send_cmd(hdev, cmd, sizeof(cmd), NULL, -1, -1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);
	return 0;
}

int btmtk_uart_send_set_uart_cmd(struct hci_dev *hdev, int fc)
{
	/* 921600 */
	u8 cmd_921600[] = { 0x01, 0x6F, 0xFC, 0x09,
		0x01, 0x04, 0x05, 0x00, 0x01, 0x00, 0x10, 0x0E, 0x00};
	/* 230400
	u8 cmd_230400[] = { 0x01, 0x6F, 0xFC, 0x09,
		0x01, 0x04, 0x05, 0x00, 0x01, 0x00, 0x84, 0x03, 0x00};*/
	/* To-Do, for event check */
	/* u8 event[] = {0x04, 0xE4, 0x06, 0x02, 0x04, 0x02, 0x00, 0x00, 0x01}; */
	u8 *cmd = NULL;
	int cmd_len = 0;

	/* if (sUartCfg->iBaudrate == 921600) { */
		cmd = cmd_921600;
		cmd_len = sizeof(cmd_921600);
	/*
	} else if (sUartCfg->iBaudrate == 230400) {
		cmd = cmd_230400;
		cmd_len = sizeof(cmd_230400);
	} else {
		BTMTK_INFO("%s uart baudrate(%d) is not supported",
				__func__, sUartCfg->iBaudrate);
		return 0;
	}*/

	if (fc == UART_HW_FC)
		cmd[12] = 0x40;

	btmtk_main_send_cmd(hdev,
			cmd, cmd_len, NULL, -1,
			-1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);

	uart_baudrate_set = 1;
	return 0;
}

int btmtk_send_set_stp_cmd(struct hci_dev *hdev)
{
	u8 cmd[] = { 0x01, 0x6F, 0xFC, 0x09, 0x01, 0x04, 0x05, 0x00, 0x03, 0x11, 0x0E, 0x00, 0x00};
	/* To-Do, for event check */
	/* u8 event[] = { 0x04, 0xE4, 0x06, 0x02, 0x04, 0x02, 0x00, 0x00, 0x03}; */

	btmtk_main_send_cmd(hdev,
		cmd, sizeof(cmd), NULL, -1,
		-1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);

	BTMTK_INFO("%s done", __func__);
	return 0;
}

int btmtk_send_set_stp1_cmd(struct hci_dev *hdev)
{
	u8 cmd[] = {0x01, 0x6F, 0xFC, 0x0C, 0x01, 0x08, 0x08, 0x00, 0x02, 0x01, 0x00, 0x01, 0x08, 0x00, 0x00, 0x80};
	/* To-Do, for event check */
	/* u8 event[] = {0x04, 0xE4, 0x10, 0x02, 0x08,
			0x0C, 0x00, 0x00, 0x00, 0x00, 0x01, 0x08, 0x00, 0x00, 0x80, 0x63, 0x76, 0x00, 0x00}; */

	btmtk_main_send_cmd(hdev,
		cmd, sizeof(cmd), NULL, -1,
		-1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);

	BTMTK_INFO("%s done", __func__);
	return 0;
}

static int btmtk_send_hci_tci_set_sleep_cmd_766x(struct hci_dev *hdev)
{
	u8 cmd[] = { 0x01, 0x7A, 0xFC, 0x07,
		0x05, 0x40, 0x06, 0x40, 0x06, 0x00, 0x00 };
	/* To-Do, for event check */
	/* u8 event[] = { 0x04, 0x0E, 0x04, 0x01, 0x7A, 0xFC, 0x00 }; */

	btmtk_main_send_cmd(hdev,
		cmd, sizeof(cmd), NULL, -1,
		-1, -1, BTMTKUART_TX_WAIT_VND_EVT, 0);

	BTMTK_INFO("%s done", __func__);

	return 0;
}


int btmtk_send_init_cmds(struct hci_dev *hdev)
{
	BTMTK_INFO("%s", __func__);

	btmtk_calibration_flow(hdev);
	btmtk_send_wmt_power_on_cmd_766x(hdev);
	btmtk_send_hci_tci_set_sleep_cmd_766x(hdev);
	return 0;
}


int btmtk_send_deinit_cmds(struct hci_dev *hdev)
{
	BTMTK_INFO("%s", __func__);
	return 0;
}

int btmtk_fops_init(void)
{
	static int BT_majorfwlog;
	dev_t devIDfwlog = MKDEV(BT_majorfwlog, 0);
	int ret = 0;
	int cdevErr = 0;
	int majorfwlog = 0;

	BTMTK_INFO("%s: Start", __func__);

	if (g_fwlog == NULL) {
		g_fwlog = kzalloc(sizeof(*g_fwlog), GFP_KERNEL);
		if (!g_fwlog) {
			BTMTK_ERR("%s: alloc memory fail (g_data)", __func__);
			return -1;
		}
	}

	BTMTK_INFO("%s: g_fwlog init", __func__);
	spin_lock_init(&g_fwlog->fwlog_lock);
	skb_queue_head_init(&g_fwlog->fwlog_queue);
	init_waitqueue_head(&(g_fwlog->fw_log_inq));

	ret = alloc_chrdev_region(&devIDfwlog, 0, 1, "BT_chrdevfwlog");
	if (ret) {
		BT_ERR("%s: fail to allocate chrdev", __func__);
		return ret;
	}

	BT_majorfwlog = majorfwlog = MAJOR(devIDfwlog);

	cdev_init(&g_fwlog->BT_cdevfwlog, &BT_fopsfwlog);
	g_fwlog->BT_cdevfwlog.owner = THIS_MODULE;

	cdevErr = cdev_add(&g_fwlog->BT_cdevfwlog, devIDfwlog, 1);
	if (cdevErr)
		goto error;

	g_fwlog->pBTClass = class_create(THIS_MODULE, "BT_chrdevfwlog");
	if (IS_ERR(g_fwlog->pBTClass)) {
		BT_ERR("%s: class create fail, error code(%ld)",
				__func__, PTR_ERR(g_fwlog->pBTClass));
		goto err1;
	}

	g_fwlog->pBTDevfwlog = device_create(g_fwlog->pBTClass,
			NULL, devIDfwlog, NULL, "stpbtfwlog");
	if (IS_ERR(g_fwlog->pBTDevfwlog)) {
		BT_ERR("%s: device(stpbtfwlog) create fail, error code(%ld)",
				__func__, PTR_ERR(g_fwlog->pBTDevfwlog));
		goto error;
	}
	BT_INFO("%s: BT_majorfwlog %d, devIDfwlog %d",
			__func__, BT_majorfwlog, devIDfwlog);

	g_fwlog->g_devIDfwlog = devIDfwlog;

	return 0;

err1:
	if (g_fwlog->pBTClass) {
		class_destroy(g_fwlog->pBTClass);
		g_fwlog->pBTClass = NULL;
	}

error:
	if (cdevErr == 0)
		cdev_del(&g_fwlog->BT_cdevfwlog);

	if (ret == 0)
		unregister_chrdev_region(devIDfwlog, 1);

	return -1;
}

int btmtk_fops_exit(void)
{
	dev_t devIDfwlog = g_fwlog->g_devIDfwlog;

	BT_INFO("%s: Start\n", __func__);
	if (g_fwlog->pBTDevfwlog) {
		device_destroy(g_fwlog->pBTClass, devIDfwlog);
		g_fwlog->pBTDevfwlog = NULL;
	}

	if (g_fwlog->pBTClass) {
		class_destroy(g_fwlog->pBTClass);
		g_fwlog->pBTClass = NULL;
	}
	BT_INFO("%s: pBTDevfwlog, pBTClass done\n", __func__);
	cdev_del(&g_fwlog->BT_cdevfwlog);
	unregister_chrdev_region(devIDfwlog, 1);
	BT_INFO("%s: BT_chrdevfwlog driver removed.\n", __func__);
	kfree(g_fwlog);

	return 0;
}

/**
 * Kernel HCI Interface Registeration
 */
static int bt_flush(struct hci_dev *hdev)
{
	return 0;
}

static int bt_close(struct hci_dev *hdev)
{
	BTMTK_INFO("%s", __func__);
	btmtk_send_deinit_cmds(hdev);
	clear_bit(HCI_RUNNING, &hdev->flags);

	return 0;
}

static int bt_open(struct hci_dev *hdev)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	BTMTK_INFO("%s, BTMTK uart version: %s", __func__, VERSION);
	//To-Do power-init sequencez
	if (test_bit(BTMTKUART_REQUIRED_DOWNLOAD, &bdev->tx_state)) {
		BTMTK_DBG("%s clear bit BTMTKUART_TX_WAIT_VND_EVT", __func__);
		clear_bit(BTMTKUART_REQUIRED_DOWNLOAD, &bdev->tx_state);
		return -EAGAIN;
	} else
		btmtk_send_init_cmds(hdev);

	set_bit(HCI_RUNNING, &hdev->flags);
	return 0;
}

static int bt_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{

	struct btmtk_dev *bdev;
	bdev = hci_get_drvdata(hdev);

	memcpy(skb_push(skb, 1), &bt_cb(skb)->pkt_type, 1);
	if (stp_enable == 1)
		skb = mtk_add_stp(bdev, skb);

	btmtk_cif_send_cmd(hdev, skb->data, skb->len, 5, 0, 0);

	return 0;
}

static int bt_setup(struct hci_dev *hdev)
{
	//struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	BTMTK_INFO("%s", __func__);

	//Download patch flow.
	/*if (test_bit(BTMTKUART_REQUIRED_DOWNLOAD, &bdev->tx_state)) {
		BTMTK_DBG("%s clear bit BTMTKUART_TX_WAIT_VND_EVT", __func__);
		clear_bit(BTMTKUART_REQUIRED_DOWNLOAD, &bdev->tx_state);
		return -EAGAIN;
	} else {
		BTMTK_DBG("%s send_init_cmd", __func__);
		btmtk_send_init_cmds(hdev);
	}*/

	return 0;
}

int btmtk_free_hci_device(struct btmtk_dev *bdev, int hci_bus_type)
{
	hci_unregister_dev(bdev->hdev);
	hci_free_dev(bdev->hdev);
	BTMTK_INFO("%s done", __func__);
	return 0;
}

int btmtk_allocate_hci_device(struct btmtk_dev *bdev, int hci_bus_type)
{
	struct hci_dev *hdev;
	int err = 0;

	/* Add hci device */
	hdev = hci_alloc_dev();
	if (!hdev)
		return -ENOMEM;
	hdev->bus = hci_bus_type;

	bdev->hdev = hdev;
	hci_set_drvdata(hdev, bdev);

	/* register hci callback */
	hdev->open	   = bt_open;
	hdev->close    = bt_close;
	hdev->flush    = bt_flush;
	hdev->send	   = bt_send_frame;
	hdev->setup    = bt_setup;

	init_waitqueue_head(&bdev->p_wait_event_q);
	SET_HCIDEV_DEV(hdev, bdev->tty->dev);

	err = hci_register_dev(hdev);
	/* After hci_register_dev completed
	 * It will set dev_flags to HCI_SETUP
	 * That cause vendor_lib create socket failed
	 */
	if (err < 0) {
		BTMTK_INFO("%s can't register", __func__);
		hci_free_dev(hdev);
		return err;
	}

	/*set_bit(HCI_RUNNING, &hdev->flags);
	set_bit(HCI_QUIRK_RAW_DEVICE, &hdev->quirks);*/
#if BLUEDROID
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,4,0))
	test_and_clear_bit(HCI_SETUP, &hdev->dev_flags);
#else
	hci_dev_test_and_clear_flag(hdev, HCI_SETUP);
#endif
#endif
	set_bit(BTMTKUART_REQUIRED_DOWNLOAD, &bdev->tx_state);

	BTMTK_INFO("%s done", __func__);
	return 0;
}

/**
 * Kernel Module init/exit Functions
 */
static int __init main_driver_init(void)
{
	int ret = -1;

	BTMTK_INFO("%s, BTMTK uart version: %s", __func__, VERSION);
	ret = main_init();
	if (ret < 0)
		return ret;

	ret = btmtk_cif_register();
	if (ret < 0) {
		BTMTK_ERR("*** USB registration failed(%d)! ***", ret);
		return ret;
	}

	btmtk_fops_init();

	BTMTK_INFO("%s: Done", __func__);
	return 0;
}

static void __exit main_driver_exit(void)
{
	BTMTK_INFO("%s", __func__);
	btmtk_fops_exit();
	btmtk_cif_deregister();
	main_exit();
}
module_init(main_driver_init);
module_exit(main_driver_exit);

/**
 * Module Common Information
 */
MODULE_DESCRIPTION("Mediatek Bluetooth Driver");
MODULE_VERSION(VERSION SUBVER);
MODULE_LICENSE("GPL");
