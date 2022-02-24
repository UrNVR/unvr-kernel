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
#ifndef __BTMTK_MAIN_H__
#define __BTMTK_MAIN_H__
#include "btmtk_define.h"
#include "btmtk_chip_if.h"


int btmtk_allocate_hci_device(struct btmtk_dev *bdev, int hci_bus_type);
int btmtk_free_hci_device(struct btmtk_dev *bdev, int hci_bus_type);
int btmtk_recv(struct hci_dev *hdev, const u8 *data, size_t count);
int btmtk_recv_event(struct hci_dev *hdev, struct sk_buff *skb);
int btmtk_recv_acl(struct hci_dev *hdev, struct sk_buff *skb);
int btmtk_dispatch_event(struct hci_dev *hdev, struct sk_buff *skb);
int btmtk_dispatch_acl(struct hci_dev *hdev, struct sk_buff *skb);
int btmtk_send_init_cmds(struct hci_dev *hdev);
int btmtk_send_deinit_cmds(struct hci_dev *hdev);
int btmtk_main_send_cmd(struct hci_dev *hdev, const uint8_t *cmd,	const int cmd_len, const uint8_t *event,
		const int event_len, int retry,	int endpoint, const int tx_state, bool wmt_cmd);
int btmtk_send_wmt_reset(struct hci_dev *hdev);
int btmtk_send_wmt_get_patch_semaphore(struct hci_dev *hdev);
int btmtk_send_wmt_release_patch_semaphore(struct hci_dev *hdev);
int btmtk_send_wmt_power_on_cmd_766x(struct hci_dev *hdev);
int btmtk_send_wmt_power_off_cmd_766x(struct hci_dev *hdev);
int btmtk_load_rom_patch_766x(struct hci_dev *hdev);
int btmtk_uart_send_wakeup_cmd(struct hci_dev *hdev);
int btmtk_uart_send_set_uart_cmd(struct hci_dev *hdev, int fc);

#if LOAD_ROM_PATCH_TO_FLASH
int btmtk_load_flash_programing(struct hci_dev *hdev);
#endif
void mtk_stp_enable(void);
int btmtk_send_set_stp_cmd(struct hci_dev *hdev);
int btmtk_send_set_stp1_cmd(struct hci_dev *hdev);

int btmtk_fops_openfwlog(struct inode *inode, struct file *file);
int btmtk_fops_closefwlog(struct inode *inode, struct file *file);
ssize_t btmtk_fops_readfwlog(struct file *filp,
			char __user *buf, size_t count, loff_t *f_pos);
ssize_t btmtk_fops_writefwlog(struct file *filp,
			const char __user *buf, size_t count, loff_t *f_pos);
unsigned int btmtk_fops_pollfwlog(struct file *filp, poll_table *wait);
long btmtk_fops_unlocked_ioctlfwlog(struct file *filp,
			unsigned int cmd, unsigned long arg);

//static inline struct sk_buff *mtk_add_stp(struct btmtk_dev *bdev, struct sk_buff *skb);

#define hci_dev_test_and_clear_flag(hdev, nr)  test_and_clear_bit((nr), (hdev)->dev_flags)

/* h4_recv */
#define hci_skb_pkt_type(skb) bt_cb((skb))->pkt_type
#define hci_skb_expect(skb) bt_cb((skb))->expect
#define hci_skb_opcode(skb) bt_cb((skb))->hci.opcode

/* HCI bus types */
#define HCI_VIRTUAL	0
#define HCI_USB		1
#define HCI_PCCARD	2
#define HCI_UART	3
#define HCI_RS232	4
#define HCI_PCI		5
#define HCI_SDIO	6
#define HCI_SPI		7
#define HCI_I2C		8
#define HCI_SMD		9

#define FWLOG_QUEUE_COUNT			400
#define FWLOG_ASSERT_QUEUE_COUNT		45000
#define FWLOG_BLUETOOTH_KPI_QUEUE_COUNT		200
#define HCI_MAX_COMMAND_SIZE			255
#define HCI_MAX_COMMAND_BUF_SIZE		(HCI_MAX_COMMAND_SIZE * 3)
#define HCI_MAX_ISO_SIZE	340

struct btmtk_fops_fwlog {
	dev_t g_devIDfwlog;
	struct cdev BT_cdevfwlog;
	wait_queue_head_t fw_log_inq;
	struct sk_buff_head fwlog_queue;
	struct class *pBTClass;
	struct device *pBTDevfwlog;
	spinlock_t fwlog_lock;
	u8 btmtk_bluetooth_kpi;
};

enum {
	HW_ERR_NONE = 0x00,
	HW_ERR_CODE_CHIP_RESET = 0xF0,
	HW_ERR_CODE_USB_DISC = 0xF1,
	HW_ERR_CODE_CORE_DUMP = 0xF2,
	HW_ERR_CODE_POWER_ON = 0xF3,
	HW_ERR_CODE_POWER_OFF = 0xF4,
	HW_ERR_CODE_WOBLE = 0xF5,
	HW_ERR_CODE_SET_SLEEP_CMD = 0xF6,
	HW_ERR_CODE_RESET_STACK_AFTER_WOBLE = 0xF7,
};

struct h4_recv_pkt {
	u8  type;	/* Packet type */
	u8  hlen;	/* Header length */
	u8  loff;	/* Data length offset in header */
	u8  lsize;	/* Data length field size */
	u16 maxlen;	/* Max overall packet length */
	int (*recv)(struct hci_dev *hdev, struct sk_buff *skb);
};

#define H4_RECV_ACL \
	.type = HCI_ACLDATA_PKT, \
	.hlen = HCI_ACL_HDR_SIZE, \
	.loff = 2, \
	.lsize = 2, \
	.maxlen = HCI_MAX_FRAME_SIZE \

#define H4_RECV_SCO \
	.type = HCI_SCODATA_PKT, \
	.hlen = HCI_SCO_HDR_SIZE, \
	.loff = 2, \
	.lsize = 1, \
	.maxlen = HCI_MAX_SCO_SIZE

#define H4_RECV_EVENT \
	.type = HCI_EVENT_PKT, \
	.hlen = HCI_EVENT_HDR_SIZE, \
	.loff = 1, \
	.lsize = 1, \
	.maxlen = HCI_MAX_EVENT_SIZE

extern struct btmtk_dev *g_bdev;
#endif /* __BTMTK_MAIN_H__ */
