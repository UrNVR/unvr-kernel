/*
 * Copyright 2017, Amazon.com, Inc. or its affiliates. All Rights Reserved
 */

/**
 *  @{
 * @file  al_mod_hal_eth_epe_regs.h
 *
 * @brief Ethernet Parsing Engine registers
 *
 */

#ifndef __AL_HAL_ETH_EPE_REGS_H__
#define __AL_HAL_ETH_EPE_REGS_H__

#include "al_mod_hal_eth_ec_regs.h"

/* EPE Compare table macros */
#define EC_EPE_V3_P_COMP_DATA_DATA_2_STAGE_MASK		0x0007
#define EC_EPE_V3_P_COMP_DATA_DATA_2_STAGE_SHIFT		0

#define EC_EPE_V3_P_COMP_DATA_DATA_2_BRANCH_ID_MASK	0x01F8
#define EC_EPE_V3_P_COMP_DATA_DATA_2_BRANCH_ID_SHIFT	3

#define EC_EPE_V3_P_COMP_MASK_DATA_2_STAGE_MASK		0x0007
#define EC_EPE_V3_P_COMP_MASK_DATA_2_STAGE_SHIFT		0

#define EC_EPE_V3_P_COMP_MASK_DATA_2_BRANCH_ID_MASK	0x01F8
#define EC_EPE_V3_P_COMP_MASK_DATA_2_BRANCH_ID_SHIFT	3

#define EC_EPE_P_COMP_CTRL_CMD_EQ			0x0 /* Equal */
#define EC_EPE_P_COMP_CTRL_CMD_LTE			0x1 /* Lesser Than or Equal */
#define EC_EPE_P_COMP_CTRL_CMD_GTE			0x2 /* Greater Than or Equal */

#define AL_ETH_V3_P_COMP_DATA_DATA_2(branch_id, stage)			\
	((EC_EPE_V3_P_COMP_DATA_DATA_2_BRANCH_ID_MASK &			\
	  ((branch_id) << EC_EPE_V3_P_COMP_DATA_DATA_2_BRANCH_ID_SHIFT)) |	\
	 (EC_EPE_V3_P_COMP_DATA_DATA_2_STAGE_MASK &			\
	  ((stage) << EC_EPE_V3_P_COMP_DATA_DATA_2_STAGE_SHIFT)))

#define AL_ETH_V3_P_COMP_MASK_DATA_2(branch_id, stage)			\
	((EC_EPE_V3_P_COMP_MASK_DATA_2_BRANCH_ID_MASK &			\
	  ((branch_id) << EC_EPE_V3_P_COMP_MASK_DATA_2_BRANCH_ID_SHIFT)) |	\
	 (EC_EPE_V3_P_COMP_MASK_DATA_2_STAGE_MASK &			\
	  ((stage) << EC_EPE_V3_P_COMP_MASK_DATA_2_STAGE_SHIFT)))

#define AL_ETH_V3_P_COMP_DATA(data_data2, data_data1)		\
	((EC_EPE_P_COMP_DATA_DATA_2_MASK &			\
	  ((data_data2) << EC_EPE_P_COMP_DATA_DATA_2_SHIFT)) |	\
	 (EC_EPE_P_COMP_DATA_DATA_1_MASK &			\
	  ((data_data1) << EC_EPE_P_COMP_DATA_DATA_1_SHIFT)))

#define AL_ETH_V3_P_COMP_MASK(mask_data2, mask_data1)		\
	((EC_EPE_P_COMP_MASK_DATA_2_MASK &			\
	  ((mask_data2) << EC_EPE_P_COMP_MASK_DATA_2_SHIFT)) |	\
	 (EC_EPE_P_COMP_MASK_DATA_1_MASK &			\
	  ((mask_data1) << EC_EPE_P_COMP_MASK_DATA_1_SHIFT)))

#define AL_ETH_V3_P_COMP_CTRL(valid, cmd2, cmd1, res)	\
	(((valid) << 31) |				\
	 (EC_EPE_P_COMP_CTRL_CMD_2_MASK &		\
	  ((cmd2) << EC_EPE_P_COMP_CTRL_CMD_2_SHIFT)) |	\
	 (EC_EPE_P_COMP_CTRL_CMD_1_MASK &		\
	  ((cmd1) << EC_EPE_P_COMP_CTRL_CMD_1_SHIFT)) |	\
	 (EC_EPE_P_COMP_CTRL_RES_MASK &			\
	  ((res) << EC_EPE_P_COMP_CTRL_RES_SHIFT)))


#define AL_ETH_EPE_V3_P_COMP_ENTRY(data_val, mask_val, ctrl_val)	\
	{							\
		.data = (data_val),					\
		.mask = (mask_val),					\
		.ctrl = (ctrl_val),					\
	}

/**
 * EPE <=3 extra masks and shifts
 */
/**** res_def register ****/
/* Parsing enable */
#define EC_EPE_RES_DEF_P_SOP_DEF_PARSE_EN (1 << 7)
/* Header offset */
#define EC_EPE_RES_DEF_P_SOP_DEF_HDR_OFFSET_MASK 0x00FF8000
#define EC_EPE_RES_DEF_P_SOP_DEF_HDR_OFFSET_SHIFT 15
/* Action table address */
#define EC_EPE_RES_DEF_DEF_SOP_ACTION_TABLE_ADDR_MASK 0xF8000000
#define EC_EPE_RES_DEF_DEF_SOP_ACTION_TABLE_ADDR_SHIFT 27

#define EC_EPE_ACT_DATA_1_NEXT_PROTO_OFFSET_MASK	AL_FIELD_MASK(5, 0)
#define EC_EPE_ACT_DATA_1_NEXT_PROTO_OFFSET_SHIFT	0
#define EC_EPE_ACT_DATA_1_NEXT_PROTO_AVAIL			(1 << 6)
#define EC_EPE_ACT_DATA_1_DEFAULT_NEXT_PROTO_MASK	AL_FIELD_MASK(11, 7)
#define EC_EPE_ACT_DATA_1_DEFAULT_NEXT_PROTO_SHIFT	7
#define EC_EPE_ACT_DATA_1_PROTO_WR					(1 << 12)
#define EC_EPE_ACT_DATA_1_PROTO_WR_PTR_MASK			AL_FIELD_MASK(22, 13)
#define EC_EPE_ACT_DATA_1_PROTO_WR_PTR_SHIFT		13
#define EC_EPE_ACT_DATA_1_PROTO_WR_SEL				(1 << 23)
#define EC_EPE_ACT_DATA_1_HDR_LEN_WR_SEL			(1 << 24)
#define EC_EPE_ACT_DATA_1_HDR_OFFSET_WR_SEL			(1 << 25)

#define EC_EPE_ACT_DATA_1_PROTO_WR_SEL_CUR		EC_EPE_ACT_DATA_1_PROTO_WR_SEL
#define EC_EPE_ACT_DATA_1_PROTO_WR_SEL_NEXT		(0 << 23)
#define EC_EPE_ACT_DATA_1_HDR_LEN_WR_SEL_CUR		EC_EPE_ACT_DATA_1_HDR_LEN_WR_SEL
#define EC_EPE_ACT_DATA_1_HDR_LEN_WR_SEL_NEXT		(0 << 24)
#define EC_EPE_ACT_DATA_1_HDR_OFFSET_WR_SEL_CUR		EC_EPE_ACT_DATA_1_HDR_OFFSET_WR_SEL
#define EC_EPE_ACT_DATA_1_HDR_OFFSET_WR_SEL_NEXT	(0 << 25)

#define EC_EPE_ACT_DATA_1_NEXT_PROTO_OFFSET(offset)	\
	(EC_EPE_ACT_DATA_1_NEXT_PROTO_OFFSET_MASK &	\
	 ((offset) << EC_EPE_ACT_DATA_1_NEXT_PROTO_OFFSET_SHIFT))

#define EC_EPE_ACT_DATA_1_DEFAULT_NEXT_PROTO(offset)	\
	(EC_EPE_ACT_DATA_1_DEFAULT_NEXT_PROTO_MASK &	\
	 ((offset) << EC_EPE_ACT_DATA_1_DEFAULT_NEXT_PROTO_SHIFT))

#define EC_EPE_ACT_DATA_1_PROTO_WR_PTR(ptr)	\
	(EC_EPE_ACT_DATA_1_PROTO_WR_PTR_MASK &	\
	 ((ptr) << EC_EPE_ACT_DATA_1_PROTO_WR_PTR_SHIFT))

#define EC_EPE_ACT_DATA_2_DATA_OFFSET_MASK			AL_FIELD_MASK(8, 0)
#define EC_EPE_ACT_DATA_2_DATA_OFFSET_SHIFT			0
#define EC_EPE_ACT_DATA_2_DATA_SIZE_MASK			AL_FIELD_MASK(17, 9)
#define EC_EPE_ACT_DATA_2_DATA_SIZE_SHIFT			9
#define EC_EPE_ACT_DATA_2_DATA_WR					(1 << 18)
#define EC_EPE_ACT_DATA_2_DATA_PTR_MASK				AL_FIELD_MASK(28, 19)
#define EC_EPE_ACT_DATA_2_DATA_PTR_SHIFT			19

#define EC_EPE_ACT_DATA_2_DATA_OFFSET(offset)	\
	(EC_EPE_ACT_DATA_2_DATA_OFFSET_MASK &	\
	 ((offset) << EC_EPE_ACT_DATA_2_DATA_OFFSET_SHIFT))

#define EC_EPE_ACT_DATA_2_DATA_SIZE(size)	\
	(EC_EPE_ACT_DATA_2_DATA_SIZE_MASK &	\
	 ((size) << EC_EPE_ACT_DATA_2_DATA_SIZE_SHIFT))

#define EC_EPE_ACT_DATA_2_DATA_PTR(ptr)	\
	(EC_EPE_ACT_DATA_2_DATA_PTR_MASK &	\
	 ((ptr) << EC_EPE_ACT_DATA_2_DATA_PTR_SHIFT))

#define EC_EPE_ACT_DATA_3_DATA_OFFSET_MASK			AL_FIELD_MASK(8, 0)
#define EC_EPE_ACT_DATA_3_DATA_OFFSET_SHIFT			0
#define EC_EPE_ACT_DATA_3_DATA_SIZE_MASK			AL_FIELD_MASK(17, 9)
#define EC_EPE_ACT_DATA_3_DATA_SIZE_SHIFT			9
#define EC_EPE_ACT_DATA_3_DATA_WR					(1 << 18)
#define EC_EPE_ACT_DATA_3_DATA_PTR_MASK				AL_FIELD_MASK(28, 19)
#define EC_EPE_ACT_DATA_3_DATA_PTR_SHIFT			19

#define EC_EPE_ACT_DATA_3_DATA_OFFSET(offset)	\
	(EC_EPE_ACT_DATA_3_DATA_OFFSET_MASK &	\
	 ((offset) << EC_EPE_ACT_DATA_3_DATA_OFFSET_SHIFT))

#define EC_EPE_ACT_DATA_3_DATA_SIZE(size)	\
	(EC_EPE_ACT_DATA_3_DATA_SIZE_MASK &	\
	 ((size) << EC_EPE_ACT_DATA_3_DATA_SIZE_SHIFT))

#define EC_EPE_ACT_DATA_3_DATA_PTR(ptr)	\
	(EC_EPE_ACT_DATA_3_DATA_PTR_MASK &	\
	 ((ptr) << EC_EPE_ACT_DATA_3_DATA_PTR_SHIFT))

#define EC_EPE_ACT_DATA_4_HDR_LEN_OFFSET_MASK		AL_FIELD_MASK(7, 0)
#define EC_EPE_ACT_DATA_4_HDR_LEN_OFFSET_SHIFT		0
#define EC_EPE_ACT_DATA_4_HDR_LEN_SIZE_MASK			AL_FIELD_MASK(12, 8)
#define EC_EPE_ACT_DATA_4_HDR_LEN_SIZE_SHIFT		8
#define EC_EPE_ACT_DATA_4_HDR_LEN_UNITS_MASK		AL_FIELD_MASK(15, 13)
#define EC_EPE_ACT_DATA_4_HDR_LEN_UNITS_SHIFT		13
#define EC_EPE_ACT_DATA_4_HDR_LEN_COMMAND_MASK		AL_FIELD_MASK(17, 16)
#define EC_EPE_ACT_DATA_4_HDR_LEN_COMMAND_SHIFT		16
#define EC_EPE_ACT_DATA_4_HDR_LEN_COMMAND_ADD_DEFAULT_LEN	(1 << 18)
#define EC_EPE_ACT_DATA_4_DEFAULT_HDR_LEN_MASK		AL_FIELD_MASK(27, 20)
#define EC_EPE_ACT_DATA_4_DEFAULT_HDR_LEN_SHIFT		20

#define EC_EPE_ACT_DATA_4_HDR_LEN_OFFSET(offset)	\
	(EC_EPE_ACT_DATA_4_HDR_LEN_OFFSET_MASK &	\
	 ((offset) << EC_EPE_ACT_DATA_4_HDR_LEN_OFFSET_SHIFT))

#define EC_EPE_ACT_DATA_4_HDR_LEN_SIZE(size)	\
	(EC_EPE_ACT_DATA_4_HDR_LEN_SIZE_MASK &	\
	 ((size) << EC_EPE_ACT_DATA_4_HDR_LEN_SIZE_SHIFT))

#define EC_EPE_ACT_DATA_4_HDR_LEN_UNITS(units)	\
	(EC_EPE_ACT_DATA_4_HDR_LEN_UNITS_MASK &	\
	 ((units) << EC_EPE_ACT_DATA_4_HDR_LEN_UNITS_SHIFT))

/*
 * hdr_len_cmd
 * [1:0] :
 *  00 - use 0
 *  01 - use hdr_len from packet data
 *  10 - use packet hdr length data as address to hdr_len_table_1
 *  11 - use packet hdr length data as address to hdr_len_table_2

 * [2] - indicate if the default header length value should be added or not
 *   0 - add default header length
 * NOTE: if (bits [1:0] == '00') and bit [2] is '0' than the default header length is used)
 * [3] - reserved
 */
#define EC_EPE_ACT_DATA_4_HDR_LEN_COMMAND_0					(0x0 << EC_EPE_ACT_DATA_4_HDR_LEN_COMMAND_SHIFT)
#define EC_EPE_ACT_DATA_4_HDR_LEN_COMMAND_FROM_PKT_DATA				(0x1 << EC_EPE_ACT_DATA_4_HDR_LEN_COMMAND_SHIFT)
#define EC_EPE_ACT_DATA_4_HDR_LEN_COMMAND_PKT_HDR_LEN_AS_ADDR_TO_TABLE_1	(0x2 << EC_EPE_ACT_DATA_4_HDR_LEN_COMMAND_SHIFT)
#define EC_EPE_ACT_DATA_4_HDR_LEN_COMMAND_PKT_HDR_LEN_AS_ADDR_TO_TABLE_2	(0x3 << EC_EPE_ACT_DATA_4_HDR_LEN_COMMAND_SHIFT)

#define EC_EPE_ACT_DATA_4_HDR_LEN_COMMAND_DEFAULT_HDR_LEN_ADD			((0 << 2) << EC_EPE_ACT_DATA_4_HDR_LEN_COMMAND_SHIFT)
#define EC_EPE_ACT_DATA_4_HDR_LEN_COMMAND_DEFAULT_HDR_LEN_NO_ADD		((1 << 2) << EC_EPE_ACT_DATA_4_HDR_LEN_COMMAND_SHIFT)

#define EC_EPE_ACT_DATA_4_HDR_LEN_COMMAND_USE_DEFAULT_HDR_LEN			(EC_EPE_ACT_DATA_4_HDR_LEN_COMMAND_DEFAULT_HDR_LEN_ADD |\
										 EC_EPE_ACT_DATA_4_HDR_LEN_COMMAND_0)


#define EC_EPE_ACT_DATA_4_DEFAULT_HDR_LEN(hdr_len)	\
	(EC_EPE_ACT_DATA_4_DEFAULT_HDR_LEN_MASK &	\
	 ((hdr_len) << EC_EPE_ACT_DATA_4_DEFAULT_HDR_LEN_SHIFT))

#define EC_EPE_ACT_DATA_5_END_OF_PARSING			(1 << 0)
#define EC_EPE_ACT_DATA_5_NEXT_STAGE_MASK			AL_FIELD_MASK(3, 1)
#define EC_EPE_ACT_DATA_5_NEXT_STAGE_SHIFT			1
#define EC_EPE_ACT_DATA_5_NEXT_BRANCH_ID_SET_MASK	AL_FIELD_MASK(9, 4)
#define EC_EPE_ACT_DATA_5_NEXT_BRANCH_ID_SET_SHIFT	4
#define EC_EPE_ACT_DATA_5_NEXT_BRANCH_ID_VAL_MASK	AL_FIELD_MASK(15, 10)
#define EC_EPE_ACT_DATA_5_NEXT_BRANCH_ID_VAL_SHIFT	10
#define EC_EPE_ACT_DATA_5_NEXT_CTRL_BIT_SET_MASK	AL_FIELD_MASK(23, 16)
#define EC_EPE_ACT_DATA_5_NEXT_CTRL_BIT_SET_SHIFT	16
#define EC_EPE_ACT_DATA_5_NEXT_CTRL_BIT_VAL_MASK	AL_FIELD_MASK(31, 24)
#define EC_EPE_ACT_DATA_5_NEXT_CTRL_BIT_VAL_SHIFT	24

#define EC_EPE_ACT_DATA_5_NEXT_STAGE(next_stage)		\
	(EC_EPE_ACT_DATA_5_NEXT_STAGE_MASK &			\
	 ((next_stage) << EC_EPE_ACT_DATA_5_NEXT_STAGE_SHIFT))

#define EC_EPE_ACT_DATA_5_NEXT_BRANCH_ID_SET(next_stage_set)		\
	(EC_EPE_ACT_DATA_5_NEXT_BRANCH_ID_SET_MASK &			\
	 ((next_stage_set) << EC_EPE_ACT_DATA_5_NEXT_BRANCH_ID_SET_SHIFT))

#define EC_EPE_ACT_DATA_5_NEXT_BRANCH_ID_VAL(next_stage_val)		\
	(EC_EPE_ACT_DATA_5_NEXT_BRANCH_ID_VAL_MASK &			\
	 ((next_stage_val) << EC_EPE_ACT_DATA_5_NEXT_BRANCH_ID_VAL_SHIFT))

#define EC_EPE_ACT_DATA_5_NEXT_CTRL_BIT_SET(next_ctrl_bit_set)		\
	(EC_EPE_ACT_DATA_5_NEXT_CTRL_BIT_SET_MASK &			\
	 ((next_ctrl_bit_set) << EC_EPE_ACT_DATA_5_NEXT_CTRL_BIT_SET_SHIFT))

#define EC_EPE_ACT_DATA_5_NEXT_CTRL_BIT_VAL(next_ctrl_bit_val)		\
	(EC_EPE_ACT_DATA_5_NEXT_CTRL_BIT_VAL_MASK &			\
	 ((next_ctrl_bit_val) << EC_EPE_ACT_DATA_5_NEXT_CTRL_BIT_VAL_SHIFT))

#define EC_EPE_ACT_DATA_6_HDR_LEN_WR				(1 << 0)
#define EC_EPE_ACT_DATA_6_HDR_LEN_WR_PTR_MASK		AL_FIELD_MASK(10, 1)
#define EC_EPE_ACT_DATA_6_HDR_LEN_WR_PTR_SHIFT		1
#define EC_EPE_ACT_DATA_6_HDR_OFFSET_WR				(1 << 11)
#define EC_EPE_ACT_DATA_6_HDR_OFFSET_WR_PTR_MASK	AL_FIELD_MASK(21, 12)
#define EC_EPE_ACT_DATA_6_HDR_OFFSET_WR_PTR_SHIFT	12
#define EC_EPE_ACT_DATA_6_NEXT_PARSE_DIS			(1 << 22)

#define EC_EPE_ACT_DATA_6_NEXT_PARSE_EN			(0 << 22)

#define EC_EPE_ACT_DATA_6_HDR_LEN_WR_PTR(ptr)		\
	(EC_EPE_ACT_DATA_6_HDR_LEN_WR_PTR_MASK &	\
	 ((ptr) << EC_EPE_ACT_DATA_6_HDR_LEN_WR_PTR_SHIFT))

#define EC_EPE_ACT_DATA_6_HDR_OFFSET_WR_PTR(ptr)		\
	(EC_EPE_ACT_DATA_6_HDR_OFFSET_WR_PTR_MASK &	\
	 ((ptr) << EC_EPE_ACT_DATA_6_HDR_OFFSET_WR_PTR_SHIFT))

#define EC_EPE_P_COMP_DATA_DATA_2_STAGE_MASK		AL_FIELD_MASK(18, 16)
#define EC_EPE_P_COMP_DATA_DATA_2_STAGE_SHIFT		16
#define EC_EPE_P_COMP_DATA_DATA_2_BRANCH_ID_MASK	AL_FIELD_MASK(24, 19)
#define EC_EPE_P_COMP_DATA_DATA_2_BRANCH_ID_SHIFT	19

#define EC_EPE_P_COMP_MASK_DATA_2_STAGE_MASK		AL_FIELD_MASK(18, 16)
#define EC_EPE_P_COMP_MASK_DATA_2_STAGE_SHIFT		16
#define EC_EPE_P_COMP_MASK_DATA_2_BRANCH_ID_MASK	AL_FIELD_MASK(24, 19)
#define EC_EPE_P_COMP_MASK_DATA_2_BRANCH_ID_SHIFT	19

#endif /* __AL_HAL_ETH_EPE_REGS_H__ */