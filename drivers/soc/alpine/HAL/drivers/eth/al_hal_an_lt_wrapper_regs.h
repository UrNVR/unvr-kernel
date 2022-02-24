/*
 * Copyright 2018, Amazon.com, Inc. or its affiliates. All Rights Reserved
 */

/**
 *  @{
 * @file   al_hal_an_lt_wrapper_regs.h
 *
 * @brief an_lt_wrapper registers
 *
 * This file was auto-generated by RegGen v1.2.1
 *
 */

#ifndef __AL_HAL_AN_LT_WRAPPER_REGS_H__
#define __AL_HAL_AN_LT_WRAPPER_REGS_H__

#include "al_hal_plat_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Unit Registers
 */

/*
 * General AN LT wrapper configurations
 */
struct al_an_lt_wrapper_gen {
	/* [0x0] AN LT wrapper Version */
	uint32_t version;
	/* [0x4] AN LT general configuration */
	uint32_t cfg;
	/* [0x8] */
	uint32_t rsrvd_0[14];
};

/*
 * AN LT per module registers
 */
struct al_an_lt_wrapper_an_lt {
	/* [0x0] AN LT register file address */
	uint32_t addr;
	/* [0x4] PCS register file data */
	uint32_t data;
	/* [0x8] AN LT control signals */
	uint32_t ctrl;
	/* [0xc] AN LT status signals */
	uint32_t status;
	/* [0x10] DME down-sample configuration override (autoneg  25G) */
	uint32_t dme_ds;
	/* [0x14] */
	uint32_t rsrvd_0[3];
};

struct al_an_lt_wrapper_regs {
	/* [0x0] 4 groups of interrupts */
	uint32_t int_ctrl_mem[64];
	/* [0x100] General AN LT wrapper configurations */
	struct al_an_lt_wrapper_gen gen;
	/* [0x140] AN LT per module registers */
	struct al_an_lt_wrapper_an_lt an_lt[3];
	/* [0x1a0] */
	uint32_t rsrvd_0[16280];
};


/*
 * Registers Fields
 */

/**** version register ****/
/*
 * Revision number (Minor)
 * Reset: 0x1         Access: RO
 */
#define AN_LT_WRAPPER_GEN_VERSION_RELEASE_NUM_MINOR_MASK 0x000000FF
#define AN_LT_WRAPPER_GEN_VERSION_RELEASE_NUM_MINOR_SHIFT 0
/*
 * Revision number (Major)
 * Reset: 0x0         Access: RO
 */
#define AN_LT_WRAPPER_GEN_VERSION_RELEASE_NUM_MAJOR_MASK 0x0000FF00
#define AN_LT_WRAPPER_GEN_VERSION_RELEASE_NUM_MAJOR_SHIFT 8
/*
 * Date of release
 * Reset: 0x0         Access: RO
 */
#define AN_LT_WRAPPER_GEN_VERSION_DATE_DAY_MASK 0x001F0000
#define AN_LT_WRAPPER_GEN_VERSION_DATE_DAY_SHIFT 16
/*
 * Month of release
 * Reset: 0x0         Access: RO
 */
#define AN_LT_WRAPPER_GEN_VERSION_DATA_MONTH_MASK 0x01E00000
#define AN_LT_WRAPPER_GEN_VERSION_DATA_MONTH_SHIFT 21
/*
 * Year of release (starting from 2000)
 * Reset: 0x0         Access: RO
 */
#define AN_LT_WRAPPER_GEN_VERSION_DATE_YEAR_MASK 0x3E000000
#define AN_LT_WRAPPER_GEN_VERSION_DATE_YEAR_SHIFT 25
/*
 * Reserved
 * Reset: 0x0         Access: RO
 */
#define AN_LT_WRAPPER_GEN_VERSION_RESERVED_MASK 0xC0000000
#define AN_LT_WRAPPER_GEN_VERSION_RESERVED_SHIFT 30

/**** cfg register ****/
/*
 * selection between different bus widths:
 * 0 - 40bit (with dme_10g)
 * 1 - N/A
 * 2 - 20bit
 * 3 - 40bit (with dme_25g)
 * Reset: 0x1         Access: RW
 */
#define AN_LT_WRAPPER_GEN_CFG_AN_LT_SEL_RX_MASK 0x00000003
#define AN_LT_WRAPPER_GEN_CFG_AN_LT_SEL_RX_SHIFT 0
/*
 * selection between different bus widths:
 * 0 - 40bit (with dme_10g)
 * 1 - N/A
 * 2 - 20bit
 * 3 - 40bit (with dme_25g)
 * Reset: 0x1         Access: RW
 */
#define AN_LT_WRAPPER_GEN_CFG_AN_LT_SEL_TX_MASK 0x0000000C
#define AN_LT_WRAPPER_GEN_CFG_AN_LT_SEL_TX_SHIFT 2
/*
 * bypass the AN/LT block
 * Reset: 0x1         Access: RW
 */
#define AN_LT_WRAPPER_GEN_CFG_BYPASS_RX  (1 << 4)
/*
 * bypass the AN/LT block
 * Reset: 0x1         Access: RW
 */
#define AN_LT_WRAPPER_GEN_CFG_BYPASS_TX  (1 << 5)

/**** addr register ****/
/*
 * Address value
 * Reset: 0x0         Access: RW
 */
#define AN_LT_WRAPPER_AN_LT_ADDR_VAL_MASK 0x000007FF
#define AN_LT_WRAPPER_AN_LT_ADDR_VAL_SHIFT 0

/**** data register ****/
/*
 * Data value
 * Reset: 0x0         Access: RW
 */
#define AN_LT_WRAPPER_AN_LT_DATA_VAL_MASK 0x0000FFFF
#define AN_LT_WRAPPER_AN_LT_DATA_VAL_SHIFT 0

/**** ctrl register ****/
/*
 * Default Auto-Negotiation Enable. If ‘1’, the auto-negotiation process will start after reset
 * de-assertion. The application can also start the auto-negotiation process by writing the
 * KXAN_CONTROL.an_enable bit with ‘1’.
 * Important: This signal is OR'ed with the KXAN_CONTROL.an_enable bit. Hence, when asserted (1) the
 * application is unable to disable autonegotiation and writing the an_enable bit has no effect.
 * Note: Even if enabled by this pin, the application must write the correct abilities in the
 * KXAN_ABILITY_1/2/3 registers within 60ms from reset deassertion (break_link_timer).
 * Reset: 0x0         Access: RW
 */
#define AN_LT_WRAPPER_AN_LT_CTRL_AN_ENA  (1 << 0)
/*
 * If set to 1, the Arbitration State Machine reached the TRANSMIT_DISABLE state.
 * Reset: 0x0         Access: RW
 */
#define AN_LT_WRAPPER_AN_LT_CTRL_AN_DIS_TIMER (1 << 1)
/* Reset: 0x0         Access: RW */
#define AN_LT_WRAPPER_AN_LT_CTRL_LINK_STATUS_KX (1 << 4)
/* Reset: 0x0         Access: RW */
#define AN_LT_WRAPPER_AN_LT_CTRL_LINK_STATUS_KX4 (1 << 5)
/* Reset: 0x0         Access: RW */
#define AN_LT_WRAPPER_AN_LT_CTRL_LINK_STATUS (1 << 6)
/*
 * PHY LOS indication selection
 * 0 - Select input from the SerDes
 * 1 - Select register value from phy_los_in_def
 * Reset: 0x0         Access: RW
 */
#define AN_LT_WRAPPER_AN_LT_CTRL_PHY_LOS_IN_SEL (1 << 8)
/*
 * PHY LOS default value
 * Reset: 0x0         Access: RW
 */
#define AN_LT_WRAPPER_AN_LT_CTRL_PHY_LOS_IN_DEF (1 << 9)
/*
 * PHY LOS polarity
 * Reset: 0x0         Access: RW
 */
#define AN_LT_WRAPPER_AN_LT_CTRL_PHY_LOS_IN_POL (1 << 10)
/*
 * PHY LOS indication selection
 * 0 - select AN output
 * 1 - Select register value from phy_los_out_def
 * 2 - Select input from the SerDes
 * 3 - 0
 * Reset: 0x0         Access: RW
 */
#define AN_LT_WRAPPER_AN_LT_CTRL_PHY_LOS_OUT_SEL_MASK 0x00003000
#define AN_LT_WRAPPER_AN_LT_CTRL_PHY_LOS_OUT_SEL_SHIFT 12
/*
 * PHY LOS default value
 * Reset: 0x0         Access: RW
 */
#define AN_LT_WRAPPER_AN_LT_CTRL_PHY_LOS_OUT_DEF (1 << 14)
/*
 * PHY LOS polarity
 * Reset: 0x0         Access: RW
 */
#define AN_LT_WRAPPER_AN_LT_CTRL_PHY_LOS_OUT_POL (1 << 15)
/*
 * AN VAL indication selection
 * 0 - select AN output
 * 1 - Select register value from an_val_out_def
 * 2 - 0
 * 3 - N/A
 * Reset: 0x0         Access: RW
 */
#define AN_LT_WRAPPER_AN_LT_CTRL_AN_VAL_SEL_MASK 0x00030000
#define AN_LT_WRAPPER_AN_LT_CTRL_AN_VAL_SEL_SHIFT 16
/*
 * AN VAL default value
 * Reset: 0x0         Access: RW
 */
#define AN_LT_WRAPPER_AN_LT_CTRL_AN_VAL_DEF (1 << 18)
/*
 * AN VAL polarity
 * Reset: 0x0         Access: RW
 */
#define AN_LT_WRAPPER_AN_LT_CTRL_AN_VAL_POL (1 << 19)

/**** status register ****/
/*
 * Auto-Negotiation Done. If ‘1’, the auto-negotiation process has completed.
 * Reset: 0x0         Access: RO
 */
#define AN_LT_WRAPPER_AN_LT_STATUS_AN_DONE (1 << 0)
/*
 * If set to 1, auto-negotiation is enabled on the link. It represents the enable control bit
 * KXAN_CONTROL.an_enable. When set to 1, the signals an_status/an_select are valid.
 * Reset: 0x0         Access: RO
 */
#define AN_LT_WRAPPER_AN_LT_STATUS_AN_VAL (1 << 1)
/*
 * If set to 0, auto-negotiation is in progress, if set to 1, the Arbitration State Machine reached
 * the AN_GOOD_CHECK state (i.e. before autonegotiation is done, but the link no longer is used to
 * transfer DME pages). Stays asserted also during AN_GOOD (autoneg done).
 * Reset: 0x0         Access: RO
 */
#define AN_LT_WRAPPER_AN_LT_STATUS_AN_STATUS (1 << 2)
/*
 * Selected Technology. Becomes valid when an_status is 1.
 * The selection mode number (from 0 to 24) corresponds to the Technology Ability (A0-A24) from the
 * ability pages (see 4.3.2.3 page 13). The mode selection is based on the matching technology
 * abilities and priority.
 * A value of 31 is an invalid setting that indicates that no common technology could be resolved.
 * The application should then inspect the base page results to determine if the link is operable or
 * not.
 * Reset: 0x0         Access: RO
 */
#define AN_LT_WRAPPER_AN_LT_STATUS_AN_SELECT_MASK 0x000001F0
#define AN_LT_WRAPPER_AN_LT_STATUS_AN_SELECT_SHIFT 4
/*
 * If set to 1, the Arbitration State Machine reached the TRANSMIT_DISABLE state
 * Reset: 0x0         Access: RO
 */
#define AN_LT_WRAPPER_AN_LT_STATUS_AN_TR_DIS_STATUS (1 << 16)
/*
 * FEC Enable. Asserts when autonegotiation base page exchange identified both link partners
 * advertising FEC capability and at least one is requesting FEC.
 * The signal stays constant following base page exchange until autonegotiation is disabled or
 * restarted.
 * Note: the information can also be extracted from the base page exchange or the BP_ETH_STATUS
 * register.
 * Reset: 0x0         Access: RO
 */
#define AN_LT_WRAPPER_AN_LT_STATUS_FEC_ENA (1 << 17)
/*
 * Link Training Frame Lock. If set to 1 the training frame delineation has been acquired.
 * Reset: 0x0         Access: RO
 */
#define AN_LT_WRAPPER_AN_LT_STATUS_LT_LOCK (1 << 20)
/*
 * If set to 0, link-training is in progress, if set to 1, the training is completed and the PCS
 * datapath has been enabled (phy_los_out no longer gated).
 * Reset: 0x0         Access: RO
 */
#define AN_LT_WRAPPER_AN_LT_STATUS_LT_STATUS (1 << 21)
/*
 * If set to 1, link-training is enabled on the link. It represents the enable control bit PMD
 * Control.taining enable. When set to 1, the signal lt_status is valid
 * Reset: 0x0         Access: RO
 */
#define AN_LT_WRAPPER_AN_LT_STATUS_LT_VAL (1 << 22)

/**** dme_ds register ****/
/*
 * [7:0]   - max current sample point
 * [14:8] - mask find first vec
 * [15]    - enable override
 * Reset: 0x7028      Access: RW
 */
#define AN_LT_WRAPPER_AN_LT_DME_DS_CONF_MASK 0x0000FFFF
#define AN_LT_WRAPPER_AN_LT_DME_DS_CONF_SHIFT 0
/* Reset: 0x0         Access: RW */
#define AN_LT_WRAPPER_AN_LT_DME_DS_RESERVED_31_16_MASK 0xFFFF0000
#define AN_LT_WRAPPER_AN_LT_DME_DS_RESERVED_31_16_SHIFT 16

#ifdef __cplusplus
}
#endif

#endif

/** @} */
