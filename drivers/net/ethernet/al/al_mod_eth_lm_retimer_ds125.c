/*
 * Copyright 2019, Ui.com, Inc. or its affiliates. All Rights Reserved
 */

#include "al_mod_eth_lm_retimer.h"
#include "al_mod_eth_lm_retimer_internal.h"

/*******************************************************************************
 ** Retimer Registers
 ******************************************************************************/
#define LM_DS25_CHANNEL_EN_REG		0xff
#define LM_DS25_CHANNEL_EN_MASK		0x03
#define LM_DS25_CHANNEL_EN_VAL		0x01

#define LM_DS25_CHANNEL_SEL_REG		0xfc
#define LM_DS25_CHANNEL_SEL_MASK	0xff

#define LM_DS25_CDR_RESET_REG		0x0a
#define LM_DS25_CDR_RESET_MASK		0x0c
#define LM_DS25_CDR_RESET_ASSERT	0x0c
#define LM_DS25_CDR_RESET_RELEASE	0x00

#define LM_DS25_SIGNAL_DETECT_REG	0x78
#define LM_DS25_SIGNAL_DETECT_MASK	0x20

#define LM_DS25_CDR_LOCK_REG		0x78
#define LM_DS25_CDR_LOCK_MASK		0x10

#define LM_DS25_DRV_PD_REG		0x15
#define LM_DS25_DRV_PD_MASK		0x08

#define LM_DS25_CDR_STATUS_CTRL_REG	0xc
#define LM_DS25_CDR_STATUS_CTRL_DIV_MASK 0xff
#define LM_DS25_CDR_STATUS_CTRL_DIV_VAL	0x30
#define LM_DS25_CDR_STATUS_REG		0x2
#define LM_DS25_CDR_STATUS_DIV_10G	0x9
#define LM_DS25_CDR_STATUS_RX_EQ	0x40

/*******************************************************************************
 ** Registers configuration sequences
 ******************************************************************************/
struct retimer_config_reg {
	uint8_t addr;
	uint8_t value;
	uint8_t mask;
};

/** 10G */
static struct retimer_config_reg retimer_ds125_10g_mode[] = {
    /* Enable Broadcast. All writes target all channel register sets */
    {.addr = 0xFF, .value = 0x0C, .mask = 0xFF},
    /* Reset Channel Registers */
    {.addr = 0x00, .value = 0x04, .mask = 0xFF},
    /* Enable override divider select and Enable Override Output Mux */
    {.addr = 0x09, .value = 0x24, .mask = 0xFF},
    /* Channel B idle control */
    {.addr = 0x15, .value = 0x12, .mask = 0xFF},
    /* Select VCO Divider to full rate (000) */
    {.addr = 0x18, .value = 0x00, .mask = 0xFF},
    /* Selects active PFD MUX Input as Re-timed Data (001) */
    {.addr = 0x1E, .value = 0x21, .mask = 0xFF},
    /* VOD control for channel B */
    {.addr = 0x2D, .value = 0x81, .mask = 0xFF},
    /* Set data rate as 10.3125 Gbps */
    {.addr = 0x60, .value = 0x00, .mask = 0xFF},
    {.addr = 0x61, .value = 0xB2, .mask = 0xFF},
    {.addr = 0x62, .value = 0x90, .mask = 0xFF},
    {.addr = 0x63, .value = 0xB3, .mask = 0xFF},
    {.addr = 0x64, .value = 0xCD, .mask = 0xFF},
};


/*******************************************************************************
 ** Helper Functions
 ******************************************************************************/

static int al_mod_eth_lm_retimer_ds125_write_reg(struct al_mod_eth_lm_retimer	*handle,
					    uint8_t			reg_addr,
					    uint8_t			reg_mask,
					    uint8_t			reg_value)
{
	uint8_t reg = 0;
	int rc;

	al_mod_assert((reg_mask & reg_value) == reg_value);

	if (reg_mask != 0xff) {
		rc = handle->i2c_read(handle->i2c_context,
					handle->i2c_master,
					handle->i2c_bus,
					handle->i2c_addr,
					reg_addr,
					&reg);

		if (rc)
			return -EIO;

		reg &= ~(reg_mask);
	}

	reg |= reg_value;

	rc = handle->i2c_write(handle->i2c_context,
				   handle->i2c_master,
				   handle->i2c_bus,
				   handle->i2c_addr,
				   reg_addr,
				   reg);

	if (rc)
		return -EIO;

	return 0;
}

static int al_mod_eth_lm_retimer_ds125_channel_config(struct al_mod_eth_lm_retimer	*handle,
						 uint8_t			channel,
						 struct retimer_config_reg	*config,
						 uint8_t			config_size)
{
	uint8_t i;
	int rc = 0;

	for (i = 0; i < config_size; i++) {
		rc = al_mod_eth_lm_retimer_ds125_write_reg(handle,
						      config[i].addr,
						      config[i].mask,
						      config[i].value);

		if (rc)
			goto config_done;
	}

config_done:
	retimer_debug("%s: retimer channel config done for channel %d\n", __func__, channel);

	return rc;
}

/*******************************************************************************
 ** Callbacks
 ******************************************************************************/

static int al_mod_eth_lm_retimer_ds125_config(struct al_mod_eth_lm_retimer *handle,
	unsigned int channel,
	struct al_mod_eth_lm_retimer_config_params *params)
{
	int rc = 0;
	struct retimer_config_reg *config_regs = NULL;
	uint32_t config_regs_size;

	config_regs = retimer_ds125_10g_mode;
	config_regs_size = AL_ARR_SIZE(retimer_ds125_10g_mode);

	rc = al_mod_eth_lm_retimer_ds125_channel_config(handle,
					channel,
					config_regs,
					config_regs_size);

	return rc;
}


/*******************************************************************************
 ** Internal API Functions
 ******************************************************************************/
int al_mod_eth_lm_retimer_ds125_handle_init(
	struct al_mod_eth_lm_retimer *handle,
	const struct al_mod_eth_lm_retimer_params *params)
{
	al_mod_assert(handle);
	al_mod_assert(params);

	handle->config = al_mod_eth_lm_retimer_ds125_config;
	handle->reset = NULL;
	handle->signal_detect = NULL;
	handle->cdr_lock = NULL;
	handle->rx_eq_done = NULL;
	handle->speed_detect = NULL;

	return 0;
}
