/*
 * bcm59111.c BRCM59111 PSE driver
 *
 */

#include <error.h>
#include <linux/i2c.h>
#include <socket.h>
#include <linux/netlink.h>
#include <types.h>
#include <unistd.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include "al_hal_common.h"
#include "bcm59111.h"
#include "BCM59111_POETEC_VER_21_RELEASE.h"
#include "BCM59121_POETEC_VER_15_RELEASE.h"

#define BCM59111_LED_MAX	4

struct bcm59111_chip {
        struct i2c_client *cl;
        struct mutex lock;      /* lock for user-space interface */
        u8 mode;
        int en_gpio;
        int irq_gpio;
        bool is2chs; /* 2 channels on one port */
        struct gpio_desc *led_gpios[BCM59111_LED_MAX];
        const struct firmware *fw;
};

#define I2C_POE_SADDR1   0x29

#ifdef ARRAY_SIZE
#undef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif

#define BCM59111_P1_CONNECT        0x40
#define BCM59111_P2_CONNECT     0x10
#define BCM59111_P3_CONNECT     0x04
#define BCM59111_P4_CONNECT     0x01

#define BCM59111_P1_DISCONNECT     0x3f
#define BCM59111_P2_DISCONNECT     0xcf
#define BCM59111_P3_DISCONNECT     0xf3
#define BCM59111_P4_DISCONNECT     0xfc

#define BCM59111_RST_IOCTL_CMD      0x0
#define BCM59111_INIT_IOCTL_CMD   0x1
#define BCM59111_CFG_IOCTL_CMD    0x2
#define BCM59111_CRC_IOCTL_CMD    0x3
#define BCM59111_LED_INIT_IOCTL_CMD    0x4
#define BCM59111_PROBE_IOCTL_CMD    0x5

#define BCM59111_POE0_MUX_MASK         (0x1 << 4)
#define BCM59111_POE1_MUX_MASK         (0x1 << 5)
#define BCM59111_RST_GPIO         9

#define CHIPCOMMONA_GPIOOUTEN 0x18000068
#define CHIPCOMMONA_GPIOOUT   0x18000064

#define DEVICE_NUM 	1
#define I2C_POE_SADDR0  0x30
#define SOC_E_NONE 0
enum {
    BCM59111_EVT_P1 = 0x1,
    BCM59111_EVT_P2 = 0x2,
    BCM59111_EVT_P3 = 0x4,
    BCM59111_EVT_P4 = 0x8,
};

//extern int bcm59111_netlink_init(int unit);


int soc_i2c_read_byte_data(int unit, uint8_t saddr, uint16_t addr, uint8_t *data)
{
    return 0;
}

static void
bcm59111_firmware_load(struct i2c_client *client, const uint8_t *firmware, int size);
static void
bcm59111_post_init(struct i2c_client *client);
static int
bcm59111_detect(struct i2c_client *client, struct i2c_board_info *info);

static int bcm59111_firmware_crc(struct i2c_client *client);

void bcm59111_board_alert(void)
{
    /* Not Yet Implement */
}

void (* bcm59111_board_handler[])(void) = {
    bcm59111_board_alert,
};

static const struct reg_init pse_init_cmds[] = {
    {
        .reg = BCM59111_INT_MASK_REG, //0x01
        .val = BCM59111_INT_F_PWR_GOOD |
            BCM59111_INT_F_DIS,
    },
    {
        .reg = BCM59111_OP_MOD_REG,  //0x12
        .val = 0xff,
    },
    {
        .reg = BCM59111_DIS_SEN_EN_REG, //0x13
        .val = 0xf,
    },
    {
        .reg = BCM59111_DET_CLASS_EN_REG, //0x14
        .val = 0xff,
    },

    /* enable global high power feature */
    {
        .reg = BCM59111_HP_EN_REG, //0x44
        .val = 0x00,
    },

    /* Port 1 high power feature */
    {
        .reg = BCM59111_HP_P1_MOD_REG, //0x46
        .val = 0x01,
    },
    {
        .reg = BCM59111_HP_P1_OV_CUT_REG, //0x47
        .val = 0x50,
    },
    {
        .reg = BCM59111_HP_P1_CL_FBC_REG, //0x48
        .val = 0x0,
    },

    /* Port 2 high power feature */
    {
        .reg = BCM59111_HP_P2_MOD_REG,
        .val = 0x01,
    },
    {
        .reg = BCM59111_HP_P2_OV_CUT_REG,
        .val = 0x50,
    },
    {
        .reg = BCM59111_HP_P2_CL_FBC_REG,
        .val = 0x0,
    },

    /* Port 3 high power feature */
    {
        .reg = BCM59111_HP_P3_MOD_REG,
        .val = 0x01,
    },
    {
        .reg = BCM59111_HP_P3_OV_CUT_REG,
        .val = 0x50,
    },

    {
        .reg = BCM59111_HP_P3_CL_FBC_REG,
        .val = 0x0,
    },

    /* Port 4 high power feature */
    {
        .reg = BCM59111_HP_P4_MOD_REG,
        .val = 0x01,
    },
    {
        .reg = BCM59111_HP_P4_OV_CUT_REG,
        .val = 0x50,
    },
    {
        .reg = BCM59111_HP_P4_CL_FBC_REG,
        .val = 0x0,
    },

};

static void
bcm59111_post_init(struct i2c_client *client)
{
    int i, ret;
    char data[1];
    struct bcm59111_chip *chip_data = i2c_get_clientdata(client);

    for (i = 0; i < ARRAY_SIZE(pse_init_cmds); i++) {
	data[0] = pse_init_cmds[i].val;
	ret = i2c_smbus_write_i2c_block_data(client, pse_init_cmds[i].reg, 1, data);
        dev_dbg(&client->dev, "%s post_init_cmd[%d] reg 0x%x data 0x%x ret %d\n", __func__, i, pse_init_cmds[i].reg, data[0], ret);

    }
    if(chip_data->is2chs) {
        data[0] = 0xcc;
        i2c_smbus_write_i2c_block_data(client, BCM59111_OP_MOD_REG, 1, data);
    }
}

void
bcm59111_firmware_load(struct i2c_client *client, const uint8_t *firmware, int size)
{
    int i;
    int ret = 0;
    uint16_t pos = 0, tx_size = (SEGMENT_SIZE + BCM59111_HEADER_OFFSET);
    char segment[SEGMENT_SIZE + BCM59111_HEADER_OFFSET] = {0};
    char crc[1];
    /*  The follwing is firmware downlod format
    *  START <addr> 0x70 0x80 <img_off> <img_data> STOP
    */
    segment[0] = BCM59111_FW_PROGRAM;

    while (size) {
        segment[1] = (pos >> 8) & 0xff;
        segment[2] = ((pos & 0xff));

        for (i = 0; i < SEGMENT_SIZE; i++)
            segment[i+BCM59111_HEADER_OFFSET] = firmware[pos+i];

        ret = i2c_smbus_write_i2c_block_data(client, BCM59111_FW_DWN_CTRL, tx_size, segment);
	if(ret != 0)
	        pr_err("%s size %d, ret %d\n", __func__, size, ret);
        size = size - SEGMENT_SIZE;
        pos = pos + SEGMENT_SIZE;
    }

    crc[0] = BCM59111_FW_FORCE_CRC;
    ret = i2c_smbus_write_i2c_block_data(client, BCM59111_FW_DWN_CTRL, 1, crc);
    if(ret != 0)
        dev_err( &client->dev, "FW_DWN_CTRL crc ret %d\n", ret);

}

static int
bcm59111_firmware_crc(struct i2c_client *client)
{
    int i, firmware_status = 0;

    /* switch to poe board specific bus */
    for (i = 0 ; i < DEVICE_NUM; i++){
        uint8_t crc_stat = 0xff;
	crc_stat = i2c_smbus_read_byte_data(client, BCM59111_CRC_STATUS);
        dev_dbg(&client->dev, "%s crc_stat %x\n", __func__, crc_stat);

        if (crc_stat == 0xaa) {
            firmware_status |= (0x1 << i);
        }
    }
    return firmware_status;
}

static int
bcm59111_chips_detect(struct i2c_client *client)
{
    int i, probe_status = 0x0, dev_id = 0x0;
    struct bcm59111_chip *chip_data = i2c_get_clientdata(client);

    for (i = 0 ; i < DEVICE_NUM; i++){
        dev_id = i2c_smbus_read_byte_data(client, BCM59111_DEV_ID_REG);
        chip_data->mode = dev_id;
        dev_dbg(&client->dev, "%s probe dev id %x\n", __func__, dev_id);

        if (dev_id != BCM59111_DEV_ID_REV && dev_id != BCM59121_DEV_ID_REV) {
            probe_status |= (0x1 << i);
        }
    }

    return probe_status;
}


static ssize_t firmware_enabled_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{

	struct i2c_client *client = to_i2c_client(dev);
	struct bcm59111_chip *chip_data = i2c_get_clientdata(client);
	u8 fw_ver;

	fw_ver = i2c_smbus_read_byte_data(client, BCM59111_FW_REV_REG);
	dev_dbg(&client->dev, "%s read fw rev %x\n", __func__, fw_ver);

	if(chip_data && chip_data->mode == BCM59111_DEV_ID_REV && fw_ver != BCM59111_POETEC_VER) {
		bcm59111_firmware_load(client, Kc_BCM59111_POETEC_VER_21_RELEASE, SZ_BCM59111_POETEC_VER_21_RELEASE);
	}
	else if(chip_data && chip_data->mode == BCM59121_DEV_ID_REV && fw_ver != BCM59121_POETEC_VER) {
		bcm59111_firmware_load(client, Kc_BCM59121_POETEC_VER_15_RELEASE, SZ_BCM59121_POETEC_VER_15_RELEASE);
	}

	return size;
}

static ssize_t
firmware_enabled_show(struct device *dev, struct device_attribute *attr,
                        char *buf)
{
        return sprintf(buf, "0x%x\n", bcm59111_firmware_crc(to_i2c_client(dev)));
}


static ssize_t
post_init_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{

        struct i2c_client *client = to_i2c_client(dev);

        if(bcm59111_firmware_crc(to_i2c_client(dev))) {
                bcm59111_post_init(client);
        }

        return size;
}

#define BCM59111_CURR_LSB 0
#define BCM59111_CURR_MSB 1
#define BCM59111_VOLT_LSB 2
#define BCM59111_VOLT_MSB 3
#define D14 (100000000000000UL)
#define D11 (100000000000UL)
#define POWER_UNIT (unsigned long)(12207*5835)

#define BCM59111_POWER_ADDR_BASE 0x30
#define BCM59111_POWER_CLASS_BASE 0x0C
#define BCM59111_PORT_OFFSET 4
#define IS_AF(class) \
	((class > 0 && class <= 3) || class == 6)

#define BCM59111_PORT_STATUS_GOOD 4

static ssize_t
report_show(struct device *dev, struct device_attribute *attr,
                        char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int i;
	for(i=0; i<4; i++) {
		unsigned short volt, curr, class, detect;
		unsigned long ints1=0, ints2=0, power;
		curr = i2c_smbus_read_byte_data(client, BCM59111_POWER_ADDR_BASE + BCM59111_CURR_MSB + BCM59111_PORT_OFFSET*i);
		curr = (curr << 8) + i2c_smbus_read_byte_data(client, BCM59111_POWER_ADDR_BASE + BCM59111_CURR_LSB + BCM59111_PORT_OFFSET*i);


		volt = i2c_smbus_read_byte_data(client, BCM59111_POWER_ADDR_BASE + BCM59111_VOLT_MSB + BCM59111_PORT_OFFSET*i);
		volt = (volt << 8) + i2c_smbus_read_byte_data(client, BCM59111_POWER_ADDR_BASE + BCM59111_VOLT_LSB + BCM59111_PORT_OFFSET*i);

		class = i2c_smbus_read_byte_data(client, BCM59111_POWER_CLASS_BASE + i) >> 4;
		detect = i2c_smbus_read_byte_data(client, BCM59111_POWER_CLASS_BASE + i) & 0x0F;
		if(detect != BCM59111_PORT_STATUS_GOOD) {
			class = 0;
		}

		power = curr*volt*POWER_UNIT;
		ints1 = power/D14;
		ints2 = (power - ints1*D14)/D11;

		dev_dbg(&client->dev, "\tP%d: %lu.%03lu W\n", i+1, ints1, ints2);

		sprintf(buf + strlen(buf), "\tP%d: %lu.%03lu W %s\n", i+1, ints1, ints2, class == 4 ? "AT":(IS_AF(class) ? "AF":"N/A"));
	}
        return strlen(buf);
}


static DEVICE_ATTR_RW(firmware_enabled);
static DEVICE_ATTR_WO(post_init);
static DEVICE_ATTR_RO(report);


/*
 * ATTRIBUTES:
 *
 */

static struct attribute *bcm59111_attrs[] = {
        &dev_attr_firmware_enabled.attr,
        &dev_attr_post_init.attr,
        &dev_attr_report.attr,
        NULL,
};

static struct attribute_group bcm59111_attr_group = {
        .attrs = bcm59111_attrs,
};

static irqreturn_t bcm59111_irq(int irq, void *clnt)
{
        struct i2c_client *client = (struct i2c_client *)clnt;
        struct bcm59111_chip *chip_data = i2c_get_clientdata(client);
        uint8_t pw_ev = 0, fault_ev = 0, i=0;
        uint8_t pw_stat = i2c_smbus_read_byte_data(client, BCM59111_SAT_PWR_REG);

        pw_ev = i2c_smbus_read_byte_data(client, BCM59111_PWR_EVT_CLR_REG);
        fault_ev = i2c_smbus_read_byte_data(client, BCM59111_FAULT_EVT_CLR_REG);
        dev_info(&client->dev, "%s irq stats pwr ev 0x%x, fault 0x%x pwr stat 0x%x\n", __func__, pw_ev, fault_ev, pw_stat);

        for(i = 0; i < (chip_data->is2chs ? (BCM59111_LED_MAX/2): BCM59111_LED_MAX); i++) {
            if(chip_data->led_gpios[i] && BCM59111_DEV_GET_PORT_PWR_STATUS( chip_data->is2chs ? (i*2 + 1) : i, pw_stat)) {
                gpiod_set_value_cansleep(chip_data->led_gpios[i], 1);
            } else {
                gpiod_set_value_cansleep(chip_data->led_gpios[i], 0);
            }
        }
        return IRQ_HANDLED;

}
static int bcm59111_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
	int status = 0, ret = 0, i;
	struct bcm59111_chip *chip_data;
	struct device_node *np = client->dev.of_node;
	int irq = 0;

        status = sysfs_create_group(&client->dev.kobj, &bcm59111_attr_group);

        if (status) {
                dev_err(&client->dev, "Unable to export , error: %d\n",
                        status);
        }

        chip_data = devm_kzalloc(&client->dev, sizeof(struct bcm59111_chip),
                                GFP_KERNEL);
        if (!chip_data)
                return -ENOMEM;

        i2c_set_clientdata(client, chip_data);
        chip_data->cl = client;

	if(!of_property_read_u32(np, "en-gpio", &chip_data->en_gpio)) {
		dev_info(&client->dev, "en-gpio%d\n",
			chip_data->en_gpio);

		if (gpio_is_valid(chip_data->en_gpio)) {
			ret = devm_gpio_request(&client->dev, chip_data->en_gpio,
					dev_name(&client->dev));
			if (ret) {
				dev_err(&client->dev, "unable to req gpio%d: %d\n",
					chip_data->en_gpio, ret);
				goto err_probe;
			}
			ret = gpio_direction_output(chip_data->en_gpio, 1);
			if (ret)
				dev_err(&client->dev, "unable to enable gpio%d: %d\n",
					chip_data->en_gpio, ret);
			devm_gpio_free(&client->dev, chip_data->en_gpio);
		}
	}

	chip_data->is2chs = of_property_read_bool(np, "2-channels-per-port");
	if(!of_property_read_u32(np, "irq-gpio", &chip_data->irq_gpio)) {
		dev_info(&client->dev, "irq-gpio%d\n",
			chip_data->irq_gpio);

		if (gpio_is_valid(chip_data->irq_gpio)) {
			irq = gpio_to_irq(chip_data->irq_gpio);
		}
	}

	/* Make use of INT pin only if valid IRQ no. is given */
	if (irq > 0) {
		ret = request_threaded_irq(irq, NULL, bcm59111_irq,
				IRQF_TRIGGER_FALLING | IRQF_SHARED | IRQF_ONESHOT,
				"bcm59111", client);
		if (ret) {
			dev_err(&client->dev, "failed to request IRQ #%d\n", irq);
			return ret;
		}
	}

	/* Request GPIOs. */
	for (i = 0; i < (chip_data->is2chs ? (BCM59111_LED_MAX/2): BCM59111_LED_MAX); i++) {
		chip_data->led_gpios[i] =
			devm_gpiod_get_index_optional(&client->dev, "led", i,
                                                      GPIOD_OUT_LOW);
		if (IS_ERR(chip_data->led_gpios[i]))
			continue;

		if (chip_data->led_gpios[i])
			dev_info(&client->dev, "Handling LED %u GPIO acquired.\n", i);
	}

	status = bcm59111_detect(client, NULL);
	return status;
err_probe:
        return ret;

}

static int bcm59111_remove(struct i2c_client *client)
{
	struct bcm59111_chip *chip_data = i2c_get_clientdata(client);

	kfree(chip_data);
	sysfs_remove_group(&client->dev.kobj, &bcm59111_attr_group);
	return 0;
}

/* Return 0 if detection is successful, -ENODEV otherwise */
static int bcm59111_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	int status = 0;
        status = bcm59111_chips_detect(client);
        dev_dbg(&client->dev, "%s status = %d", __func__, status);
	return status ? -ENODEV:status;
}
static const struct i2c_device_id bcm59111_id[] = {
        { "bcm59111", 0 },
        { }
};
MODULE_DEVICE_TABLE(i2c, bcm59111_id);


#ifdef CONFIG_OF
static const struct of_device_id of_bcm59111_pse_match[] = {
        { .compatible = "bcm59111", },
        { .compatible = "bcm59121", },
        {},
};

static struct i2c_driver bcm59111_driver = {
        .driver = {
                .name   = "bcm59111",
                .of_match_table = of_match_ptr(of_bcm59111_pse_match),
        },
        .probe          = bcm59111_probe,
        .detect         = bcm59111_detect,
        .remove         = bcm59111_remove,
        .id_table       = bcm59111_id,
};

#endif

static int __init bcm59111_chip_init(void)
{
        return i2c_add_driver(&bcm59111_driver);
}

module_init(bcm59111_chip_init);

static void __exit bcm59111_chip_exit(void)
{
        i2c_del_driver(&bcm59111_driver);
}
module_exit(bcm59111_chip_exit);

MODULE_DESCRIPTION("BCM59111 PSE driver");
MODULE_LICENSE("GPL v2");

