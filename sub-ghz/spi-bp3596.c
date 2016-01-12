/*
 * BP3596 SPI Driver
 * 
 * File:  spi-bp3596.c
 * 
 * Copyright 2015 Lapis Semiconductor Co.,Ltd.
 * Author: Naotaka Saito
 * 
 * The program is based on BP3596 driver by Communication Technology
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include "spi-bp3596.h"

#include "phy-bp3596.h"
#include "common_802154e.h"


#define DRVNAME "bp3596"
#define SPI_ADR_BANK_SEL 0x00  /* BANKÀÚ¤ê´¹¤¨ */
#define SPI_RESET_WAIT 3  /* ¥ê¥»¥Ã¥È¥Ñ¥ë¥¹Éý[usec] */
#define SPI_BUFFERSIZE 256
#define SINTN_HIGH !0
#define SINTN_LOW 0

#define ISERROR(func) ((func) < 0)  /* 戻り値のエラー判定 */
#define ONERRORGOTO(func) if (ISERROR(status = (func))) goto error

static uint8_t local_bank = 0;

// call back PHY when spi_probe
int (*callback_func)(void);

//	SPI API
struct bp_spi_dev {
	struct spi_device *spi;
	char name[SPI_NAME_SIZE];
	u16 irq;
	struct mutex		lock;
};

struct bp_spi_dev *m_bp_spi;

// SPI probe
static int bp_spi_probe(struct spi_device *spi)
{
	int err;
	struct bp_spi_dev *bp_spi;

//	finish_flag = 0;
	DEBUGONDISPLAY(MODE_PHY_DEBUG,printk("[SPI] probe\n"));

	// initializing SPI
	bp_spi = kzalloc(sizeof(*bp_spi), GFP_KERNEL);
	if (!bp_spi) {
	        return -ENOMEM;
	}

	spi->bits_per_word = 8;
	spi->max_speed_hz = 5000000;
	err = spi_setup(spi);
	if (err < 0) {
		printk(KERN_ERR"spi_setup error %d\n", err);
		goto error_spi_setup;
	}

	mutex_init(&bp_spi->lock);

	bp_spi->spi = spi;
	strcpy(bp_spi->name, spi->modalias);
	bp_spi->irq = (u16)spi->irq;

	m_bp_spi = bp_spi;
	printk(KERN_INFO "bp_spi_probe name[%s]]\n", bp_spi->name);

	spi_set_drvdata(spi, bp_spi);

	DEBUGONDISPLAY(MODE_PHY_DEBUG,printk("[SPI] set drvdata\n"));


	// callback PHY
	err = callback_func();
	if(err != 0)
	{
		printk(KERN_INFO "[SPI] MAC callback func error\n");
		goto error_mac_callback;
	}
	DEBUGONDISPLAY(MODE_PHY_DEBUG,printk("[SPI] probe done\n"));
	return 0;

error_mac_callback:
	spi_set_drvdata(bp_spi->spi, NULL);
error_spi_setup:
	kfree(bp_spi);
	return err;
}

static int bp_spi_remove(struct spi_device *dev)
{
	struct bp_spi_dev *bp_spi = spi_get_drvdata(dev);

//	finish_flag = 1;
	// irq disable
	disable_irq(gpio_to_irq(GPIO_SINTN));
	free_irq(gpio_to_irq(GPIO_SINTN), NULL);

	spi_set_drvdata(bp_spi->spi, NULL);
	if (!bp_spi)
		return 0;
	kfree(bp_spi);
	
	/* gpio uninit */
	//gpio_free(GPIO_SINTN);

	printk(KERN_INFO "bp_spi_remove\n");
	return 0;
}


static struct spi_driver bp_spi_driver = {
	.driver = {
		.name   = "bp3596_spi",
		.owner  = THIS_MODULE,
	},
	.probe          = bp_spi_probe,
	.remove         = bp_spi_remove,
};

int bp_spi_add_driver(int (*callback)(void))
{
	int status = 0;
	callback_func = callback;
	status = spi_register_driver(&bp_spi_driver);
	if(status == SPI_OK)
	{	
		DEBUGONDISPLAY(MODE_PHY_DEBUG,printk("[SPI] init success\n"));
	}
	else
	{
		printk("[SPI] init fail\n");
	}

	return status;
}


int bp_spi_del_driver(void)
{
	int status = 0;
	spi_unregister_driver(&bp_spi_driver);
	DEBUGONDISPLAY(MODE_PHY_DEBUG,printk("[SPI] delete driver\n"));

	return status;
}

static int spi_write_law(uint8_t address, const uint8_t *data, uint8_t size) {
    int status = SPI_ERR_UNKNOWN;
    static uint8_t tx[SPI_BUFFERSIZE], rx[SPI_BUFFERSIZE];

    tx[0] = 0x01 | (address << 1);
    memcpy(tx + 1, data, size);
	if (spi_write_then_read(m_bp_spi->spi, tx, size+1, rx, 0) != 0) {
		printk(KERN_ERR"spi_write_then_read failed\n");
		return -1;
	}
    status = size;
    return status;
}

int bp_spi_write(uint8_t bank, uint8_t address, const uint8_t *data, uint8_t length) {
    int status = SPI_ERR_UNKNOWN;

    if (bank != local_bank) {
        ONERRORGOTO(spi_write_law(SPI_ADR_BANK_SEL, &bank, 1));
        local_bank = bank;
    }
    ONERRORGOTO(spi_write_law(address, data, length));
    status = length;
error:
    return status;
}

static int spi_read_law(uint8_t address, uint8_t *data, uint8_t size) {
    int status = SPI_ERR_UNKNOWN;
    static uint8_t tx[SPI_BUFFERSIZE], rx[SPI_BUFFERSIZE];

    tx[0] = 0x00 | (address << 1);
	if (spi_write_then_read(m_bp_spi->spi, tx, 1, rx, size) != 0) {
		printk(KERN_ERR"spi_write_then_read failed\n");
		return -1;
	}
	if( data ) /* NULL»ØÄê¤Ç¶õÆÉ¤ß */
	    memcpy(data, rx, size);
    status = size;
    return status;
}

int bp_spi_read(uint8_t bank, uint8_t address, uint8_t *data, uint8_t size) {
    int status = SPI_ERR_UNKNOWN;
    if (bank != local_bank) {
        ONERRORGOTO(spi_write_law(SPI_ADR_BANK_SEL, &bank, 1));
        local_bank = bank;
    }
    ONERRORGOTO(spi_read_law(address, data, size));
    status = size;
error:
    return status;
}
