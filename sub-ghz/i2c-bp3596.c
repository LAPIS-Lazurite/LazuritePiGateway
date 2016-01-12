/*
 * BP3596 I2C Driver
 * 
 * File:  i2c-bp3596.c
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
#include <linux/i2c.h>
#include "i2c-bp3596.h"
#include "common_802154e.h"

#define I2C_BUFFERSIZE 16

#define I2C_NOT_PROBED	NULL
static struct i2c_client *g_client=I2C_NOT_PROBED;

// local function
static int bp_i2c_write(uint8_t address, const uint8_t *data, uint8_t length) {
    static unsigned char buffer[I2C_BUFFERSIZE] = {0};
	int ret;
	if (g_client == NULL)
		return -1;
    buffer[0] = address;
	if (data != NULL)
		memcpy(buffer + 1, data, length);
	ret = i2c_master_send(g_client, buffer, length+1);
	if (ret < 1)
		printk("i2c_master_send failed\n");
	return ret;
}

static int bp_i2c_read(uint8_t address, uint8_t *data, uint8_t length) {
	int ret = 0;
	if (g_client == NULL)
		return -1;
	ret = bp_i2c_write(address, NULL, 0);
	if (ret < 1) {
		printk("i2c_write failed\n");
		return ret;
	}
	ret = i2c_master_recv(g_client, data, length);
	if (ret < 1) {
		printk("i2c_master_recv failed\n");
		return ret;
	}
	return ret;
}
// Driver Interface
static const struct i2c_device_id bp_i2c_id[] = {
	{ "bp3596_i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bp_i2c_id);

static const unsigned short normal_i2c[] = { 0x50, I2C_CLIENT_END};

static int bp_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *did) {
	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("[I2C] driver success\n"));
	if(i2c_get_clientdata(client))
	{
		return -EBUSY;
	}
	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk(KERN_INFO "I2C %s[%02x]\n",client->name, client->addr));
	g_client = client;
	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk(KERN_INFO "I2C %s[%02x]\n success",client->name, client->addr));
	return 0;
}

static int bp_i2c_remove(struct i2c_client *client) {
	if (!client->adapter)
		printk(KERN_INFO "[I2C] client is not attached\n");
		return -ENODEV;	/* our client isn't attached */
	return 0;
}
static struct i2c_driver bp3596_i2c_driver = {
	.driver	= {
		.name = "bp3596_i2c",
	},
	.probe	= bp_i2c_probe,
	.remove	= bp_i2c_remove,
	.id_table = bp_i2c_id,
	.address_list = normal_i2c,
};

// API
int bp_read_eeprom( uint8_t reg )
{
	uint8_t buffer[1];
	uint8_t addr = reg;
	bp_i2c_read(addr, buffer, 1);
	return buffer[0];
}

int bp_i2c_init(void)
{
	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("[I2C] i2c add driver\n"));
	return i2c_add_driver(&bp3596_i2c_driver);
}

int bp_i2c_adapter_init(void)
{
	//int status;
	struct i2c_client *client;
	static struct i2c_adapter *adapter = NULL;
	static struct i2c_board_info info;

	/* I2C adapter init */
	if(g_client == I2C_NOT_PROBED)
	{
		DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("[I2C] i2c get adapter\n"));
		adapter = i2c_get_adapter(1);
		if(adapter == NULL)
		{
			i2c_del_driver(&bp3596_i2c_driver);
			printk("[I2C] i2c adapter fail\n");
			return -2;
		}
		DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("[I2C] i2c new device\n"));
		memset(&info, 0, sizeof(struct i2c_board_info));
		info.addr = 0x50;
		memcpy(info.type, "bp3596_i2c", strlen("bp3596_i2c"));
		client = i2c_new_device(adapter,&info);
		if (client == NULL)
		{
			i2c_del_driver(&bp3596_i2c_driver);
			printk("[I2C] i2c new device fail\n");
			return -3;
		}
		DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("[I2C] i2c put driver\n"));
		i2c_put_adapter(adapter); 
	}
	else
	{
		printk("[I2C] i2c adapter already probed\n");
	}

	return 0;
}

int bp_i2c_del_driver(void)
{
	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("[I2C] del driver\n"));
	i2c_unregister_device(g_client);
	i2c_del_driver(&bp3596_i2c_driver);
	return 0;
}

