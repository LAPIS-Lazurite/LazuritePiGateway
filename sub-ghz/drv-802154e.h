/*
 * IEEE802.15.4e driver IO
 * 
 * File:  drv-802154e.c
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


#ifndef _DRV_802154E_H_
#define _DRV_802154E_H_

#include "mac-802154e.h"
#define	DRV_OK				0
#define DRV_ERR_DEVNAME		-1

typedef struct {
	char *devname;
	int (*addlist)(t_MAC_HEADER* phdr);
	t_802154E_SETTING* set;
} t_DRV_INIT_PARAM;


#endif // _DRV_802154E_H_


