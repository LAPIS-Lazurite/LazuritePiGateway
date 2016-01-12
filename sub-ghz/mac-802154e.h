/*
 * IEEE802.15.4e driver IO
 * 
 * File:  drv-802154e.c
 * 
 * Copyright 2015 Lapis Semiconductor Co.,Ltd.
 * Author: Naotaka Saito
 * 
 * The program is developed refered to BP3596 driver by Communication Technology
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/



#ifndef _MAC_802154E_H_
#define _MAC_802154E_H_

#include <linux/types.h>
#include <linux/string.h>
#ifdef IEEE_HEADR_TEST
	#include <inttypes.h>
	#include <stdbool.h>
#endif

#define MAC_OK				0

#define MAC_FUNC_ERROR		-1
#define MAC_INIT_ERROR		-2

#define ADDR_TYPE_INVALID	0
#define ADDR_TYPE_8BIT		1
#define ADDR_TYPE_16BIT		2
#define ADDR_TYPE_64BIT		3
typedef enum {
	BEACON_FRAME = 0,
	DATA_FRAME,
	ACK_FRAME,
	CMD_FRAME,
	LLDN_FRAME,
	MULTI_FRAME
} e_IEEE_FRAME_TYPE;

typedef enum {
	IEEE802154_2003 = 0,
	IEEE802154_2006,
	IEEE802154E
} e_IEEE_FRAME_VER;

typedef union {
	uint8_t				addr8;
	uint16_t			addr16;
	uint8_t				addr64[8];
} u_MAC_ADDR;

typedef union {
	uint8_t				sec1;
	uint16_t			sec2;
	uint8_t				sec5[5];
	uint8_t				sec6[6];
	uint8_t				sec10[10];
	uint8_t				sec16[16];
} u_MAC_SECURITY;

typedef struct {
	uint8_t				panid_enb;
	uint16_t			panid;
	uint8_t				addr_type;
	u_MAC_ADDR 			addr;
} t_MAC_ADDR;


typedef struct {
//	uint8_t				enb;
	uint16_t			len;
	uint8_t				*data;
} t_MAC_DATA;

typedef struct {
	t_MAC_DATA			raw;
	e_IEEE_FRAME_TYPE	frame_type;
	e_IEEE_FRAME_VER	frame_ver;
	t_MAC_ADDR			tx_addr;
	t_MAC_ADDR			rx_addr;
	uint8_t				seq_comp;
	uint8_t				seq;
	int16_t				last_seq;
	uint8_t				sec_enb;
	u_MAC_SECURITY		sec;
	uint8_t				pending;
	uint8_t				ack_req;
	uint8_t				crc;
	uint8_t				rssi;			// in case of tx, it is rssi of ack.
	t_MAC_DATA			ielist;
	t_MAC_DATA			payload;
} t_MAC_HEADER;

typedef struct {
	uint16_t			panid;
	u_MAC_ADDR			myAddr;
} t_MY_INFO;

typedef struct {
	uint8_t				ch;
	uint8_t				tx_pwr;
	uint8_t 			bitrate;
	t_MAC_DATA			*rx_raw;
	t_MAC_DATA			*tx_raw;
} t_PHY_INIT_PARAM;

typedef struct {
	//uint8_t				mac;
	t_MY_INFO			myInfo;
	t_PHY_INIT_PARAM	phyInfo;
	t_MAC_HEADER		txHdr;
	t_MAC_HEADER		rxHdr;
} t_MAC_PARAM;

typedef struct {
	uint8_t	mode;			//MAC mode ( 0: trace all data, 1: compare address)
							// 				set by module_param
	uint8_t	ch;				// channel 		set by module_param
	uint8_t bitrate;		// bitrate		set by module_param
	uint8_t tx_pwr;			// tx power		set by module_param
	uint16_t panid;			// my panid	set by module_param
} t_802154E_SETTING;

typedef struct {
	int (*init)(t_802154E_SETTING *param,int (*rx_callback)(t_MAC_HEADER *phdr) );
//	int (*rx_callback)(t_MAC_HEADER *phdr);
	int (*send)(const uint8_t *data, uint16_t len);
	int (*get_name)(char* name);
	int (*remove)(void);
} MAC;

extern const MAC mac;

#endif // _MAC_802154E_H_


