/*
 * IEEE802.15.4e MAC Layer
 * 
 * File:  mac-802154e.c
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
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/parport.h>
#include <linux/ctype.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/major.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/timer.h>

#include "common_802154e.h"

#include "drv-802154e.h"
#include "mac-802154e.h"
#include "phy.h"
#include "ieee802154e.h"

#define MAC_THROUGH		1			// MAC THROGH
#define MAC_VALID		0

// 2015.08.21 Eiichi Saito : Retry Packet 
#define MAX_RETRY		3
#define EXPIRES_TIME	10	// Original: 10:100ms, 20:50ms, 30:30ms


t_MAC_PARAM		macParam;
uint8_t			rx_raw[256];
uint8_t			tx_raw[256];
int (*drv_rx_callback)(t_MAC_HEADER *phdr);

int finish_flag2;
static wait_queue_head_t tx_ack_q;
static struct timer_list g_timer;
int recv_ack;

static int addr_comp(t_MAC_ADDR *pAddr)
{
	int result=false;



	if(((pAddr->panid_enb == 1) &&( macParam.myInfo.panid == pAddr->panid)) ||
	    (pAddr->panid_enb == 0))
	{
		DEBUGONDISPLAY(MODE_MAC_DEBUG, printk("PANID : addrType=%04x,panid_enb=%04x, myPanid=%04x, rxPanid=%04x\n",
			pAddr->addr_type,      pAddr->panid_enb,
			macParam.myInfo.panid, pAddr->panid));
		switch(pAddr->addr_type)
		{
		case 1:
			if(pAddr->addr.addr8 == macParam.myInfo.myAddr.addr8) result = true;
			DEBUGONDISPLAY(MODE_MAC_DEBUG, printk("myAddr=%02x,rxAddr=%02x\n", pAddr->addr.addr8, macParam.myInfo.myAddr.addr8));
			break;
		case 2:
			if(pAddr->addr.addr16 == macParam.myInfo.myAddr.addr16) result = true;
			DEBUGONDISPLAY(MODE_MAC_DEBUG, printk("rxAddr=%04x,myAddr=%04x\n", pAddr->addr.addr16, macParam.myInfo.myAddr.addr16));
			break;
		case 3:
			if(memcmp(pAddr->addr.addr64, macParam.myInfo.myAddr.addr64,8)==0) result = true;
			DEBUGONDISPLAY(MODE_MAC_DEBUG, printk("64bit address is not supported!!\n"));
			break;
		}
	}
	else
	{
		DEBUGONDISPLAY(MODE_MAC_DEBUG, printk("PANID ERROR: myPanid=%04x, rxPanid=%04x\n", macParam.myInfo.panid,pAddr->panid));
	}
	return result;
}

// fuction of MAC
const static int mac_func_beacon(void)
{
	printk("receiving beacon frame");
	return 0;
}
const static int mac_func_data(void)
{
	int status = 0;
	DEBUGONDISPLAY(MODE_MAC_DEBUG, printk("Recieving data frame\n"));
	if(addr_comp(&macParam.rxHdr.rx_addr) == true)
	{
		// check sequence number
		if(macParam.rxHdr.ack_req == 1)
		{
			// add send ack process
			uint8_t buf[16];
			t_MAC_DATA raw = {
				.data = buf,
				.len = 0,
			};
			enc_ieee802154e_ack_header(&macParam.rxHdr,&raw);
			status = phy.send_now(raw.data,raw.len, true);
			DEBUGONDISPLAY(MODE_MAC_DEBUG, printk("SEND ACK!!\n"));
			DEBUGONDISPLAY(MODE_MAC_DEBUG, PAYLOADDUMP(raw.data,raw.len));
			if(macParam.rxHdr.last_seq != macParam.rxHdr.seq)
			{
				macParam.rxHdr.last_seq = macParam.rxHdr.seq;
		 		status = drv_rx_callback(&macParam.rxHdr);
			}
			else DEBUGONDISPLAY(MODE_MAC_DEBUG, printk("same seq number\n"));
		}
		else DEBUGONDISPLAY(MODE_MAC_DEBUG, printk("no ack\n"));
	}
	else
	{
		DEBUGONDISPLAY(MODE_MAC_DEBUG, printk("mismatch address\n"));
	}
	return status;
}
const static int mac_func_ack(void)
{
	if((addr_comp(&macParam.rxHdr.rx_addr) == true) &&
			(macParam.txHdr.seq == macParam.rxHdr.seq ))
	{
		uint8_t size = 0;
		switch(macParam.rxHdr.tx_addr.addr_type)
		{
		case 1:
			size = 1;
			break;
		case 2:
			size = 2;
			break;
		case 3:
			size = 8;
			break;
		}
		if(memcmp(&macParam.rxHdr.tx_addr.addr, &macParam.txHdr.rx_addr.addr,size) == 0)
		{
			recv_ack = true;
			finish_flag2 = 1;
			wake_up_interruptible( &tx_ack_q );
		}
	}

	return 0;
}
const static int mac_func_cmd(void)
{
	printk("receiving cmd frame");
	return 0;
}
const static int mac_func_lldn(void)
{
	printk("receiving LLDN frame");
	return 0;
}
const static int mac_func_multi(void)
{
	printk("receiving MULTI frame");
	return 0;
}

const static int (*mac_func_list[])(void) = 
{
	mac_func_beacon,
	mac_func_data,
	mac_func_ack,
	mac_func_cmd,
	mac_func_lldn,
	mac_func_multi
};

static int mac_recv_data(uint8_t *data, uint16_t len)
{
	int status = 0;

	DEBUGONDISPLAY(MODE_MAC_DEBUG, printk(KERN_INFO "[MAC-802154E]recv data dump\n"));
	DEBUGONDISPLAY(MODE_MAC_DEBUG, PAYLOADDUMP(data,len));

	memcpy(macParam.rxHdr.raw.data,data,len);
	macParam.rxHdr.raw.len = len;

	// decode raw data to parameter
	dec_ieee802154e_header(&macParam.rxHdr,true);

	// SNIFFER MODE CHECK
	if(drv_mode & MODE_INVALID_MAC)
	{
		drv_rx_callback(&macParam.rxHdr);
		goto INVALID;
	}

	// MAC Process
	// check frame header
	if(macParam.rxHdr.frame_ver != IEEE802154E)
		goto INVALID;
	switch(macParam.rxHdr.frame_type)
	{
	case BEACON_FRAME:
		status = mac_func_list[BEACON_FRAME]();
		break;
	case DATA_FRAME:
		status = mac_func_list[DATA_FRAME]();
		break;
	case ACK_FRAME:
		status = mac_func_list[ACK_FRAME]();
		break;
	case CMD_FRAME:
		status = mac_func_list[CMD_FRAME]();
		break;
	case LLDN_FRAME:
		status = mac_func_list[LLDN_FRAME]();
		break;
	case MULTI_FRAME:
		status = mac_func_list[MULTI_FRAME]();
		break;
	default:
		goto INVALID;
		break;
	}

INVALID:
	return status;
}

// timer handler for ack timeout
void timer_ack_timeout(unsigned long data)
{
	recv_ack = false;
	finish_flag2 = 1;
	wake_up_interruptible( &tx_ack_q );
}

// ssdebug
void mac_add_timer(void)
{
	add_timer(&g_timer);
}

int mac_send(const uint8_t *data,uint16_t len)
{
	int status = len;
	// data copy to mac 
	memcpy(macParam.txHdr.raw.data,data,len);
	macParam.txHdr.raw.len = len;

	//update my address , ack 
	dec_ieee802154e_header(&macParam.txHdr,false);
	set_ieee802154e_addr(&macParam.txHdr,&macParam.myInfo.myAddr, false);
	enc_ieee802154e_header(&macParam.txHdr);

	//printk("send data\n");
	//PAYLOADDUMP(macParam.txHdr.raw.data,macParam.txHdr.raw.len);
	// initializing timer for waiting ack
	g_timer.data = 0;
	g_timer.expires = jiffies + HZ/EXPIRES_TIME;
	g_timer.function = timer_ack_timeout;
	//initializing ack status
	recv_ack = -1;

	if(macParam.txHdr.ack_req == 1)
	{
		// 2015.08.21 Eiichi Saito : Retry Packet 
		uint8_t retry=MAX_RETRY;

		//printk("waiting ack...\n");
		// ssdebug
		// add_timer(&g_timer);
		//finish_flag = 0;
		// ssdebug
        phy.send(macParam.txHdr.raw.data,macParam.txHdr.raw.len,mac_add_timer);
		finish_flag2 = 0;
		do
		{
			wait_event_interruptible(tx_ack_q, finish_flag2);
			if(recv_ack == false)
			{
				del_timer(&g_timer);
				// 2015.08.21 Eiichi Saito : Retry Packet 
				if (retry) {
					retry--;
					g_timer.data = 0;
					g_timer.expires = jiffies + HZ/EXPIRES_TIME;
					g_timer.function = timer_ack_timeout;

					recv_ack = -1;

					// ssdebug
					// add_timer(&g_timer);
			  		phy.send(macParam.txHdr.raw.data,macParam.txHdr.raw.len,mac_add_timer);
				}else
				{
					status = -1;
				}
			}
			else if(recv_ack == true)
			{
				del_timer(&g_timer);
				status = macParam.txHdr.raw.len;
			}
		} while(recv_ack<0);

	}
	else
	{
		// ssdebug
		phy.send(macParam.txHdr.raw.data,macParam.txHdr.raw.len,mac_add_timer);

	}

	return status;
}

int mac_remove(void)
{
	int status = 0;
	// MAC remove
	DEBUGONDISPLAY(MODE_MAC_DEBUG,printk(KERN_INFO "[MAC-802154E]remove\n"));
	phy.remove();
	return status;
}

int mac_param_init(t_802154E_SETTING *p, int (*rx_callback)(t_MAC_HEADER *phdr))
{
	int status = MAC_OK;
	
	// link buffer pointer
	macParam.rxHdr.raw.data = rx_raw;
	macParam.txHdr.raw.data = tx_raw;
	macParam.rxHdr.last_seq = -1;
	macParam.txHdr.last_seq = 0;
	
	// set initializing parameter of phy
	macParam.myInfo.panid = p->panid;
	macParam.phyInfo.ch = p->ch;
	macParam.phyInfo.tx_pwr = p->tx_pwr;
	macParam.phyInfo.bitrate = p->bitrate;
	//macParam.phyInfo.rx_raw = &macParam.rxHdr.raw;
	//macParam.phyInfo.tx_raw = &macParam.txHdr.raw;
	drv_rx_callback = rx_callback;
	printk(KERN_INFO "[MAC-802154E]parameter init\n");
	
	// PHY initialization
	//phy.rx_callback = mac_recv_data;
	status = phy.init(&macParam.phyInfo,mac_recv_data);
	if(status != PHY_OK) return status;

	phy.get_address(macParam.myInfo.myAddr.addr64);
	printk("[MAC Address: %02x %02x %02x %02x %02x %02x %02x %02x\n",
		macParam.myInfo.myAddr.addr64[0],
		macParam.myInfo.myAddr.addr64[1],
		macParam.myInfo.myAddr.addr64[2],
		macParam.myInfo.myAddr.addr64[3],
		macParam.myInfo.myAddr.addr64[4],
		macParam.myInfo.myAddr.addr64[5],
		macParam.myInfo.myAddr.addr64[6],
		macParam.myInfo.myAddr.addr64[7]);
	
	// initializign interrupt for waiting ack
	init_waitqueue_head( &tx_ack_q );
	// initializing timer for ack timeout
	init_timer(&g_timer);

	return status;
}
int mac_get_name(char* name)
{
	int len;

	// DEVNAME
	len = strlen(phy.name);
	if((len > 0) || len < 16)
	{
		strncpy(name,phy.name,sizeof(phy.name));
		return MAC_OK;
	}
	return -1;
}

const MAC mac ={
	.init = mac_param_init,
	.send = mac_send,
	.get_name = mac_get_name,
	.remove = mac_remove,
};


