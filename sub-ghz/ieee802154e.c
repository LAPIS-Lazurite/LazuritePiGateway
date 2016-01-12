/*
 * IEEE 802.15.4e header
 * Copyright 2015 Lapis Semiconductor Co.,Ltd.
 * Author: Naotaka Sasito
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


//#include <linux/module.h>
//#include <linux/init.h>
//#include <linux/wait.h>
//#include <linux/sched.h>
//#include <linux/ioctl.h>
//#include <linux/parport.h>
//#include <linux/ctype.h>
//#include <linux/poll.h>
//#include <linux/slab.h>
//#include <linux/major.h>
//#include <linux/errno.h>
//#include <linux/input.h>
//#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/string.h>
#include "mac-802154e.h"

#define ACK_ADDRESS
#ifdef IEEE_HEADR_TEST
	#include <stdio.h>
	#include <string.h>
#endif

#define BEACON_FRAME	0x00
#define DATA_FRAME	0x01
#define ACK_FRAME	0x02
#define COMMAND_FRAME	0x03
#define LLDN_FRAME	0x04
#define MULTI_FRAME	0x05

typedef struct {
	uint8_t frame_type:3;
	uint8_t sec_enb:1;
	uint8_t pending:1;
	uint8_t ack_req:1;
	uint8_t panid_comp:1;
	uint8_t none:1;
	uint8_t seq_comp:1;
	uint8_t ielist:1;
	uint8_t rx_addr_type:2;
	uint8_t frame_ver:2;
	uint8_t tx_addr_type:2;
} t_FRAME_HEADER_BIT;

int enc_ieee802154e_header(t_MAC_HEADER* pHdr)
{
	int status = 0;
	static uint8_t seq=0x80;
	//int i;
	//uint8_t *buf = pHdr->raw.data;
	union  {
		uint16_t dat16;
		t_FRAME_HEADER_BIT hdr_bit;
	} tmp;
	uint8_t addr_type;
	uint16_t offset = 0;

	tmp.dat16 = 0;
	tmp.hdr_bit.frame_type = pHdr->frame_type;
	tmp.hdr_bit.sec_enb = pHdr->sec_enb;
	tmp.hdr_bit.pending = pHdr->pending;
	tmp.hdr_bit.seq_comp = pHdr->seq_comp;
	if(tmp.hdr_bit.seq_comp == 0)
		pHdr->seq = seq++;		// update seqence number
	if(pHdr->ielist.len == 0)
	{
		tmp.hdr_bit.ielist = 0;
	}
	else
	{
		// dpes not support
		tmp.hdr_bit.ielist = 1;
	}
	tmp.hdr_bit.tx_addr_type = pHdr->tx_addr.addr_type;
	
	tmp.hdr_bit.rx_addr_type = pHdr->rx_addr.addr_type;
	tmp.hdr_bit.frame_ver = pHdr->frame_ver;

	if(pHdr->tx_addr.addr_type == 0) addr_type = 0;
	else addr_type = 4;

	if(pHdr->rx_addr.addr_type != 0) addr_type += 2;

	switch(addr_type)
	{
	case 0:
		if((pHdr->tx_addr.panid_enb == 0)&& (pHdr->rx_addr.panid_enb == 0))
		{
			tmp.hdr_bit.panid_comp = 0;
			addr_type = 0;
		}
		else
		{
			tmp.hdr_bit.panid_comp = 1;
			addr_type = 1;
		}
		break;
	case 2:
		if((pHdr->tx_addr.panid_enb == 1)&& (pHdr->rx_addr.panid_enb == 0))
		{
			tmp.hdr_bit.panid_comp = 0;
			addr_type = 2;
		}
		else
		{
			tmp.hdr_bit.panid_comp = 1;
			addr_type = 3;
		}
		break;
	case 4:
		if((pHdr->tx_addr.panid_enb == 0)&& (pHdr->rx_addr.panid_enb == 1))
		{
			tmp.hdr_bit.panid_comp = 0;
			addr_type = 4;
		}
		else
		{
			tmp.hdr_bit.panid_comp = 1;
			addr_type = 5;
		}
		break;
	case 6:
		if((pHdr->tx_addr.panid_enb == 0)&& (pHdr->rx_addr.panid_enb == 1))
		{
			tmp.hdr_bit.panid_comp = 0;
			addr_type = 6;
		}
		else
		{
			tmp.hdr_bit.panid_comp = 1;
			addr_type = 7;
		}
		break;
	}

	if((addr_type == 6 )||(addr_type == 7)) pHdr->ack_req = 1;
	else pHdr->ack_req = 0;
	tmp.hdr_bit.ack_req = pHdr->ack_req;

	// copy to raw data
	*((uint16_t *)(pHdr->raw.data + offset)) = tmp.dat16;
	offset+=2;

	pHdr->raw.data[offset++]=pHdr->seq;

	if(pHdr->rx_addr.panid_enb == 1)
	{
		*((uint16_t *)(pHdr->raw.data + offset)) = pHdr->rx_addr.panid;
		offset+=2;
	}
	switch(pHdr->rx_addr.addr_type)
	{
	case 0:
		break;
	case 1:
		pHdr->raw.data[offset] = pHdr->rx_addr.addr.addr8;
		offset+=1;
		break;
	case 2:
		*((uint16_t *)(pHdr->raw.data + offset)) = pHdr->rx_addr.addr.addr16;
		offset += 2;
		break;
	case 3:
		memcpy(&pHdr->raw.data[offset],pHdr->rx_addr.addr.addr64,8);
		offset+=8;
		break;
	}
	if(pHdr->tx_addr.panid_enb == 1)
	{
		*((uint16_t *)(pHdr->raw.data + offset)) = pHdr->tx_addr.panid;
		offset+=2;
	}
	switch(pHdr->tx_addr.addr_type)
	{
	case 0:
		break;
	case 1:
		pHdr->raw.data[offset] = pHdr->tx_addr.addr.addr8;
		offset+=1;
		break;
	case 2:
		*((uint16_t *)(pHdr->raw.data + offset)) = pHdr->tx_addr.addr.addr16;
		offset += 2;
		break;
	case 3:
		memcpy(&pHdr->raw.data[offset],pHdr->tx_addr.addr.addr64,8);
		offset+=8;
		break;
	}
	memcpy(&pHdr->raw.data[offset],pHdr->payload.data, pHdr->payload.len);
	offset += pHdr->payload.len;
	pHdr->raw.len = offset;

	return status;
}
int dec_ieee802154e_header(t_MAC_HEADER* pHdr, bool rssi_enb)
{
	uint16_t offset = 0;
	uint8_t panid_comp;
	uint8_t addr_type;
	int status;
	uint8_t *buf = pHdr->raw.data;
	union  {
		uint16_t dat16;
		t_FRAME_HEADER_BIT hdr_bit;
	} tmp;

	tmp.dat16 = (uint16_t)buf[offset++];
	tmp.dat16 += ((uint16_t)buf[offset++]<<8); 


	pHdr->frame_type = (e_IEEE_FRAME_TYPE)tmp.hdr_bit.frame_type;
	pHdr->sec_enb = tmp.hdr_bit.sec_enb;
	pHdr->pending = tmp.hdr_bit.pending;
	pHdr->ack_req = tmp.hdr_bit.ack_req;
	panid_comp = tmp.hdr_bit.panid_comp;
	pHdr->seq_comp = tmp.hdr_bit.seq_comp;
	if(tmp.hdr_bit.ielist == 0)
	{
		pHdr->ielist.len = 0;
		pHdr->ielist.data = NULL;
	}
	else
	{
		// does not support ielist
		// need to add code
		pHdr->ielist.len = 1;
	}
	pHdr->tx_addr.addr_type = tmp.hdr_bit.tx_addr_type;
	pHdr->frame_ver = tmp.hdr_bit.frame_ver;
	pHdr->rx_addr.addr_type = tmp.hdr_bit.rx_addr_type;

	//	address type
	addr_type=0;
	if (pHdr->tx_addr.addr_type != 0) addr_type = 2;
	if (pHdr->rx_addr.addr_type != 0) addr_type += 4;
	if (panid_comp != 0) addr_type += 1;

#ifdef IEEE_HEADR_TEST
	printf("addr_type=%d\n",addr_type);
#endif

	//	address
	switch(addr_type)
	{
	case 0:
		pHdr->tx_addr.panid_enb = 0, pHdr->rx_addr.panid_enb = 0;
		break;
	case 1:
		pHdr->tx_addr.panid_enb = 0, pHdr->rx_addr.panid_enb = 1;
		break;
	case 2:
		pHdr->tx_addr.panid_enb = 1, pHdr->rx_addr.panid_enb = 0;
		break;
	case 3:
		pHdr->tx_addr.panid_enb = 0, pHdr->rx_addr.panid_enb = 0;
		break;
	case 4:
		pHdr->tx_addr.panid_enb = 0, pHdr->rx_addr.panid_enb = 1;
		break;
	case 5:
		pHdr->tx_addr.panid_enb = 0, pHdr->rx_addr.panid_enb = 0;
		break;
	case 6:
		pHdr->tx_addr.panid_enb = 0, pHdr->rx_addr.panid_enb = 1;
		break;
	case 7:
		pHdr->tx_addr.panid_enb = 0, pHdr->rx_addr.panid_enb = 0;
		break;
	}

	//	sequence
	if(pHdr->seq_comp) pHdr->seq = 0; else pHdr->seq = buf[offset++];

	//	rx_panid
	if(pHdr->rx_addr.panid_enb)
	{
		pHdr->rx_addr.panid = (uint16_t)buf[offset++];
		pHdr->rx_addr.panid += ((uint16_t)buf[offset++]<<8);
	}
	else pHdr->rx_addr.panid = 0;

	//	rx_address
	if(pHdr->rx_addr.addr_type)
	{
		switch(pHdr->rx_addr.addr_type)
		{
		case 1:
			pHdr->rx_addr.addr.addr8 = buf[offset++];
			break;
		case 2:
			pHdr->rx_addr.addr.addr16 = (uint16_t)buf[offset++];
			pHdr->rx_addr.addr.addr16 += ((uint16_t)buf[offset++]<<8);
			break;
		case 3:
			memcpy(pHdr->rx_addr.addr.addr64,(buf+offset),8);
			offset += 8;
			break;
		default:
			goto error;
			break;
		}
	}

	//	tx panid
	if(pHdr->tx_addr.panid_enb)
	{
		pHdr->tx_addr.panid = (uint16_t)buf[offset++];
		pHdr->tx_addr.panid += ((uint16_t)buf[offset++]<<8);
	}
	else pHdr->tx_addr.panid = 0;

	//	tx_address
	if(pHdr->tx_addr.addr_type)
	{
		switch(pHdr->tx_addr.addr_type)
		{
		case 1:
			pHdr->tx_addr.addr.addr8 = buf[offset++];
			break;
		case 2:
			pHdr->tx_addr.addr.addr16 = (uint16_t)buf[offset++];
			pHdr->tx_addr.addr.addr16 += ((uint16_t)buf[offset++]<<8);
			break;
		case 3:
			memcpy(pHdr->tx_addr.addr.addr64,(buf+offset),8);
			offset += 8;
			break;
		default:
			goto error;
			break;
		}
	}



	//	IE LIST
	//	not supported. data is recognized as payload.

	//	payload
	//	data check (if no data, error)
	if(offset < pHdr->raw.len)
	{
		pHdr->payload.len = pHdr->raw.len - offset -rssi_enb;
		pHdr->payload.data = buf + offset;
		offset = pHdr->raw.len-1;
	}
	else
	{
		pHdr->payload.len = 0;
		pHdr->payload.data = NULL;
		status = -1;
		goto error;
	}

	//	rssi
	if(rssi_enb != 0)
		pHdr->rssi = buf[offset++];

error:
	return status;
}

int enc_ieee802154e_ack_header(t_MAC_HEADER* pHdr,t_MAC_DATA *raw)
{
    //uint8_t buf[4];
    int status = 0;
	int len = 0;
    union {
        uint16_t data16;
        t_FRAME_HEADER_BIT hbit;
    } tmp;
	tmp.data16 = 0;

	// set frame control
    tmp.hbit.frame_type = ACK_FRAME;
    tmp.hbit.sec_enb = 0;
    tmp.hbit.pending = 0;
    tmp.hbit.ack_req = 0;
    tmp.hbit.seq_comp = pHdr->seq_comp;
    tmp.hbit.ielist = 0;
    tmp.hbit.frame_ver = pHdr->frame_ver;

#ifdef	ACK_CTI
	tmp.hbit.tx_addr_type = 2;
	tmp.hbit.rx_addr_type = 2;
	tmp.hbit.panid_comp = 0;

    raw->data[len++] = (uint8_t)(tmp.data16 & 0x00FF);
    raw->data[len++] = (uint8_t)(tmp.data16 >> 8); 

    if(pHdr->seq_comp == 0) raw->data[len++] = pHdr->seq;

    raw->data[len++] = (uint8_t)(pHdr->rx_addr.panid & 0x00FF);
    raw->data[len++] = (uint8_t)(pHdr->rx_addr.panid >> 8); 

    raw->data[len++] = (uint8_t)(pHdr->rx_addr.addr.addr16 & 0x00FF);
    raw->data[len++] = (uint8_t)(pHdr->rx_addr.addr.addr16 >> 8); 

    raw->data[len++] = (uint8_t)(pHdr->tx_addr.addr.addr16 & 0x00FF);
    raw->data[len++] = (uint8_t)(pHdr->tx_addr.addr.addr16 >> 8); 
#endif	//ACK_CTI

#ifdef ACK_SHORT
	// set address type = 0 (tx addr = off, rx addr = off, panid_comp = 0)
	// tx panid = off, tx addr = off
	// rx panid = off, rx addr = off
    tmp.hbit.tx_addr_type = 0;
    tmp.hbit.rx_addr_type = 0;
    tmp.hbit.panid_comp = 0;

	// set frame control to buffer
    raw->data[len++] = (uint8_t)(tmp.data16 & 0x00FF);
    raw->data[len++] = (uint8_t)(tmp.data16 >> 8); 

	// set sequence number to buffer
    if(pHdr->seq_comp == 0)
	{
        raw->data[len++] = pHdr->seq;
    }   
#endif	// ACK_SHORT

#ifdef	ACK_ADDRESS
	// set address type = 2 (tx addr = on, rx addr = off, panid_comp = 0)
	// tx panid = on,  tx addr = on
	// rx panid = off, rx addr = 0ff
    tmp.hbit.tx_addr_type = pHdr->rx_addr.addr_type;
    tmp.hbit.rx_addr_type = 0;
    tmp.hbit.panid_comp = 0;

	// set frame control to buffer
    raw->data[len++] = (uint8_t)(tmp.data16 & 0x00FF);
    raw->data[len++] = (uint8_t)(tmp.data16 >> 8); 

	// set sequence number to buffer
    if(pHdr->seq_comp == 0)
	{
        raw->data[len++] = pHdr->seq;
    }   

	// set tx panid to buffer
    raw->data[len++] = (uint8_t)(pHdr->rx_addr.panid & 0x00FF);
    raw->data[len++] = (uint8_t)(pHdr->rx_addr.panid >> 8); 

	// set tx panid to buffer
	{
		const unsigned char addr_len[]={0,1,2,8};
		memcpy(&raw->data[len], pHdr->tx_addr.addr.addr64, addr_len[pHdr->tx_addr.addr_type]);
		len += addr_len[pHdr->tx_addr.addr_type];
	}
#endif	// ACK_ADDRESS

	raw->len = len;

    return status;
}

int set_ieee802154e_addr(t_MAC_HEADER* pHdr,u_MAC_ADDR *paddr_from, bool rx )
{
	int status = 0;
	t_MAC_ADDR *paddr_to;

	if(rx==true) paddr_to = &pHdr->rx_addr;
	else paddr_to = &pHdr->tx_addr;

	switch(paddr_to->addr_type)
	{
	case 0:
		paddr_to->addr.addr8 = paddr_from->addr8;
		break;
	case 1:
		paddr_to->addr.addr16 = paddr_from->addr16;
		break;
	case 2:
		memcpy(paddr_to->addr.addr64,paddr_from->addr64,8);
		break;
	default:
		status = -1;
		break;
	}
	return status;
}

