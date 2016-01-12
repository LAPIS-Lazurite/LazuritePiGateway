/*
 * IEEE802.15.4 header process
 * 
 * File:  ieee802154e.h
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
#include "mac-802154e.h"
#ifndef	_IEEE802154E_H_
#define	_IEEE802154E_H_

extern int dec_ieee802154e_header(t_MAC_HEADER* pHdr, bool rx);
extern int enc_ieee802154e_header(t_MAC_HEADER* pHdr);
extern int enc_ieee802154e_ack_header(t_MAC_HEADER* pHdr, t_MAC_DATA* raw);
extern int set_ieee802154e_addr(t_MAC_HEADER* pHdr,u_MAC_ADDR *paddr_from, bool rssi_enb );


#endif	// _I2C_BP3596_H_
