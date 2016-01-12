/*
 * BP3596 PHY layer
 * 
 * File:  phy-bp3596.h
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


#ifndef _PHY_H_
#define _PHY_H_

#include "mac-802154e.h"
#include "drv-802154e.h"

#define PHY_OK				0
#define PHY_ERR_UNKNOWN		-1
#define PHY_ERR_PARAMETER	-2	// 無効な引数 */
#define PHY_ERR_DEVOPEN		-3	// デバイスファイルオープン失敗 */
#define PHY_ERR_DEVMAP		-4	// mmap失敗 */
#define PHY_ERR_DEVIOCTL	-5	// ioctl失敗 */
#define PHY_ERR_DEVWRITE	-6	// デバイスファイルへの書き込み失敗 */
#define PHY_ERR_DEVREAD		-7	// デバイスファイルからの読み出し失敗 */
#define PHY_ERR_CRC			-8	// CRCエラー */
#define PHY_ERR_FIFO		-9	// FIFOアクセスエラー */
#define PHY_ERR_PLLLOCK		-10	// PLLロック外れ */
#define PHY_ERR_CCA			-11	// CCA Error


typedef struct {
	char name[32];
	int	(*init)(t_PHY_INIT_PARAM *p, int (*rx_callback)(uint8_t* data, uint16_t len));
	int	(*get_address)(uint8_t *addr);
	int	(*send_now)(const uint8_t *data, uint16_t len, uint8_t ack);
	int	(*send)(const uint8_t *data, uint16_t len);
//	int (*rx_callback)(uint8_t* data, uint16_t len);
	int (*remove)(void);
	// 2015.08.27 Eiichi Saito
	void (*set_ack)(int status);
}PHY;


extern const PHY phy;

#endif // _PHY_BP3596_H_


