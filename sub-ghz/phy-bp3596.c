/*
 * PHY driver of BP3596
 * 
 * File:  phy-bp3596.c
 * 
 * Copyright 2015 Lapis Semiconductor Co.,Ltd.
 * Author: Naotaka Sasito
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
#include <linux/kthread.h>
#include <linux/delay.h>
//#include <linux/list.h>

#include "drv-802154e.h"
#include "mac-802154e.h"
#include "phy.h"
#include "i2c-bp3596.h"
#include "spi-bp3596.h"
#include "phy-bp3596.h"
#include "common_802154e.h"

#define DEVNAME "bp3596"

#define SPI_ADR_BANK_SEL 0x00  /* BANKAU?e´1?¨ */
#define SPI_RESET_WAIT 3  /* \e\≫\A\E\N\e\1Ey[usec] */
#define SPI_BUFFERSIZE 256
#define SINTN_HIGH !0
#define SINTN_LOW 0

#define MIN(a,b) (( a < b )? a : b)
#define SYMBOL_US		16
#define LIFS			(40 * SYMBOL_US)
#define SIFS			(12 * SYMBOL_US)
#define MACMINBE		3
#define MACMAXBE		8
#define MACMAXCSMABACKOFFS		4 /* 最大バックオフ回数 0-5 */
#define MACMAXFRAMERETRIES		3 /* フレーム再送回数 0-7 */
#define aUnitBackoffPeriod		( 20 * SYMBOL_US )
#define BACKOFF( be )		( (xor128()%(be-MACMINBE+1)+MACMINBE) * aUnitBackoffPeriod )

#define GET_TRX_RXON		0x60
#define GET_TRX_TRXOFF		0x80
#define GET_TRX_TXON		0x90

#define SET_TRX_FORCEOFF	0x03
#define SET_TRX_RXON		0x06
#define SET_TRX_TRXOFF		0x08
#define SET_TRX_TXON		0x09

#define GPIO_TX_LED			13
#define GPIO_RX_LED			6

//#define GPIO_CCA_ERR		21

// LED control parameters
#ifdef GPIO_RX_LED
static struct task_struct *rx_led_tsk;      // rx led thread
static wait_queue_head_t rx_led_q;          // rx LED flag
#endif
#ifdef GPIO_TX_LED
static struct task_struct *tx_led_tsk;      // tx thread
static wait_queue_head_t tx_led_q;          // tx LED flag
#endif
 
// mac parameters
static uint8_t macaddr[8];
static uint8_t rx_raw[256];
static uint8_t tx_raw[256];
static uint16_t tx_len=0;
//static uint8_t tx_data_buffer[256];
static struct {
    volatile uint8_t param[256];
} bp = {
    {0}
};

t_PHY_INIT_PARAM phy_param;					// paramter for phy
int (*mac_rx_callback)(uint8_t* data, uint16_t len); // callback to mac, when raw data is received

// thread 
static struct task_struct *kthread_tsk;		// for phy_main
int finish_flag;
int send_flag = false;
// 2015.08.27 Eiichi Saito
int send_cnt = 0;
int phy_recv_ack = false;
static int retry = 0;               		// Send Retry
module_param(retry, int, S_IRUGO);

//	TX/RX LED Control

/* レジスタ初期化テーブル */
typedef  struct {
    uint8_t bank, address, value;
} REG_TABLE;

/* BP3596のレジスタ初期化テーブル */
/* bank | 0x80 => アクセス禁止解除 */
static const REG_TABLE reg_init[] = {
    {0x10, 0x00, 0x00},  /* 特殊コマンド: クロック安定待ち */
	/* 全割り込みの禁止とクリア */
    {SPI_ADR_INT_EN_GRP1,          0x00},
    {SPI_ADR_INT_EN_GRP2,          0x48},
    {SPI_ADR_INT_EN_GRP3,          0x3C},
    {SPI_ADR_INT_EN_GRP4,          0x00},
    {SPI_ADR_INT_SOURCE_GRP1,      0x00},
    {SPI_ADR_INT_SOURCE_GRP2,      0x00},
    {SPI_ADR_INT_SOURCE_GRP3,      0x00},
    {SPI_ADR_INT_SOURCE_GRP4,      0x00},
	/* ユーザ設定 */
    {SPI_ADR_CLK_SET,              0x0f},
    {SPI_ADR_RX_PR_LEN_SFD_LEN,    0x22},
    {SPI_ADR_SYNC_CONDITION,       0x00},
    {SPI_ADR_ACK_TIMER_EN,         0x20},
	{SPI_ADR_AUTO_ACK_EN,          0xc0},
    {SPI_ADR_PLL_MON_DIO_SEL,      0x00},
    {SPI_ADR_2DIV_CNTRL,           0x04},
    {SPI_ADR_2DIV_RSLT,            0x00},
    {SPI_ADR_SYNC_MODE,            0x04},
    {SPI_ADR_RAMP_CNTRL,           0x10},
    {SPI_ADR_PACKET_MODE_SET,      0x3b},
	/* 規格設定 */
    {0x14,                         0x00},
	/* 固定設定 */
    {SPI_ADR_GAIN_MtoL,            0x1e},
    {SPI_ADR_GAIN_LtoM,            0x02},
    {SPI_ADR_GAIN_HtoM,            0x9e},
    {SPI_ADR_GAIN_MtoH,            0x02},
    {SPI_ADR_RSSI_ADJ_M,           0x15},
    {SPI_ADR_RSSI_ADJ_L,           0x2b},
    {SPI_ADR_RSSI_STABLE_TIME,     0x22},
    {SPI_ADR_RSSI_VAL_ADJ,         0xd4},
    {SPI_ADR_AFC_CNTRL,            0x01},
    {SPI_ADR_PREAMBLE_SET,         0xaa},
    {SPI_ADR_SFD1_SET1,            0x09},
    {SPI_ADR_SFD1_SET2,            0x72},
    {SPI_ADR_SFD1_SET3,            0xf6},
    {SPI_ADR_SFD1_SET4,            0x72},
    {SPI_ADR_SFD2_SET1,            0x5e},
    {SPI_ADR_SFD2_SET2,            0x70},
    {SPI_ADR_SFD2_SET3,            0xc6},
    {SPI_ADR_SFD2_SET4,            0xb4},
    {SPI_ADR_2DIV_GAIN_CNTRL,      0xb6},
    {0x80|1,0x39,                  0x84},  /* 隠しレジスタ */
    {SPI_ADR_PLL_CTRL,             0x8f},
    {SPI_ADR_RX_ON_ADJ2,           0x32},
    {SPI_ADR_LNA_GAIN_ADJ_M,       0x0f},
    {SPI_ADR_LNA_GAIN_ADJ_L,       0x01},
    {SPI_ADR_MIX_GAIN_ADJ_M,       0xff},
    {SPI_ADR_MIX_GAIN_ADJ_L,       0xff},
    {SPI_ADR_TX_OFF_ADJ1,          0xb4},
    {SPI_ADR_RSSI_SLOPE_ADJ,       0x01},
    {SPI_ADR_PA_ON_ADJ,            0x04},
    {SPI_ADR_RX_ON_ADJ,            0x0a},
    {SPI_ADR_RXD_ADJ,              0x00},
    {0x80|2,0x2d,                  0x2c},  /* 隠しレジスタ */
    {0x80|2,0x2e,                  0xc0},  /* 隠しレジスタ */
    {0x80|2,0x2f,                  0x17},  /* 隠しレジスタ */
    {0x80|2,0x30,                  0x17},  /* 隠しレジスタ */
	/* 帯域幅/データレート設定(400kHz/100kbps) */
    {SPI_ADR_IF_FREQ_AFC_H,        0x14},
    {SPI_ADR_IF_FREQ_AFC_L,        0x7a},
    {SPI_ADR_BPF_AFC_ADJ_H,        0x02},  /* 後でoffset調整 */
    {SPI_ADR_BPF_AFC_ADJ_L,        0x4a},  /* 後でoffset調整 */
    {SPI_ADR_TX_PR_LEN,            0x04},  /* 0x04以上 */
    {SPI_ADR_DATA_SET,             0x11},
    {SPI_ADR_CH_SPACE_L,           0x82},
    {SPI_ADR_CH_SPACE_H,           0x2d},
    {SPI_ADR_F_DEV_L,              0xb0},
    {SPI_ADR_F_DEV_H,              0x05},
    {SPI_ADR_2DIV_SEARCH,          0x16},
    {SPI_ADR_PLL_CP_ADJ,           0x44},
    {SPI_ADR_IF_FREQ_H,            0x14},
    {SPI_ADR_IF_FREQ_L,            0x7a},
    {SPI_ADR_IF_FREQ_CCA_H,        0x1c},
    {SPI_ADR_IF_FREQ_CCA_L,        0x71},
    {SPI_ADR_BPF_ADJ_H,            0x02},  /* 後でoffset調整 */
    {SPI_ADR_BPF_ADJ_L,            0x4a},  /* 後でoffset調整 */
    {SPI_ADR_BPF_CCA_ADJ_H,        0x01},  /* 後でoffset調整 */
    {SPI_ADR_BPF_CCA_ADJ_L,        0x19},  /* 後でoffset調整 */
    {SPI_ADR_RSSI_LPF_ADJ,         0x1f},
    {0x15,  0x00,                  0x00},  /* レート設定 */
	/* 周波数設定 */
    {0x40|SPI_ADR_CH0_FL,          BP_PARAM_CH0_FL},
    {0x40|SPI_ADR_CH0_FM,          BP_PARAM_CH0_FM},
    {0x40|SPI_ADR_CH0_FH,          BP_PARAM_CH0_FH},
    {0x40|SPI_ADR_CH0_NA,          BP_PARAM_CH0_NA},
    {0x40|SPI_ADR_VCO_CAL_MIN_FL,  BP_PARAM_MIN_FL},
    {0x40|SPI_ADR_VCO_CAL_MIN_FM,  BP_PARAM_MIN_FM},
    {0x40|SPI_ADR_VCO_CAL_MIN_FH,  BP_PARAM_MIN_FH},
    {SPI_ADR_VCO_CAL_MAX_N,        0x07},  /* 帯域幅 400kHz */
    {SPI_ADR_VCO_CAL_START,        0x01},  /* キャリブレーション開始 */
    {0x11, 0x00, 0x00},  /* 特殊コマンド: キャリブレーション完了待ち*/
	/* ユーザ調整値*/
    {SPI_ADR_CCA_LEVEL,            0x55},
    {SPI_ADR_SW_OUT_RAMP_ADJ,      0x0f},
    {SPI_ADR_IQ_MAG_ADJ,           0x08},
    {SPI_ADR_IQ_PHASE_ADJ,         0x20},
	{SPI_ADR_PA_REG_ADJ1,          0x06},
	{SPI_ADR_PA_REG_ADJ2,          0x01},
	{SPI_ADR_PA_REG_ADJ3,          0x01},
    {0x13, 0x00, WSN_DEFPARAM_BP_TXPOW}, /* 特殊コマンド */
	/* パケットモード設定 */
    {SPI_ADR_TX_ALARM_LH,          0x40},
    {SPI_ADR_TX_ALARM_HL,          0x40},
    {SPI_ADR_RX_ALARM_LH,          0xc0},
    {SPI_ADR_RX_ALARM_HL,          0xc0},
	/* CTI追加分 */
    {SPI_ADR_RF_STATUS,            0x06},  /* 受信開始 */
    {SPI_ADR_RF_TEST_MODE,         0x00}, 
    {0xff, 0xff, 0xff}  /* end of table */
};

static REG_TABLE reg_txrate50[] = {
	/* BANK0 */
	{SPI_ADR_IF_FREQ_AFC_H,        0x1C},
	{SPI_ADR_IF_FREQ_AFC_L,        0x71},
	{SPI_ADR_BPF_AFC_ADJ_H,        0x03},
	{SPI_ADR_BPF_AFC_ADJ_L,        0x4B},
	{SPI_ADR_TX_PR_LEN,            0x04},
	{SPI_ADR_DATA_SET,             0x10},
	{SPI_ADR_CH_SPACE_L,           0xC1},
	{SPI_ADR_CH_SPACE_H,           0x16},
	{SPI_ADR_F_DEV_L,              0xD8},
	{SPI_ADR_F_DEV_H,              0x02},
	{SPI_ADR_2DIV_SEARCH,          0x11},
	/* BANK1 */
	{SPI_ADR_PLL_CP_ADJ,           0x44},
	{SPI_ADR_IF_FREQ_H,            0x1C},
	{SPI_ADR_IF_FREQ_L,            0x71},
    {0x12, 0x00, 0x00},  /* 特殊コマンド: offset調整 */
	{SPI_ADR_RSSI_LPF_ADJ,         0x1F},
	/* BANK2 */
	{0x80|2,0x0E,                  0x24}, /* 非公開レジスタ */
    {0xff,0xff,0xff}  /* end of table */
};
static REG_TABLE reg_txrate100[] = {
    {SPI_ADR_IF_FREQ_AFC_H,        0x14},
    {SPI_ADR_IF_FREQ_AFC_L,        0x7a},
    {SPI_ADR_BPF_AFC_ADJ_H,        0x02},  /* 後でoffset調整 */
    {SPI_ADR_BPF_AFC_ADJ_L,        0x4a},  /* 後でoffset調整 */
    {SPI_ADR_TX_PR_LEN,            0x04},  /* 0x04以上 */
    {SPI_ADR_DATA_SET,             0x11},
    {SPI_ADR_CH_SPACE_L,           0x82},
    {SPI_ADR_CH_SPACE_H,           0x2d},
    {SPI_ADR_F_DEV_L,              0xb0},
    {SPI_ADR_F_DEV_H,              0x05},
    {SPI_ADR_2DIV_SEARCH,          0x16},  /* 0x16以上 */
    {SPI_ADR_PLL_CP_ADJ,           0x44},
    {SPI_ADR_IF_FREQ_H,            0x14},
    {SPI_ADR_IF_FREQ_L,            0x7a},
    {SPI_ADR_IF_FREQ_CCA_H,        0x1c},
    {SPI_ADR_IF_FREQ_CCA_L,        0x71},
    {0x12, 0x00, 0x00},  /* 特殊コマンド: offset調整 */
    {SPI_ADR_RSSI_LPF_ADJ,         0x1f},
    {0x80|2,0x0e,                  0x27},  /* 隠しレジスタ */
    {0xff,0xff,0xff}  /* end of table */
};

// SPI interrupt paramter & handler
static wait_queue_head_t irq_q;
static irqreturn_t bp3596_irq_handler(int irq, void *dev_id) {
	// send event
	finish_flag = 1;
	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("[PHY-BP3596] interrupt irq_q\n"));
	wake_up_interruptible( &irq_q );
	return IRQ_WAKE_THREAD;
}

/**
 * GPIO API
 */
static int gpio_configure(int pin, int mode) {
	int err = 0;
	char buf[20];

	sprintf(buf, "GPIO_%02d", pin);
	if ((err = gpio_request(pin, buf)) < 0) {
		printk(KERN_ERR " gpio_request ERROR pin=%d err=%d", pin,err );
		err = -EFAULT;
	}
    switch (mode) {
    case GPIO_INPUT:
		if ((err = gpio_direction_input(pin)) < 0) {
			printk(KERN_ERR " gpio_direction_input ERROR %d", err );
			err = -EFAULT;
		}
        break;
    case GPIO_OUTPUT:
		if ((err = gpio_direction_output(pin, 0)) < 0) {
			printk(KERN_ERR " gpio_direction_output ERROR %d", err );
			err = -EFAULT;
		}
        break;
    default:
		err = -EFAULT;
    }
	return err;
}
void gpio_configures( void ) {
    gpio_configure(GPIO_RESETN,GPIO_OUTPUT);
    gpio_configure(GPIO_DMONI, GPIO_INPUT);
    gpio_configure(GPIO_SINTN, GPIO_INPUT);
#ifdef GPIO_TX_LED
	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("TX LED Init\n"));	// for debug
    gpio_configure(GPIO_TX_LED, GPIO_OUTPUT);
	gpio_set_value(GPIO_TX_LED, 1);
#endif
#ifdef GPIO_RX_LED
	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("RX LED Init\n"));	// for debug
    gpio_configure(GPIO_RX_LED, GPIO_OUTPUT);
	gpio_set_value(GPIO_RX_LED, 1);
#endif
#ifdef GPIO_CCA_ERR
    gpio_configure(GPIO_CCA_ERR, GPIO_OUTPUT);
	gpio_set_value(GPIO_CCA_ERR, 0);
#endif
}
void bp_reset( void ) {
	gpio_set_value(GPIO_RESETN, 0);
    msleep(SPI_RESET_WAIT);
	gpio_set_value(GPIO_RESETN, 1);
    msleep(SPI_RESET_WAIT);
}

void bp_term(void) {
	gpio_set_value(GPIO_RESETN, 0);
}

int bp_phy_reset(void) {
    uint8_t buffer[1];   /* SPIA÷?o?R\D\A\O\! */
    int status = PHY_OK;

	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk( "[PHY-BP3596] PHY Reset process executed\r\n" ));

	/* TRX OFF */
	buffer[0] = 0x03;  /*  Force_TRX_OFF */
	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk( "[PHY-BP3596] TRX OFF\r\n" ));
	ONERRORGOTO(bp_spi_write(SPI_ADR_RF_STATUS, buffer, 1));

	do {
		ONERRORGOTO(bp_spi_read(SPI_ADR_RF_STATUS, buffer, 1));
	}while( !((buffer[0] & 0xF0) == 0x80 ) );

	// INT3 clear
	buffer[0] = 0;
	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk( "[PHY-BP3596] INT_SOURCE_GRP3 Clear\r\n" ));
	ONERRORGOTO(bp_spi_write(SPI_ADR_INT_SOURCE_GRP3, buffer, 1));

	/* PHY RESET */
	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk( "[PHY-BP3596] PHY Reset\r\n" ));
	buffer[0] = 0x88;
	ONERRORGOTO(bp_spi_write(SPI_ADR_RST_SET, buffer, 1));

	buffer[0] = 0;
	ONERRORGOTO(bp_spi_write(SPI_ADR_INT_SOURCE_GRP2, buffer, 1));

	buffer[0] = 0;
	ONERRORGOTO(bp_spi_write(SPI_ADR_INT_SOURCE_GRP3, buffer, 1));

	// FIFO Clear added By Naotaka Saito
	buffer[0] = 0xc0;
	ONERRORGOTO(bp_spi_write(SPI_ADR_INT_SOURCE_GRP1, buffer, 1));

	/* RX_ON */
	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk( "[PHY-BP3596] TX ON\r\n" ));
	buffer[0] = 0x06;
	ONERRORGOTO(bp_spi_write(SPI_ADR_RF_STATUS, buffer, 1));

	do {
		ONERRORGOTO(bp_spi_read(SPI_ADR_RF_STATUS, buffer, 1));
	} while( !((buffer[0] & 0xF0) == 0x60 ) );

error:
	return status;
}
static int bp_settxpow(uint8_t txpower) {
    int status = PHY_ERR_UNKNOWN;
    uint8_t buffer[1];
    uint8_t data8;
	if (txpower == TXPOWER_20MW) {
		/* 20mW */
		buffer[0] = bp_read_eeprom(0x2A);
	} else {
		/* 1mW */
		buffer[0] = bp_read_eeprom(0x2C);
	}
    ONERRORGOTO(bp_spi_write(SPI_ADR_PA_REG_FINE_ADJ, buffer, 1));
	/* PA_CNTRL bit4 0=1mW 1=20mW */
    ONERRORGOTO(bp_spi_read(SPI_ADR_PA_CNTRL, (uint8_t*)&data8, 1));
	data8 &= ~(0x13);
	data8 |= txpower;
    ONERRORGOTO(bp_spi_write(SPI_ADR_PA_CNTRL, (uint8_t*)&data8, 1));
    status = 0;

error:
	return status;
}


static int bp_reg_setup(const REG_TABLE *reg) {
    int status = PHY_ERR_UNKNOWN;
    uint8_t buffer[1];
    uint8_t data8;

    for (; reg->bank != 0xff; ++reg)
        switch (reg->bank) {
        case 0:  /* Bank0 のレジスタへ書き込む */
        case 1:  /* Bank1 のレジスタへ書き込む */
        case 2:  /* Bank2 のレジスタへ書き込む */
            ONERRORGOTO(bp_spi_write(reg->bank, reg->address, &reg->value, 1));
            break;
        case 0x10:  /* クロック安定割り込み待ち */
		{
			uint32_t count = 0;
            do {
				count++;
				if(count > 100000) {
					printk("[PHY-BP3596]: Error CLOCK  steady IRQ failed\n");
					break;
				}
				ONERRORGOTO(bp_spi_read(SPI_ADR_INT_SOURCE_GRP1, buffer, 1));
            } while(!(buffer[0] & 0x01));
            break;
		}
        case 0x11:  /* VCOキャリブレーション完了待ち */
		{
			uint32_t count = 0;
			do {
				count++;
				if(count > 100000) {
					printk("[PHY-BP3596]: Error VCO calibration failed\n");
					break;
				}
				ONERRORGOTO(bp_spi_read(SPI_ADR_INT_SOURCE_GRP1, buffer, 1));
			} while(!(buffer[0] & 0x04));
		}
		buffer[0] = ~0x04;
		ONERRORGOTO(bp_spi_write(SPI_ADR_INT_SOURCE_GRP1, buffer, 1));
		break;
        case 0x12:  /* BPF帯域の設定(p161) */
		{
			#define BPF_NORMAL	0
			#define BPF_CCA		1
			struct _bpf_table {
				uint16_t ratekbps;
				uint8_t  rate;    /* SPI_ADR_DATA_SETレジスタのbit[1:0] */
				struct _type {
					uint32_t   coef; /* 係数 */
					uint16_t ref;  /* 基準値 */
				} type[2];
			} bpf_table[]={
				{  10, 0x00, {{1440, 0x034B}, { 1440, 0x034B}} },
				{  20, 0x00, {{ 923, 0x021C}, {  923, 0x021C}} },
				{  40, 0x00, {{ 758, 0x01BC}, {  758, 0x01BC}} },
				{  50, 0x00, {{1440, 0x034B}, { 1440, 0x034B}} },
				{ 100, 0x01, {{1000, 0x024A}, {  480, 0x0119}} },
				{ 150, 0x02, {{ 800, 0x01D4}, {  497, 0x0122}} },
				{ 200, 0x02, {{ 554, 0x0144}, {  360, 0x00D2}} },
				{ 400, 0x03, {{ 343, 0x00C8}, {  343, 0x00C8}} },
			};
			#define BPF_TABLE_MAX (sizeof( bpf_table ) / sizeof(struct _bpf_table))
			int i;
			uint8_t bpf;
			//uint16_t ratekbps = WSN_DEFPARAM_BP_DEFRATE;
			uint16_t ratekbps = phy_param.bitrate;
			uint32_t adj;
			uint32_t cca_adj;
			//uint16_t tmp = WSN_DEFPARAM_BP_DEFRATE;
			uint16_t tmp = phy_param.bitrate;
			if( tmp )
				ratekbps = tmp;
			ONERRORGOTO(bp_spi_read(SPI_ADR_BPF_ADJ_OFFSET, &bpf, 1));
			for (i = 0; i < BPF_TABLE_MAX; i++ ) {
				if (ratekbps == bpf_table[i].ratekbps) {
					uint8_t tmp;
					/*  NOURMAL & AFC */
					adj = bpf_table[i].type[BPF_NORMAL].ref;
					if (bpf & 0x80) {
						uint32_t adj_tmp=0;
						adj_tmp = ((bpf_table[i].type[BPF_NORMAL].coef * (bpf & 0x7f)) / 1000);
						adj += adj_tmp;
					} else {
						uint32_t adj_tmp=0;
						adj_tmp = ((bpf_table[i].type[BPF_NORMAL].coef * (bpf & 0x7f)) / 1000);
						adj -= adj_tmp;
					}
					tmp = ((uint16_t)adj >> 8);
					ONERRORGOTO(bp_spi_write(SPI_ADR_BPF_ADJ_H, &tmp, 1));
					ONERRORGOTO(bp_spi_write(SPI_ADR_BPF_AFC_ADJ_H, &tmp, 1));

					tmp = ((uint16_t)adj & 0xff);
					ONERRORGOTO(bp_spi_write(SPI_ADR_BPF_ADJ_L, &tmp, 1));
					ONERRORGOTO(bp_spi_write(SPI_ADR_BPF_AFC_ADJ_L, &tmp, 1));

					/*  CCA */
					cca_adj = bpf_table[i].type[BPF_CCA].ref;
					if (bpf & 0x80) {
						uint32_t cca_tmp=0;
						cca_tmp = ((bpf_table[i].type[BPF_CCA].coef * (bpf & 0x7f)) / 1000);
						cca_adj += cca_tmp;
					} else {
						uint32_t cca_tmp=0;
						cca_tmp = ((bpf_table[i].type[BPF_CCA].coef * (bpf & 0x7f)) / 1000);
						cca_adj -= cca_tmp;
					}
					tmp = ((uint16_t)cca_adj >> 8);
					ONERRORGOTO(bp_spi_write(SPI_ADR_BPF_CCA_ADJ_H, &tmp, 1));

					tmp = ((uint16_t)cca_adj & 0xff);
					ONERRORGOTO(bp_spi_write(SPI_ADR_BPF_CCA_ADJ_L, &tmp, 1));

					break;
				}
			}
			if( i >= BPF_TABLE_MAX ){
				printk( "CTI: %s(%d:) ratekbps=0x%X \r\n", __FUNCTION__, __LINE__, ratekbps );
				break;
			}
		}
		break;
        case 0x13:  /* power調整(デフォルト20mW) */
		{
			/* 20mW粗調整 */
			buffer[0] = bp_read_eeprom(0x29);
			ONERRORGOTO(bp_spi_write(SPI_ADR_PA_ADJ3, buffer, 1));

			/* 1mW粗調整 */
			buffer[0] = bp_read_eeprom(0x2B);
			ONERRORGOTO(bp_spi_write(SPI_ADR_PA_ADJ1, buffer, 1));

			/* XA */
			buffer[0] = bp_read_eeprom(0x2D);
			ONERRORGOTO(bp_spi_write(SPI_ADR_OSC_ADJ, buffer, 1));

//		Changed by Naotaka Saito
			if(phy_param.tx_pwr == 20)
			{
				bp_settxpow( TXPOWER_20MW );
				DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("[PHY-BP3596] setpow 20mW\n"));			// for debug
			}
			else
			{
				bp_settxpow( TXPOWER_1MW );
				DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("[PHY-BP3596] setpow 1mW\n"));			// for debug
			}
//			bp_settxpow( reg->value );		Naotaka Saito May,7,2015
		}
		break;
		case 0x14:  /* 規格設定 */
			/* IEEE_MODE */
			data8 = 0x1b; /* WHITENING,ED_NOTICE,IEEE_MODE,ADDFIL_IDLE_DET */
    		ONERRORGOTO(bp_spi_write(SPI_ADR_PACKET_MODE_SET, &data8, 1));
			if (data8 & 0x08)
				bp.param[BP_PARAM_ED] = 1;
			if (data8 & 0x02)
				bp.param[BP_PARAM_IEEE15_4g] = 1;
			/* CRC */
			// Naotaka Saito Test
			// 2015.08.17 Eiichi Saito
  			data8 = 0x0b; /* CRC_EN, CRC16, CRC_DONE */
//       	data8 = 0x03; /* CRC_EN, CRC16, CRC_DONE */
    		ONERRORGOTO(bp_spi_write(SPI_ADR_FEC_CRC_SET, &data8, 1));

			if (((data8 >> 1) & 0x03) & 0x01) { /* CRC16 or CRC-IBM */
				bp.param[BP_PARAM_TXCRC] = 2;
				bp.param[BP_PARAM_RXCRC] = 2;
			} else if (((data8 >> 1) & 0x03) == 0x02) {
				bp.param[BP_PARAM_TXCRC] = 4;
				bp.param[BP_PARAM_RXCRC] = 4;
			} else {
				bp.param[BP_PARAM_TXCRC] = 1;
				bp.param[BP_PARAM_RXCRC] = 1;
			}
			if (!(data8 & 0x08)) {
				/* FCS 情報からCRCを自動で計算する */
				bp.param[BP_PARAM_RXCRC] = 2;
			}
            break;
		case 0x15:  /* レート設定 */
		{
			REG_TABLE *reg;
			// uint16_t ratekbps = WSN_DEFPARAM_BP_DEFRATE;
			uint16_t ratekbps = phy_param.bitrate;
			switch (ratekbps) {
			case 100:
				reg = reg_txrate100;
				break;
			case 50:
				reg = reg_txrate50;
				break;
			default:
				return -1;
			}
			if (ISERROR(status = bp_reg_setup(reg))) {
				bp_term();
				status = PHY_ERR_PARAMETER;
				goto error;
			}
		}
		break;
        case 0x40:  /* Bank0 のレジスタへ設定パラメータを書き込む */
        case 0x41:  /* Bank1 のレジスタへ設定パラメータを書き込む */
        case 0x42:  /* Bank2 のレジスタへ設定パラメータを書き込む */
            ONERRORGOTO(bp_spi_write(reg->bank & 0x0f, reg->address, (uint8_t *)&bp.param[reg->value], 1));
            break;
        case 0x80:  /* Bank0 のレジスタへI2C読み出しデータを書き込む */
        case 0x81:  /* Bank1 のレジスタへI2C読み出しデータを書き込む */
        case 0x82:  /* Bank2 のレジスタへI2C読み出しデータを書き込む */
            ONERRORGOTO(bp_spi_write(reg->bank, reg->address, &reg->value, 1));
            break;
        default:  /* その他の値はパラメータエラー */
            status = PHY_ERR_PARAMETER;
            goto error;
        }
    status = 0;
error:
    return status;
}

/* 周波数パラメータ設定 */
static uint32_t bp_set_freq(uint32_t freq) {
    int status = PHY_ERR_UNKNOWN;
    /* 内部計算は全て1.0を1MHzとするE20フォーマットで行う */
	/* (固定小数点数値: 上位12ビットが整数部, 下位20ビットが小数部) */
    uint32_t f_ch0;
    uint32_t f_min;

    f_ch0 = freq;
    if (f_ch0 < (uint32_t)750 << 20 || f_ch0 > (uint32_t)1000 << 20) {
        status = PHY_ERR_PARAMETER;
        goto error;
    }
    f_min = f_ch0 - ((uint32_t)2 << 20);  /* CH0周波数より2MHz低い値をキャリブレーション下限値とする */
    f_ch0 /= 36;
    f_min /= 36;
    if (f_min >> 20 != f_ch0 >> 20) {  /* 36MHz 境界を跨ぐ設定は無効 */
        status = PHY_ERR_PARAMETER;
        goto error;
    }
    bp.param[BP_PARAM_CH0_NA] = (f_ch0 >> 18 & 0xf0) | (f_ch0 >> 20 & 0x03);
    bp.param[BP_PARAM_CH0_FH] = f_ch0 >> 16 & 0x0f;
    bp.param[BP_PARAM_CH0_FM] = f_ch0 >>  8 & 0xff;
    bp.param[BP_PARAM_CH0_FL] = f_ch0 >>  0 & 0xff;
    bp.param[BP_PARAM_MIN_FH] = f_min >> 16 & 0x0f;
    bp.param[BP_PARAM_MIN_FM] = f_min >>  8 & 0xff;
    bp.param[BP_PARAM_MIN_FL] = f_min >>  0 & 0xff;

	ONERRORGOTO(bp_spi_write(SPI_ADR_CH0_FL, (uint8_t *)&bp.param[BP_PARAM_CH0_FL], 1 ));
	ONERRORGOTO(bp_spi_write(SPI_ADR_CH0_FM, (uint8_t *)&bp.param[BP_PARAM_CH0_FM], 1 ));
	ONERRORGOTO(bp_spi_write(SPI_ADR_CH0_FH, (uint8_t *)&bp.param[BP_PARAM_CH0_FH], 1 ));
	ONERRORGOTO(bp_spi_write(SPI_ADR_CH0_NA, (uint8_t *)&bp.param[BP_PARAM_CH0_NA], 1 ));

    status = 0;
error:
    return status;
}

/* チャネル番号を周波数に変換 */
static uint32_t bp_freq(int ch, uint8_t rate) {
    uint32_t freq = 0;
	uint32_t base;
    /* freq = 916.00 + (ch-1)/5    when ch =  1..61
              928.15 + (ch-62)/10  when ch = 62..77
              0                    otherwise (invalid channel) */
	if (rate==100)
		base = 9159;
	else if (rate==50)
		base = 9158;
	else
		base = 0;
    if (ch < 24 || ch > 61 || base == 0)
        goto error;
    freq = ((ch << 1) + base) * ((uint32_t)1<<18) / 10;
error:
    return freq << 2;
}

// ml7396_init(originally bp_init)

int ml7396_init(void) {
    int status = SPI_ERR_UNKNOWN;

    bp.param[BP_PARAM_DEBUG] = WSN_DEFPARAM_DEBUG;

	/* 周波数設定 */
    ONERRORGOTO(bp_set_freq(bp_freq(phy_param.ch, phy_param.bitrate)));
	/* GPIO接続設定 */
	gpio_configures();
	/* リセットパルス生成 */
	bp_reset();
	
	
	/* レジスタ値を初期化 */
    if (ISERROR(status = bp_reg_setup(reg_init))) {
        bp_term();
        goto error;
    }
    status = 0;
    bp.param[BP_PARAM_INIT] = 1;



error:
    return status;
}

static int bp_idle(void) {
    /* dummy */
	return 0;
}

/* BP3596 RF設定
 * rf: RF設定値
 * 戻り値: 0=正常終了, 負数=異常終了
 */
static int bp_setrf( uint8_t rf ) {
	uint8_t value;
	int hang=0;

	if ((rf != SET_TRX_FORCEOFF) &&
	    (rf != SET_TRX_RXON) &&
	    (rf != SET_TRX_TRXOFF) &&
	    (rf != SET_TRX_TXON)) {
		return PHY_ERR_PARAMETER;
	}

    bp_spi_read(SPI_ADR_RF_STATUS, &value, 1);

	switch (rf) {
	case SET_TRX_FORCEOFF:
	case SET_TRX_TRXOFF:
		if ((value & 0xf0) == GET_TRX_TRXOFF)
			return 0;
		break;
	case SET_TRX_RXON:
		if ((value & 0xf0) == GET_TRX_RXON)
			return 0;
		break;
	case SET_TRX_TXON:
		if ((value & 0xf0) == GET_TRX_TXON)
			return 0;
		break;
	}
    value = rf;
    bp_spi_write(SPI_ADR_RF_STATUS, &value, 1);

	if (rf == SET_TRX_FORCEOFF)
		rf = SET_TRX_TRXOFF;

    do {
		hang++;
        bp_idle();
        bp_spi_read(SPI_ADR_RF_STATUS, &value, 1);
        if( hang > 1000000 ) {
			printk( "%s(%d:) ERROR : RF_STATUS hangup!! (0x%02X)\r\n", __FUNCTION__, __LINE__, value  );
			return SPI_ERR_UNKNOWN;
		}
    } while(!(value & (rf << 4)));
	return 0;
}

static int bp_cca( void ) {
    int status = SPI_ERR_UNKNOWN;
	uint8_t value = 0;
    uint8_t buffer[1];
    uint8_t intstat;    /* 割り込みフラグ */
	int retrycnt=1;
	int dbgcnt=0;

	/* Force_TRX_OFF */
	ONERRORGOTO(bp_setrf( SET_TRX_FORCEOFF ));
	/* RX ON */
#ifdef	GPIO_CCA_ERR
		gpio_set_value(GPIO_CCA_ERR, 1);
#endif
	ONERRORGOTO(bp_setrf( SET_TRX_RXON ));
#ifdef	GPIO_CCA_ERR
		gpio_set_value(GPIO_CCA_ERR, 0);
#endif
	/* CCA_EN on */
	value = 0x10;
	ONERRORGOTO(bp_spi_write(SPI_ADR_CCA_CNTRL, &value, 1));
	do {
#ifdef	GPIO_CCA_ERR
		gpio_set_value(GPIO_CCA_ERR, 1);
#endif
		// CCA_EN クリア待ち
	    do {
			dbgcnt++;
			// 2015.08.21 Eiichi Saito CCA wait
	        // bp_idle();
			udelay(128);
	        ONERRORGOTO(bp_spi_read(SPI_ADR_CCA_CNTRL, &value, 1));
//			DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("[PHY-BP3596] CCA_CTRL=0x%02x\n",value));			// for debug
	        if (dbgcnt > 1000000) { 
	        	printk( "[CCA_EN] hang up!! \r\n" );
				status = PHY_ERR_CCA;	// Naotaka Saito May,8,2015j
//				status = 1;
//				drv_mode = 0xF000;
//	        	printk( "[CCA_EN] change debug mode = %04x\r\n",drv_mode );
	        	goto error;
	        }
	    } while(value & 0x10);
#ifdef	GPIO_CCA_ERR
		gpio_set_value(GPIO_CCA_ERR, 0);
#endif
		/* CCA 完了割り込み待ち */
	    do {
			dbgcnt++;
	        bp_idle();
	        ONERRORGOTO(bp_spi_read(SPI_ADR_INT_SOURCE_GRP2, &intstat, 1));
	        if (dbgcnt > 1000000) {
	        	printk( "[GRP2 CCA_DONE] hang up!! \r\n" );
				status = PHY_ERR_CCA;	// Naotaka Saito May,8,2015j
//				status = 1;		// Naotaka Saito May,8,2015
	        	printk( "[GRP2 CCA DONE] change debug mode = %04x\r\n",drv_mode );
	        	goto error;
	        }
	    } while(!(intstat & 0x01));
		/* CCA_EN on */
		bp_spi_read(SPI_ADR_CCA_CNTRL, &value, 1);
		/* CCA 完了割り込みクリア */
		buffer[0] = ~(intstat&0x01);
		bp_spi_write(SPI_ADR_INT_SOURCE_GRP2, buffer, 1);

		if ((value & 0x3) != 0) {
			retrycnt--;
			value = 0x00;
			bp_spi_write(SPI_ADR_CCA_CNTRL, &value, 1);
			/* CCA_EN on */
			value = (1<<4);
			ONERRORGOTO(bp_spi_write(SPI_ADR_CCA_CNTRL, &value, 1));
			/* 本来ウェイトは不要 */
			/* 現状 CCA開始命令後、割り込みステータスGRP2を直ぐに参照すると、CCA完了要因がONしない */
			/* また CCA_CNTRL の CCA_ENビットもONのままになる。本来CCA完了後自動でOFFされる。 */
			/* 処置として、以下のウェイトを挿入して回避 */
			udelay( 1000 );
			continue;
		}
		break;
	} while( retrycnt != 0 );
	/* Force_TRX_OFF */
	ONERRORGOTO(bp_setrf( SET_TRX_FORCEOFF ));

	if (retrycnt == 0)
		status = 1;
	else
		status = 0;

error:
	return status;
}

/* べき乗 */
static int power(int base, int n) {
	int i, p;
	p = 1;
	for (i = 1; i <= n; ++i)
		p = p * base;
	return p;
}

/* 乱数 */
static unsigned long xor128(void) {
    static unsigned long x = 123456789, y = 362436069, z = 521288629, w = 88675123;
    unsigned long t;
    t = (x ^(x << 11));
	x = y;
	y = z;
	z = w;
    return (w = (w ^(w >> 19)) ^(t ^(t >> 8)));
}

/* BP3596 パケット送信処理(PHYレイヤ)
 * data: 送信するパケットデータ
 * length: dataのバイトサイズ
 * 戻り値: 正数=送信したバイト数, 負数=異常終了
 */
int bp_send_phy(const uint8_t *data, uint16_t length, uint8_t ack) {
    int status = SPI_ERR_UNKNOWN;
    static const uint8_t intmask = 0xc3;    /* 割り込みマスク */
    uint8_t intstat;                        /* 割り込みフラグ */
    uint8_t buffer[1];                      /* SPI送受信バッファ */
// 2015.08.18 Eiichi Saito
	uint8_t frame_type	= data[0]&0x03;		/* Frame Type 取得 */

//  MACHeader *mhr = (MACHeader *)data;     /* MACHeader */
	uint16_t fcs = ( 0<<15 | 1<<12 | 1<<11 ); /* MS=0,LEN=0,Whit=1 */
    intstat = 0;

#ifdef GPIO_TX_LED
	finish_flag = 1;
	wake_up_interruptible( &tx_led_q );		// flash tx led
#endif

	if (bp.param[BP_PARAM_IEEE15_4g]) {
		uint16_t length_be16;    /* フォーマットの送受信サイズ(big endian) */
		/* FIFOに送信バイト数を書き込む(2byte) */
    	length_be16 = cpu_to_be16( fcs | (length + bp.param[BP_PARAM_TXCRC]));
    	ONERRORGOTO(bp_spi_write(SPI_ADR_WR_TX_FIFO, (uint8_t *)&length_be16, sizeof(length_be16)));
	} else {
		uint8_t  length_be8;
		/* FIFOに送信バイト数を書き込む(1byte) */
	    length_be8 = (length + bp.param[BP_PARAM_TXCRC]);
	    ONERRORGOTO(bp_spi_write(SPI_ADR_WR_TX_FIFO, (uint8_t *)&length_be8, sizeof(length_be8)));
	}
	//printk("4G=%02x, length = %04x, TXCRC=%02x\n",bp.param[BP_PARAM_IEEE15_4g],length, bp.param[BP_PARAM_TXCRC] );

	/* FIFOに送信データを書き込む */
    ONERRORGOTO(bp_spi_write(SPI_ADR_WR_TX_FIFO, data, length));
	// PAYLOADDUMP(data,length);

	/* FIFOデータ送信要求 受け付け完了割り込み待ち */
	{
		uint32_t count = 0;
		do {
			count++;
			if(count > 100000) {
				printk("[Error]: FIFO date send req compleate IRQ failed\n");
				break;
			}
			bp_idle();
			/* FIFOに送信データを書き込む */
			ONERRORGOTO(bp_spi_read(SPI_ADR_INT_SOURCE_GRP3, buffer, 1));
			intstat = buffer[0] & intmask;
		} while(!(intstat & 0xC0));
	}

// if (!ack) {
// if (WSN_DEFPARAM_BP_CCA) {
// 2015.08.18 Eiichi Saito : Data & Command Frame only 
   if (frame_type&0x01) {
		uint16_t backoff;	/* バックオフ時間 */
		uint16_t ifs = 0;	/* IFS時間 */
		uint8_t nr = 0;		/* フレーム再送回数 */
		uint8_t nb;			/* バックオフ回数 */
		uint8_t be;			/* バックオフ指数 */

frame_retry:
		nb = 0;			/* バックオフ回数 */
		be = MACMINBE;	/* バックオフ指数 */


		if (ack == true) {
			// ack
			ifs = SIFS;
		} else {
			ifs = LIFS;
			// IFS待ち
			udelay( ifs );
		}

		/*
		if (mhr->frameControl & 0x20) {
			// ack
			ifs = SIFS;
		} else {
			ifs = LIFS;
			// IFS待ち
			udelay( ifs );
		}
		*/

backoff_retry:
		/* バックオフ設定 */
		if (ack == true) {
//		if (mhr->frameControl & 0x20) {
			backoff = 1;
		} else {
			backoff = BACKOFF( (power(2, be)-1) );
		}
		// 2015.08.21 Eiichi Saito change IDLE_WAIT for ARIB
		/*
		buffer[0] = ((backoff>>8) & 0xff);
		bp_spi_write(SPI_ADR_IDLE_WAIT_L, buffer, 1);
		buffer[0] = (backoff & 0xff);
		bp_spi_write(SPI_ADR_IDLE_WAIT_H, buffer, 1);
		*/
		if (phy_param.ch <= 32) {
			buffer[0] = 0x01;
			bp_spi_write(SPI_ADR_IDLE_WAIT_H, buffer, 1);
			buffer[0] = 0x18;
			bp_spi_write(SPI_ADR_IDLE_WAIT_L, buffer, 1);
		}else
		if (phy_param.ch <= 61) {
			buffer[0] = 0x00;
			bp_spi_write(SPI_ADR_IDLE_WAIT_H, buffer, 1);
			buffer[0] = 0x00;
			bp_spi_write(SPI_ADR_IDLE_WAIT_L, buffer, 1);
		}

		status = bp_cca();
		if( status < 0)			// CCA Error check Naotaka Saito May,8, 2015
		{						// Naotaka Saito May,8, 2015
			ml7396_init();		// Naotaka Saito May,8, 2015
			DEBUGONDISPLAY(MODE_TEST, printk("[PHY-BP3596] EXIT CCA error\n"));			// for debug
			goto error; 		// Naotaka Saito May,8, 2015
		}
		if (status != 0) {
			/* busy */
			nb++;
			be = MIN( be+1, MACMAXBE );
			/* バックオフ回数オーバー？*/
			if (nb < MACMAXCSMABACKOFFS)
				goto backoff_retry;

			/* ユニキャストフレーム？ */
//			if (mhr->destionationAddress == 0xffff)
//				goto error;
//
			/* 再送回数インクリメント */
			nr++;

			/* 再送回数オーバー？*/
			if (nr > WSN_DEFPARAM_BP_CCA){
				printk( "%s(%d:) TX RETRY OVER.\n\n", __FUNCTION__, __LINE__  );
				goto error;
			}
			/* 再送 */
			goto frame_retry;
		}
	}

	/* 送信開始 */
    ONERRORGOTO(bp_setrf(SET_TRX_TXON));

	/* 送信完了待ち */
	{
		uint32_t count = 0;
		do {
			count++;
			if (count > 100000) {
				printk("[Error]: Send compleate IRQ failed\n");
				goto send_irq_failed;
			}
			bp_idle();
			ONERRORGOTO(bp_spi_read(SPI_ADR_INT_SOURCE_GRP3, buffer, 1));
			intstat = buffer[0] & intmask;
		} while(!(intstat & 0x03));
		DEBUGONDISPLAY(MODE_TEST,printk("[PHY]: SEND ACK\n"));
send_irq_failed:
		status = length;
	}

error:
	/* 送信完了割り込みフラグクリア */
    buffer[0] = ~intstat;
    bp_spi_write(SPI_ADR_INT_SOURCE_GRP3, buffer, 1);

	/* TRX_OFF */
	bp_setrf( SET_TRX_RXON );

	if (status == length) {
		/* 送信フレームのダンプ */
		if (bp.param[BP_PARAM_DEBUG] & BP_DEBUG_TXDUMP) {
			printk( "%s(%d:) SEND PACKET!! \r\n", __FUNCTION__, __LINE__  );
         // PAYLOADDUMP( data, length );
		}
	}

    return status;
}

/* BP3596 パケット受信処理(PHYレイヤ) (Blocking/Non Blocking)
 * data: 受信データ収納領域
 * size: dataのバイトサイズ
 * noblock: 0=Blocking 1=Non Blocking
 * 戻り値: 正数=受信したバイト数, 0=受信データ無し、負数=異常終了
 */
static int bp_recv_phy(uint8_t *data, uint16_t size, int noblock) {
    int status = PHY_ERR_UNKNOWN;
    uint8_t intgrp[5];     /* 割り込みフラグ [0] dummy GRP#1-4*/
    int length=0;     /* フォーマットの送受信サイズ */
    uint8_t buffer[1];   /* SPI送受信バッファ */
//	static uint8_t rev_seqno = 0xa5;
    //MACHeader *mhr = (MACHeader *)data;

    do {
		/* データ受信完了割り込みあり？ */
		if (bp.param[BP_PARAM_IEEE15_4g]) {
	    	uint16_t length_be16=0;
			/* FIFOから受信バイト数を読み出す */
		    ONERRORGOTO(bp_spi_read(SPI_ADR_RD_RX_FIFO, (uint8_t *)&length_be16, sizeof(length_be16)));
		    length = (be16_to_cpu(length_be16) & 0x7ff);
	    	 length -= bp.param[BP_PARAM_RXCRC]; /* CRCを除く */
			DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("[PHY-BP3596] 4g length=%d, be16=%d, param=%d\n", length,(be16_to_cpu(length_be16)&0x07ff),bp.param[BP_PARAM_RXCRC]));			// for debug
		} else {
	    	uint8_t length_be8=0;  /* フォーマットの送受信サイズ(big endian) */
			/* FIFOから受信バイト数を読み出す */
		    ONERRORGOTO(bp_spi_read(SPI_ADR_RD_RX_FIFO, (uint8_t *)&length_be8, sizeof(length_be8)));
	    	length = length_be8 - bp.param[BP_PARAM_RXCRC]; /* CRCを除く */
			DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("[PHY-BP3596] length=%d, be8=%d, param=%d\n", length,length_be8,bp.param[BP_PARAM_RXCRC]));			// for debug
		}
	    if (length <= 0) {
			length = 0;
			bp_phy_reset();
			status = -1;
			goto error;
		}

		// read length check.
		//	if length > FIFO size, phy_reset is done in main process.
		//  if FIFO size >= length > 251, 251 byte is read. last 5 bytes is deleted.
		if (bp.param[BP_PARAM_ED])
		{
			// RSSI option is valid
		    if ( size > (length + 1) )
			{
		        size = length;
			}
			else
			{
				DEBUGONDISPLAY(MODE_TEST, printk("[PHY-BP3596] length over:%d,load size: %d\n", length+1, size));
				bp_phy_reset();
				status = -1;
				goto error;
			}
		} else
		{
			// RSSI option is invalid
		    if ( size > length )
			{
		        size = length;
			}
			else
			{
				DEBUGONDISPLAY(MODE_TEST, printk("[PHY-BP3596] length over:%d,load size: %d\n", length, size));
				bp_phy_reset();
				status = -1;
				goto error;
			}
		}

		/* 受信データのFIFO読み出し */
	    ONERRORGOTO(bp_spi_read(SPI_ADR_RD_RX_FIFO, data, size)); /* PSDU */
		DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("[PHY-BP3596] PSDU load\n"));			// for debug
		DEBUGONDISPLAY(MODE_PHY_DEBUG, PAYLOADDUMP( data, size ));					// for debug
		
	    ONERRORGOTO(bp_spi_read(SPI_ADR_RD_RX_FIFO, NULL, bp.param[BP_PARAM_RXCRC])); /* CRC */
		if (bp.param[BP_PARAM_ED]){
		    ONERRORGOTO(bp_spi_read(SPI_ADR_RD_RX_FIFO, &data[size], 1)); /* ED(1) */
		    length += 1;
	        size = length;
		}

		if (!(bp.param[BP_PARAM_IEEE15_4g])) {
			/* 無効データの空読み(FIFO面を正常に切り替える為らしい...) */
	    	ONERRORGOTO(bp_spi_read(SPI_ADR_RD_RX_FIFO, NULL, 1));
		}

		/* 受信完了割り込みフラグクリア */
		/* FIFO切り替わり */
		bp_spi_read(SPI_ADR_INT_SOURCE_GRP3, &intgrp[3], 1);
    	buffer[0] = ~(intgrp[3] & 0x3C );
	    ONERRORGOTO(bp_spi_write(SPI_ADR_INT_SOURCE_GRP3, buffer, 1));

		status = length;

		DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("[PHY-BP3596] RECV PHY Payload dump\n"));	// for debug
		DEBUGONDISPLAY(MODE_PHY_DEBUG, PAYLOADDUMP( data, size ));					// for debug

		if (bp.param[BP_PARAM_DEBUG] & BP_DEBUG_RXDUMP) {
			DEBUGONDISPLAY(MODE_PHY_DEBUG, printk( "%s(%d:) RECV PACKET!! \r\n", __FUNCTION__, __LINE__  ));
			DEBUGONDISPLAY(MODE_PHY_DEBUG, PAYLOADDUMP( data, size ));
		}
		break;
	}while(1);

error:

	return status;
}


/* BP3596 パケット受信処理(MACレイヤ) (Non Blocking)
 * panid: PAN_ID
 * data: 受信データ収納領域
 * size: dataのバイトサイズ
 * 戻り値: 正数=受信したバイト数, 0=受信データ無し、負数=異常終了
 *  BP_ERROR_PARAMETER=lengthがFIFOサイズをオーバー
 */
#define PHY_MAX_SIZE	250
#define RSSI_SIZE		1
int bp_recv_data(void) {
    int status = PHY_ERR_UNKNOWN;
    uint16_t length;
    uint8_t buffer[PHY_MAX_SIZE + RSSI_SIZE];  // extend 1 byte for RSSI data to phy max data size
//    MACHeader *mhr = (MACHeader *)buffer;

    ONERRORGOTO(bp_recv_phy(buffer,PHY_MAX_SIZE, 1));
	if (status <= 0)
	{
		// length over
		goto error;
	} else {
        length = status;
        if (length > sizeof(rx_raw)) {
			printk( "[PHY-BP3596] buffer overflow. (length=%d)\n", length );
            status = PHY_ERR_PARAMETER;  /* payloadのサイズがFIFOサイズをオーバー */
            goto error;
        }

		DEBUGONDISPLAY(MODE_PHY_DEBUG, printk( "[PHY-BP3596] bp_recv_data %d[bytes])\n", length ));
        memcpy(rx_raw, buffer, length);
    }
error:
    return status;
}

// 2015.08.27 Eiichi Saito
void set_recv_ack(int status)
{
	phy_recv_ack=status;
}


// bp3596 main (originally bp_init_thread)
int bp3596_main(void *p)
{
	int err = 0;
    uint8_t intgrp[5];     // 3a?e1t?s\O\e\° [0] dummy GRP#1-4
	int32_t sintn_val = 0;
	int16_t recv_size;
	//int base_ch = WSN_DEFPARAM_BP_BASECHANNEL;
	printk("[PHY-BP3596] bp3596 thread start\n");


	err = ml7396_init();
	if (err) {
		printk(KERN_INFO "ml7396_init: error\n");
		return -1;
	}

	if((drv_mode & MODE_PA_TEST) != 0)
	{
		uint8_t buf[1];
		buf[0] = 0x09;
		bp_spi_write(SPI_ADR_RF_STATUS , buf, 1);
		bp_spi_read(SPI_ADR_RF_STATUS , buf, 1);

		DEBUGONDISPLAY( MODE_PHY_DEBUG, printk("[PHY-BP3596] RF TEST RF_STATUS=0x%02x\n",buf[0]);)
	}

	if(drv_mode && MODE_REG_DUMP)
	{
		int i,j,k;
		uint8_t data[16];

		DEBUGONDISPLAY( MODE_REG_DUMP, printk("REGISTER DUMP\n"));

		for(i=0x80;i<0x83;i++)
		{
			DEBUGONDISPLAY( MODE_REG_DUMP, printk("BANK %x\n",i));
			for(k=0;k<0x80;k+=0x10)
			{
				for(j=0;j<16;j++)
				{
					bp_spi_read(i,k+j,&data[j],1);
				}
				DEBUGONDISPLAY( MODE_REG_DUMP, printk("%02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x %02x %02x %02x %02x %02x %02x\n",
					data[0], data[1], data[2],  data[3],  data[4],  data[5],  data[6],  data[7],
					data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15] ));
			}
		}
	}

	while (!kthread_should_stop()) {
		if(send_flag == true)
		{
			// 2015.08.27 Eiichi Saito
			if (send_cnt <= 0 || phy_recv_ack == true){
				send_flag = false;
				tx_len = 0;
				finish_flag = 0;
				wait_event_interruptible(irq_q, finish_flag);
			}else
			{
				DEBUGONDISPLAY(MODE_PHY_DEBUG,PAYLOADDUMP(tx_raw,tx_len));
				bp_send_phy(tx_raw,tx_len,true);		// temporary send data by ack mode
				udelay(1000);
				udelay(1000);
				send_cnt--;
			}
			// send_flag = false;
			// tx_len = 0;
			// finish_flag = 0;
			// wait_event_interruptible(irq_q, finish_flag);
		}

		if (kthread_should_stop()) 
			break;

		sintn_val = gpio_get_value(GPIO_SINTN);

		while (sintn_val == SINTN_LOW && !kthread_should_stop()){
			bp_spi_read(SPI_ADR_INT_SOURCE_GRP1, &intgrp[1], 1);
			bp_spi_read(SPI_ADR_INT_SOURCE_GRP2, &intgrp[2], 1);
			bp_spi_read(SPI_ADR_INT_SOURCE_GRP3, &intgrp[3], 1);
			bp_spi_read(SPI_ADR_INT_SOURCE_GRP4, &intgrp[4], 1);
			DEBUGONDISPLAY( MODE_PHY_DEBUG, printk("RECV Packet INT1-4 :%02x:%02x:%02x:%02x\n"
			   	, intgrp[1], intgrp[2], intgrp[3], intgrp[4]));

			if (WSN_DEFPARAM_DEBUG == BP_DEBUG_INTDUMP)
				printk("intgrp[1]:%02x intgrp[2]:%02x intgrp[3]:%02x intgrp[4]:%02x\n"
				   	, intgrp[1], intgrp[2], intgrp[3], intgrp[4]);

			if (intgrp[2] & 0x08) {
				intgrp[2] -= 0x08;
				bp_spi_write(SPI_ADR_INT_SOURCE_GRP2, &intgrp[2], 1);
			}

			if (intgrp[2] & 0x40) {
				DEBUGONDISPLAY(MODE_TEST, printk( "recv FIFO access error.\n" ));
				bp_phy_reset();
				continue;
			}
			if (intgrp[3] & 0x30) {
				DEBUGONDISPLAY(MODE_TEST, printk( "CRC ERROR.\n" ));
				bp_phy_reset();
				continue;
			}

			if (intgrp[3] & 0x08) {
				// recv data & add list

#ifdef GPIO_RX_LED
				finish_flag = 1;
				wake_up_interruptible( &rx_led_q );
#endif

//				bp_recv_data_add_list();
				recv_size = bp_recv_data();
				if(recv_size>0)
				{
					mac_rx_callback(rx_raw,recv_size);
				}
			}
			if (intgrp[3] & 0x04) {
				// recv data & add list

#ifdef GPIO_RX_LED
				finish_flag = 1;
				wake_up_interruptible( &rx_led_q );
#endif

				recv_size = bp_recv_data();
				if(recv_size>0)
				{
					mac_rx_callback(rx_raw,recv_size);
				}
			}
			

			// gpio read
			bp_spi_read(SPI_ADR_INT_SOURCE_GRP1, &intgrp[1], 1);
			bp_spi_read(SPI_ADR_INT_SOURCE_GRP2, &intgrp[2], 1);
			bp_spi_read(SPI_ADR_INT_SOURCE_GRP3, &intgrp[3], 1);
			bp_spi_read(SPI_ADR_INT_SOURCE_GRP4, &intgrp[4], 1);
			if (WSN_DEFPARAM_DEBUG == BP_DEBUG_INTDUMP)
				printk("----> intgrp[1]:%02x intgrp[2]:%02x intgrp[3]:%02x intgrp[4]:%02x \n"
				   	, intgrp[1], intgrp[2], intgrp[3], intgrp[4]);
			// check irq source error
			// INTGRP	KNOWN
			//   1		 0xC0	
			//   2		 0x48	
			//   3		 0x38	
			//   4		 0x00	
			if(( (intgrp[1] & ~0xC0) != 0) ||
				((intgrp[2] & ~0x48) != 0) ||
				((intgrp[3] & ~0x3C) != 0) ||
				((intgrp[4] & ~0x00) != 0))
				DEBUGONDISPLAY(MODE_PHY_DEBUG,printk("[PHY-BP3596] IRQ STATUS Unknown intgrp[1]:%02x intgrp[2]:%02x intgrp[3]:%02x intgrp[4]:%02x \n"
				   	, intgrp[1], intgrp[2], intgrp[3], intgrp[4]));

			sintn_val = gpio_get_value(GPIO_SINTN);
		}
	}

	return 0;
}



#ifdef GPIO_RX_LED
static int bp_rx_led_thread(void *p) {
	while (!kthread_should_stop()) {
		finish_flag = 0;
		wait_event_interruptible(rx_led_q, finish_flag);
		if (kthread_should_stop())
			break;
		gpio_set_value(GPIO_RX_LED, 0);
		msleep(1);
		gpio_set_value(GPIO_RX_LED, 1);
	}
	return 0;
}
#endif
#ifdef GPIO_TX_LED
static int bp_tx_led_thread(void *p) {
	while (!kthread_should_stop()) {
		finish_flag = 0;
		wait_event_interruptible(tx_led_q, finish_flag);
		if (kthread_should_stop())
			break;
		gpio_set_value(GPIO_TX_LED, 0);
		msleep(1);
		gpio_set_value(GPIO_TX_LED, 1);
	}
	return 0;
}
#endif

int phy_callback(void)
{
	int status = 0;
	
	finish_flag = 0;

	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk(KERN_INFO "[PHY-BP3596] start main\n"));

	// setup irq
	init_waitqueue_head( &irq_q );
	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("[PHY-BP3596] waitqueeu irq_q\n"));

	// create GPIO irq
	enable_irq(gpio_to_irq(GPIO_SINTN));
	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("[PHY-BP3596] enable irq_q\n"));

	status = request_irq(gpio_to_irq(GPIO_SINTN),
			bp3596_irq_handler,
			IRQF_TRIGGER_FALLING,
			"bp3596gpio", NULL);
	if(status)
	{
		printk(KERN_INFO "[PHY-BP3596] bp3596_init: Unable to grab IRQ\n");
		goto error_irq_request;
	}
	printk("[PHY-BP3596] finish setup irq_q\n");

	// led queue initializing
#ifdef GPIO_RX_LED
	init_waitqueue_head( &rx_led_q );
#endif
#ifdef GPIO_TX_LED
	init_waitqueue_head( &tx_led_q );
#endif


	// start thread
	kthread_tsk = kthread_run(bp3596_main, NULL, DEVNAME"_recv_thread");
	if (IS_ERR(kthread_tsk)) {
		printk(KERN_INFO "bp3596_init: kthread_run failed\n");
		goto error_thread;
	}

#ifdef GPIO_RX_LED
	rx_led_tsk = kthread_run(bp_rx_led_thread, NULL, DEVNAME"_rx_led_thread");
	if (IS_ERR(rx_led_tsk)) {
		printk(KERN_INFO "bp3596_init: rx_led_kthread_run failed\n");
		goto error_thread;
	}
#endif
#ifdef GPIO_TX_LED
	tx_led_tsk = kthread_run(bp_tx_led_thread, NULL, DEVNAME"_tx_led_thread");
	if (IS_ERR(tx_led_tsk)) {
		printk(KERN_INFO "bp3596_init: tx_led_kthread_run failed\n");
		goto error_thread;
	}
#endif

	return 0;
error_thread:
	free_irq(gpio_to_irq(GPIO_SINTN), NULL);
error_irq_request:
	disable_irq(gpio_to_irq(GPIO_SINTN));

	return status;
}

int phy_bp3596_init(t_PHY_INIT_PARAM *p, int (*rx_callback)(uint8_t* data, uint16_t len))
{
	int status = 0;

	// init parameter
	mac_rx_callback = rx_callback;
	memcpy(&phy_param,p,sizeof(phy_param));
	
	printk(KERN_INFO "[PHY-BP3596] ch=%d,pwr=%d,rate=%d\n",phy_param.ch, phy_param.tx_pwr, phy_param.bitrate);

	// I2C init
	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk(KERN_INFO "[PHY-BP3596] i2c init\n"));
	//status = bp_i2c_add_driver();
	status = bp_i2c_init();
	if(status != I2C_OK)
	{
		printk(KERN_INFO "[PHY-BP3596] i2c add driver error\n");
		goto ERR;
	}

	status = bp_i2c_adapter_init();
	if(status != I2C_OK)
	{
		printk(KERN_INFO "[PHY-BP3596] i2c adapter error\n");
		bp_i2c_del_driver();
		goto ERR;
	}
	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk(KERN_INFO "[PHY-BP3596] i2c driver success\n"));
	
	// SPI init
	status = bp_spi_add_driver(phy_callback);
	if(status != SPI_OK)
	{
		printk(KERN_INFO "[PHY-BP3596] spi adapter init error\n");
		bp_i2c_del_driver();
		goto ERR;
	}
	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk(KERN_INFO "[PHY-BP3596] SPI driver success\n"));

	macaddr[7] = bp_read_eeprom(0x20);
	macaddr[6] = bp_read_eeprom(0x21);
	macaddr[5] = bp_read_eeprom(0x22);
	macaddr[4] = bp_read_eeprom(0x23);
	macaddr[3] = bp_read_eeprom(0x24);
	macaddr[2] = bp_read_eeprom(0x25);
	macaddr[1] = bp_read_eeprom(0x26);
	macaddr[0] = bp_read_eeprom(0x27);

	DEBUGONDISPLAY(MODE_PHY_DEBUG, printk("[PHY-BP3596] MAC Address: %02x %02x %02x %02x %02x %02x %02x %02x\n",
			macaddr[7],
			macaddr[6],
			macaddr[5],
			macaddr[4],
			macaddr[3],
			macaddr[2],
			macaddr[1],
			macaddr[0]));

ERR:
	
	return status;
	
}

int phy_bp3596_getaddress(uint8_t *addr)
{
	int status = 0;
	if(addr == NULL) return -1;
	else memcpy(addr,macaddr,8);
	
	return status;
}

int phy_bp3596_remove(void)
{
	int status = 0;
	finish_flag = 1;

	printk(KERN_INFO "[PHY] remove\n");

	// reset BP3596
	bp_reset();
	// thread stop
	if (kthread_tsk != NULL)
		kthread_stop(kthread_tsk);
#ifdef GPIO_RX_LED
	if (rx_led_tsk != NULL)
		kthread_stop(rx_led_tsk);
#endif
#ifdef GPIO_TX_LED
	if (tx_led_tsk != NULL)
		kthread_stop(tx_led_tsk);
#endif
	gpio_free(GPIO_RESETN);
	gpio_free(GPIO_DMONI);
	gpio_free(GPIO_SINTN);
#ifdef GPIO_RX_LED
	gpio_free(GPIO_RX_LED);
#endif
#ifdef GPIO_TX_LED
	gpio_free(GPIO_TX_LED);
#endif
	
#ifdef GPIO_CCA_ERR
	gpio_free(GPIO_CCA_ERR);
#endif

	bp_i2c_del_driver();
	bp_spi_del_driver();

return status;
}

int bp_send(const uint8_t *data, uint16_t len)
{
	if(send_flag == true)
		return -1;
	memcpy(tx_raw,data,len);
	tx_len = len;
	send_flag = true;
	// 2015.08.27 Eiichi Saito
	send_cnt=retry+1;

	if(finish_flag == 0)
	{
		finish_flag = 1;
		wake_up_interruptible( &irq_q );
	}
	return 0;
}

const PHY phy ={
	.name = DEVNAME,
	.init = phy_bp3596_init,
	.get_address = phy_bp3596_getaddress,
	.send_now = bp_send_phy,
	.send = bp_send,
	.remove = phy_bp3596_remove,
	// 2015.08.27 Eiichi Saito
	.set_ack = set_recv_ack,
};

