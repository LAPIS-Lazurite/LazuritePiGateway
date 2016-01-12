
/*
 * PHY driver header of BP3596
 * 
 * File:  phy-802154e.h
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

#ifndef _PHY_BP3596_H_
#define _PHY_BP3596_H_

#include <linux/interrupt.h>
#include <linux/gpio.h>

//	
#define GPIO_INPUT  0x0
#define GPIO_OUTPUT 0x1
// IO pin number
#define GPIO_RESETN 27  // 
#define GPIO_DMONI  17  //
#define GPIO_SINTN  18  //
#define GPIO_TX_LED  13 //
#define GPIO_RX_LED  6  //

// SPI Buffer size
#define SPI_BUFFERSIZE 256  /* 送受信バッファサイズ[byte] */

#define ISERROR(func) ((func) < 0)  /* 戻り値のエラー判定 */
#define ONERRORGOTO(func) if (ISERROR(status = (func))) goto error

/** 設定/取得パラメータコマンド **/
typedef enum {
/* bp.param[] の割り付け */
	BP_PARAM_INIT       = 0,
	BP_PARAM_CH0_NA     = 1,
	BP_PARAM_CH0_FH     = 2,
	BP_PARAM_CH0_FM     = 3,
	BP_PARAM_CH0_FL     = 4,
	BP_PARAM_MIN_FL     = 5,
	BP_PARAM_MIN_FM     = 6,
	BP_PARAM_MIN_FH     = 7,
	BP_PARAM_IEEE15_4g  = 50, /* 0:IEEE802.15.4d 1:IEEE802.15.4g(*) */
	BP_PARAM_ED         = 51, /* 1:受信パケットに ED情報を付加する機能 有効(*)/無効 */
	BP_PARAM_TXCRC      = 52, /* TX 00:8bit 01:16bit(*) 10:32bit 11:16bit(CRC-IBM)*/
	BP_PARAM_RXCRC      = 53, /* RX 00:8bit 01:16bit(*) 10:32bit 11:16bit(CRC-IBM)*/

	BP_PARAM_DEBUG      = 255, /* デバッグ機能 */
	BP_PARAM_,
} BP_PARAM;

/** デフォルトパラメータの定義 **/
#define TXPOWER_1MW			0x01
#define TXPOWER_20MW		0x13
#define BP_DEBUG_TXDUMP		0x01
#define BP_DEBUG_RXDUMP		0x02
#define BP_DEBUG_INTDUMP	0x04

#define WSN_DEFPARAM_BP_PANID          0xABCD      /* PANID */
#define WSN_DEFPARAM_BP_BASECHANNEL    36          /* base channel */
#define WSN_DEFPARAM_BP_CCA            20          /* CCA retry count */
#define WSN_DEFPARAM_BP_TXPOW          TXPOWER_20MW/* TxPower */
#define WSN_DEFPARAM_BP_DEFRATE        100         /* DataRate */
//#define WSN_DEFPARAM_DEBUG             (BP_DEBUG_TXDUMP|BP_DEBUG_RXDUMP|BP_DEBUG_INTDUMP)
#define WSN_DEFPARAM_DEBUG             0

/** レジスタアドレス (name, bank, address) **/
/* BANK0 */
#define SPI_ADR_RST_SET           0,0x01  /* リセット制御 */
#define SPI_ADR_CLK_SET           0,0x02  /* クロック制御 */
#define SPI_ADR_CLKOUT            0,0x03  /* 外部クロック出力設定 */
#define SPI_ADR_RATE_SET1         0,0x04  /* データレート変換設定1 */
#define SPI_ADR_RATE_SET2         0,0x05  /* データレート変換設定2 */
#define SPI_ADR_ADC_CLK_SET       0x80|0,0x08  /* RSSI用ADCのクロック設定 */
#define SPI_ADR_OSC_ADJ           0x80|0,0x0b  /* 発信回路端子の負荷容量調整 */
#define SPI_ADR_RF_TEST_MODE      0x80|0,0x0c  /* RFテストパターン設定 */
#define SPI_ADR_PHY_STATE         0x80|0,0x0f  /* PHY ステート表示 */
#define SPI_ADR_FIFO_BANK         0x80|0,0x10  /* FIFO面表示 */
#define SPI_ADR_PLL_LOCK_DETECT   0x80|0,0x11  /* PLLロック判定パラメータ設定 */
#define SPI_ADR_CCA_IGNORE_LEVEL  0,0x12  /* CCAの判定除外EDレベル設定 */
#define SPI_ADR_CCA_LEVEL         0,0x13  /* CCAの閾値レベル設定 */
#define SPI_ADR_CCA_ABORT         0,0x14  /* Auto_Ack時のCCA動作の強制終了時間設定 */
#define SPI_ADR_CCA_CNTRL         0,0x15  /* CCA制御設定および結果読み出し */
#define SPI_ADR_ED_RSLT           0,0x16  /* ED(電力検出)値の読み出し */
#define SPI_ADR_IDLE_WAIT_L       0,0x17  /* CCA時のIDLE判定継続時間設定(下位8ビット) */
#define SPI_ADR_IDLE_WAIT_H       0,0x18  /* CCA時のIDLE判定継続時間設定(上位2ビット) */
#define SPI_ADR_CCA_PROG_L        0,0x19  /* CCA時のIDLE判定経過時間表示(下位8ビット) */
#define SPI_ADR_CCA_PROG_H        0,0x1a  /* CCA時のIDLE判定経過時間表示(上位2ビット) */
#define SPI_ADR_ED_CNTRL          0,0x1b  /* ED(電力検出)値の設定 */
#define SPI_ADR_GAIN_MtoL         0,0x1c  /* 中間ゲインから低ゲイン切り換えの閾値設定 */
#define SPI_ADR_GAIN_LtoM         0,0x1d  /* 低ゲインから中間ゲイン切り換えの閾値設定 */
#define SPI_ADR_GAIN_HtoM         0,0x1e  /* ゲイン更新設定および高ゲインから中間ゲイン切り換えの閾値設定 */
#define SPI_ADR_GAIN_MtoH         0,0x1f  /* 中間ゲインから高ゲイン切り換えの閾値設定 */
#define SPI_ADR_RSSI_ADJ_M        0,0x20  /* 中間ゲインでのRSSIオフセット値設定 */
#define SPI_ADR_RSSI_ADJ_L        0,0x21  /* 低ゲインでのRSSIオフセット値設定 */
#define SPI_ADR_RSSI_STABLE_TIME  0,0x22  /* ゲイン切り換え後のＲＳＳＩ安定化時間設定 */
#define SPI_ADR_RSSI_VAL_ADJ      0,0x23  /* ED変換用の乗除算値設定 */
#define SPI_ADR_INT_SOURCE_GRP1   0,0x24  /* グループ1割り込み表示 */
#define SPI_ADR_INT_SOURCE_GRP2   0,0x25  /* グループ2割り込み表示 */
#define SPI_ADR_INT_SOURCE_GRP3   0,0x26  /* グループ3割り込み表示 */
#define SPI_ADR_INT_SOURCE_GRP4   0,0x27  /* グループ4割り込み表示 */
#define SPI_ADR_PD_DATA_REQ       0,0x28  /* 送信データ要求 */
#define SPI_ADR_PD_DATA_IND       0,0x29  /* 受信データ通知 */
#define SPI_ADR_INT_EN_GRP1       0,0x2a  /* グループ1割り込みイネーブル設定 */
#define SPI_ADR_INT_EN_GRP2       0,0x2b  /* グループ2割り込みイネーブル設定 */
#define SPI_ADR_INT_EN_GRP3       0,0x2c  /* グループ3割り込みイネーブル設定 */
#define SPI_ADR_INT_EN_GRP4       0,0x2d  /* グループ4割り込みイネーブル設定 */
#define SPI_ADR_CH_EN_L           0,0x2e  /* 下位8チャネル(CH0-7)の有効設定 */
#define SPI_ADR_CH_EN_H           0,0x2f  /* 上位8チャネル(CH8-15)の有効設定 */
#define SPI_ADR_IF_FREQ_AFC_H     0,0x30  /* AFCモードのIF周波数設定(上位8ビット) */
#define SPI_ADR_IF_FREQ_AFC_L     0,0x31  /* AFCモードのIF周波数設定(下位8ビット) */
#define SPI_ADR_BPF_AFC_ADJ_H     0,0x32  /* AFCモードのBPF設定(上位2ビット） */
#define SPI_ADR_BPF_AFC_ADJ_L     0,0x33  /* AFCモードのBPF設定(下位8ビット) */
#define SPI_ADR_AFC_CNTRL         0,0x34  /* AFCモードの設定 */
#define SPI_ADR_TX_ALARM_LH       0,0x35  /* 送信FIFO残量告知レベル設定(L→H) */
#define SPI_ADR_TX_ALARM_HL       0,0x36  /* 送信FIFO残量告知レベル設定(H→L) */
#define SPI_ADR_RX_ALARM_LH       0,0x37  /* 受信FIFO残量告知レベル設定(L→H) */
#define SPI_ADR_RX_ALARM_HL       0,0x38  /* 受信FIFO残量告知レベル設定(H→L) */
#define SPI_ADR_PREAMBLE_SET      0,0x39  /* プリアンブルパターン設定 */
#define SPI_ADR_SFD1_SET1         0,0x3a  /* SFDパターン1面目の設定(1st バイト) */
#define SPI_ADR_SFD1_SET2         0,0x3b  /* SFDパターン1面目の設定(2nd バイト) */
#define SPI_ADR_SFD1_SET3         0,0x3c  /* SFDパターン1面目の設定(3rd バイト) */
#define SPI_ADR_SFD1_SET4         0,0x3d  /* SFDパターン1面目の設定(4th バイト) */
#define SPI_ADR_SFD2_SET1         0,0x3e  /* SFDパターン2面目の設定(1st バイト) */
#define SPI_ADR_SFD2_SET2         0,0x3f  /* SFDパターン2面目の設定(2nd バイト) */
#define SPI_ADR_SFD2_SET3         0,0x40  /* SFDパターン2面目の設定(3rd バイト) */
#define SPI_ADR_SFD2_SET4         0,0x41  /* SFDパターン2面目の設定(4th バイト) */
#define SPI_ADR_TX_PR_LEN         0,0x42  /* 送信プリアンブル長設定 */
#define SPI_ADR_RX_PR_LEN_SFD_LEN 0,0x43  /* プリアンブル比較長設定/SFD長設定 */
#define SPI_ADR_SYNC_CONDITION    0,0x44  /* プリアンブル及びSFD検出の誤り許容値設定 */
#define SPI_ADR_PACKET_MODE_SET   0,0x45  /* パケットモードの各種設定 */
#define SPI_ADR_FEC_CRC_SET       0,0x46  /* 送信パケットのFECとCRC設定 */
#define SPI_ADR_DATA_SET          0,0x47  /* 送受信データの各種設定 */
#define SPI_ADR_CH0_FL            0,0x48  /* チャネル#0 周波数設定(下位8ビット) */
#define SPI_ADR_CH0_FM            0,0x49  /* チャネル#0 周波数設定(中位8ビット) */
#define SPI_ADR_CH0_FH            0,0x4a  /* チャネル#0 周波数設定(上位4ビット) */
#define SPI_ADR_CH0_NA            0,0x4b  /* PLL Nカウンタ、Aカウンター設定 */
#define SPI_ADR_CH_SPACE_L        0,0x4c  /* チャネル間隔設定(下位8ビット) */
#define SPI_ADR_CH_SPACE_H        0,0x4d  /* チャネル間隔設定(上位8ビット) */
#define SPI_ADR_F_DEV_L           0,0x4e  /* GFSK周波数偏位設定(下位8ビット) */
#define SPI_ADR_F_DEV_H           0,0x4f  /* GFSK周波数偏位設定(上位8ビット) */
#define SPI_ADR_ACK_TIMER_L       0,0x50  /* Auto_Ack用Ackタイマー設定(下位8ビット) */
#define SPI_ADR_ACK_TIMER_H       0,0x51  /* Auto_Ack用Ackタイマー設定(上位8ビット) */
#define SPI_ADR_ACK_TIMER_EN      0,0x52  /* Ackタイマー設定 */
#define SPI_ADR_ACK_FRAME1        0,0x53  /* Ackパケットのパターン設定(下位8ビット) */
#define SPI_ADR_ACK_FRAME2        0,0x54  /* Ackパケットのパターン設定(上位8ビット) */
#define SPI_ADR_AUTO_ACK_EN       0,0x55  /* Aut_Ack の設定 */
#define SPI_ADR_GFIL00_FSK_FDEV1  0,0x59  /* ガウシアンフィルタ設定1FSK変調時の第一周波数偏位設定 */
#define SPI_ADR_GFIL01_FSK_FDEV2  0,0x5a  /* ガウシアンフィルタ設定2FSK変調時の第二周波数偏位設定 */
#define SPI_ADR_GFIL02_FSK_FDEV3  0,0x5b  /* ガウシアンフィルタ設定3FSK変調時の第三周波数偏位設定 */
#define SPI_ADR_GFIL03_FSK_FDEV4  0,0x5c  /* ガウシアンフィルタ設定4FSK変調時の第四周波数偏位設定 */
#define SPI_ADR_GFIL04            0,0x5d  /* ガウシアンフィルタ設定5 */
#define SPI_ADR_GFIL05            0,0x5e  /* ガウシアンフィルタ設定6 */
#define SPI_ADR_GFIL06            0,0x5f  /* ガウシアンフィルタ設定7 */
#define SPI_ADR_GFIL07            0,0x60  /* ガウシアンフィルタ設定8 */
#define SPI_ADR_GFIL08            0,0x61  /* ガウシアンフィルタ設定9 */
#define SPI_ADR_GFIL09            0,0x62  /* ガウシアンフィルタ設定10 */
#define SPI_ADR_GFIL10            0,0x63  /* ガウシアンフィルタ設定11 */
#define SPI_ADR_GFIL11            0,0x64  /* ガウシアンフィルタ設定12 */
#define SPI_ADR_FSK_TIME1         0,0x65  /* FSK周波数偏位タイミング設定(FDEV3) */
#define SPI_ADR_FSK_TIME2         0,0x66  /* FSK周波数偏位タイミング設定(FDEV2) */
#define SPI_ADR_FSK_TIME3         0,0x67  /* FSK周波数偏位タイミング設定(FDEV1) */
#define SPI_ADR_FSK_TIME4         0,0x68  /* FSK周波数偏位タイミング設定(偏位0) */
#define SPI_ADR_PLL_MON_DIO_SEL   0,0x69  /* PLロック信号出力設定、DIOモード設定、 */
#define SPI_ADR_FAST_TX_SET       0,0x6a  /* FATS_TXモードの送信開始トリガ設定 */
#define SPI_ADR_CH_SET            0,0x6b  /* 送受信チャネル設定 */
#define SPI_ADR_RF_STATUS         0,0x6c  /* RF部動作状態の設定と確認 */
#define SPI_ADR_2DIV_ED_AVG       0,0x6d  /* 2ダイバーシティ時のED算出平均回数設定 */
#define SPI_ADR_2DIV_GAIN_CNTRL   0,0x6e  /* ゲイン制御モード設定 */
#define SPI_ADR_2DIV_SEARCH       0,0x6f  /* 2ダイバーシティ時のサーチモードとサーチ時間設定 */
#define SPI_ADR_2DIV_FAST_LV      0,0x70  /* 2ダイバーシティ時のFASTモードの閾値設定 */
#define SPI_ADR_2DIV_CNTRL        0,0x71  /* 2ダイバーシティの各種設定 */
#define SPI_ADR_2DIV_RSLT         0,0x72  /* 2ダイバーシティ結果読み出し、強制設定 */
#define SPI_ADR_ANT1_ED           0,0x73  /* ANT1のED読み出し */
#define SPI_ADR_ANT2_ED           0,0x74  /* ANT2のED値読み出し */
#define SPI_ADR_RF_CNTRL_SET      0,0x75  /* RF制御端子の強制出力設定 */
#define SPI_ADR_CRC_AREA_FIFO_TRG 0,0x77  /* CRC対象範囲およびFIFOトリガ出力設定 */
#define SPI_ADR_RSSI_MON          0,0x78  /* RSSIのデジタル読み出し */
#define SPI_ADR_TEMP_MON          0,0x79  /* 温度のデジタル読み出し */
#define SPI_ADR_PN9_SET_L         0,0x7a  /* Whiteningの初期値設定(bit8～bit0) */
#define SPI_ADR_PN9_SET_H         0,0x7b  /* Whiteningの初期値設定(bit9)及び制御 */
#define SPI_ADR_RD_FIFO_LAST      0,0x7c  /* FIFOの残量またはアドレスの表示 */
#define SPI_ADR_WR_TX_FIFO        0,0x7e  /* 送信FIFO */
#define SPI_ADR_RD_RX_FIFO        0,0x7f  /* 受信FIFO */
/* BANK1 */
#define SPI_ADR_DEMOD_SET       1,0x01  /* 復調器調整 */
#define SPI_ADR_RSSI_ADJ        1,0x02  /* RSSI調整 */
#define SPI_ADR_RSSI_TEMP_OUT   1,0x03  /* RSSIと温度情報の出力設定 */
#define SPI_ADR_PA_ADJ1         1,0x04  /* PA調整レジスタ1の設定 */
#define SPI_ADR_PA_ADJ2         1,0x05  /* PA調整レジスタ2の設定 */
#define SPI_ADR_PA_ADJ3         1,0x06  /* PA調整レジスタ3の設定 */
#define SPI_ADR_PA_CNTRL        1,0x07  /* 外部PA制御およびPAモードの設定 */
#define SPI_ADR_SW_OUT_RAMP_ADJ 1,0x08  /* SW信号の出力設定と送信立ち上がり時間調整 */
#define SPI_ADR_PLL_CP_ADJ      1,0x09  /* 送受信時のPLLチャージポンプ電流値調整 */
#define SPI_ADR_IF_FREQ_H       1,0x0a  /* IF周波数設定(上位8ビット) */
#define SPI_ADR_IF_FREQ_L       1,0x0b  /* IF周波数設定(下位8ビット) */
#define SPI_ADR_IF_FREQ_CCA_H   1,0x0c  /* CCA時のIF周波数設定(上位8ビット) */
#define SPI_ADR_IF_FREQ_CCA_L   1,0x0d  /* CCA時のIF周波数設定(下位8ビット) */
#define SPI_ADR_BPF_ADJ_H       1,0x0e  /* BPF容量設定(上位2ビット) */
#define SPI_ADR_BPF_ADJ_L       1,0x0f  /* BPF容量設定(下位8ビット) */
#define SPI_ADR_BPF_CCA_ADJ_H   1,0x10  /* CCA時のBPF容量設定(上位2ビット) */
#define SPI_ADR_BPF_CCA_ADJ_L   1,0x11  /* CCA時のBPF容量設定(下位8ビット) */
#define SPI_ADR_RSSI_LPF_ADJ    1,0x12  /* RSSIの出力時定数調整 */
#define SPI_ADR_PA_REG_FINE_ADJ 0x80|1,0x13  /* PA用レギュレータの微調整 */
#define SPI_ADR_IQ_MAG_ADJ      1,0x14  /* IFのI/Q振幅バランス調整 */
#define SPI_ADR_IQ_PHASE_ADJ    1,0x15  /* IFのI/Q位相バランス調整 */
#define SPI_ADR_VCO_CAL_MIN_FL  1,0x16  /* VCOキャリブレーション用下限周波数設定(下位8ビット) */
#define SPI_ADR_VCO_CAL_MIN_FM  1,0x17  /* VCOキャリブレーション用下限周波数設定(中位8ビット) */
#define SPI_ADR_VCO_CAL_MIN_FH  1,0x18  /* VCOキャリブレーション用下限周波数設定(上位4ビット) */
#define SPI_ADR_VCO_CAL_MAX_N   1,0x19  /* VCOキャリブレーション用上限周波数設定 */
#define SPI_ADR_VCO_CAL_MIN     1,0x1a  /* 下限側VCO キャリブレーション値の表示と設定 */
#define SPI_ADR_VCO_CAL_MAX     1,0x1b  /* 上限側VCO キャリブレーション値の表示と設定 */
#define SPI_ADR_VCO_CAL         1,0x1c  /* 現在のキャリブレーション値の表示と設定 */
#define SPI_ADR_VCO_CAL_START   1,0x1d  /* VCOキャリブレーションの実行 */
#define SPI_ADR_BPF_ADJ_OFFSET  1,0x1e  /* BPF調整オフセット値表示 */
#define SPI_ADR_ID_CODE         0x80|1,0x2b  /* LSIのID コード読み出し */
#define SPI_ADR_PA_REG_ADJ1     0x80|1,0x33  /* PA用レギュレータの調整1 */
#define SPI_ADR_PA_REG_ADJ2     0x80|1,0x34  /* PA用レギュレータの調整2 */
#define SPI_ADR_PA_REG_ADJ3     0x80|1,0x35  /* PA用レギュレータの調整3 */
#define SPI_ADR_PLL_CTRL        0x80|1,0x3a  /* RF調整 */
#define SPI_ADR_RX_ON_ADJ2      0x80|1,0x3f  /* RX_ON調整レジスタ2 */
#define SPI_ADR_LNA_GAIN_ADJ_M  0x80|1,0x49  /* 中間ゲイン時のLNA ゲイン調整 */
#define SPI_ADR_LNA_GAIN_ADJ_L  0x80|1,0x4a  /* 低ゲイン時のLNA ゲイン調整 */
#define SPI_ADR_MIX_GAIN_ADJ_M  0x80|1,0x4e  /* 中間ゲイン時のミキサーゲイン調整 */
#define SPI_ADR_MIX_GAIN_ADJ_L  0x80|1,0x4f  /* 低ゲイン時のミキサーゲイン調整 */
#define SPI_ADR_TX_OFF_ADJ1     0x80|1,0x55  /* TX_OFF調整レジスタ1 */
#define SPI_ADR_RSSI_SLOPE_ADJ  0x80|1,0x5a  /* RSSIの傾き調整 */
/* BANK2 */
#define SPI_ADR_SYNC_MODE       0x80|2,0x12  /* ビット同期のモード設定 */
#define SPI_ADR_PA_ON_ADJ       0x80|2,0x1e  /* PA_ON信号のタイミング調整 */
#define SPI_ADR_RX_ON_ADJ       0x80|2,0x22  /* RX_ON信号のタイミング調整 */
#define SPI_ADR_RXD_ADJ         0x80|2,0x24  /* RXD信号のタイミング調整 */
#define SPI_ADR_RATE_ADJ1       0x80|2,0x2A  /* bit7-0:150kbpsモード時復調器調整値格納(下位8bit) */
#define SPI_ADR_RATE_ADJ2       0x80|2,0x2B  /* bit4:150kbpsモード有効イネーブル(0=OFF/1=ON) bit1-0:150kbpsモード時復調器調整値格納(上位2bit)*/
#define SPI_ADR_RAMP_CNTRL      0x80|2,0x2c  /* ランプ制御 */
#define SPI_ADR_ADDFILCNTRL     2,0x60  /* アドレスフィルタ機能の設定 */
#define SPI_ADR_PANID_L         2,0x61  /* アドレスフィルタPANID設定(下位8ビット) */
#define SPI_ADR_PANID_H         2,0x62  /* アドレスフィルタPANID設定(上位8ビット) */
#define SPI_ADR_64ADDR1         2,0x63  /* 64ビットアドレス設定(1stバイト:最下位) */
#define SPI_ADR_64ADDR2         2,0x64  /* 64ビットアドレス設定(2ndバイト) */
#define SPI_ADR_64ADDR3         2,0x65  /* 64ビットアドレス設定(3rdバイト) */
#define SPI_ADR_64ADDR4         2,0x66  /* 64ビットアドレス設定(4thバイト) */
#define SPI_ADR_64ADDR5         2,0x67  /* 64ビットアドレス設定(5thバイト) */
#define SPI_ADR_64ADDR6         2,0x68  /* 64ビットアドレス設定(6thバイト) */
#define SPI_ADR_64ADDR7         2,0x69  /* 64ビットアドレス設定(7thバイト) */
#define SPI_ADR_64ADDR8         2,0x6a  /* 64ビットアドレス設定(8thバイト:最上位) */
#define SPI_ADR_SHT_ADDR0_L     2,0x6b  /* ショートアドレス0の設定(下位8ビット) */
#define SPI_ADR_SHT_ADDR0_H     2,0x6c  /* ショートアドレス0の設定(上位8ビット) */
#define SPI_ADR_SHT_ADDR1_L     2,0x6d  /* ショートアドレス1の設定(下位8ビット) */
#define SPI_ADR_SHT_ADDR1_H     2,0x6e  /* ショートアドレス1の設定(上位8ビット) */
#define SPI_ADR_DISCARD_COUNT_L 2,0x6f  /* 廃棄パケット数の表示(下位8ビット) */
#define SPI_ADR_DISCARD_COUNT_H 2,0x70  /* 廃棄パケット数の表示(上位8ビット) */

#endif // _PHY_BP3596_H_



