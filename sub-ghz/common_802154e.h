#ifndef _COMMON_802154E_H_
#define _COMMON_802154E_H_


#define	MODE_NORMAL			0x0000
#define	MODE_INVALID_MAC	0x0001
#define MODE_DRV_DEBUG		0x8000
#define MODE_MAC_DEBUG		0x4000
#define MODE_PHY_DEBUG		0x2000
#define MODE_TEST			0x1000
#define MODE_PA_TEST		0x0800
#define MODE_REG_DUMP		0x0400
#define MODE_DEBUG			( MODE_DRV_DEBUG | MODE_MAC_DEBUG | MODE_PHY_DEBUG )
extern uint16_t drv_mode;

#define DEBUGONDISPLAY(a,b)		{				\
	if(drv_mode & a )					\
	{											\
		b;										\
	}											\
}												\

#ifndef PAYLOADDUMP
#define PAYLOADDUMP( data, size ) {                                                   \
	int d;                                                                            \
	if (size > 0){                                                                    \
		printk( " [PAYLOAD] 00 01 02 03 04 05 06 07  08 09 0A 0B 0C 0D 0E 0F\r\n");   \
		printk( "-----------------------------------------------------------\r\n");   \
		printk( "00000000   " );                                                      \
		for (d = 0; d < size; d++){                                                   \
			printk( "%02X %s", data[d], !((d + 1) % 8)?" ":"" );                      \
			if (!((d + 1) % 16)) {                                                    \
				printk( "\r\n%08X   ", d + 1);                                        \
			}                                                                         \
		}                                                                             \
		printk( "\r\n" );                                                             \
	}                                                                                 \
}
#endif


#endif // _COMMON_802154E_H_
