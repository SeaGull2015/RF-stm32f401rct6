#ifndef NRF24L01_MACROS_H_
#define NRF24L01_MACROS_H_

#define NRF24_CE_PIN ( 1 << 0 )
#define NRF24_CS_PIN ( 1 << 1 ) /* set to low while talking with nrf */

////////��������� ����
#define NRF24_COMMAND_W_TX_PAYLOAD ( 0xA0 )

/////// ������� CONFIG
#define NRF_REG_CONFIG ( 0x00 )

#define NRF_REG_CONFIG_EN_CRC	( 1<<3 )
#define NRF_REG_CONFIG_CRCO		( 1<<2 )
#define NRF_REG_CONFIG_PWR_UP	( 1<<1 )
#define NRF_REG_CONFIG_PRIM_RX	( 1<<0 )

////////// ������� EN_AA
#define NRF_REG_EN_AA ( 0x01 )

////////////������� EN_RXADDR
#define NRF_REG_EN_RXADDR ( 0x02 )

#define NRF_REG_EN_RXADDR_ERX_P5 ( 1<<5 )
#define NRF_REG_EN_RXADDR_ERX_P4 ( 1<<4 )
#define NRF_REG_EN_RXADDR_ERX_P3 ( 1<<3 )
#define NRF_REG_EN_RXADDR_ERX_P2 ( 1<<2 )
#define NRF_REG_EN_RXADDR_ERX_P1 ( 1<<1 )
#define NRF_REG_EN_RXADDR_ERX_P0 ( 1<<0 )

/////////register SETUP_AW
#define NRF_REG_SETUP_AW ( 0x03 )

#define NRF_REG_SETUP_AW_AW1 ( 1<<1 )
#define NRF_REG_SETUP_AW_AW0 ( 1<<0 )

//////////// ������� RF_CH
#define NRF_REG_RF_CH ( 0x05 )

#define NRF_CHANEL ( 30 )

/////////// ������� RSETUP_AW
#define NRF_REG_RSETUP_AW ( 0x06 )

#define NRF_REG_RSETUP_AW_RF_CONT_WAVE	( 1<<7 )
#define NRF_REG_RSETUP_AW_RF_RF_DR_LOW ( 1<<5 )
#define NRF_REG_RSETUP_AW_RF_PLL_LOCK ( 1<<4 )
#define NRF_REG_RSETUP_AW_RF_RF_DR_HIGH ( 1<<3 )
#define NRF_REG_RSETUP_AW_RF_PWR1 ( 1<<2 )
#define NRF_REG_RSETUP_AW_RF_PWR0 ( 1<<1 )

//Register STATUS *********************************************************************
#define NRF_REG_STATUS ( 0x07 )

#define NRF_REG_STATUS_RX_DR ( 1<<6 )
#define NRF_REG_STATUS_TX_DS ( 1<<5 )
#define NRF_REG_STATUS_MAX_RT ( 1<<4 )
#define NRF_REG_STATUS_RX_P_NO2 ( 1<<3 )
#define NRF_REG_STATUS_RX_P_NO1 ( 1<<2 )
#define NRF_REG_STATUS_RX_P_NO0 ( 1<<1 )
#define NRF_REG_STATUS_TX_FULL ( 1<<0 )
//������� RX_ADDR_P0
#define NRF_REG_RX_ADDR_P0 ( 0x0A )

///////////Register TX_ADDR
#define NRF_REG_TX_ADDR ( 0x10 )

//////////// Register RX_PW_P0
#define NRF_REG_RX_PW_P0 ( 0x11 )

#define NRF_PACKET_SIZE ( 32 )

#define NRF_DYNPD ( 0x1C ) //was C1
#define NRF_FEATURE ( 0x1D ) //was D1
#define NRF_EN_AA ( 0x01 )
#define NRF_STATUS ( 0x07 )
#define NRF_R_RX_PAYLOAD ( 0x61 )
#define NRF_WRITE_MASK ( 0b00100000 )

//Register FIFO_STATUS

#define NRF_REG_FIFO_STATUS ( 0x17 )

#define NRF_REG_FIFO_STATUS_TX_REUSE ( 1<<6 )
#define NRF_REG_FIFO_STATUS_TX_FULL ( 1<<5 )
#define NRF_REG_FIFO_STATUS_TX_EMPTY ( 1<<4 )
#define NRF_REG_FIFO_STATUS_RX_FULL ( 1<<1 )
#define NRF_REG_FIFO_STATUS_RX_EMPTY ( 1<<0 )

#define NOP                             0xFF

#endif /* NRF24L01_MACROS_H_ */
