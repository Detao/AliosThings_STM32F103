/*************************************************
  Copyright (C), 2011-2015 ZheJiang Dahua Technology Co., Ltd.
  �ļ���:   lcsfc.h
  ��  ��:   zheng_xingjian(11853)<zheng_xingjian@dahuatech.com>
  ��  ��:   V1.0.0
  ��  �ڣ�  2016-02-22
  ��  ��:   LCSFCģ��ͷ�ļ�
  
            1��ʹ��˵��
            ��
           
            2��������
            ��������eCos3.0
  
  �޶���ʷ:
  1. ��    ��: 
     �޶��汾:
     ��    ��:
     �޶���ע:
     
  2. ��    ��:
     �޶��汾:
     ��    ��:
     �޶���ע:
*************************************************/
#ifndef _LCSFC_H_
#define _LCSFC_H_


#define DAHUA_FLAG		            0x0C646874

/* */
#define LCSFC_REG_BASE              (0x03000000)

#define LCSFC_WRITE_HOLD_BIT        (23)
#define LCSFC_WRITE_HOLD            (1 << LCSFC_WRITE_HOLD_BIT)
#define LCSFC_WRTIE_HOLD_ADDLINE    (LCSFC_REG_BASE | LCSFC_WRITE_HOLD)

#define LCSFC_KV_ADDR               (0x030f0800)
#define LCSFC_CODE_ADDR             (0x03001000)

#define LCSFC_CMD_ERASE             (0x20000000)
#define LCSFC_CMD_PROGPAGE          (0x02000000)

#define LCSFC_ADDR_MASK             (0x7FFFFF)
#define LCSFC_CMD_MASK              (0xFF000000)

/* 
Ϊ����Ӳ����ƣ�LCSFCû�в�ѯFlash״̬��������ˣ���æ
�ö�ȡFlash��0x00��ַ���˵�ַΪFlash�Ĵ󻪱�־����
��Ӳ��Ϊ��߶�Ч�ʣ��Ỻ��һ��SPI FIFO��ȵ�Flash������
LCSFC�ڲ������ԣ����������0x00��ַ�Ļ����������������ݣ�������
��ʵ��Flash���ݣ���ˣ���æ����Ҫ��ȡһ��0x1000�����ַ��
��ʱ�ڲ����屻�滻����0x1000��0x1020��ַ�����ݣ��ٶ�ȡ0x00�����ַ��
��ȷ������ʵFlash�е�����
*/
#define LCSFC_CHK_FLAGADDR          (LCSFC_REG_BASE )
#define LCSFC_CHK_FLUSHADDR         (LCSFC_REG_BASE + 0x1000)

/* SPI���� */
#define LCSFC_TX_FIFO_DEPTH         (92 )
#define LCSFC_RX_FIFO_DEPTH         (32 )
#define LCSFC_FLASH_MAX_SIZE        (4 * 1024 * 1024)
#define LCSFC_FLASH_PAGE_SIZE       (0x100)



/* ���������üĴ��� */
#define LCSFC_REG_BAUDRATE          (0x11003104)
#define LCSFC_REG_SAMDLY            (0x11003108)


#define SPI_PAGE_MASK               0xFF
#define SPI_BLOCK_MASK              0xFFF
#define SPI_BLOCK_NOTMASK           (~SPI_BLOCK_MASK)
#define SPI_SECTOR_MASK             0xFFFF

#define SPI_BLOCK_SIZE              0x1000

/* protection command */
#define SPI_CMD_WRITE_EN            0x06
#define SPI_CMD_WRITE_DIS           0x04
#define SPI_CMD_4KBLOCKE            0x20
#define SPI_CMD_PAGE_PROG           0x02

#endif /* #define _LCSFC_H_ */
