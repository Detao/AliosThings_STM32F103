/*************************************************
  Copyright (C), 2011-2015 ZheJiang Dahua Technology Co., Ltd.
  �ļ���:   lcsfc.c
  ��  ��:   zheng_xingjian(11853)<zheng_xingjian@dahuatech.com>
  ��  ��:   V1.0.0
  ��  �ڣ�  2016-02-22
  ��  ��:   LCSFCģ��������ֻҪʵ���˳�ʼ�����Լ���Flash�ı�̲���
  
            1��ʹ��˵��
            ��LCSFCģ��ֻ��DH5010M�У�����ֱ����ARCH��ʵ��API
            
            ���ļ��У��ⲿ����ʹ��DH_MDK_LCSFC_xxx, �ڲ�����ʹ��LcsfcXxxx
           
            2��������
            ��������eCos3.0
  
  �޶���ʷ:
  1. ��    ��: 2015-10-24c
     �޶��汾: v0.1
     ��    ��: zheng_xingjian
     �޶���ע: ����
     
  2. ��    ��:
     �޶��汾:2018-1-22
     ��    ��: zhang_jian
     �޶���ע:��ֲ��dh5021a��
*************************************************/
#include <dh_type.h>
#include "lcsfc.h"


/* ��������Flash��С */
static DH_U32 gu32FlashMaxSize;

#define STATIC_CODE_SECTION    __attribute__((section (".staticcode")))
/* ���Դ���  */
#if 0
/*************************************************
  ������ :  LcsfcUartPutChar
  ��  �� :  ���ڷ���һ���ֽ�
  ��  �� :  ����        ����
            c           ����������
  ��  �� :  ����        ����
            ��          ��
  ��  �� :  ����ֵ      ����
            ��          ��
*************************************************/
static DH_VOID STATIC_CODE_SECTION LcsfcUartPutChar(const char c)
{
    volatile DH_U32 lu32Tmp;
    
    /*
    while(1)
    {
        lu32Tmp = *(volatile DH_U32 *)(0x14030014);
        
        if (0 == (lu32Tmp & (1 << 0x05)))
        {
            break;
        }
    }*/
    
    *(volatile DH_U8 *)(0x14030000) = c;
    
}


/*************************************************
  ������ :  LcsfcUartPutString
  ��  �� :  �Ӵ��ڶ�ȡһ���ֽڵ����ݣ�������ʽ
  ��  �� :  ����        ����
            pszString   �����͵��ַ���ָ��
  ��  �� :  ����        ����
            ��          ��
  ��  �� :  ����ֵ      ����
            ��          ��
*************************************************/
static DH_VOID STATIC_CODE_SECTION LcsfcUartPutString(char *pszString)
{
    char *p = pszString;
    
    while(1)
    {
        if ('\0' == *p)
        {
            break;
        }
        else
        {
            /* ��Ϊ����̨�Ļ�����Ҫ������\r��� */
            if ('\n' == *p)
            {
                LcsfcUartPutChar('\r');
            }
            
            LcsfcUartPutChar(*p);
        }
        
        p++;
    }
}


/*************************************************
������ :  SerialDrvNumToStr
��  �� :  ����ת�ַ�,���ڲ�ʹ�ã��򻯴���,��ת��Hex
��  �� :  ����        ����
          u32Num     ��ת����
��  �� :  ����        ����
          pszStr      ת���ַ�����
��  �� :  ����ֵ      ����
          ��          ��
*************************************************/
static DH_VOID STATIC_CODE_SECTION LcsfcNumToStr(DH_CHAR * pszStr, DH_U32 u32Num)
{
    DH_U32 i, lu32Tmp, lu32Offset, lu32Zero;
    char *lpStr;
    char lcTmp;
    
    lpStr = pszStr;
    
    *lpStr ++ = '0';
    *lpStr ++ = 'x';
    lu32Offset = 28;
    lu32Zero = 0;
    
    for (i = 0; i < 8; i++)
    {
        lu32Tmp = (u32Num  >> lu32Offset);
        lu32Offset -= 4;
        
        lu32Tmp &= 0x0F;
        
        if (lu32Tmp < 10)
        {
            lcTmp = 0x30 + lu32Tmp;
        }
        else
        {
            lcTmp = 0x57 + lu32Tmp;
        }
        
        if ((lu32Tmp) && (0 == lu32Zero))
        {
            lu32Zero = 1;
        }
    
        if (lu32Zero)
        {
            *lpStr ++ = lcTmp;
        }
    }
    
    /* ����ȫ������ */
    if (0 == lu32Zero)
    {
        *lpStr ++ = '0';
    }
    
    /* ���뻻�кͽ����� */
    *lpStr ++ = '\n';
    *lpStr ++ = '\0';
    
}


/*************************************************
������ :  LcsfcPrintf
��  �� :  ʵ�ֵ�����������ת�����ַ�������������ַ��ϲ����
          ȫ���ַ����ܳ���32���ֽڣ�����������
��  �� :  ����        ����
          pszSrcStr   ���ϲ�������ַ���������Ϣ��
          u32Num      ��ת�����ַ�������
��  �� :  ����        ����
          ��          ��
��  �� :  ����ֵ      ����
          ��          ��
*************************************************/
static DH_VOID STATIC_CODE_SECTION LcsfcPrintf(DH_U32 u32Num)
{
    static char szBuf[32];
    char *lpszDst;
    char lcTmp;
    
    lpszDst = szBuf;
    
    /* ����Դ�ַ��� */
    *lpszDst++ = 'V';
    *lpszDst++ = 'a';
    *lpszDst++ = 'l';
    *lpszDst++ = '=';
    *lpszDst++ = ' ';
    
    /* �������ֺͽ����� */
    LcsfcNumToStr(lpszDst, u32Num);
    
    LcsfcUartPutString(szBuf);
}
#endif /* LCSFC_DEBUG */

/* ����������ʵ�ֵ��ֽں�4�ֽ�д���� */
static inline void Lcsfc_WrCmdByte(DH_U8 u8Cmd)
{
    *(volatile DH_U8 *)(LCSFC_REG_BASE) = u8Cmd;
}

static inline void Lcsfc_WrCmdDw(DH_U32 u32Cmd)
{
    *(volatile DH_U32 *)(LCSFC_REG_BASE) = u32Cmd;
}

/*************************************************
  ������ :  DH_MDK_LCSFC_Init
  ��  �� :  LCSFC��ʼ��
  ��  �� :  ����            ����
            u32FMaxSize     ���֧��Flash��С
            u32Baudrate     ��Flash�ķ�Ƶ���ã�ֻ����ż����Ƶ�����������������ǿ�Ƽ�1��ż��
  ��  �� :  ��
  ��  �� :  ����ֵ          ����
            DH_SUCCESS      �ɹ�
            DH_FAILURE      ʧ��
*************************************************/
DH_S32 DH_MDK_LCSFC_Init(DH_U32 u32FMaxSize, DH_U32 u32Baudrate)
{
    DH_U32 lu32Tmp, lu32ChkSum;
    DH_S32 ls32Ret;
    
    ls32Ret = DH_FAILURE;
    
    /* ���ò����� */
    if (0 == u32Baudrate)
    {
        u32Baudrate = 2;
    }
    else if (u32Baudrate & 0x01)
    {
        u32Baudrate++;
    }
    else
    {
        /* nothing */
    }
    lu32Tmp = *(volatile DH_U32 *)(LCSFC_REG_BAUDRATE);
    if(u32Baudrate > lu32Tmp)
    {
        *(volatile DH_U32 *)(LCSFC_REG_BAUDRATE) = u32Baudrate;  
        *(volatile DH_U32 *)(LCSFC_REG_SAMDLY) = u32Baudrate >> 1;         
    }
    else
    {
        *(volatile DH_U32 *)(LCSFC_REG_SAMDLY) = u32Baudrate >> 1;
        *(volatile DH_U32 *)(LCSFC_REG_BAUDRATE) = u32Baudrate;
    } 
    
    if (u32FMaxSize > LCSFC_FLASH_MAX_SIZE)
    {
        gu32FlashMaxSize = LCSFC_FLASH_MAX_SIZE;
    }
    else
    {
        gu32FlashMaxSize = u32FMaxSize;
    }
    
    /* ���0��ַ�Ƿ���DH_FLAG�����Ҷ�ͷ���ռ���У��ͣ�
    ���������һ�������򷵻ش��� */
    lu32Tmp = *(volatile DH_U32 *)(LCSFC_REG_BASE);
    if ( DAHUA_FLAG == lu32Tmp )
    {
        /* ����У�飬���У��ʧ�ܣ���Ȼ��Ҫ����Ĭ�ϱ�� */
        lu32Tmp += *(volatile DH_U32 *)(LCSFC_REG_BASE + 0x04);
        lu32ChkSum = *(volatile DH_U32 *)(LCSFC_REG_BASE + 0x08);
        if (lu32Tmp == lu32ChkSum)
        {
            ls32Ret = DH_SUCCESS;
        }
    }
    
    return ls32Ret;
}


/*************************************************
  ������ :  DH_MDK_LCSFC_DeInit
  ��  �� :  LCSFCȥ��ʼ��
  ��  �� :  ����            ����
            ��              ��
  ��  �� :  ��
  ��  �� :  ����ֵ          ����
            ��              ��
*************************************************/
DH_VOID DH_MDK_LCSFC_DeInit(DH_VOID)
{
    /* nothing */
}


/*************************************************
  ������ :  DH_MDK_LCSFC_Erase
  ��  �� :  LCSFC��������
  ��  �� :  ����            ����
            u32TimeOutCnt   ��ʱ����
  ��  �� :  ��
  ��  �� :  ����ֵ          ����
            DH_SUCCESS      Flash���ڿ���״̬
            DH_FAILURE      Flashæ������ʱ
*************************************************/
static DH_S32 STATIC_CODE_SECTION  LcsfcCheckBusy(DH_U32 u32TimeOutCnt)
{
    DH_U32 lu32Tmp, lu32TmCnt;
    
    lu32TmCnt = 0;
    while (1)
    {
        lu32Tmp = *(volatile DH_U32 *)(LCSFC_CHK_FLUSHADDR);
        lu32Tmp = *(volatile DH_U32 *)(LCSFC_CHK_FLAGADDR);
        if (DAHUA_FLAG == lu32Tmp)
        {
            return DH_SUCCESS;
        }
        
        lu32TmCnt++;
        if (lu32TmCnt >= u32TimeOutCnt)
        {
            return DH_FAILURE;
        }
    }
}


/*************************************************
  ������ :  DH_MDK_LCSFC_Erase
  ��  �� :  LCSFC��������
  ��  �� :  ����            ����
            u32StartAddr    ������ʼ��ַ��ע�⣺�����ڲ�����4K����
            u32Length       �������ȣ��ֽ�Ϊ��λ
  ��  �� :  ��
  ��  �� :  ����ֵ          ����
            DH_SUCCESS      �ɹ�
            DH_FAILURE      ʧ��
*************************************************/
DH_S32 STATIC_CODE_SECTION DH_MDK_LCSFC_Erase(DH_U32 u32StartAddr, DH_U32 u32Length)
{
    DH_U32 lu32BlkAddr, lu32BlkCnt, lu32TmpLen;
    DH_U32 i;
    DH_S32 ls32Ret;
    
    lu32BlkAddr = (u32StartAddr & LCSFC_ADDR_MASK) & SPI_BLOCK_NOTMASK;
    
    lu32TmpLen = ((u32StartAddr & LCSFC_ADDR_MASK) & SPI_BLOCK_MASK);
    lu32TmpLen += u32Length;
    
    lu32BlkCnt = (lu32TmpLen / SPI_BLOCK_SIZE);
    if (lu32TmpLen & SPI_BLOCK_MASK)
    {
        lu32BlkCnt++;
    }
    
    ls32Ret = DH_SUCCESS;
    
    /* ���жϣ�����ָ�����Flash, spinlock_t����Ϊ0����Ϊʵ�ʲ����� */
    asm("psrclr ie");
    
    
    /* Ԥ�����ַ�Ͳ������� */
    lu32BlkAddr = ((SPI_CMD_4KBLOCKE << 24) | (lu32BlkAddr & LCSFC_ADDR_MASK));
    for (i = 0; i < lu32BlkCnt; i++)
    {

        Lcsfc_WrCmdByte(SPI_CMD_WRITE_EN);
        Lcsfc_WrCmdDw(lu32BlkAddr);

        /* ����Ƿ���ɲ��� */
        ls32Ret = LcsfcCheckBusy(0xfffff);
        if (DH_SUCCESS != ls32Ret)
        {
            ls32Ret = DH_FAILURE;
            break;
        }
        
        lu32BlkAddr += SPI_BLOCK_SIZE;
    }
    
    /* ���� */
    asm("psrset ie"); 

    return ls32Ret;
}


/*************************************************
  ������ :  LcsfcWrFlashPage
  ��  �� :  дһҳFlash
  ��  �� :  ����            ����
            u32FlashAddr    Flashҳ��ַ
            u32DatCnt       Ҫд����������
            pu8Buf          ���ݻ���
  ��  �� :  ��
  ��  �� :  ����ֵ          ����
            DH_FAILURE      Flashдʧ��
            DH_SUCCESS      Flashд�ɹ�
*************************************************/
static DH_S32 STATIC_CODE_SECTION LcsfcWrFlashPage(DH_U32 u32FlashAddr, DH_U32 u32DatCnt, DH_U8 *pu8Buf)
{
    DH_U32 lu32Cmd, i;
    DH_U8 *lpu8WrBuf;
    
    lpu8WrBuf = pu8Buf;
    
    /* ֧��23λ��ַ */
    lu32Cmd = (u32FlashAddr & LCSFC_ADDR_MASK);
    lu32Cmd |= (SPI_CMD_PAGE_PROG << 24);
    
    /* ����дʹ������ */
    Lcsfc_WrCmdByte(SPI_CMD_WRITE_EN);
    
    *(volatile DH_U32 *)(LCSFC_WRTIE_HOLD_ADDLINE) = lu32Cmd;
    for (i = 0; i < (u32DatCnt - 1); i++)
    {
        *(volatile DH_U8 *)(LCSFC_WRTIE_HOLD_ADDLINE) = *lpu8WrBuf++;
    }
    
    *(volatile DH_U8 *)(LCSFC_REG_BASE) = *lpu8WrBuf;
    
    return DH_SUCCESS;         
}


/*************************************************
  ������ :  DH_MDK_LCSFC_WriteFlash
  ��  �� :  дflash������ҳд��������Ҫ�ж��Ƿ���ڿ�ҳ
  ��  �� :  ����            ����
            u32Addr        Flashд��ַ
            u32Cnt          Ҫд�ĸ���
            pu8Buf         ���ݻ���
  ��  �� :  ��
  ��  �� :  ����ֵ      ����
            DH_FAILURE  Flashдʧ��
            DH_SUCCESS  Flashд�ɹ�
*************************************************/
DH_S32 STATIC_CODE_SECTION DH_MDK_LCSFC_WriteFlash(DH_U32 u32Addr, DH_U32 u32Cnt, DH_U8 *pu8Buf)
{
	DH_U32 lu32BaseAddr;
    DH_U32 lu32PreWrCnt, lu32PageWrCnt, lu32WrCnt;
    DH_U8 *lpu8Ptr;
    DH_S32 ls32Ret;
    
    if((DH_NULL == pu8Buf) || ((u32Addr & LCSFC_ADDR_MASK) > gu32FlashMaxSize) || (0 == u32Cnt))
    {
       return DH_FAILURE;
    }
    
    lu32WrCnt = u32Cnt;
    lpu8Ptr = pu8Buf;
    lu32BaseAddr = u32Addr & LCSFC_ADDR_MASK;
    lu32PreWrCnt = 0;
    lu32PageWrCnt = 0;
    
    /* ���жϣ�����ָ�����Flash, spinlock_t����Ϊ0����Ϊʵ�ʲ����� */
    asm("psrclr ie");
    
    while (1)
    {
        ls32Ret = LcsfcCheckBusy(0xfffff);
        if ( (0 == lu32WrCnt) || (ls32Ret != DH_SUCCESS))
        {
            break;
        }

        /* ���д��ַҳ���룬��ֹ��ҳдflash */
        lu32PageWrCnt = (LCSFC_FLASH_PAGE_SIZE - (lu32BaseAddr & (LCSFC_FLASH_PAGE_SIZE - 1)));
        
        if (lu32PageWrCnt > LCSFC_TX_FIFO_DEPTH)
        {
            lu32PreWrCnt = LCSFC_TX_FIFO_DEPTH;
        }
        else
        {
            lu32PreWrCnt = lu32PageWrCnt;
        }

        /* �ж�ʣ��������ֹ����д���� */
        if (lu32PreWrCnt > lu32WrCnt)
        {
            lu32PreWrCnt = lu32WrCnt;
        }

        ls32Ret = LcsfcWrFlashPage(lu32BaseAddr, lu32PreWrCnt, lpu8Ptr);
        if (ls32Ret != DH_SUCCESS)
        {
            break;
        }

        /* inc env */
        lu32BaseAddr += lu32PreWrCnt;
        lpu8Ptr += lu32PreWrCnt;
        lu32WrCnt -= lu32PreWrCnt;
    }
    
    /* ���� */
    asm("psrset ie"); 
    
    return ls32Ret;

}
