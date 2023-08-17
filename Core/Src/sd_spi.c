
#include "diskio.h"
#include "gpio.h"
#include "spi.h"
#include "rtos_bus.h"
#include <string.h>
/* Use OS delay tick to wait timeout instead of polling spi */
#include "FreeRTOS.h"
#include "task.h"

#define sd_spi_txrx(pTxData, pRxData, size)        \
        spi_txrx(&hspi1, pTxData, pRxData, size)
#define sd_spi_tx(pData, size)                     \
        spi_tx(&hspi1, pData, size)
#define cs_low()        \
        HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET)
#define cs_high()       \
        HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET)
#define set_spi_low_clk()                                               \
        do {                                                            \
                __HAL_SPI_DISABLE(&hspi1);                              \
                hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;\
                __HAL_SPI_ENABLE(&hspi1);                               \
        } while (0)
#define set_spi_high_clk()                                              \
        do {                                                            \
                __HAL_SPI_DISABLE(&hspi1);                              \
                hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; \
                __HAL_SPI_ENABLE(&hspi1);                               \
        } while (0)

#define CMD_SIZE        6      /* 6 bytes long and start with MSB */

#define CMD0    (0)             /* GO_IDLE_STATE */
#define CMD8    (8)             /* SEND_IF_COND */
#define CMD9    (9)             /* SEND_CSD */
#define CMD10   (10)            /* SEND_CID */
#define CMD12	(12)		/* STOP_TRANSMISSION */
#define CMD17	(17)		/* READ_SINGLE_BLOCK */
#define CMD18	(18)		/* READ_MULTIPLE_BLOCK */
#define CMD24	(24)		/* WRITE_BLOCK */
#define CMD25	(25)		/* WRITE_MULTIPLE_BLOCK */
#define CMD32   (32)            /* ERASE_WR_BLK_START_ADDR */
#define CMD33   (33)            /* ERASE_WR_BLK_END_ADDR */
#define CMD38   (38)            /* ERASE */
#define CMD55   (55)            /* APP_CMD */
#define CMD58   (58)            /* READ_OCR */
#define ACMD13  (0x80 + 13)     /* SD_STATUS */
#define ACMD23  (0x80 + 23)     /* SET_WR_BLK_ERASE_COUNT */
#define ACMD41  (0x80 + 41)     /* SEND_OP_COND (SDC) */

#define CMD0_CRC7       (0x4A)
#define CMD8_CRC7       (0x43)

static volatile DSTATUS stat = STA_NOINIT;
static BYTE cardtype;
static BYTE DO_H[512];

extern WORD Timer1, Timer2;

static inline void sd_spi_rx(BYTE *data, WORD size)
{
        sd_spi_txrx(DO_H, data, size);
}

/**
 * @brief Wait ready until timeout
 * 
 * @param ms < 65535
 * @return int 1: ready / 0: timeout
 */
static int wait_ready(WORD ms)
{
        uint8_t token;

        /* Nec = 1 */
        sd_spi_rx(&token, 1);
        if (token == 0xFF)
                return 1;
        Timer2 = ms;
        do {
                cs_high();
                sd_spi_rx(&token, 1); /* Nds = 1 */
                vTaskDelay(1); /* us OS delay one tick to wait state */
                cs_low();
                sd_spi_rx(&token, 1);
        } while ((token != 0xFF) && Timer2);

        return (token == 0xFF);
}

/**
 * @brief Receive a data packet from SD card
 * 
 * @param buff 
 * @param btr 
 * @return int 1: OK / 0: ERROR
 */
static int rcvr_datablock(BYTE *buff, UINT btr)
{
        BYTE token, crc[2];

        Timer1 = 100; /* Nac (max read access time) = 100 ms */
        do {    /* Wait for data block token */
                sd_spi_rx(&token, 1);
        } while ((token == 0xFF) && Timer1);
        if (token != 0xFE)
                return 0;

        sd_spi_rx((BYTE *)buff, btr);
        sd_spi_rx(crc, 2);

        return 1;
}

#if FF_FS_READONLY == 0
/**
 * @brief Send a data packet to SD card
 * 
 * @param buff 
 * @param token 
 * @return int 1: OK / 0: ERROR
 */
static int xmit_datablock(const BYTE *buff, BYTE token)
{
	BYTE resp, crc[2] = {0xFF, 0xFF};

	sd_spi_rx(&resp, 1); /* Wait 8 cycle(Nwr = 1) before writing data */
	sd_spi_tx(&token, 1);    /* Send start block token */
	if (token != 0xFD) {  /* Send data if token is other than Stop Tran */
                sd_spi_tx((BYTE *)buff, 512);
		sd_spi_tx(crc, 2); /* Transmit dummy CRC (0xFFFF) */
		/* Receive data response token */
                sd_spi_rx(&resp, 1);
		/* bit[3:1]: 010(accepted), 101: CRC err, 110: write err */
                if ((resp & 0x1F) != 0x05)
                        return 0;
	} else {                /* Wait 8 cycle(Nbr = 1) after stop tran */
                sd_spi_rx(&resp, 1);
        }
	return 1;
}
#endif

/**
 * @brief Send a command pachet to SDC
 * bit         |   47  | 46 | [45:40] |  [39:8]  | [7:1] |  0  |
 * value       |    0  |  1 |    x    |     x    |   x   |  1  |
 * description | start | tx |  cmd_id | argument |  CRC7 | end |
 *
 * @param cmd_id command index
 * @param arg argument
 * @return BYTE R1 response
 */
static BYTE send_cmd(BYTE cmd_id, DWORD arg)
{
        BYTE cmd[CMD_SIZE], resp, i;

        if (cmd_id & 0x80) {
                cmd_id &= 0x3F;
                resp = send_cmd(CMD55, 0);
                if (resp > 1)
                        return resp;
        }
        cmd[0] = 0x40 | cmd_id;
        cmd[1] = (BYTE)(arg >> 24);
        cmd[2] = (BYTE)(arg >> 16);
        cmd[3] = (BYTE)(arg >> 8);
        cmd[4] = (BYTE)(arg);
        cmd[5] = 0x01; /* End bit */

        if (cmd_id == CMD8)
                cmd[5] |= CMD8_CRC7 << 1;
        if (cmd_id == CMD0)
                cmd[5] |= CMD0_CRC7 << 1;
        else if ((cmd_id != CMD12) && !wait_ready(500))
                return 0xFF;

        sd_spi_tx(cmd, CMD_SIZE); /* Send command first */
        if (cmd_id == CMD12)
                sd_spi_rx(&resp, 1);
        i = 8; /* Receive R1 during 8 byte cycle (Ncr <= 8) */
        do {
                sd_spi_rx(&resp, 1);
        } while ((resp & 0x80) && --i);

        return resp;
}

/**
 * @brief Initialize sd card(only v2+) in spi mode
 * 
 * @param drv physical drive number (0)
 * @return DSTATUS see diskio.h
 */
DSTATUS disk_initialize(BYTE drv)
{
        BYTE resp[4], i;

        stat = STA_NOINIT;
        if (drv)
                return stat; /* Support only drive 0 */
        if (stat & STA_NODISK)
                return stat;
        set_spi_low_clk();       /* 100 - 400 kHz in ID mode */
        memset(DO_H, 0xFF, 512);
        /* Hold CS/MOSI at least 74 cycles */
        cs_high();
        sd_spi_tx(DO_H, 10);
        cs_low();
        i = 10; /* Try to reset SD card 10 times */
        while ((send_cmd(CMD0, 0) != 0x01) && --i)
                ;
        if (!i)
                return stat;
        /* Send interface condition command, arg: 01(VHS) AA(pattern) */
        if(send_cmd(CMD8, 0x1AA) == 0x01) {
                sd_spi_rx(resp, 4); /* Check R7 following echo packet */
                if ((resp[2] != 0x01) || (resp[3] != 0xAA))
                        return stat;
        } else {
                return stat;
        }
        // send_cmd(CMD58, 0, R); /* get supported voltage (not mandantory) */
        /* Set HCS = 1 and activate card initialization process */
        Timer1 = 1000; /* 1 sec timout */
        while (send_cmd(ACMD41, 1UL << 30) && Timer1)
                ;
        if (!Timer1)
                return stat;
        /* read OCR */
        if (!send_cmd(CMD58, 0)) {
                sd_spi_rx(resp, 4); /* Receive R3 following OCR */
                cardtype = (resp[0] & 0x40) ? CT_SDC2 | CT_BLOCK : CT_SDC2;       
                set_spi_high_clk(); /* Enter data transfer mode */
                stat &= ~STA_NOINIT;
        }
        cs_high();

        return stat;
}

DSTATUS disk_status(BYTE drv)
{
        if (drv)
                return STA_NOINIT;
        return stat;
}

/**
 * @brief Read sector(s)
 * 
 * @param drv physical drive number (0)
 * @param buff 
 * @param sector 
 * @param count (1...128)
 * @return DRESULT 1: OK / 0: ERROR
 */
DRESULT disk_read(BYTE drv, BYTE *buff, LBA_t sector, UINT count)
{
	DWORD sect = (DWORD)sector;

	if (drv || !count)
                return RES_PARERR;
	if (stat & STA_NOINIT)
                return RES_NOTRDY;
        /* LBA to BA conversion (byte addressing cards) */
	if (!(cardtype & CT_BLOCK))
                sect *= 512;

        cs_low();
	if (count == 1) {	/* Single sector read */
		if (!send_cmd(CMD17, sect) && rcvr_datablock(buff, 512))
			count = 0;
	}
	else {                  /* Multiple sector read */
		if (!send_cmd(CMD18, sect)) {
                	do {
				if (!rcvr_datablock(buff, 512))
                                        break;
				buff += 512;
			} while (--count);
			send_cmd(CMD12, 0);
		}
	}
        cs_high();

	return count ? RES_ERROR : RES_OK;
}

#if FF_FS_READONLY == 0
/**
 * @brief Write sector(s)
 * 
 * @param drv physical drive number (0)
 * @param buff 
 * @param sector 
 * @param count (1...128)
 * @return DRESULT 1: OK / 0: ERROR
 */
DRESULT disk_write (BYTE drv, const BYTE *buff, LBA_t sector, UINT count)
{
	DWORD sect = (DWORD)sector;

	if (drv || !count)
                return RES_PARERR;
	if (stat & STA_NOINIT)
                return RES_NOTRDY;
	if (stat & STA_PROTECT)
                return RES_WRPRT;
        /* LBA to BA conversion (byte addressing cards) */
	if (!(cardtype & CT_BLOCK))
                sect *= 512;

        cs_low();
	if (count == 1) {	/* Single sector write */
                if (!send_cmd(CMD24, sect) && xmit_datablock(buff, 0xFE))
			count = 0;
	}
	else {		        /* Multiple sector write */
		if (cardtype & CT_SDC)
                        send_cmd(ACMD23, count);  
		if (!send_cmd(CMD25, sect)) {
			do {
				if (!xmit_datablock(buff, 0xFC))
                                        break;
				buff += 512;
			} while (--count);
			if (!xmit_datablock(0, 0xFD)) /* STOP TRAN token */
                                count = 1;
		}
	}
        cs_high();

	return count ? RES_ERROR : RES_OK;
}
#endif

/**
 * @brief Drive IO controls
 * 
 * @param drv physical drive number (0)
 * @param cmd control command mode
 * @param buff 
 * @return DRESULT 1: OK / 0: ERROR
 */
DRESULT disk_ioctl (BYTE drv, BYTE cmd, void *buff)
{
	DRESULT result;
	BYTE resp, n, csd[16];
	DWORD start, end, csize;
	LBA_t *dp;


	if (drv)
                return RES_PARERR;
	if (stat & STA_NOINIT)
                return RES_NOTRDY;

	result = RES_ERROR;

        cs_low();

	switch (cmd) {
	case CTRL_SYNC: /* Wait unfinished write process of the drive */
		if (wait_ready(100))
                        result = RES_OK;
		break;
	case GET_SECTOR_COUNT:/* Get drive capacity in unit of sector(DWORD) */
		if (!send_cmd(CMD9, 0) && rcvr_datablock(csd, 16)) {
			if ((csd[0] >> 6) == 1) {	/* SD Card CSD ver 2 */
				csize = (csd[9] | ((WORD)csd[8] << 8) |
                                        ((DWORD)(csd[7] & 0x3F) << 16)) + 1;
				*(LBA_t *)buff = csize << 10;
			}
			result = RES_OK;
		}
		break;
	case GET_BLOCK_SIZE:/* Get erase block size in unit of sector(DWORD) */
		if (cardtype & CT_SDC2) {	/* SDC ver 2+ */
                        if (send_cmd(ACMD13, 0)) { /* Get SD status */
                                break;
                        }
                        sd_spi_rx(csd, 1);         /* Purge R2 second byte */
                        if (rcvr_datablock(csd, 16)) {	/* [511:384] */
                                /* Purge trailing data */
                                for (n = 64 - 16; n; n--) {
                                        sd_spi_rx(&resp, 48); 
                                }
                                *(DWORD *)buff = 16UL << (csd[10] >> 4);
                                result = RES_OK;
                        }
                }
		break;
	case CTRL_TRIM : /* Erase a block of sectors */
		if (!(cardtype & CT_SDC)) 
                        break;
		if (disk_ioctl(drv, MMC_GET_CSD, csd))
                        break;
		if (!(csd[10] & 0x40))  /* Check if ERASE_BLK_EN = 1 */
                        break;
		dp = buff;
                start = (DWORD)dp[0];
                end = (DWORD)dp[1];	/* Load sector block */
		if (!(cardtype & CT_BLOCK)) {
			start *= 512;
                        end *= 512;
		}
                if (!send_cmd(CMD32, start))
                        break;            
                if (!send_cmd(CMD33, end))
                        break;
                /* FatFs does not check result of this command */
		if (!send_cmd(CMD38, 0))
                        result = RES_OK;
		break;
	/* Following commands are never used by FatFs module */
	case MMC_GET_TYPE:      /* Get MMC/SDC type (BYTE) */
		*(BYTE*)buff = cardtype;
		result = RES_OK;
		break;
	case MMC_GET_CSD:	/* Read CSD (16 bytes) */
		if (!send_cmd(CMD9, 0) && rcvr_datablock((BYTE *)buff, 16))
			result = RES_OK;
		break;
	case MMC_GET_CID:	/* Read CID (16 bytes) */
                if (!send_cmd(CMD10, 0) && rcvr_datablock((BYTE *)buff, 16))
			result = RES_OK;
		break;
	case MMC_GET_OCR:	/* Read OCR (4 bytes) */   
		if (!send_cmd(CMD58, 0)) {
                        sd_spi_rx((BYTE *)buff, 4);
			result = RES_OK;
		}
		break;
	case MMC_GET_SDSTAT:	/* Read SD status (64 bytes) */
                if (send_cmd(ACMD13, 0))
                        break;
                sd_spi_rx(csd, 1); /* Purge R2 second byte */
		if (rcvr_datablock((BYTE *)buff, 64))
                        result = RES_OK;
		break;
	default:
		result = RES_PARERR;
	}
        cs_high();

	return result;
}