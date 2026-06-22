// SPDX-License-Identifier: no SPDX license
/**
 * @file mgr_at_cmd_list.c
 * @author  Kineis
 * @brief AT commands list main file
 */

/**
 * @addtogroup MGR_SPI_CMD
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include "build_info.h"
#include "mgr_spi_cmd_common.h"
#include "mgr_spi_cmd_list.h"
#include "mgr_spi_cmd_list_general.h"
#include "mgr_spi_cmd_list_mac.h"
#include "mgr_spi_cmd_list_user_data.h"
#include "mgr_spi_cmd_list_previpass.h"
#include "mgr_spi_cmd_list_certif.h"

const char *spicmd_version = "v0.8.1";

/** @attention update AT cmd version above if you add or remove commands in this list */
const struct spicmd_desc_t cas_spicmd_list_array[SPICMD_MAX_COUNT] = {
	/**< General commands */
	{ CMD_NONE, CMD_NONE,			                NULL},
	{ CMD_READ, CMD_NONE,            			    bMGR_SPI_CMD_READ_cmd},
	{ CMD_PING, CMD_NONE,            			    bMGR_SPI_CMD_PING_cmd},
	{ CMD_MAC_STATUS, CMD_NONE,      			    bMGR_SPI_CMD_MACSTATUS_cmd},
	{ CMD_SPI_STATUS, CMD_NONE,      			    bMGR_SPI_CMD_SPISTATE_cmd},
	{ CMD_READ_VERSION, CMD_NONE,    			    bMGR_SPI_CMD_READSPIVERSION_cmd},
	{ CMD_READ_FIRMWARE, CMD_NONE,   			    bMGR_SPI_CMD_READFIRMWARE_cmd},
	{ CMD_READ_ADDR, CMD_NONE,       			    bMGR_SPI_CMD_READADDRESS_cmd},
	{ CMD_READ_ID, CMD_NONE,         			    bMGR_SPI_CMD_READID_cmd},
	{ CMD_READ_SN, CMD_NONE,         			    bMGR_SPI_CMD_READSN_cmd},
	{ CMD_READ_RCONF, CMD_NONE,      			    bMGR_SPI_CMD_READRCONF_cmd},
	{ CMD_WRITE_RCONF_REQ, CMD_WRITE_RCONF,         bMGR_SPI_CMD_WRITERCONFREQ_cmd},
	{ CMD_WRITE_RCONF, CMD_NONE,      			    bMGR_SPI_CMD_WRITERCONF_cmd},
	{ CMD_SAVE_RCONF, CMD_NONE,      			    bMGR_SPI_CMD_SAVERCONF_cmd},
	{ CMD_READ_KMAC, CMD_NONE,      			    bMGR_SPI_CMD_READKMAC_cmd},
	{ CMD_WRITE_KMAC_REQ, CMD_WRITE_KMAC,		    bMGR_SPI_CMD_WRITEKMACREQ_cmd},
	{ CMD_WRITE_KMAC, CMD_NONE,      			    bMGR_SPI_CMD_WRITEKMAC_cmd},
	{ CMD_READ_LPM, CMD_NONE,      				    bMGR_SPI_CMD_READLPM_cmd},
	{ CMD_WRITE_LPM_REQ, CMD_WRITE_LPM,             bMGR_SPI_CMD_WRITELPMREQ_cmd},
	{ CMD_WRITE_LPM, CMD_NONE,      			    bMGR_SPI_CMD_WRITELPM_cmd},
	{ CMD_WRITE_TX_REQ, CMD_WRITE_TX_SIZE,		    bMGR_SPI_CMD_WRITETXREQ_cmd},
	{ CMD_WRITE_TX_SIZE, CMD_WRITE_TX,              bMGR_SPI_CMD_WRITETXSIZE_cmd},
	{ CMD_WRITE_TX, CMD_NONE,      				    bMGR_SPI_CMD_WRITETX_cmd},
	{ CMD_READ_CW, CMD_NONE,            		    bMGR_SPI_CMD_READCW_cmd},
	{ CMD_WRITE_CW_REQ, CMD_WRITE_CW,               bMGR_SPI_CMD_WRITECWREQ_cmd},
	{ CMD_WRITE_CW, CMD_NONE,      				    bMGR_SPI_CMD_WRITECW_cmd},
	{ CMD_READ_PREPASSEN, CMD_NONE,				    bMGR_SPI_CMD_READPREPASSEN_cmd},
	{ CMD_WRITE_PREPASSEN_REQ, CMD_WRITE_PREPASSEN, bMGR_SPI_CMD_WRITEPREPASSENREQ_cmd},
	{ CMD_WRITE_PREPASSEN, CMD_NONE,      		    bMGR_SPI_CMD_WRITEPREPASSEN_cmd},
	{ CMD_READ_UDATE, CMD_NONE,         		    bMGR_SPI_CMD_READUDATE_cmd},
	{ CMD_WRITE_UDATE_REQ, CMD_WRITE_UDATE,         bMGR_SPI_CMD_WRITEUDATEREQ_cmd},
	{ CMD_WRITE_UDATE, CMD_NONE,      		        bMGR_SPI_CMD_WRITEUDATE_cmd},
	{ CMD_WRITE_ID_REQ, CMD_WRITE_ID,     		    bMGR_SPI_CMD_WRITEIDREQ_cmd},
	{ CMD_WRITE_ID, CMD_NONE,     				    bMGR_SPI_CMD_WRITEID_cmd},
	{ CMD_WRITE_ADDR_REQ, CMD_NONE,    			    bMGR_SPI_CMD_WRITEADDRESSREQ_cmd},
	{ CMD_WRITE_ADDR, CMD_NONE,       			    bMGR_SPI_CMD_WRITEADDRESS_cmd},
	{ CMD_READ_SECKEY, CMD_NONE,     				bMGR_SPI_CMD_READSECKEY_cmd},
	{ CMD_WRITE_SECKEY_REQ, CMD_WRITE_SECKEY,	    bMGR_SPI_CMD_WRITESECKEYREQ_cmd},
	{ CMD_WRITE_SECKEY, CMD_NONE,       		    bMGR_SPI_CMD_WRITESECKEY_cmd},
	{ CMD_READ_SPIMAC_STATE, CMD_NONE,       		bMGR_SPI_CMD_READSPIMACSTATE_cmd},
	{ CMD_READ_TCXO_WU, CMD_NONE,         		    bMGR_SPI_CMD_READTCXO_cmd},
	{ CMD_WRITE_TCXOWU_REQ, CMD_WRITE_TCXOWU, 	    bMGR_SPI_CMD_WRITETCXOREQ_cmd},
	{ CMD_WRITE_TCXOWU, CMD_NONE,     				bMGR_SPI_CMD_WRITETCXO_cmd},
	/* Reserved commands 0x2C - 0x3E (gap for future use and alignment with bootloader DFU range) */
	{ CMD_READ_RCONF_RAW, CMD_NONE, bMGR_SPI_CMD_READRCONFRAW_cmd},  /* 0x2B - Read raw RCONF from flash */
	{ CMD_READ_MC, CMD_NONE, bMGR_SPI_CMD_READMC_cmd},              /* 0x2C - Read message counter */
	{ CMD_WRITE_MC_REQ, CMD_WRITE_MC, bMGR_SPI_CMD_WRITEMCREQ_cmd}, /* 0x2D - Write MC request */
	{ CMD_WRITE_MC, CMD_NONE, bMGR_SPI_CMD_WRITEMC_cmd},            /* 0x2E - Write MC value */
	{ CMD_READ_KCFG, CMD_NONE, bMGR_SPI_CMD_READKCFG_cmd},          /* 0x2F - Read stack config bitmap (read-only) */
	{ CMD_NONE, CMD_NONE, NULL},  /* 0x30 - SPI_CMD_DFU_PING in bootloader */
	{ CMD_NONE, CMD_NONE, NULL},  /* 0x31 - SPI_CMD_DFU_GET_INFO */
	{ CMD_NONE, CMD_NONE, NULL},  /* 0x32 - SPI_CMD_DFU_ERASE */
	{ CMD_NONE, CMD_NONE, NULL},  /* 0x33 - SPI_CMD_DFU_WRITE_REQ */
	{ CMD_NONE, CMD_NONE, NULL},  /* 0x34 - SPI_CMD_DFU_WRITE_DATA */
	{ CMD_NONE, CMD_NONE, NULL},  /* 0x35 - SPI_CMD_DFU_READ_REQ */
	{ CMD_NONE, CMD_NONE, NULL},  /* 0x36 - SPI_CMD_DFU_READ_DATA */
	{ CMD_NONE, CMD_NONE, NULL},  /* 0x37 - SPI_CMD_DFU_VERIFY */
	{ CMD_NONE, CMD_NONE, NULL},  /* 0x38 - SPI_CMD_DFU_RESET */
	{ CMD_NONE, CMD_NONE, NULL},  /* 0x39 - SPI_CMD_DFU_JUMP */
	{ CMD_NONE, CMD_NONE, NULL},  /* 0x3A - SPI_CMD_DFU_GET_STATUS */
	{ CMD_NONE, CMD_NONE, NULL},  /* 0x3B - SPI_CMD_DFU_ABORT */
	{ CMD_NONE, CMD_NONE, NULL},  /* 0x3C - SPI_CMD_DFU_SET_HEADER */
	{ CMD_NONE, CMD_NONE, NULL},  /* 0x3D */
	{ CMD_NONE, CMD_NONE, NULL},  /* 0x3E */
	/* DFU Enter command - triggers reboot to bootloader */
	{ CMD_DFU_ENTER, CMD_NONE, bMGR_SPI_CMD_DFU_ENTER_cmd},  /* 0x3F */
};

/**
 * @}
 */
