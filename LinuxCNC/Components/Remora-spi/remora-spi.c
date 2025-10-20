/********************************************************************
* Description:  remora-spi.c
*               This file, 'remora-rpispi.c', is a HAL component that
*               provides and SPI connection to a external STM32 running Remora PRU firmware.
*  				
*		Initially developed for RaspberryPi -> Arduino Due.
*		Further developed for RaspberryPi -> Smoothieboard and clones (LPC1768).
                Even further developed for RaspberryPi -> STM32 boards
*
* Author: Scott Alford
* License: GPL Version 3
*
*		Credit to GP Orcullo and PICnc V2 which originally inspired this
*		and portions of this code is based on stepgen.c by John Kasunich
*		and hm2_rpspi.c by Matsche
*
* Copyright (c) 2024	All rights reserved.
*
* Last change: updated for RPi5 with RP1 southbridge
********************************************************************/


#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"		/* HAL public API decls */

#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>


// Include these in the source directory when using "halcompile --install remora-spi.c"

// Using BCM2835 driver library by Mike McCauley, why reinvent the wheel!
// http://www.airspayce.com/mikem/bcm2835/index.html
#include "bcm2835.h"
#include "bcm2835.c"

// Raspberry Pi 5 uses the RP1
#include "rp1lib.h"
#include "rp1lib.c"
#include "gpiochip_rp1.h"
#include "gpiochip_rp1.c"
#include "spi-dw.h"
#include "spi-dw.c"

#include "dtcboards.h"

#include "remora.h"

#define MODNAME "remora-spi"
#define PREFIX "remora"

MODULE_AUTHOR("Scott Alford AKA scotta");
MODULE_DESCRIPTION("Driver for Remora STM32 control boards")
MODULE_LICENSE("GPL v3");

#define RPI5_RP1_PERI_BASE 0x7c000000

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

typedef struct {
	hal_bit_t		*SPIenable;
	hal_bit_t		*SPIreset;
	hal_bit_t		*PRUreset;
	bool			SPIresetOld;
	hal_bit_t		*SPIstatus;
	hal_bit_t 		*stepperEnable[JOINTS];
	int				pos_mode[JOINTS];
	hal_float_t 	*pos_cmd[JOINTS];			// pin: position command (position units)
	hal_float_t 	*vel_cmd[JOINTS];			// pin: velocity command (position units/sec)
	hal_float_t 	*pos_fb[JOINTS];			// pin: position feedback (position units)
	hal_s32_t		*count[JOINTS];				// pin: psition feedback (raw counts)
	hal_float_t 	pos_scale[JOINTS];			// param: steps per position unit
	float 			freq[JOINTS];				// param: frequency command sent to PRU
	hal_float_t 	*freq_cmd[JOINTS];			// pin: frequency command monitoring, available in LinuxCNC
	hal_float_t 	maxvel[JOINTS];				// param: max velocity, (pos units/sec)
	hal_float_t 	maxaccel[JOINTS];			// param: max accel (pos units/sec^2)
	hal_float_t		*pgain[JOINTS];
	hal_float_t		*ff1gain[JOINTS];
	hal_float_t		*deadband[JOINTS];
	//float 			old_pos_cmd[JOINTS];		// previous position command (counts)
	//float 			old_pos_cmd_raw[JOINTS];		// previous position command (counts)
	float 			old_scale[JOINTS];			// stored scale value
	float 			scale_recip[JOINTS];		// reciprocal value used for scaling
	float			prev_cmd[JOINTS];
	float			cmd_d[JOINTS];					// command derivative
	hal_float_t 	*setPoint[VARIABLES];
	hal_float_t 	*processVariable[VARIABLES];
	hal_bit_t   	*outputs[DIGITAL_OUTPUTS];
	hal_bit_t   	*inputs[DIGITAL_INPUTS*2];
} data_t;

static data_t *data;

#pragma pack(push, 1)

typedef union
{
  // this allow structured access to the outgoing SPI data without having to move it
  // this is the same structure as the PRU rxData structure
  struct
  {
    uint8_t txBuffer[SPIBUFSIZE];
  };
  struct
  {
	int32_t header;
    int32_t jointFreqCmd[JOINTS];
    float 	setPoint[VARIABLES];
	uint8_t jointEnable;
	uint16_t outputs;
    uint8_t spare0;
  };
} txData_t;

static txData_t txData;


typedef union
{
  // this allow structured access to the incoming SPI data without having to move it
  // this is the same structure as the PRU txData structure
  struct
  {
    uint8_t rxBuffer[SPIBUFSIZE];
  };
  struct
  {
    int32_t header;
    int32_t jointFeedback[JOINTS];
    float 	processVariable[VARIABLES];
    uint16_t inputs;
  };
} rxData_t;

#pragma pack(pop)
static rxData_t rxData;



/* other globals */
static int 			comp_id;				// component ID
static const char 	*modname = MODNAME;
static const char 	*prefix = PREFIX;

static bool			bcm;					// use BCM2835 driver
static bool			rp1;					// use RP1 driver
static bool			h618;					// use H618 driver

static int 			num_chan = 0;			// number of step generators configured
static long 		old_dtns;				// update_freq function period in nsec - (THIS IS RUNNING IN THE PI)
static double		dt;						// update_freq period in seconds  - (THIS IS RUNNING IN THE PI)
static double 		recip_dt;				// recprocal of period, avoids divides

static int64_t 		accum[JOINTS] = { 0 };
static int32_t 		count[JOINTS] = { 0 };
static int32_t 		old_count[JOINTS] = { 0 };
static int8_t		filter_count[JOINTS] = { 0 };
static int32_t		accum_diff = 0;

typedef enum CONTROL { POSITION, VELOCITY, INVALID } CONTROL;

char *ctrl_type[JOINTS] = { "p" };
RTAPI_MP_ARRAY_STRING(ctrl_type,JOINTS,"control type (pos or vel)");

int PRU_base_freq = -1;
RTAPI_MP_INT(PRU_base_freq, "PRU base thread frequency");

// for BCM based SPI (Raspberry Pi 5)
int SPI_clk_div = -1;
RTAPI_MP_INT(SPI_clk_div, "SPI clock divider");

// for RP1 based SPI (Raspberry Pi 5)
int SPI_num = -1;
RTAPI_MP_INT(SPI_num, "SPI number");

int CS_num = -1;
RTAPI_MP_INT(CS_num, "CS number");

int32_t SPI_freq = -1;
RTAPI_MP_INT(SPI_freq, "SPI frequency");

static int reset_gpio_pin = 233;				// RPI GPIO pin number used to force watchdog reset of the PRU 




/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/
static int rt_peripheral_init(void);
static int rt_bcm2835_init(void);
static int rt_rp1lib_init(void);

static void update_freq(void *arg, long period);
static void spi_write();
static void spi_read();
static void spi_transfer();
static CONTROL parse_ctrl_type(const char *ctrl);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    char name[HAL_NAME_LEN + 1];
	int n, retval;

	for (n = 0; n < JOINTS; n++) {
		if(parse_ctrl_type(ctrl_type[n]) == INVALID) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"STEPGEN: ERROR: bad control type '%s' for axis %i (must be 'p' or 'v')\n",
					ctrl_type[n], n);
			return -1;
		}
    }

	
	// check to see PRU chip type has been set at the command line
	/*
	if (!strcmp(chip_type, "LPC") || !strcmp(chip_type, "lpc"))
	{
		rtapi_print_msg(RTAPI_MSG_INFO,"PRU: Chip type set to LPC\n");
		chip = LPC;
	}
	else if (!strcmp(chip_type, "STM") || !strcmp(chip_type, "stm"))
	{
		rtapi_print_msg(RTAPI_MSG_INFO,"PRU: Chip type set to STM\n");
		chip = STM;
	}
	else
	{
		rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: PRU chip type (must be 'LPC' or 'STM')\n");
		return -1;
	}
	*/
	
	// check to see if the PRU base frequency has been set at the command line
	if (PRU_base_freq != -1)
	{
		if ((PRU_base_freq < 40000) || (PRU_base_freq > 240000))
		{
			rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: PRU base frequency incorrect\n");
			return -1;
		}
	}
	else
	{
		PRU_base_freq = PRU_BASEFREQ;
	}
	

    // connect to the HAL, initialise the driver
    comp_id = hal_init(modname);
    if (comp_id < 0)
	{
		rtapi_print_msg(RTAPI_MSG_ERR, "%s ERROR: hal_init() failed \n", modname);
		return -1;
    }

	// allocate shared memory
	data = hal_malloc(sizeof(data_t));
	if (data == 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			"%s: ERROR: hal_malloc() failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}
	
	bcm = false;
	rp1 = false;
	
	// initialise the gpio and spi peripherals
	if(!rt_peripheral_init())
	{
	  rtapi_print_msg(RTAPI_MSG_ERR,"rt_peripheral_init failed.\n");
      return -1;
		
	}

	// export remoraPRU SPI enable and status bits
	retval = hal_pin_bit_newf(HAL_IN, &(data->SPIenable),
			comp_id, "%s.SPI-enable", prefix);
	if (retval != 0) goto error;
	
	retval = hal_pin_bit_newf(HAL_IN, &(data->SPIreset),
			comp_id, "%s.SPI-reset", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_OUT, &(data->SPIstatus),
			comp_id, "%s.SPI-status", prefix);
	if (retval != 0) goto error;

	if (bcm == true)
	{
		bcm2835_gpio_fsel(reset_gpio_pin, BCM2835_GPIO_FSEL_OUTP);

	}
	else if (rp1 == true)
	{
		gpio_set_fsel(reset_gpio_pin, GPIO_FSEL_OUTPUT);
	}
	else if (h618 == true)
	{
		// 设置H618的reset GPIO引脚为输出
		h618_gpio_set_fsel(reset_gpio_pin, 1);  // 1表示输出模式
	}
	
	retval = hal_pin_bit_newf(HAL_IN, &(data->PRUreset),
			comp_id, "%s.PRU-reset", prefix);
	if (retval != 0) goto error;

    // export all the variables for each joint
    for (n = 0; n < JOINTS; n++) {
		// export pins

		data->pos_mode[n] = (parse_ctrl_type(ctrl_type[n]) == POSITION);
/*
This is throwing errors from axis.py for some reason...
		
		if (data->pos_mode[n]){
			rtapi_print_msg(RTAPI_MSG_ERR, "Creating pos_mode[%d] = %d\n", n, data->pos_mode[n]);
			retval = hal_pin_float_newf(HAL_IN, &(data->pos_cmd[n]),
					comp_id, "%s.joint.%01d.pos-cmd", prefix, n);
			if (retval < 0) goto error;
			*(data->pos_cmd[n]) = 0.0;
		} else {
			rtapi_print_msg(RTAPI_MSG_ERR, "Creating vel_mode[%d] = %d\n", n, data->pos_mode[n]);
			retval = hal_pin_float_newf(HAL_IN, &(data->vel_cmd[n]),
					comp_id, "%s.joint.%01d.vel-cmd", prefix, n);
			if (retval < 0) goto error;
			*(data->vel_cmd[n]) = 0.0;			
		}
*/

		retval = hal_pin_bit_newf(HAL_IN, &(data->stepperEnable[n]),
				comp_id, "%s.joint.%01d.enable", prefix, n);
		if (retval != 0) goto error;

		retval = hal_pin_float_newf(HAL_IN, &(data->pos_cmd[n]),
				comp_id, "%s.joint.%01d.pos-cmd", prefix, n);
		if (retval < 0) goto error;
		*(data->pos_cmd[n]) = 0.0;
		
		if (data->pos_mode[n] == 0){
			retval = hal_pin_float_newf(HAL_IN, &(data->vel_cmd[n]),
					comp_id, "%s.joint.%01d.vel-cmd", prefix, n);
			if (retval < 0) goto error;
			*(data->vel_cmd[n]) = 0.0;			
		}

		retval = hal_pin_float_newf(HAL_OUT, &(data->freq_cmd[n]),
		        comp_id, "%s.joint.%01d.freq-cmd", prefix, n);
		if (retval < 0) goto error;
		*(data->freq_cmd[n]) = 0.0;

		retval = hal_pin_float_newf(HAL_OUT, &(data->pos_fb[n]),
		        comp_id, "%s.joint.%01d.pos-fb", prefix, n);
		if (retval < 0) goto error;
		*(data->pos_fb[n]) = 0.0;
		
		retval = hal_param_float_newf(HAL_RW, &(data->pos_scale[n]),
		        comp_id, "%s.joint.%01d.scale", prefix, n);
		if (retval < 0) goto error;
		data->pos_scale[n] = 1.0;

		retval = hal_pin_s32_newf(HAL_OUT, &(data->count[n]),
		        comp_id, "%s.joint.%01d.counts", prefix, n);
		if (retval < 0) goto error;
		*(data->count[n]) = 0;
		
		retval = hal_pin_float_newf(HAL_IN, &(data->pgain[n]),
				comp_id, "%s.joint.%01d.pgain", prefix, n);
		if (retval < 0) goto error;
		*(data->pgain[n]) = 0.0;
		
		retval = hal_pin_float_newf(HAL_IN, &(data->ff1gain[n]),
				comp_id, "%s.joint.%01d.ff1gain", prefix, n);
		if (retval < 0) goto error;
		*(data->ff1gain[n]) = 0.0;
		
		retval = hal_pin_float_newf(HAL_IN, &(data->deadband[n]),
				comp_id, "%s.joint.%01d.deadband", prefix, n);
		if (retval < 0) goto error;
		*(data->deadband[n]) = 0.0;
		
		retval = hal_param_float_newf(HAL_RW, &(data->maxaccel[n]),
		        comp_id, "%s.joint.%01d.maxaccel", prefix, n);
		if (retval < 0) goto error;
		data->maxaccel[n] = 1.0;
	}

	for (n = 0; n < VARIABLES; n++) {
	// export pins

		retval = hal_pin_float_newf(HAL_IN, &(data->setPoint[n]),
		        comp_id, "%s.SP.%01d", prefix, n);
		if (retval < 0) goto error;
		*(data->setPoint[n]) = 0.0;

		retval = hal_pin_float_newf(HAL_OUT, &(data->processVariable[n]),
		        comp_id, "%s.PV.%01d", prefix, n);
		if (retval < 0) goto error;
		*(data->processVariable[n]) = 0.0;
	}

	for (n = 0; n < DIGITAL_OUTPUTS; n++) {
		retval = hal_pin_bit_newf(HAL_IN, &(data->outputs[n]),
				comp_id, "%s.output.%02d", prefix, n);
		if (retval != 0) goto error;
		*(data->outputs[n])=0;
	}

	for (n = 0; n < DIGITAL_INPUTS; n++) {
		retval = hal_pin_bit_newf(HAL_OUT, &(data->inputs[n]),
				comp_id, "%s.input.%02d", prefix, n);
		if (retval != 0) goto error;
		*(data->inputs[n])=0;
			
		retval = hal_pin_bit_newf(HAL_OUT, &(data->inputs[n+DIGITAL_INPUTS]),
				comp_id, "%s.input.%02d.not", prefix, n);
		if (retval != 0) goto error;
		*(data->inputs[n+DIGITAL_INPUTS])=1;   
	}

	error:
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: pin export failed with err=%i\n",
		        modname, retval);
		hal_exit(comp_id);
		return -1;
	}


	// Export functions
	rtapi_snprintf(name, sizeof(name), "%s.update-freq", prefix);
	retval = hal_export_funct(name, update_freq, data, 1, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: update function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}

	rtapi_snprintf(name, sizeof(name), "%s.write", prefix);
	/* no FP operations */
	retval = hal_export_funct(name, spi_write, 0, 0, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: write function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}

	rtapi_snprintf(name, sizeof(name), "%s.read", prefix);
	retval = hal_export_funct(name, spi_read, data, 1, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: read function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}

	rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
	hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    // 清理H618资源
    if (h618 == true) {
        h618_spi_cleanup();
    }
    hal_exit(comp_id);
}


/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

int rt_peripheral_init(void)
{
	FILE *fp;
    int i, j;
    char buf[256];
    ssize_t buflen;
    char *cptr;
    const int DTC_MAX = 8;
    const char *dtcs[DTC_MAX + 1];
    
    // 首先检查是否是H618平台
    if (h618_detect()) {
        h618 = true;
        rtapi_print_msg(RTAPI_MSG_INFO, "H618 platform detected, using H618 driver\n");
        
        // 初始化H618 SPI
        if (SPI_num == -1) SPI_num = 0; // 默认使用SPI0
        if (CS_num == -1) CS_num = 0;   // 默认使用CS0
        if (SPI_freq == -1) SPI_freq = 20000000; // 默认20MHz
        
        if (!h618_spi_init(SPI_num, CS_num, H618_SPI_MODE_0, SPI_freq)) {
            rtapi_print_msg(RTAPI_MSG_ERR, "H618 SPI initialization failed\n");
            return -1;
        }
        
        return 1;
    }
    
    // 检查是否是树莓派平台
    if ((fp = fopen("/proc/device-tree/compatible" , "rb"))){

        // Read the 'compatible' string-list from the device-tree
        buflen = fread(buf, 1, sizeof(buf), fp);
        if(buflen < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,"Failed to read platform identity.\n");
            return -1;
        }

        // Decompose the device-tree buffer into a string-list with the pointers to
        // each string in dtcs. Don't go beyond the buffer's size.
        memset(dtcs, 0, sizeof(dtcs));
        for(i = 0, cptr = buf; i < DTC_MAX && cptr; i++) {
            dtcs[i] = cptr;
            j = strlen(cptr);
            if((cptr - buf) + j + 1 < buflen)
                cptr += j + 1;
            else
                cptr = NULL;
        }

        for(i = 0; dtcs[i] != NULL; i++) {
            if(        !strcmp(dtcs[i], DTC_RPI_MODEL_4B)
                ||    !strcmp(dtcs[i], DTC_RPI_MODEL_4CM)
                ||    !strcmp(dtcs[i], DTC_RPI_MODEL_400)
                ||    !strcmp(dtcs[i], DTC_RPI_MODEL_3BP)
                ||    !strcmp(dtcs[i], DTC_RPI_MODEL_3AP)
                ||    !strcmp(dtcs[i], DTC_RPI_MODEL_3B)) {
                rtapi_print_msg(RTAPI_MSG_ERR, "Raspberry Pi 3 or 4, using BCM2835 driver\n");
                bcm = true;
                break;    // Found our supported board
            } else if(!strcmp(dtcs[i], DTC_RPI_MODEL_5B) || !strcmp(dtcs[i], DTC_RPI_MODEL_5CM)) {
                rtapi_print_msg(RTAPI_MSG_ERR, "Raspberry Pi 5, using rp1 driver\n");
                rp1 = true;
                break;    // Found our supported board
            } else {
                rtapi_print_msg(RTAPI_MSG_ERR, "Error, RPi not detected\n");
                return -1;
            }
        }
        fclose(fp);
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR,"Cannot open '/proc/device-tree/compatible' for read.\n");
    }    	  	
        
	if (bcm == true)
	{
		// Map the RPi BCM2835 peripherals - uses "rtapi_open_as_root" in place of "open"
		if (!rt_bcm2835_init())
		{
		  rtapi_print_msg(RTAPI_MSG_ERR,"rt_bcm2835_init failed. Are you running with root privlages??\n");
		  return -1;
		}

		// Set the SPI0 pins to the Alt 0 function to enable SPI0 access, setup CS register
		// and clear TX and RX fifos
		if (!bcm2835_spi_begin())
		{
		  rtapi_print_msg(RTAPI_MSG_ERR,"bcm2835_spi_begin failed. Are you running with root privlages??\n");
		  return -1;
		}

		// Configure SPI0
		bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
		bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default

		//bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128);		// 3.125MHz on RPI3
		//bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);		// 6.250MHz on RPI3
		//bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_32);		// 12.5MHz on RPI3
		//bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16);		// 25MHz on RPI3
		
		// check if the default SPI clock divider has been overriden at the command line
		if (SPI_clk_div != -1)
		{
			// check that the setting is a power of 2
			if ((SPI_clk_div & (SPI_clk_div - 1)) == 0)
			{
				bcm2835_spi_setClockDivider(SPI_clk_div);
				rtapi_print_msg(RTAPI_MSG_INFO,"PRU: SPI clk divider overridden and set to %d\n", SPI_clk_div);			
			}
			else
			{
				// it's not a power of 2
				rtapi_print_msg(RTAPI_MSG_ERR,"ERROR: PRU SPI clock divider incorrect\n");
				return -1;
			}	
		}
		else
		{
			bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16);
			rtapi_print_msg(RTAPI_MSG_INFO,"PRU: SPI default clk divider set to 16\n");
		}

		bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
		bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default


		/* RPI_GPIO_P1_19        = 10 		MOSI when SPI0 in use
		 * RPI_GPIO_P1_21        =  9 		MISO when SPI0 in use
		 * RPI_GPIO_P1_23        = 11 		CLK when SPI0 in use
		 * RPI_GPIO_P1_24        =  8 		CE0 when SPI0 in use
		 * RPI_GPIO_P1_26        =  7 		CE1 when SPI0 in use
		 */

		// Configure pullups on SPI0 pins - source termination and CS high (does this allows for higher clock frequencies??? wiring is more important here)
		bcm2835_gpio_set_pud(RPI_GPIO_P1_19, BCM2835_GPIO_PUD_DOWN);	// MOSI
		bcm2835_gpio_set_pud(RPI_GPIO_P1_21, BCM2835_GPIO_PUD_DOWN);	// MISO
		bcm2835_gpio_set_pud(RPI_GPIO_P1_24, BCM2835_GPIO_PUD_UP);		// CS0			
	}
	else if (rp1 == true)
	{
		if (!rt_rp1lib_init())
		{
			rtapi_print_msg(RTAPI_MSG_ERR,"rt_rp1_init failed.\n");
			return -1;
		}

		if (SPI_num == -1) SPI_num = 0; // default to SPI0
		if (CS_num == -1) CS_num = 0; // default to CS0
		if (SPI_freq == -1) SPI_freq = 20000000; // default to 20MHz
		
		if (!rp1spi_init(SPI_num, CS_num, SPI_MODE_0, SPI_freq))  // SPIx, CSx, mode, freq
		{
			rtapi_print_msg(RTAPI_MSG_ERR,"rp1spi_init failed.\n");
			return -1;
		}
	}
	else
	{
		return -1;
	}

}

// This is the same as the standard bcm2835 library except for the use of
// "rtapi_open_as_root" in place of "open"

int rt_bcm2835_init(void)
{
    int  memfd;
    int  ok;
    FILE *fp;

    if (debug) 
    {
        bcm2835_peripherals = (uint32_t*)BCM2835_PERI_BASE;

	bcm2835_pads = bcm2835_peripherals + BCM2835_GPIO_PADS/4;
	bcm2835_clk  = bcm2835_peripherals + BCM2835_CLOCK_BASE/4;
	bcm2835_gpio = bcm2835_peripherals + BCM2835_GPIO_BASE/4;
	bcm2835_pwm  = bcm2835_peripherals + BCM2835_GPIO_PWM/4;
	bcm2835_spi0 = bcm2835_peripherals + BCM2835_SPI0_BASE/4;
	bcm2835_bsc0 = bcm2835_peripherals + BCM2835_BSC0_BASE/4;
	bcm2835_bsc1 = bcm2835_peripherals + BCM2835_BSC1_BASE/4;
	bcm2835_st   = bcm2835_peripherals + BCM2835_ST_BASE/4;
	bcm2835_aux  = bcm2835_peripherals + BCM2835_AUX_BASE/4;
	bcm2835_spi1 = bcm2835_peripherals + BCM2835_SPI1_BASE/4;

	return 1; /* Success */
    }

    /* Figure out the base and size of the peripheral address block
    // using the device-tree. Required for RPi2/3/4, optional for RPi 1
    */
    if ((fp = fopen(BMC2835_RPI2_DT_FILENAME , "rb")))
    {
        unsigned char buf[16];
        uint32_t base_address;
        uint32_t peri_size;
        if (fread(buf, 1, sizeof(buf), fp) >= 8)
        {
            base_address = (buf[4] << 24) |
              (buf[5] << 16) |
              (buf[6] << 8) |
              (buf[7] << 0);
            
            peri_size = (buf[8] << 24) |
              (buf[9] << 16) |
              (buf[10] << 8) |
              (buf[11] << 0);
            
            if (!base_address)
            {
                /* looks like RPI 4 */
                base_address = (buf[8] << 24) |
                      (buf[9] << 16) |
                      (buf[10] << 8) |
                      (buf[11] << 0);
                      
                peri_size = (buf[12] << 24) |
                (buf[13] << 16) |
                (buf[14] << 8) |
                (buf[15] << 0);
            }
            /* check for valid known range formats */
            if ((buf[0] == 0x7e) &&
                    (buf[1] == 0x00) &&
                    (buf[2] == 0x00) &&
                    (buf[3] == 0x00) &&
                    ((base_address == BCM2835_PERI_BASE) || (base_address == BCM2835_RPI2_PERI_BASE) || (base_address == BCM2835_RPI4_PERI_BASE)))
            {
                bcm2835_peripherals_base = (off_t)base_address;
                bcm2835_peripherals_size = (size_t)peri_size;
                if( base_address == BCM2835_RPI4_PERI_BASE )
                {
                    pud_type_rpi4 = 1;
                }
            }
        
        }
        
	fclose(fp);
    }
    /* else we are prob on RPi 1 with BCM2835, and use the hardwired defaults */

    /* Now get ready to map the peripherals block 
     * If we are not root, try for the new /dev/gpiomem interface and accept
     * the fact that we can only access GPIO
     * else try for the /dev/mem interface and get access to everything
     */
    memfd = -1;
    ok = 0;
    if (geteuid() == 0)
    {
      /* Open the master /dev/mem device */
      if ((memfd = rtapi_open_as_root("/dev/mem", O_RDWR | O_SYNC) ) < 0) 
	{
	  fprintf(stderr, "bcm2835_init: Unable to open /dev/mem: %s\n",
		  strerror(errno)) ;
	  goto exit;
	}
      
      /* Base of the peripherals block is mapped to VM */
      bcm2835_peripherals = mapmem("gpio", bcm2835_peripherals_size, memfd, bcm2835_peripherals_base);
      if (bcm2835_peripherals == MAP_FAILED) goto exit;
      
      /* Now compute the base addresses of various peripherals, 
      // which are at fixed offsets within the mapped peripherals block
      // Caution: bcm2835_peripherals is uint32_t*, so divide offsets by 4
      */
      bcm2835_gpio = bcm2835_peripherals + BCM2835_GPIO_BASE/4;
      bcm2835_pwm  = bcm2835_peripherals + BCM2835_GPIO_PWM/4;
      bcm2835_clk  = bcm2835_peripherals + BCM2835_CLOCK_BASE/4;
      bcm2835_pads = bcm2835_peripherals + BCM2835_GPIO_PADS/4;
      bcm2835_spi0 = bcm2835_peripherals + BCM2835_SPI0_BASE/4;
      bcm2835_bsc0 = bcm2835_peripherals + BCM2835_BSC0_BASE/4; /* I2C */
      bcm2835_bsc1 = bcm2835_peripherals + BCM2835_BSC1_BASE/4; /* I2C */
      bcm2835_st   = bcm2835_peripherals + BCM2835_ST_BASE/4;
      bcm2835_aux  = bcm2835_peripherals + BCM2835_AUX_BASE/4;
      bcm2835_spi1 = bcm2835_peripherals + BCM2835_SPI1_BASE/4;

      ok = 1;
    }
    else
    {
      /* Not root, try /dev/gpiomem */
      /* Open the master /dev/mem device */
      if ((memfd = open("/dev/gpiomem", O_RDWR | O_SYNC) ) < 0) 
	{
	  fprintf(stderr, "bcm2835_init: Unable to open /dev/gpiomem: %s\n",
		  strerror(errno)) ;
	  goto exit;
	}
      
      /* Base of the peripherals block is mapped to VM */
      bcm2835_peripherals_base = 0;
      bcm2835_peripherals = mapmem("gpio", bcm2835_peripherals_size, memfd, bcm2835_peripherals_base);
      if (bcm2835_peripherals == MAP_FAILED) goto exit;
      bcm2835_gpio = bcm2835_peripherals;
      ok = 1;
    }

exit:
    if (memfd >= 0)
        close(memfd);

    if (!ok)
	bcm2835_close();

    return ok;
}

int rt_rp1lib_init(void)
{
    uint64_t phys_addr = RP1_BAR1;

    DEBUG_PRINT("Initialising RP1 library: %s\n", __func__);

    // rp1_chip is declared in gpiochip_rp1.c
    chip = &rp1_chip;

    inst = rp1_create_instance(chip, phys_addr, NULL);
    if (!inst)
        return -1;

    inst->phys_addr = phys_addr;

    // map memory
    inst->mem_fd = rtapi_open_as_root("/dev/mem", O_RDWR | O_SYNC);
    if (inst->mem_fd < 0)
        return errno;

    inst->priv = mmap(
        NULL,
        RP1_BAR1_LEN,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        inst->mem_fd,
        inst->phys_addr
        );

    DEBUG_PRINT("Base address: %11lx, size: %lx, mapped at address: %p\n", inst->phys_addr, RP1_BAR1_LEN, inst->priv);

    if (inst->priv == MAP_FAILED)
        return errno;

    return 1;
}


void update_freq(void *arg, long period)
{
	int i;
	data_t *data = (data_t *)arg;
	double max_ac, vel_cmd, dv, new_vel, max_freq, desired_freq;
		   
	double error, command, feedback;
	double periodfp, periodrecip;
	float pgain, ff1gain, deadband;

	// precalculate timing constants
    periodfp = period * 0.000000001;
    periodrecip = 1.0 / periodfp;

    // calc constants related to the period of this function (LinuxCNC SERVO_THREAD)
    // only recalc constants if period changes
    if (period != old_dtns) 			// Note!! period = LinuxCNC SERVO_PERIOD
	{
		old_dtns = period;				// get ready to detect future period changes
		dt = period * 0.000000001; 		// dt is the period of this thread, used for the position loop
		recip_dt = 1.0 / dt;			// calc the reciprocal once here, to avoid multiple divides later
    }

    // loop through generators
	for (i = 0; i < JOINTS; i++)
	{
		// check for scale change
		if (data->pos_scale[i] != data->old_scale[i])
		{
			data->old_scale[i] = data->pos_scale[i];		// get ready to detect future scale changes
			// scale must not be 0
			if ((data->pos_scale[i] < 1e-20) && (data->pos_scale[i] > -1e-20))	// validate the new scale value
				data->pos_scale[i] = 1.0;										// value too small, divide by zero is a bad thing
				// we will need the reciprocal, and the accum is fixed point with
				//fractional bits, so we precalc some stuff
			data->scale_recip[i] = (1.0 / STEP_MASK) / data->pos_scale[i];
		}

		// calculate frequency limit
		//max_freq = PRU_BASEFREQ/(2.0); 	
		max_freq = PRU_base_freq; // step pulses now happen in a single base thread interval


		// check for user specified frequency limit parameter
		if (data->maxvel[i] <= 0.0)
		{
			// set to zero if negative
			data->maxvel[i] = 0.0;
		}
		else
		{
			// parameter is non-zero, compare to max_freq
			desired_freq = data->maxvel[i] * fabs(data->pos_scale[i]);

			if (desired_freq > max_freq)
			{
				// parameter is too high, limit it
				data->maxvel[i] = max_freq / fabs(data->pos_scale[i]);
			}
			else
			{
				// lower max_freq to match parameter
				max_freq = data->maxvel[i] * fabs(data->pos_scale[i]);
			}
		}
		
		/* set internal accel limit to its absolute max, which is
		zero to full speed in one thread period */
		max_ac = max_freq * recip_dt;
		
		// check for user specified accel limit parameter
		if (data->maxaccel[i] <= 0.0)
		{
			// set to zero if negative
			data->maxaccel[i] = 0.0;
		}
		else 
		{
			// parameter is non-zero, compare to max_ac
			if ((data->maxaccel[i] * fabs(data->pos_scale[i])) > max_ac)
			{
				// parameter is too high, lower it
				data->maxaccel[i] = max_ac / fabs(data->pos_scale[i]);
			}
			else
			{
				// lower limit to match parameter
				max_ac = data->maxaccel[i] * fabs(data->pos_scale[i]);
			}
		}

		/* at this point, all scaling, limits, and other parameter
		changes have been handled - time for the main control */

		

		if (data->pos_mode[i]) {

			/* POSITION CONTROL MODE */

			// use Proportional control with feed forward (pgain, ff1gain and deadband)
			
			if (*(data->pgain[i]) != 0)
			{
				pgain = *(data->pgain[i]);
			}
			else
			{
				pgain = 1.0;
			}
			
			if (*(data->ff1gain[i]) != 0)
			{
				ff1gain = *(data->ff1gain[i]);
			}
			else
			{
				ff1gain = 1.0;
			}
			
			if (*(data->deadband[i]) != 0)
			{
				deadband = *(data->deadband[i]);
			}
			else
			{
				deadband = 1 / data->pos_scale[i];
			}	

			// read the command and feedback
			command = *(data->pos_cmd[i]);
			feedback = *(data->pos_fb[i]);
			
			// calcuate the error
			error = command - feedback;
			
			// apply the deadband
			if (error > deadband)
			{
				error -= deadband;
			}
			else if (error < -deadband)
			{
				error += deadband;
			}
			else
			{
				error = 0;
			}
			
			// calcuate command and derivatives
			data->cmd_d[i] = (command - data->prev_cmd[i]) * periodrecip;
			
			// save old values
			data->prev_cmd[i] = command;
				
			// calculate the output value
			vel_cmd = pgain * error + data->cmd_d[i] * ff1gain;
		
		} else {

			/* VELOCITY CONTROL MODE */
			
			// calculate velocity command in counts/sec
			vel_cmd = *(data->vel_cmd[i]);
		}	
			
		vel_cmd = vel_cmd * data->pos_scale[i];
			
		// apply frequency limit
		if (vel_cmd > max_freq) 
		{
			vel_cmd = max_freq;
		} 
		else if (vel_cmd < -max_freq) 
		{
			vel_cmd = -max_freq;
		}
		
		// calc max change in frequency in one period
		dv = max_ac * dt;
		
		// apply accel limit
		if ( vel_cmd > (data->freq[i] + dv) )
		{
			new_vel = data->freq[i] + dv;
		} 
		else if ( vel_cmd < (data->freq[i] - dv) ) 
		{
			new_vel = data->freq[i] - dv;
		}
		else
		{
			new_vel = vel_cmd;
		}
		
		// test for disabled stepgen
		if (*data->stepperEnable == 0) {
			// set velocity to zero
			new_vel = 0; 
		}
		
		data->freq[i] = new_vel;		// to be sent to the PRU
		*(data->freq_cmd[i]) = data->freq[i];	// feedback to LinuxCNC
	}

}


void spi_read()
{
	int i;
	double curr_pos;
	
	// following error spike filter pramaters
	int n = 2;
	int M = 250;

	// Data header
	txData.header = PRU_READ;
	
	// update the PRUreset output
	// TODO: fix this up to include RP1
	
	if (*(data->PRUreset))
	{ 
		if (bcm == true)
		{
			bcm2835_gpio_set(reset_gpio_pin);
		}
		else if (rp1 == true)
		{
			gpio_set(reset_gpio_pin);
		}
		else if (h618 == true)
		{
			h618_gpio_set(reset_gpio_pin);
		}
    }
	else
	{
		if (bcm == true)
		{
			bcm2835_gpio_clr(reset_gpio_pin);
		}
		else if (rp1 == true)
		{
			gpio_clear(reset_gpio_pin);
		}
		else if (h618 == true)
		{
			h618_gpio_clear(reset_gpio_pin);
		}
    }
	
	
	if (*(data->SPIenable))
	{
		if( (*(data->SPIreset) && !(data->SPIresetOld)) || *(data->SPIstatus) )
		{
			// reset rising edge detected, try SPI transfer and reset OR PRU running
			
			// Transfer to and from the PRU
			spi_transfer();

			switch (rxData.header)		// only process valid SPI payloads. This rejects bad payloads
			{
				case PRU_DATA:
					// we have received a GOOD payload from the PRU
					*(data->SPIstatus) = 1;

					for (i = 0; i < JOINTS; i++)
					{
						// the PRU DDS accumulator uses 32 bit counter, this code converts that counter into 64 bits */
						old_count[i] = count[i];
						count[i] = rxData.jointFeedback[i];
						accum_diff = count[i] - old_count[i];
						
						// spike filter
						if (abs(count[i] - old_count[i]) > M && filter_count[i] < n)
						{
							// recent big change: hold previous value
							++filter_count[i];
							count[i] = old_count[i];
							rtapi_print("Spike filter active[%d][%d]: %d\n", i, filter_count[i], accum_diff);
						}
						else
						{
							// normal operation, or else the big change must be real after all
							filter_count[i] = 0;
						}

						
						*(data->count[i]) = count[i];
						*(data->pos_fb[i]) = (float)(count[i]) / data->pos_scale[i];

					}

					// Feedback
					for (i = 0; i < VARIABLES; i++)
					{
						*(data->processVariable[i]) = rxData.processVariable[i]; 
					}

					// Inputs
					for (i = 0; i < DIGITAL_INPUTS; i++)
					{
						if ((rxData.inputs & (1 << i)) != 0)
						{
							*(data->inputs[i]) = 1; 		// input is high
							*(data->inputs[i+DIGITAL_INPUTS]) = 0;  // inverted
						}
						else
						{
							*(data->inputs[i]) = 0;			// input is low
							*(data->inputs[i+DIGITAL_INPUTS]) = 1;  // inverted
						}
					}
					break;
					
				case PRU_ESTOP:
					// we have an eStop notification from the PRU
					*(data->SPIstatus) = 0;
					 rtapi_print_msg(RTAPI_MSG_ERR, "An E-stop is active");

				default:
					// we have received a BAD payload from the PRU
					*(data->SPIstatus) = 0;

					rtapi_print("Bad SPI payload = %x\n", rxData.header);
					//for (i = 0; i < SPIBUFSIZE; i++) {
					//	rtapi_print("%d\n",rxData.rxBuffer[i]);
					//}
					break;
			}
		}
	}
	else
	{
		*(data->SPIstatus) = 0;
	}
	
	data->SPIresetOld = *(data->SPIreset);
}


void spi_write()
{
	int i;

	// Data header
	txData.header = PRU_WRITE;

	// Joint frequency commands
	for (i = 0; i < JOINTS; i++)
	{
		txData.jointFreqCmd[i] = data->freq[i];
	}

	for (i = 0; i < JOINTS; i++)
	{
		if (*(data->stepperEnable[i]) == 1)
		{
			txData.jointEnable |= (1 << i);		
		}
		else
		{
			txData.jointEnable &= ~(1 << i);	
		}
	}

	// Set points
	for (i = 0; i < VARIABLES; i++)
	{
		txData.setPoint[i] = *(data->setPoint[i]);
	}

	// Outputs
	for (i = 0; i < DIGITAL_OUTPUTS; i++)
	{
		if (*(data->outputs[i]) == 1)
		{
			txData.outputs |= (1 << i);		// output is high
		}
		else
		{
			txData.outputs &= ~(1 << i);	// output is low
		}
	}

	if( *(data->SPIstatus) )
	{
		// Transfer to and from the PRU
		spi_transfer();
	}

}


void spi_transfer()
{
	// send and receive data to and from the Remora PRU concurrently

	if (bcm == true)
	{
		bcm2835_spi_transfernb(txData.txBuffer, rxData.rxBuffer, SPIBUFSIZE);
	}
	else if (rp1 == true)
	{
		rp1spi_transfer(0, txData.txBuffer, rxData.rxBuffer, SPIBUFSIZE);
	}
	else if (h618 == true)
	{
		h618_spi_transfer(txData.txBuffer, rxData.rxBuffer, SPIBUFSIZE);
	}
}

static CONTROL parse_ctrl_type(const char *ctrl)
{
    if(!ctrl || !*ctrl || *ctrl == 'p' || *ctrl == 'P') return POSITION;
    if(*ctrl == 'v' || *ctrl == 'V') return VELOCITY;
    return INVALID;
}

#include <stdint.h>

// SPI模式定义（与Linux spidev兼容）
#define H618_SPI_MODE_0    0  // CPOL=0, CPHA=0
#define H618_SPI_MODE_1    1  // CPOL=0, CPHA=1
#define H618_SPI_MODE_2    2  // CPOL=1, CPHA=0
#define H618_SPI_MODE_3    3  // CPOL=1, CPHA=1

// H618 support - directly including implementation
/* h618_spi.c
 * Implementation file for H618 SPI interface using spidev
 * Used for Remora LinuxCNC driver
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>
// 添加libgpiod头文件
#include <gpiod.h>

// 全局变量
static int h618_spi_fd = -1;
static uint8_t current_spi_num = 0;
static uint8_t current_cs_num = 0;
// libgpiod相关变量
static struct gpiod_chip *gpio_chip = NULL;  // GPIO芯片句柄
static struct gpiod_line *reset_gpio_line = NULL;  // 重置GPIO线句柄
static const char *gpio_chip_path = "/dev/gpiochip0";  // 默认GPIO芯片路径

// SPI模式定义
#define SPI_MODE_0 0x00
#define SPI_MODE_1 0x01
#define SPI_MODE_2 0x02
#define SPI_MODE_3 0x03

// 检测是否运行在H618平台上
int h618_detect(void)
{
    FILE *fp;
    char buf[256];
    ssize_t buflen;
    
    // 尝试读取设备树信息来检测平台
    if ((fp = fopen("/proc/device-tree/compatible", "rb")) != NULL) {
        buflen = fread(buf, 1, sizeof(buf), fp);
        fclose(fp);
        
        // 检查是否包含h618相关字符串
        if (buflen > 0 && strstr(buf, "h616") != NULL) {
            rtapi_print_msg(RTAPI_MSG_INFO, "H618 platform detected\n");
            return 1;
        }
    }
    
    // 如果设备树检测失败，尝试其他方法（如读取/proc/cpuinfo）
    if ((fp = fopen("/proc/cpuinfo", "r")) != NULL) {
        while (fgets(buf, sizeof(buf), fp) != NULL) {
            if (strstr(buf, "h618") != NULL) {
                fclose(fp);
                rtapi_print_msg(RTAPI_MSG_INFO, "H618 platform detected from cpuinfo\n");
                return 1;
            }
        }
        fclose(fp);
    }
    
    // 直接检查spidev1.0设备是否存在
    if (access("/dev/spidev1.0", F_OK) == 0) {
        rtapi_print_msg(RTAPI_MSG_INFO, "H618 platform detected (spidev1.0 exists)\n");
        return 1;
    }
    
    return 0;
}

// 使用libgpiod设置GPIO方向
static int set_gpio_direction(int pin, const char *direction)
{
    rtapi_print_msg(RTAPI_MSG_INFO, "Setting GPIO %d direction to %s using libgpiod\n", pin, direction);
    
    // 确保GPIO芯片已打开
    if (!gpio_chip) {
        rtapi_print_msg(RTAPI_MSG_ERR, "GPIO chip not initialized\n");
        return -1;
    }
    
    // 获取GPIO线
    struct gpiod_line *line = gpiod_chip_get_line(gpio_chip, pin);
    if (!line) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to get GPIO %d line: %s\n", pin, strerror(errno));
        return -1;
    }
    
    // 设置GPIO方向
    int ret;
    if (strcmp(direction, "out") == 0) {
        ret = gpiod_line_request_output(line, "remora-spi", 0);
    } else {
        ret = gpiod_line_request_input(line, "remora-spi");
    }
    
    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to set GPIO %d direction to %s: %s\n", pin, direction, strerror(errno));
        gpiod_line_release(line);
        return -1;
    }
    
    rtapi_print_msg(RTAPI_MSG_DBG, "Successfully set GPIO %d direction to %s\n", pin, direction);
    
    // 对于重置引脚，保存线句柄
    if (pin == reset_gpio_pin && reset_gpio_line != line) {
        if (reset_gpio_line) {
            gpiod_line_release(reset_gpio_line);
        }
        reset_gpio_line = line;
    } else {
        // 对于其他引脚，使用完后释放
        gpiod_line_release(line);
    }
    
    return 0;
}

// 使用libgpiod设置GPIO值
static int set_gpio_value(int pin, int value)
{
    rtapi_print_msg(RTAPI_MSG_DBG, "Setting GPIO %d value to %d using libgpiod\n", pin, value);
    
    // 尝试获取GPIO线（优先使用已保存的重置引脚句柄）
    struct gpiod_line *line = NULL;
    
    if (pin == reset_gpio_pin && reset_gpio_line) {
        line = reset_gpio_line;
    } else {
        // 确保GPIO芯片已打开
        if (!gpio_chip) {
            rtapi_print_msg(RTAPI_MSG_ERR, "GPIO chip not initialized\n");
            return -1;
        }
        
        line = gpiod_chip_get_line(gpio_chip, pin);
        if (!line) {
            rtapi_print_msg(RTAPI_MSG_ERR, "Failed to get GPIO %d line: %s\n", pin, strerror(errno));
            return -1;
        }
        
        // 如果是首次使用该引脚作为输出，需要请求输出方向
        if (gpiod_line_request_output(line, "remora-spi", value) < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "Failed to request GPIO %d as output: %s\n", pin, strerror(errno));
            gpiod_line_release(line);
            return -1;
        }
    }
    
    // 设置GPIO值
    int ret = gpiod_line_set_value(line, value);
    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to set GPIO %d value to %d: %s\n", pin, value, strerror(errno));
        
        // 仅在不是重置引脚的情况下释放
        if (pin != reset_gpio_pin || line != reset_gpio_line) {
            gpiod_line_release(line);
        }
        return -1;
    }
    
    rtapi_print_msg(RTAPI_MSG_DBG, "Successfully set GPIO %d value to %d\n", pin, value);
    
    // 仅在不是重置引脚的情况下释放
    if (pin != reset_gpio_pin || line != reset_gpio_line) {
        gpiod_line_release(line);
    }
    
    return 0;
}

// 检查GPIO是否可访问（使用libgpiod）
static int check_gpio_access(int pin)
{
    rtapi_print_msg(RTAPI_MSG_INFO, "Checking GPIO %d access using libgpiod\n", pin);
    
    // 尝试打开GPIO芯片
    struct gpiod_chip *temp_chip = gpiod_chip_open(gpio_chip_path);
    if (!temp_chip) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to open GPIO chip %s: %s\n", gpio_chip_path, strerror(errno));
        return 0;
    }
    
    // 尝试获取GPIO线
    struct gpiod_line *line = gpiod_chip_get_line(temp_chip, pin);
    int accessible = 0;
    
    if (line) {
        // 检查线是否有效
        const char *name = gpiod_line_get_name(line);
        rtapi_print_msg(RTAPI_MSG_INFO, "GPIO %d line name: %s\n", pin, name ? name : "(no name)");
        accessible = 1;
        gpiod_line_release(line);
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to get GPIO %d line: %s\n", pin, strerror(errno));
    }
    
    gpiod_chip_close(temp_chip);
    return accessible;
}

// 初始化GPIO（使用libgpiod）
int h618_gpio_init(void)
{
    rtapi_print_msg(RTAPI_MSG_INFO, "Initializing H618 GPIO using libgpiod\n");
    
    // 关闭已有的GPIO资源
    if (reset_gpio_line) {
        gpiod_line_release(reset_gpio_line);
        reset_gpio_line = NULL;
    }
    
    if (gpio_chip) {
        gpiod_chip_close(gpio_chip);
        gpiod_chip = NULL;
    }
    
    // 打开GPIO芯片
    gpio_chip = gpiod_chip_open(gpio_chip_path);
    if (!gpio_chip) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to open GPIO chip %s: %s\n", gpio_chip_path, strerror(errno));
        return 0;
    }
    
    rtapi_print_msg(RTAPI_MSG_INFO, "Successfully opened GPIO chip %s\n", gpio_chip_path);
    rtapi_print_msg(RTAPI_MSG_INFO, "H618 GPIO initialized using libgpiod\n");
    return 1;
}

// 设置GPIO功能（使用libgpiod）
void h618_gpio_set_fsel(uint8_t pin, uint8_t fsel)
{
    // fsel=1表示输出模式
    if (fsel == 1) {
        set_gpio_direction(pin, "out");
    } else {
        set_gpio_direction(pin, "in");
    }
}

// 设置GPIO为高电平（使用libgpiod）
void h618_gpio_set(uint8_t pin)
{
    set_gpio_value(pin, 1);
}

// 设置GPIO为低电平（使用libgpiod）
void h618_gpio_clear(uint8_t pin)
{
    set_gpio_value(pin, 0);
}

// 初始化SPI（使用spidev）
int h618_spi_init(uint8_t spi_num, uint8_t cs_num, uint8_t mode, uint32_t freq)
{
    char spi_dev_path[32];
    uint8_t bits = 8;
    uint16_t delay = 0;
    int gpio_available = 0;
    
    // 记录当前配置
    current_spi_num = spi_num;
    current_cs_num = cs_num;
    
    // 关闭之前打开的SPI设备
    if (h618_spi_fd >= 0) {
        close(h618_spi_fd);
        h618_spi_fd = -1;
    }
    
    // 构建SPI设备路径
    // 用户提到有spidev1.0设备，我们优先使用这个
    if (spi_num == 1 && cs_num == 0) {
        snprintf(spi_dev_path, sizeof(spi_dev_path), "/dev/spidev1.0");
    } else {
        snprintf(spi_dev_path, sizeof(spi_dev_path), "/dev/spidev%d.%d", spi_num, cs_num);
    }
    
    rtapi_print_msg(RTAPI_MSG_INFO, "Trying to open SPI device: %s\n", spi_dev_path);
    
    // 打开SPI设备
    h618_spi_fd = rtapi_open_as_root(spi_dev_path, O_RDWR);
    if (h618_spi_fd < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to open SPI device %s: %s\n", spi_dev_path, strerror(errno));
        return 0;
    }
    
    // 设置SPI模式
    if (ioctl(h618_spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to set SPI mode: %s\n", strerror(errno));
        close(h618_spi_fd);
        h618_spi_fd = -1;
        return 0;
    }
    
    // 验证SPI模式设置
    if (ioctl(h618_spi_fd, SPI_IOC_RD_MODE, &mode) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to get SPI mode: %s\n", strerror(errno));
        close(h618_spi_fd);
        h618_spi_fd = -1;
        return 0;
    }
    
    // 设置位长度
    if (ioctl(h618_spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to set bits per word: %s\n", strerror(errno));
        close(h618_spi_fd);
        h618_spi_fd = -1;
        return 0;
    }
    
    // 验证位长度设置
    if (ioctl(h618_spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to get bits per word: %s\n", strerror(errno));
        close(h618_spi_fd);
        h618_spi_fd = -1;
        return 0;
    }
    
    // 设置SPI频率
    if (ioctl(h618_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &freq) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to set max speed Hz: %s\n", strerror(errno));
        close(h618_spi_fd);
        h618_spi_fd = -1;
        return 0;
    }
    
    // 验证频率设置
    if (ioctl(h618_spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &freq) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to get max speed Hz: %s\n", strerror(errno));
        close(h618_spi_fd);
        h618_spi_fd = -1;
        return 0;
    }
    
    // 初始化libgpiod GPIO
    if (!h618_gpio_init()) {
        rtapi_print_msg(RTAPI_MSG_WARN, "Warning: Failed to initialize GPIO subsystem, continuing without reset capability\n");
        gpio_available = 0;
    } else {
        // 检查GPIO是否可访问
        gpio_available = check_gpio_access(reset_gpio_pin);
        
        // 尝试初始化GPIO（用于重置），但失败不会中断初始化
        if (gpio_available) {
            if (set_gpio_direction(reset_gpio_pin, "out") == 0) {
                rtapi_print_msg(RTAPI_MSG_INFO, "GPIO %d configured as output (libgpiod)\n", reset_gpio_pin);
                
                // 尝试执行重置操作
                if (set_gpio_value(reset_gpio_pin, 0) == 0) {
                    rtapi_print_msg(RTAPI_MSG_INFO, "Asserted reset on GPIO %d\n", reset_gpio_pin);
                    
                    // 等待一段时间
                    usleep(100000); // 100ms
                    
                    // 释放重置
                    if (set_gpio_value(reset_gpio_pin, 1) == 0) {
                        rtapi_print_msg(RTAPI_MSG_INFO, "Deasserted reset on GPIO %d\n", reset_gpio_pin);
                        // 再次等待
                        usleep(100000); // 100ms
                    } else {
                        rtapi_print_msg(RTAPI_MSG_WARN, "Warning: Could not deassert reset, continuing without proper reset\n");
                    }
                } else {
                    rtapi_print_msg(RTAPI_MSG_WARN, "Warning: Could not assert reset, continuing without proper reset\n");
                }
            } else {
                rtapi_print_msg(RTAPI_MSG_WARN, "Warning: Could not configure GPIO direction, continuing without reset capability\n");
            }
        } else {
            rtapi_print_msg(RTAPI_MSG_WARN, "Warning: GPIO %d not available, continuing without reset capability\n", reset_gpio_pin);
        }
    }
    
    rtapi_print_msg(RTAPI_MSG_INFO, "H618 SPI%d.%d initialized successfully, mode=%d, freq=%d Hz, bits=%d\n",
                   spi_num, cs_num, mode, freq, bits);
    if (!gpio_available) {
        rtapi_print_msg(RTAPI_MSG_WARN, "Note: Reset functionality disabled due to GPIO access issues\n");
    }
    
    return 1;
}

// SPI数据传输（使用spidev）
void h618_spi_transfer(const void *txbuf, void *rxbuf, uint8_t len)
{
    int ret;
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)txbuf,
        .rx_buf = (unsigned long)rxbuf,
        .len = len,
        .delay_usecs = 0,
        .speed_hz = 0,  // 使用之前设置的速度
        .bits_per_word = 8,
    };
    
    if (h618_spi_fd < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "SPI device not initialized\n");
        return;
    }
    
    ret = ioctl(h618_spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "SPI transfer failed: %s\n", strerror(errno));
    }
}

// 清理资源
void h618_spi_cleanup(void)
{
    if (h618_spi_fd >= 0) {
        close(h618_spi_fd);
        h618_spi_fd = -1;
    }
    
    // 清理GPIO资源
    if (reset_gpio_line) {
        gpiod_line_release(reset_gpio_line);
        reset_gpio_line = NULL;
    }
    
    if (gpio_chip) {
        gpiod_chip_close(gpio_chip);
        gpio_chip = NULL;
        rtapi_print_msg(RTAPI_MSG_INFO, "H618 GPIO resources cleaned up\n");
    }
    
    rtapi_print_msg(RTAPI_MSG_INFO, "H618 SPI resources cleaned up\n");
}
