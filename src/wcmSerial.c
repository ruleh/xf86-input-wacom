/*
 * Copyright 1995-2003 by Frederic Lepied, France. <Lepied@XFree86.org>
 *                                                                            
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is  hereby granted without fee, provided that
 * the  above copyright   notice appear  in   all  copies and  that both  that
 * copyright  notice   and   this  permission   notice  appear  in  supporting
 * documentation, and that   the  name of  Frederic   Lepied not  be  used  in
 * advertising or publicity pertaining to distribution of the software without
 * specific,  written      prior  permission.     Frederic  Lepied   makes  no
 * representations about the suitability of this software for any purpose.  It
 * is provided "as is" without express or implied warranty.                   
 *                                                                            
 * FREDERIC  LEPIED DISCLAIMS ALL   WARRANTIES WITH REGARD  TO  THIS SOFTWARE,
 * INCLUDING ALL IMPLIED   WARRANTIES OF MERCHANTABILITY  AND   FITNESS, IN NO
 * EVENT  SHALL FREDERIC  LEPIED BE   LIABLE   FOR ANY  SPECIAL, INDIRECT   OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA  OR PROFITS, WHETHER  IN  AN ACTION OF  CONTRACT,  NEGLIGENCE OR OTHER
 * TORTIOUS  ACTION, ARISING    OUT OF OR   IN  CONNECTION  WITH THE USE    OR
 * PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "xf86Wacom.h"
#include "wcmSerial.h"

/* Serial Support */
static Bool serialDetect(LocalDevicePtr pDev);
static Bool serialInit(LocalDevicePtr pDev);
static void serialRead(LocalDevicePtr pDev);

static int serialInitTablet(WacomCommonPtr common, int fd);
static void serialInitIntuos(WacomCommonPtr common, int fd,
	const char* id, float version);
static void serialInitIntuos2(WacomCommonPtr common, int fd,
	const char* id, float version);
static void serialInitCintiq(WacomCommonPtr common, int fd,
	const char* id, float version);
static void serialInitPenPartner(WacomCommonPtr common, int fd,
	const char* id, float version);
static void serialInitGraphire(WacomCommonPtr common, int fd,
	const char* id, float version);
static void serialInitProtocol4(WacomCommonPtr common, int fd,
	const char* id, float version);
static void serialGetResolution(WacomCommonPtr common, int fd);
static int serialGetRanges(WacomCommonPtr common, int fd);
static int serialResetIntuos(WacomCommonPtr common, int fd);
static int serialResetCintiq(WacomCommonPtr common, int fd);
static int serialResetPenPartner(WacomCommonPtr common, int fd);
static int serialResetProtocol4(WacomCommonPtr common, int fd);
static int serialEnableTiltProtocol4(WacomCommonPtr common, int fd);
static int serialEnableSuppressProtocol4(WacomCommonPtr common, int fd);
static int serialSetLinkSpeedIntuos(WacomCommonPtr common, int fd);
static int serialSetLinkSpeedProtocol5(WacomCommonPtr common, int fd);
static int serialStartTablet(WacomCommonPtr common, int fd);
static void serialParseCintiq(WacomCommonPtr common);
static void serialParseGraphire(WacomCommonPtr common);
static void serialParseProtocol4(WacomCommonPtr common);
static void serialParseProtocol5(WacomCommonPtr common);

/*****************************************************************************
 * Global Structures
 ****************************************************************************/

	WacomDeviceClass gWacomSerialDevice =
	{
		serialDetect,
		serialInit,
		serialRead,
	};

/*****************************************************************************
 * Static Structures
 ****************************************************************************/

	static WacomModel serialIntuos =
	{
		"Intuos",
		serialInitIntuos,
		NULL,           /* resolution not queried */
		serialGetRanges,
		serialResetIntuos,
		NULL,           /* tilt automatically enabled */
		NULL,           /* suppress implemented in software */
		serialSetLinkSpeedIntuos,
		serialStartTablet,
		serialParseProtocol5,
	};

	static WacomModel serialIntuos2 =
	{
		"Intuos2",
		serialInitIntuos2,
		NULL,                 /* resolution not queried */
		serialGetRanges,
		serialResetIntuos,    /* same as Intuos */
		NULL,                 /* tilt automatically enabled */
		NULL,                 /* suppress implemented in software */
		serialSetLinkSpeedProtocol5,
		serialStartTablet,
		serialParseProtocol5,
	};

	static WacomModel serialCintiq =
	{
		"Cintiq",
		serialInitCintiq,
		serialGetResolution,
		serialGetRanges,
		serialResetCintiq,
		serialEnableTiltProtocol4,
		serialEnableSuppressProtocol4,
		NULL,               /* link speed cannot be changed */
		serialStartTablet,
		serialParseCintiq,
	};

	static WacomModel serialPenPartner =
	{
		"PenPartner",
		serialInitPenPartner,
		NULL,               /* resolution not queried */
		serialGetRanges,
		serialResetPenPartner,
		serialEnableTiltProtocol4,
		serialEnableSuppressProtocol4,
		NULL,              /* link speed cannot be changed */
		serialStartTablet,
		serialParseProtocol4,
	};	

	static WacomModel serialGraphire =
	{
		"Graphire",
		serialInitGraphire,
		NULL,                     /* resolution not queried */
		NULL,                     /* ranges not supported */
		serialResetPenPartner,    /* functionally very similar */
		serialEnableTiltProtocol4,
		serialEnableSuppressProtocol4,
		NULL,                    /* link speed cannot be changed */
		serialStartTablet,
		serialParseGraphire,
	};

	static WacomModel serialProtocol4 =
	{
		"Protocol4",
		serialInitProtocol4,
		serialGetResolution,
		serialGetRanges,
		serialResetProtocol4,
		serialEnableTiltProtocol4,
		serialEnableSuppressProtocol4,
		NULL,               /* link speed cannot be changed */
		serialStartTablet,
		serialParseProtocol4,
	};

/*****************************************************************************
 * Setup strings
 ****************************************************************************/

	static const char * setup_string = WC_MULTI WC_UPPER_ORIGIN
		WC_ALL_MACRO WC_NO_MACRO1 WC_RATE WC_NO_INCREMENT
		WC_STREAM_MODE WC_ZFILTER;
	static const char * pl_setup_string = WC_UPPER_ORIGIN WC_RATE
		WC_STREAM_MODE;
	static const char * penpartner_setup_string = WC_PRESSURE_MODE
		WC_START;
	static const char * intuos_setup_string = WC_V_MULTI WC_V_ID WC_RATE
		WC_START;

	/* PROTOCOL 4 */

	/* Format of 7 bytes data packet for Wacom Tablets
	Byte 1
	bit 7  Sync bit always 1
	bit 6  Pointing device detected
	bit 5  Cursor = 0 / Stylus = 1
	bit 4  Reserved
	bit 3  1 if a button on the pointing device has been pressed
	bit 2  Reserved
	bit 1  X15
	bit 0  X14

	Byte 2
	bit 7  Always 0
	bits 6-0 = X13 - X7

	Byte 3
	bit 7  Always 0
	bits 6-0 = X6 - X0

	Byte 4
	bit 7  Always 0
	bit 6  B3
	bit 5  B2
	bit 4  B1
	bit 3  B0
	bit 2  P0
	bit 1  Y15
	bit 0  Y14

	Byte 5
	bit 7  Always 0
	bits 6-0 = Y13 - Y7

	Byte 6
	bit 7  Always 0
	bits 6-0 = Y6 - Y0

	Byte 7
	bit 7 Always 0
	bit 6  Sign of pressure data
	bit 5  P6
	bit 4  P5
	bit 3  P4
	bit 2  P3
	bit 1  P2
	bit 0  P1

	byte 8 and 9 are optional and present only
	in tilt mode.

	Byte 8
	bit 7 Always 0
	bit 6 Sign of tilt X
	bit 5  Xt6
	bit 4  Xt5
	bit 3  Xt4
	bit 2  Xt3
	bit 1  Xt2
	bit 0  Xt1

	Byte 9
	bit 7 Always 0
	bit 6 Sign of tilt Y
	bit 5  Yt6
	bit 4  Yt5
	bit 3  Yt4
	bit 2  Yt3
	bit 1  Yt2
	bit 0  Yt1
	*/

/*****************************************************************************
 * xf86WcmSendRequest --
 *   send a request and wait for the answer.
 *   the answer must begin with the first two chars of the request.
 *   The last character in the answer string is replaced by a \0.
 ****************************************************************************/

char* xf86WcmSendRequest(int fd, const char* request, char* answer, int maxlen)
{
	int len, nr;
	int maxtry = MAXTRY;

	if (maxlen < 3)
		return NULL;
  
	/* send request string */
	do
	{
		SYSCALL(len = xf86WcmWrite(fd, request, strlen(request)));
		if ((len == -1) && (errno != EAGAIN))
		{
			ErrorF("Wacom xf86WcmWrite error : %s", strerror(errno));
			return NULL;
		}
		maxtry--;
	} while ((len == -1) && maxtry);

	if (maxtry == 0)
	{
		ErrorF("Wacom unable to xf86WcmWrite request string '%s' "
				"after %d tries\n", request, MAXTRY);
		return NULL;
	}
  
	do
	{
		maxtry = MAXTRY;
    
		/* Read the first byte of the answer which must
		 * be equal to the first byte of the request.
		 */
		do
		{
			if ((nr = xf86WcmWaitForTablet(fd)) > 0)
			{
				SYSCALL(nr = xf86WcmRead(fd, answer, 1));
				if ((nr == -1) && (errno != EAGAIN))
				{
					ErrorF("Wacom xf86WcmRead error : %s\n",
							strerror(errno));
					return NULL;
				}
				DBG(10, ErrorF("%c err=%d [0]\n",
					answer[0], nr));
			}
			maxtry--;  
		} while ((answer[0] != request[0]) && maxtry);

		if (maxtry == 0)
		{
			ErrorF("Wacom unable to read first byte of "
					"request '%c%c' answer after %d tries\n",
					request[0], request[1], MAXTRY);
			return NULL;
		}

		/* Read the second byte of the answer which must be equal
		 * to the second byte of the request. */
		do
		{    
			maxtry = MAXTRY;
			do
			{
				if ((nr = xf86WcmWaitForTablet(fd)) > 0)
				{
					SYSCALL(nr = xf86WcmRead(fd, answer+1, 1));
					if ((nr == -1) && (errno != EAGAIN))
					{
						ErrorF("Wacom xf86WcmRead error : %s\n",
								strerror(errno));
						return NULL;
					}
					DBG(10, ErrorF("%c err=%d [1]\n",
							answer[1], nr));
				}
				maxtry--;  
			} while ((nr <= 0) && maxtry);
	      
			if (maxtry == 0)
			{
				ErrorF("Wacom unable to read second byte of "
					"request '%c%c' answer after %d "
					"tries\n", request[0], request[1],
					MAXTRY);
				return NULL;
			}

			if (answer[1] != request[1])
				answer[0] = answer[1];
	      
		} while ((answer[0] == request[0]) && (answer[1] != request[1]));

	} while ((answer[0] != request[0]) && (answer[1] != request[1]));

	/* Read until we don't get anything or timeout. */

	len = 2;
	maxtry = MAXTRY;
	do
	{ 
		do
		{
			if ((nr = xf86WcmWaitForTablet(fd)) > 0)
			{
				SYSCALL(nr = xf86WcmRead(fd, answer+len, 1));
				if ((nr == -1) && (errno != EAGAIN))
				{
					ErrorF("Wacom xf86WcmRead error : %s\n", strerror(errno));
					return NULL;
				}
				DBG(10, ErrorF("%c err=%d [%d]\n", answer[len], nr, len));
			}
			else
			{
				if (len == 2)
				{
					DBG(10, ErrorF("timeout remains %d tries\n", maxtry));
					maxtry--;
				}
			}
		} while ((nr <= 0) && len == 2 && maxtry);

		if (nr > 0)
		{
			len += nr;
			if (len >= (maxlen - 1))
				return NULL;
		}

		if (maxtry == 0)
		{
			ErrorF("Wacom unable to read last byte of request '%c%c' answer after %d tries\n",
					request[0], request[1], MAXTRY);
			break;
		}
	} while (nr > 0);

	if (len <= 3)
		return NULL;
    
	answer[len-1] = '\0';
  
	return answer;
}

static Bool serialDetect(LocalDevicePtr pDev)
{
	return 1;
}

static Bool serialInit(LocalDevicePtr local)
{
	int err;
	WacomCommonPtr common = ((WacomDevicePtr)(local->private))->common;
    
	DBG(1, ErrorF("initializing serial tablet\n"));    

	/* Set the speed of the serial link to 38400 */
	if (xf86WcmSetSerialSpeed(local->fd, 38400) < 0)
		return !Success;
    
	/* Send reset to the tablet */
	SYSCALL(err = xf86WcmWrite(local->fd, WC_RESET_BAUD,
		strlen(WC_RESET_BAUD)));

	if (err == -1)
	{
		ErrorF("Wacom xf86WcmWrite error : %s\n", strerror(errno));
		return !Success;
	}
    
	/* Wait 250 mSecs */
	if (xf86WcmWait(250))
		return !Success;

	/* Send reset to the tablet */
	SYSCALL(err = xf86WcmWrite(local->fd, WC_RESET, strlen(WC_RESET)));
	if (err == -1)
	{
		ErrorF("Wacom xf86WcmWrite error : %s\n", strerror(errno));
		return !Success;
	}
    
	/* Wait 75 mSecs */
	if (xf86WcmWait(75))
		return !Success;

	/* Set the speed of the serial link to 19200 */
	if (xf86WcmSetSerialSpeed(local->fd, 19200) < 0)
		return !Success;
    
	/* Send reset to the tablet */
	SYSCALL(err = xf86WcmWrite(local->fd, WC_RESET_BAUD,
		strlen(WC_RESET_BAUD)));

	if (err == -1)
	{
		ErrorF("Wacom xf86WcmWrite error : %s\n", strerror(errno));
		return !Success;
	}
    
	/* Wait 250 mSecs */
	if (xf86WcmWait(250))
		return !Success;

	/* Send reset to the tablet */
	SYSCALL(err = xf86WcmWrite(local->fd, WC_RESET, strlen(WC_RESET)));
	if (err == -1)
	{
		ErrorF("Wacom xf86WcmWrite error : %s\n", strerror(errno));
		return !Success;
	}
    
	/* Wait 75 mSecs */
	if (xf86WcmWait(75))
		return !Success;

	/* Set the speed of the serial link to 9600 */
	if (xf86WcmSetSerialSpeed(local->fd, 9600) < 0)
		return !Success;
    
	/* Send reset to the tablet */
	SYSCALL(err = xf86WcmWrite(local->fd, WC_RESET_BAUD, strlen(WC_RESET_BAUD)));
	if (err == -1)
	{
		ErrorF("Wacom xf86WcmWrite error : %s\n", strerror(errno));
		return !Success;
	}
    
	/* Wait 250 mSecs */
	if (xf86WcmWait(250))
		return !Success;

	SYSCALL(err = xf86WcmWrite(local->fd, WC_STOP, strlen(WC_STOP)));
	if (err == -1)
	{
		ErrorF("Wacom xf86WcmWrite error : %s\n", strerror(errno));
		return !Success;
	}

	/* Wait 30 mSecs */
	if (xf86WcmWait(30))
		return !Success;

	xf86WcmFlushTablet(local->fd);

	if (serialInitTablet(common,local->fd) == !Success)
	{
		SYSCALL(xf86WcmClose(local->fd));
		local->fd = -1;
		return !Success;
	}

	return Success;
}

/*****************************************************************************
 * serialRead --
 *   Read the new events from the device, and enqueue them.
 ****************************************************************************/

static void serialRead(LocalDevicePtr local)
{
	WacomCommonPtr common = ((WacomDevicePtr)(local->private))->common;
	int len, loop;
	unsigned char buffer[BUFFER_SIZE];

	DBG(7, ErrorF("serialRead BEGIN device=%s fd=%d\n",
		common->wcmDevice, local->fd));

	SYSCALL(len = xf86WcmRead(local->fd, buffer, sizeof(buffer)));

	if (len <= 0)
	{
		ErrorF("Error reading wacom device : %s\n", strerror(errno));
		return;
	}

	DBG(10, ErrorF("serialRead read %d bytes\n", len));

	for (loop=0; loop<len; loop++)
	{
		/* magic bit is not OK */
		if ((common->wcmIndex == 0) && !(buffer[loop] & HEADER_BIT))
		{
			DBG(6, ErrorF("serialRead bad magic "
				"number 0x%x (pktlength=%d) %d\n",
				buffer[loop], common->wcmPktLength, loop));
			continue;
		}

		/* magic bit at wrong place */
		else if ((common->wcmIndex != 0) && (buffer[loop] & HEADER_BIT))
		{
			DBG(6, ErrorF("serialRead magic "
				"number 0x%x detetected at index %d loop=%d\n",
				(unsigned int)buffer[loop],common->wcmIndex,
				loop));
			common->wcmIndex = 0;
		}
 
		common->wcmData[common->wcmIndex++] = buffer[loop];

		/* if there are enough bytes, parse */
		if (common->wcmIndex == common->wcmPktLength)
		{
			if (common->wcmModel)
				common->wcmModel->Parse(common);

			/* reset for next packet */
			common->wcmIndex = 0;
		}
	} /* next data */

	DBG(7, ErrorF("serialRead END   local=0x%x index=%d\n",
		local, common->wcmIndex));
}

/*****************************************************************************
 * serialInitTablet --
 *   Read the new events from the device, and enqueue them.
 ****************************************************************************/


static int serialInitTablet(WacomCommonPtr common, int fd)
{
	int loop, idx;
	char id[BUFFER_SIZE];
	float version;

	WacomModelPtr model = NULL;

	/* if model is forced, initialize */
	if (model != NULL)
	{
		model->Initialize(common,fd,"",0.0F);
	}

	/* otherwise, query and initialize */
	else
	{
		DBG(2, ErrorF("reading model\n"));
		if (!xf86WcmSendRequest(fd, WC_MODEL, id, sizeof(id)))
			return !Success;

		DBG(2, ErrorF("%s\n", id));
  
		if (xf86Verbose)
			ErrorF("%s Wacom tablet model : %s\n",
					XCONFIG_PROBED, id+2);
    
		/* Answer is in the form ~#Tablet-Model VRom_Version 
		 * look for the first V from the end of the string
		 * this seems to be the better way to find the version
		 * of the ROM */
		for(loop=strlen(id); loop>=0 && *(id+loop) != 'V'; loop--);
		for(idx=loop; idx<strlen(id) && *(id+idx) != '-'; idx++);
		*(id+idx) = '\0';

		/* Extract version numbers */
		sscanf(id+loop+1, "%f", &version);

		/* Detect tablet model based on identifier */
		if (id[2] == 'G' && id[3] == 'D')
			model = &serialIntuos;
		else if (id[2] == 'X' && id[3] == 'D')
			model = &serialIntuos2;
		else if (id[2] == 'P' && id[3] == 'L')
			model = &serialCintiq;
		else if (id[2] == 'C' && id[3] == 'T')
			model = &serialPenPartner;
		else if (id[2] == 'E' && id[3] == 'T')
			model = &serialGraphire;
		else
			model = &serialProtocol4;

		/* Initialize the tablet */
		model->Initialize(common,fd,id,version);
	}

	/* Get tablet resolution */
	if (model->GetResolution)
		model->GetResolution(common,fd);

	/* Get tablet range */
	if (model->GetRanges && (model->GetRanges(common,fd) != Success))
		return !Success;
	
	/* Default threshold value if not set */
	if (common->wcmThreshold <= 0)
	{
		/* Threshold for counting pressure as a button */
		common->wcmThreshold = common->wcmMaxZ / 32;
		ErrorF("%s Wacom using pressure threshold of %d for button 1\n",
			XCONFIG_PROBED, common->wcmThreshold);
	}

	/* Reset tablet to known state */
	if (model->Reset && (model->Reset(common,fd) != Success))
	{
		ErrorF("Wacom xf86WcmWrite error : %s\n", strerror(errno));
		return !Success;
	}

	/* Enable tilt mode, if requested and available */
	if ((common->wcmFlags & TILT_REQUEST_FLAG) && model->EnableTilt)
	{
		if (model->EnableTilt(common,fd) != Success)
			return !Success;
	}

	/* Enable hardware suppress, if requested and available */
	if ((common->wcmSuppress != 0) && model->EnableSuppress)
	{
		if (model->EnableSuppress(common,fd) != Success)
			return !Success;
	}

	/* change the serial speed, if requested */
	if (common->wcmLinkSpeed != 9600)
	{
		if (model->SetLinkSpeed)
		{
			if (model->SetLinkSpeed(common,fd) != Success)
				return !Success;
		}
		else
		{
			ErrorF("Tablet does not support setting link "
				"speed, or not yet implemented\n");
		}
	}

	/* output tablet state as probed */
	if (xf86Verbose)
		ErrorF("%s Wacom %s tablet speed=%d maxX=%d maxY=%d maxZ=%d "
			"resX=%d resY=%d suppress=%d tilt=%s\n",
			XCONFIG_PROBED,
			model->name, common->wcmLinkSpeed,
			common->wcmMaxX, common->wcmMaxY, common->wcmMaxZ,
			common->wcmResolX, common->wcmResolY,
			common->wcmSuppress,
			HANDLE_TILT(common) ? "enabled" : "disabled");
  
	/* start the tablet data */
	if (model->Start && (model->Start(common,fd) != Success))
		return !Success;

	/* save model for use in parsing */
	common->wcmModel = model;

	return Success;
}

/* ugly kludge - temporarily exploded out */

static void serialParseGraphire(WacomCommonPtr common)
{
	WacomDeviceState* orig = &common->wcmChannel[0].state;
	WacomDeviceState ds = *orig;
	int is_stylus, cur_type;

	ds.proximity = (common->wcmData[0] & PROXIMITY_BIT);
     
	ds.x = (((common->wcmData[0] & 0x3) << 14) +
		(common->wcmData[1] << 7) +
		common->wcmData[2]);
	ds.y = (((common->wcmData[3] & 0x3) << 14) +
		(common->wcmData[4] << 7) +
		common->wcmData[5]);

	/* get pressure */
	ds.pressure = ((common->wcmData[6]&ZAXIS_BITS) << 2 ) +
		((common->wcmData[3]&ZAXIS_BIT) >> 1) +
		((common->wcmData[3]&PROXIMITY_BIT) >> 6) +
		((common->wcmData[6]&ZAXIS_SIGN_BIT) ? 0 : 0x100);

	/* get buttons */
	ds.buttons = (common->wcmData[3] & 0x38) >> 3;

	/* check which device we have */
	is_stylus = (common->wcmData[0] & POINTER_BIT);
	if (is_stylus)
	{
		/* first time into prox */
		cur_type = (ds.buttons & 4) ? ERASER_ID : STYLUS_ID;
		if (!orig->proximity)
		{
 			if (ds.proximity) 
				ds.device_type = cur_type;
		}

		/* we were fooled by tip and second
		 * sideswitch when it came into prox */
		else if ((ds.device_type != cur_type) &&
			(ds.device_type == ERASER_ID))
		{
			/* send a prox-out for old device */
			ds.proximity = 0;
			ds.buttons = 0;
			ds.device_type = 0;
		}
	}

	/* If it is not a stylus, it's a cursor */
	else if (!is_stylus && !orig->proximity && ds.proximity)
		ds.device_type = CURSOR_ID;

	/* If it is out of proximity, there is no device type */
	else if (!ds.proximity)
		ds.device_type = 0;

	DBG(8, ErrorF("serialParseGraphire %s\n",
		ds.device_type == CURSOR_ID ? "CURSOR" :
		ds.device_type == ERASER_ID ? "ERASER " :
		ds.device_type == STYLUS_ID ? "STYLUS" : "NONE"));

	/* handle relative wheel for non-stylus device */
	if (!is_stylus && ds.proximity)
	{
		ds.relwheel = (common->wcmData[6] & 0x30) >> 4;
		if (common->wcmData[6] & 0x40)
			ds.relwheel = -ds.relwheel;
	}

	/* handle tilt values only for stylus */
	if (HANDLE_TILT(common) && is_stylus)
	{
		ds.tiltx = (common->wcmData[7] & TILT_BITS);
		ds.tilty = (common->wcmData[8] & TILT_BITS);
		if (common->wcmData[7] & TILT_SIGN_BIT)
			ds.tiltx -= 64;
		if (common->wcmData[8] & TILT_SIGN_BIT)
			ds.tilty -= 64;
	}

	xf86WcmEvent(common,0,&ds);
}

/* temporarily exploded out */

static void serialParseCintiq(WacomCommonPtr common)
{
	WacomDeviceState* orig = &common->wcmChannel[0].state;
	WacomDeviceState ds = *orig;
	int is_stylus, cur_type;

	ds.proximity = (common->wcmData[0] & PROXIMITY_BIT);
     
	ds.x = (((common->wcmData[0] & 0x3) << 14) +
		(common->wcmData[1] << 7) +
		common->wcmData[2]);
	ds.y = (((common->wcmData[3] & 0x3) << 14) +
		(common->wcmData[4] << 7) +
		common->wcmData[5]);

	/* get pressure */
	if (common->wcmMaxZ == 255)
	{
		ds.pressure = ((common->wcmData[6] & ZAXIS_BITS) << 1 ) |
			((common->wcmData[3] & ZAXIS_BIT) >> 2) |
			((common->wcmData[6] & ZAXIS_SIGN_BIT) ? 0 : 0x80);
	}
	else
	{
		/* which tablets use this? */
		/* PL550, PL800, and Graphire apparently */
		ds.pressure = ((common->wcmData[6]&ZAXIS_BITS) << 2 ) +
			((common->wcmData[3]&ZAXIS_BIT) >> 1) +
			((common->wcmData[3]&PROXIMITY_BIT) >> 6) +
			((common->wcmData[6]&ZAXIS_SIGN_BIT) ? 0 : 0x100);
	}

	/* get buttons */
	ds.buttons = (common->wcmData[3] & 0x38) >> 3;

	/* If stylus comes into focus, use button to determine if eraser */
	is_stylus = (common->wcmData[0] & POINTER_BIT);
	if (is_stylus)
	{
		/* first time into prox */
		cur_type = (ds.buttons & 4) ? ERASER_ID : STYLUS_ID;
		if (!orig->proximity)
		{
 			if (ds.proximity) 
				ds.device_type = cur_type;
		}

		/* we were fooled by tip and second
		 * sideswitch when it came into prox */
		else if (ds.device_type != cur_type && ds.device_type == ERASER_ID)
		{
			/* send a prox-out for old device */
			ds.proximity = 0;
			ds.buttons = 0;
			ds.device_type = 0;
		}
	}

	/* If it is not a stylus, it's a cursor */
	else if (!is_stylus && !orig->proximity && ds.proximity)
		ds.device_type = CURSOR_ID;

	/* If it is out of proximity, there is no device type */
	else if (!ds.proximity)
		ds.device_type = 0;

	DBG(8, ErrorF("serialParseCintiq %s\n",
		ds.device_type == CURSOR_ID ? "CURSOR" :
		ds.device_type == ERASER_ID ? "ERASER " :
		ds.device_type == STYLUS_ID ? "STYLUS" : "NONE"));

	/* handle tilt values only for stylus */
	if (HANDLE_TILT(common) && is_stylus)
	{
		ds.tiltx = (common->wcmData[7] & TILT_BITS);
		ds.tilty = (common->wcmData[8] & TILT_BITS);
		if (common->wcmData[7] & TILT_SIGN_BIT)
			ds.tiltx -= 64;
		if (common->wcmData[8] & TILT_SIGN_BIT)
			ds.tilty -= 64;
	}

	xf86WcmEvent(common,0,&ds);
}

/* temporarily exploded out - used by most older tablets */

static void serialParseProtocol4(WacomCommonPtr common)
{
	WacomDeviceState* orig = &common->wcmChannel[0].state;
	WacomDeviceState ds = *orig;
	int is_stylus, cur_type;

	ds.proximity = (common->wcmData[0] & PROXIMITY_BIT);
     
	ds.x = (((common->wcmData[0] & 0x3) << 14) +
		(common->wcmData[1] << 7) +
		common->wcmData[2]);
	ds.y = (((common->wcmData[3] & 0x3) << 14) +
		(common->wcmData[4] << 7) +
		common->wcmData[5]);

	/* get pressure */
	if (common->wcmMaxZ == 255)
		ds.pressure = ((common->wcmData[6] & ZAXIS_BITS) << 1 ) |
			((common->wcmData[3] & ZAXIS_BIT) >> 2) |
			((common->wcmData[6] & ZAXIS_SIGN_BIT) ? 0 : 0x80);

	else
		ds.pressure = (common->wcmData[6] & ZAXIS_BITS) |
			(common->wcmData[6] & ZAXIS_SIGN_BIT) ? 0 : 0x40;

	/* get button state */
	ds.buttons = (common->wcmData[3] & BUTTONS_BITS) >> 3;

	/* If stylus comes into focus, use button to determine if eraser */
	is_stylus = (common->wcmData[0] & POINTER_BIT);
	if (is_stylus)
	{
		/* first time into prox */
		cur_type = (ds.buttons & 4) ? ERASER_ID : STYLUS_ID;
		if (!orig->proximity)
		{
 			if(ds.proximity) 
			ds.device_type = cur_type;
		}
		/* we were fooled by tip and second
		 * sideswitch when it came into prox */
		else if ((ds.device_type != cur_type) &&
			(ds.device_type == ERASER_ID))
		{
			/* send a prox-out for old device */
			ds.proximity = 0;
			ds.buttons = 0;
			ds.device_type = 0;
		}
	}

	/* If it is not a stylus, it's a cursor */
	else if (!is_stylus && !orig->proximity && ds.proximity)
		ds.device_type = CURSOR_ID;

	/* If it is out of proximity, there is no device type */
	else if (!ds.proximity)
		ds.device_type = 0;

	DBG(8, ErrorF("serialParseProtocol4 %s\n",
		ds.device_type == CURSOR_ID ? "CURSOR" :
		ds.device_type == ERASER_ID ? "ERASER " :
		ds.device_type == STYLUS_ID ? "STYLUS" : "NONE"));

	/* handle tilt values only for stylus */
	if (HANDLE_TILT(common) && is_stylus)
	{
		ds.tiltx = (common->wcmData[7] & TILT_BITS);
		ds.tilty = (common->wcmData[8] & TILT_BITS);
		if (common->wcmData[7] & TILT_SIGN_BIT)
			ds.tiltx -= 64;
		if (common->wcmData[8] & TILT_SIGN_BIT)
			ds.tilty -= 64;
	}

	xf86WcmEvent(common,0,&ds);
}

static void serialParseProtocol5(WacomCommonPtr common)
{
	int is_stylus=0, have_data=0;
	int channel;
	WacomDeviceState ds;

	/* start with previous state */
	channel = common->wcmData[0] & 0x01;
	ds = common->wcmChannel[channel].state;

	DBG(7, ErrorF("packet header = 0x%x\n",
			(unsigned int)common->wcmData[0]));
     
	/* Device ID packet */
	if ((common->wcmData[0] & 0xfc) == 0xc0)
	{
		/* reset channel */
		memset(&ds, 0, sizeof(ds));

		ds.proximity = 1;
		ds.device_id = (((common->wcmData[1] & 0x7f) << 5) |
				((common->wcmData[2] & 0x7c) >> 2));
		ds.serial_num = (((common->wcmData[2] & 0x03) << 30) |
				((common->wcmData[3] & 0x7f) << 23) |
				((common->wcmData[4] & 0x7f) << 16) |
				((common->wcmData[5] & 0x7f) << 9) |
				((common->wcmData[6] & 0x7f) << 2) |
				((common->wcmData[7] & 0x60) >> 5));

		if ((ds.device_id & 0xf06) != 0x802)
			ds.discard_first = 1;

		if (STYLUS_TOOL(&ds))
			ds.device_type = STYLUS_ID;
		else if (CURSOR_TOOL(&ds))
			ds.device_type = CURSOR_ID;
		else
			ds.device_type = ERASER_ID;
  
		DBG(6, ErrorF("device_id=0x%x serial_num=%u type=%s\n",
			ds.device_id, ds.serial_num,
			(ds.device_type == STYLUS_ID) ? "stylus" :
			(ds.device_type == CURSOR_ID) ? "cursor" :
			"eraser"));
	}

	/* Out of proximity packet */
	else if ((common->wcmData[0] & 0xfe) == 0x80)
	{
		ds.proximity = 0;
		have_data = 1;
	}

	/* General pen packet or eraser packet or airbrush first packet
	 * airbrush second packet */
	else if (((common->wcmData[0] & 0xb8) == 0xa0) ||
			((common->wcmData[0] & 0xbe) == 0xb4))
	{
		is_stylus = 1;
		ds.x = (((common->wcmData[1] & 0x7f) << 9) |
				((common->wcmData[2] & 0x7f) << 2) |
				((common->wcmData[3] & 0x60) >> 5));
		ds.y = (((common->wcmData[3] & 0x1f) << 11) |
				((common->wcmData[4] & 0x7f) << 4) |
				((common->wcmData[5] & 0x78) >> 3));
		if ((common->wcmData[0] & 0xb8) == 0xa0)
		{
			ds.pressure = (((common->wcmData[5] & 0x07) << 7) |
				(common->wcmData[6] & 0x7f));
			ds.buttons = (((common->wcmData[0]) & 0x06) |
				(ds.pressure >= common->wcmThreshold));
			/* pressure button should go down stream */
		}
		else
		{
			ds.relwheel = (((common->wcmData[5] & 0x07) << 7) |
				(common->wcmData[6] & 0x7f));
		}
		ds.tiltx = (common->wcmData[7] & TILT_BITS);
		ds.tilty = (common->wcmData[8] & TILT_BITS);
		if (common->wcmData[7] & TILT_SIGN_BIT)
			ds.tiltx -= (TILT_BITS + 1);
		if (common->wcmData[8] & TILT_SIGN_BIT)
			ds.tilty -= (TILT_BITS + 1);
		ds.proximity = (common->wcmData[0] & PROXIMITY_BIT);
		have_data = 1;
	} /* end pen packet */

	/* 4D mouse 1st packet or Lens cursor packet or 2D mouse packet*/
	else if (((common->wcmData[0] & 0xbe) == 0xa8) ||
			((common->wcmData[0] & 0xbe) == 0xb0))
	{
		is_stylus = 0;
		ds.x = (((common->wcmData[1] & 0x7f) << 9) |
				((common->wcmData[2] & 0x7f) << 2) |
				((common->wcmData[3] & 0x60) >> 5));
		ds.y = (((common->wcmData[3] & 0x1f) << 11) |
				((common->wcmData[4] & 0x7f) << 4) |
				((common->wcmData[5] & 0x78) >> 3));
		ds.tilty = 0;

		/* 4D mouse */
		if (MOUSE_4D(&ds))
		{
			ds.throttle = (((common->wcmData[5] & 0x07) << 7) |
				(common->wcmData[6] & 0x7f));
			if (common->wcmData[8] & 0x08)
				ds.throttle = -ds.throttle;
			ds.buttons = (((common->wcmData[8] & 0x70) >> 1) |
				(common->wcmData[8] & 0x07));
			have_data = !ds.discard_first;
		}

		/* Lens cursor */
		else if (LENS_CURSOR(&ds))
		{
			ds.buttons = common->wcmData[8];
			have_data = 1;
		}

		/* 2D mouse */
		else if (MOUSE_2D(&ds))
		{
			ds.buttons = (common->wcmData[8] & 0x1C) >> 2;
			ds.relwheel = - (common->wcmData[8] & 1) +
					((common->wcmData[8] & 2) >> 1);
			have_data = 1;
		}

		ds.proximity = (common->wcmData[0] & PROXIMITY_BIT);
	} /* end 4D mouse 1st packet */

	/* 4D mouse 2nd packet */
	else if ((common->wcmData[0] & 0xbe) == 0xaa)
	{
		is_stylus = 0;
		ds.x = (((common->wcmData[1] & 0x7f) << 9) |
			((common->wcmData[2] & 0x7f) << 2) |
			((common->wcmData[3] & 0x60) >> 5));
		ds.y = (((common->wcmData[3] & 0x1f) << 11) |
			((common->wcmData[4] & 0x7f) << 4) |
			((common->wcmData[5] & 0x78) >> 3));
		ds.rotation = (((common->wcmData[6] & 0x0f) << 7) |
			(common->wcmData[7] & 0x7f));
		if (ds.rotation < 900) ds.rotation = -ds.rotation;
		else ds.rotation = 1799 - ds.rotation;
		ds.proximity = (common->wcmData[0] & PROXIMITY_BIT);
		have_data = 1;
		ds.discard_first = 0;
	}
	else
	{
		DBG(10, ErrorF("unknown wacom V packet 0x%x\n",
				common->wcmData[0]));
	}

	/* if new data is available, send it */
	if (have_data)
	       	xf86WcmEvent(common,channel,&ds);

	/* otherwise, initialize channel and wait for next packet */
	else
	{
		common->wcmChannel[channel].state = ds;
		common->wcmChannel[channel].pDev = NULL;
	}
}

/*****************************************************************************
 * Model-specific functions
 ****************************************************************************/

static void serialInitIntuos(WacomCommonPtr common, int fd,
	const char* id, float version)
{
	DBG(2, ErrorF("detected an Intuos model\n"));

	common->wcmProtocolLevel = 5;
	common->wcmVersion = version;

	common->wcmMaxZ = 1023;   /* max Z value */
	common->wcmResolX = 2540; /* tablet X resolution in points/inch */
	common->wcmResolY = 2540; /* tablet Y resolution in points/inch */
	common->wcmPktLength = 9; /* length of a packet */
	common->wcmFlags |= TILT_ENABLED_FLAG;
}

static void serialInitIntuos2(WacomCommonPtr common, int fd,
	const char* id, float version)
{
	DBG(2, ErrorF("detected an Intuos2 model\n"));

	common->wcmProtocolLevel = 5;
	common->wcmVersion = version;

	common->wcmMaxZ = 1023;       /* max Z value */
	common->wcmResolX = 2540;     /* tablet X resolution in points/inch */
	common->wcmResolY = 2540;     /* tablet Y resolution in points/inch */
	common->wcmPktLength = 9;     /* length of a packet */
	common->wcmFlags |= TILT_ENABLED_FLAG;
}

static void serialInitCintiq(WacomCommonPtr common, int fd,
	const char* id, float version)
{
	DBG(2, ErrorF("detected a Cintiq model\n"));

	common->wcmProtocolLevel = 4;
	common->wcmPktLength = 7;
	common->wcmVersion = version;

	if (id[5] == '2')
	{
		/* PL-250  */
		if ( id[6] == '5' )
		{
			common->wcmMaxZ = 255;
		}
		/* PL-270  */
		else
		{
			common->wcmMaxZ = 255;
		}
	}
	else if (id[5] == '3')
	{
		/* PL-300  */
		common->wcmMaxZ = 255;
	}
	else if (id[5] == '4')
	{
		/* PL-400  */
		common->wcmMaxZ = 255;
	}
	else if (id[5] == '5')
	{
		/* PL-550  */
		if ( id[6] == '5' )
		{
			common->wcmMaxZ = 511;
		}
		/* PL-500  */
		else
		{
			common->wcmMaxZ = 255;
		}
	}
	else if (id[5] == '6')
	{
		/* PL-600SX  */
		if ( id[8] == 'S' )
		{
			common->wcmMaxZ = 255;
		}
		/* PL-600  */
		else
		{
			common->wcmMaxZ = 255;
		}
	}
	else if (id[5] == '8')
	{
		/* PL-800  */
		common->wcmMaxZ = 511;
	}

	common->wcmResolX = 508; /* tablet X resolution in points/inch */
	common->wcmResolY = 508; /* tablet Y resolution in points/inch */
}

static void serialInitPenPartner(WacomCommonPtr common, int fd,
	const char* id, float version)
{
	DBG(2, ErrorF("detected a PenPartner model\n"));

	common->wcmProtocolLevel = 4;
	common->wcmPktLength = 7;
	common->wcmVersion = version;

	common->wcmMaxZ = 256;
	common->wcmResolX = 1000; /* tablet X resolution in points/inch */
	common->wcmResolY = 1000; /* tablet Y resolution in points/inch */
}

static void serialInitGraphire(WacomCommonPtr common, int fd,
	const char* id, float version)
{
	DBG(2, ErrorF("detected a Graphire model\n"));

	common->wcmProtocolLevel = 4;
	common->wcmPktLength = 7;
	common->wcmVersion = version;

	/* Graphire models don't answer WC_COORD requests */
	common->wcmMaxX = 5103;
	common->wcmMaxY = 3711;
	common->wcmMaxZ = 512;
	common->wcmResolX = 1000; /* tablet X resolution in points/inch */
	common->wcmResolY = 1000; /* tablet Y resolution in points/inch */
}

static void serialInitProtocol4(WacomCommonPtr common, int fd,
	const char* id, float version)
{
	DBG(2, ErrorF("detected a Protocol4 model\n"));

	common->wcmProtocolLevel = 4;
	common->wcmPktLength = 7;
	common->wcmVersion = version;

	/* If no maxZ is set, determine from version */
	if (!common->wcmMaxZ)
	{
		/* the rom version determines the max z */
		if (version >= (float)1.2)
			common->wcmMaxZ = 255;
		else
			common->wcmMaxZ = 120;
	}
}

static void serialGetResolution(WacomCommonPtr common, int fd)
{
	int a, b;
	char buffer[BUFFER_SIZE], header[BUFFER_SIZE];

	if (!(common->wcmResolX && common->wcmResolY))
	{
		DBG(2, ErrorF("Requesting resolution from device\n"));
		if (xf86WcmSendRequest(fd, WC_CONFIG, buffer,
			sizeof(buffer)))
		{
			DBG(2, ErrorF("%s\n", buffer));
			/* The header string is simply a place to put the
			 * unwanted config header don't use buffer+xx because
			 * the header size varies on different tablets */

			if (sscanf(buffer, "%[^,],%d,%d,%d,%d", header,
				 &a, &b, &common->wcmResolX,
				 &common->wcmResolY) == 5)
			{
				DBG(6, ErrorF("WC_CONFIG Header = %s\n",
					header));
			}
			else
			{
				ErrorF("WACOM: unable to parse resolution. "
					"Using default.\n");
				common->wcmResolX = common->wcmResolY = 1270;
			}
		}
		else
		{
			ErrorF("WACOM: unable to read resolution. "
				"Using default.\n");
			common->wcmResolX = common->wcmResolY = 1270;
		}
	}

	DBG(2, ErrorF("serialGetResolution: ResolX=%d ResolY=%d\n",
		common->wcmResolX, common->wcmResolY));
}

static int serialGetRanges(WacomCommonPtr common, int fd)
{
	char buffer[BUFFER_SIZE];

	if (!(common->wcmMaxX && common->wcmMaxY))
	{
		DBG(2, ErrorF("Requesting max coordinates\n"));
		if (!xf86WcmSendRequest(fd, WC_COORD, buffer,
			sizeof(buffer)))
		{
			ErrorF("WACOM: unable to read max coordinates. "
				"Use the MaxX and MaxY options.\n");
			return !Success;
		}
		DBG(2, ErrorF("%s\n", buffer));
		if (sscanf(buffer+2, "%d,%d", &common->wcmMaxX,
			&common->wcmMaxY) != 2)
		{
			ErrorF("WACOM: unable to parse max coordinates. "
				"Use the MaxX and MaxY options.\n");
			return !Success;
		}
	}

	DBG(2, ErrorF("serialGetRanges: maxX=%d maxY=%d (%g,%g in)\n",
		common->wcmMaxX, common->wcmMaxY,
		(double)common->wcmMaxX / common->wcmResolX,
		(double)common->wcmMaxY / common->wcmResolY));

	return Success;
}

static int serialResetIntuos(WacomCommonPtr common, int fd)
{
	int err;
	SYSCALL(err = xf86WcmWrite(fd, intuos_setup_string,
		strlen(intuos_setup_string)));
	return (err == -1) ? !Success : Success;
}

static int serialResetCintiq(WacomCommonPtr common, int fd)
{
	int err;

	SYSCALL(err = xf86WcmWrite(fd, WC_RESET, strlen(WC_RESET)));
  
	if (xf86WcmWait(75)) return !Success;

	SYSCALL(err = xf86WcmWrite(fd, pl_setup_string,
		strlen(pl_setup_string)));
	if (err == -1) return !Success;

	SYSCALL(err = xf86WcmWrite(fd, penpartner_setup_string,
		strlen(penpartner_setup_string)));

	return (err == -1) ? !Success : Success;
}

static int serialResetPenPartner(WacomCommonPtr common, int fd)
{
	int err;
	SYSCALL(err = xf86WcmWrite(fd, penpartner_setup_string,
		strlen(penpartner_setup_string)));
	return (err == -1) ? !Success : Success;
}

static int serialResetProtocol4(WacomCommonPtr common, int fd)
{
	int err;

	SYSCALL(err = xf86WcmWrite(fd, WC_RESET, strlen(WC_RESET)));
  
	if (xf86WcmWait(75)) return !Success;

	SYSCALL(err = xf86WcmWrite(fd, setup_string,
		strlen(setup_string)));
	if (err == -1) return !Success;

	SYSCALL(err = xf86WcmWrite(fd, penpartner_setup_string,
		strlen(penpartner_setup_string)));
	return (err == -1) ? !Success : Success;
}

static int serialEnableTiltProtocol4(WacomCommonPtr common, int fd)
{
	int err;

	/* Send the tilt mode command after reset and setup because
	 * it must be enabled after multi-mode to take precedence */

	/* Tilt works on ROM 1.4 and above */
	if (common->wcmVersion >= 1.4F)
	{
		DBG(2, ErrorF("Sending tilt mode order\n"));

		common->wcmPktLength = 9;
		common->wcmFlags |= TILT_ENABLED_FLAG;
 
		SYSCALL(err = xf86WcmWrite(fd, WC_TILT_MODE,
			strlen(WC_TILT_MODE)));

		if (err == -1)
		{
			ErrorF("Wacom xf86WcmWrite error : %s\n",
				strerror(errno));
			return !Success;
		}
	}

	return Success;
}

static int serialEnableSuppressProtocol4(WacomCommonPtr common, int fd)
{
	char buf[20];
	int err;

	sprintf(buf, "%s%d\r", WC_SUPPRESS, common->wcmSuppress);
	SYSCALL(err = xf86WcmWrite(fd, buf, strlen(buf)));

	if (err == -1)
	{
		ErrorF("Wacom xf86WcmWrite error : %s\n",
			strerror(errno));
		return !Success;
	}
	return Success;
}

static int serialSetLinkSpeedIntuos(WacomCommonPtr common, int fd)
{
	if ((common->wcmLinkSpeed == 38400) &&
		(common->wcmVersion < 2.0F))
	{
		ErrorF("Wacom: 38400 speed not supported with this Intuos "
			"firmware (%f)\n", common->wcmVersion);
		ErrorF("Switching to 19200\n");
		common->wcmLinkSpeed = 19200;
	}
	return serialSetLinkSpeedProtocol5(common,fd);
}

static int serialSetLinkSpeedProtocol5(WacomCommonPtr common, int fd)
{
	int err;
	char* speed_init_string;

	DBG(1, ErrorF("Switching serial link to %d\n",
		common->wcmLinkSpeed));

	/* set init string according to speed */
	speed_init_string = (common->wcmLinkSpeed == 38400) ?
		WC_V_38400 : WC_V_19200;

	/* Switch the tablet to the requested speed */
	SYSCALL(err = xf86WcmWrite(fd, speed_init_string,
		strlen(speed_init_string)));

	if (err == -1)
	{
		ErrorF("Wacom xf86WcmWrite error : %s\n", strerror(errno));
		return !Success;
	}

	/* Wait 75 mSecs */
	if (xf86WcmWait(75))
		return !Success;

	/* Set speed of serial link to requested speed */
	if (xf86WcmSetSerialSpeed(fd, common->wcmLinkSpeed) < 0)
		return !Success;

	return Success;
}

static int serialStartTablet(WacomCommonPtr common, int fd)
{
	int err;

	/* Tell the tablet to start sending coordinates */
	SYSCALL(err = xf86WcmWrite(fd, WC_START, strlen(WC_START)));

	if (err == -1)
	{
		ErrorF("Wacom xf86WcmWrite error : %s\n", strerror(errno));
		return !Success;
	}

	return Success;
}
