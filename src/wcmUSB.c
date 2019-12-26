/*
 * Copyright 1995-2002 by Frederic Lepied, France. <Lepied@XFree86.org>
 * Copyright 2002-2013 by Ping Cheng, Wacom. <pingc@wacom.com>
 *                                                                            
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software 
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "xf86Wacom.h"

#include <asm/types.h>
#include <linux/input.h>
#include <sys/utsname.h>
#include <linux/version.h>

#define MAX_USB_EVENTS 32

typedef struct {
	int wcmLastToolSerial;
	int wcmDeviceType;
	Bool wcmUseMT;
	int wcmMTChannel;
	int wcmEventCnt;
	struct input_event wcmEvents[MAX_USB_EVENTS];
	uint32_t wcmEventFlags;      /* event types received in this frame */
	int nbuttons;                /* total number of buttons */
	int npadkeys;                /* number of pad keys in the above array */
	int padkey_code[WCM_MAX_BUTTONS];/* hardware codes for buttons */
	int lastChannel;
} wcmUSBData;

static Bool usbDetect(InputInfoPtr);
static Bool usbWcmInit(InputInfoPtr pDev, char* id, size_t id_len, float *version);
static int usbProbeKeys(InputInfoPtr pInfo);
static int usbStart(InputInfoPtr pInfo);
static void usbInitProtocol5(WacomCommonPtr common, const char* id,
	float version);
int usbWcmGetRanges(InputInfoPtr pInfo);
static int usbParse(InputInfoPtr pInfo, const unsigned char* data, int len);
static int usbDetectConfig(InputInfoPtr pInfo);
static void usbParseEvent(InputInfoPtr pInfo,
	const struct input_event* event);
static void usbParseSynEvent(InputInfoPtr pInfo,
			     const struct input_event *event);
static void usbParseMscEvent(InputInfoPtr pInfo,
			     const struct input_event *event);
static void usbDispatchEvents(InputInfoPtr pInfo);
static int usbChooseChannel(WacomCommonPtr common, int device_type, unsigned int serial);

	WacomDeviceClass gWacomUSBDevice =
	{
		usbDetect,
		NULL, /* no USB-specific options */
		usbWcmInit,
		usbProbeKeys
	};

static struct _WacomModel usbUnknown =		
{						
	.name = "Unknown USB",			
	.Initialize = usbInitProtocol5,
	.GetResolution = NULL,			
	.GetRanges = usbWcmGetRanges,		
	.Start = usbStart,			
	.Parse = usbParse,			
	.DetectConfig = usbDetectConfig,	
};

/*****************************************************************************
 * usbDetect --
 *   Test if the attached device is USB.
 ****************************************************************************/

static Bool usbDetect(InputInfoPtr pInfo)
{
	int version;
	int err;
#ifdef DEBUG
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;

	DBG(1, priv, "\n");
#endif

	SYSCALL(err = ioctl(pInfo->fd, EVIOCGVERSION, &version));

	if (err < 0)
	{
		xf86Msg(X_ERROR, "%s: usbDetect: can not ioctl version\n", pInfo->name);
		return 0;
	}

	return 1;
}

/*****************************************************************************
 * usbStart --
 ****************************************************************************/
static int
usbStart(InputInfoPtr pInfo)
{
	int err;

	if (xf86CheckBoolOption(pInfo->options, "GrabDevice", 0))
	{
		/* Try to grab the event device so that data don't leak to /dev/input/mice */
		SYSCALL(err = ioctl(pInfo->fd, EVIOCGRAB, (pointer)1));

		/* this is called for all tools, so all but the first one fails with
		 * EBUSY */
		if (err < 0 && errno != EBUSY)
			xf86Msg(X_ERROR, "%s: Wacom X driver can't grab event device (%s)\n",
				pInfo->name, strerror(errno));
	}
	return Success;
}

/* Key codes used to mark tablet buttons -- must be in sync
 * with the keycode array in wacom kernel drivers.
 */
static unsigned short padkey_codes [] = {
	BTN_0, BTN_1, BTN_2, BTN_3, BTN_4,
	BTN_5, BTN_6, BTN_7, BTN_8, BTN_9,
	BTN_A, BTN_B, BTN_C, BTN_X, BTN_Y, BTN_Z,
	BTN_BASE, BTN_BASE2, BTN_BASE3,
	BTN_BASE4, BTN_BASE5, BTN_BASE6,
	BTN_TL, BTN_TR, BTN_TL2, BTN_TR2, BTN_SELECT
};

/* Fixed mapped stylus and mouse buttons */

#define WCM_USB_MAX_MOUSE_BUTTONS 5
#define WCM_USB_MAX_STYLUS_BUTTONS 4

static unsigned short mouse_codes [] = {
	BTN_LEFT, BTN_MIDDLE, BTN_RIGHT, BTN_BACK, BTN_FORWARD,
	BTN_SIDE, BTN_EXTRA
};

static Bool usbWcmInit(InputInfoPtr pInfo, char* id, size_t id_len, float *version)
{
	int i;
	struct input_id sID;
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common = priv->common;
	wcmUSBData *usbdata;

	DBG(1, priv, "initializing USB tablet\n");

	/* fetch vendor, product, and model name */
	if (ioctl(pInfo->fd, EVIOCGID, &sID) == -1 ||
	    ioctl(pInfo->fd, EVIOCGNAME(id_len), id) == -1) {
		xf86Msg(X_ERROR, "%s: failed to ioctl ID or name.\n",
					pInfo->name);
		return !Success;
	}

	if (!common->private &&
	    !(common->private = calloc(1, sizeof(wcmUSBData))))
	{
		xf86Msg(X_ERROR, "%s: unable to alloc event queue.\n",
					pInfo->name);
		return !Success;
	}

	usbdata = common->private;
	*version = 0.0;

	if (!common->wcmModel)
	{
		common->wcmModel = &usbUnknown;
		common->wcmResolX = common->wcmResolY = 1016;
	}

	/* Find out supported button codes. */
	usbdata->npadkeys = 0;
	for (i = 0; i < ARRAY_SIZE(padkey_codes); i++)
		if (ISBITSET (common->wcmKeys, padkey_codes [i]))
			usbdata->padkey_code [usbdata->npadkeys++] = padkey_codes [i];

	if (usbdata->npadkeys == 0) {
		/* If no pad keys were detected, entertain the possibility that any
		 * mouse buttons which exist may belong to the pad (e.g. Graphire4).
		 * If we're wrong, this will over-state the capabilities of the pad
		 * but that shouldn't actually cause problems.
		 */
		for (i = ARRAY_SIZE(mouse_codes) - 1; i > 0; i--)
			if (ISBITSET(common->wcmKeys, mouse_codes[i]))
				break;

		/* Make sure room for fixed map mouse buttons.  This
		 * means mappings may overlap with padkey_codes[].
		 */
		if (i != 0)
			usbdata->npadkeys = WCM_USB_MAX_MOUSE_BUTTONS;
	}

	/* nbuttons tracks maximum buttons on all tools (stylus/mouse).
	 *
	 * Mouse support left, middle, right, side, and extra side button.
	 */
	if (ISBITSET (common->wcmKeys, BTN_TOOL_MOUSE))
		usbdata->nbuttons = WCM_USB_MAX_MOUSE_BUTTONS;
	else
		usbdata->nbuttons = WCM_USB_MAX_STYLUS_BUTTONS;

	return Success;
}

static void usbInitProtocol5(WacomCommonPtr common, const char* id,
	float version)
{
	common->wcmPktLength = sizeof(struct input_event);
	common->wcmCursorProxoutDistDefault = PROXOUT_INTUOS_DISTANCE;

	/* tilt enabled */
	common->wcmFlags |= TILT_ENABLED_FLAG;
}

/* Initialize fixed PAD channel's state to in proximity.
 *
 * Some, but not all, Wacom protocol 4/5 devices are always in proximity.
 * Because of evdev filtering, there will never be a BTN_TOOL_FINGER
 * sent to initialize state.
 * Generic protocol devices never send anything to help initialize PAD
 * device as well.
 * This helps those 2 cases and does not hurt the cases where kernel
 * driver sends out-of-proximity event for PAD since PAD is always on
 * its own channel, PAD_CHANNEL.
 */
static void usbWcmInitPadState(InputInfoPtr pInfo)
{
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common = priv->common;
	WacomDeviceState *ds;
	int channel = PAD_CHANNEL;

	DBG(6, common, "Initializing PAD channel %d\n", channel);

	ds = &common->wcmChannel[channel].work;

	ds->proximity = 1;
	ds->device_type = PAD_ID;
	ds->device_id = PAD_DEVICE_ID;
	ds->serial_num = channel;
}

int usbWcmGetRanges(InputInfoPtr pInfo)
{
	struct input_absinfo absinfo;
	unsigned long ev[NBITS(EV_MAX)] = {0};
	unsigned long abs[NBITS(ABS_MAX)] = {0};
	unsigned long sw[NBITS(SW_MAX)] = {0};
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common =	priv->common;
	wcmUSBData* private = common->private;
	int is_touch = IsTouch(priv);

	/* Devices such as Bamboo P&T may have Pad data reported in the same
	 * packet as Touch.  It's normal for Pad to be called first but logic
	 * requires it to act the same as Touch.
	 */
	if (ISBITSET(common->wcmKeys, BTN_TOOL_DOUBLETAP)
	     && ISBITSET(common->wcmKeys, BTN_FORWARD))
		is_touch = 1;

	if (ioctl(pInfo->fd, EVIOCGBIT(0 /*EV*/, sizeof(ev)), ev) < 0)
	{
		xf86Msg(X_ERROR, "%s: unable to ioctl event bits.\n", pInfo->name);
		return !Success;
	}

	if (!ISBITSET(ev,EV_ABS))
	{
		/* may be an expresskey only interface */
		if (ISBITSET(common->wcmKeys, BTN_FORWARD) ||
		    ISBITSET(common->wcmKeys, BTN_0))
			goto pad_init;

		xf86Msg(X_ERROR, "%s: no abs bits.\n", pInfo->name);
		return !Success;
	}

	/* absolute values */
        if (ioctl(pInfo->fd, EVIOCGBIT(EV_ABS, sizeof(abs)), abs) < 0)
	{
		xf86Msg(X_ERROR, "%s: unable to ioctl max values.\n", pInfo->name);
		return !Success;
	}

	/* max x */
	if (ioctl(pInfo->fd, EVIOCGABS(ABS_X), &absinfo) < 0)
	{
		/* may be a PAD only interface */
		if (ISBITSET(common->wcmKeys, BTN_FORWARD) ||
		    ISBITSET(common->wcmKeys, BTN_0))
			goto pad_init;

		xf86Msg(X_ERROR, "%s: unable to ioctl xmax value.\n", pInfo->name);
		return !Success;
	}

	if (absinfo.maximum <= 0)
	{
		xf86Msg(X_ERROR, "%s: xmax value is %d, expected > 0.\n",
			pInfo->name, absinfo.maximum);
		return !Success;
	}

	if (!is_touch)
	{
		common->wcmMinX = absinfo.minimum;
		common->wcmMaxX = absinfo.maximum;

		if (absinfo.resolution > 0)
			common->wcmResolX = absinfo.resolution * 1000;
	}
	else
	{
		common->wcmMaxTouchX = absinfo.maximum;

		if (absinfo.resolution > 0)
			common->wcmTouchResolX = absinfo.resolution * 1000;
	}

	/* max y */
	if (ioctl(pInfo->fd, EVIOCGABS(ABS_Y), &absinfo) < 0)
	{
		xf86Msg(X_ERROR, "%s: unable to ioctl ymax value.\n", pInfo->name);
		return !Success;
	}

	if (absinfo.maximum <= 0)
	{
		xf86Msg(X_ERROR, "%s: ymax value is %d, expected > 0.\n",
			pInfo->name, absinfo.maximum);
		return !Success;
	}

	if (!is_touch)
	{
		common->wcmMinY = absinfo.minimum;
		common->wcmMaxY = absinfo.maximum;

		if (absinfo.resolution > 0)
			common->wcmResolY = absinfo.resolution * 1000;
	}
	else
	{
		common->wcmMaxTouchY = absinfo.maximum;

		if (absinfo.resolution > 0)
			common->wcmTouchResolY = absinfo.resolution * 1000;
	}

	/* max finger strip X for tablets with Expresskeys
	 * or physical X for touch devices in hundredths of a mm */
	if (ISBITSET(abs, ABS_RX) &&
			!ioctl(pInfo->fd, EVIOCGABS(ABS_RX), &absinfo))
	{
		if (is_touch)
			common->wcmTouchResolX =
				(int)(((double)common->wcmMaxTouchX * 100000.0
				 / (double)absinfo.maximum) + 0.5);
		else
			common->wcmMaxStripX = absinfo.maximum;
	}

	/* max touchring value for standalone pad tools */
	common->wcmMinRing = 0;
	common->wcmMaxRing = 71;
	if (!ISBITSET(ev,EV_MSC) && ISBITSET(abs, ABS_WHEEL) &&
			!ioctl(pInfo->fd, EVIOCGABS(ABS_WHEEL), &absinfo))
	{
		common->wcmMinRing = absinfo.minimum;
		common->wcmMaxRing = absinfo.maximum;
	}

	/* X tilt range */
	if (ISBITSET(abs, ABS_TILT_X) &&
			!ioctl(pInfo->fd, EVIOCGABS(ABS_TILT_X), &absinfo))
	{
		/* If resolution is specified */
		if (absinfo.resolution > 0)
		{
			/* Assume the range is centered on zero */
			common->wcmTiltOffX = 0;
			/* Convert to resolution expected by applications */
			common->wcmTiltFactX = TILT_RES /
					       (double)absinfo.resolution;
		}
		else
		{
			/*
			 * Center the reported range on zero to support
			 * kernel drivers still reporting non-zero-centered
			 * values.
			 */
			common->wcmTiltOffX = - (absinfo.minimum +
						 absinfo.maximum) / 2;
			/*
			 * Assume reported resolution is the one expected by
			 * applications
			 */
			common->wcmTiltFactX = 1.0;
		}
		common->wcmTiltMinX = round((absinfo.minimum +
					     common->wcmTiltOffX) *
					    common->wcmTiltFactX);
		common->wcmTiltMaxX = round((absinfo.maximum +
					     common->wcmTiltOffX) *
					    common->wcmTiltFactX);
	}

	/* Y tilt range */
	if (ISBITSET(abs, ABS_TILT_Y) &&
			!ioctl(pInfo->fd, EVIOCGABS(ABS_TILT_Y), &absinfo))
	{
		/* If resolution is specified */
		if (absinfo.resolution > 0)
		{
			/* Assume the range is centered on zero */
			common->wcmTiltOffY = 0;
			/* Convert to resolution expected by applications */
			common->wcmTiltFactY = TILT_RES /
					       (double)absinfo.resolution;
		}
		else
		{
			/*
			 * Center the reported range on zero to support
			 * kernel drivers still reporting non-zero-centered
			 * values.
			 */
			common->wcmTiltOffY = - (absinfo.minimum +
						 absinfo.maximum) / 2;
			/*
			 * Assume reported resolution is the one expected by
			 * applications
			 */
			common->wcmTiltFactY = 1.0;
		}
		common->wcmTiltMinY = round((absinfo.minimum +
					     common->wcmTiltOffY) *
					    common->wcmTiltFactY);
		common->wcmTiltMaxY = round((absinfo.maximum +
					     common->wcmTiltOffY) *
					    common->wcmTiltFactY);
	}

	/* max finger strip Y for tablets with Expresskeys
	 * or physical Y for touch devices in hundredths of a mm */
	if (ISBITSET(abs, ABS_RY) &&
			!ioctl(pInfo->fd, EVIOCGABS(ABS_RY), &absinfo))
	{
		if (is_touch)
			common->wcmTouchResolY =
				 (int)(((double)common->wcmMaxTouchY * 100000.0
				 / (double)absinfo.maximum) + 0.5);
		else
			common->wcmMaxStripY = absinfo.maximum;
	}

	/* max z cannot be configured */
	if (ISBITSET(abs, ABS_PRESSURE) &&
			!ioctl(pInfo->fd, EVIOCGABS(ABS_PRESSURE), &absinfo))
		common->wcmMaxZ = absinfo.maximum;

	/* max distance */
	if (ISBITSET(abs, ABS_DISTANCE) &&
			!ioctl(pInfo->fd, EVIOCGABS(ABS_DISTANCE), &absinfo))
		common->wcmMaxDist = absinfo.maximum;

	if (ISBITSET(abs, ABS_MT_SLOT))
	{
		private->wcmUseMT = 1;

		if (!ioctl(pInfo->fd, EVIOCGABS(ABS_MT_SLOT), &absinfo))
			common->wcmMaxContacts = absinfo.maximum + 1;
	}

	if (ioctl(pInfo->fd, EVIOCGBIT(EV_SW, sizeof(sw)), sw) < 0)
	{
		xf86Msg(X_ERROR, "%s: unable to ioctl sw bits.\n", pInfo->name);
		return 0;
	}
	else if (ISBITSET(sw, SW_MUTE_DEVICE))
	{
		common->wcmHasHWTouchSwitch = TRUE;

		memset(sw, 0, sizeof(sw));

		if (ioctl(pInfo->fd, EVIOCGSW(sizeof(sw)), sw) < 0)
			xf86Msg(X_ERROR, "%s: unable to ioctl sw state.\n", pInfo->name);

		if (ISBITSET(sw, SW_MUTE_DEVICE))
			common->wcmHWTouchSwitchState = 0;
		else
			common->wcmHWTouchSwitchState = 1;
	}

pad_init:
	usbWcmInitPadState(pInfo);

	return Success;
}

static int usbDetectConfig(InputInfoPtr pInfo)
{
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common = priv->common;
	wcmUSBData *usbdata = common->private;

	DBG(10, common, "\n");
	if (IsPad (priv))
		priv->nbuttons = usbdata->npadkeys;
	else
		priv->nbuttons = usbdata->nbuttons;

	if (!common->wcmCursorProxoutDist)
		common->wcmCursorProxoutDist
			= common->wcmCursorProxoutDistDefault;
	return TRUE;
}

static int usbParse(InputInfoPtr pInfo, const unsigned char* data, int len)
{
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common = priv->common;
	struct input_event event;

	if (len < sizeof(struct input_event))
		return 0;

	memcpy(&event, data, sizeof(event));
	usbParseEvent(pInfo, &event);
	return common->wcmPktLength;
}

/**
 * Returns a serial number for the provided device_type and serial, as
 * through it came from from a Protocol 5 device.
 *
 * Protocol 5 serial numbers will be returned unchanged. Otherwise,
 * anonymous tools (from Protocol 4 and Generic Protocol) will have
 * serial numbers of: -1 (pad), 1 (pen/1st finger), 2 (2nd finger),
 * etc.
 *
 * @param[in] device_type  Type of device (e.g. STYLUS_ID, TOUCH_ID, PAD_ID)
 * @param[in] serial       Serial number of device
 * @return                 Serial number of device as through from Protocol 5
 */
static int protocol5Serial(int device_type, unsigned int serial) {
	if (!serial) {
		/* Generic Protocol does not send serial numbers */
		return device_type == PAD_ID ? -1 : 1;
	}
	else if (serial == 0xf0) {
		/* Protocol 4 uses the expected anonymous serial
		 * numbers, but has the wrong PAD serial number.
		 * This could cause problem if 0xf0 is ever used
		 * for a Protocol 5 serial number, but isn't a
		 * problem as yet.
		 */
		return -1;
	}
	else {
		/* Protocol 5 FTW */
		return serial;
	}
}

/**
 * Find an appropriate channel to track the specified tool's state in.
 * If the tool is already in proximity, the channel currently being used
 * to store its state will be returned. Otherwise, an arbitrary available
 * channel will be cleaned and returned. Up to MAX_CHANNEL tools can be
 * tracked concurrently by driver.
 *
 * @param[in] common
 * @param[in] device_type  Type of tool (e.g. STYLUS_ID, TOUCH_ID, PAD_ID)
 * @param[in] serial       Serial number of tool
 * @return                 Channel number to track the tool's state
 */
static int usbChooseChannel(WacomCommonPtr common, int device_type, unsigned int serial)
{
	/* figure out the channel to use based on serial number */
	int i, channel = -1;

	/* force events from PAD device to PAD_CHANNEL */
	if (serial == -1)
		channel = PAD_CHANNEL;

	/* find existing channel */
	if (channel < 0)
	{
		for (i=0; i<MAX_CHANNELS; i++)
		{
			if (common->wcmChannel[i].work.proximity &&
			    common->wcmChannel[i].work.device_type == device_type &&
			    common->wcmChannel[i].work.serial_num == serial)
			{
				channel = i;
				break;
			}
		}
	}

	/* find and clean an empty channel */
	if (channel < 0)
	{
		for (i=0; i<MAX_CHANNELS; i++)
		{
			if (i == PAD_CHANNEL)
				continue;

			if (!common->wcmChannel[i].work.proximity &&
			    !common->wcmChannel[i].valid.state.proximity)
			{
				channel = i;
				memset(&common->wcmChannel[channel],0, sizeof(WacomChannel));
				break;
			}
		}
	}

	/* fresh out of channels */
	if (channel < 0)
	{
		/* This should never happen in normal use.
		 * Let's start over again. Force prox-out for all channels.
		 */
		for (i=0; i<MAX_CHANNELS; i++)
		{
			if (i == PAD_CHANNEL)
				continue;

			if (common->wcmChannel[i].work.proximity &&
			    (common->wcmChannel[i].work.serial_num != -1))
			{
				common->wcmChannel[i].work.proximity = 0;
				/* dispatch event */
				wcmEvent(common, i, &common->wcmChannel[i].work);
				DBG(2, common, "free channels: dropping %u\n",
						common->wcmChannel[i].work.serial_num);
			}
		}
		DBG(1, common, "device with serial number: %u"
		    " at %d: Exceeded channel count; ignoring the events.\n",
		    serial, (int)GetTimeInMillis());
	}

	return channel;
}

static inline void
usbResetEventCounter(wcmUSBData *private)
{
	private->wcmEventCnt = 0;
	private->wcmEventFlags = 0;
}

static void usbParseEvent(InputInfoPtr pInfo,
	const struct input_event* event)
{
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common = priv->common;
	wcmUSBData* private = common->private;

	DBG(10, common, "\n");

	/* store events until we receive a SYN_REPORT */

	/* space left? bail if not. */
	if (private->wcmEventCnt >= ARRAY_SIZE(private->wcmEvents))
	{
		LogMessageVerbSigSafe(X_ERROR, 0, "%s: usbParse: Exceeded event queue (%d) \n",
				      pInfo->name, private->wcmEventCnt);
		usbResetEventCounter(private);
		return;
	}

	/* save it for later */
	private->wcmEvents[private->wcmEventCnt++] = *event;
	private->wcmEventFlags |= 1 << event->type;

	switch (event->type)
	{
		case EV_MSC:
			usbParseMscEvent(pInfo, event);
			break;
		case EV_SYN:
			usbParseSynEvent(pInfo, event);
			break;
		default:
			break;
	}
}

static void usbParseMscEvent(InputInfoPtr pInfo,
			     const struct input_event *event)
{
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common = priv->common;
	wcmUSBData* private = common->private;

	if (event->code != MSC_SERIAL)
		return;

	if (event->value != 0)
	{
		/* save the serial number so we can look up the channel number later */
		private->wcmLastToolSerial = event->value;
	}
	else
	{
		/* we don't report serial numbers for some tools but we never report
		 * a serial number with a value of 0 - if that happens drop the
		 * whole frame */
		LogMessageVerbSigSafe(X_ERROR, 0,
				      "%s: usbParse: Ignoring event from invalid serial 0\n",
				      pInfo->name);
		usbResetEventCounter(private);
	}
}

/**
 * EV_SYN marks the end of a set of events containing axes and button info.
 * Check for valid data and hand over to dispatch to extract the actual
 * values and process them. At this point, all events up to the EV_SYN are
 * queued up in wcmEvents.
 */
static void usbParseSynEvent(InputInfoPtr pInfo,
			     const struct input_event *event)
{
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common = priv->common;
	wcmUSBData* private = common->private;
	const uint32_t significant_event_types = ~(1 << EV_SYN | 1 << EV_MSC);

	if (event->code != SYN_REPORT)
		return;

	/* ignore events without information */
	if ((private->wcmEventCnt < 2) && private->wcmLastToolSerial)
	{
		DBG(3, common, "%s: dropping empty event for serial %d\n",
		    pInfo->name, private->wcmLastToolSerial);
		goto skipEvent;
	}


	/* If all we get in an event frame is EV_SYN/EV_MSC, we don't have
	 * real data to process. */
	if ((private->wcmEventFlags & significant_event_types) == 0)
	{
		DBG(6, common, "no real events received\n");
		goto skipEvent;
	}

	/* dispatch all queued events */
	usbDispatchEvents(pInfo);

skipEvent:
	usbResetEventCounter(private);
}

static int usbFilterEvent(WacomCommonPtr common, struct input_event *event)
{
	wcmUSBData* private = common->private;

	/* For devices that report multitouch, the following list is a set of
	 * duplicate data from one slot and needs to be filtered out.
	 */
	if (private->wcmUseMT)
	{
		if (event->type == EV_KEY)
		{
			switch(event->code)
			{
				case BTN_TOUCH:
				case BTN_TOOL_FINGER:
				case BTN_TOOL_DOUBLETAP:
				case BTN_TOOL_TRIPLETAP:
					return 1;
			}
		}
		else if (event->type == EV_ABS)
		{
			if (private->wcmDeviceType == TOUCH_ID)
			{
				/* filter ST for MT */
				switch(event->code)
				{
					case ABS_X:
					case ABS_Y:
					case ABS_PRESSURE:
						return 1;
				}
			}
			else
			{
				/* filter MT for pen */
				switch(event->code)
				{
					case ABS_MT_SLOT:
					case ABS_MT_TRACKING_ID:
					case ABS_MT_POSITION_X:
					case ABS_MT_POSITION_Y:
					case ABS_MT_PRESSURE:
						return 1;
				}
			}
		}
	}

	/* For generic devices, filter out doubletap/tripletap that
	 * can be confused with older protocol.
	 */
	if (event->type == EV_KEY)
	{
		switch(event->code)
		{
			case BTN_TOOL_DOUBLETAP:
			case BTN_TOOL_TRIPLETAP:
				return 1;
		}
	}

	return 0;
}

#define ERASER_BIT      0x008
#define PUCK_BITS	0xf00
#define PUCK_EXCEPTION  0x806

/**
 * Handle ABS events strictly according to their definition as documented
 * in the Linux kernel.
 *
 * @param common
 * @param event
 * @param channel_number
 */
static int usbParseGenericAbsEvent(WacomCommonPtr common,
			    struct input_event *event, int channel_number)
{
	WacomChannel *channel = &common->wcmChannel[channel_number];
	WacomDeviceState *ds = &channel->work;
	int change = 1;

	switch(event->code)
	{
		case ABS_X:
			ds->x = event->value;
			break;
		case ABS_Y:
			ds->y = event->value;
			break;
		case ABS_RZ:
			ds->rotation = event->value;
			break;
		case ABS_TILT_X:
			ds->tiltx = round((event->value + common->wcmTiltOffX) *
					  common->wcmTiltFactX);
			break;
		case ABS_TILT_Y:
			ds->tilty = round((event->value + common->wcmTiltOffY) *
					  common->wcmTiltFactY);
			break;
		case ABS_PRESSURE:
			ds->pressure = event->value;
			break;
		case ABS_DISTANCE:
			ds->distance = event->value;
			break;
		case ABS_WHEEL:
			ds->abswheel = event->value;
			break;
		case ABS_THROTTLE:
			ds->throttle = event->value;
			break;
		default:
			change = 0;
	}

	return change;
}

/**
 * Handle an incoming ABS event.
 *
 * @param common
 * @param event
 * @param channel_number
 */
static void usbParseAbsEvent(WacomCommonPtr common,
			    struct input_event *event, int channel_number)
{
	WacomChannel *channel = &common->wcmChannel[channel_number];
	WacomDeviceState *ds = &channel->work;
	Bool change;

	change = usbParseGenericAbsEvent(common, event, channel_number);

	ds->time = (int)GetTimeInMillis();
	channel->dirty |= change;
}

/**
 * Flip the mask bit in buttons corresponding to btn to the specified state.
 *
 * @param buttons The current button mask
 * @param btn Zero-indexed button number to change
 * @param state Zero to unset, non-zero to set the mask for the button
 *
 * @return The new button mask
 */
TEST_NON_STATIC int
mod_buttons(int buttons, int btn, int state)
{
	int mask;

	if (btn >= sizeof(int) * 8)
	{
		LogMessageVerbSigSafe(X_ERROR, 0,
				      "%s: Invalid button number %d. Insufficient storage\n",
				      __func__, btn);
		return buttons;
	}

	mask = 1 << btn;

	if (state)
		buttons |= mask;
	else
		buttons &= ~mask;

	return buttons;
}

static void usbParseAbsMTEvent(WacomCommonPtr common, struct input_event *event)
{
	int change = 1;
	wcmUSBData* private = common->private;
	WacomDeviceState *ds;

	ds = &common->wcmChannel[private->wcmMTChannel].work;

	switch(event->code)
	{
		case ABS_MT_SLOT:
			if (event->value >= 0) {
				int serial = event->value + 1;
				private->wcmMTChannel = usbChooseChannel(common, TOUCH_ID, serial);
				if (private->wcmMTChannel < 0)
					return;
				ds = &common->wcmChannel[private->wcmMTChannel].work;
				ds->serial_num = serial;
			}
			break;

		case ABS_MT_TRACKING_ID:
			ds->proximity = (event->value != -1);
			/* set this here as type for this channel doesn't get set in usbDispatchEvent() */
			ds->device_type = TOUCH_ID;
			ds->device_id = TOUCH_DEVICE_ID;
			ds->sample = (int)GetTimeInMillis();
			break;

		case ABS_MT_POSITION_X:
			ds->x = event->value;
			break;

		case ABS_MT_POSITION_Y:
			ds->y = event->value;
			break;

		case ABS_MT_PRESSURE:
			ds->pressure = event->value;
			break;

		default:
			change = 0;
	}

	ds->time = (int)GetTimeInMillis();
	(&common->wcmChannel[private->wcmMTChannel])->dirty |= change;
}

static void usbParseKeyEvent(WacomCommonPtr common,
			    struct input_event *event, int channel_number)
{
	int change = 1;
	WacomChannel *channel = &common->wcmChannel[channel_number];
	WacomDeviceState *ds = &channel->work;
	WacomDeviceState *dslast = &channel->valid.state;

	/* BTN_TOOL_* are sent to indicate when a specific tool is going
	 * in our out of proximity.  When going in proximity, here we
	 * initialize tool specific values.  Making sure shared values
	 * are correct values during tool change is done elsewhere.
	 */
	switch (event->code)
	{
		case BTN_TOOL_PEN:
		case BTN_TOOL_PENCIL:
		case BTN_TOOL_BRUSH:
		case BTN_TOOL_AIRBRUSH:
			/* V5 tools use ABS_MISC to report device_id */
			ds->device_id = STYLUS_DEVICE_ID;
			ds->proximity = (event->value != 0);
			DBG(6, common,
			    "USB stylus detected %x\n",
			    event->code);
			break;

		case BTN_TOOL_RUBBER:
			/* V5 tools use ABS_MISC to report device_id */
			ds->device_id = ERASER_DEVICE_ID;
			ds->proximity = (event->value != 0);
			DBG(6, common,
			    "USB eraser detected %x (value=%d)\n",
			    event->code, event->value);
			break;

		case BTN_TOOL_MOUSE:
		case BTN_TOOL_LENS:
			DBG(6, common,
			    "USB mouse detected %x (value=%d)\n",
			    event->code, event->value);
			/* V5 tools use ABS_MISC to report device_id */
			ds->device_id = CURSOR_DEVICE_ID;
			ds->proximity = (event->value != 0);
			break;

               case BTN_TOUCH:
			break;

		case BTN_TOOL_FINGER:
			/* A pad tool */
			/* fall through */
		case BTN_TOOL_DOUBLETAP:
			DBG(6, common,
			    "USB Touch detected %x (value=%d)\n",
			    event->code, event->value);
			ds->device_id = TOUCH_DEVICE_ID;
			ds->proximity = event->value;
			/* time stamp for 2FGT gesture events */
			if ((ds->proximity && !dslast->proximity) ||
			    (!ds->proximity && dslast->proximity))
				ds->sample = (int)GetTimeInMillis();
			break;

		case BTN_TOOL_TRIPLETAP:
			DBG(6, common,
			    "USB Touch second finger detected %x (value=%d)\n",
			    event->code, event->value);
			ds->device_id = TOUCH_DEVICE_ID;
			ds->proximity = event->value;
			/* time stamp for 2GT gesture events */
			if ((ds->proximity && !dslast->proximity) ||
			    (!ds->proximity && dslast->proximity))
				ds->sample = (int)GetTimeInMillis();
			/* Second finger events will be considered in
			 * combination with the first finger data */
			break;

		default:
			change = 0;
	}

	ds->time = (int)GetTimeInMillis();
	channel->dirty |= change;

	if (change)
		return;
}

/* Handle all button presses except for stylus buttons */
static void usbParseBTNEvent(WacomCommonPtr common,
			    struct input_event *event, int channel_number)
{
	int nkeys;
	int change = 1;
	wcmUSBData *usbdata = common->private;
	WacomChannel *channel = &common->wcmChannel[channel_number];
	WacomDeviceState *ds = &channel->work;

	switch (event->code)
	{
		case BTN_LEFT:
			ds->buttons = mod_buttons(ds->buttons, 0, event->value);
			break;

		case BTN_MIDDLE:
			ds->buttons = mod_buttons(ds->buttons, 1, event->value);
			break;

		case BTN_RIGHT:
			ds->buttons = mod_buttons(ds->buttons, 2, event->value);
			break;

		case BTN_SIDE:
		case BTN_BACK:
			ds->buttons = mod_buttons(ds->buttons, 3, event->value);
			break;

		case BTN_EXTRA:
		case BTN_FORWARD:
			ds->buttons = mod_buttons(ds->buttons, 4, event->value);
			break;

		default:
			for (nkeys = 0; nkeys < usbdata->npadkeys; nkeys++)
			{
				if (event->code == usbdata->padkey_code[nkeys])
				{
					ds->buttons = mod_buttons(ds->buttons, nkeys, event->value);
					break;
				}
			}
			if (nkeys >= usbdata->npadkeys)
				change = 0;
	}

	ds->time = (int)GetTimeInMillis();
	channel->dirty |= change;
}

/**
 * Translates an event code from the kernel (e.g. type: EV_ABS code: ABS_MISC value: STYLUS_DEVICE_ID)
 * into the corresponding device type for the driver (e.g. STYLUS_ID).
 *
 * @param[in] common
 * @param[in] type      Linux input tool type (e.g. EV_KEY)
 * @param[in] code      Linux input tool code (e.g. BTN_STYLUS_PEN)
 * @param[in] value     Linux input tool value (e.g. STYLUS_DEVICE_ID)
 * @return              Wacom device ID (e.g. STYLUS_ID) or 0 if no match.
 */
static int deviceTypeFromEvent(WacomCommonPtr common, int type, int code, int value)
{
	if (type == EV_KEY) {
		return TOUCH_ID;
	} else if (type == EV_ABS) {
		switch (code) {
			case ABS_MT_SLOT:
			case ABS_MT_TRACKING_ID:
				return TOUCH_ID;
		}
	}

	return 0;
}

/**
 * Queries the kernel through EVIOCGKEY for the latest device type
 * information. The result is the first tool type (e.g. STYLUS_ID)
 * found associated with the in-prox tool.
 *
 * @param[in] common
 * @return            A tool type (e.g. STYLUS_ID) associated with the in-prox tool
 */
static int refreshDeviceType(WacomCommonPtr common, int fd)
{
	int device_type = 0;
	unsigned long keys[NBITS(KEY_MAX)] = { 0 };
	int rc = ioctl(fd, EVIOCGKEY(sizeof(keys)), keys);
	int i;

	if (rc == -1) {
		xf86Msg(X_ERROR, "%s: failed to retrieve key bits\n", common->device_path);
		return 0;
	}

	for (i = 0; i < KEY_MAX; i++)
	{
		if (ISBITSET(keys, i))
			device_type = deviceTypeFromEvent(common, EV_KEY, i, 0);
		if (device_type)
			return device_type;
	}

	return 0;
}

static int deriveDeviceTypeFromButtonEvent(WacomCommonPtr common,
					   const struct input_event *event_ptr)
{
	wcmUSBData *usbdata = common->private;
	int nkeys;

	if (event_ptr->type == EV_KEY) {

		switch (event_ptr->code) {
		case BTN_LEFT:
		case BTN_MIDDLE:
		case BTN_RIGHT:
		case BTN_SIDE:
		case BTN_BACK:
		case BTN_EXTRA:
		case BTN_FORWARD:
			return PAD_ID;
		default:
			for (nkeys = 0; nkeys < usbdata->npadkeys; nkeys++)
			{
				if (event_ptr->code == usbdata->padkey_code[nkeys]) {
					return PAD_ID;
				}
			}
		}
	}
	return 0;
}

/***
 * Retrieve the tool type from an USB data packet by looking at the event
 * codes. Refer to linux/input.h for event codes that define tool types.
 *
 * @param[in] common
 * @param[in] event_ptr A pointer to the USB data packet that contains the
 * events to be processed.
 * @param[in] nevents Number of events in the packet.
 * @param[in] last_device_type The device type for the last event
 *
 * @return The tool type. This falls back on last_device_type if no
 *         pen/touch/eraser event code in the event, and on EVIOCGKEY
 *         if last_device_type is not a tool. If all else fails, '0'
 *         is returned.
 */
static int usbInitToolType(WacomCommonPtr common, int fd,
                           const struct input_event *event_ptr,
                           int nevents, int last_device_type)
{
	int i, device_type = 0;

	for (i = 0; (i < nevents) && !device_type; ++i, event_ptr++)
	{
		device_type = deviceTypeFromEvent(common, event_ptr->type, event_ptr->code, event_ptr->value);
	}

	if (!device_type)
		device_type = last_device_type;

	if (!device_type)
		device_type = refreshDeviceType(common, fd);

	if (!device_type) /* expresskey pressed at startup or missing type */
		for (i = 0; (i < nevents) && !device_type; ++i, event_ptr++)
			device_type = deriveDeviceTypeFromButtonEvent(common, event_ptr);

	return device_type;
}

static void usbDispatchEvents(InputInfoPtr pInfo)
{
	int i, c;
	WacomDeviceState *ds;
	struct input_event* event;
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common = priv->common;
	int channel;
	wcmUSBData* private = common->private;
	WacomDeviceState dslast = common->wcmChannel[private->lastChannel].valid.state;

	DBG(6, common, "%d events received\n", private->wcmEventCnt);

	private->wcmDeviceType = usbInitToolType(common, pInfo->fd,
	                                         private->wcmEvents,
	                                         private->wcmEventCnt,
	                                         dslast.device_type);

	private->wcmLastToolSerial = protocol5Serial(private->wcmDeviceType, private->wcmLastToolSerial);
	channel = usbChooseChannel(common, private->wcmDeviceType, private->wcmLastToolSerial);

	/* couldn't decide channel? invalid data */
	if (channel == -1) {
		private->wcmEventCnt = 0;
		return;
	}

	ds = &common->wcmChannel[channel].work;
	dslast = common->wcmChannel[channel].valid.state;

	if (ds->device_type && ds->device_type != private->wcmDeviceType)
		LogMessageVerbSigSafe(X_ERROR, 0,
				      "usbDispatchEvents: Device Type mismatch - %d -> %d. This is a BUG.\n",
				      ds->device_type, private->wcmDeviceType);
	/* no device type? */
	if (!ds->device_type && private->wcmDeviceType) {
		ds->device_type = private->wcmDeviceType;
		ds->proximity = 1;
	}

	/* all USB data operates from previous context except relative values*/
	ds->relwheel = 0;
	ds->serial_num = private->wcmLastToolSerial;

	/* loop through all events in group */
	for (i=0; i<private->wcmEventCnt; ++i)
	{
		event = private->wcmEvents + i;
		DBG(11, common,
			"event[%d]->type=%d code=%d value=%d\n",
			i, event->type, event->code, event->value);

		/* Check for events to be ignored and skip them up front. */
		if (usbFilterEvent(common, event))
			continue;

		if (common->wcmHasHWTouchSwitch)
		{
			if ((event->type == EV_SW) &&
			    (event->code == SW_MUTE_DEVICE))
			{
				/* touch is disabled when SW_MUTE_DEVICE is set */
				int touch_enabled = (event->value == 0);

				if (touch_enabled != common->wcmHWTouchSwitchState)
				/* this property is only set for touch device */
					wcmUpdateHWTouchProperty(
						common->wcmTouchDevice,
						touch_enabled);
			}
		}

		/* absolute events */
		if (event->type == EV_ABS)
		{
			usbParseAbsEvent(common, event, channel);
			usbParseAbsMTEvent(common, event);
		}
		else if (event->type == EV_REL)
		{
			if (event->code == REL_WHEEL)
			{
				ds->relwheel = -event->value;
				ds->time = (int)GetTimeInMillis();
				common->wcmChannel[channel].dirty |= TRUE;
			}
			else
				LogMessageVerbSigSafe(X_ERROR, 0,
						      "%s: rel event recv'd (%d)!\n",
						      pInfo->name, event->code);
		}
		else if (event->type == EV_KEY)
		{
			usbParseKeyEvent(common, event, channel);
			usbParseBTNEvent(common, event, PAD_CHANNEL);
		}
	} /* next event */

	/* DTF720 and DTF720a don't support eraser */
	if (((common->tablet_id == 0xC0) || (common->tablet_id == 0xC2)) && 
		(ds->device_type == ERASER_ID)) 
	{
		DBG(10, common,
			"DTF 720 doesn't support eraser ");
		return;
	}

	/* verify we have minimal data when entering prox */
	if (ds->proximity && !dslast.proximity && ds->device_type != PAD_ID) {
		struct input_absinfo absinfo;

		if (!ds->x) {
			if (ioctl(priv->pInfo->fd, EVIOCGABS(ABS_X), &absinfo) < 0)
			{
				DBG(-1, common, "unable to ioctl current x value.\n");
				return;
			}
			ds->x = absinfo.value;
		}
		if (!ds->y) {
			if (ioctl(priv->pInfo->fd, EVIOCGABS(ABS_Y), &absinfo) < 0)
			{
				DBG(-1, common, "unable to ioctl current x value.\n");
				return;
			}
			ds->y = absinfo.value;
		}
	}

	/*reset the serial number when the tool is going out */
	if (!ds->proximity)
		private->wcmLastToolSerial = 0;

	private->lastChannel = channel;

	for (c = 0; c < MAX_CHANNELS; c++) {
		ds = &common->wcmChannel[c].work;

		/* walk through all channels */
		if (common->wcmChannel[c].dirty) {
			DBG(10, common, "Dirty flag set on channel %d; sending event.\n", c);
			common->wcmChannel[c].dirty = FALSE;
			/* don't send touch event when touch isn't enabled */
			if (ds->device_type != TOUCH_ID || common->wcmTouch)
				wcmEvent(common, c, ds);
		}
	}
}

/* Quirks to unify the tool and tablet types for GENERIC protocol tablet PCs
 *
 * @param[in,out] keys Contains keys queried from hardware. If a
 *   touchscreen is detected, keys are modified to add BTN_TOOL_FINGER so
 *   that a TOUCH device is created later.
 * @param[in] abs Used to detect multi-touch touchscreens.  When detected,
 *   updates keys to add possibly missing BTN_TOOL_DOUBLETAP.
 * @param[in,out] common Used only for tablet features.  Adds TCM_TPC for
 *   touchscreens so correct defaults, such as absolute mode, are used.
 */
static void usbGenericTouchscreenQuirks(unsigned long *keys,
					unsigned long *abs,
					WacomCommonPtr common)
{
	/* Serial Tablet PC two finger touch devices do not emit
	 * BTN_TOOL_DOUBLETAP since they are not touchpads.
	 */
	if (ISBITSET(abs, ABS_MT_SLOT) && !ISBITSET(keys, BTN_TOOL_DOUBLETAP))
		SETBIT(keys, BTN_TOOL_DOUBLETAP); /* 2FGT */
}

/**
 * Query the device's fd for the key bits and the tablet ID. Returns the ID
 * on success or 0 on failure.
 * For USB devices, we simply copy the information the kernel gives us.
 */
static int usbProbeKeys(InputInfoPtr pInfo)
{
	struct input_id wacom_id;
	WacomDevicePtr  priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr  common = priv->common;
	unsigned long abs[NBITS(ABS_MAX)] = {0};

	if (ioctl(pInfo->fd, EVIOCGBIT(EV_KEY, (sizeof(unsigned long)
						* NBITS(KEY_MAX))), common->wcmKeys) < 0)
	{
		xf86Msg(X_ERROR, "%s: usbProbeKeys unable to "
				"ioctl USB key bits.\n", pInfo->name);
		return 0;
	}

	if (ioctl(pInfo->fd, EVIOCGID, &wacom_id) < 0)
	{
		xf86Msg(X_ERROR, "%s: usbProbeKeys unable to "
				"ioctl Device ID.\n", pInfo->name);
		return 0;
	}

        if (ioctl(pInfo->fd, EVIOCGBIT(EV_ABS, sizeof(abs)), abs) < 0)
	{
		xf86Msg(X_ERROR, "%s: usbProbeKeys unable to ioctl "
			"abs bits.\n", pInfo->name);
		return 0;
	}

	/* The wcmKeys stored above have different meaning for generic
	 * protocol.  Detect that and change default protocol 4 to
	 * generic.
	 */
	usbGenericTouchscreenQuirks(common->wcmKeys, abs, common);

	common->vendor_id = wacom_id.vendor;
	common->tablet_id = wacom_id.product;

	return wacom_id.product;
}


/* vim: set noexpandtab tabstop=8 shiftwidth=8: */
