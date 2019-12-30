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
	int wcmMTChannel;
	int wcmEventCnt;
	struct input_event wcmEvents[MAX_USB_EVENTS];
	uint32_t wcmEventFlags;      /* event types received in this frame */
	int nbuttons;                /* total number of buttons */
	int lastChannel;
} wcmUSBData;

static Bool usbDetect(InputInfoPtr);
static Bool usbWcmInit(InputInfoPtr pDev, char* id, size_t id_len, float *version);
static int usbProbeKeys(InputInfoPtr pInfo);
static int usbStart(InputInfoPtr pInfo);
int usbWcmGetRanges(InputInfoPtr pInfo);
static int usbParse(InputInfoPtr pInfo, const unsigned char* data, int len);
static int usbDetectConfig(InputInfoPtr pInfo);
static void usbParseEvent(InputInfoPtr pInfo,
	const struct input_event* event);
static void usbParseSynEvent(InputInfoPtr pInfo,
			     const struct input_event *event);
static void usbDispatchEvents(InputInfoPtr pInfo);
static int usbChooseChannel(WacomCommonPtr common, unsigned int serial);

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

static Bool usbWcmInit(InputInfoPtr pInfo, char* id, size_t id_len, float *version)
{
	struct input_id sID;
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common = priv->common;

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

	*version = 0.0;

	common->wcmModel = &usbUnknown;
	common->wcmResolX = common->wcmResolY = 1016;

	return Success;
}

int usbWcmGetRanges(InputInfoPtr pInfo)
{
	struct input_absinfo absinfo;
	unsigned long ev[NBITS(EV_MAX)] = {0};
	unsigned long abs[NBITS(ABS_MAX)] = {0};
	unsigned long sw[NBITS(SW_MAX)] = {0};
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common =	priv->common;

	if (ioctl(pInfo->fd, EVIOCGBIT(0 /*EV*/, sizeof(ev)), ev) < 0)
	{
		xf86Msg(X_ERROR, "%s: unable to ioctl event bits.\n", pInfo->name);
		return !Success;
	}

	if (!ISBITSET(ev,EV_ABS))
	{
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
		xf86Msg(X_ERROR, "%s: unable to ioctl xmax value.\n", pInfo->name);
		return !Success;
	}

	if (absinfo.maximum <= 0)
	{
		xf86Msg(X_ERROR, "%s: xmax value is %d, expected > 0.\n",
			pInfo->name, absinfo.maximum);
		return !Success;
	}

	common->wcmMaxTouchX = absinfo.maximum;

	if (absinfo.resolution > 0)
		common->wcmTouchResolX = absinfo.resolution * 1000;

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

	common->wcmMaxTouchY = absinfo.maximum;

	if (absinfo.resolution > 0)
		common->wcmTouchResolY = absinfo.resolution * 1000;


	/* max z cannot be configured */
	if (ISBITSET(abs, ABS_PRESSURE) &&
			!ioctl(pInfo->fd, EVIOCGABS(ABS_PRESSURE), &absinfo))
		common->wcmMaxZ = absinfo.maximum;

	if (ISBITSET(abs, ABS_MT_SLOT))
	{
		if (!ioctl(pInfo->fd, EVIOCGABS(ABS_MT_SLOT), &absinfo))
			common->wcmMaxContacts = absinfo.maximum + 1;
	}

	if (ioctl(pInfo->fd, EVIOCGBIT(EV_SW, sizeof(sw)), sw) < 0)
	{
		xf86Msg(X_ERROR, "%s: unable to ioctl sw bits.\n", pInfo->name);
		return 0;
	}

	return Success;
}

static int usbDetectConfig(InputInfoPtr pInfo)
{
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common = priv->common;
	wcmUSBData *usbdata = common->private;

	DBG(10, common, "\n");

	priv->nbuttons = usbdata->nbuttons;

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
 * Find an appropriate channel to track the specified tool's state in.
 * If the tool is already in proximity, the channel currently being used
 * to store its state will be returned. Otherwise, an arbitrary available
 * channel will be cleaned and returned. Up to MAX_CHANNEL tools can be
 * tracked concurrently by driver.
 *
 * @param[in] common
 * @param[in] serial       Serial number of tool
 * @return                 Channel number to track the tool's state
 */
static int usbChooseChannel(WacomCommonPtr common, unsigned int serial)
{
	/* figure out the channel to use based on serial number */
	int i, channel = -1;

	/* find existing channel */
	if (channel < 0)
	{
		for (i=0; i<MAX_CHANNELS; i++)
		{
			if (common->wcmChannel[i].work.proximity &&
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

	if (event->type == EV_SYN)
		usbParseSynEvent(pInfo, event);
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
	const uint32_t significant_event_types = ~(1 << EV_SYN);

	if (event->code != SYN_REPORT)
		return;

	/* ignore events without information */
	if ((private->wcmEventCnt < 2) && private->wcmLastToolSerial)
	{
		DBG(3, common, "%s: dropping empty event for serial %d\n",
		    pInfo->name, private->wcmLastToolSerial);
		goto skipEvent;
	}

	/* If all we get in an event frame is EV_SYNC, we don't have
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
	/* The following list is a set of
	 * duplicate data from one slot and needs to be filtered out.
	 */
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
		/* filter ST for MT */
		switch(event->code)
		{
			case ABS_X:
			case ABS_Y:
			case ABS_PRESSURE:
				return 1;
		}
	}

	return 0;
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
				private->wcmMTChannel = usbChooseChannel(common, serial);
				if (private->wcmMTChannel < 0)
					return;
				ds = &common->wcmChannel[private->wcmMTChannel].work;
				ds->serial_num = serial;
			}
			break;

		case ABS_MT_TRACKING_ID:
			ds->proximity = (event->value != -1);
			/* set this here as type for this channel doesn't get set in usbDispatchEvent() */
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

	private->wcmLastToolSerial = 1;
	channel = usbChooseChannel(common, private->wcmLastToolSerial);

	/* couldn't decide channel? invalid data */
	if (channel == -1) {
		private->wcmEventCnt = 0;
		return;
	}

	ds = &common->wcmChannel[channel].work;
	dslast = common->wcmChannel[channel].valid.state;

	/* all USB data operates from previous context except relative values*/
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

		/* absolute events */
		if (event->type == EV_ABS)
		{
			usbParseAbsMTEvent(common, event);
		}
	} /* next event */

	/* verify we have minimal data when entering prox */
	if (ds->proximity && !dslast.proximity) {
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
			if (common->wcmTouch)
				wcmEvent(common, c, ds);
		}
	}
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

	common->vendor_id = wacom_id.vendor;
	common->tablet_id = wacom_id.product;

	return wacom_id.product;
}


/* vim: set noexpandtab tabstop=8 shiftwidth=8: */
