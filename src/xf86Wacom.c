/*
 * Copyright 1995-2002 by Frederic Lepied, France. <Lepied@XFree86.org> 
 * Copyright 2002-2010 by Ping Cheng, Wacom. <pingc@wacom.com>
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

/*
 * This driver is currently able to handle USB Wacom IV and V, serial ISDV4,
 * and bluetooth protocols.
 *
 * Wacom V protocol work done by Raph Levien <raph@gtk.org> and
 * Frédéric Lepied <lepied@xfree86.org>.
 *
 * Modified for Linux USB by MATSUMURA Namihiko,
 * Daniel Egger, Germany. <egger@suse.de>,
 * Frederic Lepied <lepied@xfree86.org>,
 * Brion Vibber <brion@pobox.com>,
 * Aaron Optimizer Digulla <digulla@hepe.com>,
 * Jonathan Layes <jonathan@layes.com>,
 * John Joganic <jej@j-arkadia.com>,
 * Magnus Vigerlöf <Magnus.Vigerlof@ipbo.se>,
 * Peter Hutterer <peter.hutterer@redhat.com>.
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "xf86Wacom.h"
#include <xf86_OSproc.h>
#include <exevents.h>           /* Needed for InitValuator/Proximity stuff */

#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 7
#include <xserver-properties.h>
#include <X11/extensions/XKB.h>
#include <xkbsrv.h>
#else
#define XIGetKnownProperty(prop) 0
#endif

#ifndef XI86_SERVER_FD
#define XI86_SERVER_FD 0x20
#endif

static int wcmDevOpen(DeviceIntPtr pWcm);
static int wcmReady(InputInfoPtr pInfo);
static void wcmDevReadInput(InputInfoPtr pInfo);
static void wcmDevControlProc(DeviceIntPtr device, PtrCtrl* ctrl);
int wcmDevChangeControl(InputInfoPtr pInfo, xDeviceCtl * control);
static void wcmDevClose(InputInfoPtr pInfo);
static int wcmDevProc(DeviceIntPtr pWcm, int what);

WacomModule gWacomModule =
{
	NULL,           /* input driver pointer */

	/* device procedures */
	wcmDevOpen,
	wcmDevReadInput,
	wcmDevControlProc,
	wcmDevChangeControl,
	wcmDevClose,
	wcmDevProc,
	wcmDevSwitchMode,
};

static void wcmKbdLedCallback(DeviceIntPtr di, LedCtrl * lcp)
{
}

static void wcmKbdCtrlCallback(DeviceIntPtr di, KeybdCtrl* ctrl)
{
}

/*****************************************************************************
 * wcmInitialToolSize --
 *    Initialize logical size and resolution for individual tool.
 ****************************************************************************/

TEST_NON_STATIC void
wcmInitialToolSize(InputInfoPtr pInfo)
{
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common = priv->common;

	/* assign max and resolution here since we don't get them during
	 * the configuration stage */
	priv->maxX = common->wcmMaxTouchX;
	priv->maxY = common->wcmMaxTouchY;
	priv->resolX = common->wcmTouchResolX;
	priv->resolY = common->wcmTouchResolY;

	if (!priv->topX)
		priv->topX = priv->minX;
	if (!priv->topY)
		priv->topY = priv->minY;
	if (!priv->bottomX)
		priv->bottomX = priv->maxX;
	if (!priv->bottomY)
		priv->bottomY = priv->maxY;

	return;
}

static void wcmInitAxis(DeviceIntPtr dev, int axis, Atom label, int min, int max, int res, int min_res, int max_res, int mode) {
	InitValuatorAxisStruct(dev, axis,
#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 7
	                       label,
#endif
	                       min, max, res, min_res, max_res
#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 12
	                       ,mode
#endif
	);
}

/**
 * Initialize the device axes with their proper attributes.
 *
 * For each axis on the device, we need to provide X with its attributes
 * so that its values can be interpreted properly. To support older X
 * servers without axis labels, each axis index has a de-facto meaning.
 * Any de-facto defined axis index left unused is initialized with default
 * attributes.
 */
static int wcmInitAxes(DeviceIntPtr pWcm)
{
	InputInfoPtr pInfo = (InputInfoPtr)pWcm->public.devicePrivate;
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;

	Atom label;
	int index;
	int min, max, min_res, max_res, res;
	int mode;

	/* first valuator: x */
	index = 0;
	label = XIGetKnownProperty(AXIS_LABEL_PROP_ABS_X);
	min = priv->topX;
	max = priv->bottomX;
	min_res = 0;
	max_res = priv->resolX;
	res = priv->resolX;
	mode = Absolute;

	wcmInitAxis(pInfo->dev, index, label, min, max, res, min_res, max_res, mode);


	/* second valuator: y */
	index = 1;
	label = XIGetKnownProperty(AXIS_LABEL_PROP_ABS_Y);
	min = priv->topY;
	max = priv->bottomY;
	min_res = 0;
	max_res = priv->resolY;
	res = priv->resolY;
	mode = Absolute;

	wcmInitAxis(pInfo->dev, index, label, min, max, res, min_res, max_res, mode);


	/* third valuator: pressure */
	index = 2;
	label = XIGetKnownProperty(AXIS_LABEL_PROP_ABS_PRESSURE);
	mode = Absolute;
	min_res = max_res = res = 1;
	min = 0;
	max = priv->maxCurve;

	wcmInitAxis(pInfo->dev, index, label, min, max, res, min_res, max_res, mode);


	return TRUE;
}

/*****************************************************************************
 * wcmDevInit --
 *    Set up the device's buttons, axes and keys
 ****************************************************************************/

static int wcmDevInit(DeviceIntPtr pWcm)
{
	InputInfoPtr pInfo = (InputInfoPtr)pWcm->public.devicePrivate;
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common =	priv->common;
	unsigned char butmap[WCM_MAX_BUTTONS+1];
	int nbaxes, nbbuttons, nbkeys;
	int loop;
#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 7
        Atom btn_labels[WCM_MAX_BUTTONS] = {0};
        Atom axis_labels[MAX_VALUATORS] = {0};
#endif

	/* Detect tablet configuration, if possible */
	if (priv->common->wcmModel->DetectConfig)
		priv->common->wcmModel->DetectConfig (pInfo);

	nbaxes = priv->naxes;       /* X, Y, Pressure, Tilt-X, Tilt-Y, Wheel */
	nbbuttons = priv->nbuttons; /* Use actual number of buttons, if possible */

	/* if more than 3 buttons, offset by the four scroll buttons,
	 * otherwise, alloc 7 buttons for scroll wheel. */
	nbbuttons = min(max(nbbuttons + 4, 7), WCM_MAX_BUTTONS);
	nbkeys = nbbuttons;         /* Same number of keys since any button may be 
	                             * configured as an either mouse button or key */

	DBG(10, priv,
		"(%s) %d buttons, %d keys, %d axes\n",
		pInfo->type_name,
		nbbuttons, nbkeys, nbaxes);

	for(loop=1; loop<=nbbuttons; loop++)
		butmap[loop] = loop;

	/* FIXME: button labels would be nice */
	if (InitButtonClassDeviceStruct(pInfo->dev, nbbuttons,
#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 7
					btn_labels,
#endif
					butmap) == FALSE)
	{
		xf86Msg(X_ERROR, "%s: unable to allocate Button class device\n", pInfo->name);
		return FALSE;
	}

	if (InitFocusClassDeviceStruct(pInfo->dev) == FALSE)
	{
		xf86Msg(X_ERROR, "%s: unable to init Focus class device\n", pInfo->name);
		return FALSE;
	}

	if (InitPtrFeedbackClassDeviceStruct(pInfo->dev,
		wcmDevControlProc) == FALSE)
	{
		xf86Msg(X_ERROR, "%s: unable to init ptr feedback\n", pInfo->name);
		return FALSE;
	}

	if (InitProximityClassDeviceStruct(pInfo->dev) == FALSE)
	{
			xf86Msg(X_ERROR, "%s: unable to init proximity class device\n", pInfo->name);
			return FALSE;
	}

	if (!nbaxes || nbaxes > 3)
		nbaxes = priv->naxes = 3;

	/* axis_labels is just zeros, we set up each valuator with the
	 * correct property later */
	if (InitValuatorClassDeviceStruct(pInfo->dev, nbaxes,
#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 7
					  axis_labels,
#endif
					  GetMotionHistorySize(),
					  (is_absolute(pInfo) ?  Absolute : Relative) | OutOfProximity) == FALSE)
	{
		xf86Msg(X_ERROR, "%s: unable to allocate Valuator class device\n", pInfo->name);
		return FALSE;
	}


#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 7
	if (!InitKeyboardDeviceStruct(pInfo->dev, NULL, NULL, wcmKbdCtrlCallback)) {
		xf86Msg(X_ERROR, "%s: unable to init kbd device struct\n", pInfo->name);
		return FALSE;
	}
#endif
	if(InitLedFeedbackClassDeviceStruct (pInfo->dev, wcmKbdLedCallback) == FALSE) {
		xf86Msg(X_ERROR, "%s: unable to init led feedback device struct\n", pInfo->name);
		return FALSE;
	}

#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 16
	if (!InitTouchClassDeviceStruct(pInfo->dev, common->wcmMaxContacts,
					XIDirectTouch, 2))
	{
		xf86Msg(X_ERROR, "Unable to init touch class device struct!\n");
		return FALSE;
	}
	priv->common->touch_mask = valuator_mask_new(2);
#endif

	wcmInitialToolSize(pInfo);

	if (!wcmInitAxes(pWcm))
		return FALSE;

	InitWcmDeviceProperties(pInfo);
	XIRegisterPropertyHandler(pInfo->dev, wcmSetProperty, wcmGetProperty, wcmDeleteProperty);

	return TRUE;
}

/*****************************************************************************
 * wcmEventAutoDevProbe -- Probe for right input device
 ****************************************************************************/
#define DEV_INPUT_EVENT "/dev/input/event%d"
#define EVDEV_MINORS    32
char *wcmEventAutoDevProbe (InputInfoPtr pInfo)
{
	/* We are trying to find the right eventX device */
	int i = 0, wait = 0;
	const int max_wait = 2000;

	/* If device is not available after Resume, wait some ms */
	while (wait <= max_wait) 
	{
		for (i = 0; i < EVDEV_MINORS; i++) 
		{
			char fname[64];
			sprintf(fname, DEV_INPUT_EVENT, i);
		}
		wait += 100;
		xf86Msg(X_ERROR, "%s: waiting 100 msec (total %dms) for device to become ready\n", pInfo->name, wait);
		usleep(100*1000);
	}
	xf86Msg(X_ERROR, "%s: no Wacom event device found (checked %d nodes, waited %d msec)\n",
		pInfo->name, i + 1, wait);
	xf86Msg(X_ERROR, "%s: unable to probe device\n", pInfo->name);
	return NULL;
}

/*****************************************************************************
 * wcmOpen --
 ****************************************************************************/

Bool wcmOpen(InputInfoPtr pInfo)
{
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common = priv->common;

	DBG(1, priv, "opening device file\n");

	pInfo->fd = xf86OpenSerial(pInfo->options);
	if (pInfo->fd < 0)
	{
		xf86Msg(X_ERROR, "%s: Error opening %s (%s)\n", pInfo->name,
			common->device_path, strerror(errno));
		return !Success;
	}

	return Success;
}

/*****************************************************************************
 * wcmClose --
 ****************************************************************************/

void wcmClose(InputInfoPtr pInfo)
{
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;

	DBG(1, priv, "closing device file\n");

	if (pInfo->fd > -1 && !(pInfo->flags & XI86_SERVER_FD)) {
		xf86CloseSerial(pInfo->fd);
		pInfo->fd = -1;
	}
}

/*****************************************************************************
 * wcmDevOpen --
 *    Open the physical device and init information structs.
 ****************************************************************************/

static int wcmDevOpen(DeviceIntPtr pWcm)
{
	InputInfoPtr pInfo = (InputInfoPtr)pWcm->public.devicePrivate;
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common = priv->common;
	WacomModelPtr model = common->wcmModel;
	struct stat st;

	DBG(10, priv, "\n");

	/* If fd management is done by the server, skip common fd handling */
	if (pInfo->flags & XI86_SERVER_FD)
		goto got_fd;

	/* open file, if not already open */
	if (common->fd_refs == 0)
	{
		if ((wcmOpen (pInfo) != Success) || !common->device_path)
		{
			DBG(1, priv, "Failed to open device (fd=%d)\n", pInfo->fd);
			wcmClose(pInfo);
			return FALSE;
		}

		if (fstat(pInfo->fd, &st) == -1)
		{
			/* can not access major/minor */
			DBG(1, priv, "stat failed (%s).\n", strerror(errno));

			/* older systems don't support the required ioctl.
			 * So, we have to let it pass */
			common->min_maj = 0;
		}
		else
			common->min_maj = st.st_rdev;
		common->fd = pInfo->fd;
		common->fd_refs = 1;
	}

	/* Grab the common descriptor, if it's available */
	if (pInfo->fd < 0)
	{
		pInfo->fd = common->fd;
		common->fd_refs++;
	}

got_fd:
	/* start the tablet data */
	if (model->Start && (model->Start(pInfo) != Success))
		return !Success;

	return TRUE;
}

static int wcmReady(InputInfoPtr pInfo)
{
#ifdef DEBUG
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
#endif
	int n = xf86WaitForInput(pInfo->fd, 0);
	DBG(10, priv, "%d numbers of data\n", n);

	if (n >= 0) return n ? 1 : 0;
	xf86Msg(X_ERROR, "%s: select error: %s\n", pInfo->name, strerror(errno));
	return 0;
}

/*****************************************************************************
 * wcmDevReadInput --
 *   Read the device on IO signal
 ****************************************************************************/

static void wcmDevReadInput(InputInfoPtr pInfo)
{
	int loop=0;
	#define MAX_READ_LOOPS 10

	/* move data until we exhaust the device */
	for (loop=0; loop < MAX_READ_LOOPS; ++loop)
	{
		/* verify that there is still data in pipe */
		if (!wcmReady(pInfo)) break;

		/* dispatch */
		if (!wcmReadPacket(pInfo))
			break;
	}

#ifdef DEBUG
	/* report how well we're doing */
	if (loop > 0)
	{
		WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;

		if (loop >= MAX_READ_LOOPS)
			DBG(1, priv, "Can't keep up!!!\n");
		else
			DBG(10, priv, "Read (%d)\n",loop);
	}
#endif
}

Bool wcmReadPacket(InputInfoPtr pInfo)
{
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common = priv->common;
	int len, pos, cnt, remaining;

	DBG(10, common, "fd=%d\n", pInfo->fd);

	remaining = sizeof(common->buffer) - common->bufpos;

	DBG(1, common, "pos=%d remaining=%d\n", common->bufpos, remaining);

	/* fill buffer with as much data as we can handle */
	len = xf86ReadSerial(pInfo->fd,
		common->buffer + common->bufpos, remaining);

	if (len <= 0)
	{
		/* for all other errors, hope that the hotplugging code will
		 * remove the device */
		if (errno != EAGAIN && errno != EINTR)
			LogMessageVerbSigSafe(X_ERROR, 0,
					      "%s: Error reading wacom device : %s\n", pInfo->name, strerror(errno));
		if (errno == ENODEV)
			xf86RemoveEnabledDevice(pInfo);

		return FALSE;
	}

	/* account for new data */
	common->bufpos += len;
	DBG(10, common, "buffer has %d bytes\n", common->bufpos);

	len = common->bufpos;
	pos = 0;

	while (len > 0)
	{
		/* parse packet */
		cnt = common->wcmModel->Parse(pInfo, common->buffer + pos, len);
		if (cnt <= 0)
		{
			if (cnt < 0)
				DBG(1, common, "Misbehaving parser returned %d\n",cnt);
			break;
		}
		pos += cnt;
		len -= cnt;
	}

	/* if half a packet remains, move it down */
	if (len)
	{
		DBG(7, common, "MOVE %d bytes\n", common->bufpos - pos);
		memmove(common->buffer,common->buffer+pos, len);
	}

	common->bufpos = len;

	return TRUE;
}

int wcmDevChangeControl(InputInfoPtr pInfo, xDeviceCtl * control)
{
#ifdef DEBUG
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	DBG(3, priv, "\n");
#endif
	return Success;
}

/*****************************************************************************
 * wcmDevControlProc --
 ****************************************************************************/

static void wcmDevControlProc(DeviceIntPtr device, PtrCtrl* ctrl)
{
#ifdef DEBUG
	InputInfoPtr pInfo = (InputInfoPtr)device->public.devicePrivate;
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;

	DBG(4, priv, "called\n");
#endif
	return;
}

/*****************************************************************************
 * wcmDevClose --
 ****************************************************************************/

static void wcmDevClose(InputInfoPtr pInfo)
{
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common = priv->common;

	/* If fd management is done by the server, skip common fd handling */
	if (pInfo->flags & XI86_SERVER_FD)
		return;

	DBG(4, priv, "Wacom number of open devices = %d\n", common->fd_refs);

	if (pInfo->fd >= 0)
	{
		if (!--common->fd_refs)
			wcmClose(pInfo);
		pInfo->fd = -1;
	}
}

static void wcmEnableDisableTool(DeviceIntPtr dev, Bool enable)
{
	InputInfoPtr	pInfo	= dev->public.devicePrivate;
	WacomDevicePtr	priv	= pInfo->private;
	WacomToolPtr	tool	= priv->tool;

	tool->enabled = enable;
}

static void wcmEnableTool(DeviceIntPtr dev)
{
	wcmEnableDisableTool(dev, TRUE);
}
static void wcmDisableTool(DeviceIntPtr dev)
{
	wcmEnableDisableTool(dev, FALSE);
}

/*****************************************************************************
 * wcmDevProc --
 *   Handle the initialization, etc. of a wacom tablet. Called by the server
 *   once with DEVICE_INIT when the device becomes available, then
 *   DEVICE_ON/DEVICE_OFF possibly multiple times as the device is enabled
 *   and disabled. DEVICE_CLOSE is called before removal of the device.
 ****************************************************************************/

static int wcmDevProc(DeviceIntPtr pWcm, int what)
{
	InputInfoPtr pInfo = (InputInfoPtr)pWcm->public.devicePrivate;
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	Status rc = !Success;

	DBG(2, priv, "BEGIN dev=%p priv=%p "
			"type=%s flags=%d fd=%d what=%s\n",
			(void *)pWcm, (void *)priv,
			pInfo->type_name,
			priv->flags, pInfo ? pInfo->fd : -1,
			(what == DEVICE_INIT) ? "INIT" :
			(what == DEVICE_OFF) ? "OFF" :
			(what == DEVICE_ON) ? "ON" :
			(what == DEVICE_CLOSE) ? "CLOSE" : "???");

	switch (what)
	{
		case DEVICE_INIT:
			if (!wcmDevInit(pWcm))
				goto out;
			break;

		case DEVICE_ON:
			if (!wcmDevOpen(pWcm))
				goto out;
			wcmEnableTool(pWcm);
			xf86AddEnabledDevice(pInfo);
			pWcm->public.on = TRUE;
			break;

		case DEVICE_OFF:
			TimerCancel(priv->tap_timer);
			TimerCancel(priv->serial_timer);
			TimerCancel(priv->touch_timer);
			wcmDisableTool(pWcm);
			if (pInfo->fd >= 0)
			{
				xf86RemoveEnabledDevice(pInfo);
				wcmDevClose(pInfo);
			}
			pWcm->public.on = FALSE;
			break;
		case DEVICE_CLOSE:
			break;
#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) * 100 + GET_ABI_MINOR(ABI_XINPUT_VERSION) >= 1901
		case DEVICE_ABORT:
			break;
#endif
		default:
			xf86Msg(X_ERROR, "%s: invalid mode=%d. This is an X server bug.\n",
				pInfo->name, what);
			goto out;
	} /* end switch */

	rc = Success;

out:
	if (rc != Success)
		DBG(1, priv, "Failed during %d\n", what);
	return rc;
}

/* vim: set noexpandtab tabstop=8 shiftwidth=8: */
