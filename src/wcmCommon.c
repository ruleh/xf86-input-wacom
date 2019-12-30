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

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "xf86Wacom.h"
#include "Xwacom.h"
#include "wcmFilter.h"
#include "wcmTouchFilter.h"
#include <xkbsrv.h>
#include <xf86_OSproc.h>


struct _WacomDriverRec WACOM_DRIVER = {
	.active = NULL,
};

/* X servers pre 1.9 didn't copy data passed into xf86Post*Event.
 * Data passed in would be modified, requiring the driver to copy the
 * data beforehand.
 */
#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) < 11
static int v[MAX_VALUATORS];
static int *VCOPY(const int *valuators, int nvals)
{
	memcpy(v, valuators, nvals * sizeof(int));
	return v;
}
#else /* ABI >= 11 */
#define VCOPY(vals, nval) (vals)
#endif



/*****************************************************************************
 * Static functions
 ****************************************************************************/

static void commonDispatchDevice(InputInfoPtr pInfo,
				 const WacomChannelPtr pChannel);
/*****************************************************************************
 * Utility functions
 ****************************************************************************/

/**
 * @return TRUE if the device is set to abolute mode, or FALSE otherwise
 */
Bool is_absolute(InputInfoPtr pInfo)
{
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	return !!(priv->flags & ABSOLUTE_FLAG);
}

/**
 * Set the device to absolute or relative mode
 *
 * @param absolute TRUE to set the device to absolute mode.
 */
void set_absolute(InputInfoPtr pInfo, Bool absolute)
{
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;

	if (absolute)
		priv->flags |= ABSOLUTE_FLAG;
	else
		priv->flags &= ~ABSOLUTE_FLAG;
}

/* rotate x and y before post X inout events */
void wcmRotateAndScaleCoordinates(InputInfoPtr pInfo, int* x, int* y)
{
	WacomDevicePtr priv = (WacomDevicePtr) pInfo->private;
	WacomCommonPtr common = priv->common;
	DeviceIntPtr dev = pInfo->dev;
	AxisInfoPtr axis_x, axis_y;
	int tmp_coord;

	/* scale into on topX/topY area */
	axis_x = &dev->valuator->axes[0];
	axis_y = &dev->valuator->axes[1];

	/* Don't try to scale relative axes */
	if (axis_x->max_value > axis_x->min_value)
		*x = xf86ScaleAxis(*x, axis_x->max_value, axis_x->min_value,
				   priv->bottomX, priv->topX);

	if (axis_y->max_value > axis_y->min_value)
		*y = xf86ScaleAxis(*y, axis_y->max_value, axis_y->min_value,
				   priv->bottomY, priv->topY);

	/* coordinates are now in the axis rage we advertise for the device */

	if (common->wcmRotate == ROTATE_CW || common->wcmRotate == ROTATE_CCW)
	{
		tmp_coord = *x;

		*x = xf86ScaleAxis(*y,
				   axis_x->max_value, axis_x->min_value,
				   axis_y->max_value, axis_y->min_value);
		*y = xf86ScaleAxis(tmp_coord,
				   axis_y->max_value, axis_y->min_value,
				   axis_x->max_value, axis_x->min_value);
	}

	if (common->wcmRotate == ROTATE_CW)
		*y = axis_y->max_value - (*y - axis_y->min_value);
	else if (common->wcmRotate == ROTATE_CCW)
		*x = axis_x->max_value - (*x - axis_x->min_value);
	else if (common->wcmRotate == ROTATE_HALF)
	{
		*x = axis_x->max_value - (*x - axis_x->min_value);
		*y = axis_y->max_value - (*y - axis_y->min_value);
	}


	DBG(10, priv, "rotate/scaled to %d/%d\n", *x, *y);
}

static void wcmUpdateOldState(const InputInfoPtr pInfo,
			      const WacomDeviceState *ds, int currentX, int currentY)
{
	const WacomDevicePtr priv = (WacomDevicePtr) pInfo->private;

	priv->oldState = *ds;
	priv->oldState.x = currentX;
	priv->oldState.y = currentY;
}

/* Send events for all tools but pads */
static void
wcmSendNonPadEvents(InputInfoPtr pInfo, const WacomDeviceState *ds,
		    int first_val, int num_vals, int *valuators)
{
	WacomDevicePtr priv = (WacomDevicePtr) pInfo->private;

	if (!is_absolute(pInfo))
	{
		valuators[0] -= priv->oldState.x;
		valuators[1] -= priv->oldState.y;
		valuators[2] -= priv->oldState.pressure;
	}

	/* coordinates are ready we can send events */
	if (ds->proximity && !(priv->flags & TOUCHKEY_FLAG))
	{
		xf86PostMotionEventP(pInfo->dev, is_absolute(pInfo),
				     first_val, num_vals,
				     VCOPY(valuators, num_vals));

	}
}

/*****************************************************************************
 * wcmSendEvents --
 *   Send events according to the device state.
 ****************************************************************************/

void wcmSendEvents(InputInfoPtr pInfo, const WacomDeviceState* ds)
{
	unsigned int serial = ds->serial_num;
	int x = ds->x;
	int y = ds->y;
	int z = ds->pressure;
	WacomDevicePtr priv = (WacomDevicePtr) pInfo->private;
	int valuators[priv->naxes];

	if (priv->serial && serial != priv->serial)
	{
		DBG(10, priv, "serial number"
				" is %u but your system configured %u",
				serial, (int)priv->serial);
		return;
	}

	wcmUpdateSerial(pInfo, serial);

	/* don't move the cursor when going out-prox */
	if (!ds->proximity)
	{
		x = priv->oldState.x;
		y = priv->oldState.y;
	}

	DBG(7, priv, "[%s] o_prox=%s x=%d y=%d z=%d ",
		pInfo->type_name,
		priv->oldState.proximity ? "true" : "false",
		x, y, z);


	if (ds->proximity)
		wcmRotateAndScaleCoordinates(pInfo, &x, &y);



	DBG(6, priv, "%s prox=%d\tx=%d"
		"\ty=%d\tz=%d"
		"\tserial=%u\n",
		is_absolute(pInfo) ? "abs" : "rel",
		ds->proximity,
		x, y, z, serial);

	/* when entering prox, replace the zeroed-out oldState with a copy of
	 * the current state to prevent jumps. reset the prox and button state
	 * to zero to properly detect changes.
	 */
	if(!priv->oldState.proximity)
	{
		wcmUpdateOldState(pInfo, ds, x, y);
		priv->oldState.proximity = 0;
	}

	valuators[0] = x;
	valuators[1] = y;
	valuators[2] = z;

	/* don't move the cursor if in gesture mode (except drag mode) */
	if (wcmTouchNeedSendEvents(priv->common))
		wcmSendNonPadEvents(pInfo, ds, 0, priv->naxes, valuators);

	if (ds->proximity)
		wcmUpdateOldState(pInfo, ds, x, y);
	else
	{
		priv->oldState = OUTPROX_STATE;
		priv->oldState.serial_num = serial;
		wcmUpdateSerial(pInfo, 0);
	}
}

/**
 * Determine whether device state has changed enough to warrant further
 * processing. The driver's "suppress" setting decides how much
 * movement/state change must occur before we process events to avoid
 * overloading the server with minimal changes (and getting fuzzy events).
 * wcmCheckSuppress ensures that events meet this standard.
 *
 * @param dsOrig Previous device state
 * @param dsNew Current device state
 *
 * @retval SUPPRESS_ALL Ignore this event completely.
 * @retval SUPPRESS_NONE Process event normally.
 * @retval SUPPRESS_NON_MOTION Suppress all data but motion data.
 */
TEST_NON_STATIC enum WacomSuppressMode
wcmCheckSuppress(WacomCommonPtr common,
		 const WacomDeviceState* dsOrig,
		 WacomDeviceState* dsNew)
{
	int suppress = common->wcmSuppress;
	enum WacomSuppressMode returnV = SUPPRESS_NONE;

	/* Ignore all other changes that occur after initial out-of-prox. */
	if (!dsNew->proximity && !dsOrig->proximity)
		return SUPPRESS_ALL;

	/* Never ignore proximity changes. */
	if (dsOrig->proximity != dsNew->proximity) goto out;

	/* FIXME: we should have different suppress values for different
	 * axes with vastly different ranges.
	 */
	if (abs(dsOrig->pressure - dsNew->pressure) > suppress) goto out;

	returnV = SUPPRESS_ALL;

out:
	/* Special handling for cursor: if nothing else changed but the
	 * pointer x/y, suppress all but cursor movement. This return value
	 * is used in commonDispatchDevice to short-cut event processing.
	 */
	if ((abs(dsOrig->x - dsNew->x) > suppress) || 
			(abs(dsOrig->y - dsNew->y) > suppress)) 
	{
		if (returnV == SUPPRESS_ALL)
			returnV = SUPPRESS_NON_MOTION;
	}
	else /* don't move cursor */
	{
		dsNew->x = dsOrig->x;
		dsNew->y = dsOrig->y;
	}

	DBG(10, common, "level = %d"
		" return value = %d\n", suppress, returnV);
	return returnV;
}

/**
 * Find the device the current events are meant for. If multiple tools are
 * configured on this tablet, the one that matches the serial number for the
 * current device state is returned. If none match, the tool that has a
 * serial of 0 is returned.
 *
 * @param ds The current device state as read from the fd
 * @return The tool that should be used to emit the current events.
 */
static WacomToolPtr findTool(const WacomCommonPtr common,
			     const WacomDeviceState *ds)
{
	WacomToolPtr tooldefault = NULL;
	WacomToolPtr tool = NULL;

	/* 1: Find the tool (the one with correct serial or in second
	 * hand, the one with serial set to 0 if no match with the
	 * specified serial exists) that is used for this event */
	for (tool = common->wcmTool; tool; tool = tool->next)
	{
		if (tool->serial == ds->serial_num)
			break;
		else if (!tool->serial)
			tooldefault = tool;
	}

	/* Use default tool (serial == 0) if no specific was found */
	if (!tool)
		tool = tooldefault;

	return tool;
}

/*****************************************************************************
 * wcmEvent -
 *   Handles suppression, transformation, filtering, and event dispatch.
 ****************************************************************************/

void wcmEvent(WacomCommonPtr common, unsigned int channel,
	const WacomDeviceState* pState)
{
	WacomDeviceState ds;
	WacomChannelPtr pChannel;
	InputInfoPtr pInfo;
	WacomToolPtr tool;
	WacomDevicePtr priv;
	pChannel = common->wcmChannel + channel;

	DBG(10, common, "channel = %d\n", channel);

	/* sanity check the channel */
	if (channel >= MAX_CHANNELS)
		return;
	
	/* we must copy the state because certain types of filtering
	 * will need to change the values (ie. for error correction) */
	ds = *pState;

	DBG(10, common,
		"c=%d s=%u x=%d y=%d "
		"p=%d "
		"px=%d st=%d cs=%d \n",
		channel,
		ds.serial_num,
		ds.x, ds.y,
		ds.pressure,
		ds.proximity, ds.sample,
		pChannel->nSamples);

	/* Find the device the current events are meant for */
	tool = findTool(common, &ds);
	if (!tool || !tool->device)
	{
		DBG(11, common, "no device matches with serial=%u\n",
		    ds.serial_num);
		return;
	}

	/* Tool on the tablet when driver starts. This sometime causes
	 * access errors to the device */
	if (!tool->enabled) {
		LogMessageVerbSigSafe(X_ERROR, 0, "tool not initialized yet. Skipping event. \n");
		return;
	}

	pInfo = tool->device;
	priv = pInfo->private;
	DBG(11, common, "tool for %s\n", pInfo->name);

	/* JEJ - Do not move this code without discussing it with me.
	 * The device state is invariant of any filtering performed below.
	 * Changing the device state after this point can and will cause
	 * a feedback loop resulting in oscillations, error amplification,
	 * unnecessary quantization, and other annoying effects. */

	/* save channel device state and device to which last event went */
	memmove(pChannel->valid.states + 1,
		pChannel->valid.states,
		sizeof(WacomDeviceState) * (common->wcmRawSample - 1));
	pChannel->valid.state = ds; /*save last raw sample */
	if (pChannel->nSamples < common->wcmRawSample) ++pChannel->nSamples;

	/* arbitrate pointer control */
	if (WACOM_DRIVER.active != NULL && priv != WACOM_DRIVER.active) {
		wcmSoftOutEvent(WACOM_DRIVER.active->pInfo);
		wcmCancelGesture(WACOM_DRIVER.active->pInfo);
	}
	if (ds.proximity)
		WACOM_DRIVER.active = priv;
	else
		WACOM_DRIVER.active = NULL;

	if (common->wcmTouch)
	{
		wcmGestureFilter(priv, ds.serial_num - 1);
#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 16
		/*
		 * When using XI 2.2 multitouch events don't do common dispatching
		 * for direct touch devices
		 */
		if (!common->wcmGesture)
			return;
#endif
	}

	/* For touch, only first finger moves the cursor */
	if (common->wcmTouch && ds.serial_num == 1)
		commonDispatchDevice(pInfo, pChannel);
}

static void commonDispatchDevice(InputInfoPtr pInfo,
				 const WacomChannelPtr pChannel)
{
	WacomDevicePtr priv = pInfo->private;
	WacomCommonPtr common = priv->common;
	WacomDeviceState filtered;
	enum WacomSuppressMode suppress;

	filtered = pChannel->valid.state;

	/* Device transformations come first */
	if (priv->serial && filtered.serial_num != priv->serial)
	{
		DBG(10, priv, "serial number"
			" is %u but your system configured %u\n",
			filtered.serial_num, priv->serial);
		return;
	}

	/* Optionally filter values only while in proximity */
	if (filtered.proximity)
	{
		/* Start filter fresh when entering proximity */
		if (!priv->oldState.proximity)
			wcmResetSampleCounter(pChannel);

		wcmFilterCoord(common,pChannel,&filtered);
	}

	/* skip event if we don't have enough movement */
	suppress = wcmCheckSuppress(common, &priv->oldState, &filtered);
	if (suppress == SUPPRESS_ALL)
		return;

	/* User-requested filtering comes next */

	/* User-requested transformations come last */

	if (!is_absolute(pInfo)) 
	{
		/* To improve the accuracy of relative x/y,
		 * don't send motion event when there is no movement.
		 */
		double deltx = filtered.x - priv->oldState.x;
		double delty = filtered.y - priv->oldState.y;

		/* less than one device coordinate movement? */
		if (fabs(deltx)<1 && fabs(delty)<1)
		{
			/* We have no other data in this event, skip */
			if (suppress == SUPPRESS_NON_MOTION)
			{
				DBG(10, common, "Ignore non-movement relative data \n");
				return;
			}

			/* send other events, such as button */
			filtered.x = priv->oldState.x;
			filtered.y = priv->oldState.y;
		}
	}
	wcmSendEvents(pInfo, &filtered);
}

/*****************************************************************************
 * wcmInitTablet -- common initialization for all tablets
 ****************************************************************************/

int wcmInitTablet(InputInfoPtr pInfo, const char* id, float version)
{
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common = priv->common;
	WacomModelPtr model = common->wcmModel;

	/* Initialize the tablet */
	common->wcmPktLength = sizeof(struct input_event);

	/* Get tablet resolution */
	if (model->GetResolution)
		model->GetResolution(pInfo);

	/* Get tablet range */
	if (model->GetRanges && (model->GetRanges(pInfo) != Success))
		return !Success;

	xf86Msg(X_PROBED, "%s: maxX=%d maxY=%d maxZ=%d "
		"resX=%d resY=%d \n",
		pInfo->name,
		common->wcmMaxTouchX, common->wcmMaxTouchY,
		common->wcmMaxZ,
		common->wcmTouchResolX, common->wcmTouchResolY);

	return Success;
}

/* Send a soft prox-out event for the device */
void wcmSoftOutEvent(InputInfoPtr pInfo)
{
	WacomDeviceState out = OUTPROX_STATE;
	WacomDevicePtr priv = (WacomDevicePtr) pInfo->private;

	DBG(2, priv->common, "send a soft prox-out\n");
	wcmSendEvents(pInfo, &out);
}

/*****************************************************************************
 * wcmRotateTablet
 ****************************************************************************/

void wcmRotateTablet(InputInfoPtr pInfo, int value)
{
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common = priv->common;
	WacomToolPtr tool;

	DBG(10, priv, "\n");
	common->wcmRotate = value;

	/* Only try updating properties once we're enabled, no point
	 * otherwise. */
	tool = priv->tool;
	if (tool->enabled)
		wcmUpdateRotationProperty(priv);
}

/* Common pointer refcounting utilities.
 * Common is shared across all wacom devices off the same port. These
 * functions implement basic refcounting to avoid double-frees and memleaks.
 *
 * Usage:
 *  wcmNewCommon() to create a new struct.
 *  wcmRefCommon() to get a new reference to an already exiting one.
 *  wcmFreeCommon() to unref. After the last ref has been unlinked, the
 *  struct is freed.
 *
 */

WacomCommonPtr wcmNewCommon(void)
{
	WacomCommonPtr common;
	common = calloc(1, sizeof(WacomCommonRec));
	if (!common)
		return NULL;;

	common->refcnt = 1;
	common->wcmGestureParameters.wcmScrollDirection = 0;
	common->wcmGestureParameters.wcmTapTime = 250;
	common->wcmRotate = ROTATE_NONE;   /* default tablet rotation to off */
	common->wcmMaxX = 0;               /* max digitizer logical X value */
	common->wcmMaxY = 0;               /* max digitizer logical Y value */
	common->wcmMaxTouchX = 1024;       /* max touch X value */
	common->wcmMaxTouchY = 1024;       /* max touch Y value */
			/* default to Intuos */
	common->wcmSuppress = DEFAULT_SUPPRESS;
			/* transmit position if increment is superior */
	common->wcmRawSample = DEFAULT_SAMPLES;
			/* number of raw data to be used to for filtering */
	return common;
}


void wcmFreeCommon(WacomCommonPtr *ptr)
{
	WacomCommonPtr common = *ptr;

	if (!common)
		return;

	DBG(10, common, "common refcount dec to %d\n", common->refcnt - 1);
	if (--common->refcnt == 0)
	{
		free(common->private);
		while (common->serials)
		{
			WacomToolPtr next;

			DBG(10, common, "Free common serial: %d %s\n",
					common->serials->serial,
					common->serials->name);

			free(common->serials->name);
			next = common->serials->next;
			free(common->serials);
			common->serials = next;
		}
		free(common->device_path);
#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 16
		free(common->touch_mask);
#endif
		free(common);
	}
	*ptr = NULL;
}

WacomCommonPtr wcmRefCommon(WacomCommonPtr common)
{
	if (!common)
		common = wcmNewCommon();
	else
		common->refcnt++;
	DBG(10, common, "common refcount inc to %d\n", common->refcnt);
	return common;
}

/* vim: set noexpandtab tabstop=8 shiftwidth=8: */
