/*
 * Copyright 2009 - 2013 by Ping Cheng, Wacom. <pingc@wacom.com>
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
#include "wcmFilter.h"
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/* wcmCheckSource - Check if there is another source defined this device
 * before or not: don't add the tool by hal/udev if user has defined at least
 * one tool for the device in xorg.conf. One device can have multiple tools
 * with the same type to individualize tools with serial number or areas */
static Bool wcmCheckSource(InputInfoPtr pInfo, dev_t min_maj)
{
	int match = 0;
	InputInfoPtr pDevices = xf86FirstLocalDevice();

	for (; !match && pDevices != NULL; pDevices = pDevices->next)
	{
		char* device = xf86CheckStrOption(pDevices->options, "Device", NULL);

		/* device can be NULL on some distros */
		if (!device)
			continue;

		free(device);

		if (!strstr(pDevices->drv->driverName, "wacom"))
			continue;

		if (pInfo != pDevices)
		{
			WacomCommonPtr pCommon = ((WacomDevicePtr)pDevices->private)->common;
			char* fsource = xf86CheckStrOption(pInfo->options, "_source", "");
			char* psource = xf86CheckStrOption(pDevices->options, "_source", "");

			if (pCommon->min_maj &&
				pCommon->min_maj == min_maj)
			{
				/* only add the new tool if the matching major/minor
				* was from the same source */
				if (strcmp(fsource, psource))
					match = 1;
			}
			free(fsource);
			free(psource);
		}
	}
	if (match)
		xf86Msg(X_WARNING, "%s: device file already in use by %s. "
			"Ignoring.\n", pInfo->name, pDevices->name);
	return match;
}

/* check if the device has been added.
 * Open the device and check it's major/minor, then compare this with every
 * other wacom device listed in the config. If they share the same
 * major/minor and the same source/type, fail.
 * This is to detect duplicate devices if a device was added once through
 * the xorg.conf and is then hotplugged through the server backend (HAL,
 * udev). In this case, the hotplugged one fails.
 */
int wcmIsDuplicate(const char* device, InputInfoPtr pInfo)
{
	struct stat st;
	int isInUse = 0;
	char* lsource = xf86CheckStrOption(pInfo->options, "_source", NULL);

	/* always allow xorg.conf defined tools to be added */
	if (!lsource || !strlen(lsource)) goto ret;

	if (stat(device, &st) == -1)
	{
		/* can not access major/minor to check device duplication */
		xf86Msg(X_ERROR, "%s: stat failed (%s). cannot check for duplicates.\n",
				pInfo->name, strerror(errno));

		/* older systems don't support the required ioctl.  let it pass */
		goto ret;
	}

	if (st.st_rdev)
	{
		/* device matches with another added port */
		if (wcmCheckSource(pInfo, st.st_rdev))
		{
			isInUse = 3;
			goto ret;
		}
	}
	else
	{
		/* major/minor can never be 0, right? */
		xf86Msg(X_ERROR, "%s: device opened with a major/minor of 0. "
			"Something was wrong.\n", pInfo->name);
		isInUse = 4;
	}
ret:
	free(lsource);
	return isInUse;
}

// TODO: remove this
static struct
{
	const char* type;
	__u16 tool[3]; /* tool array is terminated by 0 */
} wcmType [] =
{
	{ "touch",  { BTN_TOUCH, BTN_TOOL_FINGER, 0 } }
};

/* validate tool type for device/product */
Bool wcmIsAValidType(InputInfoPtr pInfo, const char* type)
{
	int j, k, ret = FALSE;
	WacomDevicePtr priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr common = priv->common;
	char* dsource;

	if (!type)
	{
		xf86Msg(X_ERROR, "%s: No type specified\n", pInfo->name);
		return FALSE;
	}

	dsource = xf86CheckStrOption(pInfo->options, "_source", NULL);

	/* walkthrough all types */
	for (j = 0; j < ARRAY_SIZE(wcmType); j++)
	{
		if (!strcmp(wcmType[j].type, type))
		{
			for (k = 0; wcmType[j].tool[k] && !ret; k++)
			{
				if (ISBITSET (common->wcmKeys, wcmType[j].tool[k]))
				{
					ret = TRUE;
				}
				else if (!dsource || !strlen(dsource)) /* an user defined type */
				{
					/* assume it is a valid type */
					SETBIT(common->wcmKeys, wcmType[j].tool[k]);
					ret = TRUE;
				}
			}
		}
	}

	if (!ret)
		xf86Msg(X_ERROR, "%s: Invalid type '%s' for this device.\n",
			pInfo->name, type);

	free(dsource);
	return ret;
}

/* Choose valid types according to device ID. */
int wcmDeviceTypeKeys(InputInfoPtr pInfo)
{
	int ret = 1;
	WacomDevicePtr priv = pInfo->private;
	WacomCommonPtr common = priv->common;

	priv->common->tablet_id = common->wcmDevCls->ProbeKeys(pInfo);

	return ret;
}

#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) < 14
static InputOption*
input_option_new(InputOption *list, char *key, char *value)
{
	InputOption *new;

	new = calloc(1, sizeof(InputOption));
	new->key = strdup(key);
	new->value = strdup(value);
	new->next = list;
	return new;
}

static void
input_option_free_list(InputOption **opts)
{
	InputOption *tmp = *opts;
	while(*opts)
	{
		tmp = (*opts)->next;
		free((*opts)->key);
		free((*opts)->value);
		free((*opts));
		*opts = tmp;
	}
}
#endif

/**
 * Duplicate xf86 options, replace the "type" option with the given type
 * (and the name with "$name $type" and convert them to InputOption
 *
 * @param basename Kernel device name for this device
 * @param type Tool type (cursor, eraser, etc.)
 * @param serial Serial number this device should be bound to (-1 for "any")
 */
static InputOption *wcmOptionDupConvert(InputInfoPtr pInfo, const char* basename, const char *type, int serial)
{
	WacomDevicePtr priv = pInfo->private;
	WacomCommonPtr common = priv->common;
	pointer original = pInfo->options;
	WacomToolPtr ser = common->serials;
	InputOption *iopts = NULL;
	char *name;
	pointer options, o;
	int rc;

#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 12
	options = xf86OptionListDuplicate(original);
#else
	{
		InputInfoRec dummy;

		memset(&dummy, 0, sizeof(dummy));
		xf86CollectInputOptions(&dummy, NULL, original);
		options = dummy.options;
	}
#endif
	if (serial > -1)
	{
		while (ser->serial && ser->serial != serial)
			ser = ser->next;

		if (strlen(ser->name) > 0)
			rc = asprintf(&name, "%s %s %s", basename, ser->name, type);
		else
			rc = asprintf(&name, "%s %d %s", basename, ser->serial, type);
	}
	else
		rc = asprintf(&name, "%s %s", basename, type);

	if (rc == -1) /* if asprintf fails, strdup will probably too... */
		name = strdup("unknown");

	options = xf86ReplaceStrOption(options, "Type", type);
	options = xf86ReplaceStrOption(options, "Name", name);

	if (serial > -1)
		options = xf86ReplaceIntOption(options, "Serial", ser->serial);

	free(name);

	o = options;
	while(o)
	{
		iopts = input_option_new(iopts,
					 xf86OptionName(o),
					 xf86OptionValue(o));
		o = xf86NextOption(o);
	}
	xf86OptionListFree(options);
	return iopts;
}


#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 11
/**
 * Duplicate the attributes of the given device. "product" gets the type
 * appended, so a device of product "Wacom" will then have a product "Wacom
 * eraser", "Wacom cursor", etc.
 */
static InputAttributes* wcmDuplicateAttributes(InputInfoPtr pInfo,
					       const char *type)
{
	int rc;
	InputAttributes *attr;
	char *product;

	attr = DuplicateInputAttributes(pInfo->attrs);
	rc = asprintf(&product, "%s %s", attr->product, type);
	free(attr->product);
	attr->product = (rc != -1) ? product : NULL;
	return attr;
}
#endif

/**
 * This struct contains the necessary info for hotplugging a device later.
 * Memory must be freed after use.
 */
typedef struct {
	InputOption *input_options;
#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 9
	InputAttributes *attrs;
#endif
} WacomHotplugInfo;

/**
 * Actually hotplug the device. This function is called by the server when
 * the WorkProcs are processed.
 *
 * @param client The server client. unused
 * @param closure A pointer to a struct WcmHotplugInfo containing the
 * necessary information to create a new device.
 * @return TRUE to remove this function from the server's work queue.
 */
static Bool
wcmHotplugDevice(ClientPtr client, pointer closure )
{
	WacomHotplugInfo *hotplug_info = closure;
	DeviceIntPtr dev; /* dummy */

#if HAVE_THREADED_INPUT
	input_lock();
#endif

	NewInputDeviceRequest(hotplug_info->input_options,
#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 9
			      hotplug_info->attrs,
#endif
			      &dev);
#if HAVE_THREADED_INPUT
	input_unlock();
#endif

	input_option_free_list(&hotplug_info->input_options);

#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 11
	FreeInputAttributes(hotplug_info->attrs);
#endif
	free(hotplug_info);

	return TRUE;
}

/**
 * Queue the hotplug for one tool/device of the given type.
 * Device has the same options as the "parent" device, type is one of
 * erasor, stylus, pad, touch, cursor, etc.
 * Name of the new device is set automatically to "<device name> <type>".
 *
 * Note that we don't actually hotplug the device here. We store the
 * information needed to hotplug the device later and then queue the
 * hotplug. The server will come back and call the @ref wcmHotplugDevice
 * later.
 *
 * @param pInfo The parent device
 * @param basename The base name for the device (type will be appended)
 * @param type Type name for this tool
 * @param serial Serial number this device should be bound to (-1 for "any")
 */
static void wcmQueueHotplug(InputInfoPtr pInfo, const char* basename, const char *type, int serial)
{
	WacomHotplugInfo *hotplug_info;

	hotplug_info = calloc(1, sizeof(WacomHotplugInfo));

	if (!hotplug_info)
	{
		xf86Msg(X_ERROR, "%s: OOM, cannot hotplug dependent devices\n", pInfo->name);
		return;
	}

	hotplug_info->input_options = wcmOptionDupConvert(pInfo, basename, type, serial);
#if GET_ABI_MAJOR(ABI_XINPUT_VERSION) >= 11
	hotplug_info->attrs = wcmDuplicateAttributes(pInfo, type);
#endif
	QueueWorkProc(wcmHotplugDevice, serverClient, hotplug_info);
}

/**
 * Hotplug all serial numbers configured on this device.
 *
 * @param basename The kernel device name
 */
static void wcmHotplugSerials(InputInfoPtr pInfo, const char *basename)
{
	WacomDevicePtr  priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr  common = priv->common;
	WacomToolPtr    ser = common->serials;

	while (ser)
	{
		xf86Msg(X_INFO, "%s: hotplugging serial %d.\n", pInfo->name, ser->serial);
		ser = ser->next;
	}
}

void wcmHotplugOthers(InputInfoPtr pInfo, const char *basename)
{
	int i, skip = 1;

	xf86Msg(X_INFO, "%s: hotplugging dependent devices.\n", pInfo->name);

        /* same loop is used to init the first device, if we get here we
         * need to start at the second one */
	for (i = 0; i < ARRAY_SIZE(wcmType); i++)
	{
		if (wcmIsAValidType(pInfo, wcmType[i].type))
		{
			if (skip)
				skip = 0;
			else
				wcmQueueHotplug(pInfo, basename, wcmType[i].type, -1);
		}
	}

	wcmHotplugSerials(pInfo, basename);

        xf86Msg(X_INFO, "%s: hotplugging completed.\n", pInfo->name);
}

/**
 * Return 1 if the device needs auto-hotplugging from within the driver.
 * This is the case if we don't get passed a "type" option (invalid in
 * xorg.conf configurations) and we come from HAL, udev or whatever future
 * config backend.
 *
 * This changes the source to _driver/wacom, all auto-hotplugged devices
 * will have the same source.
 */
int wcmNeedAutoHotplug(InputInfoPtr pInfo, char **type)
{
	char *source = xf86CheckStrOption(pInfo->options, "_source", NULL);
	int i;
	int rc = 0;

	if (*type) /* type specified, don't hotplug */
		goto out;

	if (!source) /* xorg.conf device, don't auto-pick type */
		goto out;

	if (source && strcmp(source, "server/hal") && strcmp(source, "server/udev"))
		goto out;

	/* no type specified, so we need to pick the first one applicable
	 * for our device */
	for (i = 0; i < ARRAY_SIZE(wcmType); i++)
	{
		if (wcmIsAValidType(pInfo, wcmType[i].type))
		{
			free(*type);
			*type = strdup(wcmType[i].type);
			break;
		}
	}

	if (!*type)
		goto out;

	xf86Msg(X_INFO, "%s: type not specified, assuming '%s'.\n", pInfo->name, *type);
	xf86Msg(X_INFO, "%s: other types will be automatically added.\n", pInfo->name);

	/* Note: wcmIsHotpluggedDevice() relies on this */
	pInfo->options = xf86AddNewOption(pInfo->options, "Type", *type);
	pInfo->options = xf86ReplaceStrOption(pInfo->options, "_source", "_driver/wacom");

	rc = 1;

	free(source);
out:
	return rc;
}

int wcmParseSerials (InputInfoPtr pInfo)
{
	WacomDevicePtr  priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr  common = priv->common;
	char            *s;

	if (common->serials)
	{
		return 0; /*Parse has been already done*/
	}

	s = xf86SetStrOption(pInfo->options, "ToolSerials", NULL);
	if (s) /*Dont parse again, if the commons have values already*/
	{
		char* tok = strtok(s, ";");
		while (tok != NULL)
		{
			int serial, nmatch;
			char type[strlen(tok) + 1];
			char name[strlen(tok) + 1];
			WacomToolPtr ser = calloc(1, sizeof(WacomTool));

			if (ser == NULL)
				return 1;

			nmatch = sscanf(tok,"%d,%[a-z],%[A-Za-z ]",&serial, type, name);

			if (nmatch < 1)
			{
				xf86Msg(X_ERROR, "%s: %s is invalid serial string.\n",
					pInfo->name, tok);
				free(ser);
				return 1;
			}
			if (nmatch == 3)
			{
				xf86Msg(X_CONFIG, "%s: Tool %d is named %s.\n",
					pInfo->name, serial, name);
				ser->name = strdup(name);
			}
			else ser->name = strdup(""); /*no name yet*/

			if (common->serials == NULL)
				common->serials = ser;
			else
			{
				WacomToolPtr tool = common->serials;
				while (tool->next)
					tool = tool->next;
				tool->next = ser;
			}

			tok = strtok(NULL,";");
		}
	}
	return 0;
}

/**
 * Parse the pre-init options for this device. Most useful for options
 * needed to properly init a device (baud rate for example).
 *
 * Note that parameters is_primary and is_dependent are mutually exclusive,
 * though both may be false in the case of an xorg.conf device.
 *
 * @param is_primary True if the device is the parent device for
 * hotplugging, False if the device is a depent or xorg.conf device.
 * @param is_hotplugged True if the device is a dependent device, FALSE
 * otherwise.
 * @retval True on success or False otherwise.
 */
Bool wcmPreInitParseOptions(InputInfoPtr pInfo, Bool is_primary,
			    Bool is_dependent)
{
	WacomDevicePtr  priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr  common = priv->common;
	char            *s;
	int		i;
	WacomToolPtr    tool = NULL;

	/* Optional configuration */
	s = xf86SetStrOption(pInfo->options, "Mode", NULL);

	if (s && (xf86NameCmp(s, "absolute") == 0))
		set_absolute(pInfo, TRUE);
	else if (s && (xf86NameCmp(s, "relative") == 0))
		set_absolute(pInfo, FALSE);
	else
	{
		if (s)
			xf86Msg(X_ERROR, "%s: invalid Mode (should be absolute"
				" or relative). Using default.\n", pInfo->name);

		/* If Mode not specified or is invalid then rely on
		 * Type specific defaults from initialization.
		 */
	}

	free(s);

	s = xf86SetStrOption(pInfo->options, "Rotate", NULL);

	if (s)
	{
		int rotation = ROTATE_NONE;

		if (xf86NameCmp(s, "CW") == 0)
			rotation = ROTATE_CW;
		else if (xf86NameCmp(s, "CCW") ==0)
			rotation = ROTATE_CCW;
		else if (xf86NameCmp(s, "HALF") ==0)
			rotation = ROTATE_HALF;
		else if (xf86NameCmp(s, "NONE") !=0)
		{
			xf86Msg(X_ERROR, "%s: invalid Rotate option '%s'.\n",
				pInfo->name, s);
			goto error;
		}

		if (is_dependent && rotation != common->wcmRotate)
			xf86Msg(X_INFO, "%s: ignoring rotation of dependent"
					" device\n", pInfo->name);
		else
			wcmRotateTablet(pInfo, rotation);
		free(s);
	}

	common->wcmRawSample = xf86SetIntOption(pInfo->options, "RawSample",
			common->wcmRawSample);
	if (common->wcmRawSample < 1 || common->wcmRawSample > MAX_SAMPLES)
	{
		xf86Msg(X_ERROR, "%s: RawSample setting '%d' out of range [1..%d]. Using default.\n",
			pInfo->name, common->wcmRawSample, MAX_SAMPLES);
		common->wcmRawSample = DEFAULT_SAMPLES;
	}

	common->wcmSuppress = xf86SetIntOption(pInfo->options, "Suppress",
			common->wcmSuppress);
	if (common->wcmSuppress != 0) /* 0 disables suppression */
	{
		if (common->wcmSuppress > MAX_SUPPRESS)
			common->wcmSuppress = MAX_SUPPRESS;
		if (common->wcmSuppress < DEFAULT_SUPPRESS)
			common->wcmSuppress = DEFAULT_SUPPRESS;
	}

	/* pressure curve takes control points x1,y1,x2,y2
	 * values in range from 0..100.
	 * Linear curve is 0,0,100,100
	 * Slightly depressed curve might be 5,0,100,95
	 * Slightly raised curve might be 0,5,95,100
	 */
	s = xf86SetStrOption(pInfo->options, "PressCurve", "0,0,100,100");
	if (s)
	{
		int a,b,c,d;
		if ((sscanf(s,"%d,%d,%d,%d",&a,&b,&c,&d) != 4) ||
				!wcmCheckPressureCurveValues(a, b, c, d))
			xf86Msg(X_CONFIG, "%s: PressCurve not valid\n",
				pInfo->name);
		else
			wcmSetPressureCurve(priv,a,b,c,d);
	}
	free(s);

	if (xf86SetBoolOption(pInfo->options, "Pressure2K", 0)) {
		xf86Msg(X_CONFIG, "%s: Using 2K pressure levels\n", pInfo->name);
		priv->maxCurve = 2048;
	}

	s = xf86SetStrOption(pInfo->options, "TouchKeys", "");
	if (s)
	{
		/* No checking or error reporting for now */
		wcmSetTouchKeys(priv, s);

	}
	free(s);


	/*Serials of tools we want hotpluged*/
	if (wcmParseSerials (pInfo) != 0)
		goto error;

	priv->topX = xf86SetIntOption(pInfo->options, "TopX", 0);
	priv->topY = xf86SetIntOption(pInfo->options, "TopY", 0);
	priv->bottomX = xf86SetIntOption(pInfo->options, "BottomX", 0);
	priv->bottomY = xf86SetIntOption(pInfo->options, "BottomY", 0);
	priv->serial = xf86SetIntOption(pInfo->options, "Serial", 0);

	tool = priv->tool;
	tool->serial = priv->serial;

	/* The first device doesn't need to add any tools/areas as it
	 * will be the first anyway. So if different, add tool
	 * and/or area to the existing lists
	 */
	if(tool != common->wcmTool)
	{
		WacomToolPtr toollist = NULL;
		for(toollist = common->wcmTool; toollist; toollist = toollist->next)
			if(tool->serial == toollist->serial)
				break;

		if(toollist) /* Already have a tool with the same type/serial */
		{
			xf86Msg(X_ERROR, "%s: already have a tool with serial %d.\n",
					pInfo->name, tool->serial);
			goto error;
		} else /* No match on existing tool/serial, add tool to the end of the list */
		{
			toollist = common->wcmTool;
			while(toollist->next)
				toollist = toollist->next;
			toollist->next = tool;
		}
	}

	common->wcmThreshold = xf86SetIntOption(pInfo->options, "Threshold",
			common->wcmThreshold);

	// set touch settings
	common->wcmTouchDefault = 1;
	common->wcmGestureDefault = 1;
	common->wcmMaxContacts = 2;

	common->wcmTouch = xf86SetBoolOption(pInfo->options, "Touch",
					common->wcmTouchDefault);

	common->wcmGesture = xf86SetBoolOption(pInfo->options, "Gesture",
				    common->wcmGestureDefault);

	common->wcmGestureParameters.wcmTapTime =
		xf86SetIntOption(pInfo->options, "TapTime",
		common->wcmGestureParameters.wcmTapTime);


	for (i=0; i<WCM_MAX_BUTTONS; i++)
	{
		char b[12];
		sprintf(b, "Button%d", i+1);
		priv->button_default[i] = xf86SetIntOption(pInfo->options, b, priv->button_default[i]);
	}

	/* Now parse class-specific options */
	if (common->wcmDevCls->ParseOptions &&
	    !common->wcmDevCls->ParseOptions(pInfo))
		goto error;

	return TRUE;
error:
	return FALSE;
}

/* The values were based on trail and error. */
#define WCM_BAMBOO3_MAXX 4096.0
#define WCM_BAMBOO3_ZOOM_DISTANCE 180.0
#define WCM_BAMBOO3_SCROLL_DISTANCE 80.0
#define WCM_BAMBOO3_SCROLL_SPREAD_DISTANCE 350.0

/**
 * Parse post-init options for this device. Useful for overriding HW
 * specific options computed during init phase (HW distances for example).
 *
 * Note that parameters is_primary and is_dependent are mutually exclusive,
 * though both may be false in the case of an xorg.conf device.
 *
 * @param is_primary True if the device is the parent device for
 * hotplugging, False if the device is a depent or xorg.conf device.
 * @param is_hotplugged True if the device is a dependent device, FALSE
 * otherwise.
 * @retval True on success or False otherwise.
 */
Bool wcmPostInitParseOptions(InputInfoPtr pInfo, Bool is_primary,
			     Bool is_dependent)
{
	WacomDevicePtr  priv = (WacomDevicePtr)pInfo->private;
	WacomCommonPtr  common = priv->common;

	int zoom_distance = common->wcmMaxTouchX *
		(WCM_BAMBOO3_ZOOM_DISTANCE / WCM_BAMBOO3_MAXX);
	int scroll_distance = common->wcmMaxTouchX *
		(WCM_BAMBOO3_SCROLL_DISTANCE / WCM_BAMBOO3_MAXX);


	common->wcmMaxZ = xf86SetIntOption(pInfo->options, "MaxZ",
					   common->wcmMaxZ);

	common->wcmGestureParameters.wcmZoomDistance =
		xf86SetIntOption(pInfo->options, "ZoomDistance",
				 zoom_distance);

	common->wcmGestureParameters.wcmScrollDistance =
		xf86SetIntOption(pInfo->options, "ScrollDistance",
				 scroll_distance);

	// TODO: add this as option
	common->wcmGestureParameters.wcmMaxScrollFingerSpread =
		common->wcmMaxTouchX *
		(WCM_BAMBOO3_SCROLL_SPREAD_DISTANCE / WCM_BAMBOO3_MAXX);


	return TRUE;
}

/* vim: set noexpandtab tabstop=8 shiftwidth=8: */
