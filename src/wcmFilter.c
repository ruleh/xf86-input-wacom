/*
 * Copyright 1995-2002 by Frederic Lepied, France. <Lepied@XFree86.org>
 * Copyright 2002-2008 by Ping Cheng, Wacom. <pingc@wacom.com> 
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

#include <math.h>
#include "xf86Wacom.h"
#include "wcmFilter.h"

/*****************************************************************************
 * Static functions
 ****************************************************************************/

static void filterCurveToLine(int* pCurve, int nMax, double x0, double y0,
		double x1, double y1, double x2, double y2,
		double x3, double y3);
static int filterOnLine(double x0, double y0, double x1, double y1,
		double a, double b);
static void filterLine(int* pCurve, int nMax, int x0, int y0, int x1, int y1);


/*****************************************************************************
 * wcmCheckPressureCurveValues -- check pressure curve values for sanity.
 * Return TRUE if values are sane or FALSE otherwise.
 ****************************************************************************/
int wcmCheckPressureCurveValues(int x0, int y0, int x1, int y1)
{
	return !((x0 < 0) || (x0 > 100) || (y0 < 0) || (y0 > 100) ||
		 (x1 < 0) || (x1 > 100) || (y1 < 0) || (y1 > 100));
}


/*****************************************************************************
 * wcmSetPressureCurve -- apply user-defined curve to pressure values
 ****************************************************************************/
void wcmSetPressureCurve(WacomDevicePtr pDev, int x0, int y0,
	int x1, int y1)
{
	/* sanity check values */
	if (!wcmCheckPressureCurveValues(x0, y0, x1, y1))
		return;

	/* A NULL pPressCurve indicates the (default) linear curve */
	if (x0 == 0 && y0 == 0 && x1 == 100 && y1 == 100) {
		free(pDev->pPressCurve);
		pDev->pPressCurve = NULL;
	}
	else if (!pDev->pPressCurve) {
		pDev->pPressCurve = calloc(FILTER_PRESSURE_RES+1, sizeof(*pDev->pPressCurve));

		if (!pDev->pPressCurve) {
			LogMessageVerbSigSafe(X_WARNING, 0,
			                      "Unable to allocate memory for pressure curve; using default.\n");
			x0 = 0;
			y0 = 0;
			x1 = 100;
			y1 = 100;
		}
	}

	if (pDev->pPressCurve)
		filterCurveToLine(pDev->pPressCurve,
				pDev->maxCurve,
				0.0, 0.0,               /* bottom left  */
				x0/100.0, y0/100.0,     /* control point 1 */
				x1/100.0, y1/100.0,     /* control point 2 */
				1.0, 1.0);              /* top right */

	pDev->nPressCtrl[0] = x0;
	pDev->nPressCtrl[1] = y0;
	pDev->nPressCtrl[2] = x1;
	pDev->nPressCtrl[3] = y1;
}

/*
 * wcmResetSampleCounter --
 * Device specific filter routines are responcable for storing raw data
 * as well as filtering.  wcmResetSampleCounter is called to reset
 * raw counters.
 */
void wcmResetSampleCounter(const WacomChannelPtr pChannel)
{
	pChannel->nSamples = 0;
	pChannel->rawFilter.npoints = 0;
}


static void filterNearestPoint(double x0, double y0, double x1, double y1,
		double a, double b, double* x, double* y)
{
	double vx, vy, wx, wy, d1, d2, c;

	wx = a - x0; wy = b - y0;
	vx = x1 - x0; vy = y1 - y0;

	d1 = vx * wx + vy * wy;
	if (d1 <= 0)
	{
		*x = x0;
		*y = y0;
	}
	else
	{
		d2 = vx * vx + vy * vy;
		if (d1 >= d2)
		{
			*x = x1;
			*y = y1;
		}
		else
		{
			c = d1 / d2;
			*x = x0 + c * vx;
			*y = y0 + c * vy;
		}
	}
}

static int filterOnLine(double x0, double y0, double x1, double y1,
		double a, double b)
{
        double x, y, d;
	filterNearestPoint(x0,y0,x1,y1,a,b,&x,&y);
	d = (x-a)*(x-a) + (y-b)*(y-b);
	return d < 0.00001; /* within 100th of a point (1E-2 squared) */
}

static void filterCurveToLine(int* pCurve, int nMax, double x0, double y0,
		double x1, double y1, double x2, double y2,
		double x3, double y3)
{
	double x01,y01,x32,y32,xm,ym;
	double c1,d1,c2,d2,e,f;

	/* check if control points are on line */
	if (filterOnLine(x0,y0,x3,y3,x1,y1) && filterOnLine(x0,y0,x3,y3,x2,y2))
	{
		filterLine(pCurve,nMax,
			(int)(x0*nMax),(int)(y0*nMax),
			(int)(x3*nMax),(int)(y3*nMax));
		return;
	}

	/* calculate midpoints */
	x01 = (x0 + x1) / 2; y01 = (y0 + y1) / 2;
	x32 = (x3 + x2) / 2; y32 = (y3 + y2) / 2;

	/* calc split point */
	xm = (x1 + x2) / 2; ym = (y1 + y2) / 2;
	
	/* calc control points and midpoint */
	c1 = (x01 + xm) / 2; d1 = (y01 + ym) / 2;
	c2 = (x32 + xm) / 2; d2 = (y32 + ym) / 2;
	e = (c1 + c2) / 2; f = (d1 + d2) / 2;

	/* do each side */
	filterCurveToLine(pCurve,nMax,x0,y0,x01,y01,c1,d1,e,f);
	filterCurveToLine(pCurve,nMax,e,f,c2,d2,x32,y32,x3,y3);
}

static void filterLine(int* pCurve, int nMax, int x0, int y0, int x1, int y1)
{
	int dx, dy, ax, ay, sx, sy, x, y, d;

	/* sanity check */
	if ((x0 < 0) || (y0 < 0) || (x1 < 0) || (y1 < 0) ||
		(x0 > nMax) || (y0 > nMax) || (x1 > nMax) || (y1 > nMax))
		return;

	dx = x1 - x0; ax = abs(dx) * 2; sx = (dx>0) ? 1 : -1;
	dy = y1 - y0; ay = abs(dy) * 2; sy = (dy>0) ? 1 : -1;
	x = x0; y = y0;

	/* x dominant */
	if (ax > ay)
	{
		d = ay - ax / 2;
		while (1)
		{
			pCurve[x] = y;
			if (x == x1) break;
			if (d >= 0)
			{
				y += sy;
				d -= ax;
			}
			x += sx;
			d += ay;
		}
	}

	/* y dominant */
	else
	{
		d = ax - ay / 2;
		while (1)
		{
			pCurve[x] = y;
			if (y == y1) break;
			if (d >= 0)
			{
				x += sx;
				d -= ay;
			}
			y += sy;
			d += ax;
		}
	}
}
static void storeRawSample(WacomCommonPtr common, WacomChannelPtr pChannel,
			   WacomDeviceStatePtr ds)
{
	WacomFilterState *fs;
	int i;

	fs = &pChannel->rawFilter;
	if (!fs->npoints)
	{
		DBG(10, common, "initialize channel data.\n");
		/* Store initial value over whole average window */
		for (i=common->wcmRawSample - 1; i>=0; i--)
		{
			fs->x[i]= ds->x;
			fs->y[i]= ds->y;
		}
		++fs->npoints;
	} else {
		/* Shift window and insert latest sample */
		for (i=common->wcmRawSample - 1; i>0; i--)
		{
			fs->x[i]= fs->x[i-1];
			fs->y[i]= fs->y[i-1];
		}
		fs->x[0] = ds->x;
		fs->y[0] = ds->y;
		if (fs->npoints < common->wcmRawSample)
			++fs->npoints;
	}
}

static int wcmFilterAverage(int *samples, int n)
{
	int x = 0;
	int i;

	for (i = 0; i < n; i++)
	{
		x += samples[i];
	}
	return x / n;
}

/*****************************************************************************
 * wcmFilterCoord -- provide noise correction to all transducers
 ****************************************************************************/

int wcmFilterCoord(WacomCommonPtr common, WacomChannelPtr pChannel,
	WacomDeviceStatePtr ds)
{
	WacomFilterState *state;

	DBG(10, common, "common->wcmRawSample = %d \n", common->wcmRawSample);

	storeRawSample(common, pChannel, ds);

	state = &pChannel->rawFilter;

	ds->x = wcmFilterAverage(state->x, common->wcmRawSample);
	ds->y = wcmFilterAverage(state->y, common->wcmRawSample);

	return 0; /* lookin' good */
}

/* vim: set noexpandtab tabstop=8 shiftwidth=8: */
