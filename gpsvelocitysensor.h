/**
 ******************************************************************************
 * @addtogroup UAVObjects LibrePilot UAVObjects
 * @{
 * @addtogroup GPSVelocitySensor GPSVelocitySensor
 * @brief Raw GPS velocity in NED frame and m/s from @ref GPSModule.
 *
 * Autogenerated files and functions for GPSVelocitySensor Object
 *
 * @{
 *
 * @file       gpsvelocitysensor.h
 *
 * @author     The LibrePilot Project, https://www.librepilot.org, (C) 2017.
 *             The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010-2013.
 *
 * @brief      Arduino Header of the GPSVelocitySensor object. This file has been
 *             automatically generated by the UAVObjectGenerator.
 *
 * @note       Object definition file: gpsvelocitysensor.xml.
 *             This is an automatically generated file.
 *             DO NOT modify manually.
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef GPSVELOCITYSENSOR_H
#define GPSVELOCITYSENSOR_H
#include <stdbool.h>
/* Object constants */
#define GPSVELOCITYSENSOR_OBJID 0xBC57454
#define GPSVELOCITYSENSOR_ISSINGLEINST 1
#define GPSVELOCITYSENSOR_ISSETTINGS 0
#define GPSVELOCITYSENSOR_ISPRIORITY 0
#define GPSVELOCITYSENSOR_NUMBYTES sizeof(GPSVelocitySensorData)

/* Field North information */

/* Field East information */

/* Field Down information */




/*
 * Packed Object data (unaligned).
 * Should only be used where 4 byte alignment can be guaranteed
 * (eg a single instance on the heap)
 */
typedef struct {
    float North;
    float East;
    float Down;

} __attribute__((packed)) GPSVelocitySensorDataPacked;

/*
 * Packed Object data.
 * Alignment is forced to 4 bytes
 */
typedef GPSVelocitySensorDataPacked __attribute__((aligned(4))) GPSVelocitySensorData;

/*
 * Union to apply the data array to and to use as structured object data
 */
union {
    GPSVelocitySensorDataPacked data;
    byte arr[GPSVELOCITYSENSOR_NUMBYTES];
 } GPSVelocitySensorDataUnion;

#endif // GPSVELOCITYSENSOR_H

/**
 * @}
 * @}
 */