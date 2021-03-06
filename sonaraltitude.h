/**
 ******************************************************************************
 * @addtogroup UAVObjects LibrePilot UAVObjects
 * @{
 * @addtogroup SonarAltitude SonarAltitude
 * @brief The raw data from the ultrasound sonar sensor with altitude estimate.
 *
 * Autogenerated files and functions for SonarAltitude Object
 *
 * @{
 *
 * @file       sonaraltitude.h
 *
 * @author     The LibrePilot Project, https://www.librepilot.org, (C) 2017.
 *             The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010-2013.
 *
 * @brief      Arduino Header of the SonarAltitude object. This file has been
 *             automatically generated by the UAVObjectGenerator.
 *
 * @note       Object definition file: sonaraltitude.xml.
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

#ifndef SONARALTITUDE_H
#define SONARALTITUDE_H
#include <stdbool.h>
/* Object constants */
#define SONARALTITUDE_OBJID 0x6C5A0CBC
#define SONARALTITUDE_ISSINGLEINST 1
#define SONARALTITUDE_ISSETTINGS 0
#define SONARALTITUDE_ISPRIORITY 0
#define SONARALTITUDE_NUMBYTES sizeof(SonarAltitudeData)

/* Field Altitude information */




/*
 * Packed Object data (unaligned).
 * Should only be used where 4 byte alignment can be guaranteed
 * (eg a single instance on the heap)
 */
typedef struct {
    float Altitude;

} __attribute__((packed)) SonarAltitudeDataPacked;

/*
 * Packed Object data.
 * Alignment is forced to 4 bytes
 */
typedef SonarAltitudeDataPacked __attribute__((aligned(4))) SonarAltitudeData;

/*
 * Union to apply the data array to and to use as structured object data
 */
union {
    SonarAltitudeDataPacked data;
    byte arr[SONARALTITUDE_NUMBYTES];
 } SonarAltitudeDataUnion;

#endif // SONARALTITUDE_H

/**
 * @}
 * @}
 */
