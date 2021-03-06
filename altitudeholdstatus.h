/**
 ******************************************************************************
 * @addtogroup UAVObjects LibrePilot UAVObjects
 * @{
 * @addtogroup AltitudeHoldStatus AltitudeHoldStatus
 * @brief Status Data from Altitude Hold Control Loops
 *
 * Autogenerated files and functions for AltitudeHoldStatus Object
 *
 * @{
 *
 * @file       altitudeholdstatus.h
 *
 * @author     The LibrePilot Project, https://www.librepilot.org, (C) 2017.
 *             The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010-2013.
 *
 * @brief      Arduino Header of the AltitudeHoldStatus object. This file has been
 *             automatically generated by the UAVObjectGenerator.
 *
 * @note       Object definition file: altitudeholdstatus.xml.
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

#ifndef ALTITUDEHOLDSTATUS_H
#define ALTITUDEHOLDSTATUS_H
#include <stdbool.h>
/* Object constants */
#define ALTITUDEHOLDSTATUS_OBJID 0x8D06190A
#define ALTITUDEHOLDSTATUS_ISSINGLEINST 1
#define ALTITUDEHOLDSTATUS_ISSETTINGS 0
#define ALTITUDEHOLDSTATUS_ISPRIORITY 0
#define ALTITUDEHOLDSTATUS_NUMBYTES sizeof(AltitudeHoldStatusData)

/* Field VelocityDesired information */

/* Field ThrustDemand information */

/* Field State information */

// Enumeration options for field State
typedef enum __attribute__ ((__packed__)) {
    ALTITUDEHOLDSTATUS_STATE_DIRECT=0,
    ALTITUDEHOLDSTATUS_STATE_ALTITUDEVARIO=1,
    ALTITUDEHOLDSTATUS_STATE_ALTITUDEHOLD=2
} AltitudeHoldStatusStateOptions;




/*
 * Packed Object data (unaligned).
 * Should only be used where 4 byte alignment can be guaranteed
 * (eg a single instance on the heap)
 */
typedef struct {
    float VelocityDesired;
    float ThrustDemand;
    AltitudeHoldStatusStateOptions State;

} __attribute__((packed)) AltitudeHoldStatusDataPacked;

/*
 * Packed Object data.
 * Alignment is forced to 4 bytes
 */
typedef AltitudeHoldStatusDataPacked __attribute__((aligned(4))) AltitudeHoldStatusData;

/*
 * Union to apply the data array to and to use as structured object data
 */
union {
    AltitudeHoldStatusDataPacked data;
    byte arr[ALTITUDEHOLDSTATUS_NUMBYTES];
 } AltitudeHoldStatusDataUnion;

#endif // ALTITUDEHOLDSTATUS_H

/**
 * @}
 * @}
 */
