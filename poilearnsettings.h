/**
 ******************************************************************************
 * @addtogroup UAVObjects LibrePilot UAVObjects
 * @{
 * @addtogroup PoiLearnSettings PoiLearnSettings
 * @brief Settings for the @ref Point of Interest feature
 *
 * Autogenerated files and functions for PoiLearnSettings Object
 *
 * @{
 *
 * @file       poilearnsettings.h
 *
 * @author     The LibrePilot Project, https://www.librepilot.org, (C) 2017.
 *             The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010-2013.
 *
 * @brief      Arduino Header of the PoiLearnSettings object. This file has been
 *             automatically generated by the UAVObjectGenerator.
 *
 * @note       Object definition file: poilearnsettings.xml.
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

#ifndef POILEARNSETTINGS_H
#define POILEARNSETTINGS_H
#include <stdbool.h>
/* Object constants */
#define POILEARNSETTINGS_OBJID 0x9E22E820
#define POILEARNSETTINGS_ISSINGLEINST 1
#define POILEARNSETTINGS_ISSETTINGS 1
#define POILEARNSETTINGS_ISPRIORITY 0
#define POILEARNSETTINGS_NUMBYTES sizeof(PoiLearnSettingsData)

/* Field Input information */

// Enumeration options for field Input
typedef enum __attribute__ ((__packed__)) {
    POILEARNSETTINGS_INPUT_ACCESSORY0=0,
    POILEARNSETTINGS_INPUT_ACCESSORY1=1,
    POILEARNSETTINGS_INPUT_ACCESSORY2=2,
    POILEARNSETTINGS_INPUT_ACCESSORY3=3,
    POILEARNSETTINGS_INPUT_ACCESSORY4=4,
    POILEARNSETTINGS_INPUT_ACCESSORY5=5,
    POILEARNSETTINGS_INPUT_NONE=6
} PoiLearnSettingsInputOptions;




/*
 * Packed Object data (unaligned).
 * Should only be used where 4 byte alignment can be guaranteed
 * (eg a single instance on the heap)
 */
typedef struct {
    PoiLearnSettingsInputOptions Input;

} __attribute__((packed)) PoiLearnSettingsDataPacked;

/*
 * Packed Object data.
 * Alignment is forced to 4 bytes
 */
typedef PoiLearnSettingsDataPacked __attribute__((aligned(4))) PoiLearnSettingsData;

/*
 * Union to apply the data array to and to use as structured object data
 */
union {
    PoiLearnSettingsDataPacked data;
    byte arr[POILEARNSETTINGS_NUMBYTES];
 } PoiLearnSettingsDataUnion;

#endif // POILEARNSETTINGS_H

/**
 * @}
 * @}
 */
