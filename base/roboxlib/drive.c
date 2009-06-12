/***************************************************************************
 *   Copyright (C) 2009 by Ralf Kaestner                                   *
 *   ralf.kaestner@gmail.com                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include "drive.h"

#include "global.h"

const char* robox_drive_errors[] = {
  "success",
  "error starting drive",
  "error stopping drive",
  "error fastening brake",
  "error releasing brake",
  "error setting motor current",
};

int robox_drive_init(robox_drive_p drive, const char* brake_disengage_dev,
  const char* brake_disengaged_dev, const char* motor_enable_dev,
  const char* motor_right_dev, const char* motor_left_dev) {
  int result;

  if (!(result = robox_device_open(&drive->brake_disengage_dev,
      brake_disengage_dev, robox_device_output, ROBOX_DRIVE_READ_TIMEOUT)) &&
    !(result = robox_device_open(&drive->brake_disengaged_dev,
      brake_disengaged_dev, robox_device_input, ROBOX_DRIVE_READ_TIMEOUT)) &&
    !(result = robox_device_open(&drive->motor_enable_dev, motor_enable_dev, 
      robox_device_output, ROBOX_DRIVE_READ_TIMEOUT)) &&
    !(result = robox_device_open(&drive->motor_right_dev, motor_right_dev, 
      robox_device_output, ROBOX_DRIVE_READ_TIMEOUT)))
    return robox_device_open(&drive->motor_left_dev, motor_left_dev, 
      robox_device_output, ROBOX_DRIVE_READ_TIMEOUT);                                        
  else
    return result;
}

int robox_drive_destroy(robox_drive_p drive) {
  int result;

  if (!(result = robox_device_close(&drive->brake_disengage_dev)) &&
    !(result = robox_device_close(&drive->brake_disengaged_dev)) &&
    !(result = robox_device_close(&drive->motor_enable_dev)) &&
    !(result = robox_device_close(&drive->motor_right_dev)))
    return robox_device_close(&drive->motor_left_dev);
  else
    return result;
}

int robox_drive_start(robox_drive_p drive) {
  if (!robox_drive_set_current(drive, ROBOX_DRIVE_ZERO_CURRENT,
      ROBOX_DRIVE_ZERO_CURRENT) &&
    !robox_drive_release(drive) &&
    !robox_device_write(&drive->motor_enable_dev, 1))
    return ROBOX_DRIVE_ERROR_NONE;
  else
    return ROBOX_DRIVE_ERROR_START;
}

int robox_drive_stop(robox_drive_p drive) {
  if (!robox_device_write(&drive->motor_enable_dev, 0) &&
    !robox_drive_brake(drive) &&
    !robox_drive_set_current(drive, ROBOX_DRIVE_ZERO_CURRENT,
      ROBOX_DRIVE_ZERO_CURRENT))
    return ROBOX_DRIVE_ERROR_NONE;
  else
    return ROBOX_DRIVE_ERROR_STOP;
}

int robox_drive_brake(robox_drive_p drive) {
  if (!robox_device_write(&drive->brake_disengage_dev, 0))
    return ROBOX_DRIVE_ERROR_NONE;
  else
    return ROBOX_DRIVE_ERROR_BRAKE;
}

int robox_drive_release(robox_drive_p drive) {
  if (!robox_device_write(&drive->brake_disengage_dev, 1))
    return ROBOX_DRIVE_ERROR_NONE;
  else
    return ROBOX_DRIVE_ERROR_RELEASE;
}

int robox_drive_set_current(robox_drive_p drive, short right_current,
  short left_current) {
  if (!robox_device_write(&drive->motor_right_dev, clip(right_current, 
    ROBOX_DRIVE_MIN_CURRENT, ROBOX_DRIVE_MAX_CURRENT)) &&
    !robox_device_write(&drive->motor_left_dev, clip(left_current, 
    ROBOX_DRIVE_MIN_CURRENT, ROBOX_DRIVE_MAX_CURRENT)))
    return ROBOX_DRIVE_ERROR_NONE;
  else
    return ROBOX_DRIVE_ERROR_SET_CURRENT;
}
