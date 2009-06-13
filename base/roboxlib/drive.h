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

#ifndef ROBOX_DRIVE_H
#define ROBOX_DRIVE_H

#include "device.h"

/** \brief Predefined RoboX drive constants
  */
#define ROBOX_DRIVE_READ_TIMEOUT               0.01

#define ROBOX_DRIVE_MIN_CURRENT                0
#define ROBOX_DRIVE_ZERO_CURRENT               2048
#define ROBOX_DRIVE_MAX_CURRENT                4095

/** \brief Predefined RoboX security error codes
  */
#define ROBOX_DRIVE_ERROR_NONE                 0
#define ROBOX_DRIVE_ERROR_START                1
#define ROBOX_DRIVE_ERROR_STOP                 2
#define ROBOX_DRIVE_ERROR_BRAKE                3
#define ROBOX_DRIVE_ERROR_RELEASE              4
#define ROBOX_DRIVE_ERROR_SET_CURRENT          5

/** \brief Structure defining the RoboX drive
  */
typedef struct robox_drive_t {
  robox_device_t brake_disengage_dev;   //!< The brake disengage device.
  robox_device_t brake_disengaged_dev;  //!< The brake disengaged device.

  robox_device_t motor_enable_dev;      //!< The motor enable device.
  robox_device_t motor_right_dev;       //!< The right motor device.
  robox_device_t motor_left_dev;        //!< The left motor device.

  double gear_trans;                //!< The drive's gear transmission.
  double wheel_base;                //!< The drive's wheel base in [m].
  double wheel_right_radius;        //!< The drive's right wheel radius in [m].
  double wheel_left_radius;         //!< The drive's left wheel radius in [m].
} robox_drive_t, *robox_drive_p;

/** \brief Predefined RoboX drive error descriptions
  */
extern const char* robox_drive_errors[];

/** \brief Initialize RoboX drive
  * \param[in] drive The RoboX drive to be initialized.
  * \param[in] brake_disengage_dev The name of the brake disengage device.
  * \param[in] brake_disengaged_dev The name of the brake disengaged device.
  * \param[in] motor_enable_dev The name of the motor enable device.
  * \param[in] motor_right_dev The name of the right motor device.
  * \param[in] motor_left_dev The name of the left motor device.
  * \param[in] gear_trans The gear transmission of the drive.
  * \param[in] wheel_base The wheel base of the drive in [m].
  * \param[in] wheel_right_radius The right wheel radius of the drive in [m].
  * \param[in] wheel_left_radius The left wheel radius of the drive in [m].
  * \return The resulting device error code.
  */
int robox_drive_init(
  robox_drive_p drive,
  const char* brake_disengage_dev,
  const char* brake_disengaged_dev,
  const char* motor_enable_dev,
  const char* motor_right_dev,
  const char* motor_left_dev,
  double gear_trans,
  double wheel_base,
  double wheel_right_radius,
  double wheel_left_radius);

/** \brief Destroy RoboX drive
  * \param[in] drive The initialized RoboX drive to be destroyed.
  * \return The resulting device error code.
  */
int robox_drive_destroy(
  robox_drive_p drive);

/** \brief Start the drive
  * \param[in] drive The drive to be started.
  * \return The resulting device error code.
  */
int robox_drive_start(
  robox_drive_p drive);

/** \brief Stop the drive
  * \param[in] drive The drive to be stopped.
  * \return The resulting device error code.
  */
int robox_drive_stop(
  robox_drive_p drive);

/** \brief Fasten the brake
  * \param[in] drive The drive to fasten the break for.
  * \return The resulting device error code.
  */
int robox_drive_brake(
  robox_drive_p drive);

/** \brief Release the brake
  * \param[in] drive The drive to release the break for.
  * \return The resulting device error code.
  */
int robox_drive_release(
  robox_drive_p drive);

/** \brief Set motor current
  * \param[in] drive The drive to set the motor current for.
  * \param[in] right_current The current value to be set for the right motor.
  * \param[in] left_current The current value to be set for the left motor.
  * \return The resulting device error code.
  */
int robox_drive_set_current(
  robox_drive_p drive,
  short right_current,
  short left_current);

#endif
