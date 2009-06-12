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

#ifndef ROBOX_ENCODERS_H
#define ROBOX_ENCODERS_H

#include "device.h"

/** \brief Predefined RoboX encoder constants
  */
#define ROBOX_ENCODERS_READ_TIMEOUT               0.01

/** \brief Structure defining the RoboX encoders
  */
typedef struct robox_encoders_t {
  robox_device_t right_dev;     //!< The right encoder device.
  robox_device_t left_dev;      //!< The left encoder device.
} robox_encoders_t, *robox_encoders_p;

/** \brief Initialize RoboX encoders
  * \param[in] encoders The RoboX encoders to be initialized.
  * \param[in] right_dev The name of the right encoder device.
  * \param[in] left_dev The name of the left encoder device.
  * \return The resulting device error code.
  */
int robox_encoders_init(
  robox_encoders_p encoders,
  const char* right_dev,
  const char* left_dev);

/** \brief Destroy RoboX encoders
  * \param[in] encoders The initialized RoboX encoders to be destroyed.
  * \return The resulting device error code.
  */
int robox_encoders_destroy(
  robox_encoders_p encoders);

#endif
