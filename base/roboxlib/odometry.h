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

#ifndef ROBOX_BUMPERS_H
#define ROBOX_BUMPERS_H

#include "device.h"

/** \brief Bumper state enumeratable type
  */
typedef enum {
  robox_bumper_released = 0,    //!< Bumper released.
  robox_bumper_pressed = 1,     //!< Bumper pressed.
} robox_bumper_state_t;

/** \brief Structure defining the RoboX bumpers
  */
typedef struct robox_bumpers_t {
  robox_device_p devs;    //!< The bumper devices.
  ssize_t num;            //!< The number of bumpers.
} robox_bumpers_t, *robox_bumpers_p;

/** \brief Initialize RoboX bumpers
  * \param[in] bumpers The RoboX bumpers to be initialized.
  * \param[in] dev_names The names of the RoboX bumper devices.
  * \param[in] num_bumpers The number of bumpers to be initialized.
  * \return The resulting device error code.
  */
int robox_bumpers_init(
  robox_bumpers_p bumpers,
  const char** dev_names,
  ssize_t num_bumpers);

/** \brief Destroy RoboX bumpers
  * \param[in] bumpers The initialized RoboX bumpers to be destroyed.
  * \return The resulting device error code.
  */
int robox_bumpers_destroy(
  robox_bumpers_p bumpers);

#endif
