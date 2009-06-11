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

#include "power.h"

#include "global.h"

int robox_power_init(robox_power_p power, const char* engage_dev, const char* 
  battery_dev) {
  int result;

  if (!(result = robox_device_open(&power->engage_dev, engage_dev, 
      robox_device_output, ROBOX_READ_TIMEOUT)))
    return robox_device_open(&power->battery_dev, battery_dev, 
      robox_device_input, ROBOX_READ_TIMEOUT);
  else
    return result;
}

int robox_power_destroy(robox_power_p power) {
  int result;

  if (!(result = robox_device_close(&power->engage_dev)))
    return robox_device_close(&power->battery_dev);
  else
    return result;
}
