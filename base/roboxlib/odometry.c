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

#include <stdlib.h>

#include "bumpers.h"

#include "global.h"

int robox_bumpers_init(robox_bumpers_p bumpers, const char** dev_names,
  ssize_t num_bumpers) {
  bumpers->devs = malloc(num_bumpers*sizeof(robox_device_t));
  bumpers->num = 0;

  int i, result = ROBOX_DEVICE_ERROR_NONE;
  for (i = 0; i < num_bumpers; ++i) {
    if (!(result = robox_device_open(&bumpers->devs[i], dev_names[i], 
      ROBOX_READ_TIMEOUT)))
      ++bumpers->num;
    else
      break;
  }
  
  return result;
}

int robox_bumpers_destroy(robox_bumpers_p bumpers) {
  int i, result = ROBOX_DEVICE_ERROR_NONE;

  for (i = bumpers->num-1; i >= 0; --i) {
    if (!(result = robox_device_close(&bumpers->devs[i])))
      --bumpers->num;
    else
      break;
  }

  if (!bumpers->num)
    free(bumpers->devs);

  return result;
}
