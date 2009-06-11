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

#include "bumper.h"

#include "global.h"

int robox_bumper_init(robox_bumper_p bumper, const char** segment_devs, 
  ssize_t num_segments) {
  bumper->segment_devs = malloc(num_segments*sizeof(robox_device_t));
  bumper->num_segments = 0;

  int i, result = ROBOX_DEVICE_ERROR_NONE;
  for (i = 0; i < num_segments; ++i) {
    if (!(result = robox_device_open(&bumper->segment_devs[i], segment_devs[i],  
      robox_device_input, ROBOX_READ_TIMEOUT)))
      ++bumper->num_segments;
    else
      break;
  }
  
  return result;
}

int robox_bumper_destroy(robox_bumper_p bumper) {
  int i, result = ROBOX_DEVICE_ERROR_NONE;

  for (i = bumper->num_segments-1; i >= 0; --i) {
    if (!(result = robox_device_close(&bumper->segment_devs[i])))
      --bumper->num_segments;
    else
      break;
  }

  if (!bumper->num_segments)
    free(bumper->segment_devs);

  return result;
}
