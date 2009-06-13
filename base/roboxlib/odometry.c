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

#include <timer.h>

#include "odometry.h"

#include "global.h"
#include "robox.h"

const char* robox_odometry_errors[] = {
  "success",
  "error starting odometry",
  "error integrating odometry",
};

void robox_odometry_init(robox_odometry_p odometry, robox_encoders_p encoders,
  robox_drive_p drive) {
  odometry->encoders = encoders;
  odometry->drive = drive;
}

void robox_odometry_destroy(robox_odometry_p odometry) {
  odometry->encoders = 0;
  odometry->drive = 0;
}

int robox_odometry_start(robox_odometry_p odometry) {
  if (!robox_encoders_get_values(odometry->encoders, &odometry->right_value,
    &odometry->left_value)) {
    timer_start(&odometry->timestamp);
    return ROBOX_ODOMETRY_ERROR_NONE;
  }
  else
    return ROBOX_ODOMETRY_ERROR_START;
}

double robox_odometry_mod_2pi(double theta) {
    int n = floor(theta/(2.0*M_PI));
    theta -= n*(2.0*M_PI);
    
    if (theta > M_PI)
      theta -= (2.0*M_PI);

    return theta;
}

int robox_odometry_integrate(robox_odometry_p odometry, robox_pose_p pose,
  robox_velocity_p velocity) {
  int right_value, left_value;

  if (!robox_encoders_get_values(odometry->encoders, &right_value, 
    &left_value)) {
    double dalpha_right = robox_encoders_to_angle(odometry->encoders,
      odometry->right_value, right_value, odometry->drive->gear_trans);
    double dalpha_left = -robox_encoders_to_angle(odometry->encoders,
      odometry->left_value, left_value, odometry->drive->gear_trans);

    double drho_right = odometry->drive->wheel_right_radius*dalpha_right;
    double drho_left = odometry->drive->wheel_left_radius*dalpha_left;
    double drho = 0.5*(drho_right+drho_left);
    double dtheta = (drho_right-drho_left)/odometry->drive->wheel_base;
    double dt = timer_stop(odometry->timestamp);

    pose->theta = robox_odometry_mod_2pi(pose->theta+dtheta);
    pose->x += drho*cos(pose->theta);
    pose->y += drho*sin(pose->theta);
    velocity->translational = drho/dt;
    velocity->rotational = dtheta/dt;
  
    odometry->right_value = right_value;
    odometry->left_value = left_value;
    timer_start(&odometry->timestamp);

    return ROBOX_ODOMETRY_ERROR_NONE;
  }
  else
    return ROBOX_ODOMETRY_ERROR_INTEGRATE;
}
