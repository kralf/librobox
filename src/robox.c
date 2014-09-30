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

#include <stdio.h>
#include <string.h>

#include "robox.h"

#include "global.h"

const char* robox_errors[] = {
  "success",
  "error initializing robot",
  "error starting robot",
  "error resetting robot",
};

void* robox_run(void* arg);
void robox_cleanup(void* arg);

config_param_t robox_default_params[] = {
  {ROBOX_PARAMETER_MODEL,
    config_param_type_enum,
    "robox",
    "robox|biba",
    "RoboX model, where 'robox' refers to a standard RoboX, and 'biba' "
    "indicates the RoboX-based Biba robot"},
  {ROBOX_PARAMETER_SECURITY_ESTOP_DEV,
    config_param_type_string,
    "/dev/robox/security/stop/emergency"
    "",
    "Path to the special file of the emergency stop device"},
  {ROBOX_PARAMETER_SECURITY_SSTOP_DEV,
    config_param_type_string,
    "/dev/robox/security/stop/supervisor",
    "",
    "Path to the special file of the supervisor stop device"},
  {ROBOX_PARAMETER_SECURITY_WATCHDOG_DEV,
    config_param_type_string,
    "/dev/robox/security/watchdog",
    "",
    "Path to the special file of the watchdog device"},
  {ROBOX_PARAMETER_SECURITY_FLASHLIGHT_DEV,
    config_param_type_string,
    "/dev/robox/security/flashlight",
    "",
    "Path to the special file of the flashlight device"},
  {ROBOX_PARAMETER_POWER_ENGAGE_DEV,
    config_param_type_string,
    "/dev/robox/power/engage",
    "",
    "Path to the special file of the power engage device"},
  {ROBOX_PARAMETER_POWER_BATTERY_DEV,
    config_param_type_string,
    "/dev/robox/power/battery",
    "",
    "Path to the special file of the battery device"},
  {ROBOX_PARAMETER_SENSORS_CHECK_DEV,
    config_param_type_string,
    "/dev/robox/sensors/check",
    "",
    "Path to the special file of the sensor check device"},
  {ROBOX_PARAMETER_SENSORS_OK_DEV,
    config_param_type_string,
    "/dev/robox/sensors/ok",
    "",
    "Path to the special file of the sensors okay device"},
  {ROBOX_PARAMETER_ENCODER_RIGHT_DEV,
    config_param_type_string,
    "/dev/robox/sensors/encoders/right"
    "",
    "Path to the special file of the right encoder device"},
  {ROBOX_PARAMETER_ENCODER_LEFT_DEV,
    config_param_type_string,
    "/dev/robox/sensors/encoders/left",
    "",
    "Path to the special file of the left encoder device"},
  {ROBOX_PARAMETER_BUMPER_DEV_DIR,
    config_param_type_string,
    "/dev/robox/sensors/bumper",
    "",
    "Path to the special file of the bumper device"},
  {ROBOX_PARAMETER_MOTOR_ENABLE_DEV,
    config_param_type_string,
    "/dev/robox/drive/motor/enable",
    "",
    "Path to the special file of the motor enable device"},
  {ROBOX_PARAMETER_MOTOR_RIGHT_DEV,
    config_param_type_string,
    "/dev/robox/drive/motor/right",
    "",
    "Path to the special file of the right motor device"},
  {ROBOX_PARAMETER_MOTOR_LEFT_DEV,
    config_param_type_string,
    "/dev/robox/drive/motor/left",
    "",
    "Path to the special file of the left motor device"},
  {ROBOX_PARAMETER_MOTOR_BRAKE_DEV,
    config_param_type_string,
    "/dev/robox/drive/brake/disengage",
    "",
    "Path to the special file of the brake disengage device"},
  {ROBOX_PARAMETER_ENCODER_PULSES,
    config_param_type_int,
    "500",
    "[16, 2500000]",
    "Number of encoder pulses per revolution in [ticks]"},
  {ROBOX_PARAMETER_GEAR_TRANSMISSION,
    config_param_type_float,
    "50.0",
    "(-inf, inf)",
    "Transmission ratio of the drive gears"},
  {ROBOX_PARAMETER_WHEEL_BASE,
    config_param_type_float,
    "0.545",
    "[0.0, inf)",
    "Wheel base distance in [m]"},
  {ROBOX_PARAMETER_WHEEL_RIGHT_RADIUS,
    config_param_type_float,
    "0.09",
    "[0.0, inf)",
    "Radius of right wheel in [m]"},
  {ROBOX_PARAMETER_WHEEL_LEFT_RADIUS,
    config_param_type_float,
    "0.09",
    "[0.0, inf)",
    "Radius of left wheel in [m]"},
  {ROBOX_PARAMETER_CONTROL_P_GAIN,
    config_param_type_float,
    "1e-1",
    "(-inf, inf)",
    "Proportional gain of velocity controller"},
  {ROBOX_PARAMETER_CONTROL_I_GAIN,
    config_param_type_float,
    "1e-4",
    "(-inf, inf)",
    "Integral gain of velocity controller"},
  {ROBOX_PARAMETER_CONTROL_D_GAIN,
    config_param_type_float,
    "-4e-2",
    "(-inf, inf)",
    "Differential gain of velocity controller"},
};
 
const config_default_t robox_default_config = {
  robox_default_params,
  sizeof(robox_default_params)/sizeof(config_param_t),
};

int robox_init(robox_robot_p robot, const config_t* config) {
  thread_mutex_init(&robot->mutex);
  config_init_default(&robot->config, &robox_default_config);
  if (config)
    config_set(&robot->config, config);

  robot->model = config_get_int(&robot->config, ROBOX_PARAMETER_MODEL);

  robot->security_error = ROBOX_SECURITY_ERROR_NONE;
  memset(&robot->pose, 0, sizeof(robox_drive_pose_t));
  memset(&robot->velocity, 0, sizeof(robox_drive_vel_t));

  memset(&robot->set_vel, 0, sizeof(robox_drive_vel_t));

  if (!robox_security_init(&robot->security, 
      config_get_string(&robot->config, ROBOX_PARAMETER_SECURITY_ESTOP_DEV),
      config_get_string(&robot->config, ROBOX_PARAMETER_SECURITY_SSTOP_DEV),
      config_get_string(&robot->config, ROBOX_PARAMETER_SECURITY_WATCHDOG_DEV),
      config_get_string(&robot->config, 
        ROBOX_PARAMETER_SECURITY_FLASHLIGHT_DEV)) && 
    !robox_power_init(&robot->power, 
      config_get_string(&robot->config, ROBOX_PARAMETER_POWER_ENGAGE_DEV),
      config_get_string(&robot->config, ROBOX_PARAMETER_POWER_BATTERY_DEV)) &&
    !robox_sensors_init(&robot->sensors, 
      config_get_string(&robot->config, ROBOX_PARAMETER_SENSORS_CHECK_DEV),
      config_get_string(&robot->config, ROBOX_PARAMETER_SENSORS_OK_DEV)) &&
    !robox_encoders_init(&robot->encoders, 
      config_get_string(&robot->config, ROBOX_PARAMETER_ENCODER_LEFT_DEV),
      config_get_string(&robot->config, ROBOX_PARAMETER_ENCODER_RIGHT_DEV),
      config_get_int(&robot->config, ROBOX_PARAMETER_ENCODER_PULSES)) &&
    !robox_bumper_init(&robot->bumper, 
      config_get_string(&robot->config, ROBOX_PARAMETER_BUMPER_DEV_DIR)) &&
    !robox_motors_init(&robot->motors, 
      config_get_string(&robot->config, ROBOX_PARAMETER_MOTOR_ENABLE_DEV),
      config_get_string(&robot->config, ROBOX_PARAMETER_MOTOR_RIGHT_DEV),
      config_get_string(&robot->config, ROBOX_PARAMETER_MOTOR_LEFT_DEV),
      config_get_string(&robot->config, ROBOX_PARAMETER_MOTOR_BRAKE_DEV))) {
    robox_drive_init(&robot->drive, 
      config_get_float(&robot->config, ROBOX_PARAMETER_GEAR_TRANSMISSION),
      config_get_float(&robot->config, ROBOX_PARAMETER_WHEEL_BASE),
      config_get_float(&robot->config, ROBOX_PARAMETER_WHEEL_RIGHT_RADIUS),
      config_get_float(&robot->config, ROBOX_PARAMETER_WHEEL_LEFT_RADIUS));
    robox_odometry_init(&robot->odometry, &robot->encoders, &robot->drive);
    robox_control_init(&robot->control, 
      &robot->encoders, &robot->motors, &robot->drive, 
      config_get_float(&robot->config, ROBOX_PARAMETER_CONTROL_P_GAIN),
      config_get_float(&robot->config, ROBOX_PARAMETER_CONTROL_I_GAIN),
      config_get_float(&robot->config, ROBOX_PARAMETER_CONTROL_D_GAIN));

    return ROBOX_ERROR_NONE;
  }
  else
    return ROBOX_ERROR_INIT;
}

void robox_destroy(robox_robot_p robot) {
  robox_control_destroy(&robot->control);
  robox_odometry_destroy(&robot->odometry);

  robox_motors_destroy(&robot->motors);
  robox_bumper_destroy(&robot->bumper);
  robox_encoders_destroy(&robot->encoders);
  robox_sensors_destroy(&robot->sensors);
  robox_power_destroy(&robot->power);
  robox_security_destroy(&robot->security);

  config_destroy(&robot->config);
  thread_mutex_destroy(&robot->mutex);
}

int robox_start(robox_robot_p robot, double frequency) {
  if (!robox_power_up(&robot->power) &&
    !robox_security_start(&robot->security) &&
    !robox_motors_start(&robot->motors) &&
    !robox_odometry_start(&robot->odometry) &&
    !robox_control_start(&robot->control) &&
    !thread_start(&robot->thread, robox_run, robox_cleanup, robot, frequency))
    return ROBOX_ERROR_NONE;
  else
    return ROBOX_ERROR_START;
}

int robox_stop(robox_robot_p robot) {
  thread_exit(&robot->thread, 1);
  return ROBOX_ERROR_NONE;
}

int robox_reset(robox_robot_p robot) {
  int result = ROBOX_ERROR_RESET;

  thread_mutex_lock(&robot->mutex);

  if (!robox_control_start(&robot->control) &&
    !robox_odometry_start(&robot->odometry)) {
    memset(&robot->pose, 0, sizeof(robox_drive_pose_t));
    memset(&robot->velocity, 0, sizeof(robox_drive_vel_t));

    memset(&robot->set_vel, 0, sizeof(robox_drive_vel_t));

    result = ROBOX_ERROR_NONE;
  }

  thread_mutex_unlock(&robot->mutex);

  return result;
}

void robox_get_state(robox_robot_p robot, robox_drive_pose_p pose, 
  robox_drive_vel_p velocity) {
  thread_mutex_lock(&robot->mutex);

  *pose = robot->pose;
  *velocity = robot->velocity;

  thread_mutex_unlock(&robot->mutex);
}

void robox_set_velocity(robox_robot_p robot, robox_drive_vel_p velocity) {
  thread_mutex_lock(&robot->mutex);

  robot->set_vel = *velocity;

  thread_mutex_unlock(&robot->mutex);
}

void* robox_run(void* arg) {
  int security_error;
  robox_robot_p robot = arg;

  thread_mutex_lock(&robot->mutex);

  if (!(security_error = robox_security_check(&robot->security, 
    &robot->bumper))) {
    if (robot->security_error) {
      robox_motors_start(&robot->motors);
      robox_control_start(&robot->control);
    }
    else
      robox_control_iterate(&robot->control, &robot->set_vel);
  }
  else
    robox_motors_stop(&robot->motors);

  robot->security_error = security_error;
  robox_odometry_integrate(&robot->odometry, &robot->pose, 
    &robot->velocity);

  thread_mutex_unlock(&robot->mutex);

  return 0;
}

void robox_cleanup(void* arg) {
  robox_robot_p robot = arg;

  robox_motors_stop(&robot->motors);
  robox_security_stop(&robot->security);
  robox_power_down(&robot->power);
}
