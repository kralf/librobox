#ifndef ROBOX_H
#define ROBOX_H

/**
 * Initializes the robox platform.
 * This function should be called before any other function of the API.
 */
void roboxInit();

/**
 * Shuts down the robox platform.
 * Should be called before exiting the calling program.
 */
void roboxShutdown();

/**
 * Returns a bumper value.
 *
 * @param number Bumper number to check, the values start from zero (front) and
 *               grow counterclockwise up to 7 (front-right).
 * @return one if the bumper is active, zero otherwise.
 */
int roboxGetBumper( int number );

/**
 * Gets the brake status.
 *
 * @return 0: break on, 1: break off.
 */
int roboxGetBrake();

/**
 * Sets/unsets the brake.
 *
 * @param value 1: break on, 0: break off.
 */
void roboxSetBrake( int value );

/**
 * Sets/unsets the stroboscopic light.
 *
 * @param value 1: strobo on, 0: strobo off.
 */
void roboxSetStrobo( int value );

/**
 * Get the left encoder value
 *
 * @param value 24 bit value [transformation in han kernel module]
 */
int roboxGetEncoderLeft();

/**
 * Get the right encoder value
 *
 * @param value 24 bit value [transformation in han kernel module]
 */
int roboxGetEncoderRight();

/**
 * Set the left motor speed
 *
 * @param value ??
 */
void roboxSetMotorLeft ( int value );

/**
 * Set the right motor speed
 *
 * @param value ??
 */
void roboxSetMotorRight ( int value );

/**
 * Set the motors enable
 *
 * @param value 1: motors enable, 0: motors disable
 */
void roboxSetMotorEnable ( int value );

/**
 * Watchdog that has to be toggled every 50ms
 *
 * @param value 1:, 0: ?
 */
void roboxSetWatchdog ( int value );

/**
 * Sets/unsets the main power.
 *
 * @param value 1: power on, 0: power off.
 */
void roboxSetPower( int value );

/**
 * activate/inactivte the emergency stop
 *
 * @param value 1: active, 0: inactive
 */
int roboxGetEmergency ();

/**
 * activate/inactivte supervisor
 *
 * @param value 1: active, 0: inactive
 */
int roboxGetSupervisor ();

/**
 * get the values of the odometry
 *
 *
 */

int roboxGetOdometry ( double* x, double* y, double* theta );

#endif /*ROBOX_H*/
