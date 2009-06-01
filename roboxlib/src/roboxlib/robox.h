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
 * @param value 1: strobo on, 1: strobo off.
 */
void roboxSetStrobo( int value );

#endif /*ROBOX_H*/
