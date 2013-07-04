/*
 * spiffs_wrapper.h
 *
 *  Created on: Jul 3, 2013
 *      Author: petera
 */

#ifndef SPIFFS_WRAPPER_H_
#define SPIFFS_WRAPPER_H_

#include "spiffs.h"

void SPIFFS_sys_init();
void SPIFFS_mount();
spiffs *SPIFFS_get_filesystem();


#endif /* SPIFFS_WRAPPER_H_ */
