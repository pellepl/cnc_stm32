/*
 * spiffs_wrapper.h
 *
 *  Created on: Jul 3, 2013
 *      Author: petera
 */

#ifndef SPIFFS_WRAPPER_H_
#define SPIFFS_WRAPPER_H_

#include "spiffs.h"

void FS_sys_init();
void FS_mount();
spiffs *FS_get_filesystem();


#endif /* SPIFFS_WRAPPER_H_ */
