/*
 * parser.h
 *
 *  Created on: 29 Nov 2017
 *      Author: beni
 */

#ifndef INC_PARSER_H_
#define INC_PARSER_H_

#include "lpc_types.h"
#include "hid.h"
#include <stdio.h>
#include <string.h>

uint8_t parseBuffer(uint8_t *buf, uint8_t len);

extern void ESP32_ResetBootloader(void);
extern void LPC_InvokeBootloader(void);

#endif /* INC_PARSER_H_ */
