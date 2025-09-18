/**
 * Copyright (c) 2021 mjcross
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// SDK types and declarations
//https://github.com/raspberrypi/pico-examples/tree/master/pio/ir_nec

#include "pico/stdlib.h"
#include "hardware/pio.h"

// public API

int nec_rx_init(PIO pio, uint pin);
bool nec_decode_frame(uint32_t sm, uint8_t *p_address, uint8_t *p_data);

// Codes des touches de la telecommande Ebay
#define TOUCHE_CH_MOINS     0xA2
#define TOUCHE_CH           0x62
#define TOUCHE_CH_PLUS      0xE2
#define TOUCHE_0            0x68
#define TOUCHE_1            0x30
#define TOUCHE_2            0x18
#define TOUCHE_3            0x7A
#define TOUCHE_4            0x10
#define TOUCHE_5            0x38
#define TOUCHE_6            0x5A
#define TOUCHE_7            0x42
#define TOUCHE_8            0x4A
#define TOUCHE_9            0x52
#define TOUCHE_MOINS        0xE0
#define TOUCHE_PLUS         0xA8
#define TOUCHE_AUCUNE       0x00
