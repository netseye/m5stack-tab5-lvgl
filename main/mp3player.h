/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include "usb/uac_host.h"

/** Major version number (X.x.x) */
#define VERSION_MAJOR 0
/** Minor version number (x.X.x) */
#define VERSION_MINOR 0
/** Patch version number (x.x.X) */
#define VERSION_PATCH 1

typedef enum {
    AUDIO_PLAYER_I2S = 0,
    AUDIO_PLAYER_USB,
} audio_player_t;

/**
 * @brief get audio player type
 *
 */
audio_player_t get_audio_player_type(void);

/**
 * @brief get audio player handle
 *
 */
uac_host_device_handle_t get_audio_player_handle(void);

void player_init(void);
void usb_host_init(void);

/**
 * Macro to convert version number into an integer
 *
 * To be used in comparisons, such as VERSION >= VERSION_VAL(4, 0, 0)
 */
#define VERSION_VAL(major, minor, patch) ((major << 16) | (minor << 8) | (patch))

/**
 * Current version, as an integer
 *
 * To be used in comparisons, such as VERSION >= VERSION_VAL(4, 0, 0)
 */
#define VERSION VERSION_VAL(VERSION_MAJOR, \
                            VERSION_MINOR, \
                            VERSION_PATCH)
