/*
Copyright 2016 Jun Wako <wakojun@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <avr/pgmspace.h>
#include "unimap_trans.h"
#include "bootloader.h"
#include "wait.h"
#include "keycode.h"
#include "action.h"
#include "action_util.h"
#include "keymap.h"

extern bool is_ver_jp;

enum function_id {
    TRICKY_ESC,
    LALT_F4,
    JMP_BL,
    RGBLED_ACTION = 0x0A,
    HOST_SWITCH,
    BT_CLEAR,
    SLEEP_EN,
    INNER_USB_TOGGLE,
};

enum macro_id {
    HOME_PGUP,
    END_PGDN,
};


// Layers
#define _MAC   0
#define _WINS  1
#define _LOWER 2
#define _RAISE 3
#define _MOUSE 4

// Functions
#define AC_RESET ACTION_FUNCTION(JMP_BL)
#define AC_RST   ACTION_FUNCTION(JMP_BL)
#define AC_BUT   ACTION_FUNCTION(HOST_SWITCH)
#define AC_BTC   ACTION_FUNCTION(BT_CLEAR)
#define AC_USB   ACTION_FUNCTION(INNER_USB_TOGGLE)

// Dual-role keys
#define AC_MAC  ACTION_DEFAULT_LAYER_SET(_MAC)
#define AC_WINS ACTION_DEFAULT_LAYER_SET(_WINS)
#define AC_CESC ACTION_MODS_TAP_KEY(MOD_LCTL, KC_ESC)
#define AC_CSLS ACTION_MODS_TAP_KEY(MOD_RCTL, KC_SLSH)
#define AC_GQUO ACTION_MODS_TAP_KEY(MOD_RGUI, KC_SLSH)
#define AC_AQUO ACTION_MODS_TAP_KEY(MOD_RALT, KC_SLSH)
#define AC_MSCL ACTION_LAYER_TAP_KEY(_MOUSE, KC_SCLN)
#define AC_LGRV ACTION_LAYER_TAP_KEY(_LOWER, KC_GRV)
#define AC_LSPC ACTION_LAYER_TAP_KEY(_LOWER, KC_SPC)
#define AC_RDEL ACTION_LAYER_TAP_KEY(_RAISE, KC_DEL)

// Keys with Shift
#define AC_TILD ACTION_MODS_KEY(MOD_LSFT, KC_GRV)
#define AC_EXLM ACTION_MODS_KEY(MOD_LSFT, KC_1)
#define AC_AT   ACTION_MODS_KEY(MOD_LSFT, KC_2)
#define AC_HASH ACTION_MODS_KEY(MOD_LSFT, KC_3)
#define AC_DLR  ACTION_MODS_KEY(MOD_LSFT, KC_4)
#define AC_PERC ACTION_MODS_KEY(MOD_LSFT, KC_5)
#define AC_CIRC ACTION_MODS_KEY(MOD_LSFT, KC_6)
#define AC_AMPR ACTION_MODS_KEY(MOD_LSFT, KC_7)
#define AC_ASTR ACTION_MODS_KEY(MOD_LSFT, KC_8)
#define AC_LPRN ACTION_MODS_KEY(MOD_LSFT, KC_9)
#define AC_RPRN ACTION_MODS_KEY(MOD_LSFT, KC_0)
#define AC_UNDS ACTION_MODS_KEY(MOD_LSFT, KC_MINS)
#define AC_PLUS ACTION_MODS_KEY(MOD_LSFT, KC_EQUAL)
#define AC_PIPE ACTION_MODS_KEY(MOD_LSFT, KC_SLSH)


#define UNIMAP_JP( \
    K35,K1E,K1F,K20,K21,K22,K23,K24,K25,K26,K27,K2D,K2E,K74,K2A,  \
    K2B,K14,K1A,K08,K15,K17,K1C,K18,K0C,K12,K13,K2F,K30,          \
    K39,K04,K16,K07,K09,K0A,K0B,K0D,K0E,K0F,K33,K34,K32,    K28,  \
    K79,    K1D,K1B,K06,K19,K05,K11,K10,K36,K37,K38,K75,K52,K7D,  \
    K78,K29,K7B,K7A,K77,    K2C,    K76,K00,K7E,K7C,K50,K51,K4F   \
) UNIMAP( \
            NO, NO, NO, NO, NO, NO, NO, NO, NO, NO, NO, NO,                                     \
    K29,    NO, NO, NO, NO, NO, NO, NO, NO, NO, NO, NO, NO,       NO, NO, NO,       NO, NO, NO, \
    K35,K1E,K1F,K20,K21,K22,K23,K24,K25,K26,K27,K2D,K2E,K74,K2A,  NO, NO, NO,   NO, NO, NO, NO, \
    K2B,K14,K1A,K08,K15,K17,K1C,K18,K0C,K12,K13,K2F,K30,    NO,   NO, NO, NO,   NO, NO, NO, NO, \
    K39,K04,K16,K07,K09,K0A,K0B,K0D,K0E,K0F,K33,K34,    K32,K28,                NO, NO, NO, NO, \
    K79,NO, K1D,K1B,K06,K19,K05,K11,K10,K36,K37,K38,    K75,K7D,      K52,      NO, NO, NO, NO, \
    K78,K7B,K7A,K77,        K2C,        K76,K00,K7E,NO, NO, K7C,  K50,K51,K4F,  NO,     NO, NO  \
)


#ifdef KEYMAP_SECTION_ENABLE
const action_t actionmaps[][UNIMAP_ROWS][UNIMAP_COLS] __attribute__ ((section (".keymap.keymaps"))) = {
#else
const action_t actionmaps[][UNIMAP_ROWS][UNIMAP_COLS] PROGMEM = {
#endif
    UNIMAP_JP(
        ESC,  1,    2,    3,    4,    5,    6,    7,    8,    9,    0,    MINS, EQL,  BSLS, BSPC,
        TAB,   Q,    W,    E,    R,    T,    Y,    U,    I,    O,    P,    LBRC,  RBRC,
        CESC,   A,    S,    D,    F,    G,    H,    J,    K,    L,    MSCL, GQUO,   GRV,    ENT,
        LSFT,    Z,    X,    C,    V,    B,    N,    M,    COMM, DOT,  CSLS,  RSFT,   UP,   RDEL,
        LGRV, LCTL, LALT, LGUI, LSPC,       SPC,    RDEL, RGUI, RALT, WINS,   LEFT,   DOWN, RGHT
    ),
    UNIMAP_JP(
        ESC,  1,    2,    3,    4,    5,    6,    7,    8,    9,    0,    MINS, EQL,  BSLS, BSPC,
        TAB,   Q,    W,    E,    R,    T,    Y,    U,    I,    O,    P,    LBRC,  RBRC,
        CESC,   A,    S,    D,    F,    G,    H,    J,    K,    L,    MSCL, AQUO,   GRV,    ENT,
        LSFT,    Z,    X,    C,    V,    B,    N,    M,    COMM, DOT,  CSLS,  RSFT,   UP,   RDEL,
        LGRV, LCTL, LGUI, LALT, LSPC,       SPC,    RDEL, RALT, RGUI, MAC,    LEFT,   DOWN, RGHT
    ),
    UNIMAP_JP(
        TILD, EXLM, AT,   HASH, DLR,  PERC, CIRC, AMPR, ASTR, LPRN, RPRN, UNDS, PLUS, PIPE, DEL,
        CAPS,  BUT,  NO,   NO,   NO,   NO,   NO,   USB,  PSCR, SLCK, PAUS, UP,    NO,
        TRNS,   VOLD, VOLU, MUTE, NO,   NO,   PAST, PSLS, HOME, PGUP, LEFT, RGHT,   NO,     PENT,
        TRNS,    NO,   NO,   BTC,  NO,   RST,  PPLS, PMNS, END,  PGDN, DOWN,  TRNS,   TRNS, TRNS,
        TRNS, TRNS, TRNS, TRNS, TRNS,      TRNS,    MENU, TRNS, TRNS, TRNS,   TRNS,   TRNS, TRNS
    ),
    UNIMAP_JP(
        PWR,  F1,   F2,   F3,   F4,   F5,   F6,   F7,   F8,   F9,   F10,  F11,  F12,  INS,  DEL,
        CAPS,  BUT,  NO,   NO,   NO,   NO,   NO,   USB,  PSCR, SLCK, PAUS, UP,    NO,
        TRNS,   VOLD, VOLU, MUTE, NO,   NO,   PAST, PSLS, HOME, PGUP, LEFT, RGHT,   NO,     PENT,
        TRNS,    NO,   NO,   BTC,  NO,   RST,  PPLS, PMNS, END,  PGDN, DOWN,  TRNS,   TRNS, TRNS,
        TRNS, TRNS, TRNS, TRNS, APP,       TRNS,    TRNS, TRNS, TRNS, TRNS,   TRNS,   TRNS, TRNS
    ),
    UNIMAP_JP(
        NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,
        NO,    NO,   BTN3, MS_U, BTN2, NO,   NO,   BTN2, WH_U, BTN3, NO,   NO,    NO,
        TRNS,   BTN1, MS_L, MS_D, MS_R, BTN1, BTN1, WH_L, WH_D, WH_R, NO,   NO,     NO,     NO,
        TRNS,    NO,   NO,   NO,   BTN3, NO,   BTN3, NO,   NO,   NO,   NO,    TRNS,   TRNS, TRNS,
        TRNS, TRNS, TRNS, TRNS, F21,       TRNS,    F20,  TRNS, TRNS, TRNS,   TRNS,   TRNS, TRNS
    ),
    // UNIMAP_JP(
    //     NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,
    //     NO,    NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,    NO,
    //     TRNS,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,     NO,     NO,
    //     TRNS,    NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,   NO,    TRNS,   TRNS, TRNS,
    //     TRNS, TRNS, TRNS, TRNS, TRNS,      TRNS,    TRNS, TRNS, TRNS, TRNS,   TRNS,   TRNS, TRNS
    // ),
};

keypos_t unimap_translate(keypos_t key)
{
    uint8_t unimap_pos;
    if (is_ver_jp) unimap_pos = pgm_read_byte(&unimap_trans_jp[key.row][key.col]);
    else unimap_pos = pgm_read_byte(&unimap_trans[key.row][key.col]);
    return (keypos_t) {
        .row = ((unimap_pos & 0xf0) >> 4),
        .col = (unimap_pos & 0x0f)
    };
};

#define MODS_SHIFT_MASK (MOD_BIT(KC_LSHIFT)|MOD_BIT(KC_RSHIFT))
void action_function(keyrecord_t *record, uint8_t id, uint8_t opt)
{
    static uint8_t mod_keys_registered;
    switch (id) {
        case TRICKY_ESC:
            if (record->event.pressed) {
                if ((get_mods() & MODS_SHIFT_MASK) && (!(get_mods() & MOD_BIT(KC_LCTRL)))) {
                    mod_keys_registered = KC_GRV;
                }
                else {
                    mod_keys_registered = KC_ESC;
                }
                register_code(mod_keys_registered);
                send_keyboard_report();
            }
            else {
                unregister_code(mod_keys_registered);
                send_keyboard_report();
            }
            break;
        case LALT_F4:
            if (record->event.pressed) {
                if (get_mods() & MOD_BIT(KC_LALT)) {
                    mod_keys_registered = KC_F4;
                }
                else {
                    mod_keys_registered = KC_4;
                }
                register_code(mod_keys_registered);
                send_keyboard_report();
            } else {
                unregister_code(mod_keys_registered);
                send_keyboard_report();
            }
            break;
    }
    if (record->event.pressed) {
        switch (id) {
            case JMP_BL:
                if (get_mods() & MOD_BIT(KC_LCTRL)) {
                    uint16_t *const bootKeyPtr = (uint16_t *)0x0800;
                    *bootKeyPtr = 0xFC2B;
                    command_extra(KC_B);
                }
                register_mods(0b11111111);
                unregister_mods(0b11111111);
                clear_keyboard();
                break;
            case RGBLED_ACTION:
                //rgblight_action(opt);
                break;
            case BT_CLEAR:
                command_extra(KC_R);
                break;
            case HOST_SWITCH:
                command_extra(KC_U);
                break;
            case SLEEP_EN:
                command_extra(KC_P);
                break;
            case INNER_USB_TOGGLE:
                if (PORTF & (1<<PF7)) {
                    DDRF  |=  (1<<PF7);
                    PORTF &= ~(1<<PF7);
                } else {
                    DDRF  &= ~(1<<PF7);
                    PORTF |=  (1<<PF7);
                }
                break;
        }
    }
};

const macro_t *action_get_macro(keyrecord_t *record, uint8_t id, uint8_t opt)
{
    static uint16_t key_timer;
    switch(id) {
        case HOME_PGUP:
            if (record->event.pressed) {
                key_timer = timer_read();
            }
            else {
                if (timer_elapsed(key_timer) > 200) {
                    return MACRO( T(HOME), END );
                }
                else {
                    return MACRO( T(PGUP), END );
                }
            }
            break;
        case END_PGDN:
            if (record->event.pressed) {
                key_timer = timer_read();
            }
            else {
                if (timer_elapsed(key_timer) > 200) {
                    return MACRO( T(END), END );
                }
                else {
                    return MACRO( T(PGDN), END );
                }
            }
            break;
    }
    return MACRO_NONE;
};
