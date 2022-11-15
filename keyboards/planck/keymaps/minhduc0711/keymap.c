/* Copyright 2015-2017 Jack Humbert
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H
#include "muse.h"


enum planck_layers {
  _QWERTY,
  _COLEMAK,
  _COLEMAK_FR,
  _GAMING,
  _LOWER,
  _RAISE,
  _ADJUST,
  _FN,
  _SPACE_FN
};

// Define layer light indicators
const rgblight_segment_t PROGMEM colemak_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {1, 8, HSV_PURPLE}
);
const rgblight_segment_t PROGMEM colemak_fr_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {2, 2, HSV_BLUE},
    {1, 1, HSV_WHITE},
    {4, 1, HSV_WHITE},
    {5, 1, HSV_WHITE},
    {8, 1, HSV_WHITE},
    {6, 2, HSV_RED}
);
const rgblight_segment_t PROGMEM gaming_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {1, 8, HSV_CORAL}
);
const rgblight_segment_t PROGMEM qwerty_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {1, 8, HSV_YELLOW}
);
const rgblight_segment_t PROGMEM lower_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {1, 8, HSV_DARK_AZURE}
);
const rgblight_segment_t PROGMEM raise_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {1, 8, HSV_ORANGE}
);
const rgblight_segment_t PROGMEM fn_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {1, 8, HSV_SPRINGGREEN}
);
const rgblight_segment_t PROGMEM space_fn_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {1, 8, HSV_GOLDENROD}
);
const rgblight_segment_t PROGMEM adjust_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {1, 8, HSV_RED}
);
const rgblight_segment_t* const PROGMEM rgb_layers[] = RGBLIGHT_LAYERS_LIST(
    colemak_layer,
    colemak_fr_layer,
    gaming_layer,
    qwerty_layer,
    lower_layer,
    raise_layer,
    fn_layer,
    space_fn_layer,
    adjust_layer
);


enum planck_keycodes {
  QWERTY = SAFE_RANGE,
  COLEMAK,
  COLEMAK_FR,
  GAMING,
  LENNY,
  SHRUG,
  GIFF
};

char LENNY_STR[] = "( ͡° ͜ʖ ͡°)";
char SHRUG_STR[] = "¯\\_(ツ)_/¯";
char GIFF_STR[] = "つ◕_◕つ";

#define LOWER MO(_LOWER)
#define RAISE MO(_RAISE)
#define FN MO(_FN)
#define SPC_FN LT(_SPACE_FN, KC_SPC)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

[_QWERTY] = LAYOUT_planck_1x2uC(
    KC_TAB,          KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_BSPC,
    LCTL_T(KC_ESC),  KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,
    KC_LSFT,         KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, RSFT_T(KC_ENT),
    KC_LCTL,         FN  ,    KC_LGUI, KC_LALT, LOWER,       SPC_FN,       RAISE,   KC_HOME, KC_PGDN, KC_PGUP, KC_END
),

[_COLEMAK] = LAYOUT_planck_1x2uC(
    KC_TAB,          KC_Q,    KC_W,    KC_F,    KC_P,    KC_G,    KC_J,    KC_L,    KC_U,    KC_Y,    KC_SCLN, KC_BSPC,
    LCTL_T(KC_ESC),  KC_A,    KC_R,    KC_S,    KC_T,    KC_D,    KC_H,    KC_N,    KC_E,    KC_I,    KC_O,    KC_QUOT,
    KC_LSFT,         KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_K,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, RSFT_T(KC_ENT),
    KC_LCTL,         FN  ,    KC_LGUI, KC_LALT, LOWER,       SPC_FN,       RAISE,   KC_HOME, KC_PGDN, KC_PGUP, KC_END
),

// Only use altgr instead of Lalt for typing french accents
[_COLEMAK_FR] = LAYOUT_planck_1x2uC(
    KC_TAB,          KC_Q,    KC_W,    KC_F,    KC_P,    KC_G,    KC_J,    KC_L,    KC_U,    KC_Y,    KC_SCLN, KC_BSPC,
    LCTL_T(KC_ESC),  KC_A,    KC_R,    KC_S,    KC_T,    KC_D,    KC_H,    KC_N,    KC_E,    KC_I,    KC_O,    KC_QUOT,
    KC_LSFT,         KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_K,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, RSFT_T(KC_ENT),
    KC_LCTL,         FN  ,    KC_LGUI, KC_RALT, LOWER,       SPC_FN,       RAISE,   KC_HOME, KC_PGDN, KC_PGUP, KC_END
),

// Colemak but with no mod-taps
[_GAMING] = LAYOUT_planck_1x2uC(
    KC_TAB,  KC_Q,    KC_W,    KC_F,    KC_P,    KC_G,    KC_J,    KC_L,    KC_U,    KC_Y,    KC_SCLN, KC_BSPC,
    KC_ESC,  KC_A,    KC_R,    KC_S,    KC_T,    KC_D,    KC_H,    KC_N,    KC_E,    KC_I,    KC_O,    KC_QUOT,
    KC_LSFT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_K,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_ENT,
    KC_LCTL, FN  ,    KC_LGUI, KC_LALT, LOWER,       KC_SPC,       RAISE,   KC_HOME, KC_PGDN, KC_PGUP, KC_END
),

[_LOWER] = LAYOUT_planck_1x2uC(
    XXXXXXX, KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,     KC_7,     KC_8,    KC_9,     KC_0,     KC_BSPC,
    XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,  KC_4,     KC_5,    KC_6,     XXXXXXX,  KC_DEL,
    _______, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,  KC_1,     KC_2,    KC_3,     XXXXXXX,  XXXXXXX,
    _______, XXXXXXX, _______, _______, _______,      KC_SPC,       _______,  KC_0,    XXXXXXX,  XXXXXXX,  XXXXXXX
),

[_RAISE] = LAYOUT_planck_1x2uC(
    KC_GRV,   KC_EXLM, KC_AT,    KC_HASH, KC_DLR,   KC_PERC,  KC_CIRC,  KC_AMPR, KC_ASTR, KC_LPRN, KC_RPRN, KC_BSPC,
    KC_TILD,  KC_PLUS, KC_UNDS,  KC_PIPE, KC_LCBR,  KC_LBRC,  KC_RBRC,  KC_RCBR, KC_BSLS, KC_MINS, KC_EQL,  KC_INS,
    XXXXXXX,  XXXXXXX, XXXXXXX,  XXXXXXX, XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,
    _______,  XXXXXXX, _______,  _______, _______,       KC_SPC,        _______, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX
),

[_FN] = LAYOUT_planck_1x2uC(
    KC_SLEP,  KC_F1,   KC_F2,   KC_F3,   KC_F4,    KC_F5,    KC_F6,    KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_SLCK,
    XXXXXXX,  KC_F11,  KC_F12,  XXXXXXX, XXXXXXX,  XXXXXXX,  XXXXXXX,  LENNY,   SHRUG,   GIFF,    XXXXXXX, XXXXXXX,
    XXXXXXX,  XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,
    XXXXXXX,  _______, XXXXXXX, XXXXXXX, XXXXXXX,       KC_SPC,        XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, UC_MOD
),

[_SPACE_FN] = LAYOUT_planck_1x2uC(
    XXXXXXX,  XXXXXXX, KC_UP,    XXXXXXX,   XXXXXXX,  XXXXXXX,  XXXXXXX,  KC_VOLD,  KC_VOLU,  KC_MUTE,  XXXXXXX, KC_PSCR,
    XXXXXXX,  KC_LEFT, KC_DOWN,  KC_RIGHT,  XXXXXXX,  XXXXXXX,  KC_LEFT,  KC_DOWN,  KC_UP,    KC_RIGHT, XXXXXXX, XXXXXXX,
    XXXXXXX,  XXXXXXX, XXXXXXX,  XXXXXXX,   XXXXXXX,  XXXXXXX,  XXXXXXX,  KC_MPRV,  KC_MPLY,  KC_MNXT,  XXXXXXX, XXXXXXX,
    XXXXXXX,  XXXXXXX, XXXXXXX,  XXXXXXX,   XXXXXXX,       _______,       XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX, XXXXXXX
),

[_ADJUST] = LAYOUT_planck_1x2uC(
    RESET,   QWERTY,  XXXXXXX, COLEMAK_FR, XXXXXXX, GAMING,  XXXXXXX, AU_ON,   AU_OFF,   RGB_TOG, RGB_MOD, XXXXXXX,
    DEBUG,   XXXXXXX, XXXXXXX, XXXXXXX,    XXXXXXX, AG_NORM, AG_SWAP, MU_ON,   MU_OFF,   RGB_HUI, RGB_HUD, XXXXXXX,
    XXXXXXX, XXXXXXX, XXXXXXX, COLEMAK,    XXXXXXX, XXXXXXX, XXXXXXX, MUV_DE,  MUV_IN,   MU_MOD,  XXXXXXX, XXXXXXX,
    _______, _______, _______, _______,    _______, _______, _______, _______,  _______, _______, _______
)

};

void keyboard_post_init_user(void) {
    // Enable the LED layers
    rgblight_layers = rgb_layers;
}

/* bool led_update_user(led_t led_state) { */
/*     rgblight_set_layer_state(0, led_state.caps_lock); */
/*     return true; */
/* } */

layer_state_t default_layer_state_set_user(layer_state_t state) {
    rgblight_set_layer_state(0, layer_state_cmp(state, _COLEMAK));
    rgblight_set_layer_state(1, layer_state_cmp(state, _COLEMAK_FR));
    rgblight_set_layer_state(2, layer_state_cmp(state, _GAMING));
    rgblight_set_layer_state(3, layer_state_cmp(state, _QWERTY));
    return state;
}

layer_state_t layer_state_set_user(layer_state_t state) {
    state = update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);

    rgblight_set_layer_state(4, layer_state_cmp(state, _LOWER));
    rgblight_set_layer_state(5, layer_state_cmp(state, _RAISE));
    rgblight_set_layer_state(6, layer_state_cmp(state, _FN));
    rgblight_set_layer_state(7, layer_state_cmp(state, _SPACE_FN));
    rgblight_set_layer_state(8, layer_state_cmp(state, _ADJUST));

    return state;
}

uint16_t get_tapping_term(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case LCTL_T(KC_ESC):
            return 50;
        case RSFT_T(KC_ENT):
            return 125;
        case SPC_FN:
            return 125;
        default:
            return TAPPING_TERM;
    }
}

bool get_ignore_mod_tap_interrupt(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case SPC_FN:
            return true;
        default:
            return false;
    }
}

#ifdef AUDIO_ENABLE
    float colemak_song[][2] = SONG(COLEMAK_SOUND);
    float qwerty_song[][2] = SONG(QWERTY_SOUND);
    float gaming_song[][2] = SONG(ONE_UP_SOUND);
    float colemak_fr_song[][2] = SONG(BAGUETTE);
#endif

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case COLEMAK:
      if (record->event.pressed) {
        set_single_persistent_default_layer(_COLEMAK);
        #ifdef AUDIO_ENABLE
          PLAY_SONG(colemak_song);
        #endif
      }
      return false;
      break;
    case COLEMAK_FR:
      if (record->event.pressed) {
        set_single_persistent_default_layer(_COLEMAK_FR);
        #ifdef AUDIO_ENABLE
          PLAY_SONG(colemak_fr_song);
        #endif
      }
      return false;
      break;
    case GAMING:
      if (record->event.pressed) {
        set_single_persistent_default_layer(_GAMING);
        #ifdef AUDIO_ENABLE
          PLAY_SONG(gaming_song);
        #endif
      }
      return false;
      break;
    case QWERTY:
      if (record->event.pressed) {
        set_single_persistent_default_layer(_QWERTY);
        #ifdef AUDIO_ENABLE
          PLAY_SONG(qwerty_song);
        #endif
      }
      return false;
      break;
    case LENNY:
      if (record->event.pressed) {
        send_unicode_string(LENNY_STR);
      }
      return false;
      break;
    case SHRUG:
      if (record->event.pressed) {
        send_unicode_string(SHRUG_STR);
      }
      return false;
      break;
    case GIFF:
      if (record->event.pressed) {
        send_unicode_string(GIFF_STR);
      }
      return false;
      break;
  }
  return true;
}

bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

bool encoder_update_user(uint8_t index, bool clockwise) {
  if (muse_mode) {
    if (IS_LAYER_ON(_RAISE)) {
      if (clockwise) {
        muse_offset++;
      } else {
        muse_offset--;
      }
    } else {
      if (clockwise) {
        muse_tempo+=1;
      } else {
        muse_tempo-=1;
      }
    }
  } else {
    if (clockwise) {
      #ifdef MOUSEKEY_ENABLE
        tap_code(KC_MS_WH_DOWN);
      #else
        tap_code(KC_PGDN);
      #endif
    } else {
      #ifdef MOUSEKEY_ENABLE
        tap_code(KC_MS_WH_UP);
      #else
        tap_code(KC_PGUP);
      #endif
    }
  }
  return true;
}

bool dip_switch_update_user(uint8_t index, bool active) {
    switch (index) {
        case 1:
            if (active) {
                muse_mode = true;
            } else {
                muse_mode = false;
            }
    }
    return true;
}

void matrix_scan_user(void) {
#ifdef AUDIO_ENABLE
    if (muse_mode) {
        if (muse_counter == 0) {
            uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
            if (muse_note != last_muse_note) {
                stop_note(compute_freq_for_midi_note(last_muse_note));
                play_note(compute_freq_for_midi_note(muse_note), 0xF);
                last_muse_note = muse_note;
            }
        }
        muse_counter = (muse_counter + 1) % muse_tempo;
    } else {
        if (muse_counter) {
            stop_all_notes();
            muse_counter = 0;
        }
    }
#endif
}

bool music_mask_user(uint16_t keycode) {
  switch (keycode) {
    case RAISE:
    case LOWER:
      return false;
    default:
      return true;
  }
}
