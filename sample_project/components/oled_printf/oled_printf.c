#include "oled_printf.h"

#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static const char TAG[] = "oled_printf";

static lv_obj_t *oled_label = NULL; // Global LVGL label for printf_oled

void oled_printf_init(lv_disp_t *disp) {
  ESP_LOGI(TAG, "Initializing oled_printf with LVGL display.");
  if (lvgl_port_lock(0)) {
    oled_label = lv_label_create(lv_display_get_screen_active(disp));
    lv_obj_align(oled_label, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_label_set_text(oled_label, "");
    // Set text wrapping so that long lines wrap to the next line
    lv_label_set_long_mode(oled_label, LV_LABEL_LONG_WRAP);
    // Set object width to the display's horizontal resolution for wrapping
    lv_obj_set_width(oled_label, lv_disp_get_hor_res(disp));
    lvgl_port_unlock();
  } else {
    ESP_LOGE(TAG, "Could not acquire LVGL lock to initialize oled_printf.");
  }
}

void printf_oled(const char *format, ...) {
  char new_message_buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(new_message_buffer, sizeof(new_message_buffer), format, args);
  va_end(args);

  if (lvgl_port_lock(0)) {
    if (oled_label) {
      // Overwrite the previous text directly to keep the screen clean
      lv_label_set_text(oled_label, new_message_buffer);
      // Optional: center the text for better visibility
      lv_obj_align(oled_label, LV_ALIGN_CENTER, 0, 0);
      lv_obj_set_style_text_align(oled_label, LV_TEXT_ALIGN_CENTER, 0);
    }
    lvgl_port_unlock();
  } else {
    ESP_LOGE(TAG, "Could not acquire LVGL lock to update oled_printf label.");
  }
}
