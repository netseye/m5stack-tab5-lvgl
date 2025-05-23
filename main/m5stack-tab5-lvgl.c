#include "lv_demos.h"
#include <bsp/esp-bsp.h>
#include <stdio.h>

extern esp_lcd_touch_handle_t _lcd_touch_handle;

lv_disp_t *lvDisp = NULL;
lv_indev_t *lvTouchpad = NULL;

static void lvgl_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    if (_lcd_touch_handle == NULL)
    {
        data->state = LV_INDEV_STATE_REL;
        return;
    }
    uint16_t touch_x[1];
    uint16_t touch_y[1];
    uint16_t touch_strength[1];
    uint8_t touch_cnt = 0;

    esp_lcd_touch_read_data(_lcd_touch_handle);
    bool touchpad_pressed =
        esp_lcd_touch_get_coordinates(_lcd_touch_handle, touch_x, touch_y, touch_strength, &touch_cnt, 1);
    // mclog::tagInfo(_tag, "touchpad pressed: {}", touchpad_pressed);

    if (!touchpad_pressed)
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = touch_x[0];
        data->point.y = touch_y[0];
    }
}

void app_main(void) {

  bsp_i2c_init();
  vTaskDelay(pdMS_TO_TICKS(200));

  i2c_master_bus_handle_t i2c_bus_handle = bsp_i2c_get_handle();
  bsp_io_expander_pi4ioe_init(i2c_bus_handle);

  bsp_reset_tp();
  lvDisp = bsp_display_start();
  lv_display_set_rotation(lvDisp, LV_DISPLAY_ROTATION_90);
  bsp_display_backlight_on();

  lvTouchpad = lv_indev_create();
  lv_indev_set_type(lvTouchpad, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(lvTouchpad, lvgl_read_cb);
  lv_indev_set_display(lvTouchpad, lvDisp);

  lv_demo_music();

  bsp_display_unlock();
}
