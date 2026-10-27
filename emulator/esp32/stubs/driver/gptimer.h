// The subset of ESP-IDF's gptimer API used by segment_player.
#pragma once
#include <stdint.h>
typedef int esp_err_t;
#define ESP_OK 0
typedef struct gptimer_t* gptimer_handle_t;
typedef enum { GPTIMER_CLK_SRC_DEFAULT = 4 } gptimer_clock_source_t;
typedef enum { GPTIMER_COUNT_DOWN, GPTIMER_COUNT_UP } gptimer_count_direction_t;
typedef struct {
    gptimer_clock_source_t clk_src;
    gptimer_count_direction_t direction;
    uint32_t resolution_hz;
    int intr_priority;
} gptimer_config_t;
typedef struct { uint64_t count_value; uint64_t alarm_value; } gptimer_alarm_event_data_t;
typedef bool (*gptimer_alarm_cb_t)(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*);
typedef struct { gptimer_alarm_cb_t on_alarm; } gptimer_event_callbacks_t;
typedef struct {
    uint64_t alarm_count;
    uint64_t reload_count;
    struct { uint32_t auto_reload_on_alarm : 1; } flags;
} gptimer_alarm_config_t;
esp_err_t gptimer_new_timer(const gptimer_config_t* config, gptimer_handle_t* ret_timer);
esp_err_t gptimer_del_timer(gptimer_handle_t timer);
esp_err_t gptimer_get_raw_count(gptimer_handle_t timer, uint64_t* value);
esp_err_t gptimer_register_event_callbacks(gptimer_handle_t timer, const gptimer_event_callbacks_t* cbs,
                                           void* user_data);
esp_err_t gptimer_set_alarm_action(gptimer_handle_t timer, const gptimer_alarm_config_t* config);
esp_err_t gptimer_enable(gptimer_handle_t timer);
esp_err_t gptimer_disable(gptimer_handle_t timer);
esp_err_t gptimer_start(gptimer_handle_t timer);
esp_err_t gptimer_stop(gptimer_handle_t timer);
