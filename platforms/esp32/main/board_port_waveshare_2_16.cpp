#include "board_port.h"

#include "rtc_clock_driver.h"

#include <cstddef>
#include <cstdint>

#include "bsp/display.h"
#include "bsp/esp-bsp.h"
#include "bsp/touch.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "esp_lcd_co5300.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch_cst9220.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

namespace {
constexpr char kTag[] = "board_waveshare";
// Keep internal SRAM available for BLE and heading task stacks. Two 320-row
// PSRAM buffers reduce repeated map traversal and still overlap drawing with
// QSPI DMA. A 32-row internal-buffer experiment exhausted BLE startup memory.
constexpr std::uint16_t kDrawBufferHeight = 320;
constexpr bool kDrawBuffersUsePsram = true;
constexpr std::size_t kLvglExtraPoolBytes = 2U * 1024U * 1024U;
static_assert(kLvglExtraPoolBytes <= LV_MEM_POOL_EXPAND_SIZE,
              "LVGL TLSF must support the board's extra PSRAM pool");
// The 2.16 board's user key is active-low on GPIO18. GPIO13 is RTC_INT on
// this board, so the 1.75C TE synchronizer must not be attached here.
constexpr gpio_num_t kPowerButtonGpio = GPIO_NUM_18;
// BOOT key. GPIO0 doubles as the download-mode strap, so it is only ever read
// after boot; nothing here touches it during startup.
constexpr gpio_num_t kBootButtonGpio = GPIO_NUM_0;
// Kept only so the legacy probe helpers remain well-formed; they are never
// registered or called for the 2.16 board.
constexpr gpio_num_t kLcdTeGpio = GPIO_NUM_13;
// Keep only one large direct-PSRAM-DMA transfer in flight to bound DMA memory.
constexpr int kDisplayQueueDepth = 1;
constexpr std::uint32_t kDisplayPclkHz = 20 * 1'000 * 1'000;
constexpr std::uint32_t kTeMinimumPeriodUs = 8'000;
constexpr std::uint32_t kTeMaximumPeriodUs = 40'000;
constexpr std::uint32_t kTeMaximumEdgeIntervalUs = 100'000;
constexpr std::uint32_t kTePeriodToleranceFloorUs = 500;
constexpr std::uint8_t kTeStablePeriodCount = 8;
constexpr std::uint32_t kTeProbeReportDelayUs = 2'000'000;
constexpr std::uint32_t kTeWaitTimeoutUs = 25'000;
// Starting a transfer long after active scan began can make the following
// scan catch the QSPI writer.  A just-seen edge may be reused, but older edges
// are discarded and the first flush waits for the next one.
constexpr std::uint32_t kTeMaximumAcceptedEdgeAgeUs = 2'000;
constexpr std::uint8_t kTeMaximumConsecutiveTimeouts = 3;
constexpr std::uint16_t kAxp2101Address = 0x34;
constexpr std::uint8_t kAxp2101CommonConfigRegister = 0x10;
constexpr std::uint8_t kAxp2101SoftwarePowerOffMask = 0x01;
lv_display_t* display = nullptr;
esp_lcd_panel_handle_t panel = nullptr;
i2c_master_dev_handle_t pmic = nullptr;
moto::esp32::RtcClock rtc;
bool display_revealed = false;
void* lvgl_extra_pool_storage = nullptr;

struct TeSyncState {
  std::int64_t last_edge_us = 0;
  std::int64_t last_active_edge_us = 0;
  std::int64_t reveal_us = 0;
  std::uint32_t high_duration_us = 0;
  std::uint32_t low_duration_us = 0;
  std::uint32_t estimated_period_us = 0;
  std::uint32_t estimated_pulse_us = 0;
  std::uint32_t edge_count = 0;
  std::int8_t last_level = -1;
  std::int8_t candidate_active_level = -1;
  std::int8_t active_level = -1;
  std::uint8_t stable_periods = 0;
  std::uint8_t consecutive_timeouts = 0;
  bool panel_revealed = false;
  bool probe_stable = false;
  bool probe_reported = false;
  bool probe_timeout_reported = false;
  bool gate_enabled = false;
  bool gate_permanently_disabled = false;
  bool wait_first_flush = false;
};

portMUX_TYPE te_sync_lock = portMUX_INITIALIZER_UNLOCKED;
TeSyncState te_sync_state{};
SemaphoreHandle_t te_active_edge_sem = nullptr;
esp_lcd_panel_io_handle_t touch_panel_io = nullptr;

// All timing counters are owned by the LVGL worker, never touched in the TE
// ISR. These measure frames actually rendered, not the configured timer rate.
struct DisplayTiming {
  std::int64_t window_start_us = 0;
  std::int64_t render_start_us = 0;
  std::int64_t flush_wait_start_us = 0;
  std::int64_t flush_callback_start_us = 0;
  std::uint64_t render_total_us = 0;
  std::uint64_t te_wait_total_us = 0;
  std::uint64_t dma_wait_total_us = 0;
  std::uint64_t flush_callback_total_us = 0;
  std::uint32_t render_max_us = 0;
  std::uint32_t flush_callback_max_us = 0;
  std::uint32_t frames = 0;
  std::uint32_t flushes = 0;
  std::uint32_t over_budget_frames = 0;
} display_timing;

void display_timing_event(lv_event_t* event) {
  const lv_event_code_t code = lv_event_get_code(event);
  const std::int64_t now_us = esp_timer_get_time();
  auto& timing = display_timing;
  if (timing.window_start_us == 0) timing.window_start_us = now_us;
  if (code == LV_EVENT_RENDER_START) {
    timing.render_start_us = now_us;
  } else if (code == LV_EVENT_FLUSH_START) {
    ++timing.flushes;
    // This observer is registered after the TE gate, so this interval isolates
    // flush_cb itself: byte swapping, cache work and panel/DMA submission.
    timing.flush_callback_start_us = now_us;
  } else if (code == LV_EVENT_FLUSH_FINISH && timing.flush_callback_start_us != 0) {
    const auto duration = static_cast<std::uint32_t>(
        now_us - timing.flush_callback_start_us);
    timing.flush_callback_total_us += duration;
    if (duration > timing.flush_callback_max_us)
      timing.flush_callback_max_us = duration;
    timing.flush_callback_start_us = 0;
  } else if (code == LV_EVENT_FLUSH_WAIT_START) {
    timing.flush_wait_start_us = now_us;
  } else if (code == LV_EVENT_FLUSH_WAIT_FINISH && timing.flush_wait_start_us != 0) {
    timing.dma_wait_total_us += now_us - timing.flush_wait_start_us;
    timing.flush_wait_start_us = 0;
  } else if (code == LV_EVENT_RENDER_READY && timing.render_start_us != 0) {
    const auto duration = static_cast<std::uint32_t>(
        now_us - timing.render_start_us);
    timing.render_total_us += duration;
    if (duration > timing.render_max_us) timing.render_max_us = duration;
    if (duration > LV_DEF_REFR_PERIOD * 1'000U) ++timing.over_budget_frames;
    ++timing.frames;
    timing.render_start_us = 0;
    const auto window_us = now_us - timing.window_start_us;
    if (window_us >= 5'000'000) {
      ESP_LOGI(kTag,
               "display perf: %.1f fps, render %.1f/%.1f ms avg/max, "
               "TE/DMA wait %.1f/%.1f ms/frame, %.1f flushes/frame, over-%dms %lu/%lu",
               static_cast<double>(timing.frames) * 1'000'000.0 / window_us,
               static_cast<double>(timing.render_total_us) / timing.frames / 1'000.0,
               static_cast<double>(timing.render_max_us) / 1'000.0,
               static_cast<double>(timing.te_wait_total_us) / timing.frames / 1'000.0,
               static_cast<double>(timing.dma_wait_total_us) / timing.frames / 1'000.0,
               static_cast<double>(timing.flushes) / timing.frames,
               LV_DEF_REFR_PERIOD,
               static_cast<unsigned long>(timing.over_budget_frames),
               static_cast<unsigned long>(timing.frames));
      ESP_LOGI(kTag,
               "display flush cb: %.1f ms total, %.1f ms/flush avg, "
               "%.1f ms max, %.1f ms/frame (excludes TE)",
               static_cast<double>(timing.flush_callback_total_us) / 1'000.0,
               timing.flushes == 0 ? 0.0 :
                   static_cast<double>(timing.flush_callback_total_us) /
                       timing.flushes / 1'000.0,
               static_cast<double>(timing.flush_callback_max_us) / 1'000.0,
               static_cast<double>(timing.flush_callback_total_us) /
                   timing.frames / 1'000.0);
      ESP_LOGI(kTag, "internal heap: free %lu KiB, minimum %lu KiB",
               static_cast<unsigned long>(heap_caps_get_free_size(
                   MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024U),
               static_cast<unsigned long>(heap_caps_get_minimum_free_size(
                   MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024U));
      timing = {};
      timing.window_start_us = now_us;
    }
  }
}

// Waveshare's public bsp_display_new() currently sends both 0x51=0xff and
// DISPON as part of its private vendor sequence, then sends DISPON once more
// before returning.  Consequently, calling DISPOFF immediately after that API
// can only shorten the unwanted white frame; it cannot prevent it.  Keep the
// supplier's sequence here, but initialize at zero brightness and deliberately
// omit DISPON.  The panel is revealed only after LVGL has committed our black
// boot frame.
constexpr std::uint8_t kPage20[] = {0x20};
constexpr std::uint8_t kGateVoltage[] = {0x10};
constexpr std::uint8_t kSourceVoltage[] = {0xA0};
constexpr std::uint8_t kPage00[] = {0x00};
constexpr std::uint8_t kGammaControl[] = {0x80};
constexpr std::uint8_t kRgb565[] = {0x55};
constexpr std::uint8_t kWriteControl[] = {0x20};
constexpr std::uint8_t kBrightnessOff[] = {0x00};
constexpr std::uint8_t kRegister63Value[] = {0xFF};
constexpr std::uint8_t kColumnRange[] = {0x00, 0x00, 0x01, 0xDF};
constexpr std::uint8_t kRowRange[] = {0x00, 0x00, 0x01, 0xDF};
constexpr std::uint8_t kTearingEffectOff[] = {0x00};
constexpr std::uint8_t kDisplayOrientation[] = {0xA0};

const co5300_lcd_init_cmd_t kHiddenPanelInitCommands[] = {
    {0x11, nullptr, 0, 600},
    {0xFE, kPage20, sizeof(kPage20), 0},
    {0x19, kGateVoltage, sizeof(kGateVoltage), 0},
    {0x1C, kSourceVoltage, sizeof(kSourceVoltage), 0},
    {0xFE, kPage00, sizeof(kPage00), 0},
    {0xC4, kGammaControl, sizeof(kGammaControl), 0},
    {0x3A, kRgb565, sizeof(kRgb565), 0},
    {0x35, kTearingEffectOff, sizeof(kTearingEffectOff), 0},
    {0x53, kWriteControl, sizeof(kWriteControl), 0},
    {0x51, kBrightnessOff, sizeof(kBrightnessOff), 0},
    {0x63, kRegister63Value, sizeof(kRegister63Value), 0},
    {0x2A, kColumnRange, sizeof(kColumnRange), 0},
    {0x2B, kRowRange, sizeof(kRowRange), 0},
    {0x36, kDisplayOrientation, sizeof(kDisplayOrientation), 0},
};

esp_err_t initialize_hidden_panel(std::size_t maximum_transfer_size,
                                  esp_lcd_panel_handle_t* panel_out,
                                  esp_lcd_panel_io_handle_t* panel_io_out) {
  // Assign fields explicitly because the vendor's C99 designated-initializer
  // macro does not follow spi_bus_config_t's declaration order and is rejected
  // when this board port is compiled as C++17.
  spi_bus_config_t bus_config{};
  bus_config.data0_io_num = BSP_LCD_DATA0;
  bus_config.data1_io_num = BSP_LCD_DATA1;
  bus_config.sclk_io_num = BSP_LCD_PCLK;
  bus_config.data2_io_num = BSP_LCD_DATA2;
  bus_config.data3_io_num = BSP_LCD_DATA3;
  bus_config.data4_io_num = -1;
  bus_config.data5_io_num = -1;
  bus_config.data6_io_num = -1;
  bus_config.data7_io_num = -1;
  bus_config.max_transfer_sz = maximum_transfer_size;
  ESP_RETURN_ON_ERROR(
      spi_bus_initialize(BSP_LCD_SPI_NUM, &bus_config, SPI_DMA_CH_AUTO), kTag,
      "CO5300 QSPI bus initialization failed");

  esp_lcd_panel_io_spi_config_t io_config{};
  io_config.cs_gpio_num = BSP_LCD_CS;
  io_config.dc_gpio_num = -1;
  io_config.spi_mode = 0;
  // Direct PSRAM DMA underflowed at 40 MHz on the live 2.16 board. A 20 MHz
  // QSPI clock leaves more margin for the PSRAM-to-GPSPI transfer path.
  io_config.pclk_hz = kDisplayPclkHz;
  io_config.trans_queue_depth = kDisplayQueueDepth;
  io_config.lcd_cmd_bits = 32;
  io_config.lcd_param_bits = 8;
  io_config.flags.quad_mode = true;
  // ESP32-S3 GPSPI can read an aligned PSRAM color buffer directly through
  // GDMA. Without this flag esp_lcd allocates and copies into a same-sized
  // internal bounce buffer for every flush, which caused the old large-buffer
  // configuration to fail once NimBLE was active.
  io_config.flags.psram_dma_direct = true;
  esp_lcd_panel_io_handle_t new_panel_io = nullptr;
  ESP_RETURN_ON_ERROR(
      esp_lcd_new_panel_io_spi(
          static_cast<esp_lcd_spi_bus_handle_t>(BSP_LCD_SPI_NUM),
          &io_config, &new_panel_io),
      kTag, "CO5300 QSPI panel IO initialization failed");

  co5300_vendor_config_t vendor_config{};
  vendor_config.init_cmds = kHiddenPanelInitCommands;
  vendor_config.init_cmds_size =
      sizeof(kHiddenPanelInitCommands) / sizeof(kHiddenPanelInitCommands[0]);
  vendor_config.flags.use_qspi_interface = 1;

  esp_lcd_panel_dev_config_t device_config{};
  device_config.reset_gpio_num = BSP_LCD_RST;
  device_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
  device_config.bits_per_pixel = BSP_LCD_BITS_PER_PIXEL;
  device_config.vendor_config = &vendor_config;

  esp_lcd_panel_handle_t new_panel = nullptr;
  ESP_RETURN_ON_ERROR(
      esp_lcd_new_panel_co5300(new_panel_io, &device_config, &new_panel), kTag,
      "CO5300 panel creation failed");
  ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(new_panel, 0, 0), kTag,
                      "CO5300 panel gap setup failed");
  ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(new_panel), kTag,
                      "CO5300 panel reset failed");
  ESP_RETURN_ON_ERROR(esp_lcd_panel_init(new_panel), kTag,
                      "CO5300 hidden initialization failed");
  ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(new_panel, false), kTag,
                      "CO5300 could not remain hidden after initialization");

  *panel_out = new_panel;
  *panel_io_out = new_panel_io;
  return ESP_OK;
}

esp_err_t read_pmic_register(std::uint8_t reg, std::uint8_t* value) {
  return i2c_master_transmit_receive(pmic, &reg, 1, value, 1, 100);
}

esp_err_t write_pmic_register(std::uint8_t reg, std::uint8_t value) {
  const std::uint8_t command[2] = {reg, value};
  return i2c_master_transmit(pmic, command, sizeof(command), 100);
}

// Keep one transfer in flight. Double buffering overlaps CPU drawing with the
// active transfer; a deeper esp_lcd transaction queue does not add another
// LVGL buffer and only increases memory/latency under continuous animation.
static_assert(kDisplayQueueDepth == 1,
              "CO5300 transfers must be serialized to bound DMA memory");

void round_draw_area(lv_area_t* area, void*) {
  // CO5300 QSPI writes must begin/end on two-pixel boundaries. Keeping the
  // buffer height even guarantees a rounded strip still fits the draw buffer.
  area->x1 = (area->x1 >> 1) << 1;
  area->y1 = (area->y1 >> 1) << 1;
  area->x2 = ((area->x2 >> 1) << 1) + 1;
  area->y2 = ((area->y2 >> 1) << 1) + 1;
}

void IRAM_ATTR lcd_te_gpio_isr(void*) {
  const int current_level = gpio_get_level(kLcdTeGpio);
  const std::int64_t now_us = esp_timer_get_time();
  BaseType_t task_woken = pdFALSE;
  bool signal_active_edge = false;

  portENTER_CRITICAL_ISR(&te_sync_lock);
  TeSyncState& state = te_sync_state;
  if (!state.panel_revealed) {
    portEXIT_CRITICAL_ISR(&te_sync_lock);
    return;
  }

  ++state.edge_count;
  if (state.last_level < 0 || state.last_edge_us == 0) {
    state.last_level = static_cast<std::int8_t>(current_level);
    state.last_edge_us = now_us;
    portEXIT_CRITICAL_ISR(&te_sync_lock);
    return;
  }

  // ANYEDGE should alternate levels. Ignore a duplicate interrupt without
  // moving the timestamp; the next real edge will still measure the complete
  // preceding level duration.
  if (current_level == state.last_level) {
    portEXIT_CRITICAL_ISR(&te_sync_lock);
    return;
  }

  const std::int64_t elapsed_us = now_us - state.last_edge_us;
  state.last_level = static_cast<std::int8_t>(current_level);
  state.last_edge_us = now_us;
  if (elapsed_us <= 0 ||
      elapsed_us > static_cast<std::int64_t>(kTeMaximumEdgeIntervalUs)) {
    state.high_duration_us = 0;
    state.low_duration_us = 0;
    state.last_active_edge_us = 0;
    if (!state.probe_stable) {
      state.estimated_period_us = 0;
      state.estimated_pulse_us = 0;
      state.candidate_active_level = -1;
      state.stable_periods = 0;
    }
    portEXIT_CRITICAL_ISR(&te_sync_lock);
    return;
  }

  // At an edge, elapsed_us is the duration of the level that just ended.
  if (current_level != 0) {
    state.low_duration_us = static_cast<std::uint32_t>(elapsed_us);
  } else {
    state.high_duration_us = static_cast<std::uint32_t>(elapsed_us);
  }

  if (state.probe_stable) {
    if (current_level == state.active_level) {
      const std::int64_t measured_period_us =
          now_us - state.last_active_edge_us;
      if (measured_period_us >=
              static_cast<std::int64_t>(kTeMinimumPeriodUs) &&
          measured_period_us <=
              static_cast<std::int64_t>(kTeMaximumPeriodUs)) {
        if (state.estimated_period_us == 0) {
          state.estimated_period_us =
              static_cast<std::uint32_t>(measured_period_us);
        } else {
          state.estimated_period_us = static_cast<std::uint32_t>(
              (static_cast<std::uint64_t>(state.estimated_period_us) * 7U +
               static_cast<std::uint32_t>(measured_period_us)) /
              8U);
        }
      }
      state.last_active_edge_us = now_us;
      signal_active_edge = true;
    }
    portEXIT_CRITICAL_ISR(&te_sync_lock);
    if (signal_active_edge && te_active_edge_sem != nullptr) {
      xSemaphoreGiveFromISR(te_active_edge_sem, &task_woken);
      if (task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
      }
    }
    return;
  }

  if (state.high_duration_us == 0 || state.low_duration_us == 0) {
    portEXIT_CRITICAL_ISR(&te_sync_lock);
    return;
  }

  const std::uint32_t short_duration_us =
      state.high_duration_us < state.low_duration_us
          ? state.high_duration_us
          : state.low_duration_us;
  const std::uint32_t long_duration_us =
      state.high_duration_us < state.low_duration_us
          ? state.low_duration_us
          : state.high_duration_us;
  const std::uint32_t candidate_period_us =
      short_duration_us + long_duration_us;
  // 0x35 mode 0 is a pulse rather than a square wave. Requiring the short
  // level to occupy less than one third of a plausible frame rejects noise
  // and prevents choosing an arbitrary edge solely from the boot-time level.
  const bool pulse_shape_is_valid =
      candidate_period_us >= kTeMinimumPeriodUs &&
      candidate_period_us <= kTeMaximumPeriodUs &&
      static_cast<std::uint64_t>(short_duration_us) * 3U <
          candidate_period_us;
  if (!pulse_shape_is_valid) {
    state.estimated_period_us = 0;
    state.estimated_pulse_us = 0;
    state.last_active_edge_us = 0;
    state.candidate_active_level = -1;
    state.stable_periods = 0;
    portEXIT_CRITICAL_ISR(&te_sync_lock);
    return;
  }

  // The edge that ends the short VBlank pulse and returns to the long idle
  // level is the start of active scan.  If high is short this is falling;
  // if low is short this is rising.
  const std::int8_t candidate_active_level =
      state.high_duration_us < state.low_duration_us ? 0 : 1;
  if (current_level != candidate_active_level) {
    portEXIT_CRITICAL_ISR(&te_sync_lock);
    return;
  }

  if (state.candidate_active_level != candidate_active_level) {
    state.candidate_active_level = candidate_active_level;
    state.last_active_edge_us = now_us;
    state.estimated_period_us = 0;
    state.estimated_pulse_us = short_duration_us;
    state.stable_periods = 0;
    portEXIT_CRITICAL_ISR(&te_sync_lock);
    return;
  }

  const std::int64_t measured_period_us = now_us - state.last_active_edge_us;
  state.last_active_edge_us = now_us;
  if (measured_period_us <
          static_cast<std::int64_t>(kTeMinimumPeriodUs) ||
      measured_period_us >
          static_cast<std::int64_t>(kTeMaximumPeriodUs)) {
    state.estimated_period_us = 0;
    state.stable_periods = 0;
    portEXIT_CRITICAL_ISR(&te_sync_lock);
    return;
  }

  const std::uint32_t measured_period =
      static_cast<std::uint32_t>(measured_period_us);
  if (state.estimated_period_us == 0) {
    state.estimated_period_us = measured_period;
    state.estimated_pulse_us = short_duration_us;
    state.stable_periods = 1;
  } else {
    const std::uint32_t period_delta =
        measured_period > state.estimated_period_us
            ? measured_period - state.estimated_period_us
            : state.estimated_period_us - measured_period;
    const std::uint32_t period_tolerance =
        state.estimated_period_us / 20U > kTePeriodToleranceFloorUs
            ? state.estimated_period_us / 20U
            : kTePeriodToleranceFloorUs;
    if (period_delta > period_tolerance) {
      state.estimated_period_us = measured_period;
      state.estimated_pulse_us = short_duration_us;
      state.stable_periods = 1;
    } else {
      state.estimated_period_us = static_cast<std::uint32_t>(
          (static_cast<std::uint64_t>(state.estimated_period_us) * 7U +
           measured_period) /
          8U);
      state.estimated_pulse_us = static_cast<std::uint32_t>(
          (static_cast<std::uint64_t>(state.estimated_pulse_us) * 7U +
           short_duration_us) /
          8U);
      if (state.stable_periods < kTeStablePeriodCount) {
        ++state.stable_periods;
      }
    }
  }

  if (state.stable_periods >= kTeStablePeriodCount) {
    state.probe_stable = true;
    state.active_level = candidate_active_level;
    signal_active_edge = true;
  }
  portEXIT_CRITICAL_ISR(&te_sync_lock);

  if (signal_active_edge && te_active_edge_sem != nullptr) {
    xSemaphoreGiveFromISR(te_active_edge_sem, &task_woken);
    if (task_woken == pdTRUE) {
      portYIELD_FROM_ISR();
    }
  }
}

bool wait_for_fresh_te_active_edge() {
  const std::int64_t deadline_us =
      esp_timer_get_time() + static_cast<std::int64_t>(kTeWaitTimeoutUs);
  while (true) {
    const std::int64_t now_us = esp_timer_get_time();
    const std::int64_t remaining_us = deadline_us - now_us;
    if (remaining_us <= 0) {
      return false;
    }

    const std::uint32_t remaining_ms =
        static_cast<std::uint32_t>((remaining_us + 999) / 1'000);
    TickType_t wait_ticks = pdMS_TO_TICKS(remaining_ms);
    if (wait_ticks == 0) {
      wait_ticks = 1;
    }
    if (xSemaphoreTake(te_active_edge_sem, wait_ticks) != pdTRUE) {
      return false;
    }

    std::int64_t active_edge_us = 0;
    portENTER_CRITICAL(&te_sync_lock);
    active_edge_us = te_sync_state.last_active_edge_us;
    portEXIT_CRITICAL(&te_sync_lock);
    const std::int64_t edge_age_us = esp_timer_get_time() - active_edge_us;
    if (active_edge_us > 0 && edge_age_us >= 0 &&
        edge_age_us <=
            static_cast<std::int64_t>(kTeMaximumAcceptedEdgeAgeUs)) {
      return true;
    }
    // A TE edge can arrive while LVGL is rendering the first strip. If that
    // work took too long, consuming the stale token and waiting for the next
    // active-scan edge is safer than beginning near the end of the frame.
  }
}

void display_te_event(lv_event_t* event) {
  const lv_event_code_t code = lv_event_get_code(event);
  if (code == LV_EVENT_RENDER_START) {
    while (xSemaphoreTake(te_active_edge_sem, 0) == pdTRUE) {
    }

    const std::int64_t now_us = esp_timer_get_time();
    bool report_lock = false;
    bool report_probe_timeout = false;
    bool gate_enabled_for_lock = false;
    std::uint32_t period_us = 0;
    std::uint32_t pulse_us = 0;
    std::uint32_t edge_count = 0;
    std::int8_t active_level = -1;

    portENTER_CRITICAL(&te_sync_lock);
    TeSyncState& state = te_sync_state;
    if (state.panel_revealed && state.probe_stable &&
        !state.probe_reported) {
      state.probe_reported = true;
      period_us = state.estimated_period_us;
      pulse_us = state.estimated_pulse_us;
      active_level = state.active_level;
      // A fixed 25 ms fail-open timeout cannot reliably gate a panel whose
      // own period is almost that long. Keep rendering unsynchronized rather
      // than introducing three guaranteed startup stalls in that case.
      state.gate_enabled =
          !state.gate_permanently_disabled &&
          state.estimated_period_us + 1'000U < kTeWaitTimeoutUs;
      gate_enabled_for_lock = state.gate_enabled;
      report_lock = true;
    }
    if (state.panel_revealed && !state.probe_stable &&
        !state.probe_timeout_reported && state.reveal_us > 0 &&
        now_us - state.reveal_us >=
            static_cast<std::int64_t>(kTeProbeReportDelayUs)) {
      state.probe_timeout_reported = true;
      edge_count = state.edge_count;
      report_probe_timeout = true;
    }
    state.wait_first_flush = state.gate_enabled;
    portEXIT_CRITICAL(&te_sync_lock);

    if (report_lock) {
      ESP_LOGI(kTag,
               "CO5300 TE locked: GPIO%d period=%lu us pulse=%lu us, "
               "active-scan %s edge; first-flush gate %s",
               static_cast<int>(kLcdTeGpio),
               static_cast<unsigned long>(period_us),
               static_cast<unsigned long>(pulse_us),
               active_level != 0 ? "rising" : "falling",
               gate_enabled_for_lock ? "enabled" : "not enabled");
      if (!gate_enabled_for_lock) {
        ESP_LOGW(kTag,
                 "TE period leaves insufficient margin for the 25 ms "
                 "fail-open timeout; keeping unsynchronized partial mode");
      }
    }
    if (report_probe_timeout) {
      ESP_LOGW(kTag,
               "CO5300 TE probe has not found a stable pulse after 2 s "
               "(%lu edges); keeping unsynchronized partial mode",
               static_cast<unsigned long>(edge_count));
    }
    return;
  }

  if (code != LV_EVENT_FLUSH_START) {
    return;
  }

  bool should_wait = false;
  portENTER_CRITICAL(&te_sync_lock);
  should_wait = te_sync_state.wait_first_flush;
  te_sync_state.wait_first_flush = false;
  portEXIT_CRITICAL(&te_sync_lock);
  if (!should_wait) {
    return;
  }

  const std::int64_t wait_start_us = esp_timer_get_time();
  const bool synchronized = wait_for_fresh_te_active_edge();
  display_timing.te_wait_total_us += esp_timer_get_time() - wait_start_us;
  std::uint8_t timeout_count = 0;
  bool gate_disabled = false;
  portENTER_CRITICAL(&te_sync_lock);
  TeSyncState& state = te_sync_state;
  if (synchronized) {
    state.consecutive_timeouts = 0;
  } else if (state.gate_enabled) {
    if (state.consecutive_timeouts < kTeMaximumConsecutiveTimeouts) {
      ++state.consecutive_timeouts;
    }
    timeout_count = state.consecutive_timeouts;
    if (state.consecutive_timeouts >= kTeMaximumConsecutiveTimeouts) {
      state.gate_enabled = false;
      state.gate_permanently_disabled = true;
      state.wait_first_flush = false;
      gate_disabled = true;
    }
  }
  portEXIT_CRITICAL(&te_sync_lock);

  if (!synchronized) {
    if (gate_disabled) {
      ESP_LOGW(kTag,
               "TE wait timed out 3 consecutive times; gate disabled for "
               "this boot and frame sent fail-open");
    } else {
      ESP_LOGW(kTag, "TE wait timeout %u/%u; frame sent fail-open",
               static_cast<unsigned>(timeout_count),
               static_cast<unsigned>(kTeMaximumConsecutiveTimeouts));
    }
  }
}

[[maybe_unused]] esp_err_t initialize_te_probe(lv_display_t* target_display) {
  te_active_edge_sem = xSemaphoreCreateBinary();
  if (te_active_edge_sem == nullptr) {
    ESP_LOGE(kTag, "CO5300 TE semaphore allocation failed");
    return ESP_ERR_NO_MEM;
  }

  gpio_config_t te_config{};
  te_config.pin_bit_mask = 1ULL << kLcdTeGpio;
  te_config.mode = GPIO_MODE_INPUT;
  te_config.pull_up_en = GPIO_PULLUP_DISABLE;
  te_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  te_config.intr_type = GPIO_INTR_ANYEDGE;
  esp_err_t result = gpio_config(&te_config);
  if (result != ESP_OK) {
    vSemaphoreDelete(te_active_edge_sem);
    te_active_edge_sem = nullptr;
    return result;
  }

  result = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
  if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
    vSemaphoreDelete(te_active_edge_sem);
    te_active_edge_sem = nullptr;
    return result;
  }
  result = gpio_isr_handler_add(kLcdTeGpio, lcd_te_gpio_isr, nullptr);
  if (result != ESP_OK) {
    vSemaphoreDelete(te_active_edge_sem);
    te_active_edge_sem = nullptr;
    return result;
  }

  lv_display_add_event_cb(target_display, display_te_event,
                          LV_EVENT_RENDER_START, nullptr);
  lv_display_add_event_cb(target_display, display_te_event,
                          LV_EVENT_FLUSH_START, nullptr);
  lv_display_add_event_cb(target_display, display_timing_event,
                          LV_EVENT_RENDER_START, nullptr);
  lv_display_add_event_cb(target_display, display_timing_event,
                          LV_EVENT_FLUSH_START, nullptr);
  lv_display_add_event_cb(target_display, display_timing_event,
                          LV_EVENT_FLUSH_FINISH, nullptr);
  lv_display_add_event_cb(target_display, display_timing_event,
                          LV_EVENT_RENDER_READY, nullptr);
  lv_display_add_event_cb(target_display, display_timing_event,
                          LV_EVENT_FLUSH_WAIT_START, nullptr);
  lv_display_add_event_cb(target_display, display_timing_event,
                          LV_EVENT_FLUSH_WAIT_FINISH, nullptr);
  ESP_LOGI(kTag,
           "CO5300 TE probe armed on GPIO%d (ANYEDGE; starts after reveal)",
           static_cast<int>(kLcdTeGpio));
  return ESP_OK;
}

[[maybe_unused]] void start_te_probe_after_reveal() {
  while (xSemaphoreTake(te_active_edge_sem, 0) == pdTRUE) {
  }
  const int initial_level = gpio_get_level(kLcdTeGpio);
  const std::int64_t now_us = esp_timer_get_time();
  portENTER_CRITICAL(&te_sync_lock);
  te_sync_state = {};
  te_sync_state.panel_revealed = true;
  te_sync_state.reveal_us = now_us;
  te_sync_state.last_edge_us = now_us;
  te_sync_state.last_level = static_cast<std::int8_t>(initial_level);
  portEXIT_CRITICAL(&te_sync_lock);
  ESP_LOGI(kTag, "CO5300 TE startup probe started: GPIO%d initial level=%d",
           static_cast<int>(kLcdTeGpio), initial_level);
}

esp_err_t initialize_power_control() {
  gpio_config_t button_config{};
  button_config.pin_bit_mask = 1ULL << kPowerButtonGpio;
  button_config.mode = GPIO_MODE_INPUT;
  button_config.pull_up_en = GPIO_PULLUP_ENABLE;
  button_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  button_config.intr_type = GPIO_INTR_DISABLE;
  ESP_RETURN_ON_ERROR(gpio_config(&button_config), kTag,
                      "GPIO18 user key configuration failed");

  gpio_config_t boot_config{};
  boot_config.pin_bit_mask = 1ULL << kBootButtonGpio;
  boot_config.mode = GPIO_MODE_INPUT;
  boot_config.pull_up_en = GPIO_PULLUP_ENABLE;
  boot_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  boot_config.intr_type = GPIO_INTR_DISABLE;
  ESP_RETURN_ON_ERROR(gpio_config(&boot_config), kTag,
                      "BOOT key configuration failed");

  i2c_device_config_t pmic_config{};
  pmic_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  pmic_config.device_address = kAxp2101Address;
  pmic_config.scl_speed_hz = 400'000;
  ESP_RETURN_ON_ERROR(
      i2c_master_bus_add_device(bsp_i2c_get_handle(), &pmic_config, &pmic),
      kTag, "AXP2101 I2C device registration failed");

  ESP_LOGI(kTag, "GPIO18 user key ready (active-low)");
  return ESP_OK;
}
}  // namespace

extern "C" esp_err_t board_port_init(void) {
  if (display != nullptr) {
    return ESP_OK;
  }

  static_assert(MOTO_DISPLAY_WIDTH == BSP_LCD_H_RES,
                "Shared UI width must match the Waveshare panel");
  static_assert(MOTO_DISPLAY_HEIGHT == BSP_LCD_V_RES,
                "Shared UI height must match the Waveshare panel");
  static_assert(MOTO_DISPLAY_BITS_PER_PIXEL == BSP_LCD_BITS_PER_PIXEL,
                "Shared UI and Waveshare panel must both use RGB565");

  // Keep the official BSP's touch driver and pin map plus the supplier's
  // CO5300 command values. Panel creation itself uses the public esp_lcd APIs
  // so DISPON can be deferred. Explicit PSRAM draw buffers feed direct DMA
  // without allocating a bounce buffer for each transaction.
  const esp_lv_adapter_config_t adapter_config =
      ESP_LV_ADAPTER_DEFAULT_CONFIG();
  ESP_RETURN_ON_ERROR(esp_lv_adapter_init(&adapter_config), kTag,
                      "LVGL adapter initialization failed");

  // The built-in 64 KiB LVGL heap is separate from the adapter's PSRAM draw
  // buffers. A map-sized strip lets the boot Logo's opacity/transform layer
  // exceed that tiny heap: LVGL keeps retrying the blocked layer and starves
  // IDLE0 before BLE can start. Keep the internal pool for small allocations
  // and add a bounded PSRAM pool for complete intermediate layers.
  static_assert(LV_USE_STDLIB_MALLOC == LV_STDLIB_BUILTIN,
                "Review the LVGL pool setup after changing allocators");
  lvgl_extra_pool_storage = heap_caps_malloc(
      kLvglExtraPoolBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (lvgl_extra_pool_storage == nullptr ||
      lv_mem_add_pool(lvgl_extra_pool_storage, kLvglExtraPoolBytes) == nullptr) {
    heap_caps_free(lvgl_extra_pool_storage);
    lvgl_extra_pool_storage = nullptr;
    ESP_LOGE(kTag, "Could not reserve LVGL intermediate-layer PSRAM pool");
    return ESP_ERR_NO_MEM;
  }
  ESP_LOGI(kTag, "LVGL intermediate-layer pool: %u KiB PSRAM",
           static_cast<unsigned>(kLvglExtraPoolBytes / 1024U));

  esp_lcd_panel_io_handle_t panel_io = nullptr;
  ESP_RETURN_ON_ERROR(
      initialize_hidden_panel(MOTO_DISPLAY_WIDTH * kDrawBufferHeight *
                                  MOTO_DISPLAY_BITS_PER_PIXEL / 8,
                              &panel, &panel_io),
      kTag, "hidden Waveshare CO5300 initialization failed");

  esp_lv_adapter_display_config_t display_config =
      ESP_LV_ADAPTER_DISPLAY_SPI_WITH_PSRAM_DEFAULT_CONFIG(
          panel, panel_io, MOTO_DISPLAY_WIDTH, MOTO_DISPLAY_HEIGHT,
          ESP_LV_ADAPTER_ROTATE_0);
  display_config.profile.buffer_height = kDrawBufferHeight;
  display_config.profile.use_psram = kDrawBuffersUsePsram;
  display_config.profile.require_double_buffer = true;
  display = esp_lv_adapter_register_display(&display_config);
  if (display == nullptr) {
    ESP_LOGE(kTag, "LVGL could not register the CO5300 display");
    return ESP_FAIL;
  }
  ESP_RETURN_ON_ERROR(
      esp_lv_adapter_set_area_rounder_cb(display, round_draw_area, nullptr),
      kTag, "CO5300 area rounder registration failed");
  // This board does not route the CO5300 TE signal to the ESP32. Do not reuse
  // GPIO13: it belongs to the RTC interrupt on the 2.16 schematic.

  ESP_RETURN_ON_ERROR(bsp_i2c_init(), kTag,
                      "shared I2C bus initialization failed");
  // Initialize fields in declaration order: the component macro uses a C
  // designated-initializer order that GCC rejects in this C++ translation unit.
  const esp_lcd_panel_io_i2c_config_t touch_io_config = {
      .dev_addr = ESP_LCD_TOUCH_IO_I2C_CST9220_ADDRESS,
      .on_color_trans_done = nullptr,
      .user_ctx = nullptr,
      .control_phase_bytes = 1,
      .dc_bit_offset = 0,
      .lcd_cmd_bits = 8,
      .lcd_param_bits = 8,
      .flags = {.dc_low_on_data = 0, .disable_control_phase = 1},
      .scl_speed_hz = CONFIG_BSP_I2C_CLK_SPEED_HZ,
      .transaction_timeout_ms = 0,
  };
  ESP_RETURN_ON_ERROR(
      esp_lcd_new_panel_io_i2c(bsp_i2c_get_handle(), &touch_io_config,
                               &touch_panel_io),
      kTag, "CST9220 touch I2C setup failed");
  const esp_lcd_touch_config_t controller_config = {
      .x_max = MOTO_DISPLAY_WIDTH,
      .y_max = MOTO_DISPLAY_HEIGHT,
      .rst_gpio_num = GPIO_NUM_40,
      .int_gpio_num = GPIO_NUM_11,
      .levels = {.reset = 0, .interrupt = 0},
      .flags = {.swap_xy = 1, .mirror_x = 0, .mirror_y = 1},
  };
  esp_lcd_touch_handle_t touch = nullptr;
  ESP_RETURN_ON_ERROR(
      esp_lcd_touch_new_i2c_cst9220(touch_panel_io, &controller_config, &touch),
      kTag, "CST9220 initialization failed");
  const esp_lv_adapter_touch_config_t adapter_touch_config =
      ESP_LV_ADAPTER_TOUCH_DEFAULT_CONFIG(display, touch);
  if (esp_lv_adapter_register_touch(&adapter_touch_config) == nullptr) {
    ESP_LOGE(kTag, "LVGL could not register the CST9220 touch device");
    return ESP_FAIL;
  }
  ESP_RETURN_ON_ERROR(initialize_power_control(), kTag,
                      "PWR/AXP2101 control initialization failed");

  // The PWR key is wired to the PMIC, so its press only reaches the firmware
  // through these status registers. Log the enable mask once: with it we can
  // tell "the key never reported" apart from "the key reported but the bit is
  // masked", which are two very different fixes.
  {
    std::uint8_t enable[3] = {};
    if (read_pmic_register(0x40, &enable[0]) == ESP_OK &&
        read_pmic_register(0x41, &enable[1]) == ESP_OK &&
        read_pmic_register(0x42, &enable[2]) == ESP_OK) {
      ESP_LOGI(kTag, "PMU IRQ enables: 40=%02x 41=%02x 42=%02x",
               enable[0], enable[1], enable[2]);
    }
  }

  // Establish black before the display worker is allowed to perform its first
  // refresh.  The app replaces this with the animated boot scene while the
  // physical panel is still hidden.
  lv_obj_t* const startup_screen = lv_display_get_screen_active(display);
  lv_obj_set_style_bg_color(startup_screen, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(startup_screen, LV_OPA_COVER, 0);
  lv_obj_remove_flag(startup_screen, LV_OBJ_FLAG_SCROLLABLE);
  ESP_RETURN_ON_ERROR(esp_lv_adapter_start(), kTag,
                      "LVGL worker task could not start");

  if (lv_display_get_horizontal_resolution(display) != MOTO_DISPLAY_WIDTH ||
      lv_display_get_vertical_resolution(display) != MOTO_DISPLAY_HEIGHT ||
      lv_display_get_color_format(display) != LV_COLOR_FORMAT_RGB565) {
    ESP_LOGE(kTag, "BSP display is %ldx%ld format=%d, expected %dx%d RGB565",
             static_cast<long>(lv_display_get_horizontal_resolution(display)),
             static_cast<long>(lv_display_get_vertical_resolution(display)),
             static_cast<int>(lv_display_get_color_format(display)),
             MOTO_DISPLAY_WIDTH, MOTO_DISPLAY_HEIGHT);
    display = nullptr;
    return ESP_ERR_INVALID_SIZE;
  }

  ESP_LOGI(
      kTag,
      "CO5300 + CST9220 ready at %dx%d RGB565 (dual %s %u-row buffers, "
      "direct DMA, QSPI queue depth %d)",
      MOTO_DISPLAY_WIDTH, MOTO_DISPLAY_HEIGHT,
      kDrawBuffersUsePsram ? "PSRAM" : "internal SRAM",
      static_cast<unsigned>(kDrawBufferHeight),
      kDisplayQueueDepth);

  // The 2.16 board carries a PCF85063ATL so the clock page keeps running when
  // the phone is away. A missing part is not fatal: the page falls back to the
  // phone alone.
  if (rtc.begin()) {
    ESP_LOGI(kTag, "PCF85063 RTC ready");
  } else {
    ESP_LOGW(kTag, "PCF85063 RTC unavailable; clock page is phone-only");
  }
  return ESP_OK;
}

extern "C" bool board_port_rtc_read(int* year, int* month, int* day, int* hour,
                                    int* minute, int* second, int* weekday) {
  moto::rtc_clock::Time time{};
  if (!rtc.read(time)) {
    return false;
  }
  if (year != nullptr) *year = time.year;
  if (month != nullptr) *month = time.month;
  if (day != nullptr) *day = time.day;
  if (hour != nullptr) *hour = time.hour;
  if (minute != nullptr) *minute = time.minute;
  if (second != nullptr) *second = time.second;
  if (weekday != nullptr) *weekday = time.weekday;
  return true;
}

extern "C" void board_port_rtc_sync(int year, int month, int day, int hour,
                                    int minute, int second, int weekday) {
  moto::rtc_clock::Time time{};
  time.year = static_cast<std::uint16_t>(year);
  time.month = static_cast<std::uint8_t>(month);
  time.day = static_cast<std::uint8_t>(day);
  time.hour = static_cast<std::uint8_t>(hour);
  time.minute = static_cast<std::uint8_t>(minute);
  time.second = static_cast<std::uint8_t>(second);
  time.weekday = static_cast<std::uint8_t>(weekday);
  rtc.sync_from_phone(time);
}

extern "C" lv_display_t* board_port_get_display(void) { return display; }

extern "C" esp_err_t board_port_reveal_display(void) {
  if (panel == nullptr || display == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  if (display_revealed) {
    return ESP_OK;
  }

  // The first LVGL refresh has already queued the black boot background.  The
  // panel command and brightness write share that QSPI queue, so the visible
  // enable cannot overtake the framebuffer transfer.
  ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(panel, true), kTag,
                      "could not reveal AMOLED");
  ESP_RETURN_ON_ERROR(esp_lcd_panel_co5300_set_brightness(panel, 100), kTag,
                      "could not restore AMOLED brightness");
  display_revealed = true;
  return ESP_OK;
}

extern "C" bool board_port_lock(uint32_t timeout_ms) {
  if (display == nullptr) {
    return false;
  }
  return bsp_display_lock(timeout_ms) == ESP_OK;
}

extern "C" void board_port_unlock(void) {
  if (display != nullptr) {
    bsp_display_unlock();
  }
}

extern "C" esp_err_t board_port_set_brightness(uint8_t percent) {
  if (panel == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  const uint8_t level = percent > 100 ? 100 : percent;
  return esp_lcd_panel_co5300_set_brightness(panel, level);
}

extern "C" bool board_port_battery(moto_battery_t* out) {
  if (pmic == nullptr || out == nullptr) {
    return false;
  }
  // AXP2101: 0x00 status 1, 0x34/0x35 battery voltage (14-bit, 1 mV/LSB),
  // 0xA4 fuel-gauge percentage. The percentages the gauge reports with no
  // battery fitted are meaningless, so presence is decided by voltage.
  std::uint8_t status = 0;
  std::uint8_t voltage[2] = {};
  std::uint8_t gauge = 0;
  if (read_pmic_register(0x00, &status) != ESP_OK ||
      read_pmic_register(0x34, &voltage[0]) != ESP_OK ||
      read_pmic_register(0x35, &voltage[1]) != ESP_OK ||
      read_pmic_register(0xA4, &gauge) != ESP_OK) {
    return false;
  }
  const std::uint16_t millivolts = static_cast<std::uint16_t>(
      ((voltage[0] & 0x3FU) << 8U) | voltage[1]);
  out->millivolts = millivolts;
  // Bit 3 of status 1 is the PMIC's VBUS-good flag.
  out->charging = (status & 0x08U) != 0U ? 1U : 0U;
  out->present = millivolts >= 3000U ? 1U : 0U;
  out->percent = out->present != 0U
                     ? static_cast<std::uint8_t>(gauge > 100U ? 100U : gauge)
                     : 0U;
  return true;
}

extern "C" bool board_port_pmic_take_irq(uint8_t status[3]) {
  if (pmic == nullptr || status == nullptr) {
    return false;
  }
  bool any = false;
  for (std::uint8_t index = 0; index < 3; ++index) {
    const std::uint8_t reg = static_cast<std::uint8_t>(0x48U + index);
    std::uint8_t value = 0;
    if (read_pmic_register(reg, &value) != ESP_OK) {
      return false;
    }
    status[index] = value;
    if (value != 0) {
      any = true;
      // The AXP2101 status bits are write-one-to-clear.
      write_pmic_register(reg, value);
    }
  }
  return any;
}

extern "C" bool board_port_boot_button_pressed(void) {
  return gpio_get_level(kBootButtonGpio) == 0;
}

extern "C" bool board_port_power_button_pressed(void) {
  // GPIO18 is pulled high and the user key connects it to ground.
  return gpio_get_level(kPowerButtonGpio) == 0;
}

extern "C" esp_err_t board_port_power_off(void) {
  if (pmic == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  if (panel != nullptr) {
    const esp_err_t display_result = esp_lcd_panel_disp_on_off(panel, false);
    if (display_result != ESP_OK) {
      ESP_LOGW(kTag, "could not blank AMOLED before power-off: %s",
               esp_err_to_name(display_result));
    }
  }

  std::uint8_t common_config = 0;
  ESP_RETURN_ON_ERROR(
      read_pmic_register(kAxp2101CommonConfigRegister, &common_config), kTag,
      "AXP2101 common config read failed");
  return write_pmic_register(
      kAxp2101CommonConfigRegister,
      static_cast<std::uint8_t>(common_config |
                                kAxp2101SoftwarePowerOffMask));
}
