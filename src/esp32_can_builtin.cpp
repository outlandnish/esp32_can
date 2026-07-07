/*
  ESP32_CAN.cpp – Library for ESP32 built-in TWAI/CAN module

  Three-tier build:
    TIER_NEW  (IDF ≥ 5.5) – esp_driver_twai: twai_new_node_onchip, callback-based async RX, CAN-FD capable chips
    TIER_V2   (IDF 5.2–5.4) – legacy driver _v2 handle API
    TIER_V1   (IDF < 5.2)   – legacy single-controller API

  Backwards-compatible with the CAN_COMMON / can_common interface so
  existing sketches compile unchanged.

  Original author: Collin Kidder
  twai_v2 updates: outlandnish / TechOverflow
  IDF 5.5+ / CAN-FD update: see changelog
*/

#include "Arduino.h"
#include "esp32_can_builtin.h"
#include <algorithm>   // for std::min

static inline uint16_t saturate_u16(uint32_t v) {
    return v > 0xFFFF ? 0xFFFF : (uint16_t)v;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Speed table (TIER_V1 / TIER_V2 only)
//  TIER_NEW passes the integer bitrate directly to twai_onchip_node_config_t
// ─────────────────────────────────────────────────────────────────────────────
#if !defined(TWAI_TIER_NEW)

static const VALID_TIMING valid_timings[] =
{
    {TWAI_TIMING_CONFIG_1MBITS(),   1000000},
    {TWAI_TIMING_CONFIG_500KBITS(),  500000},
    {TWAI_TIMING_CONFIG_250KBITS(),  250000},
    {TWAI_TIMING_CONFIG_125KBITS(),  125000},
    {TWAI_TIMING_CONFIG_800KBITS(),  800000},
    {TWAI_TIMING_CONFIG_100KBITS(),  100000},
    {TWAI_TIMING_CONFIG_50KBITS(),    50000},
    {TWAI_TIMING_CONFIG_25KBITS(),    25000},
    // Custom entries – not fully validated on all silicon
    // (brp multiples of 2 up to 128, multiples of 4 up to 256;
    //  TSEG1 1–16, TSEG2 1–8; default clock 80 MHz)
    {{.brp=100, .tseg_1=7,  .tseg_2=2, .sjw=3, .triple_sampling=false}, 80000},
    {{.brp=120, .tseg_1=15, .tseg_2=4, .sjw=3, .triple_sampling=false}, 33333},
    // ECO2 / ESP32-S3 only:
    {{.brp=200, .tseg_1=15, .tseg_2=4, .sjw=3, .triple_sampling=false}, 20000},
    {TWAI_TIMING_CONFIG_25KBITS(), 0}   // terminator – speed == 0 stops search
};

#endif // !TWAI_TIER_NEW

// ─────────────────────────────────────────────────────────────────────────────
//  Constructors
// ─────────────────────────────────────────────────────────────────────────────

ESP32CAN::ESP32CAN(gpio_num_t rxPin, gpio_num_t txPin, uint8_t busNumber)
    : CAN_COMMON(BI_NUM_FILTERS),
      _rxPin(rxPin), _txPin(txPin), _busNum(busNumber)
{
    rxBufferSize = BI_RX_BUFFER_SIZE;

#if defined(TWAI_TIER_NEW)
    // nothing extra – config built in enable()
#else
    twai_general_cfg = TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, TWAI_MODE_NORMAL);
    twai_general_cfg.tx_queue_len = BI_TX_BUFFER_SIZE;
    twai_general_cfg.rx_queue_len = 6;
    twai_speed_cfg   = TWAI_TIMING_CONFIG_500KBITS();
    twai_filters_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  #if defined(TWAI_TIER_V2)
    twai_general_cfg.controller_id = busNumber;
  #endif
#endif

    for (int i = 0; i < BI_NUM_FILTERS; i++) {
        filters[i] = {0, 0, false, false};
    }
    readyForTraffic = false;
    cyclesSinceTraffic = 0;
}

ESP32CAN::ESP32CAN() : CAN_COMMON(BI_NUM_FILTERS)
{
    _rxPin  = GPIO_NUM_16;
    _txPin  = GPIO_NUM_17;
    _busNum = 0;
    rxBufferSize = BI_RX_BUFFER_SIZE;

#if !defined(TWAI_TIER_NEW)
    twai_general_cfg = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_17, GPIO_NUM_16, TWAI_MODE_NORMAL);
    twai_general_cfg.tx_queue_len = BI_TX_BUFFER_SIZE;
    twai_general_cfg.rx_queue_len = 6;
    twai_speed_cfg   = TWAI_TIMING_CONFIG_500KBITS();
    twai_filters_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
#endif

    for (int i = 0; i < BI_NUM_FILTERS; i++) {
        filters[i] = {0, 0, false, false};
    }
    readyForTraffic    = false;
    cyclesSinceTraffic = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Pin / buffer setters (must be called before init)
// ─────────────────────────────────────────────────────────────────────────────

void ESP32CAN::setCANPins(gpio_num_t rxPin, gpio_num_t txPin)
{
    _rxPin = rxPin;
    _txPin = txPin;
#if !defined(TWAI_TIER_NEW)
    twai_general_cfg.rx_io = rxPin;
    twai_general_cfg.tx_io = txPin;
#endif
}

void ESP32CAN::setRXBufferSize(int newSize)  { rxBufferSize = newSize; }

void ESP32CAN::setTXBufferSize(int newSize)
{
#if !defined(TWAI_TIER_NEW)
    twai_general_cfg.tx_queue_len = newSize;
#endif
    // TIER_NEW: queue depth is set in twai_onchip_node_config_t at enable() time;
    // store in a member if you want runtime changes – add uint32_t _txQueueDepth if needed
}

// ─────────────────────────────────────────────────────────────────────────────
//  Filter helpers
// ─────────────────────────────────────────────────────────────────────────────

int ESP32CAN::_setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended)
{
    if (mailbox < BI_NUM_FILTERS) {
        filters[mailbox].id         = id & mask;
        filters[mailbox].mask       = mask;
        filters[mailbox].extended   = extended;
        filters[mailbox].configured = true;
        return mailbox;
    }
    return -1;
}

int ESP32CAN::_setFilter(uint32_t id, uint32_t mask, bool extended)
{
    for (int i = 0; i < BI_NUM_FILTERS; i++) {
        if (!filters[i].configured) {
            return _setFilterSpecific(i, id, mask, extended);
        }
    }
    if (debuggingMode) Serial.println("Could not set filter!");
    return -1;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Internal frame processing
//  Common to all tiers – applies the 32-slot software filter table then
//  routes accepted frames to callbacks or the rx_queue.
// ─────────────────────────────────────────────────────────────────────────────

bool ESP32CAN::processFrame(CAN_FRAME &msg)
{
    CANListener *thisListener;
    cyclesSinceTraffic = 0;

    for (int i = 0; i < BI_NUM_FILTERS; i++) {
        if (!filters[i].configured) continue;
        if ((msg.id & filters[i].mask) == filters[i].id &&
             filters[i].extended == msg.extended)
        {
            if (cbCANFrame[i]) {
                msg.fid = i;
                xQueueSend(callbackQueue, &msg, 0);
                return true;
            } else if (cbGeneral) {
                msg.fid = 0xFF;
                xQueueSend(callbackQueue, &msg, 0);
                return true;
            } else {
                for (int lp = 0; lp < SIZE_LISTENERS; lp++) {
                    thisListener = listener[lp];
                    if (thisListener != nullptr) {
                        if (thisListener->isCallbackActive(i)) {
                            msg.fid = 0x80000000ul | (lp << 24ul) | i;
                            xQueueSend(callbackQueue, &msg, 0);
                            return true;
                        } else if (thisListener->isCallbackActive(numFilters)) {
                            msg.fid = 0x80000000ul | (lp << 24ul) | 0xFF;
                            xQueueSend(callbackQueue, &msg, 0);
                            return true;
                        }
                    }
                }
            }
            xQueueSend(rx_queue, &msg, 0);
            if (debuggingMode) Serial.write('_');
            return true;
        }
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Callback dispatcher task (all tiers)
//  Reads from callbackQueue and fires registered C or object callbacks.
// ─────────────────────────────────────────────────────────────────────────────

void ESP32CAN::sendCallback(CAN_FRAME *frame)
{
    int mb  = frame->fid & 0xFF;
    if (mb == 0xFF) mb = -1;

    if (frame->fid & 0x80000000ul) {
        int idx = (frame->fid >> 24) & 0x7F;
        listener[idx]->gotFrame(frame, mb);
    } else {
        if (mb > -1) (*cbCANFrame[mb])(frame);
        else         (*cbGeneral)(frame);
    }
}

void ESP32CAN::task_CAN(void *pvParameters)
{
    ESP32CAN *espCan = static_cast<ESP32CAN *>(pvParameters);
    CAN_FRAME rxFrame;

    vTaskDelay(pdMS_TO_TICKS(100));

    while (1) {
        if (uxQueueMessagesWaiting(espCan->callbackQueue)) {
            if (xQueueReceive(espCan->callbackQueue, &rxFrame, portMAX_DELAY) == pdTRUE) {
                espCan->sendCallback(&rxFrame);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(4));
        }
#if defined(CONFIG_FREERTOS_UNICORE)
        vTaskDelay(pdMS_TO_TICKS(6));
#endif
    }
    vTaskDelete(nullptr);
}

// ═════════════════════════════════════════════════════════════════════════════
//  TIER_NEW  (IDF ≥ 5.5)  –  esp_driver_twai
// ═════════════════════════════════════════════════════════════════════════════
#if defined(TWAI_TIER_NEW)

// ── ISR callbacks ─────────────────────────────────────────────────────────────

bool IRAM_ATTR ESP32CAN::onRxDone(twai_node_handle_t handle,
                                   const twai_rx_done_event_data_t *edata,
                                   void *user_ctx)
{
    ESP32CAN *espCan = static_cast<ESP32CAN *>(user_ctx);
    if (!espCan->readyForTraffic) return false;

    // The new driver uses pointer-based frames; we pull the frame right here
    // in ISR context using twai_node_receive_from_isr.
    uint8_t buf[64] = {};           // CAN-FD can carry up to 64 bytes
    twai_frame_t raw = {
        .buffer     = buf,
        .buffer_len = sizeof(buf),
    };

    if (twai_node_receive_from_isr(handle, &raw) != ESP_OK) return false;

    // The received payload length comes from the DLC code (buffer_len is only
    // the capacity we passed in). twaifd_dlc2len maps FD DLC codes to bytes.
    uint16_t rxLen = twaifd_dlc2len(raw.header.dlc);

    CAN_FRAME msg;
    msg.id       = raw.header.id;
    msg.extended = raw.header.ide;
    msg.rtr      = raw.header.rtr;
    msg.length = (rxLen <= 8) ? rxLen : 8;
    for (int i = 0; i < 8; i++) msg.data.byte[i] = (i < (int)rxLen) ? buf[i] : 0;

    espCan->cyclesSinceTraffic = 0;
    espCan->trafficEverSeen = true;

    BaseType_t woken = pdFALSE;

#if defined(SOC_TWAI_SUPPORT_FD)
    // Feed the FD queue in parallel for any FD frame, preserving the full
    // payload and the fdf/brs flags that the classic downcast above discards.
    if (raw.header.fdf && espCan->rx_queue_fd) {
        CAN_FRAME_FD fd;
        fd.id       = raw.header.id;
        fd.extended = raw.header.ide;
        fd.fdMode   = 1;
        fd.rrs      = raw.header.brs ? 1 : 0;   // carry BRS in rrs (see header)
        fd.length   = (rxLen <= 64) ? rxLen : 64;
        for (int i = 0; i < 64; i++)
            fd.data.uint8[i] = (i < (int)rxLen) ? buf[i] : 0;
        xQueueSendFromISR(espCan->rx_queue_fd, &fd, &woken);
    }
#endif

    for (int i = 0; i < BI_NUM_FILTERS; i++) {
        if (!espCan->filters[i].configured) continue;
        if ((msg.id & espCan->filters[i].mask) == espCan->filters[i].id &&
             espCan->filters[i].extended == msg.extended)
        {
            if (espCan->cbCANFrame[i] || espCan->cbGeneral) {
                msg.fid = espCan->cbCANFrame[i] ? i : 0xFF;
                xQueueSendFromISR(espCan->callbackQueue, &msg, &woken);
            } else {
                xQueueSendFromISR(espCan->rx_queue, &msg, &woken);
            }
            return (woken == pdTRUE);
        }
    }
    return (woken == pdTRUE);
}

bool IRAM_ATTR ESP32CAN::onTxDone(twai_node_handle_t /*handle*/,
                                   const twai_tx_done_event_data_t * /*edata*/,
                                   void * /*user_ctx*/)
{
    return false; // no higher-priority task woken
}

bool IRAM_ATTR ESP32CAN::onError(twai_node_handle_t handle,
                                  const twai_error_event_data_t *edata,
                                  void *user_ctx)
{
    // The IDF 5.5 error callback fires on every bus error event — most
    // of which are transient TX-side bumps (no ACK, arbitration loss,
    // form error) that the controller absorbs on its own as TEC ebbs
    // back to zero with each successful TX. Flipping readyForTraffic
    // here would deadlock callers that gate on the flag: one error →
    // flag false → caller stops feeding sendFrame → TEC never drains
    // → flag stays false forever, even though hardware state is still
    // ERROR_ACTIVE the entire time.
    //
    // readyForTraffic is now managed exclusively by onStateChange,
    // which sees the actual transitions (active → passive → bus-off
    // and back). This callback is intentionally a no-op; if you want
    // to count errors for diagnostics, do it here, but do NOT touch
    // the ready flag.
    (void)handle;
    (void)edata;
    (void)user_ctx;
    return false;
}

// ── Watchdog task (TIER_NEW) ──────────────────────────────────────────────────
// Polls bus state every 200 ms; if bus-off is detected, disables and re-enables
// the node so the hardware can recover.

void ESP32CAN::CAN_WatchDog_Builtin(void *pvParameters)
{
    ESP32CAN *espCan = static_cast<ESP32CAN *>(pvParameters);
    const TickType_t xDelay = pdMS_TO_TICKS(200);

    for (;;) {
        vTaskDelay(xDelay);
        espCan->cyclesSinceTraffic++;

        if (espCan->node_handle == nullptr) continue;

        bool needsRecovery = false;

        // State-based failures detected by onStateChange ISR callback
        if (espCan->_errorPassive) {
            espCan->_errorPassive = false;  // clear the flag before checking

            twai_node_status_t status;
            twai_node_record_t record;
            if (twai_node_get_info(espCan->node_handle, &status, &record) == ESP_OK) {
                if (status.state == TWAI_ERROR_BUS_OFF ||
                    status.state == TWAI_ERROR_PASSIVE) {
                    needsRecovery = true;
                }
            }
        }

        // Stall detection — no frames received for > 2 seconds
        if (espCan->cyclesSinceTraffic > 10 &&
            espCan->readyForTraffic         &&
            espCan->trafficEverSeen)
        {
            needsRecovery = true;
        }

        if (needsRecovery) {
            espCan->cyclesSinceTraffic = 0;
            espCan->_errorPassive      = false;
            espCan->readyForTraffic    = false;
            twai_node_disable(espCan->node_handle);
            if (twai_node_enable(espCan->node_handle) == ESP_OK) {
                espCan->readyForTraffic = true;
            }
        }
    }
}

// ── enable / disable (TIER_NEW) ───────────────────────────────────────────────

void ESP32CAN::enable()
{
    if (node_handle != nullptr) {
        // Already installed – just start it
        if (twai_node_enable(node_handle) == ESP_OK) {
            readyForTraffic = true;
        }
        return;
    }

    // Determine mode flags
    bool isListenOnly = listenOnly;
    bool isSelfTest   = noACK;

    twai_onchip_node_config_t node_cfg = {
        .io_cfg = {
            .tx = _txPin,
            .rx = _rxPin,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
            // .clkout_io = GPIO_NUM_NC,
            // .bus_off_io = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = _baudrate,
        },
        .fail_retry_cnt = -1,
        .tx_queue_depth = BI_TX_BUFFER_SIZE,
        .flags = {
            .enable_self_test   = noACK     ? 1u : 0u,
            .enable_listen_only = listenOnly ? 1u : 0u,
        },
    };

#if SOC_TWAI_SUPPORT_FD
    // CAN-FD data phase – only populated when caller set a data baudrate
    if (_dataBaudrate > 0) {
        node_cfg.data_timing.bitrate = _dataBaudrate;
    }
#endif

    if (twai_new_node_onchip(&node_cfg, &node_handle) != ESP_OK) {
        if (debuggingMode) printf("[TWAI] Failed to create node (bus %d)\n", _busNum);
        node_handle = nullptr;
        return;
    }
    if (debuggingMode) printf("[TWAI] Node created (bus %d, %lu bps)\n", _busNum, _baudrate);

    // Register callbacks
    twai_event_callbacks_t cbs = {};
    cbs.on_rx_done = ESP32CAN::onRxDone;
    cbs.on_tx_done = ESP32CAN::onTxDone;
    cbs.on_error   = ESP32CAN::onError;
    cbs.on_state_change = ESP32CAN::onStateChange;
    twai_node_register_event_callbacks(node_handle, &cbs, this);

    // Create queues
    callbackQueue = xQueueCreate(16,          sizeof(CAN_FRAME));
    rx_queue      = xQueueCreate(rxBufferSize, sizeof(CAN_FRAME));
#if defined(SOC_TWAI_SUPPORT_FD)
    rx_queue_fd   = xQueueCreate(rxBufferSize, sizeof(CAN_FRAME_FD));
#endif

    // Callback dispatcher task
    char taskName[24];
    snprintf(taskName, sizeof(taskName), "CAN_RX_CAN%d", _busNum);
    xTaskCreate(ESP32CAN::task_CAN, taskName, 8192, this, 15, &task_CAN_handler);

    if (twai_node_enable(node_handle) == ESP_OK) {
        if (debuggingMode) printf("[TWAI] Node enabled (bus %d)\n", _busNum);
        readyForTraffic = true;
    } else {
        if (debuggingMode) printf("[TWAI] Failed to enable node (bus %d)\n", _busNum);
    }
}

void ESP32CAN::disable()
{
    readyForTraffic = false;

    if (node_handle != nullptr) {
        twai_node_disable(node_handle);
        twai_node_delete(node_handle);
        node_handle = nullptr;
    }

    if (task_CAN_handler != nullptr) {
        vTaskDelete(task_CAN_handler);
        task_CAN_handler = nullptr;
    }

    if (rx_queue) {
        vQueueDelete(rx_queue);
        rx_queue = nullptr;
    }
#if defined(SOC_TWAI_SUPPORT_FD)
    if (rx_queue_fd) {
        vQueueDelete(rx_queue_fd);
        rx_queue_fd = nullptr;
    }
#endif
    if (callbackQueue) {
        vQueueDelete(callbackQueue);
        callbackQueue = nullptr;
    }
}

// ── _init  (TIER_NEW) ─────────────────────────────────────────────────────────

void ESP32CAN::_init()
{
    if (debuggingMode) Serial.println("[TWAI] _init (TIER_NEW)");

    for (int i = 0; i < BI_NUM_FILTERS; i++) filters[i] = {0, 0, false, false};

    if (CAN_WatchDog_Builtin_handler == nullptr) {
        char wdName[28];
        snprintf(wdName, sizeof(wdName), "CAN_WD_BI_CAN%d", _busNum);
#if defined(CONFIG_FREERTOS_UNICORE)
        xTaskCreate(&CAN_WatchDog_Builtin, wdName, 2048, this, 10,
                    &CAN_WatchDog_Builtin_handler);
#else
        xTaskCreatePinnedToCore(&CAN_WatchDog_Builtin, wdName, 2048, this, 10,
                                &CAN_WatchDog_Builtin_handler, 1);
#endif
    }
}

// ── init / set_baudrate (TIER_NEW) ────────────────────────────────────────────

uint32_t ESP32CAN::init(uint32_t ul_baudrate)
{
    _baudrate = ul_baudrate;
    _init();
    enable();
    return ul_baudrate;
}

uint32_t ESP32CAN::set_baudrate(uint32_t ul_baudrate)
{
    _baudrate = ul_baudrate;
    disable();
    enable();
    return ul_baudrate;
}

uint32_t ESP32CAN::beginAutoSpeed()
{
    // Save and restore mode after probing
    bool savedListen = listenOnly;
    listenOnly = true;
    _init();
    readyForTraffic = false;

    // Probe common speeds from fast to slow
    const uint32_t speeds[] = {1000000, 800000, 500000, 250000, 125000,
                                100000, 80000, 50000, 33333, 25000, 20000, 0};
    for (int i = 0; speeds[i] != 0; i++) {
        _baudrate = speeds[i];
        disable();
        Serial.print("Trying speed "); Serial.print(speeds[i]);
        enable();
        delay(600);
        if (cyclesSinceTraffic < 2) {
            disable();
            listenOnly = savedListen;
            enable();
            Serial.println(" SUCCESS!");
            return speeds[i];
        }
        Serial.println(" FAILED.");
    }

    Serial.println("None of the tested CAN speeds worked!");
    disable();
    listenOnly = savedListen;
    return 0;
}

// ── Mode setters (TIER_NEW) ───────────────────────────────────────────────────

void ESP32CAN::setListenOnlyMode(bool state)
{
    listenOnly = state;
    disable();
    enable();
}

void ESP32CAN::setNoACKMode(bool state)
{
    noACK = state;
    disable();
    enable();
}

// ── sendFrame (TIER_NEW) ──────────────────────────────────────────────────────

bool ESP32CAN::sendFrame(CAN_FRAME &txFrame)
{
    if (node_handle == nullptr) return false;

    // Frame buffer must remain valid until on_tx_done fires.
    // We use a local static ring-buffer of 8 slots to avoid heap alloc
    // on every send while still being safe for back-to-back calls.
    // For FD frames (length > 8) the caller must ensure the frame lives
    // long enough – or enlarge the ring buffer below.
    static uint8_t  txBufs[8][64];
    static int      txBufIdx = 0;

    uint8_t *buf = txBufs[txBufIdx & 7];
    txBufIdx++;

    uint8_t len = txFrame.length <= 64 ? txFrame.length : 64;
    for (int i = 0; i < len; i++) buf[i] = txFrame.data.byte[i];

    static twai_frame_t raw[8]; // parallel slot
    int slot = (txBufIdx - 1) & 7;
    raw[slot].header.id  = txFrame.id;
    raw[slot].header.ide = txFrame.extended ? 1 : 0;
    raw[slot].header.rtr = txFrame.rtr      ? 1 : 0;
#if SOC_TWAI_SUPPORT_FD
    raw[slot].header.fdf = 0;   // set to 1 externally for FD frames if desired
    raw[slot].header.brs = 0;
#endif
    raw[slot].buffer     = buf;
    raw[slot].buffer_len = len;

    esp_err_t res = twai_node_transmit(node_handle, &raw[slot], 0 /*no wait*/);

    switch (res) {
        case ESP_OK:            if (debuggingMode) Serial.write('<'); break;
        case ESP_ERR_TIMEOUT:   if (debuggingMode) Serial.write('T'); break;
        default:                if (debuggingMode) Serial.write('!'); break;
    }
    return (res == ESP_OK);
}

#if defined(SOC_TWAI_SUPPORT_FD)
bool ESP32CAN::sendFrameFD(CAN_FRAME_FD &txFrame)
{
    if (node_handle == nullptr) return false;

    static uint8_t  txBufs[8][64];
    static int      txBufIdx = 0;
    static twai_frame_t raw[8];

    uint8_t len = txFrame.length <= 64 ? txFrame.length : 64;
    int slot    = txBufIdx & 7;
    txBufIdx++;

    uint8_t *buf = txBufs[slot];
    for (int i = 0; i < len; i++) buf[i] = txFrame.data.uint8[i];

    raw[slot].header.id  = txFrame.id;
    raw[slot].header.ide = txFrame.extended ? 1 : 0;
    raw[slot].header.rtr = 0;
    raw[slot].header.fdf = 1;  // FD frame
    // Per-frame BRS: caller sets txFrame.rrs to request bit-rate switching.
    // BRS still needs a data baudrate configured (setDataBaudrate before init).
    raw[slot].header.brs = (txFrame.rrs && _dataBaudrate > 0) ? 1 : 0;
    raw[slot].buffer     = buf;
    raw[slot].buffer_len = len;

    return (twai_node_transmit(node_handle, &raw[slot], 0) == ESP_OK);
}

uint32_t ESP32CAN::get_rx_buffFD(CAN_FRAME_FD &msg)
{
    if (!rx_queue_fd || !uxQueueMessagesWaiting(rx_queue_fd)) return false;
    CAN_FRAME_FD frame;
    if (xQueueReceive(rx_queue_fd, &frame, 0) == pdTRUE) {
        msg = frame;
        return true;
    }
    return false;
}

uint16_t ESP32CAN::availableFD()
{
    if (!rx_queue_fd) return 0;
    return (uint16_t)uxQueueMessagesWaiting(rx_queue_fd);
}
#endif

// ── Queue accessors (TIER_NEW – same as other tiers) ─────────────────────────

bool ESP32CAN::rx_avail()
{
    if (!rx_queue) return false;
    return uxQueueMessagesWaiting(rx_queue) > 0;
}

uint16_t ESP32CAN::available()
{
    if (!rx_queue) return 0;
    return (uint16_t)uxQueueMessagesWaiting(rx_queue);
}

uint32_t ESP32CAN::get_rx_buff(CAN_FRAME &msg)
{
    if (!rx_queue) return false;
    if (!uxQueueMessagesWaiting(rx_queue)) return false;
    CAN_FRAME frame;
    if (xQueueReceive(rx_queue, &frame, 0) == pdTRUE) {
        msg = frame;
        return true;
    }
    return false;
}

void ESP32CAN::resetIfStale(uint32_t stallMs)
{
    // cyclesSinceTraffic is incremented every 200 ms by the watchdog task
    uint32_t threshold = stallMs / 200;
    if (cyclesSinceTraffic > threshold) {
        disable();
        enable();
    }
}

bool IRAM_ATTR ESP32CAN::onStateChange(twai_node_handle_t handle,
                                        const twai_state_change_event_data_t *edata,
                                        void *user_ctx)
{
    // This is now the authoritative writer of readyForTraffic. The
    // IDF 5.5 twai_state_change_event_data_t struct exposes both the
    // old and new state, so we can map transitions deterministically:
    //
    //   TWAI_ERROR_ACTIVE   — fully functional, ready for TX/RX
    //   TWAI_ERROR_WARNING  — TEC/REC > 96 but still active, can TX
    //   TWAI_ERROR_PASSIVE  — TEC/REC > 127, degraded but can still TX
    //                          (won't assert dominant ACKs)
    //   TWAI_ERROR_BUS_OFF  — TEC saturated; controller is TX-blocked
    //                          and needs a disable/enable cycle to
    //                          recover. This is the only state where
    //                          we should refuse to feed sendFrame.
    //
    // We also set _errorPassive on entry to BUS_OFF so the watchdog
    // task picks up the recovery cycle on its next poll. (The name
    // is historical — it really means "watchdog: look at me".)
    ESP32CAN *espCan = static_cast<ESP32CAN *>(user_ctx);
    (void)handle;
    if (!edata) return false;

    switch (edata->new_sta) {
        case TWAI_ERROR_BUS_OFF:
            espCan->readyForTraffic = false;
            espCan->_errorPassive   = true;   // wake the watchdog
            break;
        case TWAI_ERROR_ACTIVE:
        case TWAI_ERROR_PASSIVE:
        default:
            // Anything that isn't bus-off is fine to TX into. The
            // controller will absorb transient errors on its own.
            espCan->readyForTraffic = true;
            break;
    }
    return false;
}

bool ESP32CAN::getControllerStats(CANControllerStats &stats)
{
    if (node_handle == nullptr) return false;

    twai_node_status_t status;
    twai_node_record_t record;
    if (twai_node_get_info(node_handle, &status, &record) != ESP_OK) {
        return false;
    }

    // twai_node_record_t already exposes TEC/REC as uint16_t.
    stats.txErrorCount  = status.tx_error_count;
    stats.rxErrorCount  = status.rx_error_count;
    stats.busErrorCount = saturate_u16(record.bus_err_num);
    // The new esp_driver_twai API does not expose an RX overrun counter.
    // The HW counter is still incremented internally, but there's no
    // public accessor as of IDF 5.5. Reads 0 here for forward-compat:
    // if a future IDF adds the field we can wire it in without breaking
    // the API.
    stats.rxOverrunCount = 0;
    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
//  TIER_V2  (IDF 5.2–5.4)  –  legacy _v2 handle-based API
// ═════════════════════════════════════════════════════════════════════════════
#elif defined(TWAI_TIER_V2)

// ── Watchdog task ─────────────────────────────────────────────────────────────

void ESP32CAN::CAN_WatchDog_Builtin(void *pvParameters)
{
    ESP32CAN *espCan = static_cast<ESP32CAN *>(pvParameters);
    const TickType_t xDelay = pdMS_TO_TICKS(200);
    twai_status_info_t status_info;

    for (;;) {
        vTaskDelay(xDelay);
        espCan->cyclesSinceTraffic++;

        if (twai_get_status_info_v2(espCan->bus_handle, &status_info) != ESP_OK) continue;

        bool needsRecovery = false;

        if (status_info.state == TWAI_STATE_BUS_OFF) {
            needsRecovery = true;
            twai_initiate_recovery_v2(espCan->bus_handle);
        }

        if (status_info.tx_error_counter >= 128 ||
            status_info.rx_error_counter >= 128) {
            needsRecovery = true;
        }

        if (espCan->cyclesSinceTraffic > 10 &&
            espCan->readyForTraffic         &&
            espCan->trafficEverSeen)
        {
            needsRecovery = true;
        }

        if (needsRecovery) {
            espCan->cyclesSinceTraffic = 0;
            espCan->readyForTraffic    = false;
            twai_stop_v2(espCan->bus_handle);
            if (twai_start_v2(espCan->bus_handle) == ESP_OK) {
                espCan->readyForTraffic = true;
            }
        }
    }
}

// ── RX task ───────────────────────────────────────────────────────────────────

void ESP32CAN::task_LowLevelRX(void *pvParameters)
{
    ESP32CAN *espCan = static_cast<ESP32CAN *>(pvParameters);

    while (1) {
        if (espCan->readyForTraffic) {
            twai_message_t message;
            if (twai_receive_v2(espCan->bus_handle, &message,
                                pdMS_TO_TICKS(100)) == ESP_OK)
            {
                CAN_FRAME msg;
                msg.id       = message.identifier;
                msg.length   = message.data_length_code;
                msg.rtr      = message.rtr;
                msg.extended = message.extd;
                for (int i = 0; i < 8; i++) msg.data.byte[i] = message.data[i];
                espCan->processFrame(msg);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// ── enable / disable (TIER_V2) ────────────────────────────────────────────────

void ESP32CAN::enable()
{
    twai_general_cfg.rx_io = _rxPin;
    twai_general_cfg.tx_io = _txPin;
    twai_general_cfg.mode  = listenOnly ? TWAI_MODE_LISTEN_ONLY :
                              noACK     ? TWAI_MODE_NO_ACK      :
                                          TWAI_MODE_NORMAL;

    if (twai_driver_install_v2(&twai_general_cfg, &twai_speed_cfg,
                                &twai_filters_cfg, &bus_handle) != ESP_OK) {
        if (debuggingMode) printf("[TWAI] Failed to install driver (bus %d)\n", _busNum);
        return;
    }

    callbackQueue = xQueueCreate(16,           sizeof(CAN_FRAME));
    rx_queue      = xQueueCreate(rxBufferSize,  sizeof(CAN_FRAME));

    char rxName[24], cbName[24];
    snprintf(rxName, sizeof(rxName), "CAN_LORX_CAN%d", _busNum);
    snprintf(cbName, sizeof(cbName), "CAN_RX_CAN%d",   _busNum);

    xTaskCreate(ESP32CAN::task_CAN, cbName, 8192, this, 15, &task_CAN_handler);
#if defined(CONFIG_FREERTOS_UNICORE)
    xTaskCreate(ESP32CAN::task_LowLevelRX, rxName, 4096, this, 19,
                &task_LowLevelRX_handler);
#else
    xTaskCreatePinnedToCore(ESP32CAN::task_LowLevelRX, rxName, 4096, this, 19,
                            &task_LowLevelRX_handler, 1);
#endif

    if (twai_start_v2(bus_handle) != ESP_OK) {
        if (debuggingMode) printf("[TWAI] Failed to start driver (bus %d)\n", _busNum);
        return;
    }
    if (debuggingMode) printf("[TWAI] Driver started (bus %d)\n", _busNum);
    readyForTraffic = true;
}

void ESP32CAN::disable()
{
    readyForTraffic = false;

    twai_status_info_t info;
    if (twai_get_status_info_v2(bus_handle, &info) == ESP_OK) {
        if (info.state == TWAI_STATE_RUNNING) {
            twai_stop_v2(bus_handle);
        }
    }

    for (auto task : {task_CAN_handler, task_LowLevelRX_handler}) {
        if (task != nullptr) { vTaskDelete(task); }
    }
    task_CAN_handler        = nullptr;
    task_LowLevelRX_handler = nullptr;

    if (rx_queue)      { vQueueDelete(rx_queue);      rx_queue      = nullptr; }
    if (callbackQueue) { vQueueDelete(callbackQueue); callbackQueue = nullptr; }

    twai_driver_uninstall_v2(bus_handle);
    bus_handle = nullptr;
}

// ── _init (TIER_V2) ───────────────────────────────────────────────────────────

void ESP32CAN::_init()
{
    for (int i = 0; i < BI_NUM_FILTERS; i++) filters[i] = {0, 0, false, false};

    if (CAN_WatchDog_Builtin_handler == nullptr) {
        char wdName[28];
        snprintf(wdName, sizeof(wdName), "CAN_WD_BI_CAN%d", _busNum);
#if defined(CONFIG_FREERTOS_UNICORE)
        xTaskCreate(&CAN_WatchDog_Builtin, wdName, 2048, this, 10,
                    &CAN_WatchDog_Builtin_handler);
#else
        xTaskCreatePinnedToCore(&CAN_WatchDog_Builtin, wdName, 2048, this, 10,
                                &CAN_WatchDog_Builtin_handler, 1);
#endif
    }
}

// ── init / set_baudrate (TIER_V2) ─────────────────────────────────────────────

uint32_t ESP32CAN::init(uint32_t ul_baudrate)
{
    _init();
    set_baudrate(ul_baudrate);

    if (debuggingMode) {
        uint32_t alerts = TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF |
                          TWAI_ALERT_AND_LOG  | TWAI_ALERT_ERR_ACTIVE |
                          TWAI_ALERT_ARB_LOST | TWAI_ALERT_BUS_ERROR |
                          TWAI_ALERT_TX_FAILED | TWAI_ALERT_RX_QUEUE_FULL;
        twai_reconfigure_alerts_v2(bus_handle, alerts, nullptr);
    }

    // Kick off the low-level RX task (separate from enable; init only once)
    char rxName[24];
    snprintf(rxName, sizeof(rxName), "CAN_LORX_CAN%d", _busNum);
#if defined(CONFIG_FREERTOS_UNICORE)
    xTaskCreate(ESP32CAN::task_LowLevelRX, rxName, 4096, this, 19, nullptr);
#else
    xTaskCreatePinnedToCore(ESP32CAN::task_LowLevelRX, rxName, 4096, this, 19,
                            nullptr, 1);
#endif

    readyForTraffic = true;
    return ul_baudrate;
}

uint32_t ESP32CAN::set_baudrate(uint32_t ul_baudrate)
{
    disable();
    for (int i = 0; valid_timings[i].speed != 0; i++) {
        if (valid_timings[i].speed == ul_baudrate) {
            twai_speed_cfg = valid_timings[i].cfg;
            enable();
            return ul_baudrate;
        }
    }
    if (debuggingMode) printf("[TWAI] Could not find valid bit timing for %lu bps!\n", ul_baudrate);
    return 0;
}

uint32_t ESP32CAN::beginAutoSpeed()
{
    twai_general_config_t savedMode = twai_general_cfg;
    _init();
    readyForTraffic = false;
    twai_stop_v2(bus_handle);
    twai_general_cfg.mode = TWAI_MODE_LISTEN_ONLY;

    for (int i = 0; valid_timings[i].speed != 0; i++) {
        twai_speed_cfg = valid_timings[i].cfg;
        disable();
        Serial.print("Trying speed "); Serial.print(valid_timings[i].speed);
        enable();
        delay(600);
        if (cyclesSinceTraffic < 2) {
            disable();
            twai_general_cfg.mode = savedMode.mode;
            enable();
            Serial.println(" SUCCESS!");
            return valid_timings[i].speed;
        }
        Serial.println(" FAILED.");
    }

    Serial.println("None of the tested CAN speeds worked!");
    twai_stop_v2(bus_handle);
    return 0;
}

void ESP32CAN::setListenOnlyMode(bool state)
{
    listenOnly = state;
    disable();
    twai_general_cfg.mode = state ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL;
    enable();
}

void ESP32CAN::setNoACKMode(bool state)
{
    noACK = state;
    disable();
    twai_general_cfg.mode = state ? TWAI_MODE_NO_ACK : TWAI_MODE_NORMAL;
    enable();
}

// ── sendFrame (TIER_V2) ───────────────────────────────────────────────────────

bool ESP32CAN::sendFrame(CAN_FRAME &txFrame)
{
    twai_message_t raw = {};
    raw.identifier        = txFrame.id;
    raw.data_length_code  = txFrame.length;
    raw.rtr               = txFrame.rtr;
    raw.extd              = txFrame.extended;
    for (int i = 0; i < 8; i++) raw.data[i] = txFrame.data.byte[i];

    esp_err_t res = twai_transmit_v2(bus_handle, &raw, pdMS_TO_TICKS(4));
    switch (res) {
        case ESP_OK:           if (debuggingMode) Serial.write('<'); break;
        case ESP_ERR_TIMEOUT:  if (debuggingMode) Serial.write('T'); break;
        default:               if (debuggingMode) Serial.write('!'); break;
    }
    return (res == ESP_OK);
}

// ── Queue accessors (TIER_V2) ─────────────────────────────────────────────────

bool ESP32CAN::rx_avail()
{
    if (!rx_queue) return false;
    return uxQueueMessagesWaiting(rx_queue) > 0;
}

uint16_t ESP32CAN::available()
{
    if (!rx_queue) return 0;
    return (uint16_t)uxQueueMessagesWaiting(rx_queue);
}

uint32_t ESP32CAN::get_rx_buff(CAN_FRAME &msg)
{
    if (!rx_queue || !uxQueueMessagesWaiting(rx_queue)) return false;
    CAN_FRAME frame;
    if (xQueueReceive(rx_queue, &frame, 0) == pdTRUE) { msg = frame; return true; }
    return false;
}

void ESP32CAN::resetIfStale(uint32_t stallMs)
{
    // cyclesSinceTraffic is incremented every 200 ms by the watchdog task
    uint32_t threshold = stallMs / 200;
    if (cyclesSinceTraffic > threshold) {
        disable();
        enable();
    }
}

bool ESP32CAN::getControllerStats(CANControllerStats &stats)
{
    if (bus_handle == nullptr) return false;

    twai_status_info_t info;
    if (twai_get_status_info_v2(bus_handle, &info) != ESP_OK) {
        return false;
    }

    // twai_status_info_t fields are uint32_t in the legacy driver.
    // Saturate to keep the wire format compact and stable across tiers.
    stats.txErrorCount   = saturate_u16(info.tx_error_counter);
    stats.rxErrorCount   = saturate_u16(info.rx_error_counter);
    stats.busErrorCount  = saturate_u16(info.bus_error_count);
    stats.rxOverrunCount = saturate_u16(info.rx_overrun_count);
    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
//  TIER_V1  (IDF < 5.2)  –  legacy single-controller API
// ═════════════════════════════════════════════════════════════════════════════
#else // TWAI_TIER_V1

void ESP32CAN::CAN_WatchDog_Builtin(void *pvParameters)
{
    ESP32CAN *espCan = static_cast<ESP32CAN *>(pvParameters);
    const TickType_t xDelay = pdMS_TO_TICKS(200);
    twai_status_info_t status_info;

    for (;;) {
        vTaskDelay(xDelay);
        espCan->cyclesSinceTraffic++;

        if (twai_get_status_info(&status_info) == ESP_OK) {
            if (status_info.state == TWAI_STATE_BUS_OFF) {
                espCan->cyclesSinceTraffic = 0;
                if (twai_initiate_recovery() != ESP_OK) {
                    if (debuggingMode) printf("[TWAI] Could not initiate bus recovery!\n");
                }
            }
        }
    }
}

void ESP32CAN::task_LowLevelRX(void *pvParameters)
{
    ESP32CAN *espCan = static_cast<ESP32CAN *>(pvParameters);
    while (1) {
        if (espCan->readyForTraffic) {
            twai_message_t message;
            if (twai_receive(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
                CAN_FRAME msg;
                msg.id       = message.identifier;
                msg.length   = message.data_length_code;
                msg.rtr      = message.rtr;
                msg.extended = message.extd;
                for (int i = 0; i < 8; i++) msg.data.byte[i] = message.data[i];
                espCan->processFrame(msg);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void ESP32CAN::enable()
{
    twai_general_cfg.rx_io = _rxPin;
    twai_general_cfg.tx_io = _txPin;
    twai_general_cfg.mode  = listenOnly ? TWAI_MODE_LISTEN_ONLY :
                              noACK     ? TWAI_MODE_NO_ACK      :
                                          TWAI_MODE_NORMAL;

    if (twai_driver_install(&twai_general_cfg, &twai_speed_cfg,
                             &twai_filters_cfg) != ESP_OK) {
        if (debuggingMode) printf("[TWAI] Failed to install driver\n");
        return;
    }

    callbackQueue = xQueueCreate(16,           sizeof(CAN_FRAME));
    rx_queue      = xQueueCreate(rxBufferSize,  sizeof(CAN_FRAME));

    xTaskCreate(ESP32CAN::task_CAN,         "CAN_RX_CAN",   8192, this, 15, &task_CAN_handler);
#if defined(CONFIG_FREERTOS_UNICORE)
    xTaskCreate(ESP32CAN::task_LowLevelRX,  "CAN_LORX_CAN", 4096, this, 19, &task_LowLevelRX_handler);
#else
    xTaskCreatePinnedToCore(ESP32CAN::task_LowLevelRX, "CAN_LORX_CAN", 4096, this, 19,
                            &task_LowLevelRX_handler, 1);
#endif

    if (twai_start() != ESP_OK) {
        if (debuggingMode) printf("[TWAI] Failed to start driver\n");
        return;
    }
    if (debuggingMode) printf("[TWAI] Driver started\n");
    readyForTraffic = true;
}

void ESP32CAN::disable()
{
    readyForTraffic = false;
    twai_status_info_t info;
    if (twai_get_status_info(&info) == ESP_OK) {
        if (info.state == TWAI_STATE_RUNNING) twai_stop();
    }

    for (auto task : {task_CAN_handler, task_LowLevelRX_handler}) {
        if (task != nullptr) { vTaskDelete(task); }
    }
    task_CAN_handler        = nullptr;
    task_LowLevelRX_handler = nullptr;

    if (rx_queue)      { vQueueDelete(rx_queue);      rx_queue      = nullptr; }
    if (callbackQueue) { vQueueDelete(callbackQueue); callbackQueue = nullptr; }

    twai_driver_uninstall();
}

void ESP32CAN::_init()
{
    for (int i = 0; i < BI_NUM_FILTERS; i++) filters[i] = {0, 0, false, false};

    if (CAN_WatchDog_Builtin_handler == nullptr) {
#if defined(CONFIG_FREERTOS_UNICORE)
        xTaskCreate(&CAN_WatchDog_Builtin, "CAN_WD_BI", 2048, this, 10,
                    &CAN_WatchDog_Builtin_handler);
#else
        xTaskCreatePinnedToCore(&CAN_WatchDog_Builtin, "CAN_WD_BI", 2048, this, 10,
                                &CAN_WatchDog_Builtin_handler, 1);
#endif
    }
}

uint32_t ESP32CAN::init(uint32_t ul_baudrate)
{
    _init();
    set_baudrate(ul_baudrate);

    if (debuggingMode) {
        uint32_t alerts = TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF |
                          TWAI_ALERT_AND_LOG  | TWAI_ALERT_ERR_ACTIVE |
                          TWAI_ALERT_ARB_LOST | TWAI_ALERT_BUS_ERROR |
                          TWAI_ALERT_TX_FAILED | TWAI_ALERT_RX_QUEUE_FULL;
        twai_reconfigure_alerts(alerts, nullptr);
    }

    xTaskCreatePinnedToCore(ESP32CAN::task_LowLevelRX, "CAN_LORX_CAN0", 4096, this, 19, nullptr, 1);
    readyForTraffic = true;
    return ul_baudrate;
}

uint32_t ESP32CAN::set_baudrate(uint32_t ul_baudrate)
{
    disable();
    for (int i = 0; valid_timings[i].speed != 0; i++) {
        if (valid_timings[i].speed == ul_baudrate) {
            twai_speed_cfg = valid_timings[i].cfg;
            enable();
            return ul_baudrate;
        }
    }
    if (debuggingMode) printf("[TWAI] Could not find valid bit timing for %lu bps!\n", ul_baudrate);
    return 0;
}

uint32_t ESP32CAN::beginAutoSpeed()
{
    twai_general_config_t savedMode = twai_general_cfg;
    _init();
    readyForTraffic = false;
    twai_stop();
    twai_general_cfg.mode = TWAI_MODE_LISTEN_ONLY;

    for (int i = 0; valid_timings[i].speed != 0; i++) {
        twai_speed_cfg = valid_timings[i].cfg;
        disable();
        Serial.print("Trying speed "); Serial.print(valid_timings[i].speed);
        enable();
        delay(600);
        if (cyclesSinceTraffic < 2) {
            disable();
            twai_general_cfg.mode = savedMode.mode;
            enable();
            Serial.println(" SUCCESS!");
            return valid_timings[i].speed;
        }
        Serial.println(" FAILED.");
    }

    Serial.println("None of the tested CAN speeds worked!");
    twai_stop();
    return 0;
}

void ESP32CAN::setListenOnlyMode(bool state)
{
    listenOnly = state;
    disable();
    twai_general_cfg.mode = state ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL;
    enable();
}

void ESP32CAN::setNoACKMode(bool state)
{
    noACK = state;
    disable();
    twai_general_cfg.mode = state ? TWAI_MODE_NO_ACK : TWAI_MODE_NORMAL;
    enable();
}

bool ESP32CAN::sendFrame(CAN_FRAME &txFrame)
{
    twai_message_t raw = {};
    raw.identifier       = txFrame.id;
    raw.data_length_code = txFrame.length;
    raw.rtr              = txFrame.rtr;
    raw.extd             = txFrame.extended;
    for (int i = 0; i < 8; i++) raw.data[i] = txFrame.data.byte[i];

    esp_err_t res = twai_transmit(&raw, pdMS_TO_TICKS(4));
    switch (res) {
        case ESP_OK:           if (debuggingMode) Serial.write('<'); break;
        case ESP_ERR_TIMEOUT:  if (debuggingMode) Serial.write('T'); break;
        default:               if (debuggingMode) Serial.write('!'); break;
    }
    return (res == ESP_OK);
}

bool ESP32CAN::rx_avail()
{
    if (!rx_queue) return false;
    return uxQueueMessagesWaiting(rx_queue) > 0;
}

uint16_t ESP32CAN::available()
{
    if (!rx_queue) return 0;
    return (uint16_t)uxQueueMessagesWaiting(rx_queue);
}

uint32_t ESP32CAN::get_rx_buff(CAN_FRAME &msg)
{
    if (!rx_queue || !uxQueueMessagesWaiting(rx_queue)) return false;
    CAN_FRAME frame;
    if (xQueueReceive(rx_queue, &frame, 0) == pdTRUE) { msg = frame; return true; }
    return false;
}

bool ESP32CAN::getControllerStats(CANControllerStats &stats)
{
    twai_status_info_t info;
    if (twai_get_status_info(&info) != ESP_OK) {
        return false;
    }

    stats.txErrorCount   = saturate_u16(info.tx_error_counter);
    stats.rxErrorCount   = saturate_u16(info.rx_error_counter);
    stats.busErrorCount  = saturate_u16(info.bus_error_count);
    stats.rxOverrunCount = saturate_u16(info.rx_overrun_count);
    return true;
}

#endif // TWAI_TIER_*
