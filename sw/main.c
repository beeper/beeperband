/**
 * BeeperBand main program
 *
 * Portions copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_delay.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrfx_spim.h"
#include "nrfx_twim.h"
#include "nrfx_gpiote.h"


#define DEVICE_NAME                     "BeeperBand"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "Beeper"                                /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

enum sys_state {
    STATE_ADVERTISING,
    STATE_CONNECTED,
};

static enum sys_state sys_state = STATE_ADVERTISING;

static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};

static void _spim_npxl_bytes(const uint8_t *bytes, int nbytes);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    const uint8_t ble_err[] = { 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00 };
    _spim_npxl_bytes(ble_err, sizeof(ble_err));

    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_WATCH);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
    
    /* Beeper service code is implemented later. */
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("BLE disconnected");
            sys_state = STATE_ADVERTISING;
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("BLE connected");
            sys_state = STATE_CONNECTED;
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/* Neopixel SPI I/O routines */

#define P_NEOPIX_EN NRF_GPIO_PIN_MAP(1,7)
#define P_NEOPIX    NRF_GPIO_PIN_MAP(0,16)
#define P_BTN1      NRF_GPIO_PIN_MAP(1,1)
#define P_BTN2      NRF_GPIO_PIN_MAP(1,2)
#define P_BTN3      NRF_GPIO_PIN_MAP(1,3)
#define P_BTN4      NRF_GPIO_PIN_MAP(1,4)
#define P_BTN5      NRF_GPIO_PIN_MAP(1,5)
#define P_BTN6      NRF_GPIO_PIN_MAP(1,6)


static uint8_t _spim_npxl_txbuf[512];
static const nrfx_spim_t spi = NRFX_SPIM_INSTANCE(0);

static volatile int spi_xfer_done;
void spi_evt_handler(const nrfx_spim_evt_t *evt, void *ctx) {
    spi_xfer_done++;
}

static void _spim_npxl_setup() {
    nrf_gpio_cfg_output(P_NEOPIX_EN);
    nrf_gpio_cfg_output(P_NEOPIX);
    nrf_gpio_pin_write(P_NEOPIX_EN, 1);
    nrf_gpio_pin_write(P_NEOPIX, 0);
    
    nrfx_spim_config_t config = {
        .sck_pin = P_NEOPIX + 1 /* whatever */,
        .miso_pin = NRFX_SPIM_PIN_NOT_USED,
        .mosi_pin = P_NEOPIX,
        .ss_pin = NRFX_SPIM_PIN_NOT_USED,
        .frequency = NRF_SPIM_FREQ_8M,
        .mode = NRF_SPIM_MODE_0,
        .bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST,
    };
    nrfx_spim_init(&spi, &config, spi_evt_handler, NULL);
}

static void _spim_npxl_bytes(const uint8_t *bytes, int nbytes) {
    int obytes = 0;
    int obit = 0;
    uint8_t zeroes[9] = {};
    
    /* If we're setting all zeroes, then we can just shut down the NeoPixel
     * power entirely.  */
    if (nbytes == 9 && !memcmp(bytes, zeroes, 9)) {
        nrf_gpio_pin_write(P_NEOPIX_EN, 0);
    } else {
        nrf_gpio_pin_write(P_NEOPIX_EN, 1);
    }

#define PUTBIT(b) do { \
    _spim_npxl_txbuf[obytes] |= (!!(b)) << (7 - obit); \
    obit++; \
    if (obit == 8) { obit = 0; obytes++; } \
} while(0)

    memset(_spim_npxl_txbuf, 0, sizeof(_spim_npxl_txbuf));
    obytes += 2400 / 8; /* 300us */
    while (nbytes) {
        uint8_t c = *bytes;
        for (int ibit = 0; ibit < 8; ibit++) {
            int n1 = (c & 0x80) ? 6 : 3;
            int n0 = 10 - n1;
            c <<= 1;
            
            for (int i = 0; i < n1; i++)
                PUTBIT(1);
            for (int i = 0; i < n0; i++)
                PUTBIT(0);
        }
        bytes++;
        nbytes--;
    }
    obytes++;
    NRF_LOG_INFO("npxl tx %d obytes", obytes);
    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(_spim_npxl_txbuf, obytes);
    spi_xfer_done = 0;
    nrfx_spim_xfer(&spi, &xfer_desc, 0);
    while (!spi_xfer_done)
        ;
}

#define P_SCL       NRF_GPIO_PIN_MAP(0, 11)
#define P_SDA       NRF_GPIO_PIN_MAP(0, 12)
#define P_HAP_EN    NRF_GPIO_PIN_MAP(0, 26)
#define P_HAP_IN    NRF_GPIO_PIN_MAP(0, 27)
#define DRV2605_ADR (0x5A)

#define LOG_SYNC(...) do { NRF_LOG_INFO(__VA_ARGS__); while (NRF_LOG_PROCESS()); } while(0)

static const nrfx_twim_t i2c = NRFX_TWIM_INSTANCE(1);
static volatile int i2c_done = 0;

static void _i2c_evt(const nrfx_twim_evt_t *evt, void *ctx) {
    if (evt->type == NRFX_TWIM_EVT_DONE) {
//        NRF_LOG_INFO("NRFX_TWIM_EVT_DONE");
    } else if (evt->type == NRFX_TWIM_EVT_ADDRESS_NACK) {
        NRF_LOG_INFO("NRFX_TWIM_ADDRESS_NACK");
    } else if (evt->type == NRFX_TWIM_EVT_DATA_NACK) {
        NRF_LOG_INFO("NRFX_TWIM_DATA_NACK");
    } else if (evt->type == NRFX_TWIM_EVT_OVERRUN) {
        NRF_LOG_INFO("NRFX_TWIM_OVERRUN");
    } else if (evt->type == NRFX_TWIM_EVT_BUS_ERROR) {
        NRF_LOG_INFO("NRFX_TWIM_BUS_ERROR");
    }
    i2c_done = 1;
}

static void _i2c_drv2605_write(uint8_t reg, uint8_t val) {
    ret_code_t rv;

    while (nrfx_twim_is_busy(&i2c))
        ;

    uint8_t txbuf[2] = { reg, val };
    nrfx_twim_xfer_desc_t desc = {
        .type = NRFX_TWIM_XFER_TX,
        .address = DRV2605_ADR,
        .primary_length = 2,
        .p_primary_buf = txbuf,
    };
    
    i2c_done = 0;
    rv = nrfx_twim_xfer(&i2c, &desc, 0);
    APP_ERROR_CHECK(rv);

    while (nrfx_twim_is_busy(&i2c) || !i2c_done)
        ;

    LOG_SYNC("DRV2605: %02x <- %02x", reg, val);
}

static uint8_t _i2c_drv2605_read(uint8_t reg) {
    ret_code_t rv;

    while (nrfx_twim_is_busy(&i2c))
        ;

    uint8_t rxbuf;
    nrfx_twim_xfer_desc_t desc = {
        .type = NRFX_TWIM_XFER_TXRX,
        .address = DRV2605_ADR,
        .primary_length = 1,
        .p_primary_buf = &reg,
        .secondary_length = 1,
        .p_secondary_buf = &rxbuf,
    };
    
    i2c_done = 0;
    rv = nrfx_twim_xfer(&i2c, &desc, 0);
    APP_ERROR_CHECK(rv);

    while (nrfx_twim_is_busy(&i2c) && !i2c_done)
        ;

    return rxbuf;
}

static void _i2c_drv2605_autocal() {
    LOG_SYNC("running DRV2605 autocalibration");
    _i2c_drv2605_write(0x01, 0x07); /* enter autocal mode*/
    _i2c_drv2605_write(0x1A /* FEEDBACK */, (1 << 7 /* LRA */) | (2 << 4 /* FB_BRAKE_FACTOR = 2 */) | (2 << 2 /* LOOP_GAIN = 2 */) | (2 << 0 /* BEMF_GAIN = default */));
    /* RATED_VOLTAGE: 1.8V = 81, according to this hokey formula:
     * https://android.googlesource.com/kernel/msm/+/ea5a03d280ea957b533b4938afc628aa6e323a4a/drivers/misc/drv2605.c#89
     */
    _i2c_drv2605_write(0x16 /* RATED_VOLTAGE */, 81);
    /* leave OD_CLAMP alone, I guess */
    _i2c_drv2605_write(0x1E /* CONTROL4 */, (3 << 4 /* AUTO_CAL_TIME */));
    /* DRIVE_TIME default at 0x13 */
    _i2c_drv2605_write(0x1C /* CONTROL2 */, (1 << 7 /* BIDIR_INPUT */) | (1 << 6 /* BRAKE_STABILIZER */) | (3 << 4 /* SAMPLE_TIME */) | (1 << 2 /* BLANKING_TIME */) | (1 << 0 /* IDISS_TIME */));
    _i2c_drv2605_write(0x0C, 0x01); /* GO */
    while (_i2c_drv2605_read(0x0C) & 1) /* GO */
        ;
    if (_i2c_drv2605_read(0x00) & 8 /* DIAG_RESULT */) {
        LOG_SYNC("DRV2605 autocalibration failed");
    }
    _i2c_drv2605_write(0x01, 0x00); /* enter normal mode */
    LOG_SYNC("DRV2605: autocalibration complete");
}

static void _i2c_drv2605_setup() {
    ret_code_t rv;
    nrf_gpio_cfg_output(P_HAP_EN);
    nrf_gpio_cfg_output(P_HAP_IN);
    nrf_gpio_pin_write(P_HAP_IN, 0);
    nrf_gpio_pin_write(P_HAP_EN, 0);
    nrf_delay_ms(5);
    nrf_gpio_pin_write(P_HAP_EN, 1);

    nrfx_twim_config_t config = {
        .scl = P_SCL,
        .sda = P_SDA,
        .frequency = NRF_TWIM_FREQ_100K,
    };
    rv = nrfx_twim_init(&i2c, &config, _i2c_evt, NULL);
    APP_ERROR_CHECK(rv);
    nrfx_twim_enable(&i2c);
    
    LOG_SYNC("DRV2605: reg 0 val %d", _i2c_drv2605_read(0));
    _i2c_drv2605_autocal();
    _i2c_drv2605_write(0x03 /* LIBRARY_SEL */, 6 /* LRA library */);
}

static void _i2c_drv2605_run(uint8_t program) {
    _i2c_drv2605_write(0x04, program);
    _i2c_drv2605_write(0x05, 0);
    _i2c_drv2605_write(0x0C, 0x01); /* GO */
}

static int _i2c_drv2605_is_done() {
    return !(_i2c_drv2605_read(0x0C) & 1);
}

#define CMDQ_RINGBUF_LEN 512
static int beeper_cmdq_ringbuf_prodp = 0;
static int beeper_cmdq_ringbuf_consp = 0;
static int beeper_cmdq_bytes_used = 0;
static uint8_t beeper_cmdq_ringbuf[CMDQ_RINGBUF_LEN];

#define CMDQ_RXBUF_LEN 64
static ble_uuid_t beeper_svc_uuid;
static uint16_t beeper_svc_hnd = BLE_GATT_HANDLE_INVALID;
static ble_gatts_char_handles_t beeper_svc_notify_hnd;
static ble_gatts_char_handles_t beeper_svc_cmdq_hnd;
static uint16_t beeper_svc_notify_cccd;
static uint8_t beeper_btn_buf = 0x00;
static uint8_t beeper_cmdq_rxbuf[CMDQ_RXBUF_LEN];

static void _beeper_handler(const ble_evt_t *evt, void *ctx) {
    switch (evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_INFO("BEEPER: GAP connected");
        beeper_svc_notify_cccd = 0;
        beeper_cmdq_ringbuf_prodp = beeper_cmdq_ringbuf_consp = beeper_cmdq_bytes_used = 0;
        break;
    case BLE_GATTS_EVT_WRITE: {
        const ble_gatts_evt_write_t *evtwr = &evt->evt.gatts_evt.params.write;
        if (evtwr->handle == beeper_svc_notify_hnd.cccd_handle && evtwr->len <= 2) {
            memcpy(&beeper_svc_notify_cccd, evtwr->data, evtwr->len);
            NRF_LOG_INFO("BEEPER: notify CCCD changed to %d", beeper_svc_notify_cccd);
        } else if (evtwr->handle == beeper_svc_cmdq_hnd.value_handle && evtwr->len <= CMDQ_RXBUF_LEN) {
            if (evtwr->len == 0) {
                beeper_cmdq_ringbuf_prodp = beeper_cmdq_ringbuf_consp = beeper_cmdq_bytes_used = 0;
            } else if ((beeper_cmdq_bytes_used + evtwr->len) < CMDQ_RINGBUF_LEN) {
                for (int i = 0; i < evtwr->len; i++) {
                    beeper_cmdq_ringbuf[beeper_cmdq_ringbuf_prodp++] = evtwr->data[i];
                    beeper_cmdq_ringbuf_prodp %= CMDQ_RINGBUF_LEN;
                    beeper_cmdq_bytes_used += 1;
                }
                NRF_LOG_INFO("BEEPER: cmdq len now %d", beeper_cmdq_bytes_used);
            }
        }
    }
    }
}

static void _beeper_gpiote(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    int mask = ((!nrf_gpio_pin_read(P_BTN1)) << 0) |
               ((!nrf_gpio_pin_read(P_BTN2)) << 1) |
               ((!nrf_gpio_pin_read(P_BTN3)) << 2) |
               ((!nrf_gpio_pin_read(P_BTN4)) << 3) |
               ((!nrf_gpio_pin_read(P_BTN5)) << 4) |
               ((!nrf_gpio_pin_read(P_BTN6)) << 5);
    if (mask == beeper_btn_buf)
        return;
    beeper_btn_buf = mask;
    NRF_LOG_INFO("BEEPER: btn transition -> %x", mask);
    if (beeper_svc_notify_cccd & BLE_GATT_HVX_NOTIFICATION) {
        ble_gatts_hvx_params_t hvx;
        uint16_t len16 = 1;

        memset(&hvx, 0, sizeof(hvx));
        hvx.type = BLE_GATT_HVX_NOTIFICATION;
        hvx.handle = beeper_svc_notify_hnd.value_handle;
        hvx.p_data = &beeper_btn_buf;
        hvx.p_len = &len16;

        (void) sd_ble_gatts_hvx(m_conn_handle, &hvx); /* if SD is out of resources, oh well */
        
        ble_gatts_value_t val;
        val.len = 0;
        val.offset = 0;
        val.p_value = &beeper_btn_buf;
        sd_ble_gatts_value_set(m_conn_handle, beeper_svc_notify_hnd.value_handle, &val);
        NRF_LOG_INFO("BEEPER: did HVX");
    }
}

void beeper_init() {
    ret_code_t rv;
    
    /* buttons init */
    nrfx_gpiote_init();
    nrfx_gpiote_in_config_t gpiocfg = {
        .sense = NRF_GPIOTE_POLARITY_TOGGLE,
        .pull = NRF_GPIO_PIN_PULLUP,
    };
    rv = nrfx_gpiote_in_init(P_BTN1, &gpiocfg, _beeper_gpiote);
    APP_ERROR_CHECK(rv);
    nrfx_gpiote_in_event_enable(P_BTN1, 1);
    rv = nrfx_gpiote_in_init(P_BTN2, &gpiocfg, _beeper_gpiote);
    APP_ERROR_CHECK(rv);
    nrfx_gpiote_in_event_enable(P_BTN2, 1);
    rv = nrfx_gpiote_in_init(P_BTN3, &gpiocfg, _beeper_gpiote);
    APP_ERROR_CHECK(rv);
    nrfx_gpiote_in_event_enable(P_BTN3, 1);
    rv = nrfx_gpiote_in_init(P_BTN4, &gpiocfg, _beeper_gpiote);
    APP_ERROR_CHECK(rv);
    nrfx_gpiote_in_event_enable(P_BTN4, 1);
    rv = nrfx_gpiote_in_init(P_BTN5, &gpiocfg, _beeper_gpiote);
    APP_ERROR_CHECK(rv);
    nrfx_gpiote_in_event_enable(P_BTN5, 1);
    rv = nrfx_gpiote_in_init(P_BTN6, &gpiocfg, _beeper_gpiote);
    APP_ERROR_CHECK(rv);
    nrfx_gpiote_in_event_enable(P_BTN6, 1);
    
    /* BLE init */
    
    NRF_SDH_BLE_OBSERVER(beeper_observer, 3, _beeper_handler, NULL);

#define MAKE_UUID(_uuid, ...) \
    do { \
        const ble_uuid128_t _uuid_base = { __VA_ARGS__ }; \
        rv = sd_ble_uuid_vs_add(&_uuid_base, &_uuid.type); \
        APP_ERROR_CHECK(rv); \
        _uuid.uuid = _uuid_base.uuid128[13] << 8 | _uuid_base.uuid128[12]; \
    } while(0)

    /* BC000042-9421-4267-A97B-2C5FAD7B007A */
    MAKE_UUID(beeper_svc_uuid, {
        0x7A, 0x00, 0x7B, 0xAD, 0x5F, 0x2C, 0x7B, 0xA9,
        0x67, 0x42, 0x21, 0x94, 0x42, 0x00, 0x00, 0xBC });
    rv = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &beeper_svc_uuid, &beeper_svc_hnd);
    APP_ERROR_CHECK(rv);

    ble_add_char_params_t params;

    memset(&params, 0, sizeof(params));
    params.uuid = 0x0001;
    params.uuid_type = beeper_svc_uuid.type;
    params.init_len = 1;
    params.max_len = 1;
    params.is_var_len = 0;
    params.p_init_value = &beeper_btn_buf;
    params.char_props.notify = 1;
    params.char_props.read = 1;
    params.read_access = SEC_OPEN /* XXX */;
    params.write_access = SEC_OPEN /* XXX */;
    params.cccd_write_access = SEC_OPEN /* XXX */;

    rv = characteristic_add(beeper_svc_hnd, &params, &beeper_svc_notify_hnd);
    APP_ERROR_CHECK(rv);
    beeper_svc_notify_cccd = 0;

    memset(&params, 0, sizeof(params));
    params.uuid = 0x0002;
    params.uuid_type = beeper_svc_uuid.type;
    params.init_len = 0;
    params.max_len = CMDQ_RXBUF_LEN;
    params.is_var_len = 1;
    params.p_init_value = beeper_cmdq_rxbuf;
    params.char_props.write_wo_resp = 1;
    params.read_access = SEC_OPEN /* XXX */;
    params.write_access = SEC_OPEN /* XXX */;
    params.cccd_write_access = SEC_OPEN /* XXX */;

    rv = characteristic_add(beeper_svc_hnd, &params, &beeper_svc_cmdq_hnd);
    APP_ERROR_CHECK(rv);

}

int beeper_cmdq_ringbuf_take(uint8_t *p, size_t n) {
    CRITICAL_REGION_ENTER();
    if (beeper_cmdq_bytes_used < n) {
        n = -1;
        goto out;
    }
    for (int i = 0; i < n; i++) {
        p[i] = beeper_cmdq_ringbuf[beeper_cmdq_ringbuf_consp++];
        beeper_cmdq_ringbuf_consp %= CMDQ_RINGBUF_LEN;
        beeper_cmdq_bytes_used--;
    }
out:
    CRITICAL_REGION_EXIT();
    return n;
}

/**@brief Function for application main entry.
 */
int main(void)
{
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1,15));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1,10));
    nrf_gpio_pin_write(NRF_GPIO_PIN_MAP(1, 15), 0);
    nrf_gpio_pin_write(NRF_GPIO_PIN_MAP(1, 10), 1);

    NRF_LOG_INIT(NULL);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    LOG_SYNC("BEEPER: beeperband init");

    const uint8_t poweron[] = { 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00 };
    const uint8_t ble_adv[] = { 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t ble_conn[] = { 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF };
    _spim_npxl_setup();
    _spim_npxl_bytes(poweron, sizeof(poweron));
    
    _i2c_drv2605_setup();

    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    beeper_init();
    conn_params_init();
    (void) ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

    enum sys_state last_state = -1;
    while (1) {
        /* main loop: update the LEDs if we've changed system state (should
         * this actually just push commands to the command queue?)
         */
        if (sys_state != last_state) {
            LOG_SYNC("change sys state -> %d", sys_state);
            if (sys_state == STATE_ADVERTISING) {
                _spim_npxl_bytes(ble_adv, sizeof(ble_adv));
            } else if (sys_state == STATE_CONNECTED) {
                _spim_npxl_bytes(ble_conn, sizeof(ble_conn));
            }
            last_state = sys_state;
        }

        /* command to execute? */
        if (beeper_cmdq_bytes_used) {
            switch (beeper_cmdq_ringbuf[beeper_cmdq_ringbuf_consp]) {
            case '\x01': {
                uint8_t npxlbuf[10];
                if (beeper_cmdq_ringbuf_take(npxlbuf, 10) == 10) {
                    LOG_SYNC("BEEPER: set neopixel");
                    _spim_npxl_bytes(npxlbuf + 1, 9);
                }
                break;
            }
            case '\x02': {
                uint8_t buzzbuf[2];
                if (!_i2c_drv2605_is_done())
                    continue;
                if (beeper_cmdq_ringbuf_take(buzzbuf, 2) == 2) {
                    LOG_SYNC("BEEPER: buzz %02x", buzzbuf[1]);
                    _i2c_drv2605_run(buzzbuf[1]);
                }
                break;
            }
            case '\x03': {
                if (!_i2c_drv2605_is_done())
                    continue;
                LOG_SYNC("BEEPER: buzz complete");
                uint8_t c;
                beeper_cmdq_ringbuf_take(&c, 1);
                break;
            }
            /* wait command? */
            /* fade LEDs? */
            default: {
                uint8_t c;
                LOG_SYNC("BEEPER: unknown opcode 0x%02x", beeper_cmdq_ringbuf[beeper_cmdq_ringbuf_consp]);
                beeper_cmdq_ringbuf_take(&c, 1);
                break;
            }
            }
        }
        idle_state_handle();
    }
}
