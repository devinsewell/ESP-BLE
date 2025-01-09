/*
 * Author: Devin Sewell
 * Date: January 9, 2025
 * 
 * This code is a rewrite based on the NimBLE example source, designed as a quick prototyping template for 
 * simple BLE functionality including GATT-based read, write, and notify operations, as well as BLE advertising data setup.
 */

#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define TAG "LuminaSet-BLE"

// Global variables for GATT characteristic value and handle
static uint8_t gatt_svr_chr_val[4];
static uint16_t gatt_svr_chr_val_handle;

// Function declarations for GATT and BLE event handling
static int gatt_svr_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static void bleprph_host_task(void *param);
static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
static int gatt_svr_descr_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);
static void bleprph_on_reset(int reason);
static void bleprph_on_sync(void);

// GATT Service and Characteristic UUIDs
static const ble_uuid128_t gatt_svr_svc_uuid = BLE_UUID128_INIT(0xDA, 0xBD, 0xE8, 0xFC, 0x9E, 0xDE, 0x46, 0xB2, 0x0C, 0x4A, 0x5F, 0xD6, 0xFF, 0xFF, 0xE1, 0xA0);
static const ble_uuid128_t gatt_svr_chr_uuid = BLE_UUID128_INIT(0xDA, 0xBD, 0xE8, 0xFC, 0x9E, 0xDE, 0x46, 0xB2, 0x0C, 0x4A, 0x5F, 0xD6, 0xFF, 0xFF, 0xE1, 0xA1);
static const ble_uuid16_t gatt_svr_chr_descr_uuid = BLE_UUID16_INIT(0x2901);

// Descriptor definitions
static const struct ble_gatt_dsc_def gatt_svr_chr_descr[] = {
    {
        .uuid = &gatt_svr_chr_descr_uuid.u,
        .att_flags = BLE_ATT_F_READ,
        .access_cb = gatt_svr_descr_access_cb,
        .arg = "Device RX/TX API",
    },
    { 0 }, // End of descriptors
};

// Characteristic definitions
static const struct ble_gatt_chr_def gatt_svr_chr[] = {
    {
        .uuid = &gatt_svr_chr_uuid.u,
        .access_cb = gatt_svr_chr_access_cb,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &gatt_svr_chr_val_handle,
        .descriptors = (struct ble_gatt_dsc_def *)gatt_svr_chr_descr,
    },
    { 0 }, // End of characteristics
};

// Service definitions
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = gatt_svr_chr,
    },
    { 0 }, // End of services
};

// Callback for registering GATT services, characteristics, and descriptors
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    char buf[BLE_UUID_STR_LEN];
    switch (ctxt->op) {
        case BLE_GATT_REGISTER_OP_SVC:
            ESP_LOGI(TAG, "Service registered: %s", ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf));
            break;
        case BLE_GATT_REGISTER_OP_CHR:
            ESP_LOGI(TAG, "Characteristic registered: %s", ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf));
            break;
        case BLE_GATT_REGISTER_OP_DSC:
            ESP_LOGI(TAG, "Descriptor registered: %s", ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf));
            break;
        default:
            ESP_LOGE(TAG, "Unknown registration event");
    }
}

// Descriptor access callback for read/write operations
static int gatt_svr_descr_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    const char *descr_value = (const char *)arg;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_DSC:
            return os_mbuf_append(ctxt->om, descr_value, strlen(descr_value));
        case BLE_GATT_ACCESS_OP_WRITE_DSC:
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// Characteristic access callback for read/write operations
static int gatt_svr_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    int rc;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            rc = os_mbuf_append(ctxt->om, gatt_svr_chr_val, sizeof(gatt_svr_chr_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            rc = ble_hs_mbuf_to_flat(ctxt->om, gatt_svr_chr_val, sizeof(gatt_svr_chr_val), NULL);
            if (rc == 0) {
                ESP_LOGI(TAG, "Characteristic written");
            }
            return rc;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// Callback for BLE stack reset
static void bleprph_on_reset(int reason) {
    ESP_LOGE(TAG, "Resetting BLE state; reason=%d", reason);
}

// Callback for BLE stack synchronization
static void bleprph_on_sync(void) {
    int rc;

    // Configure advertisement data
    struct ble_hs_adv_fields adv_fields = {0};
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // Set Model 
    adv_fields.name = (uint8_t *)"RGB v1_0_0";
    adv_fields.name_len = strlen("RGB v1_0_0");
    adv_fields.name_is_complete = 1;

    // Set Manufacturer 
    const uint8_t manufacturer_data[] = {'L', 'u', 'm', 'i', 'n', 'a', 'S', 'e', 't'};
    adv_fields.mfg_data = manufacturer_data;
    adv_fields.mfg_data_len = sizeof(manufacturer_data);

    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertisement data; rc=%d", rc);
        return;
    }

    // Start advertising
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };

    rc = ble_gap_adv_start(0, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising; rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising started");
}

// BLE host task for handling FreeRTOS integration
void bleprph_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// Initialize GATT server and register services
static int gatt_svr_init(void) {
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) return rc;

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    return rc;
}

// Callback for handling GAP events
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "Central connected. Connection handle: %d", event->connect.conn_handle);
            } else {
                ESP_LOGE(TAG, "Connection failed. Status: %d", event->connect.status);
                bleprph_on_sync(); // Restart advertising on failure
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGW(TAG, "Central disconnected. Reason: %d", event->disconnect.reason);
            bleprph_on_sync(); // Restart advertising on disconnection
            break;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Advertising completed. Restarting.");
            bleprph_on_sync();
            break;
        default:
            ESP_LOGD(TAG, "Unhandled GAP event: %d", event->type);
            break;
    }
    return 0;
}

// Main
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.reset_cb = bleprph_on_reset;
    ble_hs_cfg.sync_cb = bleprph_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;

    ESP_ERROR_CHECK(gatt_svr_init());
    ESP_ERROR_CHECK(ble_svc_gap_device_name_set("LuminaSet-BLE"));
    nimble_port_freertos_init(bleprph_host_task);

    ESP_LOGI(TAG, "Application started.");
}
