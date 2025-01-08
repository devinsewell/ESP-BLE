#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define TAG "LuminaSet-BLE"

static uint8_t gatt_svr_chr_val[4];
static uint16_t gatt_svr_chr_val_handle;

static int gatt_svr_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static void bleprph_host_task(void *param);
static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
static int gatt_svr_descr_access_cb(uint16_t conn_handle, uint16_t attr_handle,struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);
static void bleprph_on_reset(int reason);
static void bleprph_on_sync(void);

// GATT Service UUID
static const ble_uuid128_t gatt_svr_svc_uuid = BLE_UUID128_INIT(0xDA, 0xBD, 0xE8, 0xFC, 0x9E, 0xDE, 0x46, 0xB2, 0x0C, 0x4A, 0x5F, 0xD6, 0xFF, 0xFF, 0xE1, 0xA0);

// GATT Characteristic UUID
static const ble_uuid128_t gatt_svr_chr_uuid = BLE_UUID128_INIT(0xDA, 0xBD, 0xE8, 0xFC, 0x9E, 0xDE, 0x46, 0xB2, 0x0C, 0x4A, 0x5F, 0xD6, 0xFF, 0xFF, 0xE1, 0xA1);

// Static UUID for the Characteristic Descriptor
static const ble_uuid16_t gatt_svr_chr_descr_uuid = BLE_UUID16_INIT(0x2901);

static const struct ble_gatt_dsc_def gatt_svr_chr_descr[] = {
    {
        .uuid = &gatt_svr_chr_descr_uuid.u, // Reference the UUID object
        .att_flags = BLE_ATT_F_READ,
        .access_cb = gatt_svr_descr_access_cb, // Callback for descriptor read/write
        .arg = "Device RX/TX API", // Optional argument
    },
    {
        0, // No more descriptors
    },
};

static const struct ble_gatt_chr_def gatt_svr_chr[] = {
    {
        .uuid = &gatt_svr_chr_uuid.u,
        .access_cb = gatt_svr_chr_access_cb,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &gatt_svr_chr_val_handle,
        .descriptors = (struct ble_gatt_dsc_def *)gatt_svr_chr_descr,
    },
    { 0 },
};

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = gatt_svr_chr,
    },
    { 0 },
};

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

static int gatt_svr_descr_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    const char *descr_value = (const char *)arg;  // Retrieve description from the arg field
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_DSC:
        return os_mbuf_append(ctxt->om, descr_value, strlen(descr_value));

    case BLE_GATT_ACCESS_OP_WRITE_DSC:
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static int gatt_svr_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    int rc;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        rc = os_mbuf_append(ctxt->om, gatt_svr_chr_val, sizeof(gatt_svr_chr_val));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        rc = ble_hs_mbuf_to_flat(ctxt->om, gatt_svr_chr_val, sizeof(gatt_svr_chr_val), NULL);
        if (rc == 0) {
            // Handle gatt_svr_chr_val data here
            ESP_LOGI(TAG, "BLE_GATT_ACCESS_OP_WRITE_CHR");
        }
        return rc;

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static void bleprph_on_reset(int reason) {
    ESP_LOGE(TAG, "Resetting state; reason=%d", reason);
}

static void bleprph_on_sync(void) {
    int rc;

    // Set advertisement data
    struct ble_hs_adv_fields adv_fields = {0};
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.name = (uint8_t *)"RGB v1_0_0";
    adv_fields.name_len = strlen("RGB v1_0_0");
    adv_fields.name_is_complete = 1;

    // Set Manufacturer name
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


void bleprph_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static int gatt_svr_init(void) {
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) return rc;

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    return rc;
}

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "Central connected. Connection handle: %d", event->connect.conn_handle);
            } else {
                ESP_LOGE(TAG, "Connection failed. Status: %d", event->connect.status);
                bleprph_on_sync(); // Restart advertising on failed connection
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGW(TAG, "Central disconnected. Reason: %d", event->disconnect.reason);
            bleprph_on_sync(); // Restart advertising on disconnection
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Advertising completed. Restarting advertising.");
            bleprph_on_sync();
            break;

        default:
            ESP_LOGD(TAG, "Unhandled GAP event: %d", event->type);
            break;
    }
    return 0;
}

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
    ESP_ERROR_CHECK(ble_svc_gap_device_name_set("LuminaSet-RGBKit"));
    nimble_port_freertos_init(bleprph_host_task);

    ESP_LOGI(TAG, "Application started.");
}
