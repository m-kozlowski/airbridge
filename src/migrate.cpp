#include "migrate.h"
#include "debug_log.h"
#include "target_ptable.h"

#include <esp_partition.h>
#include <esp_ota_ops.h>
#include <esp_flash_partitions.h>
#include <esp_system.h>
#include <esp_flash.h>
#include <esp_flash_internal.h>
#include <string.h>

#define PTABLE_OFFSET   0x8000
#define PTABLE_SIZE     0x1000
#define OTADATA_OFFSET  0xe000
#define OTADATA_SIZE    0x2000

static int count_target_entries() {
    for (size_t i = 0; i + 1 < TARGET_PTABLE_SIZE; i += sizeof(esp_partition_info_t)) {
        if (TARGET_PTABLE[i] == 0xEB && TARGET_PTABLE[i + 1] == 0xEB)
            return i / sizeof(esp_partition_info_t);
    }
    return TARGET_PTABLE_SIZE / sizeof(esp_partition_info_t);
}

// Check if current flash partition table matches target
bool Migrate::needed() {
    size_t entries_len = count_target_entries() * sizeof(esp_partition_info_t);

    uint8_t *flash_buf = (uint8_t*)malloc(entries_len);
    if (!flash_buf) return true;

    esp_err_t err = esp_flash_read(NULL, flash_buf, PTABLE_OFFSET, entries_len);
    if (err != ESP_OK) { free(flash_buf); return true; }

    bool differs = memcmp(flash_buf, TARGET_PTABLE, entries_len) != 0;
    free(flash_buf);
    return differs;
}

// Copy src partition to dst partition
static bool copy_partition(const esp_partition_t *dst, const esp_partition_t *src, size_t size) {
    const size_t BLK = 4096;
    uint8_t *buf = (uint8_t*)malloc(BLK);
    if (!buf) {
        Log::logf(CAT_OTA, LOG_ERROR, "[MIG] copy_partition malloc failed\n");
        return false;
    }

    esp_err_t err = esp_partition_erase_range(dst, 0, dst->size);
    if (err != ESP_OK) {
        Log::logf(CAT_OTA, LOG_ERROR, "[MIG] Erase dst failed: %s\n", esp_err_to_name(err));
        free(buf);
        return false;
    }

    size_t offset = 0;
    while (offset < size) {
        size_t chunk = ((size - offset) > BLK) ? BLK : (size - offset);
        err = esp_partition_read(src, offset, buf, chunk);
        if (err != ESP_OK) {
            Log::logf(CAT_OTA, LOG_ERROR, "[MIG] Read src @%u failed: %s\n", offset, esp_err_to_name(err));
            free(buf);
            return false;
        }
        err = esp_partition_write(dst, offset, buf, chunk);
        if (err != ESP_OK) {
            Log::logf(CAT_OTA, LOG_ERROR, "[MIG] Write dst @%u failed: %s\n", offset, esp_err_to_name(err));
            free(buf);
            return false;
        }
        offset += chunk;
    }

    offset = 0;
    while (offset < size) {
        size_t chunk = ((size - offset) > BLK) ? BLK : (size - offset);
        err = esp_partition_read(dst, offset, buf, chunk);
        if (err != ESP_OK) {
            Log::logf(CAT_OTA, LOG_ERROR, "[MIG] Verify read dst @%u failed\n", offset);
            free(buf);
            return false;
        }
        uint32_t dst_sum = 0;
        for (size_t i = 0; i < chunk; i++) dst_sum += buf[i];

        err = esp_partition_read(src, offset, buf, chunk);
        if (err != ESP_OK) {
            Log::logf(CAT_OTA, LOG_ERROR, "[MIG] Verify read src @%u failed\n", offset);
            free(buf);
            return false;
        }
        uint32_t src_sum = 0;
        for (size_t i = 0; i < chunk; i++) src_sum += buf[i];

        if (dst_sum != src_sum) {
            Log::logf(CAT_OTA, LOG_ERROR, "[MIG] Verify failed @%u\n", offset);
            free(buf);
            return false;
        }
        offset += chunk;
    }

    free(buf);
    return true;
}

void Migrate::run(void (*lct)(const char *msg)) {
    if (!needed()) {
        Log::logf(CAT_OTA, LOG_INFO, "[MIG] Partition table already at target layout\n");
        return;
    }

    const esp_partition_t *running = esp_ota_get_running_partition();
    if (!running) {
        Log::logf(CAT_OTA, LOG_ERROR, "[MIG] Cannot determine running partition\n");
        return;
    }

    Log::logf(CAT_OTA, LOG_INFO, "[MIG] Running from %s (0x%X)\n",
              running->label, running->address);

    // sync ota slots if not on app0
    if (running->address != 0x10000) {
        if (lct) lct("SLOT SYNC");
        Log::logf(CAT_OTA, LOG_INFO, "[MIG] Stage 1: Copying self from %s to app0\n",
                  running->label);

        // Find app0 by address using the CURRENT partition table
        const esp_partition_t *app0 = nullptr;
        esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP,
                                                         ESP_PARTITION_SUBTYPE_ANY, NULL);
        while (it) {
            const esp_partition_t *p = esp_partition_get(it);
            if (p->address == 0x10000) {
                app0 = p;
                break;
            }
            it = esp_partition_next(it);
        }
        esp_partition_iterator_release(it);

        if (!app0) {
            Log::logf(CAT_OTA, LOG_ERROR, "[MIG] Cannot find app0 partition at 0x10000\n");
            return;
        }

        size_t fw_size = running->size;

        if (!copy_partition(app0, running, fw_size)) {
            Log::logf(CAT_OTA, LOG_ERROR, "[MIG] Slot sync FAILED. Aborting.\n");
            if (lct) lct("SYNC FAIL");
            return;
        }

        esp_err_t err = esp_ota_set_boot_partition(app0);
        if (err != ESP_OK) {
            Log::logf(CAT_OTA, LOG_ERROR, "[MIG] Set boot partition failed: %s\n",
                      esp_err_to_name(err));
            if (lct) lct("SYNC FAIL");
            return;
        }

        Log::logf(CAT_OTA, LOG_INFO, "[MIG] Slot sync complete. Rebooting to app0...\n");
        if (lct) lct("REBOOTING");
        delay(500);
        esp_restart();
    }

    // Write new partition table
    if (lct) lct("REPARTITIONING");
    Log::logf(CAT_OTA, LOG_INFO, "[MIG] Stage 2: Writing new partition table\n");

    uint8_t *table_image = (uint8_t*)malloc(PTABLE_SIZE);
    if (!table_image) {
        Log::logf(CAT_OTA, LOG_ERROR, "[MIG] malloc failed\n");
        return;
    }
    memset(table_image, 0xFF, PTABLE_SIZE);
    memcpy(table_image, TARGET_PTABLE,
           TARGET_PTABLE_SIZE < PTABLE_SIZE ? TARGET_PTABLE_SIZE : PTABLE_SIZE);

    // Validate using esp-idf verifier
    int num_parts = 0;
    esp_err_t verify = esp_partition_table_verify(
        (const esp_partition_info_t*)table_image, true, &num_parts);
    if (verify != ESP_OK) {
        Log::logf(CAT_OTA, LOG_ERROR, "[MIG] Generated table INVALID: %s\n",
                  esp_err_to_name(verify));
        free(table_image);
        if (lct) lct("TABLE ERR");
        return;
    }
    Log::logf(CAT_OTA, LOG_INFO, "[MIG] Table validated (%d partitions)\n", num_parts);

    // Disable write protection
    esp_err_t err;
    err = esp_flash_set_dangerous_write_protection(esp_flash_default_chip, false);
    if (err != ESP_OK) {
        Log::logf(CAT_OTA, LOG_ERROR, "[MIG] Cannot disable write protect: %s\n",
                  esp_err_to_name(err));
        free(table_image);
        return;
    }

    Log::logf(CAT_OTA, LOG_INFO, "[MIG] Erasing ptable sector\n");
    err = esp_flash_erase_region(NULL, PTABLE_OFFSET, PTABLE_SIZE);
    if (err != ESP_OK) {
        Log::logf(CAT_OTA, LOG_ERROR, "[MIG] Erase ptable failed: %s\n", esp_err_to_name(err));
        free(table_image);
        if (lct) lct("ERASE FAIL");
        return;
    }

    Log::logf(CAT_OTA, LOG_INFO, "[MIG] Writing ptable\n");
    err = esp_flash_write(NULL, table_image, PTABLE_OFFSET, PTABLE_SIZE);
    if (err != ESP_OK) {
        Log::logf(CAT_OTA, LOG_ERROR, "[MIG] Write ptable failed: %s\n", esp_err_to_name(err));
        free(table_image);
        if (lct) lct("WRITE FAIL");
        return;
    }

    Log::logf(CAT_OTA, LOG_INFO, "[MIG] Verifying ptable\n");
    size_t entries_len = count_target_entries() * sizeof(esp_partition_info_t);
    uint8_t *verify_buf = (uint8_t*)malloc(entries_len);
    if (verify_buf) {
        esp_flash_read(NULL, verify_buf, PTABLE_OFFSET, entries_len);
        if (memcmp(verify_buf, TARGET_PTABLE, entries_len) != 0) {
            Log::logf(CAT_OTA, LOG_ERROR, "[MIG] Partition table verify FAILED!\n");
            free(verify_buf);
            free(table_image);
            if (lct) lct("VERIFY FAIL");
            return;
        }
        free(verify_buf);
    }
    free(table_image);

    Log::logf(CAT_OTA, LOG_INFO, "[MIG] Erasing otadata\n");
    esp_flash_erase_region(NULL, OTADATA_OFFSET, OTADATA_SIZE);

    esp_flash_set_dangerous_write_protection(esp_flash_default_chip, true);

    Log::logf(CAT_OTA, LOG_INFO, "[MIG] Partition table written and verified\n");

    Log::logf(CAT_OTA, LOG_INFO, "[MIG] Migration complete. Rebooting with new layout...\n");
    if (lct) lct("REBOOTING");
    delay(500);
    esp_restart();
}
