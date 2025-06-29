#include "recordSample.h"

static double adjusted_rec_time;
static char elnome[128];
static int record_millis;

void RTOS_go(void) {
    TaskHandle_t record_handle;
    xTaskCreatePinnedToCore(
        record_wav,
        "RecordWav",
        10000,
        NULL,
        1,
        &record_handle,
        0
    );

    vTaskDelay(50);

    TaskHandle_t fire_handle;
    xTaskCreatePinnedToCore(
        fire_solenoid_once,
        "FireSolenoidOnce",
        10000,
        NULL,
        1,
        &fire_handle,
        1
    );

    vTaskDelay(record_millis + 200);
    vTaskDelete(fire_handle);
    vTaskDelete(record_handle);
}

void solenoid_init(void)
{
    gpio_reset_pin(ENB);
    gpio_reset_pin(IN3);
    gpio_reset_pin(IN4);
    gpio_set_direction(ENB, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN3, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN4, GPIO_MODE_OUTPUT);
    gpio_set_level(ENB, 0);
    gpio_set_level(IN3, 1);
    gpio_set_level(IN4, 0);
}

void fire_solenoid_once(void *pvParameters)
{
    printf("FIRE FIRE FIRE FIRE FIRE FIRE");
    gpio_set_level(IN3, 1);
    gpio_set_level(IN4, 0);
    gpio_set_level(ENB, 1);
    vTaskDelay(pdMS_TO_TICKS(250));

    gpio_set_level(ENB, 0);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    while (1);
}

static i2s_chan_handle_t rx_handle;
static int32_t i2s_readraw_buff[SAMPLE_SIZE];
static size_t bytes_read;

void record_wav(void *pvParameters)
{
    int flash_wr_size = 0;
    ESP_LOGI(TAG, "Opening file");

    uint32_t flash_rec_time = BYTE_RATE * (uint32_t)adjusted_rec_time;
    const wav_header_t wav_header =
        WAV_HEADER_PCM_DEFAULT(flash_rec_time, MIC_BIT_SAMPLE, MIC_SAMPLE_RATE, NUM_CHANNELS);

    struct stat st;
    char filepath[256];
    snprintf(filepath, sizeof(filepath), "%s/%s.wav", MOUNT_POINT, elnome);

    if (stat(filepath, &st) == 0) {
        unlink(filepath);
    }

    ESP_LOGI(TAG, "Attempting to open file: %s", filepath);

    FILE *f = fopen(filepath, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    fwrite(&wav_header, sizeof(wav_header), 1, f);

    int bit_counter = 1;
    while (flash_wr_size < flash_rec_time) {
        if (i2s_channel_read(rx_handle, (char *)i2s_readraw_buff, SAMPLE_SIZE, &bytes_read, 1000) == ESP_OK) {
            printf("[0] %ld [1] %ld [2] %ld [3]%ld ...\n", i2s_readraw_buff[0], i2s_readraw_buff[1], i2s_readraw_buff[2], i2s_readraw_buff[3]);
            fwrite(i2s_readraw_buff, bytes_read, 1, f);
            flash_wr_size += bytes_read;
            bit_counter += 1;
        } else {
            printf("Read Failed!\n");
        }
    }

    ESP_LOGI(TAG, "Recording done!");
    fclose(f);
    ESP_LOGI(TAG, "File written on SDCard");
    while (1);
}

void init_microphone(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));

    i2s_std_config_t std_cfg = {
        .clk_cfg.sample_rate_hz = MIC_SAMPLE_RATE,
        .clk_cfg.clk_src = I2C_CLK_SRC_DEFAULT,
        .clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256,
        .slot_cfg.slot_mode = I2S_SLOT_MODE_MONO,
        .slot_cfg.slot_bit_width = MIC_BIT_SAMPLE,
        .slot_cfg.data_bit_width = MIC_BIT_SAMPLE,
        .slot_cfg.ws_width = 32,
        .slot_cfg.slot_mask = I2S_STD_SLOT_BOTH,
        .slot_cfg.bit_shift = true,
        .slot_cfg.left_align = true,
        .slot_cfg.ws_pol = true,
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = MIC_I2S_CLK_GPIO,
            .ws = MIC_I2S_LRCL,
            .dout = I2S_GPIO_UNUSED,
            .din = MIC_I2S_DATA_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
    ESP_LOGI(TAG, "I2S initialized.");
}

void replace_char(char *str, char old_char, char new_char)
{
    if (str == NULL) {
        return;
    }
    for (int i = 0; str[i] != '\0'; i++) {
        if (str[i] == old_char) {
            str[i] = new_char;
        }
    }
}

void record_sample(int record_time, char *data_label, float x_coordinate, float y_coordinate)
{
    struct timespec tv_now;
    if (clock_gettime(CLOCK_REALTIME, &tv_now) != 0) {
        ESP_LOGE(TAG, "Failed to get time");
        return;
    }

    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing SD card over SPI");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 65536,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 65536
    };

    sdmmc_card_t *card;
    ESP_LOGI(TAG, "Mounting filesystem...");
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount filesystem: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Filesystem mounted");
    sdmmc_card_print_info(stdout, card);

    init_microphone();
    solenoid_init();

    char xcoordstr[32];
    char ycoordstr[32];
    char timestr[32];

    snprintf(xcoordstr, sizeof(xcoordstr), "%.2f", x_coordinate);
    replace_char(xcoordstr, '.', '\0');

    snprintf(ycoordstr, sizeof(ycoordstr), "%.2f", y_coordinate);
    replace_char(ycoordstr, '.', '\0');

    snprintf(timestr, sizeof(timestr), "%ld", (long)tv_now.tv_sec);

    snprintf(elnome, sizeof(elnome), "%s%s%s%s", data_label, xcoordstr, ycoordstr, timestr);

    adjusted_rec_time = (double)record_time / 1000.0;
    record_millis = record_time;

    RTOS_go();

    esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
    ESP_LOGI(TAG, "Card unmounted");

    spi_bus_free(host.slot);
}

