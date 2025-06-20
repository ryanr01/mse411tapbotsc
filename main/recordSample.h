#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/i2s_pdm.h"
#include "driver/gpio.h"
#include "math.h"
#include "format_wav.h"
#include "esp_timer.h"
#include "driver/i2s_std.h"
#include <time.h>

// Initialize solenoid pins
#define ENB 4
#define IN3 17
#define IN4 8

// MIC GPIO Pins
#define MIC_I2S_CLK_GPIO 40
#define MIC_I2S_DATA_GPIO 19
#define MIC_I2S_LRCL 5 // A.k.A. WS "Word Select"

// SPI2 GPIO pins
#define PIN_NUM_MISO 13
#define PIN_NUM_MOSI 11
#define PIN_NUM_CLK  12
#define PIN_NUM_CS   10

// Globals... shh, don't tell on me
double adjusted_rec_time;
char elnome[128];
int record_millis;

// RTOS configuration
void record_wav(void * pvParameters);
void fire_solenoid_once(void * pvParameters);

void RTOS_go() {
    // Create task one on core 0
    TaskHandle_t record_handle;
    xTaskCreatePinnedToCore(
        record_wav,           // Task function
        "RecordWav",         // Task name
        10000,             // Stack size in words
        NULL,              // Task parameters
        1,                 // Priority
        &record_handle,              // Task handle
        0                  // Core 0
    );

    vTaskDelay(50);

    // Create task two on core 1
    TaskHandle_t fire_handle;
    xTaskCreatePinnedToCore(
        fire_solenoid_once,           // Task function
        "FireSolenoidOnce",         // Task name
        10000,             // Stack size in words
        NULL,              // Task parameters
        1,                 // Priority
        &fire_handle,              // Task handle
        1                  // Core 1
    );

    vTaskDelay(record_millis + 200); // Wait for recording to finish + added buffer
    vTaskDelete(fire_handle);
    vTaskDelete(record_handle);
}

//********************************solenoid functions********************************
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
void fire_solenoid_once(void * pvParameters)
{
    printf("FIRE FIRE FIRE FIRE FIRE FIRE");
    gpio_set_level(IN3, 1);
    gpio_set_level(IN4, 0);
    gpio_set_level(ENB, 1);
    vTaskDelay(pdMS_TO_TICKS(250)); //250 miliseconds down

    gpio_set_level(ENB, 0);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 0);
    vTaskDelay(pdMS_TO_TICKS(500)); // 500 miliseconds up
    while(1);
}

#define TAG "SD_SPI"
#define MOUNT_POINT "/s"

#define MAX_CHAR_SIZE 64

#define MIC_SAMPLE_RATE 48000 // Do not exceed 64000 for SPH0645
#define MIC_BIT_SAMPLE 32
#define RECORD_TIME_MS 200
#define SAMPLES_TO_RECORD  ((MIC_SAMPLE_RATE * RECORD_TIME_MS) / 1000)
#define SPI_DMA_CHAN        SPI_DMA_CH_AUTO
#define NUM_CHANNELS        2
#define SAMPLE_SIZE         (MIC_BIT_SAMPLE * 1024) // RAM buffer size
#define BYTE_RATE           (MIC_SAMPLE_RATE * (MIC_BIT_SAMPLE / 8)) * NUM_CHANNELS

i2s_chan_handle_t rx_handle;
i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

/* Allocate a new RX channel and get the handle of this channel */
static int32_t i2s_readraw_buff[SAMPLE_SIZE]; // This line controls the RAM buffer size - each chunk is stored in a RAM buffer before it fills and is sent to the SD card
size_t bytes_read;
const int WAVE_HEADER_SIZE = 44;
struct timespec ts;

// This function records from microphone given duration_ms and saves the resultant
// recording in .wav format to the mounted SD card.
// Note that this function does not perform any configuration, that must be done before
// this function is called.
// This function assumes that the esp 32 and sd card are in a state that the
// sd card is ready to be written to.
void record_wav(void * pvParameters)
{
    // Use POSIX and C standard library functions to work with files.
    int flash_wr_size = 0;
    ESP_LOGI(TAG, "Opening file");

    uint32_t flash_rec_time = BYTE_RATE * (uint32_t)adjusted_rec_time; // 2/3 multiple tells wav header to be the correct number of bits (we want to exclude redundant bits 19 - 32)
    const wav_header_t wav_header =
        WAV_HEADER_PCM_DEFAULT(flash_rec_time, MIC_BIT_SAMPLE, MIC_SAMPLE_RATE, NUM_CHANNELS);

    // First check if file exists before creating a new file.
    struct stat st;

    // Start building file path
    char filepath[256];
    snprintf(filepath, sizeof(filepath), "%s/%s.wav", MOUNT_POINT, elnome);
    // End of filepath building

    if (stat(filepath, &st) == 0) {
        // Delete it if it exists
        unlink(filepath);
    }

    // Create new WAV file with debug logging
    ESP_LOGI(TAG, "Attempting to open file: %s", filepath);

    FILE *f = fopen(filepath, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    // Write the header to the WAV file
    fwrite(&wav_header, sizeof(wav_header), 1, f);

    // Create task to call fire_solenoid asynchronously
    //static uint8_t ucParameterToPass;
   // TaskHandle_t async_tap = NULL;

    // Start recording
    int bit_counter = 1; // Use this counter to exclude redundant bits
   // bool trigger_flag = 0;
    while (flash_wr_size < flash_rec_time) {
        // Read the RAW samples from the microphone
        if (i2s_channel_read(rx_handle, (char *)i2s_readraw_buff, SAMPLE_SIZE, &bytes_read, 1000) == ESP_OK) {
            printf("[0] %ld [1] %ld [2] %ld [3]%ld ...\n", i2s_readraw_buff[0], i2s_readraw_buff[1], i2s_readraw_buff[2], i2s_readraw_buff[3]);
            // Write the samples to the WAV file
            fwrite(i2s_readraw_buff, bytes_read, 1, f);
            flash_wr_size += bytes_read;
            bit_counter += 1;

            // Trigger solenoid halfway through recording
            //if(((float)flash_wr_size / (float)flash_rec_time) >= 0.5 && trigger_flag == 0) {
               // trigger_flag = 1; // Reset trigger flag
                //xTaskCreatePinnedToCore( fire_solenoid_once, "NAME", STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY, &async_tap, 1);
                //configASSERT( async_tap );
           // }
        } else {
            printf("Read Failed!\n");
        }
    }

    // Use the handle to delete the task.
    //if( async_tap != NULL )
    //{
     //vTaskDelete( async_tap );
    //}

    ESP_LOGI(TAG, "Recording done!");
    fclose(f);
    ESP_LOGI(TAG, "File written on SDCard");
    while(1);
}

// This function initializes the microphone for use
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
        .slot_cfg.ws_width = 32, // ws_width of 32 (per half cycle translates to 64 bclk cycles per complete ws cycle)
        .slot_cfg.slot_mask = I2S_STD_SLOT_BOTH, // Both slots indicates WS both high and low to signal data
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

void replace_char(char *str, char old_char, char new_char) {
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
    host.slot = SPI2_HOST; // VSPI

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

    // Initialize microphone
    init_microphone();

    // Initialize solenoid
    solenoid_init();

    // Build file name
    char xcoordstr[32];
    char ycoordstr[32];
    char timestr[32];

    snprintf(xcoordstr, sizeof(xcoordstr), "%.2f", x_coordinate);
    replace_char(xcoordstr, '.', '\0');

    snprintf(ycoordstr, sizeof(ycoordstr), "%.2f", y_coordinate);
    replace_char(ycoordstr, '.', '\0');

    snprintf(timestr, sizeof(timestr), "%ld", (long)tv_now.tv_sec);

    snprintf(elnome, sizeof(elnome), "%s%s%s%s", data_label, xcoordstr, ycoordstr, timestr);

    adjusted_rec_time = (double)record_time / 1000.0 * 3.0 / 3.0; // Adjust as needed
    
    RTOS_go(); // Start record and tap tasks

    // Unmount card
    esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
    ESP_LOGI(TAG, "Card unmounted");

    spi_bus_free(host.slot);
}