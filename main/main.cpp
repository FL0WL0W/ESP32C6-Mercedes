#include <cstdint>
#include <stdio.h>
#include <string.h>
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "hal/gpio_hal.h"
#include "hal/wdt_hal.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "mount.h"
#include "Esp32IdfCommunicationService_UART.h"
#include "http_server.h"
#include <ATTiny_UPDI.h>
#include "hal/uart_ll.h"
#include "driver/gpio.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "ExpanderMain.h"
#include "ServiceRegistry.h"
#include "Variable.h"
#include "CallBack.h"
#include "Config.h"
#include "CommunicationHandler_Prefix.h"
#include "CommunicationHandlers/CommunicationHandler_EFIGenie.h"
#include "CommunicationHandlers/CommunicationHandler_OBD2PIDResponder.h"
#include "AggregateCommunicationService.h"
#include "AnalogService_Mercedes.h"
#include "DigitalService_Mercedes.h"
#include "PwmService_Mercedes.h"
#include "ATTiny427_ExpanderService.h"
#include "AnalogService_ATTiny427Expander.h"
#include "DigitalService_ATTiny427Expander.h"
#include "PwmService_ATTiny427Expander.h"
#include "Esp32IdfAnalogService.h"
#include "Esp32IdfDigitalService.h"
#include "Esp32IdfTimerService.h"
#include "Esp32IdfPwmService.h"
#include "Esp32IdfCANService.h"
#include "Esp32IdfCommunicationService_Socket.h"
#include "Esp32IdfCommunicationService_WebSocket.h"

using namespace OperationArchitecture;
using namespace EmbeddedIOServices;
using namespace EmbeddedIOOperations;
using namespace Esp32;
using namespace EFIGenie;

#define UPDI_UART_RX_PIN 18
#define UPDI_UART_TX_PIN 14

#define ATTINY_TX 16
#define ATTINY_RX 17

ICommunicationService *_uartCommServices[UART_NUM_MAX] = {nullptr};

extern void RegisterCANTasks();

extern "C"
{
    //read UPDI byte
    bool UPDI_Read(uint8_t *val)
    {
        uint8_t timeout = 0;
        while(UPDI_rx_buffer_length < 1 && timeout++ < 100)
            vTaskDelay(1);
        if(timeout > 99){
            return false;
        }
        // ESP_LOGE("UPDI", "read %x %d %d %d", UPDI_rx_buffer[UPDI_rx_buffer_index], UPDI_rx_buffer_index, UPDI_rx_buffer_length, timeout);
        *val = UPDI_rx_buffer[UPDI_rx_buffer_index++];
        UPDI_rx_buffer_length--;
        UPDI_rx_buffer_index %= UPDI_RX_BUFFER_LENGTH;
        return true;
    }

    //write UPDI byte
    bool UPDI_Write(uint8_t val)
    {
        if(UPDI_uart_num >= UART_NUM_MAX || _uartCommServices[UPDI_uart_num] == nullptr)
            return false;
        _uartCommServices[UPDI_uart_num]->Send(&val, 1);
        uint8_t verify;
        if(!UPDI_Read(&verify))
            return false;
        return verify == val;
    }

    //UPDI send Break
    void UPDI_Break()
    {
        uint32_t baudrate = 100000;

        uart_wait_tx_done(UPDI_uart_num, pdMS_TO_TICKS(30)); //wait for all bytes to be flushed
        uart_get_baudrate(UPDI_uart_num, &baudrate);
        uart_set_baudrate(UPDI_uart_num, 300);
        UPDI_Write(0);  // send a zero byte
        uart_wait_tx_done(UPDI_uart_num, pdMS_TO_TICKS(30)); // wait for 0 byte to finish before restore normal baudrate
        uart_set_baudrate(UPDI_uart_num, baudrate); // set baudrate back to normal after break is sent
    }

    void UPDI_Idle()
    {
        uint32_t baudrate = 100000;
        uart_get_baudrate(UPDI_uart_num, &baudrate);
        esp_rom_delay_us(15000000/baudrate);
    }

    void UPDI_Enable_Task(void *arg) {
        vTaskDelay(pdMS_TO_TICKS(100));
        UPDI_Enable((uart_port_t)1, (gpio_num_t)UPDI_UART_TX_PIN, (gpio_num_t)UPDI_UART_RX_PIN);
        vTaskDelete(NULL);
    }

    uint8_t updi_enabled = 0;
    bool UPDI_RX_Hook(const uint8_t *data, size_t len)
    {
        if(updi_enabled != 1) {
            xTaskCreate(UPDI_Enable_Task, "UPDI_Enable", 4096, 0, 5, NULL);
        }
        updi_enabled = 1;
        return true;
    }

    void *_config = 0;
    GeneratorMap<Variable> _variableMap;
    ServiceRegistry _serviceRegistry;
    OperationFactory _operationFactory;
    ExpanderMain<&_operationFactory, &_variableMap, &_serviceRegistry> _expanderMain;
    Esp32IdfAnalogService _esp32AnalogService;
    Esp32IdfDigitalService _esp32DigitalService;
    Esp32IdfPwmService _esp32PwmService;
    ATTiny427_ExpanderService _attinyExpanderService(ATTiny427_ExpanderComm_UART0);
    ATTiny427_EVSYSService _attinyEVSYSService(&_attinyExpanderService);
    ATTiny427_PassthroughService _attinyPassthroughService(&_attinyExpanderService, &_attinyEVSYSService);
    AnalogService_ATTiny427Expander _attinyAnalogService(&_attinyExpanderService, 0x0F);
    DigitalService_ATTiny427Expander _attinyDigitalService(&_attinyExpanderService);
    PwmService_ATTiny427Expander _attinyPwmService(&_attinyExpanderService);
    AnalogService_Mercedes _analogService(&_esp32AnalogService, &_attinyAnalogService);
    DigitalService_Mercedes _digitalService(&_esp32DigitalService, &_attinyDigitalService, &_attinyPassthroughService);
    PwmService_Mercedes _pwmService(&_esp32PwmService, &_attinyPwmService, &_attinyPassthroughService, &_attinyDigitalService);
    Esp32IdfTimerService _timerService;
    Esp32IdfCANServiceChannelConfig canconfigs[2] 
    {
        {
            .enabled = true,
            .t_config = TWAI_TIMING_CONFIG_500KBITS(),
            .f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(),
            .g_config = {
                .controller_id = 0,
                .mode = TWAI_MODE_NORMAL, 
                .tx_io = (gpio_num_t)9, 
                .rx_io = (gpio_num_t)8,
                .clkout_io = TWAI_IO_UNUSED, 
                .bus_off_io = TWAI_IO_UNUSED,
                .tx_queue_len = 32, 
                .rx_queue_len = 128,
                .alerts_enabled = TWAI_ALERT_NONE,  
                .clkout_divider = 0,
                .intr_flags = ESP_INTR_FLAG_LEVEL1, 
                .general_flags = {0}
            }
        },
        {
            .enabled = true,
            .t_config = TWAI_TIMING_CONFIG_500KBITS(),
            .f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(),
            .g_config = {
                .controller_id = 1,
                .mode = TWAI_MODE_NORMAL, 
                .tx_io = (gpio_num_t)2, 
                .rx_io = (gpio_num_t)3,
                .clkout_io = TWAI_IO_UNUSED, 
                .bus_off_io = TWAI_IO_UNUSED,
                .tx_queue_len = 32, 
                .rx_queue_len = 128,
                .alerts_enabled = TWAI_ALERT_NONE,  
                .clkout_divider = 0,
                .intr_flags = ESP_INTR_FLAG_LEVEL1, 
                .general_flags = {0}
            }
        }
    };
    Esp32IdfCANService _canService(canconfigs);
    ICommunicationService *broadcastService = _canService.GetISOTPService({.CANIdentifier = 0x7DF, .CANBusNumber = 1 }, {.CANIdentifier = 0x7E8, .CANBusNumber = 1 });
    ICommunicationService *unicastService = _canService.GetISOTPService({.CANIdentifier = 0x7E0, .CANBusNumber = 1 }, {.CANIdentifier = 0x7E8, .CANBusNumber = 1 });
    AggregateCommunicationService _isotpService((ICommunicationService*[]){broadcastService, unicastService}, 2);
    CommunicationHandler_OBD2PIDResponder _obd2PIDHandler(&_isotpService);
    ICommunicationService *_communicationService;
    CommunicationHandler_EFIGenie *_efiGenieHandler;

    bool loadConfig()
    {
        const char * configPath = "/mm/config.bin";
        FILE *fd = NULL;
        struct stat file_stat;

        if(stat(configPath, &file_stat) == -1)
            return false;
        fd = fopen(configPath, "r");
        if(!fd)
            return false;
        free(_config);
        _config = malloc(file_stat.st_size);
        fread(_config, 1, file_stat.st_size, fd);
        fclose(fd);
        return true;
    }

    bool expandermain_write(void *destination, const void *data, size_t length) {
        if(reinterpret_cast<size_t>(destination) >= 0x20000000 && reinterpret_cast<size_t>(destination) <= 0x2000FA00)
        {
            std::memcpy(destination, data, length);
        }
        else if(reinterpret_cast<size_t>(destination) >= 0x8004000 && reinterpret_cast<size_t>(destination) <= 0x8008000)
        {
            //TODO Write to flash
        }

        return true;
    }

    bool expandermain_quit() {
        _expanderMain.Stop();
        return true;
    }

    bool expandermain_start() {
        if(!loadConfig())
            return false;

        size_t configSize = 0;
        _expanderMain.Start(_config, configSize);

        return true;
    }

    void Setup() 
    {
        if(!loadConfig())
            return;
        if(_config == 0)
            return;

        size_t configSize = 0;
        _expanderMain.Start(_config, configSize);

        _efiGenieHandler = new CommunicationHandler_EFIGenie(&_variableMap, expandermain_write, expandermain_quit, expandermain_start, _config);
        // ESP_LOGI("ASDF", "_config %p ", _efiGenieHandler->_config);
        _communicationService->RegisterReceiveCallBack([](communication_send_callback_t send, const void *data, size_t length){ return _efiGenieHandler->Receive(send, data, length);});
    }

    void Loop() 
    {
        _expanderMain.Loop();
    }

    static wdt_hal_context_t rtc_wdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();
    
    TaskHandle_t loop_task_handle = NULL;
    UBaseType_t loop_original_priority = 0;
    void IRAM_ATTR priority_restore_timer_callback()
    {
        vTaskPrioritySet(loop_task_handle, loop_original_priority);
    }

    uint32_t transactionCount = 0;
    void IRAM_ATTR loop(void *parameters)
    {
        Setup();

        // Store the app_main task handle and priority
        loop_task_handle = xTaskGetCurrentTaskHandle();
        loop_original_priority = uxTaskPriorityGet(loop_task_handle);

        const tick_t timerTicksPerSecond = _timerService.GetTicksPerSecond();
        Task *priority_restore_task = new Task(priority_restore_timer_callback, false);
        tick_t averageTick = timerTicksPerSecond / 1000;
        tick_t minTick = averageTick * 100;
        tick_t maxTick = 0;
        tick_t lastTick = _timerService.GetTick();
        int loopcnt = 0;
        while (1)
        {
            // Start timer to restore priority after 1 tick, then set app_main to the lowest priority and yield
            // _timerService->ScheduleTask(priority_restore_task, _timerService->GetTick() + timerTicksPerSecond / 1000);
            // vTaskPrioritySet(loop_task_handle, 0);
            // taskYIELD();
            vTaskDelay(1);

            Loop();

            if (wdt_hal_is_enabled(&rtc_wdt_ctx)) {
                wdt_hal_write_protect_disable(&rtc_wdt_ctx);
                wdt_hal_feed(&rtc_wdt_ctx);
                wdt_hal_write_protect_enable(&rtc_wdt_ctx);
            }
            const tick_t tickDelta = _timerService.GetTick() - lastTick;
            averageTick = (averageTick * 99 + (tickDelta)) / 100;
            if(tickDelta < minTick)
            {
                minTick = tickDelta;
            }
            if(tickDelta > maxTick)
            {
                maxTick = tickDelta;
            }
            loopcnt++;
            if(loopcnt % 1000 == 0)
            {
                ESP_LOGI("MainLoop", "transaction: %d", transactionCount);
            }
            lastTick = _timerService.GetTick();
        }
    }

    static QueueHandle_t uart0_queue;
    uint8_t uart0_buffer[256];
    bool attinyStarted = false;

    static void uart_event_task(void *pvParameters)
    {
        uart_event_t event;
        for (;;) 
        {
            //Waiting for UART event.
            if (xQueueReceive(uart0_queue, (void *)&event, (TickType_t)portMAX_DELAY)) 
            {
                switch (event.type) 
                {
                case UART_DATA:
                    bzero(uart0_buffer, 256);
                    uart_read_bytes(UART_NUM_0, uart0_buffer, event.size, portMAX_DELAY);
                    if(!attinyStarted)
                        break;
                    _attinyExpanderService.Receive(uart0_buffer, event.size);
                    if(!_attinyExpanderService.WaitingForResponse())
                    {
                        bzero(uart0_buffer, 256);
                        const size_t tx_size = _attinyExpanderService.Transmit(uart0_buffer);
		                ESP_ERROR_CHECK(uart_set_rx_full_threshold(UART_NUM_0, tx_size));
                        uart_write_bytes(UART_NUM_0, uart0_buffer, tx_size);
                        transactionCount++;
                    }
                    break;
                default:
                    break;
                }
            }
        }
        vTaskDelete(NULL);
    }

    void installAttiny427UART() {

        //setup interrupt based uart receive
        ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 256, 10, &uart0_queue, 0));

        uart_config_t uart_config = {
            .baud_rate = 1000000,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        // Configure UART parameters
        ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
		ESP_ERROR_CHECK(uart_set_rx_full_threshold(UART_NUM_0, 1));
        // Set UART pins
        ESP_ERROR_CHECK(gpio_set_drive_capability((gpio_num_t)ATTINY_TX, GPIO_DRIVE_CAP_3));
        ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, ATTINY_TX, ATTINY_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        
        xTaskCreate(uart_event_task, "uart_event_task", 3072, NULL, 12, NULL);

        attinyStarted = true;
        const size_t tx_size = _attinyExpanderService.Transmit(uart0_buffer);
		ESP_ERROR_CHECK(uart_set_rx_full_threshold(UART_NUM_0, tx_size));
        uart_write_bytes(UART_NUM_0, uart0_buffer, tx_size);
        transactionCount++;
    }

    void IRAM_ATTR wdthitter(void *parameters)
    {
        while (true)
        {
            vTaskDelay(1000);

            if (wdt_hal_is_enabled(&rtc_wdt_ctx)) {
                wdt_hal_write_protect_disable(&rtc_wdt_ctx);
                wdt_hal_feed(&rtc_wdt_ctx);
                wdt_hal_write_protect_enable(&rtc_wdt_ctx);
            }
        }
    }

    #define NVS_WIFI_NAMESPACE "wifi_config"
    #define NVS_WIFI_SSID_KEY "ssid"
    #define NVS_WIFI_PASSWORD_KEY "password"
    #define NVS_WIFI_AUTHMODE_KEY "authmode"
    // Load WiFi config from NVS, fallback to defaults if not found
    static void load_wifi_config_from_nvs(wifi_config_t *wifi_config)
    {
        nvs_handle_t nvs_handle;    
        esp_err_t err = nvs_open(NVS_WIFI_NAMESPACE, NVS_READWRITE, &nvs_handle);
        
        size_t ssid_len = sizeof(wifi_config->ap.ssid);
        size_t password_len = sizeof(wifi_config->ap.password);

        if (err == ESP_OK) {
            // Try to read SSID
            if (nvs_get_str(nvs_handle, NVS_WIFI_SSID_KEY, (char *)wifi_config->ap.ssid, &ssid_len) != ESP_OK) {
                // Use default SSID with unique chip ID (MAC address)
                uint8_t mac[6];
                esp_read_mac(mac, ESP_MAC_EFUSE_FACTORY);
                snprintf((char *)wifi_config->ap.ssid, sizeof(wifi_config->ap.ssid), 
                         "EFIGenie-Expander-%02X%02X%02X", mac[3], mac[4], mac[5]);
                nvs_set_str(nvs_handle, NVS_WIFI_SSID_KEY, (char *)wifi_config->ap.ssid);
                nvs_commit(nvs_handle);
            }
            
            // Try to read password
            nvs_get_str(nvs_handle, NVS_WIFI_PASSWORD_KEY, (char *)wifi_config->ap.password, &password_len);
            
            // Try to read authmode
            if (nvs_get_u8(nvs_handle, NVS_WIFI_AUTHMODE_KEY, (uint8_t *)&wifi_config->ap.authmode) != ESP_OK) {
                // Use default authmode (set below)
                wifi_config->ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
            }
            
            nvs_close(nvs_handle);
        } else {
            // NVS not available, use defaults
            uint8_t mac[6];
            esp_read_mac(mac, ESP_MAC_EFUSE_FACTORY);
            snprintf((char *)wifi_config->ap.ssid, sizeof(wifi_config->ap.ssid), 
                     "EFIGenie-Expander-%02X%02X%02X", mac[3], mac[4], mac[5]);
            wifi_config->ap.authmode = WIFI_AUTH_OPEN;
        }

        wifi_config->ap.authmode = (strlen((char *)wifi_config->ap.password) > 0) ? wifi_config->ap.authmode : WIFI_AUTH_OPEN;
        
        wifi_config->ap.ssid_len = strlen((char *)wifi_config->ap.ssid);
        wifi_config->ap.max_connection = 5;
    }
    void wifi_init_softap()
    {
        esp_netif_create_default_wifi_ap();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

        wifi_config_t wifi_config;
        memset(&wifi_config, 0, sizeof(wifi_config_t));
        load_wifi_config_from_nvs(&wifi_config);
        wifi_config.ap.pmf_cfg.required = true;

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
    }

    void app_main()
    {
        //Initialize NVS
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        //initialize netlibs
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());

        //initialize wifi
        wifi_init_softap();

        //install uart communication services
        for(uint8_t i = UART_NUM_1; i < UART_NUM_MAX; i++)
        {
            _uartCommServices[i] = new Esp32IdfCommunicationService_UART((uart_port_t)i, 2048);
        }

        installAttiny427UART();

        _serviceRegistry.Register<IAnalogService>(&_analogService);
        _serviceRegistry.Register<IDigitalService>(&_digitalService);
        _serviceRegistry.Register<IPwmService>(&_pwmService);
        _serviceRegistry.Register<ITimerService>(&_timerService);
        _serviceRegistry.Register<ICANService>(&_canService);

        // mount_sd("/SD");
        start_http_server();

        _communicationService = new Esp32IdfCommunicationService_WebSocket(server, "/EFIGenieCommunication");

		const httpd_uri_t resetPost = {
            .uri       = "/command/reset",
			.method     = HTTP_POST,
			.handler    = [](httpd_req_t *req) 
			{
                esp_restart();

				return ESP_OK;
			}
		};
        httpd_register_uri_handler(server, &resetPost);

		const httpd_uri_t resetToOTAUpdaterPost = {
            .uri       = "/command/resetToOTAUpdater",
			.method     = HTTP_POST,
			.handler    = [](httpd_req_t *req) 
			{
                // set the boot partition to factory and restart
                const esp_partition_t *factory_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
                if (factory_partition != NULL) {
                    esp_err_t err = esp_ota_set_boot_partition(factory_partition);
                    if (err != ESP_OK) {
                        ESP_LOGE("OTA", "Failed to set boot partition: %s", esp_err_to_name(err));
                        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to set boot partition");
                    } else {
                        ESP_LOGI("OTA", "Boot partition set to factory. Restarting...");
                        httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        esp_restart();
                    }
                } else {
                    ESP_LOGE("OTA", "Factory partition not found!");
                    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Factory partition not found");
                }

				return ESP_OK;
			}
		};
        httpd_register_uri_handler(server, &resetToOTAUpdaterPost);


        TaskHandle_t wdthitter_task_handle = NULL;
        xTaskCreate(wdthitter, "wdthitter", 4096, NULL, 1, &wdthitter_task_handle);
        mount_mmrofs("/mm");
        register_file_handler_http_server("/mm");
        vTaskDelete(wdthitter_task_handle);

        //create task for loop with max priority
        xTaskCreate(loop, "loop_task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);

        RegisterCANTasks();
    }
}
