#include "lwip/apps/httpd.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwipopts.h"
#include "ssi.h"
#include "cgi.h"
#include "FreeRTOS.h"
#include "task.h"

#include "car_config.h"

// print assigned ip addr
void print_ip_address() {
    struct netif *netif = netif_list;
    while (netif != NULL) {
        if (netif_is_up(netif)) {
            printf("IP Address: %s\n", ipaddr_ntoa(&(netif->ip_addr)));
        }
        netif = netif->next;
    }
}

// initializes server and handlers
static void webserver_run(void *params){
    car_struct_t *car_struct = (car_struct_t *)params;

    // Initialize web server
    httpd_init();
    printf("Http server initialized\n");

    // Configure SSI and CGI handler
    ssi_init(&car_struct);
    printf("SSI Handler initialized\n");
    cgi_init(&car_struct);
    printf("CGI Handler initialized\n");

    // Infinite loop
    while (1) {
        // Add any required delay or use other FreeRTOS features here if needed
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
// webserver task connect wifi & start webserver_run
static void webserver_task(void *params){
    car_struct_t *car_struct = (car_struct_t *)params;
    if (cyw43_arch_init())
    {
        printf("failed to initialise\n");
        return;
    }
 
    cyw43_arch_enable_sta_mode();
 
    printf("Connecting to WiFi...\n");
 
    // Connect to the WiFI network - loop until connected
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0) {
        printf("Attempting to connect...\n");
    }
    // Print a success message once connected
    printf("Connected! \n");
    print_ip_address();

    webserver_run(&car_struct);
    cyw43_arch_deinit();
}

// frontend initialize - create task
void webserver_init(void *params)
{
    car_struct_t *car_struct = (car_struct_t *)params;
    // Create the FreeRTOS task for the web server
    TaskHandle_t task_handle;
    xTaskCreate(webserver_task, "WebServerTask", configMINIMAL_STACK_SIZE, (void *)&car_struct, 1, &task_handle);
}