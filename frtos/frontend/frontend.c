#include "lwip/apps/httpd.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwipopts.h"
#include "ssi.h"
#include "cgi.h"
#include "lwip/inet.h"
#include "FreeRTOS.h"
#include "task.h"
//#include "lwip/sockets.h"

// WIFI Credentials - take care if pushing to GitHub!
const char WIFI_SSID[] = "XXX";
const char WIFI_PASSWORD[] = "XXX";

void print_ip_address() {
    struct netif *netif = netif_list;
    while (netif != NULL) {
        if (netif_is_up(netif)) {
            printf("IP Address: %s\n", ipaddr_ntoa(&(netif->ip_addr)));
        }
        netif = netif->next;
    }
}

static void webserver_run(){
    // Initialize web server
    httpd_init();
    printf("Http server initialized\n");

    // Configure SSI and CGI handler
    ssi_init();
    printf("SSI Handler initialized\n");
    cgi_init();
    printf("CGI Handler initialized\n");

    // Infinite loop
    while (1) {
        // Add any required delay or use other FreeRTOS features here if needed
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
static void webserver_task(){
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

    webserver_run();
    cyw43_arch_deinit();
}
int main() {
    stdio_init_all();

    // Create the FreeRTOS task for the web server
    TaskHandle_t task_handle;
    xTaskCreate(webserver_task, "WebServerTask", configMINIMAL_STACK_SIZE, NULL, 1, &task_handle);
    
    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    while (1);
}
