#include "lwip/apps/httpd.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwipopts.h"
#include "ssi.h"
#include "cgi.h"
#include "lwip/inet.h"
//#include "FreeRTOS.h"
//#include "task.h"
#include "lwip/sockets.h"

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

int main() {
    stdio_init_all();

    cyw43_arch_init();

    cyw43_arch_enable_sta_mode();

    // Connect to the WiFI network - loop until connected
    while(cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0){
        printf("Attempting to connect...\n");
    }
    // Print a success message once connected
    printf("Connected! \n");

    // Print the assigned IP address
    print_ip_address();
    
    // Initialize web server
    httpd_init();
    printf("Http server initialized\n");

    // Configure SSI and CGI handler
    ssi_init(); 
    printf("SSI Handler initialized\n");
    cgi_init();
    printf("CGI Handler initialized\n");
    
    // Infinite loop
    while(1);

}