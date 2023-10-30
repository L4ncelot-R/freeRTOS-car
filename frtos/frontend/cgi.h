#include "lwip/apps/httpd.h"
#include "pico/cyw43_arch.h"
#include "stdio.h"

// CGI handler for start/stop car
const char * cgi_status_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    // Check if an request for LED has been made (/led.cgi?led=x)
    // check if start/stop car button
    if (strcmp(pcParam[0] , "status") == 0){
        // Look at the argument to check if LED is to be turned on (x=1) or off (x=0)
        if(strcmp(pcValue[0], "0") == 0){
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            printf("CAR STOP"); // call car stop func
        }else if(strcmp(pcValue[0], "1") == 0){
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            printf("CAR START"); // call car start func
        }
    }
    
    // Send the index page back to the user
    return "/index.shtml";
}

// CGI handler for speed control
const char * cgi_speed_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    // Check if an request for LED has been made (/led.cgi?led=x)
    // check for speed increment/decrement control
    if (strcmp(pcParam[0] , "speed") == 0){
        if(strcmp(pcValue[0], "0") == 0){
            printf("SPEED Decrease"); // call speed decrement
        }else if(strcmp(pcValue[0], "1") == 0){
            printf("Speed Increase"); // call speed increment
        }
    }
    
    // Send the index page back to the user
    return "/index.shtml";
}



// tCGI Struct
// Fill this with all of the CGI requests and their respective handlers
static const tCGI cgi_handlers[] = {
    {
        // Html request for "/status.cgi" triggers cgi_handler
        "/status.cgi", cgi_status_handler
    },
    {
        // Html request for "/speed.cgi" triggers cgi_handler
        "/speed.cgi", cgi_speed_handler
    },
};

void cgi_init(void)
{
    http_set_cgi_handlers(cgi_handlers, 2);
}