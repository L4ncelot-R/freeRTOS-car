#include "lwip/apps/httpd.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"

// SSI tags - tag length limited to 8 bytes by default
const char * ssi_tags[] = {"speed","barcode","pid","orient"};

u16_t ssi_handler(int iIndex, char *pcInsert, int iInsertLen) {
  size_t printed;
  switch (iIndex) {
  case 0: // Speed
    {
      // call getSpeed() function
      const float Speed = adc_read() * 3.3f / (1 << 12);
      printed = snprintf(pcInsert, iInsertLen, "%f ",Speed);
    }
    break;
  case 1: // barcode
    {
      // call getBarcodeOutput() function
      const char* barC = "36"  ;  
      printed = snprintf(pcInsert, iInsertLen, "%s", barC);
    }
    break;
  case 2: //PID
    {
      // whatever to display for PID  
      const char* PID = "54"  ;  
      printed = snprintf(pcInsert, iInsertLen, "%s", PID);
    }
    break;
    case 3: //Orientation
    {
      // call getOrientation() function
      const char* orien = "South"  ;  
      printed = snprintf(pcInsert, iInsertLen, "%s", orien);
    }
    break;
  default:
    printed = 0;
    break;
  }

  return (u16_t)printed;
}

// Initialise the SSI handler
void ssi_init() {
  // Initialise ADC (internal pin)
  adc_init();
  adc_set_temp_sensor_enabled(true);
  adc_select_input(4);

  http_set_ssi_handler(ssi_handler, ssi_tags, LWIP_ARRAYSIZE(ssi_tags));
}
