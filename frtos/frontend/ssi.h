#include "lwip/apps/httpd.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"

#include "car_config.h"
car_struct_t *car_struct;

// SSI tags - tag length limited to 8 bytes by default
const char * ssi_tags[] = {"speed","barcode","pid","obstacle","orient"};

u16_t ssi_handler(int iIndex, char *pcInsert, int iInsertLen) {
  size_t printed;
  switch (iIndex) {
  case 0: // Speed
    {
      // call getSpeed() function

      /*
      const float c_speed = car_struct->p_right_motor->speed.current_cms;
      printf("\n\t%f\n",c_speed);
      printed = snprintf(pcInsert, iInsertLen, "%f ",c_speed);
      */

      const float speed = adc_read() * 3.3f / (1 << 12);
      printed = snprintf(pcInsert, iInsertLen, "%f ",speed);
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
      const char* PID = "-"  ;  
      printed = snprintf(pcInsert, iInsertLen, "%s", PID);
    }
    break;
  case 3: //Obstacle detected
    {
      // call getObstacleDetected() function

      /*
      bool obstacle = car_struct->obs->ultrasonic_detected;
      if (obstacle){
        printed = snprintf(pcInsert, iInsertLen, "%s", "YES");
      }
      else{
        printed = snprintf(pcInsert, iInsertLen, "%s", "NO");
      }
      */

      printed = snprintf(pcInsert, iInsertLen, "%s", "YES");
    }
  case 4: //Orientation
    {
      // call getOrientation() function

      const char* orien = "South";  
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
void ssi_init(void *params) {

  car_struct = (car_struct_t *)params;
  
  // Initialise ADC (internal pin)
  adc_init();
  adc_set_temp_sensor_enabled(true);
  adc_select_input(4);

  http_set_ssi_handler(ssi_handler, ssi_tags, LWIP_ARRAYSIZE(ssi_tags));
}
