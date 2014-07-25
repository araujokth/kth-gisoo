#include "contiki.h"
#include "dev/sht11-sensor.h"
#include "dev/light-sensor.h"
#include "dev/leds.h"
#include <stdio.h>

//Declare the process
PROCESS(send_sensor_info_process, "Print the Sensors Information");

//Make the process start when the module is loaded
AUTOSTART_PROCESSES(&send_sensor_info_process);

/*---------------------------------------------------------------------------*/
static int
get_light(void)
{
  return 10 * light_sensor.value(LIGHT_SENSOR_PHOTOSYNTHETIC) / 7;
}
/*---------------------------------------------------------------------------*/
static int
get_temp(void)
{
  return ((sht11_sensor.value(SHT11_SENSOR_TEMP) / 10) - 396) / 10;
}
/*---------------------------------------------------------------------------*/

//Define the process code
PROCESS_THREAD(send_sensor_info_process, ev, data)
{
  PROCESS_BEGIN();
  SENSORS_ACTIVATE(light_sensor);
  SENSORS_ACTIVATE(sht11_sensor);
  printf("Light: %d \n", get_light());
  printf("Temperature: %d \n", get_temp());

  PROCESS_END();
}