
#include <driver/ledc.h>
#include "esp_err.h"
#include "propulsion.h"
#include <Arduino.h>
#include <math.h>

int HOVER = 1445;

#define PIN_ESC_PROA      27 
#define PIN_ESC_ESTRIBOR  26
#define PIN_ESC_POPA      25
#define PIN_ESC_BABOR     33

#define PIN_SERVO         32 //¿el pin 32 esta jodido?

#define CANAL_ESC_PROA      0
#define CANAL_ESC_ESTRIBOR  1
#define CANAL_ESC_POPA      2
#define CANAL_ESC_BABOR     3
#define CANAL_SERVO         4

/*
 * LEDC CHAN to Group/Channel/Timer Mapping
 * ----------------------------------------
 * ledc: 0  => Group: 0, Channel: 0, Timer: 0
 * ledc: 1  => Group: 0, Channel: 1, Timer: 0
 * ledc: 2  => Group: 0, Channel: 2, Timer: 1
 * ledc: 3  => Group: 0, Channel: 3, Timer: 1
 * ledc: 4  => Group: 0, Channel: 4, Timer: 2
 * ledc: 5  => Group: 0, Channel: 5, Timer: 2
 * ledc: 6  => Group: 0, Channel: 6, Timer: 3
 * ledc: 7  => Group: 0, Channel: 7, Timer: 3
 * ledc: 8  => Group: 1, Channel: 0, Timer: 0
 * ledc: 9  => Group: 1, Channel: 1, Timer: 0
 * ledc: 10 => Group: 1, Channel: 2, Timer: 1
 * ledc: 11 => Group: 1, Channel: 3, Timer: 1
 * ledc: 12 => Group: 1, Channel: 4, Timer: 2
 * ledc: 13 => Group: 1, Channel: 5, Timer: 2
 * ledc: 14 => Group: 1, Channel: 6, Timer: 3
 * ledc: 15 => Group: 1, Channel: 7, Timer: 3
 */
#define TIMER_ESC_PROA      0
#define TIMER_ESC_ESTRIBOR  1
#define TIMER_ESC_POPA      2
#define TIMER_ESC_BABOR     3
#define TIMER_SERVO         0

const uint32_t FRECUENCIA = 250; // sampling-frequency => 1000/250=4 milisegundos
const uint16_t NUMERO_BITS = 12; // resolution (number of bits)

uint32_t resolution;

#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
//#define MIN_PULSE_LENGTH 500 // Minimum pulse length in µs (servos)
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
//#define MAX_PULSE_LENGTH 2500 // Maximum pulse length in µs (servos)

struct datosPROP prop;

void init_propulsion() {
  pwm_init(PIN_ESC_PROA, CANAL_ESC_PROA, TIMER_ESC_PROA, 1000);
  pwm_init(PIN_ESC_POPA, CANAL_ESC_POPA, TIMER_ESC_POPA, 1000);
  pwm_init(PIN_ESC_BABOR, CANAL_ESC_BABOR, TIMER_ESC_BABOR, 1000);
  pwm_init(PIN_ESC_ESTRIBOR, CANAL_ESC_ESTRIBOR, TIMER_ESC_ESTRIBOR, 1000);  
  pwm_init(PIN_SERVO, CANAL_SERVO, TIMER_SERVO, (1500 + prop.OFFSET_SERVO));
}

void testPROA() {
   //hacia una posicion neutra
  pwm_set(1000, CANAL_ESC_PROA);
  vTaskDelay(100);

  //maniobra
  pwm_set(prop.UMBRAL, CANAL_ESC_PROA);
  vTaskDelay(1000); 
  pwm_set(1000, CANAL_ESC_PROA);
}

void testPOPA() {
   //hacia una posicion neutra
  pwm_set(1000, CANAL_ESC_POPA);
  vTaskDelay(100);

  //maniobra
  pwm_set(prop.UMBRAL, CANAL_ESC_POPA);
  vTaskDelay(1000); 
  pwm_set(1000, CANAL_ESC_POPA);
}

void testBABOR() {
   //hacia una posicion neutra
  pwm_set(1000, CANAL_ESC_BABOR);
  vTaskDelay(100);

  //maniobra
  pwm_set(prop.UMBRAL, CANAL_ESC_BABOR);
  vTaskDelay(1000); 
  pwm_set(1000, CANAL_ESC_BABOR);
}

void testESTRIBOR() {
  //hacia una posicion neutra
  pwm_set(1000, CANAL_ESC_ESTRIBOR);
  vTaskDelay(100);

  //maniobra
  pwm_set(prop.UMBRAL, CANAL_ESC_ESTRIBOR);
  vTaskDelay(1000); 
  pwm_set(1000, CANAL_ESC_ESTRIBOR);
}

void testSERVO () {
  //hacia una posicion neutra
  pwm_set(1500 + prop.OFFSET_SERVO, CANAL_SERVO);
  vTaskDelay(100);

  //maniobra
  pwm_set(1500 + prop.OFFSET_SERVO + prop.LIMITE_SERVO, CANAL_SERVO);
  vTaskDelay(1000);
  pwm_set(1500 + prop.OFFSET_SERVO, CANAL_SERVO);
  vTaskDelay(1000);
  pwm_set(1500 + prop.OFFSET_SERVO - prop.LIMITE_SERVO, CANAL_SERVO);
  vTaskDelay(1000);
  pwm_set(1500 + prop.OFFSET_SERVO, CANAL_SERVO);
}

void setTestProp(int P1, int P2, int P3, int P4, int P5) {
  if (P1 != 0) {
    if ((P1 <= prop.LIMITE_ESC) && (P1 >= 1000))
          pwm_set(P1, CANAL_ESC_PROA);
  }
  
  if (P2 != 0) {
    if ((P2 <= prop.LIMITE_ESC) && (P2 >= 1000))
          pwm_set(P2, CANAL_ESC_POPA);
  }
  
  if (P3 != 0) {
    if ((P3 <= prop.LIMITE_ESC) && (P3 >= 1000))
          pwm_set(P3, CANAL_ESC_ESTRIBOR);
  }
  
  if (P4 != 0) {
    if ((P4 <= prop.LIMITE_ESC) && (P4 >= 1000))
          pwm_set(P4, CANAL_ESC_BABOR);
  }

  if (P5 != 0) {
    if ((P5 < prop.ALIGN_SERVO + prop.LIMITE_SERVO) && (P5 > prop.ALIGN_SERVO - prop.LIMITE_SERVO)) {
        //P5 = P5 + prop.OFFSET_SERVO; //¿en los tests todo viene mascadito por la WEB?
        pwm_set(P5, CANAL_SERVO);
    }
  }
}



void pwm_init(int PIN, int CANAL, int TIMER, uint32_t duty) {
  ledc_channel_config_t ledc_channel = { 0 };
      ledc_channel.gpio_num = PIN; //uint8_t
      ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE; 
      //typedef enum {
      //    LEDC_HIGH_SPEED_MODE = 0, // LEDC high speed speed_mode 
      //    LEDC_LOW_SPEED_MODE,      // LEDC low speed speed_mode 
      //    LEDC_SPEED_MODE_MAX,      // LEDC speed limit 
      //} ledc_mode_t;
      ledc_channel.channel = (ledc_channel_t) CANAL;  
      //typedef enum {
      //    LEDC_CHANNEL_0 = 0,
      //    LEDC_CHANNEL_1,
      //    LEDC_CHANNEL_2,
      //    LEDC_CHANNEL_3,
      //    LEDC_CHANNEL_4, 
      //    LEDC_CHANNEL_5,
      //    LEDC_CHANNEL_6,
      //    LEDC_CHANNEL_7,
      //    LEDC_CHANNEL_MAX,
      //} ledc_channel_t;
      ledc_channel.intr_type = LEDC_INTR_DISABLE;     
      //typedef enum {
      //    LEDC_INTR_DISABLE = 0,    // Disable LEDC interrupt
      //    LEDC_INTR_FADE_END,       // Enable LEDC interrupt 
      //} ledc_intr_type_t;
      ledc_channel.timer_sel = (ledc_timer_t) TIMER;       
      //typedef enum {
      //    LEDC_TIMER_0 = 0,
      //    LEDC_TIMER_1,   
      //    LEDC_TIMER_2, 
      //    LEDC_TIMER_3, 
      //    LEDC_TIMER_MAX,
      //} ledc_timer_t;    
      ledc_channel.duty = duty; //uint32_t
      //ledc_channel.hpoint = 0; //uint32_t
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

  ledc_timer_config_t ledc_timer; // = { 0 };
      ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
      ledc_timer.bit_num = (ledc_timer_bit_t) NUMERO_BITS;  
      //typedef enum {
      //    LEDC_TIMER_1_BIT = 1, // LEDC PWM duty resolution
      //    LEDC_TIMER_2_BIT,
      //    LEDC_TIMER_3_BIT,
      //    LEDC_TIMER_4_BIT,   
      //    LEDC_TIMER_5_BIT,
      //    LEDC_TIMER_6_BIT,
      //    LEDC_TIMER_7_BIT,   
      //    LEDC_TIMER_8_BIT,
      //    LEDC_TIMER_9_BIT,
      //    LEDC_TIMER_10_BIT,   
      //    LEDC_TIMER_11_BIT,
      //    LEDC_TIMER_12_BIT,
      //    LEDC_TIMER_13_BIT,   
      //    LEDC_TIMER_14_BIT,   
      //    LEDC_TIMER_15_BIT,
      //    LEDC_TIMER_16_BIT,
      //    LEDC_TIMER_17_BIT,   
      //    LEDC_TIMER_18_BIT,   
      //    LEDC_TIMER_19_BIT,
      //    LEDC_TIMER_20_BIT,
      //    LEDC_TIMER_BIT_MAX,
      //} ledc_timer_bit_t;
      ledc_timer.timer_num = (ledc_timer_t) TIMER;
      ledc_timer.freq_hz = FRECUENCIA; //uint32_t
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
}

void pwm_set(uint32_t duty, int CANAL) {
    //duty = (resolution / MAX_PULSE_LENGTH ) * min (duty, MAX_PULSE_LENGTH); //¡¡¡ OJO, MUY IMPORTANTE !!!
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t) CANAL, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t) CANAL);   
}

void setProp() {
  pwm_set(prop.pwmPROA + prop.OFFSET_PROA, CANAL_ESC_PROA);
  pwm_set(prop.pwmPOPA + prop.OFFSET_POPA, CANAL_ESC_POPA);
  pwm_set(prop.pwmESTRIBOR + prop.OFFSET_ESTRIBOR, CANAL_ESC_ESTRIBOR);  
  pwm_set(prop.pwmBABOR + prop.OFFSET_BABOR, CANAL_ESC_BABOR);
}

void setPROA() {
  pwm_set(prop.pwmPROA + prop.OFFSET_PROA, CANAL_ESC_PROA);
}

void setPOPA() {
  pwm_set(prop.pwmPOPA + prop.OFFSET_POPA, CANAL_ESC_POPA);
}

void setESTRIBOR() {
  pwm_set(prop.pwmESTRIBOR + prop.OFFSET_ESTRIBOR, CANAL_ESC_ESTRIBOR);  
}

void setBABOR() {
  pwm_set(prop.pwmBABOR + prop.OFFSET_BABOR, CANAL_ESC_BABOR);
}

void setServo() {  
  pwm_set(prop.anguloSERVO + prop.ALIGN_SERVO, CANAL_SERVO);
}



/*
 * LEDC CHAN to Group/Channel/Timer Mapping
 * ----------------------------------------
 * ledc: 0  => Group: 0, Channel: 0, Timer: 0
 * ledc: 1  => Group: 0, Channel: 1, Timer: 0
 * ledc: 2  => Group: 0, Channel: 2, Timer: 1
 * ledc: 3  => Group: 0, Channel: 3, Timer: 1
 * ledc: 4  => Group: 0, Channel: 4, Timer: 2
 * ledc: 5  => Group: 0, Channel: 5, Timer: 2
 * ledc: 6  => Group: 0, Channel: 6, Timer: 3
 * ledc: 7  => Group: 0, Channel: 7, Timer: 3
 * ledc: 8  => Group: 1, Channel: 0, Timer: 0
 * ledc: 9  => Group: 1, Channel: 1, Timer: 0
 * ledc: 10 => Group: 1, Channel: 2, Timer: 1
 * ledc: 11 => Group: 1, Channel: 3, Timer: 1
 * ledc: 12 => Group: 1, Channel: 4, Timer: 2
 * ledc: 13 => Group: 1, Channel: 5, Timer: 2
 * ledc: 14 => Group: 1, Channel: 6, Timer: 3
 * ledc: 15 => Group: 1, Channel: 7, Timer: 3
 */
 
#define LEDC_CHAN(g,c) LEDC.channel_group[(g)].channel[(c)]
#define LEDC_TIMER(g,t) LEDC.timer_group[(g)].timer[(t)]

/********** Funcion local **********/
/*
double getDutyByPercentage(double percentage){
  if (percentage <= 0){
    return 0;
  }
  if (percentage > 100){
    percentage = 100;
  }
  return (percentage / 100.0) * ((2<<(NUMERO_BITS-1))-1); // LEDC_TIMER_12_BIT
}
*/

/********** Funcion local **********/
/*
double getDutyByuS(double uS){
  return getDutyByPercentage(((uS * 100.0)/(1000000/FRECUENCIA)));
}
*/

/*
  //***** 1ºMETODO *****
      //ledcSetup(CANAL_ESC_POPA, FRECUENCIA, NUMERO_BITS);
      //ledcSetup(CANAL_SERVO_POPA, FRECUENCIA, NUMERO_BITS);
      //ledcAttachPin(PIN_ESC_POPA, CANAL_ESC_POPA); 
      //ledcAttachPin(PIN_SERVO_POPA, CANAL_SERVO_POPA); 
  //***** 2ºMETODO *****
      //ESC_POPA.setPeriodHertz(FRECUENCIA);
      //ESC_POPA.attach(PIN_ESC_POPA, 1000, 2000);
      //SERVO_POPA.setPeriodHertz(FRECUENCIA);
      //SERVO_POPA.attach(PIN_SERVO_POPA, 500, 2500); //REVISAR A LOS VALORES 500/2500
*/



void init_propulsion_ledc () {
  // 500hz PWM, 11-bit resolution
  ledcSetup(CANAL_ESC_PROA, 500, 11);
  ledcSetup(CANAL_ESC_POPA, 500, 11);
  ledcSetup(CANAL_ESC_ESTRIBOR, 500, 11);
  ledcSetup(CANAL_ESC_BABOR, 500, 11);
  
  ledcAttachPin(PIN_ESC_PROA, CANAL_ESC_PROA); 
  ledcAttachPin(PIN_ESC_POPA, CANAL_ESC_POPA); 
  ledcAttachPin(PIN_ESC_ESTRIBOR, CANAL_ESC_ESTRIBOR); 
  ledcAttachPin(PIN_ESC_BABOR, CANAL_ESC_BABOR); 

  ledcWrite(CANAL_ESC_PROA, MIN_PULSE_LENGTH);
  ledcWrite(CANAL_ESC_POPA, MIN_PULSE_LENGTH);
  ledcWrite(CANAL_ESC_ESTRIBOR, MIN_PULSE_LENGTH);
  ledcWrite(CANAL_ESC_BABOR, MIN_PULSE_LENGTH);
}

void set_prop_ledc () {
  ledcWrite(CANAL_ESC_PROA, prop.pwmPROA);
  ledcWrite(CANAL_ESC_POPA, prop.pwmPOPA);
  ledcWrite(CANAL_ESC_ESTRIBOR, prop.pwmESTRIBOR);
  ledcWrite(CANAL_ESC_BABOR, prop.pwmBABOR);
}
