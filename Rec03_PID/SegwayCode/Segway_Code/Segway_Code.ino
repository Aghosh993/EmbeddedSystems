#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
////////////////////////////////////////////////////////////////////////////////
/*
 * Program CONFIGURATION AREA
 * The options below are BUILD-TIME SETTINGS that control various aspects of the
 * run-time functionality of this software.
 */
////////////////////////////////////////////////////////////////////////////////

//#define RUNTIME_VERBOSE_OUTPUT  1
#define CONTROL_DT 0.01 // In seconds

////////////////////////////////////////////////////////////////////////////////
/*
 * BEGINNING of supporting functions and declarations:
 */
////////////////////////////////////////////////////////////////////////////////

/*
 * Holds Motor rotation direction in robot global coordinate frame (so has mirroring
 * of motors due to opposite-facing mounting taken into account):
 */
typedef enum {
  DIR_FWD,
  DIR_REV
} motor_dir;

/*
 * Initialize Timer 1 to FAST PWM, TOP = 512 (i.e. 9-bit PWM) with Output Compare
 * on both A and B channels:
 */

void init_timer1_pwm()
{
  DDRB |= (1<<1) | (1<<2);
  TCCR1A = (1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) |
            (1<<WGM11) | (0<<WGM10);
  TCCR1B = (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);

  OCR1A = (uint16_t)128;
  OCR1B = (uint16_t)256;
}

void init_timer0_timebase()
{
  TCCR2A = (1<<WGM21);  // We don't want any connection to the OC0A or OC0B pins
                        // CTC mode. Compare on OCR2A
  TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20); // Prescaler=1024

  OCR2A = 156;

  TIMSK2 |= (1<<OCIE2A);  
}

/*
 * Initializes the DIRECTION control pins as outputs by setting appropriate bits in DDRx
 * DIR control pins:  PD2, 4 for IN1, IN2 on M1
 *                    PB4, 5 for IN1, IN2 on M2
 */
void setup_motor_control_io()
{
  DDRD |= (1<<2) | (1<<4);
  DDRB |= (1<<4) | (1<<5);

  init_timer1_pwm();
}

/*
 * Drives the left-side motor (in robot body coordinate frame) at speed set by "left_motor"
 * and in direction set by "left_motor_dir"
 * 
 * left_motor: An unsigned 16-bit integer from 0 to 511 inclusive
 * left_motor_dir: An argument of enum "motor_dir"
 */

void drive_left_motor(uint16_t left_motor, motor_dir left_motor_dir)
{
  switch(left_motor_dir)
  {
    case DIR_FWD:
      PORTD |= (1<<2);
      PORTD &= ~(1<<4);
      break;
    case DIR_REV:
      PORTD &= ~(1<<2);
      PORTD |= (1<<4);
      break;
  }
  if(left_motor <= 511)
  {
    OCR1B = left_motor;
  }
}

/*
 * Drives the right-side motor (in robot body coordinate frame) at speed set by "right_motor"
 * and in direction set by "right_motor_dir"
 * 
 * right_motor: An unsigned 16-bit integer from 0 to 511 inclusive
 * right_motor_dir: An argument of enum "motor_dir"
 */
 
void drive_right_motor(uint16_t right_motor, motor_dir right_motor_dir)
{
  switch(right_motor_dir)
  {
    case DIR_FWD:
      PORTB &= ~(1<<4);
      PORTB |= (1<<5);
      break;
    case DIR_REV:
      PORTB |= (1<<4);
      PORTB &= ~(1<<5);
      break;
  }
  if(right_motor <= 511)
  {
   OCR1A = right_motor; 
  }
}

void drive_motors(uint16_t left_motor, motor_dir left_motor_dir,
                  uint16_t right_motor, motor_dir right_motor_dir)
{
  drive_left_motor(left_motor, left_motor_dir);
  drive_right_motor(right_motor, right_motor_dir);  
}

/*
 * The IMU Setup Code below is based on the "sensorapi" example from Adafruit's Unified Sensor
 * API:
 */

/* Assign a unique base ID for this sensor */   
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000
sensors_event_t accel, mag, gyro, temp;

volatile uint8_t get_data;
volatile float angle;

volatile uint8_t debug_ticks;

void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

int init_imu(void)
{
  /* Initialise the sensor */
  if(!lsm.begin())
  {
    // Error condition... most likely, sensor not on bus!!
    return -1;    // Houston, we have a problem... :)
  }
  // Else, continue to configuration of IMU registers:
  configureSensor();  
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
/*
 * End of supporting functions and declarations... BEGIN USER APP:
 */
////////////////////////////////////////////////////////////////////////////////

void setup() {
  cli(); // Globally disable interrupts on the AVR

  get_data = 0U;
  angle = 0.0f;
  debug_ticks = 0U;
  
  Serial.begin(9600);
  
  setup_motor_control_io();

  init_timer0_timebase();

  drive_motors(0, DIR_REV,
                0, DIR_FWD);
  
  DDRB |= (1<<0); // Turn PB0 (Digital Pin 8 on Arduino) into an OUTPUT for debug purposes

  sei(); // Globally enable interrupts

  init_imu();

  Wire.setClock(400000L);
}

void loop() {
    if(get_data)
    {
      /* Get a new sensor event */ 
      lsm.getEvent(&accel, &mag, &gyro, &temp); 
      get_data = 0U;
      PORTB ^= (1<<0);
    }

  float a_accel = (float)57.30*accel.acceleration.y/(float)9.810;
  angle = (float)0.98*(angle + ((float)CONTROL_DT*gyro.gyro.z)) + (float)0.02*a_accel;
  int16_t motor_drive_val = (int16_t)((float)50.0 * angle);

  if(motor_drive_val < 0)
  {
    motor_drive_val *= -1;
    drive_motors(motor_drive_val, DIR_FWD,
                  motor_drive_val, DIR_FWD);
  }
  else
  {
    drive_motors(motor_drive_val, DIR_REV,
                  motor_drive_val, DIR_REV);    
  }

  if(debug_ticks > 9U)
  {
    Serial.println(angle);
    debug_ticks = 0U;
  }
    
  #ifdef RUNTIME_VERBOSE_OUTPUT
  
    // print out accelleration data
    Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
    Serial.print("  \tY: "); Serial.print(accel.acceleration.y);       Serial.print(" ");
    Serial.print("  \tZ: "); Serial.print(accel.acceleration.z);     Serial.println("  \tm/s^2");
  
    // print out magnetometer data
    Serial.print("Magn. X: "); Serial.print(mag.magnetic.x); Serial.print(" ");
    Serial.print("  \tY: "); Serial.print(mag.magnetic.y);       Serial.print(" ");
    Serial.print("  \tZ: "); Serial.print(mag.magnetic.z);     Serial.println("  \tgauss");
    
    // print out gyroscopic data
    Serial.print("Gyro  X: "); Serial.print(gyro.gyro.x); Serial.print(" ");
    Serial.print("  \tY: "); Serial.print(gyro.gyro.y);       Serial.print(" ");
    Serial.print("  \tZ: "); Serial.print(gyro.gyro.z);     Serial.println("  \tdps");
  
    // print out temperature data
    Serial.print("Temp: "); Serial.print(temp.temperature); Serial.println(" *C");
  
    Serial.println("**********************\n");
  
    delay(250);
  #endif
}

ISR(TIMER2_COMPA_vect)
{
  get_data = 1U;
  debug_ticks += 1U;
//  PORTB ^= (1<<0);
}

