#include "uLaren_CAN_Driver.h"
#include "FlexCAN.h"
#include "kinetis_flexcan.h"
#include "input_handler.h"
#include "output_handler.h"
#include "loop.h"
#include "fault_handler.h"
#include "structs.h"
#include "Adafruit_VL53L0X.h"

#define NODE_1 1
#define NODE_2 2
#define NODE_3 3
#define NODE_4 4

#define MC_VOLTAGE_THRESHOLD 220

#define SIMULINK 1

//main globals
FlexCAN CANbus(1000000);
State next_state;

//Laser Sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;

//other data globals
// throttle input value (ranges from ~-500 to 500)
volatile int16_t THR_in;
//steering input value (ranges from ~-500 to 500)
volatile int16_t ST_in;
// URF measured distance, in mm (ranges from ~0-5000)
uint16_t URF_dist;
// IMU structure
extern IMUstruct IMUdat;

// Output variables 
long g1 = 0;
long g2 = 0;
long g3 = 0;
  imu::Vector<3> gyro1;   // Stores the 16-bit signed gyro sensor output
int16_t servo_out = 0;
int16_t throttle_out = 0;
int16_t throttle_out_LF = 0;
int16_t throttle_out_RF = 0;
int16_t throttle_out_LR = 0;
int16_t throttle_out_RR = 0;

//timing variables
unsigned long start_time_motors;
unsigned long current_time_motors;
unsigned long start_time_servo;
unsigned long current_time_servo;
unsigned long start_time_voltage;
unsigned long current_time_voltage;

//loop variables
int ret = 0;
int error = NO_ERROR;
int voltage = 0;
int data_int = 0;
/*********************************/
/*********   NOTES **************/
/*
 * ERROR_CAN_WRITE: a write to can error usually means one controller is offline.
 *        In this scenario we will go to INITIALIZE_CONTROLLERS and try to reboot them
 */
 
void setup() {
  //begin serial port operation
  Serial.begin(115200);
  if (SIMULINK)
  {
    Serial1.begin(115200);
    pinMode(PI_GPIO, OUTPUT);
    delay(10);
    digitalWrite(PI_GPIO, LOW);
  }
  //startup CAN network
  CANbus.begin();
  next_state = INITIALIZE_PERIPHERALS;
  indicatorSet(rgbWHITE);
}

void loop() {
  CAN_message_t msg;
  
   switch(next_state)
   {
      case(INITIALIZE_PERIPHERALS):
         //initialize other things
         if (PRINT) 
         {
          Serial.println("Initializing Peripherals");
         }
         initPWMin();
         initServo();
         if (!lox.begin())
         {
            //error state
            indicatorSet(rgbPURPLE);
            while(1);
         }
         next_state = INITIALIZE_CONTROLLERS;
         break;
      case(INITIALIZE_CONTROLLERS):
         if (PRINT)
         {
           Serial.println("Initializing Controllers");
         }
         ret = reset_nodes();
         if (ret > 0)
         {
          error = ret;
         }
         delay(1000);
         
         ret = initialize_CAN();
         if (ret > 0)
         {
          error = ret;
         }
         delay(50);
         
         ret = initialize_MC(NODE_1);
         if (ret > 0)
         {
          error = ret;
         }
         
         process_available_msgs();
         delay(100);
         ret = initialize_MC(NODE_2);
         if (ret > 0)
         {
          error = ret;
         }
         
         process_available_msgs();
         delay(100);
         ret = initialize_MC(NODE_3);
         if (ret > 0)
         {
          error = ret;
         }
         
         process_available_msgs();
         delay(100);
         ret = initialize_MC(NODE_4);
         if (ret > 0)
         {
          error = ret;
         }
        
         if (error == ERROR_CAN_WRITE)
         {
            //reinitialize controllers
            if (PRINT)
            {
              Serial.println("Stopping Nodes.");
            }
            
            delay(500);
            stop_remote_node(NODE_1);
            stop_remote_node(NODE_2);
            stop_remote_node(NODE_3);
            stop_remote_node(NODE_4);
            delay(500);
            process_available_msgs();
            if (PRINT)
            {
              Serial.println("Reinitializing Controllers.");
            }
            
            next_state = INITIALIZE_CONTROLLERS;
            error = 0;
         }
         else 
         {
            next_state = WAIT_FOR_ARM;
            indicatorSet(rgbYELLOW);
         }
         break;
      case(WAIT_FOR_ARM):
         if (PRINT)
         {
           Serial.println("Awaiting arming sequence... ");
         }
         
         while (ST_in < 400)
         {
            //do nothing until armed
            if (PRINT)
            {
              Serial.println(ST_in);
            }
            
            delay(500);
         }
         
         if (ST_in >= 400)
         {
            next_state = LINK_COMMUNICATION;
            indicatorSet(rgbRED);
         }
         else
         {
            next_state = WAIT_FOR_ARM;
         }
         
         break;
      case(LINK_COMMUNICATION):
          //arm
          link_node(NODE_1);
          delay(500);
          link_node(NODE_2);
          delay(500);
          link_node(NODE_3);
          delay(500);
          link_node(NODE_4);
          delay(500);

          if (SIMULINK)
          {
            next_state = RUNNING_SIMULINK;
            //write simulink pin high (pin 15) (PI_GPIO)
            digitalWrite(PI_GPIO, HIGH);
            indicatorSet(rgbCYAN);
          }
          else{
            next_state = RUNNING_NOMINALLY;
            indicatorSet(rgbGREEN);
          } 
            
           start_time_motors = micros();
           start_time_servo = micros();
           start_time_voltage = millis();
           if (PRINT)
           {
             Serial.println("Running under normal operations.");
           }  
          break;
      case(RUNNING_NOMINALLY):
          //check for messages
          while (CANbus.available())
          {
            if (CANbus.read(msg) != 0)
            {
              //Serial.println("ello matey");
              print_incoming_CAN_message(msg);
            }
          }

          //write to motor controllers
          current_time_motors = micros(); 
          if ((current_time_motors - start_time_motors) >= 20000)  //20ms => 50hz
          {
            start_time_motors = micros();
              
            write_velocity_and_enable_MC(NODE_1, -THR_in * SCALE_FACTOR);
            write_velocity_and_enable_MC(NODE_2, THR_in * SCALE_FACTOR);
            write_velocity_and_enable_MC(NODE_3, THR_in * SCALE_FACTOR);
            write_velocity_and_enable_MC(NODE_4, -THR_in * SCALE_FACTOR);
            
          }  

          //check voltage level
          current_time_voltage = millis();
          if ((current_time_voltage - start_time_voltage) >= 800)
          {
            voltage = query_voltage_level(NODE_1);
            start_time_voltage = millis();
            
             if (voltage < MC_VOLTAGE_THRESHOLD)
             {
                Serial.println("Voltage to motors is below our threshold. System is shutting down. Charge your battery!");
                //shutdown drivers 
                shutdown_MC(NODE_1);
                shutdown_MC(NODE_2);
                shutdown_MC(NODE_3);
                shutdown_MC(NODE_4);
                
                //make LED purple
                indicatorSet(rgbPURPLE);
                while(1);
             }
             indicatorSet(rgbGREEN);
          }

        
          // Push Actuator Data---------------------------------------
          // Set steering angle
          current_time_servo = micros();
          if ((current_time_servo - start_time_servo) >= 10000)
          {
            start_time_servo = micros();
            writeServo(ST_in);
          }
          
         break;
      case(RUNNING_SIMULINK):
          //check for CAN messages
          while (CANbus.available())
          {
            if (CANbus.read(msg) != 0)
            {
              //Serial.println("ello matey");
              print_incoming_CAN_message(msg);
            }
          }

          //write to motor controllers if we need to (50Hz)
          current_time_motors = micros(); 
          if ((current_time_motors - start_time_motors) >= 20000)  //20ms => 50hz
          {
            start_time_motors = micros();

            lox.rangingTest(&measure, false);
              
            write_velocity_and_enable_MC(NODE_1, -throttle_out_RF * SCALE_FACTOR);
            write_velocity_and_enable_MC(NODE_2, throttle_out_LF * SCALE_FACTOR);
            write_velocity_and_enable_MC(NODE_3, throttle_out_LR * SCALE_FACTOR);
            write_velocity_and_enable_MC(NODE_4, -throttle_out_RR * SCALE_FACTOR);
            Serial.println("I wrote to motors!");
          }   

          //process serial data (simulink) (simulink running at appx. 50Hz)
          while (Serial1.available() > 0)
          {
            data_int = Serial1.read();
            Serial.println(data_int);
            switch(data_int) 
            {
              case 11: //servo read from pi
                while (!Serial1.available())
                {
                  ;
                }
                servo_out = Serial1.read();
                while (!Serial1.available())
                {
                  ;
                }
                servo_out |= (Serial1.read()  << 8);
                break;
              case 12: //throttle RF read from pi
                while (!Serial1.available())
                {
                  ;
                }
                throttle_out_RF = Serial1.read();
                while (!Serial1.available())
                {
                  ;
                }
                throttle_out_RF |= (Serial1.read()  << 8);
                break;
              case 13: //throttle LF read from pi
              while (!Serial1.available())
                {
                  ;
                }
                throttle_out_LF = Serial1.read();
                while (!Serial1.available())
                {
                  ;
                }
                throttle_out_LF |= (Serial1.read()  << 8);
                break;
              case 14: //throttle LR read from pi
              while (!Serial1.available())
                {
                  ;
                }
                throttle_out_LR = Serial1.read();
                while (!Serial1.available())
                {
                  ;
                }
                throttle_out_LR |= (Serial1.read()  << 8);
                break;
              case 15: //throttle RR read from pi
                while (!Serial1.available())
                {
                  ;
                }
                throttle_out_RR = Serial1.read();
                while (!Serial1.available())
                {
                  ;
                }
                throttle_out_RR |= (Serial1.read()  << 8);
                break;
              case 21:  //servo write to pi
                if (PRINT)
                {
                  Serial.println(ST_in);
                }
                Serial1.write((const uint8_t*)&ST_in, 2);
                break;
              case 22:  //URF write to pi
                Serial1.write((const uint8_t*)&URF_dist, 2);
                break;
              case 23:  //gyroX write to pi
                Serial1.write((const uint8_t*)&g2, 2);
                break;
              case 24:  //gyroY write to pi
                Serial1.write((const uint8_t*)&g3, 2);
                break;
              case 25:  //gyroZ write to pi
                Serial1.write((const uint8_t*)&g1, 2);
                break;
              case 26:  //throttle write to pi
                if (PRINT)
                {
                  Serial.println(THR_in);
                }
                Serial1.write((const uint8_t*)&THR_in, 2);
                break;
              case 27:  //laser write to pi
                Serial1.write((const uint8_t*)&measure.RangeMilliMeter, 2);
                break;
             default:
                Serial.print("In Default case. Should NOT BE HERE. **");
                Serial.println(data_int);
                break;
            }
          }

          //check voltage level (1Hz)
          current_time_voltage = millis();
          if ((current_time_voltage - start_time_voltage) >= 1000)
          {
            voltage = query_voltage_level(NODE_1);
            start_time_voltage = millis();
            
             if (voltage < MC_VOLTAGE_THRESHOLD)
             {
                Serial.println("Voltage to motors is below our threshold. System is shutting down. Charge your battery!");
                //shutdown drivers 
                shutdown_MC(NODE_1);
                shutdown_MC(NODE_2);
                shutdown_MC(NODE_3);
                shutdown_MC(NODE_4);
                
                //make LED purple
                indicatorSet(rgbPURPLE);
             }
             indicatorSet(rgbCYAN);
          }

        
          // Push Actuator Data---------------------------------------
          // Set steering angle (100Hz)
          current_time_servo = micros();
          if ((current_time_servo - start_time_servo) >= 10000)
          {
            start_time_servo = micros();
            writeServo(servo_out);
          }
          
         break;
      case(INDICATE_AND_LOG_ERROR):
         if (PRINT)
         {
           Serial.println("Indicating and logging error.");
         }
         
         next_state = WAIT_FOR_CLEAR;
         break;
      case(WAIT_FOR_CLEAR):  
         delay(1000);
         next_state = WAIT_FOR_ARM;
         break;
      default:
         if (PRINT)
         {
          Serial.println("In default case. Should not be here. Ever.");
          Serial.print("Current state is: ");
          Serial.println(next_state);
         }
         
         exit(0);
         break;  
   }

}
