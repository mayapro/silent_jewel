//#include <helper_3dmath.h>
//#include <MPU6050.h>
#include <MPU6050_6Axis_MotionApps20.h>
//#include <MPU6050_9Axis_MotionApps41.h>


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */


// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL


//========================================================== i changed this line (maya)==========================
//========================================================== i changed this line (maya)==========================

//bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//=================================================================
//===                  our staf                                 ===
//=================================================================

  float data[70];
  int put_index = 0;
  int get_index = 1;
  
  float con_low_filter = 0.0;
  float con_high_filter = 0.0;


  float high_filter[31] = {0.00302799536756454, 0.00392726072766937, 0.00621875911701459, 0.00913449193762858  , 0.0126760963622435, 
                           0.0167956019825058  ,0.0214030968604617 ,0.0263620817540305 ,0.0314939601104913 ,0.0365957127447513 ,0.0414374907388913 ,
                           0.0457917067453814  ,0.0494406886302698 ,0.0521971391604441 ,0.0539106648414111 ,0.0544924628778581 ,0.0539106648414111 ,
                          0.0521971391604441,0.0494406886302698,0.0457917067453814  ,0.0414374907388913 ,0.0365957127447513 ,0.0314939601104913 ,
                          0.0263620817540305  ,0.0214030968604617 ,0.0167956019825058 ,0.0126760963622435 ,0.00913449193762858  ,0.00621875911701459 ,
                          0.00392726072766937 ,0.00302799536756454};
                          



  float low_filter[31] = {0.00293345071425451, -0.00180758787188182, -0.0103303568011080,  -0.0234792367727002,  -0.0360827269012234,  
                            -0.0402726554835675,  -0.0295027761297028,  -0.00352893274581854, 0.0286421465786498, 0.0504639327978519, 0.0445387684372069, 
                            0.00161120549097823,  -0.0727773406776381,  -0.158020068476713, -0.225707495479763, 0.748541581475890,  
                             -0.225707495479763, -0.158020068476713, -0.0727773406776381 ,0.00161120549097823  ,0.0445387684372069 ,
                            0.0504639327978519  ,0.0286421465786498 ,-0.00352893274581854,  -0.0295027761297028,  -0.0402726554835675 ,
                            -0.0360827269012234 ,-0.0234792367727002  ,-0.0103303568011080  ,-0.00180758787188182,  0.00293345071425451};
  
  float alpha = 0.05; 
  
  float avr_low = 0.0;
  float avr_high = 0.0;

  float dif = 0.0;


//=========================== game staff==========================================

  const int motorPin = 5;
  const int PIEZO_PIN = A0; // Piezo output


  int begin_game = 0;
  int garbage = 0;
  int no_count = 0;
  int x = 0;

  int world_counter =0;
  
  bool do_hello = false;
  bool do_world = false;
  bool do_i = false;
  bool do_love = false;
  bool do_you = false;
  bool free_time = false;

  bool second_hello = false;
  

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {

        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }
    
    
    // ============================== our staf =========================================
    // clear the data
    for (int i=0; i<70; i++)
      data[i] = 0.0;

    // ====== pin mode for vibretion(*2):
    pinMode(motorPin,OUTPUT);

  
}

void winSign(){
     digitalWrite(motorPin, HIGH);
     delay(1000);
     digitalWrite(motorPin,LOW);
     return;
}

void nextMove(int num) {
     String xPrint = String(num)+"00";
     Serial.println(xPrint);
     delay(10);
     Serial.println(xPrint);
     delay(13);
     Serial.println(xPrint);
     delay(16);
     Serial.println(xPrint);
     delay(20);
     delay(2000);

     return;
}

void sendGarbage() {
   delay(10);
   x = 0;
   nextMove(x);
   return;
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    
    if (!dmpReady) {
      return;
    }

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here

        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
            mpuInterrupt = true;
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

   
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) {
          fifoCount = mpu.getFIFOCount();
           
        }

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;



        #ifdef OUTPUT_READABLE_WORLDACCEL
        
        
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            
            //==================================================================================
            data[put_index] = aaWorld.x;
            put_index = (put_index + 1) % 70;
            //==================================================================================
            
            //Serial.print(aaWorld.x);
            //Serial.print("\n");
            

//======================================================================================================
            int piezoADC = analogRead(PIEZO_PIN);
//====================================================================================================== convolotion

            
            for (int i=0; i<70; i++)
            {
                int index = (i + get_index) % 70;
                float point = data[index];
                
                con_low_filter = 0.0;
                con_high_filter =0.0;
                
                for (int j=0; j<31; j++){
                  // convolve: multiply and accumulate for low!
                  con_low_filter += point * low_filter[j];
                
                  // convolve: multiply and accumulate for high!
                  con_high_filter += point * high_filter[j];  
                }
            }
            
            avr_low = alpha*abs(con_low_filter) + (1.0 - alpha) * avr_low;
            avr_high = alpha*abs(con_high_filter) + (1.0 - alpha) * avr_high;

//====================================================================================================== convolotion

            
//====================================================================================================== for rhino for filter
           dif = abs(avr_high) - abs(avr_low);
//            
//            Serial.print("dif: ");
//            Serial.print(dif);
//            Serial.print("\n");

//====================================================================================================== for rhino

            if (garbage < 60){
              Serial.println(" i work");
            }

            if (dif < 400 && begin_game == 0 && garbage > 250){
              x = 1;                                    // doll do hello
              nextMove(x);
              do_hello = true;

              delay(1000);

              begin_game = 1;
            }

            if (do_hello){                                // wait for humen to do hello ( exlometer )
              if (dif > 1300 && dif < 2500) { 
                winSign();
                Serial.println(" helloooooooooooooooooooooooooooooooooooooooo");

                do_hello = false;
                do_world = true;

                x = 2;                                   // doll do world
                nextMove(x);
                
                delay(3000);
                world_counter = 0;
              }
              else if (no_count > 6){
                x = 6;                                   // doll do noooooo
                nextMove(x);
                no_count = 0;

                sendGarbage();
              }
            }

            if (do_world){ 
              if (piezoADC > 500 && piezoADC < 900 && world_counter > 5){                 // wait for humen to do world ( piezo (weak) )
                winSign();
                Serial.println(" worldddddddddddddddddddddddddddddddddddd");


                do_world = false;
                do_i = true;
                
                x= 3;                                   // doll do I
                nextMove(x);
                world_counter = 0;


                delay (2000);
              }
              else  if (no_count > 6){
                x = 6;                                   // doll do noooooo
                nextMove(x);
                no_count = 0;

                sendGarbage();
              }
            }

            if (do_i){
              if (dif > 2800  && world_counter > 10){              //  wait for humen to do I ( exelometer )
                 winSign();
                 Serial.println(" iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii");

                 do_i = false;
                 do_love = true;

                x = 4;                                   // doll do love
                nextMove(x);
                
                delay(3000);
                world_counter = 0;

              }
              else  if (no_count > 6){
                x = 6;                                   // doll do noooooo
                nextMove(x);
                no_count = 0;

                sendGarbage();
              }
            }

            if (do_love){
              if (piezoADC > 1000 && world_counter > 12) {               //  wait for humen to do love ( piezo (strong) )
                 winSign();
                 Serial.println(" loveeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee");


                 do_love = false;
                 do_you = true;

                x= 5;                                  // doll do you
                nextMove(x);

                delay(1000);

              }
              else if (no_count > 6){
                x = 6;                                   // doll do noooooo
                nextMove(x);
                no_count = 0;

                sendGarbage();
              }
            }

            if (do_you){
              if ((dif > 2300) && ( piezoADC > 900)) {                           //  wait for humen to do you ( exelometer + piezo (weak) )
                winSign();
                Serial.println("youuuuuuuuuuuuuuuuuuuuuu");

                do_you = false;
                free_time = true;
                second_hello = true;

                world_counter = 0;

              }
              else  if (no_count > 6){
                x = 6;                                   // doll do noooooo
                nextMove(x);
                no_count = 0;

                sendGarbage();
              }
            }

            if (free_time && world_counter > 30){
              if (dif > 1300 && dif < 2000 && second_hello){
                world_counter = 0;
                second_hello = false;
                delay(2500);

              }
              if ( (piezoADC > 1000 && world_counter > 5) && !second_hello){
                x = 1;                                  // doll do hello
                nextMove(x);
                delay(2000);
                x= 4;                                  // doll do love
                nextMove(x);

                free_time = false;
                begin_game = 0;
              }
              else  if (no_count > 5){
                x = 6;                                   
                nextMove(x);
                no_count = 0;

                sendGarbage();
              }
            }



            
//======================================================================================================

            get_index = (get_index + 1) % 70;

            no_count += 1;
            garbage += 1;
            world_counter +=1;

            delay(50);
            
            //===================================================================================
            
            
        #endif
    

    }
}
