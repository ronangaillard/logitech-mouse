

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "logitech-mouse.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
logiMouse logi_mouse(7, 8);

#define OUTPUT_READABLE_YAWPITCHROLL

// Unccomment if you are using an Arduino-Style Board
#define ARDUINO_BOARD

#define SIGN(x) ((x > 0) - (x < 0))
#define LED_PIN 13      // (Galileo/Arduino is 13)
#define GRAVITY_SCALE 16384.0
#define GRAVITY_CONSTANT 9.81
#define ACC_UPDATE_RATIO 0.98
#define ACC_UPDATE_RATIO_SLOW 0.998
#define INIT_ACC_UPDATE_RATIO 0.98
#define MOVE_THRESHOLD_HIGH 0.3
#define HIGH_THRESHOLD_RECOVERY_RATE 0.95
#define MOVE_THRESHOLD_LOW 0.15
#define MOVE_THRESHOLD_REVERSE 0.3
#define VELOCITY_THRESHOLD 0.03
#define UNDER_THRESHOLD_RESET 13
#define MOVE_VALUES_NUMBER 30
#define HIGH_PASS_LENGTH 3
#define NORM_FILTER 2
#define WHEEL_THRESHOLD 0.50
#define GYRO_THRESHOLD 1500
#define YAW_THRESHOLD 0.50
#define CLICK_ACCEL_THRESHOLD 0.3
#define CLICK_SIMPLE_ZERO_PASS_THRESHOLD 7
#define CLICK_DOUBLE_ZERO_PASS_THRESHOLD 15
#define CLICK_RESET_COUNT 5
#define YPR_THRESHOLD 0.002
#define Y_YPR_AMP 3000
#define X_YPR_AMP 3500


struct gyro_val {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
};

struct click_follow {
    int zero_pass_count=0;
    VectorFloat accel_max = VectorFloat(0, 0, 0);
    VectorFloat last_acc = VectorFloat(0, 0, 0);
    int reset_count = CLICK_RESET_COUNT + 1;
    char click=0;
};

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
VectorFloat gravity;    // [x, y, z]            gravity vector
Quaternion q;           // [w, x, y, z]         quaternion container
float euler[3];         // [psi, theta, phi]    Euler angle container
float last_ypr[3];
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int error;

gyro_val av;  //accelereometer values
click_follow click; 
VectorFloat moy_acc(0, 0, 1);
VectorFloat acc(0, 0, 1);
Quaternion user_axis;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

// This function is not required when using the Galileo 
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        int TWBR; // 400kHz I2C clock (200kHz if CPU is 8MHz)
        TWBR = 24;
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(230400);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(F("MPU6050 connection "));
    Serial.print(mpu.testConnection() ? F("successful") : F("failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    Serial.println(mpu.getRate());
    //mpu.setRate(8);

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(120);
    mpu.setYGyroOffset(-38);
    mpu.setZGyroOffset(36);
    mpu.setXAccelOffset(-4360);
    mpu.setYAccelOffset(-3695);
    mpu.setZAccelOffset(1295);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    if (!dmpReady) 
        {Serial.println("INIT FAILED!");}
    //stabilize();
    
    Serial.print("begin started : ");
    Serial.println(logi_mouse.begin());
    Serial.println("begin ended");
    Serial.println("pair started");
    if(logi_mouse.reconnect())
    {
        Serial.println("Reconnected using previous configuration !");
    }
    else
    {
        Serial.println("Unable to reconnect to dongle... Pairing with any dongle...");
        logi_mouse.pair();
    }
    Serial.println("pair ended");
    
}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
char avoid_val = 0;
int successive_send_fail = 0;
void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    error = get_gyro_vals();
    if (error == 0) {

        user_axis = user_repere(q);

        mpu.getMotion6(&(av.ax), &(av.ay), &(av.az), &(av.gx), &(av.gy), &(av.gz));
        acc.x = GRAVITY_CONSTANT * float(av.ax) / GRAVITY_SCALE;
        acc.y = GRAVITY_CONSTANT * float(av.ay) / GRAVITY_SCALE;
        acc.z = GRAVITY_CONSTANT * float(av.az) / GRAVITY_SCALE;
        acc.rotate(&user_axis);
        //update mean acceleration
            
        moy_acc.x = moy_acc.x * ACC_UPDATE_RATIO + acc.x * (1.0 - ACC_UPDATE_RATIO);
        
        moy_acc.y = moy_acc.y * ACC_UPDATE_RATIO + acc.y * (1.0 - ACC_UPDATE_RATIO);
        
        moy_acc.z = moy_acc.z * ACC_UPDATE_RATIO + acc.z * (1.0 - ACC_UPDATE_RATIO);
            
        acc.sub(moy_acc);

        int move_x = 0; 
        int move_y = 0; 
        if (abs(last_ypr[0] - ypr[0]) > YPR_THRESHOLD) {
            move_x = int((last_ypr[0] - ypr[0]) * X_YPR_AMP);
            last_ypr[0] = ypr[0];
        }
        if (abs(last_ypr[2] - ypr[2]) > YPR_THRESHOLD) {
            move_y = int((last_ypr[2] - ypr[2]) * Y_YPR_AMP);
            last_ypr[2] = ypr[2];
        }
        bool cliclick;
        cliclick = click_detection();
        Serial.print(int(cliclick) + 6);
        Serial.print(", ");
        Serial.println(click.zero_pass_count);
        if ( (move_x || move_y || cliclick != 0) && avoid_val == 0 ) {
            if (logi_mouse.move(-move_x, move_y, cliclick, false) == 0) 
                successive_send_fail = 0;
            else {
                successive_send_fail++;
            }
        }
        avoid_val = 0;
    } else {
        if (logi_mouse.move(0, 0) == 0) 
            successive_send_fail = 0;
        else {
            successive_send_fail++;
        }
    }
    if (successive_send_fail > 50) {
        logi_mouse.begin();
        if(!logi_mouse.reconnect())
            logi_mouse.pair();
    }
}


/*
Calcutate q, gravity and ypr from MPU6050
return 0 on success
-1 on fail or not enough data to read
*/
int get_gyro_vals() 
{

    // wait for MPU interrupt or extra packet(s) available

    #ifdef ARDUINO_BOARD
        while (!mpuInterrupt && fifoCount < packetSize) {
        }
    #endif

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        avoid_val = 1;
        return -1;
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        while (fifoCount > 200) {
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;
        }

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
        #endif
        return 0;
    }
    return -1;
}

bool click_detection() 
{
    click_detection_axis(&click, acc.x, &(click.last_acc.x), &(click.accel_max.x));
    click_detection_axis(&click, acc.y, &(click.last_acc.y), &(click.accel_max.y));
    click_detection_axis(&click, acc.z, &(click.last_acc.z), &(click.accel_max.z));
    click.reset_count++;
    if (click.reset_count > CLICK_RESET_COUNT) {
        click.reset_count = CLICK_RESET_COUNT + 1;
        click.zero_pass_count = 0;
        click.click = 0;
        click.accel_max.x = 0;
        click.accel_max.y = 0;
        click.accel_max.z = 0;
        click.last_acc.x = 0;
        click.last_acc.y = 0;
        click.last_acc.z = 0;
    }
    if (click.zero_pass_count > CLICK_SIMPLE_ZERO_PASS_THRESHOLD && click.click == 0) {
        click.click = 1;
        return 1;
    }
    else if (click.zero_pass_count > CLICK_DOUBLE_ZERO_PASS_THRESHOLD && click.click < 3) {
        click.click = 3;
        return 1;
    }
    return 0;
}

void click_detection_axis(click_follow* click, float acc, float* last_acc, float* max_acc)
{
    if(abs(acc) > CLICK_ACCEL_THRESHOLD ) {
        if(*max_acc == 0) 
            *max_acc = acc;
        if ( SIGN(acc) != SIGN(*last_acc) ) {
            (click->zero_pass_count)++;
            click->reset_count = 0;
        }
        *last_acc = acc;
    }
}

Quaternion user_repere(Quaternion quat) 
{
    float angle;
    Quaternion r(0,0,0,0);
    VectorFloat v(1,0,0);
    v.rotate(&quat);
    angle = -atan2(v.y,v.x);
    r.w = cos(angle/2.0);
    r.z = sin(angle/2.0);
    return r.getProduct(quat);
}