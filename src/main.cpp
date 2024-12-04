#define US_KEYBOARD 1
#include <Arduino.h>
#include "BLEDevice.h"
#include "BLEHIDDevice.h"
#include "HIDTypes.h"

#define DEVICE_NAME "Jump Controller"
#define BUTTON_PIN 25
#define RLED_PIN 32
#define GLED_PIN 33
#define SEND 26
#define ECHO 27

// Forward declarations
void bluetoothTask(void*);
void ledBlinkTask(void*);
void typeText(const char* text);
void pressSpaceKey();
void releaseSpaceKey();
float getDistance();
void setHeight();
void setUp();

bool isBleConnected = false;
TaskHandle_t ledBlinkTaskHandle = NULL;

// Variable für die Speicherung der Entfernung
double distance = 0;
float lowPoint = 0;
float releasePoint = 0;
constexpr long interval = 5000;
bool prevState = 0; //pressed 1 released 0
long lastJump= 0;


void setup(){
    Serial.begin(9600);

    //pin setup
    pinMode(SEND, OUTPUT);
    pinMode(GLED_PIN, OUTPUT);
    pinMode(RLED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(ECHO, INPUT);

    //status LED start
    analogWrite(RLED_PIN, 255);

    // start tasks (Bluetooth and blinking LED)
    // task-funktion, name, stack size, parameters, priority, task-handler
    xTaskCreate(bluetoothTask, "bluetooth", 20000, NULL, 5, NULL);
    xTaskCreate(ledBlinkTask, "LED BLINK", 1024, NULL, 1, &ledBlinkTaskHandle);
    vTaskSuspend(ledBlinkTaskHandle); //suspend task for now
}



void loop() {
    if (digitalRead(BUTTON_PIN) == LOW){
        releaseSpaceKey();
        vTaskResume(ledBlinkTaskHandle); //resume led task
        setUp();
        analogWrite(RLED_PIN, 255);
        delay(3000);
        analogWrite(RLED_PIN, 0);
        analogWrite(GLED_PIN, 255);
        if (lowPoint > releasePoint) lowPoint = releasePoint;
    }
    if (lowPoint!=0) {
        distance = getDistance();
        if (distance >= releasePoint-2 and prevState == 0) {
            pressSpaceKey();
            Serial.println("pressed");
            prevState = 1;
        } else if (distance <= lowPoint+2 and prevState == 1) {
            releaseSpaceKey();
            Serial.println("released");
            prevState = 0;
        }
    }
    if (millis()-lastJump > 30000) releaseSpaceKey();
}

float getDistance(){
    digitalWrite(SEND, LOW);
    delay(5);
    digitalWrite(SEND, HIGH);
    delayMicroseconds(10);
    digitalWrite(SEND, LOW);
    long time = pulseIn(ECHO, HIGH);
    return (time / 2) * 0.03432;
}

void setHeight() {
    Serial.println("Setting height");
    float sensorValues[89]; //89 times will the sensor get a distance, the number is set through a number of tests
    const unsigned long currentMillis = millis();
    int count = 0;
    while(millis() - currentMillis < interval) { //save 89 values in a time period of 5sec.
        sensorValues[count] = getDistance();
        count++;
        delay(50); //to reduce the number of measured values
    }
    float sum = 0;
    for(float sensorValue : sensorValues) sum += sensorValue; //sum of all values
    lowPoint = sum / 89;
    Serial.println("Low Point set!");
    analogWrite(RLED_PIN, 255);
    Serial.print("Press to set release height!");
    while(true) if(digitalRead(BUTTON_PIN)==LOW) {
        releasePoint = getDistance(); //wait for second button press to set the high point
        break;
    }
}

void setUp() {
    Serial.println("Press to start! This will scan the trigger height");
    delay(2000);
    while(true) if(digitalRead(BUTTON_PIN)==LOW) {
        vTaskSuspend(ledBlinkTaskHandle);
        analogWrite(GLED_PIN, 0);
        setHeight();
        break;
    }
    Serial.println("Release point set!");
    Serial.print("min: ");
    Serial.println(lowPoint);
    Serial.print("max: ");
    Serial.println(releasePoint);
}

void ledBlinkTask(void *parameter) {
    while (true) {
        analogWrite(GLED_PIN, 255);
        vTaskDelay(50 / portTICK_PERIOD_MS);

        analogWrite(GLED_PIN, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

/*
 *
 *Bluetooth related code
 *
 */

// Message (report) sent when a key is pressed or released
struct InputReport {
    uint8_t modifiers;	     // bitmask: CTRL = 1, SHIFT = 2, ALT = 4
    uint8_t reserved;        // must be 0
    uint8_t pressedKeys[6];  // up to six concurrenlty pressed keys
};

// The report map describes the HID device (a keyboard in this case) and
// the messages (reports in HID terms) sent and received.
static const uint8_t REPORT_MAP[] = {
    USAGE_PAGE(1),      0x01,       // Generic Desktop Controls
    USAGE(1),           0x06,       // Keyboard
    COLLECTION(1),      0x01,       // Application
    REPORT_ID(1),       0x01,       //   Report ID (1)
    USAGE_PAGE(1),      0x07,       //   Keyboard/Keypad
    USAGE_MINIMUM(1),   0xE0,       //   Keyboard Left Control
    USAGE_MAXIMUM(1),   0xE7,       //   Keyboard Right Control
    LOGICAL_MINIMUM(1), 0x00,       //   Each bit is either 0 or 1
    LOGICAL_MAXIMUM(1), 0x01,
    REPORT_COUNT(1),    0x08,       //   8 bits for the modifier keys
    REPORT_SIZE(1),     0x01,
    HIDINPUT(1),        0x02,       //   Data, Var, Abs
    REPORT_COUNT(1),    0x01,       //   1 byte (unused)
    REPORT_SIZE(1),     0x08,
    HIDINPUT(1),        0x01,       //   Const, Array, Abs
    REPORT_COUNT(1),    0x06,       //   6 bytes (for up to 6 concurrently pressed keys)
    REPORT_SIZE(1),     0x08,
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x65,       //   101 keys
    USAGE_MINIMUM(1),   0x00,
    USAGE_MAXIMUM(1),   0x65,
    HIDINPUT(1),        0x00,       //   Data, Array, Abs
    REPORT_COUNT(1),    0x05,       //   5 bits (Num lock, Caps lock, Scroll lock, Compose, Kana)
    REPORT_SIZE(1),     0x01,
    USAGE_PAGE(1),      0x08,       //   LEDs
    USAGE_MINIMUM(1),   0x01,       //   Num Lock
    USAGE_MAXIMUM(1),   0x05,       //   Kana
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x01,
    HIDOUTPUT(1),       0x02,       //   Data, Var, Abs
    REPORT_COUNT(1),    0x01,       //   3 bits (Padding)
    REPORT_SIZE(1),     0x03,
    HIDOUTPUT(1),       0x01,       //   Const, Array, Abs
    END_COLLECTION(0)               // End application collection
};

BLEHIDDevice* hid;
BLECharacteristic* input;
BLECharacteristic* output;

const InputReport NO_KEY_PRESSED = { };

class BleKeyboardCallbacks : public BLEServerCallbacks {

    void onConnect(BLEServer* server) {
        isBleConnected = true;

        // Allow notifications for characteristics
        BLE2902* cccDesc = (BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
        cccDesc->setNotifications(true);

        analogWrite(RLED_PIN, 0);
        analogWrite(GLED_PIN, 255);
        Serial.println("Client has connected");
    }

    void onDisconnect(BLEServer* server) {
        isBleConnected = false;

        // Disallow notifications for characteristics
        BLE2902* cccDesc = (BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
        cccDesc->setNotifications(false);


        analogWrite(GLED_PIN, 0);
        analogWrite(RLED_PIN, 255);
        Serial.println("Client has disconnected");
        BLEAdvertising* advertising = server->getAdvertising();
        advertising->start();  // Restart advertising
    }
};

void bluetoothTask(void*) {

    // initialize the device
    BLEDevice::init(DEVICE_NAME);
    BLEServer* server = BLEDevice::createServer();
    server->setCallbacks(new BleKeyboardCallbacks());

    // create an HID device
    hid = new BLEHIDDevice(server);
    input = hid->inputReport(1); // report ID
    output = hid->outputReport(1); // report ID

    hid->hidInfo(0x00, 0x02);

    BLESecurity* security = new BLESecurity();
    security->setAuthenticationMode(ESP_LE_AUTH_BOND);

    // set report map
    hid->reportMap((uint8_t*)REPORT_MAP, sizeof(REPORT_MAP));
    hid->startServices();

    // advertise the services
    BLEAdvertising* advertising = server->getAdvertising();
    advertising->setAppearance(HID_KEYBOARD);
    advertising->addServiceUUID(hid->hidService()->getUUID());
    advertising->addServiceUUID(hid->deviceInfo()->getUUID());
    advertising->start();

    Serial.println("BLE ready");
    delay(portMAX_DELAY);
};

void pressSpaceKey() {
    const uint8_t SPACE_KEY_CODE = 0x2C; // space key code

    // HID-Report for space key
    InputReport report = {
        .modifiers = 0,           // no modifications (Shift, Ctrl, etc.)
        .reserved = 0,            // reserved stays 0
        .pressedKeys = {SPACE_KEY_CODE, 0, 0, 0, 0, 0}  // only space key
    };

    input->setValue((uint8_t*)&report, sizeof(report)); // send report
    input->notify();  // press report
    lastJump = millis();
}

void releaseSpaceKey() {
    InputReport noKey = {0};  // all keys released
    input->setValue((uint8_t*)&noKey, sizeof(noKey)); // send empty report
    input->notify();  // release report
}