#include <FirebaseESP8266.h>
#include <ESP8266WiFi.h>
#include "MPU9250.h"


#define WIFI_SSID "View" 
#define WIFI_PASSWORD "0956240018"
#define FIREBASE_HOST "esp8266-95d34-default-rtdb.firebaseio.com"
#define FIREBASE_KEY "qapysxFO3StKTKdgehoyhLKjfe73a1S3Y1NmCYc0"
FirebaseData firebaseData;
MPU9250 mpu;

float stepX,stepY,xavg, yavg, zavg,compass,avgAcc,preAcc = 0,xva,yva,zva,totave,threshold = 0.8;
char *myStrings[5] = {0};
unsigned long prevtime = 0;
String resetOn;
float xval[100] = {0};
float yval[100] = {0};
float zval[100] = {0};
float totvect[1] = {0};
int steps[8] = {0};
int flag = 0 , walk;
bool fall = false;
void setup() {
    connectWifi();
    Firebase.begin(FIREBASE_HOST, FIREBASE_KEY);
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) { // change to your own address
        while(1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
    calibrate();
}

void loop() {
    if (mpu.update()) {
          //------------------------------------------------------Compass---------------------------------------------------------------------------------///
              mpu.getYaw()<0?compass = 360 + mpu.getYaw():compass = mpu.getYaw();
              Serial.print("Compass N(0): ");
              if(compass >= 15 && compass < 105){
                  walk = 2;
                  myStrings[0] = "E";
              }else if(compass >= 105 && compass < 200){
                  walk = 4;
                  myStrings[0] = "S";
              }else if(compass >= 195 && compass < 325){
                  walk = 6;
                  myStrings[0] = "W";
              }else{
                  myStrings[0] = "N";
                  walk = 0;
              }
              Serial.println(myStrings[0]);
              Serial.println("radius is ");Serial.println(compass);
              
              // Write Compass
              //--------------------------------------------------------------- Check Falling Down-------------------------------------------------------------------///
              xva = mpu.getAccX();
              yva = mpu.getAccY();
              zva = mpu.getAccZ();
              avgAcc = sqrt(pow(xva,2)+pow(yva,2)+pow(zva,2));
              Serial.print("Now: ");
              Serial.println(avgAcc);
              Serial.print("Now - Pre: ");
              Serial.println(avgAcc - preAcc);
              if(avgAcc > 2.2 && avgAcc-preAcc >=0.4){
                 Serial.println("Falling Ohhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh");
                 fall = true;
              }else{
                preAcc = avgAcc;  
              }


              //--------------------------------------------------------------- Walking Stepp-------------------------------------------------------------------///
              totvect[1] = sqrt(pow(xva - 0.345 - xavg,2)+pow(yva - 0.346 - yavg,2)+pow(zva - 0.416 - zavg,2));
              totave = (totvect[1] + totvect[0])/2;
              totvect[0] = totvect[1];
              if (totave > threshold && flag == 0)
                  {
                    steps[walk] = steps[walk] + 2;
                    flag = 1;
                  }
              else if (totave > threshold && flag == 1)
                  {
                    // Don't Count
                  }
              if (totave < threshold   && flag == 1)
                  {
                    flag = 0;
                  }
              if (steps < 0) {
                    clearStep();
                  }
              Serial.print("Magnitude – MagnitudePrevious: ");
              Serial.println(totave);
              Serial.print("Step Count(N): ");
              for (byte i = 0; i < 8; i = i + 1) {
                Serial.print(steps[i]);
                Serial.print(",");
              }
              Serial.print("\n");
              if(millis() - prevtime >= 500){
                updateData();
                prevtime = millis();
              }
      
    }
        
}

void updateData(){
  stepY = steps[0]-steps[4];//+(steps[1]+steps[7]-steps[3]-steps[5])/2;
  stepX = steps[2]-steps[6];//+(steps[1]+steps[7]-steps[3]-steps[5])/2;
  FirebaseJson locations;
  locations.set("stepX", String(0.7*stepX));
  locations.set("stepY", String(0.7*stepY));
        
  FirebaseJson data;
  data.set("locations", locations);
  data.set("step",String(steps[0]+steps[1]+steps[2]+steps[3]+steps[4]+steps[5]+steps[6]+steps[7]));
  data.set("fall", fall?"true":"false");
  data.set("step", String(steps[0]+steps[4]+steps[2]+steps[6]));
  data.set("compass",String(myStrings[0]));
  data.set("acc", String(avgAcc));
  data.set("radius", String(compass));
  if(Firebase.updateNode(firebaseData, "/patient", data)) {
      Serial.println("Added"); 
    } else {
      Serial.println("Error : " + firebaseData.errorReason());
      }
  
    //Get data
 if(Firebase.getString(firebaseData, "/patient/reset")) {
  
    resetOn = firebaseData.stringData();
    Serial.println(resetOn);
    data.set("reset", "false");
    if(String(resetOn) == "true"){
    clearStep();
    if(Firebase.updateNode(firebaseData, "/patient", data)) {
      Serial.println("Added"); 
    } else {
      Serial.println("Error : " + firebaseData.errorReason());
      }
    // Do something
  }
    } else {
    Serial.println("Error : " + firebaseData.errorReason());
  }
  }
 



void calibrate()
{
    float sum = 0;
    float sum1 = 0;
    float sum2 = 0;
    for (int i = 0; i < 100; i++) {
      xval[i] = (mpu.getAccX() - 0.345);
      sum = xval[i] + sum;
    }
    delay(100);
    xavg = sum / 100.0;
    Serial.println(xavg);
    for (int j = 0; j < 100; j++)
    {
      yval[j] = (mpu.getAccY() - 0.346);
      sum1 = yval[j] + sum1;
    }
    yavg = sum1 / 100.0;
    Serial.println(yavg);
    delay(100);
    for (int q = 0; q < 100; q++)
    {
      zval[q] = (mpu.getAccX() - 0.416);
      sum2 = zval[q] + sum2;
    }
    zavg = sum2 / 100.0;
    delay(100);
    Serial.println(zavg);
}
  
void clearStep(){
  for(int i=0;i<8;i++){
    steps[i] = 0;
    }
  
  fall = false;
  
}

void connectWifi() {
    Serial.begin(115200);
    Serial.println(WiFi.localIP());
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("connecting");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println();
    Serial.print("connected: ");
    Serial.println(WiFi.localIP());
}
