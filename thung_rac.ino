#define BLYNK_TEMPLATE_ID "TMPL6XjltF8tp"
#define BLYNK_TEMPLATE_NAME "ecotech"
#define BLYNK_AUTH_TOKEN "X2QZ_hnFMESK_LkEbOq_Mnra0s4_400r"
#include <DNSServer.h>
#include "HX711.h"
#include <WiFiManager.h>   
#include <BlynkSimpleEsp32.h>
#include "ESP32Servo.h"
#include <WiFiManager.h>  
#define METALPIN 34
#define SERVO 14
#define TRIG 25
#define ECHO 26
#define DOUT 12
#define CLK 13
Servo myservo;
BlynkTimer timer;
HX711 scale;
float calibration_factor = 100525; 
char token[] = "X2QZ_hnFMESK_LkEbOq_Mnra0s4_400r";
int k;
int d;
float m;
bool full = false;
void mass_cal() {
  scale.set_scale(calibration_factor); //điều chỉnh theo hệ số hiệu chỉnh
 
  m = scale.get_units(); 
  Serial.println(m,3);
}

void ultra_sonic() {
    unsigned long duration; // biến đo thời gian

    
    /* Phát xung từ chân trig */
    digitalWrite(TRIG,0);   // tắt chân trig
    delayMicroseconds(2);
    digitalWrite(TRIG,1);   // phát xung từ chân trig
    delayMicroseconds(5);   // xung có độ dài 5 microSeconds
    digitalWrite(TRIG,0);   // tắt chân trig
    
    /* Tính toán thời gian */
    // Đo độ rộng xung HIGH ở chân echo. 
    duration = pulseIn(ECHO,HIGH);  
    // Tính khoảng cách đến vật.
    d = int(duration/2/29.412);
    
    /* In kết quả ra Serial Monitor */
}
void read_sensor(){
  float weight;
  ultra_sonic();
  Serial.println(d);
  mass_cal();
  k = digitalRead(METALPIN);
  Serial.println(k);
  if(m*10>0.1) {
    weight += m;
      if (m >= 0.01) {
    if (k==0){
      myservo.write(60);
      Serial.println("test:"+String(m));
    }
    if (k==1){
      myservo.write(120);
      Serial.println("test:"+String(m));
    }
    m=0;
  }
}else{   myservo.write(90);}
  if(d<3){
    weight = 0;
    full = true;
  }
  else {full = false;}
  Blynk.virtualWrite(V0, weight);
  Blynk.virtualWrite(V1, k);
  Blynk.virtualWrite(V2, full);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  scale.begin(DOUT, CLK);
  pinMode(METALPIN, INPUT_PULLUP);

  pinMode(TRIG,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(ECHO,INPUT);     // standard 50 hz servo
	myservo.attach(SERVO, 1000, 2000); // attaches the servo on pin 18 to the servo object
  scale.set_scale();
  scale.tare(); //Reset giá trị về 0
 
  long zero_factor = scale.read_average(); 
  Serial.print("Zero factor: "); 
  Serial.println(zero_factor);
  WiFiManager wm;

  bool res;
  wm.resetSettings();
  res = wm.autoConnect("AutoConnectAP","12345678"); // password protected ap
  WiFiManagerParameter custom_blynk_token("Blynk", "blynk token", token, 34);
  wm.addParameter(&custom_blynk_token); 
  Blynk.config(custom_blynk_token.getValue());
  bool result = Blynk.connect();

  if (result != true)
  {
    Serial.println("BLYNK Connection Fail");
    wm.resetSettings();
    ESP.restart();
    delay (5000);
  }
  else
  {
    Serial.println("BLYNK Connected");
  }
  timer.setInterval(1000L, read_sensor);
}

void loop() {
  Blynk.run();
  timer.run();
}
