#ifdef ESP8266
 #include <ESP8266WiFi.h>
 #else
 #include <WiFi.h>
#endif
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ESP32Servo.h>
//___________________________________________________________________________________________________________________________________pin_setup_______________
#define enA 13//Enable1 L298 Pin enA 
#define in1 12//Motor1  L298 Pin in1 
#define in2  14 //Motor1  L298 Pin in1 
#define in3 27 //Motor2  L298 Pin in1 
#define in4 26 //Motor2  L298 Pin in1 
#define enB 25 //Enable2 L298 Pin enB 
#define L_S 18 //ir sensor Left
#define R_S 19 //ir sensor Right
#define echo 22    //Echo pin
#define trigger 23 //Trigger pin
#define ser_Pin 5  //Servo motor signal pin
//_____________________________________________________________________________________________________________________________________pwm_setup________
const int dutyCycle = 0;    // ESP32 has 16 channels which can generate 16 independent waveforms
const int PWM_FREQ = 5000;     // Recall that Arduino Uno is ~490 Hz. Official ESP32 example uses 5,000Hz
const int PWM_RESOLUTION = 8; // We'll use same resolution as Uno (8 bits, 0-255) but ESP32 can go up to 16 bits 
const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION) - 1);// The max duty cycle value based on PWM resolution (will be 255 if resolution is 8 bits) 
//______________________________________________________________________________________________________________________________________servo_setup_______
Servo myservo;

int count=3;

int Set=15;
int distance_L, distance_F, distance_R,L__S,R__S;
String incommingMessage = "";
//______________________________________________________________________________________________________________________ WiFi Connection Details
const char* ssid = "mm849";
const char* password = "12345678";

//_______________________________________________________________________________________________________________________MQTT Broker Connection Details ***/
const char* mqtt_server = "a77318ae28a342eeba9cfb75dd56927d.s2.eu.hivemq.cloud";
const char* mqtt_username = "maha_002";
const char* mqtt_password = "1212@Ap07";
const int mqtt_port =8883;
/** Secure WiFi Connectivity Initialisation ***/
WiFiClientSecure espClient;

/** MQTT Client Initialisation Using WiFi Connection ***/
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

/** root certificate ***/

static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";
/***** Connect to WiFi *****/
void setup_wifi() {
  delay(10);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("\nWiFi connected\nIP address: ");
  Serial.println(WiFi.localIP());
}
/***** Connect to MQTT Broker *****/
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-";   // Create a random client ID
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");

      client.subscribe("led_state");   // subscribe the topics here

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");   // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
/** Call back Method for Receiving MQTT messages ***/

void callback(char* topic, byte* payload, unsigned int length) {
  String icm="";
  for (int i = 0; i < length; i++) icm+=(char)payload[i];
  Serial.println("Message arrived ["+String(topic)+"]"+icm);
  incommingMessage=icm;
  Serial.println(incommingMessage);
}
/** Method for Publishing MQTT Messages ****/
/*void publishMessage(const char* topic, String payload , boolean retained){
  if (client.publish(topic, payload.c_str(), true))
      Serial.println("Message publised ["+String(topic)+"]: "+payload);
}*/
/** Application Initialisation Function****/
void setup() {
 //______________________________________________________________________________________________________________________________configure I/O pins
  pinMode(trigger, OUTPUT); // Sets the trigPin as an Output
  pinMode(echo, INPUT); // Sets the echoPin as an Input
  pinMode(L_S, INPUT); // Sets the left_IR_sensor Pin as an Input
  pinMode(R_S, INPUT); // Sets the right_IR_sensor Pin as an Input
  pinMode(enA, OUTPUT); // declare as output for L298 Pin enA 
  pinMode(in1, OUTPUT); // declare as output for L298 Pin in1 
  pinMode(in2, OUTPUT); // declare as output for L298 Pin in2 
  pinMode(in3, OUTPUT); // declare as output for L298 Pin in3   
  pinMode(in4, OUTPUT); // declare as output for L298 Pin in4 
  pinMode(enB, OUTPUT); // declare as output for L298 Pin enB 
  myservo.setPeriodHertz(50);// Standard 50hz servo
  myservo.attach(ser_Pin, 500, 2400);   // attaches the servo on pin 18 to the servo object
  
  // _________________________________________________________________________________________________________________________________Configure PWM channel
  ledcSetup(0, 5000, 8);// channel 0, 5000 Hz, 8-bit resolutio
  ledcSetup(1, 5000, 8);// channel 1, 5000 Hz, 8-bit resolution
  ledcAttachPin(enA, 0);  // Attach the channel to the GPIO to be controlled
  ledcAttachPin(enB, 1);  // Attach the channel to the GPIO to be controlled
  ledcWrite(0, dutyCycle);//setting speed of motor
  ledcWrite(1, dutyCycle);//setting speed of moto
  //____________________________________________________________________________________________________________________________________wi_fi_connection attempt
  Serial.begin(115200);
  while (!Serial) delay(1);
  setup_wifi();
  #ifdef ESP8266
    espClient.setInsecure();
  #else
    espClient.setCACert(root_ca);      // enable this line and the the "certificate" code for secure connection
  #endif
  //___________________________________________________________________________________________________________________________________mqtt_broker_connection attempt
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.subscribe("flame_data");
}
/*** Main Function ******/
void loop() {

  if (!client.connected()) reconnect(); // check if client is connected
  client.loop();
 
 
 //___________________________________________________________________________________________________________________________________rover_start______________________
 while(1){
  L__S=digitalRead(L_S);
   R__S =digitalRead(R_S);
     distance_F = Ultrasonic_read();
Serial.print("D F=");Serial.println(distance_F);
//if Right Sensor and Left Sensor are at White color then it will call forword function
 if((R__S == 0 )&& (L__S == 0)){
  if(distance_F > Set){forword();}
                  else{Check_side();}  
 }  
 
//if Right Sensor is Black and Left Sensor is White then it will call turn Right function
else if((R__S == 1)&&(L__S== 0)){turnRight();}  
//if Right Sensor is White and Left Sensor is Black then it will call turn Left function
else if((R__S == 0)&&(L__S == 1)){turnLeft();} 
//IF T joint arises
else if((R__S == 1)&&(L__S== 1)){turnLeft();} 
 //if t joint arises
 else if((R__S == 1)&&(L__S == 1) && count==3){turnLeft();count=2;} 
 //IF T joint arises
else if((R__S == 1)&&(L__S == 1) && count==2){turnRight();turnRight();count=1;} 
 //IF T joint arises
else if((R__S == 1)&&(L__S == 1) && count==1){turnRight();count=0;} 
 //IF T joint arises
else if((R__S == 1)&&(L__S == 1) && count==0){stop();} 
    
delay(10);
    
delay(10);
}
  
}

//**********************Ultrasonic_read****************************
long Ultrasonic_read(){
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  long time = pulseIn (echo, HIGH);
  return time / 29 / 2;
}
void compareDistance(){
    if(distance_L > distance_R){
  turnLeft();
  delay(500);
  forword();
  delay(600);
  turnRight();
  delay(500);
  forword();
  delay(600);
  turnRight();
  delay(400);
  }
  else{
  turnRight();
  delay(500);
  forword();
  delay(600);
  turnLeft();
  delay(500);
  forword();
  delay(600);  
  turnLeft();
  delay(400);
  }
}
void Check_side(){
    stop();
    delay(100);
 myservo.write(30);
  delay(500);
    distance_R = Ultrasonic_read();
    Serial.print("D R=");Serial.println(distance_R);
    delay(100);
  myservo.write(150);
    delay(500);
    distance_L = Ultrasonic_read();
    Serial.print("D L=");Serial.println(distance_L);
    delay(100);
  myservo.write(90);
    delay(300);
    compareDistance();
}
void forword(){  //forword
digitalWrite(in1, LOW); //Left Motor backword Pin 
digitalWrite(in2, HIGH); //Left Motor forword Pin 
digitalWrite(in3, HIGH); //Right Motor forword Pin 
digitalWrite(in4, LOW); //Right Motor backword Pin 
}
void backword(){ //backword
digitalWrite(in1, HIGH); //Left Motor backword Pin 
digitalWrite(in2, LOW); //Left Motor forword Pin 
digitalWrite(in3, LOW); //Right Motor forword Pin 
digitalWrite(in4, HIGH); //Right Motor backword Pin 
}
void turnRight(){ //turnRight
digitalWrite(in1, LOW); //Left Motor backword Pin 
digitalWrite(in2, HIGH); //Left Motor forword Pin 
digitalWrite(in3, LOW); //Right Motor forword Pin 
digitalWrite(in4, HIGH); //Right Motor backword Pin 
}
void turnLeft(){ //turnLeft
digitalWrite(in1, HIGH); //Left Motor backword Pin 
digitalWrite(in2, LOW); //Left Motor forword Pin 
digitalWrite(in3, HIGH); //Right Motor forword Pin 
analogWrite(in4, LOW); //Right Motor backword Pin 
}
void stop(){ //stop
digitalWrite(in1, LOW); //Left Motor backword Pin 
digitalWrite(in2, LOW); //Left Motor forword Pin 
digitalWrite(in3, LOW); //Right Motor forword Pin 
digitalWrite(in4, LOW); //Right Motor backword Pin 
}
