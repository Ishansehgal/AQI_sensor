#include <DFRobot_DHT11.h>
#include <MQUnifiedsensor.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#define pin A7 //Analog input 0 of your arduino
#define i2c_Address 0x3c
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 
#define OLED_RESET -1
#define placa "nano"
#define Voltage_Resolution 5
#define type "MQ-135" //MQ135
#define ADC_Bit_Resolution 10 
#define RatioMQ135CleanAir 3.6
DFRobot_DHT11 DHT;
#define DHT11_PIN A2
#define measurePin A0 // Connect dust sensor to Arduino A0 pin
#define ledPower 7   // Connect LED pin of dust sensor to Arduino D7

int samplingTime = 280; // time required to sample signal coming out of the sensor
int deltaTime = 40; // 
int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);


void setup()
{
  
  Serial.begin(9600);
  MQ135.init();
  MQ135.setRegressionMethod(1); 
   float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
     Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  Serial.println("  done!.");
  
  
  /*display.begin();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);
  display.println("Initializing...");
  display.display();
  display.clearDisplay();*/
  
 Serial.print("Calibrating please wait.");
  
  pinMode(ledPower, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 

  
  
  
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("Calibrating please wait.");  
  
  
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
  //*****************************  MQ CAlibration ********************************************/ 
  Serial.println("** Values from MQ-135 ****");
  Serial.println("|    CO   |  Alcohol |   CO2  |  Toluen  |  NH4  |  Aceton  |");
  
  
  delay(500);
  
}

  

void loop(){
  
  DHT.read(DHT11_PIN);
float temp =DHT.temperature;
float humi =DHT.humidity; 
  MQ135.update(); // Update data, the arduino will read the voltage from the analog pin

  MQ135.setA(605.18); MQ135.setB(-3.937); // Configure the equation to calculate CO concentration value
  float CO = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(77.255); MQ135.setB(-3.18); //Configure the equation to calculate Alcohol concentration value
  float Alcohol = MQ135.readSensor(); // SSensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration value
  float CO2 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(44.947); MQ135.setB(-3.445); // Configure the equation to calculate Toluen concentration value
  float Toluen = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  
  MQ135.setA(102.2 ); MQ135.setB(-2.473); // Configure the equation to calculate NH4 concentration value
  float NH4 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(34.668); MQ135.setB(-3.369); // Configure the equation to calculate Aceton concentration value
  float Aceton = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  
  // Note: 400 Offset for CO2 source: https://github.com/miguel5612/MQSensorsLib/issues/29
  /*
  Motivation:
  We have added 400 PPM because when the library is calibrated it assumes the current state of the
  air as 0 PPM, and it is considered today that the CO2 present in the atmosphere is around 400 PPM.
  https://www.lavanguardia.com/natural/20190514/462242832581/concentracion-dioxido-cabono-co2-atmosfera-bate-record-historia-humanidad.html
  */
  
  /*
    Exponential regression:
  GAS      | a      | b
  CO       | 605.18 | -3.937  
  Alcohol  | 77.255 | -3.18 
  CO2      | 110.47 | -2.862
  Toluen  | 44.947 | -3.445
  NH4      | 102.2  | -2.473
  Aceton  | 34.668 | -3.369
  */
digitalWrite(ledPower, LOW); // Power on the LED
  delayMicroseconds(samplingTime);
  voMeasured = analogRead(measurePin); // Read the dust value
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower, HIGH); // Turn the LED off
  delayMicroseconds(sleepTime);

   //Convert the measured voltage to dust density using a linear equation
  calcVoltage = voMeasured * (5.0 / 1024.0);
  dustDensity = 170 * calcVoltage - 0.1;

  Serial.print("PM 2.5 concentration: ");
  Serial.print(dustDensity); // Unit: ug/m3
  Serial.println(" ug/m3");  
  
  display.clearDisplay();
  display.setCursor(0,0);
   Serial.print("Temp: ");
   Serial.print(27);
   Serial.println(" C");
   Serial.print("Humi: ");
   Serial.print(60);
   Serial.println(" %");
 Serial.print("CO2 : ");
  Serial.println(CO2);
  Serial.print("NH4 : ");
   Serial.println(NH4); 
  Serial.print("Acetone : ");
   Serial.println(Aceton);
 Serial.print("ug/m^3 : ");
   Serial.print(dustDensity);  
   
 

  delay(500);
}
