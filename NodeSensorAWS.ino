//Library yang digunakan
//#include <Wire.h>
#include <Sodaq_SHT2x.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <BH1750.h>
#include <SoftwareSerial.h>

//Deklarasi variabel dan pin sensor 

float latitude = -6.969147; //posisi TelU
float longitude = 107.628158;

Adafruit_BMP280 bmp;

const int UV_OUT = A0;    // Sensor Output - pin A0
const int REF_3V3 = A1;   // 3.3V power pada papan Arduino - pin A1

float humidityValue = 0.0; 
float temperatureValue = 0.0; // Mendeklarasikan variabel float untuk suhu SHT21
float dewpointValue = 0.0;

float temperature; // Mendeklarasikan variabel float untuk suhu BMP280
float pressure;    // Mendeklarasikan variabel float untuk tekanan BMP280
float altitude;    // Mendeklarasikan variabel float untuk ketinggian BMP280

BH1750 lightMeter;
float lux; //BH1750

const int pin_interrupt = 2; // Menggunakan pin interrupt https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
long int jumlah_tip = 0; 
long int temp_jumlah_tip = 0;
float curah_hujan = 0.00;
float milimeter_per_tip = 0.70;
volatile boolean flag = false;
//
volatile byte rpmcount; // count signals
volatile unsigned long last_micros;
unsigned long timeold;
unsigned long timemeasure = 25.00; // seconds
int timetoSleep = 1;               // minutes
unsigned long sleepTime = 15;      // minutes
unsigned long timeNow;
int GPIO_pulse = 3; // Arduino = D2
float rpm, rps;     // frequencies
float radius = 0.1; // meters - measure of the lenght of each the anemometer wing
float velocity_kmh; // km/h
float velocity_ms;  //m/s
float omega = 0;    // rad/s
float calibration_value = 2.0;

int a, b;
String data, arah_angin, s_angin;
//
#define RE 7
#define DE 8

const byte nitro[] = {0x01,0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c};
const byte phos[] = {0x01,0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc};
const byte pota[] = {0x01,0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0};
const byte soil_ph[] = {0x01, 0x03, 0x00, 0x06, 0x00, 0x01, 0x64, 0x0b};
const byte soil_moist[] = {0x01, 0x03, 0x00, 0x12, 0x00, 0x01, 0x24, 0x0f};
const byte temp[] = {0x01, 0x03, 0x00, 0x12, 0x00, 0x02, 0x64, 0x0e};
const byte ec[] = {0x01, 0x03, 0x00, 0x15, 0x00, 0x01, 0x95, 0xce};

int values[11];
SoftwareSerial mod(10,11);////// TX AND RX Soil Sensor
//
void hitung_curah_hujan()
{
  flag = true;
}

void rpm_anemometer() {
  if (long(micros() - last_micros) >= 5000) { // time to debounce measures
    rpmcount++;
    last_micros = micros();
  }
}

void setup() {
//  Wire.begin();
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(115200); // Initialize the hardware serial port for debugging
  // dataserial.begin(9600);

   // put your setup code here, to run once:
  pinMode(GPIO_pulse, INPUT_PULLUP);
  digitalWrite(GPIO_pulse, LOW);
  
  detachInterrupt(digitalPinToInterrupt(GPIO_pulse));                         // force to initiate Interrupt on zero
  attachInterrupt(digitalPinToInterrupt(GPIO_pulse), rpm_anemometer, RISING); //Initialize the intterrupt pin
  rpmcount = 0;
  rpm = 0;
  timeold = 0;
  timeNow = 0;

  pinMode(pin_interrupt, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_interrupt), hitung_curah_hujan, FALLING); // Akan menghitung tip jika pin berlogika dari HIGH ke LOW

  lightMeter.begin();

  mod.begin(4800);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);

  if (!bmp.begin(0x76))
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
   

  
  // printSerial();
}

void loop()
{
  int val1,val2,val3,val4,val5,val6,val7;
  val1 = nitrogen();
  delay(250);
  val2 = phosphorous();
  delay(250);
  val3 = potassium();
  delay(250);
  val4 = ph();
  delay(250);
  val5 = moist();
  delay(250);
  val6 = stemp();
  delay(250);
  val7 = econd();
  delay(250);
//

//  
//  printSerial();
  Serial.print("Soil Moisture: ");
  Serial.print(val5);
  Serial.print(" %");
  Serial.print("|");
  Serial.print("Soil Temp: ");
  Serial.print(val6);
  Serial.print(" °C");
  Serial.print("|");
  Serial.print("Nitrogen: ");
  Serial.print(val1);
  Serial.print(" mg/kg");
  Serial.print("|");
  Serial.print("Phosphor: ");
  Serial.print(val2);
  Serial.print(" mg/kg");
  Serial.print("|");
  Serial.print("Soil pH: ");
  Serial.print(val4);
  Serial.print(" pH");
  Serial.print("|");
  Serial.print("Electrical Conductivity: ");
  Serial.print(val7);
  Serial.print(" mS/m");
  Serial.println("|");
  Serial.print("Potassium: ");
  Serial.print(val3);
  Serial.print(" mg/kg");
  Serial.print("|");
  

    // Mendapatkan nilai kelembaban, suhu, dan titik embun
    humidityValue = SHT2x.GetHumidity();
    temperatureValue = SHT2x.GetTemperature();
    dewpointValue = SHT2x.GetDewPoint();

    Serial.print("Temperature(C): ");
    Serial.print(temperatureValue);
    Serial.print("|");
    Serial.print("Humidity(%RH): ");
    Serial.print(humidityValue);
    Serial.print("|");
    // Serial.print("Dewpoint(C): ");
    // Serial.print(dewpointValue);
    // Serial.println("|");
    // delay(1000);

  if (lightMeter.measurementReady()) {
    lux = lightMeter.readLightLevel();
    Serial.print("Light Intensity: ");
    Serial.print(lux);
    Serial.print(" lux");
    Serial.print("|");
  }

  // Deklarasi variabel untuk nilai pembacaan sensor dan referensi tegangan GY-ML811
  int uv_Level = analogRead_average(UV_OUT);
  int ref_Level = analogRead_average(REF_3V3);

  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float output_Voltage = 3.3 / ref_Level * uv_Level;

  float uvIntensity = mapfloat(output_Voltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level -numbers from datasheet-

  Serial.print(" UV Light Intensity (mW/cm^2): ");
  Serial.print(uvIntensity);
  Serial.println("|");

    // temperature = bmp.readTemperature(); // Mendapatkan nilai suhu dari sensor
    pressure = bmp.readPressure()/100;       // Mendapatkan nilai tekanan dari sensor
    // altitude = bmp.readAltitude(1014.25); /* Adjusted to local forecast! */

    // Serial.print(F("Temperature = "));
    // Serial.print(temperature);
    // Serial.print(" *C");
    // Serial.print("|");

    Serial.print(F("Air Pressure = "));
    Serial.print(pressure);
    Serial.print(" hPa");
    Serial.println("|");

    // Serial.print(F("Alt = "));
    // Serial.print(altitude);
    // Serial.print(" m");
    // Serial.print("|");

    // put your main code here, to run repeatedly:
    if (Serial1.available()) {
    data = Serial1.readString();
    a = data.indexOf("*");
    b = data.indexOf("#");
    s_angin = data.substring(a + 1, b);

    if (s_angin.equals("1")) { // jika nilai dari sensor 1 maka arah angin utara
      arah_angin = "utara     ";
    }
    if (s_angin.equals("2")) {
      arah_angin = "timur laut";
    }
    if (s_angin.equals("3")) {
      arah_angin = "timur     ";
    }
    if (s_angin.equals("4")) {
      arah_angin = "tenggara  ";
    }
    if (s_angin.equals("5")) {
      arah_angin = "selatan   ";
    }
    if (s_angin.equals("6")) {
      arah_angin = "barat daya";
    }
    if (s_angin.equals("7")) {
      arah_angin = "barat     ";
    }
    if (s_angin.equals("8")) {
      arah_angin = "barat laut";
    }
  
  }
  Serial.print("Wind Wave Direction: ");
  Serial.print(arah_angin);
  Serial.print("|");
//

  if ((millis() - timeold) >= timemeasure * 1000) {
    detachInterrupt(digitalPinToInterrupt(GPIO_pulse)); // Disable interrupt when calculating
    rps = float(rpmcount) / float(timemeasure);         // rotations per second
    rpm = 60 * rps;                                     // rotations per minute
    omega = 2 * PI * rps;                               // rad/s
    velocity_ms = omega * radius * calibration_value;    // m/s
    velocity_kmh = velocity_ms * 3.6;                   // km/h
    
    // Serial.print("RPS =");
    // Serial.print(rps);
    // Serial.print(" s");
    // Serial.println("|");
    // Serial.print(", RPM =");
    // Serial.print(" min");
    // Serial.println("|");
    // Serial.print(rpm);
    Serial.print("Wind Speed =");
    Serial.print(velocity_ms);
    Serial.print(" m/s");
    Serial.println("|");
    // Serial.print(", Velocity_kmh =");
    // Serial.println(velocity_kmh);
    // Serial.print(" km/h");
    // Serial.println("|");
    
    timeold = millis();
    rpmcount = 0;
    attachInterrupt(digitalPinToInterrupt(GPIO_pulse), rpm_anemometer, RISING); // enable interrupt
  }
//
    if (flag == true) // don't really need the == true but makes intent clear for new users
  {
    curah_hujan += milimeter_per_tip; // Akan bertambah nilainya saat tip penuh
    jumlah_tip++;
    // delay(500);
    flag = false; // reset flag
  }
  curah_hujan = jumlah_tip * milimeter_per_tip;
  if ((jumlah_tip != temp_jumlah_tip)) // Print serial setiap 1 menit atau ketika jumlah_tip berubah
  {
    printSerial();
  }
  temp_jumlah_tip = jumlah_tip;
//
//
//

    Serial.print("Lat: ");
    Serial.print(latitude);
    Serial.print("|");
  
    Serial.print("Long: ");
    Serial.print(longitude);
    Serial.print("|");
    Serial.println();
    //Serial2.println();
    // delay(2000);


    delay(5000);
    // Serial2.print("PaketLora: ");
    //  Serial2.print("Soil Moisture: ");
  Serial2.print(val5);
//  Serial2.print(" %");
  Serial2.print(" || ");
//  Serial2.print("Temperature: ");
  Serial2.print(val6);
//  Serial2.print(" C");
  Serial2.print(" || ");
//  Serial2.print("Nitrogen: ");
  Serial2.print(val1);
//  Serial2.print(" mg/kg");
  Serial2.print(" || ");
//  Serial2.print("Phosphorous: ");
  Serial2.print(val2);
//  Serial2.print(" mg/kg");
  Serial2.print(" || ");
//  Serial2.print("Soil pH: ");
  Serial2.print(val4);
//  Serial2.print(" pH");
  //  Serial2.print("Electrical Conductivity: ");
  Serial2.print(" || ");
  Serial2.print(val7);
//  Serial2.print(" mS/m");
  Serial2.print(" || "); 
//  Serial2.print("Potassium: ");
  Serial2.print(val3);
//  Serial2.print(" mg/kg");
  Serial2.print(" || "); 
  Serial2.print(temperatureValue);
  Serial2.print(" || ");
    //Serial.print("IntHumidity(%RH): ");
  Serial2.print(humidityValue);
  Serial2.print(" || ");
    //Serial.print("IntDewpoint(C): ");
  // Serial2.print(dewpointValue);
  // Serial2.print(" || ");
    //Serial.print("Light: ");
  Serial2.print(lux);
    //Serial.print(" lux");
  Serial2.print(" || ");
    //Serial.print(" UV Intensity (mW/cm^2): ");
  Serial2.print(uvIntensity);
  Serial2.print(" || ");
    //Serial.print(F("ExtTemperature = "));
  // Serial2.print(temperature);
    //Serial.print(" *C");
  // Serial2.print(" || ");
    //Serial.print(F("ExtPressure = "));
  Serial2.print(pressure);
  Serial2.print(" || ");
  Serial2.print(arah_angin); 
  Serial2.print(" || ");
  Serial2.print(velocity_ms); 
  Serial2.print(" || ");
  Serial2.print(curah_hujan); 
  Serial2.print(" || ");
    //Serial.print(F("App alt = "));
  // Serial2.print(altitude);
  //   //Serial.print(" m");
  // Serial2.print(" || ");
    //Serial.print("GPS: ");
  Serial2.print(latitude);
  Serial2.print(", ");
  Serial2.print(longitude);
  Serial2.print(" || ");
}

//Takes an average of readings on a given pin
//Returns the average
int analogRead_average(int pinToRead)
{
  int NumberOfSamples = 8;
  int runningValue = 0; 

  for(int x = 0; x < NumberOfSamples; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= NumberOfSamples;

  return(runningValue);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//
//
//
void printSerial()
{
  //Serial.print("Jumlah tip=");
  //Serial.print(jumlah_tip);
  //Serial.print(" kali ");
  //Serial.print("|");
  Serial.print("Rainfall =");
  Serial.print(curah_hujan, 1);
  Serial.print(" mm");
  Serial.println("|");
}

  
int nitrogen(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(nitro,sizeof(nitro))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
    //Serial.print(mod.read(),HEX);
    values[i] = mod.read();
    //Serial2.print(values[i],HEX);
    }
    //Serial2.println();
  }
  return values[4];
}
  
int phosphorous(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(phos,sizeof(phos))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
    //Serial.print(mod.read(),HEX);
    values[i] = mod.read();
    //Serial2.print(values[i],HEX);
    }
    //Serial2.println();
  }
  return values[4];
}
  
int potassium(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(pota,sizeof(pota))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
    //Serial.print(mod.read(),HEX);
    values[i] = mod.read();
    //Serial2.print(values[i],HEX);
    }
    //Serial2.println();
  }
  return values[4];
}


int ph(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(soil_ph,sizeof(soil_ph))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
    //Serial.print(mod.read(),HEX);
    values[i] = mod.read();
    //Serial2.print(values[i],HEX);
    }
    //Serial2.println();
  }
  return values[4];
}


int moist(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(soil_moist,sizeof(soil_moist))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
    //Serial.print(mod.read(),HEX);
    values[i] = mod.read();
    //Serial2.print(values[i],HEX);
    }
    //Serial2.println();
  }
  return values[4];
}


int stemp(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(temp,sizeof(temp))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
    //Serial.print(mod.read(),HEX);
    values[i] = mod.read();
    //Serial2.print(values[i],HEX);
    }
    //Serial2.println();
  }
  return values[4];
}


int econd(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(ec,sizeof(ec))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
    //Serial.print(mod.read(),HEX);
    values[i] = mod.read();
    //Serial2.print(values[i],HEX);
    }
    //Serial2.println();
  }
  return values[4];
}