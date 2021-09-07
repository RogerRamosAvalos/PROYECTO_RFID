#include <Wire.h>
#include "FirebaseESP8266.h"
#include <ESP8266WiFi.h>

#define FIREBASE_HOST "iot-prueba-548fa.firebaseio.com"
#define FIREBASE_AUTH "Y0l2m1LmuVxmmmM9aIMnXLGzq3tL1K9bMGbKgbfk"
//#define WIFI_SSID "Groveriv"
//#define WIFI_PASSWORD "grc4695grc4695"
#define WIFI_SSID "Familia Camargo"
 #define WIFI_PASSWORD "Ariel1290.12"
//#define WIFI_SSID "RFRA"
//#define WIFI_PASSWORD "147258369Ramos"
#define MPU 0x68

//Ratios de conversion
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250

//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779

//MPU-6050 da los valores en enteros de 16 bits
//Valores RAW
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

//Angulos
float Acc[2];
float Gy[3];
float Angle[3];

String valores;
String ariel;
long tiempo_prev;
float dt;
int distancia;

//Define FirebaseESP8266 data object
FirebaseData firebaseData;

FirebaseJson json;

void printResult(FirebaseData &data);

int punteros = 1;
int punteros1 = 1;
void setup() {
  // put your setup code here, to run once:
  Wire.begin(4, 5); // D2(GPIO4)=SDA / D1(GPIO5)=SCL
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  //Set the size of WiFi rx/tx buffers in the case where we want to work with large data.
  firebaseData.setBSSLBufferSize(1024, 1024);

  //Set the size of HTTP response buffers in the case where we want to work with large data.
  firebaseData.setResponseSize(1024);

  //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Leer los valores del Acelerometro de la IMU
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); //A partir del 0x3B, se piden 6 registros
  AcX = Wire.read() << 8 | Wire.read(); //Cada valor ocupa 2 registros
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  //A partir de los valores del acelerometro, se calculan los angulos Y, X
  //respectivamente, con la formula de la tangente.
  Acc[1] = atan(-1 * (AcX / A_R) / sqrt(pow((AcY / A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG;
  Acc[0] = atan((AcY / A_R) / sqrt(pow((AcX / A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG;


  //Leer los valores del Giroscopio
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); //A partir del 0x43, se piden 6 registros
  GyX = Wire.read() << 8 | Wire.read(); //Cada valor ocupa 2 registros
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  //Calculo del angulo del Giroscopio
  Gy[0] = GyX / G_R;
  Gy[1] = GyY / G_R;
  Gy[2] = GyZ / G_R;

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  //Aplicar el Filtro Complementario
  Angle[0] = 0.98 * (Angle[0] + Gy[0] * dt) + 0.02 * Acc[0];
  Angle[1] = 0.98 * (Angle[1] + Gy[1] * dt) + 0.02 * Acc[1];

  //Integración respecto del tiempo paras calcular el YAW
  Angle[2] = Angle[2] + Gy[2] * dt;

  //Mostrar los valores por consola
  valores = "Gx: " + String(Angle[0]) + "| Gy:" + String(Angle[1]) + "| Gz:" + String(Angle[2]) + "| Ax: " + String(AcX / A_R) + "| Ay: " + String(AcY / A_R) + "| Az:" + String(AcZ / A_R);
  Serial.println(valores);
  distancia = ((AcX / A_R) * pow((30), 2)) / 2;


  //======================================== fragmento de carga de valores en x y z de los angulos del giroscopio===============

  if(distancia <250){
    Serial.println("Sigue dentro del rango");
    String path = "/tabla";

    String jsonStr = "";
    
    FirebaseJson json1;
    
    
    FirebaseJsonData jsonObj;
    json1.set(String(punteros1) + "/valorX", String(AcX / A_R));
    json1.set(String(punteros1) + "/valorY", String(AcY / A_R));
    json1.set(String(punteros1) + "/valorZ", String(AcZ / A_R));
    json1.set(String(punteros1) + "/description", "Computadora Hp");
    json1.set(String(punteros1) + "/id", punteros);
    json1.set(String(punteros1) + "/task", "Sigue en el rango");
    json1.set(String(punteros1) + "/distanciaCM", distancia);
    json1.set(String(punteros1) + "/giroX",String(Angle[0]) );
    json1.set(String(punteros1) + "/giroY",String(Angle[1]) );
    json1.set(String(punteros1) + "/giroZ",String(Angle[2]));
    

    Serial.println("------------------------------------");
    Serial.println("JSON Data");
    json1.toString(jsonStr, true);
    Serial.println(jsonStr);
    Serial.println("------------------------------------");

    Serial.println("------------------------------------");
    Serial.println("Set JSON test...");

    if (Firebase.push(firebaseData, path, json1))
    {
      Serial.println("PASSED");
      Serial.println("PATH: " + firebaseData.dataPath());
      Serial.println("TYPE: " + firebaseData.dataType());
      Serial.print("VALUE: ");
      printResult(firebaseData);
      Serial.println("------------------------------------");
      Serial.println();
     punteros1=punteros1+1;
    }
    else
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + firebaseData.errorReason());
      Serial.println("------------------------------------");
      Serial.println();
    }
    
    }else{
      Serial.println("Sigue dentro del rango");
    String path = "/tabla";

    String jsonStr = "";
    
    FirebaseJson json1;
    
    
    FirebaseJsonData jsonObj;
    json1.set(String(punteros1) + "/valorX", String(AcX / A_R));
    json1.set(String(punteros1) + "/valorY", String(AcY / A_R));
    json1.set(String(punteros1) + "/valorZ", String(AcZ / A_R));
    json1.set(String(punteros1) + "/description", "Computadora Hp");
    json1.set(String(punteros1) + "/id", punteros);
    json1.set(String(punteros1) + "/task", "Salio del rango admitido");
    json1.set(String(punteros1) + "/distanciaCM", distancia);
    json1.set(String(punteros1) + "/giroX",String(Angle[0]) );
    json1.set(String(punteros1) + "/giroY",String(Angle[1]) );
    json1.set(String(punteros1) + "/giroZ",String(Angle[2]));
    

    Serial.println("------------------------------------");
    Serial.println("JSON Data");
    json1.toString(jsonStr, true);
    Serial.println(jsonStr);
    Serial.println("------------------------------------");

    Serial.println("------------------------------------");
    Serial.println("Set JSON test...");

    if (Firebase.push(firebaseData, path, json1))
    {
      Serial.println("PASSED");
      Serial.println("PATH: " + firebaseData.dataPath());
      Serial.println("TYPE: " + firebaseData.dataType());
      Serial.print("VALUE: ");
      printResult(firebaseData);
      Serial.println("------------------------------------");
      Serial.println();
     punteros1=punteros1+1;
    }
    else
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + firebaseData.errorReason());
      Serial.println("------------------------------------");
      Serial.println();
    }
      
      }
  

  //============================================================================================================================
  
  if (distancia < 250) {
    Serial.println("Sigue dentro del rango");
    String path = "/task";

    String jsonStr = "";
    
    FirebaseJson json1;
    
    
    FirebaseJsonData jsonObj;
    //String ruta=String(Contador)+"/nombre";
    json1.set(String(punteros) + "/description", "Computadora Hp");
    json1.set(String(punteros) + "/id", punteros);
    json1.set(String(punteros) + "/task", "Sigue en el rango");
    

    Serial.println("------------------------------------");
    Serial.println("JSON Data");
    json1.toString(jsonStr, true);
    Serial.println(jsonStr);
    Serial.println("------------------------------------");

    Serial.println("------------------------------------");
    Serial.println("Set JSON test...");

    if (Firebase.set(firebaseData, path, json1))
    {
      Serial.println("PASSED");
      Serial.println("PATH: " + firebaseData.dataPath());
      Serial.println("TYPE: " + firebaseData.dataType());
      Serial.print("VALUE: ");
      printResult(firebaseData);
      Serial.println("------------------------------------");
      Serial.println();
     punteros=punteros+1;
    }
    else
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + firebaseData.errorReason());
      Serial.println("------------------------------------");
      Serial.println();
    }

    Serial.println("------------------------------------");
    Serial.println("Get JSON test...");
  } else {
    Serial.println("Salio del rango establecido");
    String path = "/task";

    String jsonStr = "";
    
    FirebaseJson json1;
    
    
    FirebaseJsonData jsonObj;
    //String ruta=String(Contador)+"/nombre";
    json1.set(String(punteros) + "/description", "Computadora Hp");
    json1.set(String(punteros) + "/id", punteros);
    json1.set(String(punteros) + "/task", "Se salio del Rango");
    

    Serial.println("------------------------------------");
    Serial.println("JSON Data");
    json1.toString(jsonStr, true);
    Serial.println(jsonStr);
    Serial.println("------------------------------------");

    Serial.println("------------------------------------");
    Serial.println("Set JSON test...");

    if (Firebase.set(firebaseData, path, json1))
    {
      Serial.println("PASSED");
      Serial.println("PATH: " + firebaseData.dataPath());
      Serial.println("TYPE: " + firebaseData.dataType());
      Serial.print("VALUE: ");
      printResult(firebaseData);
      Serial.println("------------------------------------");
      Serial.println();
     punteros=punteros+1;
    }
    else
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + firebaseData.errorReason());
      Serial.println("------------------------------------");
      Serial.println();
    }

    Serial.println("------------------------------------");
    Serial.println("Get JSON test...");
  }
  Serial.println("La distancia es :" + String(distancia));
  delay(2000);
}
void printResult(FirebaseData &data)
{

  if (data.dataType() == "int")
    Serial.println(data.intData());
  else if (data.dataType() == "float")
    Serial.println(data.floatData(), 5);
  else if (data.dataType() == "double")
    printf("%.9lf\n", data.doubleData());
  else if (data.dataType() == "boolean")
    Serial.println(data.boolData() == 1 ? "true" : "false");
  else if (data.dataType() == "string")
    Serial.println(data.stringData());
  else if (data.dataType() == "json")
  {
    Serial.println();
    FirebaseJson &json = data.jsonObject();
    //Print all object data
    Serial.println("Pretty printed JSON data:");
    String jsonStr;
    json.toString(jsonStr, true);
    Serial.println(jsonStr);
    Serial.println();
    Serial.println("Iterate JSON data:");
    Serial.println();
    size_t len = json.iteratorBegin();
    String key, value = "";
    int type = 0;
    for (size_t i = 0; i < len; i++)
    {
      json.iteratorGet(i, type, key, value);
      Serial.print(i);
      Serial.print(", ");
      Serial.print("Type: ");
      Serial.print(type == FirebaseJson::JSON_OBJECT ? "object" : "array");
      if (type == FirebaseJson::JSON_OBJECT)
      {
        Serial.print(", Key: ");
        Serial.print(key);
      }
      Serial.print(", Value: ");
      Serial.println(value);
    }
    json.iteratorEnd();
  }
  else if (data.dataType() == "array")
  {
    Serial.println();
    //get array data from FirebaseData using FirebaseJsonArray object
    FirebaseJsonArray &arr = data.jsonArray();
    //Print all array values
    Serial.println("Pretty printed Array:");
    String arrStr;
    arr.toString(arrStr, true);
    Serial.println(arrStr);
    Serial.println();
    Serial.println("Iterate array values:");
    Serial.println();
    for (size_t i = 0; i < arr.size(); i++)
    {
      Serial.print(i);
      Serial.print(", Value: ");

      FirebaseJsonData &jsonData = data.jsonData();
      //Get the result data from FirebaseJsonArray object
      arr.get(jsonData, i);
      if (jsonData.typeNum == FirebaseJson::JSON_BOOL)
        Serial.println(jsonData.boolValue ? "true" : "false");
      else if (jsonData.typeNum == FirebaseJson::JSON_INT)
        Serial.println(jsonData.intValue);
      else if (jsonData.typeNum == FirebaseJson::JSON_FLOAT)
        Serial.println(jsonData.floatValue);
      else if (jsonData.typeNum == FirebaseJson::JSON_DOUBLE)
        printf("%.9lf\n", jsonData.doubleValue);
      else if (jsonData.typeNum == FirebaseJson::JSON_STRING ||
               jsonData.typeNum == FirebaseJson::JSON_NULL ||
               jsonData.typeNum == FirebaseJson::JSON_OBJECT ||
               jsonData.typeNum == FirebaseJson::JSON_ARRAY)
        Serial.println(jsonData.stringValue);
    }
  }
  else
  {
    Serial.println(data.payload());
  }
}