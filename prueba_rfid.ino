#include "FirebaseESP8266.h"
#include <ESP8266WiFi.h>

#include <deprecated.h>
#include <MFRC522.h>
#include <MFRC522Extended.h>
#include <require_cpp11.h>

#include <SPI.h>
constexpr uint8_t RST_PIN = D3;     // Configurable, see typical pin layout above
constexpr uint8_t SS_PIN = D4;     // Configurable, see typical pin layout above
MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class
MFRC522::MIFARE_Key key;
String tag;

//seccion de la conexion al wifi

#define FIREBASE_HOST "iot-prueba-548fa.firebaseio.com"
#define FIREBASE_AUTH "Y0l2m1LmuVxmmmM9aIMnXLGzq3tL1K9bMGbKgbfk"
#define WIFI_SSID "Groveriv"
#define WIFI_PASSWORD "grc4695grc4695"
//#define WIFI_SSID "Familia Camargo"
//#define WIFI_PASSWORD "Ariel1290.12"
//#define WIFI_SSID "RFRA"
//#define WIFI_PASSWORD "147258369Ramos"


//Define FirebaseESP8266 data object
FirebaseData firebaseData;

FirebaseJson json;
int punteros=1;
void printResult(FirebaseData &data);
String path = "/Test";
uint8_t i=0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  /**SPI.begin(); // Init SPI bus
  rfid.PCD_Init(); // Init MFRC522**/
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
  
  SPI.begin();
  rfid.PCD_Init();
  
}

  

void loop() {
  
  // put your main code here, to run repeatedly:
if ( ! rfid.PICC_IsNewCardPresent())
    return;
  if (rfid.PICC_ReadCardSerial()) {
    for (byte i = 0; i < 4; i++) {
      tag += rfid.uid.uidByte[i];
    }
    Serial.println(tag);
    if(tag!=""){
            String path = "/prestamo";

    String jsonStr = "";
    
    FirebaseJson json1;
    
    
    FirebaseJsonData jsonObj;
    //String ruta=String(Contador)+"/nombre";
    json1.set(String(punteros) + "/tag", tag);
    json1.set(String(punteros) + "/id", punteros);
   
    

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
    tag = "";
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
  }

delay(1000);

  
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
