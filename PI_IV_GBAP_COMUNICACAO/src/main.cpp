///////////////////////// BIBLIOTECAS /////////////////////////
#include <Arduino.h>
#include <Wifi.h>
#include <HTTPClient.h>

/////////////////////////// DEFINES ///////////////////////////
#define Pino_Comunicacao 18

////////////////////////// VARIAVEIS //////////////////////////
// CONEXÃO COM A INTERNET
const char* SSID = "Gustavo";
const char* Password = "gustavo123";

// CONEXÃO COM O BANCO DE DADOS
const char* FirestoreURL = "https://firestore.googleapis.com/v1/projects/walle-ifsc/databases/(default)/documents/RobotStatus/robotData?key=AIzaSyCj2pu45s5hq-JhzcduTSn5-2heaquOetg";

// VARIÁVEL PARA ALTERAR O STATUS DE FUNCIONAMENTO DO ROBÔ
bool StatusOperacao = false;
bool StatusOperacaoAtual = false;

float BateriaLVL = 876;
float Velocidade = 45678; 
bool Robo_Ok = true;

String Alerta = "fodeu";
int i = 1;

//////////////////// DECLARAÇÃO DE FUNÇÕES ///////////////////
// CONEXÃO COM A INTERNET
void Wifi_Conect();

// CONEXÃO COM O BANCO DE DADOS
void Firestore_Conect(HTTPClient& http, const char* mask);

// FUNÇÕES PARA VERRIFICAR E ALTERAR O STATUS DE OPERAÇÃO DO ROBÔ
void Read_Operacao();

void Send_Data();
String Create_Data_String_JSON(float batteryLevel, float currentSpeed, float cycleTime);

void Send_Alert();
String Create_Data_JSON(String Alarm);

//////////////////////////// SETUP ///////////////////////////
void setup() {
    Serial.begin(115200);

    HTTPClient http; 

    Wifi_Conect();

    pinMode(Pino_Comunicacao, OUTPUT);  
}

/////////////////////////// LOOP ////////////////////////////
void loop() {
    Read_Operacao();
      if (StatusOperacao == true)
      {
        digitalWrite(Pino_Comunicacao, HIGH);
      }else{
        digitalWrite(Pino_Comunicacao, LOW);
      }
      StatusOperacaoAtual = StatusOperacao;
    Robo_Ok = StatusOperacao;
    Serial.println(StatusOperacao ? "Motor Ligado!" : "Motor Desligado!");
    delay(100);
    Send_Data();
    if(i==1){
      delay(100);
      Send_Alert();
      i++;
    }
    delay(400);
}

///////////////////////// FUNÇÕES ///////////////////////////
// CONEXÃO COM A INTERNET
void Wifi_Conect() {
  WiFi.begin(SSID, Password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}

// CONEXÃO COM O BANCO DE DADOS
void Firestore_Conect(HTTPClient& http, const char* mask) {
  String url = String(FirestoreURL) + String(mask);
  http.begin(url);  // Inicia a conexão com a URL
  http.addHeader("Content-Type", "application/json");
}

// FUNÇÕES PARA VERRIFICAR E ALTERAR O STATUS DE OPERAÇÃO DO ROBÔ
void Read_Operacao() {
  HTTPClient http;

  const char* MascaraOperacaLeitura = "?mask.fieldPaths=remotePower";
  Firestore_Conect(http, MascaraOperacaLeitura);

  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    String response = http.getString();
    // Parse JSON response to get the value of isOn
    int startIndex = response.indexOf("\"booleanValue\":");
    if (startIndex != -1) {
      startIndex += strlen("\"booleanValue\":");
      int endIndex = response.indexOf("}", startIndex);
      String isOnValue = response.substring(startIndex, endIndex);
      isOnValue.trim(); // Remove leading and trailing spaces, if any

      if (isOnValue.equals("true")) {
        StatusOperacao = true;
        //Serial.println("1");
      } else {
        StatusOperacao = false;
        //Serial.println("0");
      }
    } else {
      Serial.println("isOn field not found in the response");
    }
  } else {
    Serial.print("Error on sending GET Request: ");
    Serial.println(httpResponseCode);
  }
  Serial.println(StatusOperacao);
  http.end(); // Fecha a conexão HTTP
}

void Send_Data() {

  HTTPClient http;
  const char* MascaraDados = "&updateMask.fieldPaths=batteryLevel&updateMask.fieldPaths=currentSpeed&updateMask.fieldPaths=opStatus";
  Firestore_Conect(http, MascaraDados);

  String jsonPayload = Create_Data_String_JSON(BateriaLVL, Velocidade, Robo_Ok);
  int httpResponseCode = http.sendRequest("PATCH", jsonPayload);

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println(httpResponseCode);
    Serial.println(response);
  } else {
    Serial.print("Error on sending PATCH Request: ");
    Serial.println(httpResponseCode);
  }

  http.end(); // Fecha a conexão HTTP
}

String Create_Data_String_JSON(float batteryLevel, float currentSpeed, float cycleTime){
  String StrbatteryLevel = String(batteryLevel, 3);
  String StrcurrentSpeed = String(currentSpeed, 3);
  String StrcycleTime = String(cycleTime);

  // Atualize apenas os campos batteryLevel e currentSpeed sem afetar os outros campos
  String jsonPayload = "{\"fields\":{"
                       "\"batteryLevel\": {\"doubleValue\":" + StrbatteryLevel + "},"
                       "\"currentSpeed\": {\"doubleValue\":" + StrcurrentSpeed + "},"
                       "\"opStatus\": {\"booleanValue\":" + StrcycleTime + "}}}";

  return jsonPayload;
}


void Send_Alert(){
  HTTPClient http;
  const char* MascaraAlert = "&updateMask.fieldPaths=SafetyAlerts";  // Atualiza apenas o campo "dataString"
  Firestore_Conect(http, MascaraAlert);

  String jsonPayload = Create_Data_JSON(Alerta);
  int httpResponseCode = http.sendRequest("PATCH", jsonPayload);

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println(httpResponseCode);
    Serial.println(response);
  } else {
    Serial.print("Error on sending PATCH Request: ");
    Serial.println(httpResponseCode);
  }

  http.end(); // Fecha a conexão HTTP
}

String Create_Data_JSON(String Alarm){

String jsonPayload = "{\"fields\":{"
                       "\"SafetyAlerts\": {\"arrayValue\": {\"values\": ["
                       "{\"stringValue\": \"" + Alarm + "\"}]}}}}";


  return jsonPayload;
}