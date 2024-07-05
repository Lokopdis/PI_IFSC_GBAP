///////////////////////// BIBLIOTECAS /////////////////////////
#include <Arduino.h>
#include <Wifi.h>
#include <HTTPClient.h>

/////////////////////////// DEFINES ///////////////////////////
#define Pino_Comunicacao 16

////////////////////////// VARIAVEIS //////////////////////////
// CONEXÃO COM A INTERNET
const char* SSID = "Turcatto";
const char* Password = "36641507edu";

// CONEXÃO COM O BANCO DE DADOS
const char* FirestoreURL = "https://firestore.googleapis.com/v1/projects/teste-66dfa/databases/(default)/documents/DeviceStatus/unique-document-id?key=AIzaSyCZBwHhpabgYVOHpOxQykRPRSrq2OYSaO4";

// VARIÁVEL PARA ALTERAR O STATUS DE FUNCIONAMENTO DO ROBÔ
bool StatusOperacao = false;
bool StatusOperacaoAtual = false;

//////////////////// DECLARAÇÃO DE FUNÇÕES ///////////////////
// CONEXÃO COM A INTERNET
void Wifi_Conect();

// CONEXÃO COM O BANCO DE DADOS
void Firestore_Conect(HTTPClient& http, const char* mask);

// FUNÇÕES PARA VERRIFICAR E ALTERAR O STATUS DE OPERAÇÃO DO ROBÔ
void Read_Operacao();

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
    if (StatusOperacao = !StatusOperacaoAtual){
      if (StatusOperacao == true)
      {
        digitalWrite(Pino_Comunicacao, HIGH);
      }else{
        digitalWrite(Pino_Comunicacao, LOW);
      }
      StatusOperacaoAtual = StatusOperacao;
    }
    Serial.println(StatusOperacao ? "Motor Ligado!" : "Motor Desligado!");
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

  const char* MascaraOperacaLeitura = "?mask.fieldPaths=isOn";
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
      } else {
        StatusOperacao = false;
      }
    } else {
      Serial.println("isOn field not found in the response");
    }
  } else {
    Serial.print("Error on sending GET Request: ");
    Serial.println(httpResponseCode);
  }

  http.end(); // Fecha a conexão HTTP
}