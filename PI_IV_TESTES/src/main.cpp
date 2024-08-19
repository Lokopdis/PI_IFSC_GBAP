///////////////////////// BIBLIOTECAS /////////////////////////
#include <Arduino.h>
#include <Wifi.h>
#include <HTTPClient.h>

/////////////////////////// DEFINES ///////////////////////////
#define Pino_Comunicacao 18

// ENCODERS
#define Pino_Encoder_Motor_1 5 // Canal A
#define Pino_Encoder_Motor_2 2 // Canal B

// LEITURA BATERIA
//#define Entrada_Bateria 13

////////////////////////// VARIAVEIS //////////////////////////
// CONEXÃO COM A INTERNET
const char* SSID = "Turcatto";
const char* Password = "36641507edu";

// CONEXÃO COM O BANCO DE DADOS
const char* FirestoreURL = "https://firestore.googleapis.com/v1/projects/walle-ifsc/databases/(default)/documents/RobotStatus/robotData?key=AIzaSyCj2pu45s5hq-JhzcduTSn5-2heaquOetg";

// VARIÁVEL PARA ALTERAR O STATUS DE FUNCIONAMENTO DO ROBÔ
bool StatusOperacao = false;
bool StatusOperacaoAtual = false;

// VARIAVEIS DE OPERAÇÃO DO ROBÔ
float BateriaLVL = 87;
float Velocidade = 0; 
bool Robo_Ok = true;

// VARIAVEIS DE ALARME
String Alerta = "Teste!!";
int i = 1;

// VARIAVEIS PARA LEITURA DOS ENCODERS
volatile long Pulsos_Encoder_Motor_1 = 0;
volatile long Pulsos_Encoder_Motor_2 = 0;
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
    if (StatusOperacao == true)
    {
      digitalWrite(Pino_Comunicacao, HIGH);
      Serial.println("Ligouu");
    }else{
      digitalWrite(Pino_Comunicacao, LOW);
      Serial.println("Desligouu");
    }
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
      //Serial.println("isOn field not found in the response");
    }
  } else {
    //Serial.print("Error on sending GET Request: ");
    //Serial.println(httpResponseCode);
  }
  //Serial.println(StatusOperacao);
  http.end(); // Fecha a conexão HTTP
}