///////////////////////// BIBLIOTECAS /////////////////////////
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>

/////////////////////////// DEFINES ///////////////////////////
#define Pino_Comunicacao 18

// ENCODERS
#define Pino_Encoder_Motor_1 5 // Canal A
#define Pino_Encoder_Motor_2 2 // Canal B

////////////////////////// VARIAVEIS //////////////////////////
// CONEXÃO COM A INTERNET
const char* SSID = "Gustavo";
const char* Password = "gustavo123";

// CONEXÃO COM O BANCO DE DADOS
const char* FirestoreURL = "https://firestore.googleapis.com/v1/projects/walle-ifsc/databases/(default)/documents/RobotStatus/robotData?key=AIzaSyCj2pu45s5hq-JhzcduTSn5-2heaquOetg";

// VARIÁVEL PARA ALTERAR O STATUS DE FUNCIONAMENTO DO ROBÔ
bool StatusOperacao = false;

// VARIAVEIS DE OPERAÇÃO DO ROBÔ
float BateriaLVL = 87;
float Velocidade = 0; 

// VARIAVEIS PARA LEITURA DOS ENCODERS
volatile long Pulsos_Encoder_Motor_1 = 0;
volatile long Pulsos_Encoder_Motor_2 = 0;

// LEITURA DO RPS
float Reducao_Encoder_Motor = 3;
float Reducao_Motor_Roda = 32;
float Pulsos_Revolucao = 1000;

double RPS_1 = 0;
double RPS_2 = 0;

//////////////////// DECLARAÇÃO DE FUNÇÕES ///////////////////
// CONEXÃO COM A INTERNET
void Wifi_Conect();

// CONEXÃO COM O BANCO DE DADOS
void Firestore_Conect(HTTPClient& http, const char* mask);

// FUNÇÕES PARA VERRIFICAR E ALTERAR O STATUS DE OPERAÇÃO DO ROBÔ
void Read_Operacao(void* pvParameters);

// FUNÇÕES PARA ENVIO DE DADOS DE OPERAÇÃO
void Send_Data();
String Create_Data_String_JSON(float batteryLevel, float currentSpeed);

// FUNÇÕES PARA INCRMENTAR CONTADOR DE PULSOS DO ENCODER
void Conter_Encoder_Motor_1();
void Conter_Encoder_Motor_2();

// FUNÇÃO PARA LEITURA DO RPS
double RPS(long Pulsos);

// FUNÇÃO PARA LEITURA E ENVIO DA VELOCIDADE
void LeituraEnvioVelocidade(void* pvParameters);

//////////////////////////// SETUP ///////////////////////////
void setup() {
    Serial.begin(115200);

    Wifi_Conect();

    pinMode(Pino_Comunicacao, OUTPUT);
    pinMode(Pino_Encoder_Motor_1, INPUT_PULLUP);
    pinMode(Pino_Encoder_Motor_2, INPUT_PULLUP);

    // ENCODERS
    attachInterrupt(digitalPinToInterrupt(Pino_Encoder_Motor_1), Conter_Encoder_Motor_1, FALLING);
    attachInterrupt(digitalPinToInterrupt(Pino_Encoder_Motor_2), Conter_Encoder_Motor_2, FALLING);

    // Criação das tasks para os dois cores
    xTaskCreatePinnedToCore(Read_Operacao, "TaskOperacao", 10000, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(LeituraEnvioVelocidade, "TaskVelocidade", 10000, NULL, 1, NULL, 1);
}

/////////////////////////// LOOP ////////////////////////////
void loop() {
  // O loop principal fica vazio pois as tasks estão sendo executadas em paralelo
  delay(1000); 
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
void Read_Operacao(void* pvParameters) {
  for (;;) { // Loop infinito para a task
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
          digitalWrite(Pino_Comunicacao, HIGH);
          Serial.println("Ligado");
        } else {
          StatusOperacao = false;
          digitalWrite(Pino_Comunicacao, LOW);
          Serial.println("Desligado");
        }
      } else {
        //Serial.println("isOn field not found in the response");
      }
    } else {
      Serial.print("Error on sending GET Request: ");
      Serial.println(httpResponseCode);
    }

    http.end(); // Fecha a conexão HTTP

    delay(1000); // Delay para evitar overloading da task
  }
}

void LeituraEnvioVelocidade(void* pvParameters) {
  for (;;) { // Loop infinito para a task
    RPS_1 = RPS(Pulsos_Encoder_Motor_1);
    Velocidade = RPS_1;
    Pulsos_Encoder_Motor_1 = 0;
    //Serial.println(Velocidade);
    Send_Data();

    delay(500); // Delay para evitar overloading da task
  }
}

void Send_Data() {
  HTTPClient http;
  const char* MascaraDados = "&updateMask.fieldPaths=batteryLevel&updateMask.fieldPaths=currentSpeed";
  Firestore_Conect(http, MascaraDados);

  String jsonPayload = Create_Data_String_JSON(BateriaLVL, Velocidade);
  int httpResponseCode = http.sendRequest("PATCH", jsonPayload);

  if (httpResponseCode > 0) {
    String response = http.getString();
    //Serial.println(httpResponseCode);
    //Serial.println(response);
  } else {
    //Serial.print("Error on sending PATCH Request: ");
    //Serial.println(httpResponseCode);
  }

  http.end(); // Fecha a conexão HTTP
}

String Create_Data_String_JSON(float batteryLevel, float currentSpeed){
  String StrbatteryLevel = String(batteryLevel, 3);
  String StrcurrentSpeed = String(currentSpeed, 3);

  // Atualize apenas os campos batteryLevel e currentSpeed sem afetar os outros campos
  String jsonPayload = "{\"fields\":{"
                       "\"batteryLevel\": {\"doubleValue\":" + StrbatteryLevel + "},"
                       "\"currentSpeed\": {\"doubleValue\":" + StrcurrentSpeed + "}}}";

  return jsonPayload;
}

void Conter_Encoder_Motor_1(){
  Pulsos_Encoder_Motor_1++;
}

void Conter_Encoder_Motor_2(){
  Pulsos_Encoder_Motor_2++;
}

// LEITURA RPS 
double RPS(long Pulsos){
  double rotacao = 10; 
  rotacao *= Reducao_Encoder_Motor * Pulsos; //Converte para rotação no motor
  rotacao /= (Reducao_Motor_Roda * Pulsos_Revolucao); //Converte para rotação na roda
  
  return rotacao; //Retorna RPS da roda
}
