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

// LEITURA DO RPS
float Reducao_Encoder_Motor = 3;
float Reducao_Motor_Roda = 32;
float Pulsos_Revolucao = 1000;

float RPS_1 = 0;
float RPS_2 = 0;

// NÍVEL DA BATERIA
float Leitura_Bateria = 0;

// TIMER
esp_timer_handle_t Timer_Dados;
#define TEMPO_AMOSTRAGEM 100000 //Valor em us

//////////////////// DECLARAÇÃO DE FUNÇÕES ///////////////////
// CONEXÃO COM A INTERNET
void Wifi_Conect();

// CONEXÃO COM O BANCO DE DADOS
void Firestore_Conect(HTTPClient& http, const char* mask);

// FUNÇÕES PARA VERRIFICAR E ALTERAR O STATUS DE OPERAÇÃO DO ROBÔ
void Read_Operacao();

// FUNÇÕES PARA ENVIO DE DADOS DE OPERAÇÃO
void Send_Data();
String Create_Data_String_JSON(float batteryLevel, float currentSpeed, float cycleTime);

// FUNÇÕES PARA ENVIO DE ALERTAS
void Send_Alert();
String Create_Alert_String_JSON(String Alarm);

// FUNÇÕES PARA INCRMETAR CONTADOR DE PULSOS DO ENCODER
void Conter_Encoder_Motor_1();
void Conter_Encoder_Motor_2();

// FUNÇÃO PARA LEITURA DO RPS
double RPS(long Pulsos);

// TIMER
void IRAM_ATTR Dados(void *arg);


// FUNÇÃO PARA LEITURA DA BATERIA
//void LeituraBateria();

//////////////////////////// SETUP ///////////////////////////
void setup() {
    Serial.begin(115200);

    HTTPClient http; 

    Wifi_Conect();

    pinMode(Pino_Comunicacao, OUTPUT);
    pinMode(Pino_Encoder_Motor_1, INPUT_PULLUP);
    pinMode(Pino_Encoder_Motor_2, INPUT_PULLUP);

    // ENCODERS
    attachInterrupt(digitalPinToInterrupt(Pino_Encoder_Motor_1), Conter_Encoder_Motor_1, FALLING);
    attachInterrupt(digitalPinToInterrupt(Pino_Encoder_Motor_1), Conter_Encoder_Motor_2, FALLING);

    // TIMER
    const esp_timer_create_args_t timer_args = {
    .callback = &Dados,  // Função de callback
    .name = "Timer_Dados"      // Nome do timer (para fins de depuração)
  };

  // Criar o timer
  esp_timer_create(&timer_args, &Timer_Dados);

  // Iniciar o timer com intervalo de 100 ms (100000 microssegundos)
  esp_timer_start_periodic(Timer_Dados, TEMPO_AMOSTRAGEM);  
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
    //Serial.println(StatusOperacao ? "Motor Ligado!" : "Motor Desligado!");
    delay(10);
    Send_Data();
    if(i==1){
      delay(10);
      Send_Alert();
      i++;
    }
    delay(10);
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

void Send_Data() {

  HTTPClient http;
  const char* MascaraDados = "&updateMask.fieldPaths=batteryLevel&updateMask.fieldPaths=currentSpeed&updateMask.fieldPaths=opStatus";
  Firestore_Conect(http, MascaraDados);

  String jsonPayload = Create_Data_String_JSON(BateriaLVL, Velocidade, Robo_Ok);
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

  String jsonPayload = Create_Alert_String_JSON(Alerta);
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

String Create_Alert_String_JSON(String Alarm){

String jsonPayload = "{\"fields\":{"
                       "\"SafetyAlerts\": {\"arrayValue\": {\"values\": ["
                       "{\"stringValue\": \"" + Alarm + "\"}]}}}}";


  return jsonPayload;
}

void Conter_Encoder_Motor_1(){
  Pulsos_Encoder_Motor_1++;
}

void Conter_Encoder_Motor_2(){
  Pulsos_Encoder_Motor_2;
}

// LEITURA RPS 
double RPS(long Pulsos){
  double rotacao = 10; 
  rotacao *= Reducao_Encoder_Motor * Pulsos; //Converte para rotação no motor
  rotacao /= (Reducao_Motor_Roda * Pulsos_Revolucao); //Converte para rotação na roda
  
  return rotacao; //Retorna RPS da roda
}

/*void LeituraBateria(){
  Leitura_Bateria = analogRead(13);
  BateriaLVL = (Leitura_Bateria*100)/4096;
}*/

void IRAM_ATTR Dados(void *args){
  RPS_1 = RPS(Pulsos_Encoder_Motor_1);
  RPS_2 = RPS(Pulsos_Encoder_Motor_2);
  Pulsos_Encoder_Motor_1 = 0;
  Pulsos_Encoder_Motor_2 = 0;
  Velocidade = (RPS_1+RPS_2)/2;
  Robo_Ok = StatusOperacaoAtual;
  BateriaLVL = 87;
}