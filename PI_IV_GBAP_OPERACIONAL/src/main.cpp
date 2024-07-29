///////////////////////// BIBLIOTECAS /////////////////////////
#include <Arduino.h>
#include <Wire.h>
#include "esp_timer.h"

///////////////////////// DIRETRIZES //////////////////////////
#define INTERRUPCAO false
#define TESTE true

/////////////////////////// DEFINES ///////////////////////////
// LÓGICA DE ACIONAMENTO
#define Pino_Botao_Emergencia 15
#define Pino_Chave_ON_OFF 16
#define Pino_Rele_Seguranca 27

// ACIONAMENTO REMOTO
#define Pino_Acionamento_Remoto 33 // Alterar

// MOTOR 1 ----->
// ENCODER
#define Encoder_Canal_A_Motor_1 5 // Canala A do encoder do motor 1
#define Encoder_Canal_B_Motor_1 17 // Canal B do encoder do motor 1

// SAÍDA ANALÓGICA
#define Pino_Motor_1 25 // Saída analógica para o motor 1

// MOTOR 2 <-----
//ENCODER
#define Encoder_Canal_A_Motor_2 4 // Canal A do encoder do motor 2
#define Encoder_Canal_B_Motor_2 2 // Canal B do encoder do motor 2

// SAÍDA ANALÓGICA
#define Pino_Motor_2 26 // Saída de analógica para o motor 2

// GERAL
#define TAXA_AMOSTRAGEM 100 // Valor em ms

////////////////////////// VARIAVEIS //////////////////////////
// LÓGICA DE ACIONAMENTO
bool Botao_Emergencia_Pressionado = false; // Verrifica botão de emergência precionado
bool Chave_ON_OFF_Anterior = false; // Salva status anterior da chave liga e desliga
bool Robo_Ok = false; // Variável de acionamento do robô

// ACIONAMENTO REMOTO
bool Status_Remoto = false;

// ENCODER GERAL
const double Pulsos_Por_Revolucao_Encoder = 1000; // Número de pulsos por revolução do encoder
const double Reducao_Encoder_Motor = 3; // Redução entre encoder e eixo do motor
const double Reducao_Motor_Roda = 32; // Rudação entre eixo do motor e roda

// MOTOR 1 ----->
// ENCODER 
volatile long Contador_Pulsos_Canal_A_Motor_1 = 0; // Contador de pulsos do canal A do encoder do motor 1
volatile long Contador_Pulsos_Canal_B_Motor_1 = 0; // Contador de pulsos do canal B do encoder do motor 1

// PWM
int PWM_Motor_1 = 3103; // Valor inicial para velocidade do motor 1

// RPS
double RPS_Motor_1 = 0; // Valor em m/s do motor 1
long Aux_Convert_PWM_Motor_1 = 0; // Variável auxiliar para conversão do valor do PWM de 10 p/ 8 bits

// CONTROLE
// VELOCIDADE
float Controle_Velocidade_Anterior_Motor_1 = 0;
float Controle_Velocidade_Atual_Motor_1 = 0;
float Kp_Velocidade_Motor_1 = 0;
float Alpha_Velocidade_Motor_1 = 0;
float Erro_Velocidade_Atual_Motor_1 = 0;
float Erro_Velocidade_Anterior_Motor_1 = 0;
float Referencia_Velocidade_Motor_1 = 0;

// ÂNGULO
float Referencia_Anterior_Motor_1 = 0;
float Kp_Angulo_Motor_1 = 0;
float Alpha_Angulo_Motor_1 = 0;
float Erro_Angulo_Atual_Motor_1 = 0;
float Erro_Angulo_Anterior_Motor_1 = 0;
float Referencia_Angulo_Motor_1 = 0;

// MOTOR 2 <-----
// ENCODER 
volatile long Contador_Pulsos_Canal_A_Motor_2 = 0; // Contador de pulsos do canal A do encoder do motor 2
volatile long Contador_Pulsos_Canal_B_Motor_2 = 0; // Contador de pulsos do canal B do encoder do motor 2

// PWM
int PWM_Motor_2 = 3103; // Valor inicial para velocidade do motor 2

// RPS
double RPS_Motor_2 = 0; // Valor em m/s do motor 2
long Aux_Convert_PWM_Motor_2 = 0; // Variável auxiliar para conversão do valor do PWM de 10 p/ 8 bits

// CONTROLE
// VELOCIDADE
float Controle_Velocidade_Anterior_Motor_2 = 0;
float Controle_Velocidade_Atual_Motor_2 = 0;
float Kp_Velocidade_Motor_2 = 0;
float Alpha_Velocidade_Motor_2 = 0;
float Erro_Velocidade_Atual_Motor_2 = 0;
float Erro_Velocidade_Anterior_Motor_2 = 0;
float Referencia_Velocidade_Motor_2 = 0;

// ÂNGULO
float Referencia_Anterior_Motor_2 = 0;
float Kp_Angulo_Motor_2 = 0;
float Alpha_Angulo_Motor_2 = 0;
float Erro_Angulo_Atual_Motor_2 = 0;
float Erro_Angulo_Anterior_Motor_2 = 0;
float Referencia_Angulo_Motor_2 = 0;

#if TESTE == true
// TIMER PARA TESTE
esp_timer_handle_t periodic_timer;
#endif

// GERAL
int Motor_Index = 0;

//////////////////// DECLARAÇÃO DE FUNÇÕES ///////////////////
// SETUP INICIAL 
void SetupInicial ();

// LÓGICA DE ACIONAMENTO
void StatusAcionamento();

// ACIONAMENTO REMOTO
void AcionamentoRemoto();

// MOTOR 1 ----->
// ENCODER 
void Incrementar_Pulsos_Canal_A_Motor_1();
void Incrementar_Pulsos_Canal_B_Motor_1();

// MOTOR 2 <-----
// ENCODER 
void Incrementar_Pulsos_Canal_A_Motor_2();
void Incrementar_Pulsos_Canal_B_Motor_2();

// CONTROLE
// Velocidade
void Controle_Velocidade(int Index);

// ÂNGULO
void Controle_Angulo(int Index);

#if TESTE == true
// TIMER PARA TESTE
void IRAM_ATTR onTimer(void* arg);
#endif

//////////////////////////// SETUP ///////////////////////////
void setup() {
  Serial.begin(115200);

  // LÓGICA DE ACIONAMENTO
  pinMode(Pino_Botao_Emergencia, INPUT_PULLDOWN); // Define pino do botão de mergência como entrada
  pinMode(Pino_Chave_ON_OFF, INPUT_PULLDOWN); // Define pino da chave liga e desliga com entrada
  pinMode(Pino_Rele_Seguranca, OUTPUT); // Define pino do relé de segurança como saída

  digitalWrite(Pino_Rele_Seguranca, HIGH); // Garante que o relé estará desacionado

  // ACIONAMENTO REMOTO
  pinMode(Pino_Acionamento_Remoto, INPUT_PULLDOWN);

  // INTERRUPÇÕES
  // LÓGICA DE ACIOANEMNTO
  attachInterrupt(digitalPinToInterrupt(Pino_Botao_Emergencia), StatusAcionamento, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pino_Chave_ON_OFF), StatusAcionamento, CHANGE);

  // ACIONAMENTO REMOTO
  attachInterrupt(digitalPinToInterrupt(Pino_Acionamento_Remoto), AcionamentoRemoto, CHANGE);

  // MOTOR 1 ----->
  // ENCODER
  attachInterrupt(digitalPinToInterrupt(Encoder_Canal_A_Motor_1), Incrementar_Pulsos_Canal_A_Motor_1, FALLING);
  attachInterrupt(digitalPinToInterrupt(Encoder_Canal_B_Motor_1), Incrementar_Pulsos_Canal_B_Motor_1, FALLING);

  // PWM
  pinMode(Pino_Motor_1, OUTPUT); 

  // MOTOR 2 <-----
  // ENCODER
  attachInterrupt(digitalPinToInterrupt(Encoder_Canal_A_Motor_2), Incrementar_Pulsos_Canal_A_Motor_2, FALLING);
  attachInterrupt(digitalPinToInterrupt(Encoder_Canal_B_Motor_2), Incrementar_Pulsos_Canal_B_Motor_2, FALLING); 

    // PWM
  pinMode(Pino_Motor_1, OUTPUT);

  #if TESTE == true
  // TIMER PARA TESTE
  const esp_timer_create_args_t timer_args = {
    .callback = &onTimer,
    .name = "periodic_timer"
  };

  esp_timer_create(&timer_args, &periodic_timer);
  esp_timer_start_periodic(periodic_timer, 100000); // 100000 us = 100 ms
  #endif

}

/////////////////////////// LOOP ////////////////////////////
void loop() {  

}

///////////////////////// FUNÇÕES ///////////////////////////
// SETUP INICIAL
void SetupInicial(){

}

// LÓGICA DE ACIONAMENTO
void StatusAcionamento (){

  bool Emergencia_Atual = digitalRead(Pino_Botao_Emergencia); // Lê status atual do botão de emergência

   if (Emergencia_Atual == LOW && Botao_Emergencia_Pressionado == LOW) {
    Robo_Ok = LOW; 
    Botao_Emergencia_Pressionado = LOW;
    digitalWrite(Pino_Rele_Seguranca, HIGH); // Desaciona o Relé
  }else if (Emergencia_Atual == LOW && Botao_Emergencia_Pressionado == HIGH) {
    Botao_Emergencia_Pressionado = LOW;
    digitalWrite(Pino_Botao_Emergencia, HIGH); // Desaciona o Relé
  }

  bool Chave_ON_OFF_Atual = digitalRead(Pino_Chave_ON_OFF); // Lê status atual do botão de emergência

  // Verifica a borda de subida na chave liga e desliga
  if (Chave_ON_OFF_Atual == HIGH && Chave_ON_OFF_Anterior == LOW) {
    if (!Botao_Emergencia_Pressionado) { // Verifica se o botão de emergência não está pressionado
      Robo_Ok = !Robo_Ok; // Altera o Estado de acionamento do robô LOW --> HIGH
      digitalWrite(Pino_Rele_Seguranca, LOW); // Aciona o Relé  
    }
  }
  if (Chave_ON_OFF_Atual == LOW) {
      Robo_Ok = LOW;
      digitalWrite(Pino_Rele_Seguranca, HIGH); // Desaciona o Relé
  }

  Chave_ON_OFF_Anterior = Chave_ON_OFF_Atual;

}

// ACIONAEMNTO REMOTO
void AcionamentoRemoto(){
  Status_Remoto = digitalRead(Pino_Acionamento_Remoto);
  if (Status_Remoto == HIGH){
    digitalWrite(Pino_Rele_Seguranca, LOW);
  }else{
    digitalWrite(Pino_Rele_Seguranca, HIGH);
  }
}

// MOTOR 1 ----->
// ENCODER 
void Incrementar_Pulsos_Canal_A_Motor_1(){
  Contador_Pulsos_Canal_A_Motor_1++;
}

void Incrementar_Pulsos_Canal_B_Motor_1(){
  Contador_Pulsos_Canal_B_Motor_1++;
}

// MOTOR 2 <-----
// ENCODER 
void Incrementar_Pulsos_Canal_A_Motor_2(){
  Contador_Pulsos_Canal_A_Motor_2++;
}

void Incrementar_Pulsos_Canal_B_Motor_2(){
  Contador_Pulsos_Canal_B_Motor_2++;
}

// CONTROLE
// VELOCIDADE
void Controle_Velocidade (int Index){
  if (Index == 1){

    RPS_Motor_1 = 10;
    RPS_Motor_1 *= Reducao_Encoder_Motor*Contador_Pulsos_Canal_A_Motor_1;
    RPS_Motor_1 /= Reducao_Motor_Roda*Pulsos_Por_Revolucao_Encoder;

    Contador_Pulsos_Canal_A_Motor_1 = 0;
    Contador_Pulsos_Canal_B_Motor_1 = 0;

  Erro_Velocidade_Atual_Motor_1 = Referencia_Velocidade_Motor_1 - RPS_Motor_1;
  
  Controle_Velocidade_Atual_Motor_1 = (Controle_Velocidade_Anterior_Motor_1 + (Kp_Velocidade_Motor_1*Erro_Velocidade_Atual_Motor_1) - (Kp_Velocidade_Motor_1*Alpha_Velocidade_Motor_1*Erro_Velocidade_Anterior_Motor_1));

  Erro_Velocidade_Anterior_Motor_1 = Erro_Velocidade_Atual_Motor_1;
  Controle_Velocidade_Anterior_Motor_1 = Controle_Velocidade_Atual_Motor_1;

  if(Controle_Velocidade_Atual_Motor_1 >= 3878){
    Controle_Velocidade_Atual_Motor_1 = 3878;
  }else if(Controle_Velocidade_Atual_Motor_1 <=3103){
    Controle_Velocidade_Atual_Motor_1 = 3103;
  }

  long Auxiliar_Convert_PWM = map(Controle_Velocidade_Atual_Motor_1, 0, 4095, 0, 255);
  dacWrite(Pino_Motor_1, (int) Auxiliar_Convert_PWM);

  }else if (Index == 2){

      RPS_Motor_2 = 10;
      RPS_Motor_2 *= Reducao_Encoder_Motor*Contador_Pulsos_Canal_A_Motor_2;
      RPS_Motor_2 /= Reducao_Motor_Roda*Pulsos_Por_Revolucao_Encoder;

      Contador_Pulsos_Canal_A_Motor_2 = 0;
      Contador_Pulsos_Canal_B_Motor_2 = 0;

      Erro_Velocidade_Atual_Motor_2 = Referencia_Velocidade_Motor_2 - RPS_Motor_2;
      
      Controle_Velocidade_Atual_Motor_2 = (Controle_Velocidade_Anterior_Motor_2 + (Kp_Velocidade_Motor_2*Erro_Velocidade_Atual_Motor_2) - (Kp_Velocidade_Motor_2*Alpha_Velocidade_Motor_2*Erro_Velocidade_Anterior_Motor_2));

      Erro_Velocidade_Anterior_Motor_2 = Erro_Velocidade_Atual_Motor_2;
      Controle_Velocidade_Anterior_Motor_2 = Controle_Velocidade_Atual_Motor_2;

      if(Controle_Velocidade_Atual_Motor_2 >= 3878){
        Controle_Velocidade_Atual_Motor_2 = 3878;
      }else if(Controle_Velocidade_Atual_Motor_2 <=3103){
        Controle_Velocidade_Atual_Motor_2 = 3103;
      }

      long Auxiliar_Convert_PWM = map(Controle_Velocidade_Atual_Motor_2, 0, 4095, 0, 255);
      dacWrite(Pino_Motor_2, (int) Auxiliar_Convert_PWM);
    }
}

// ANGULO
void Controle_Angulo(int Index){
  if (Index == 1){
    float Angulo_Atual = 0;

    Erro_Angulo_Atual_Motor_1 = Referencia_Angulo_Motor_1 -Angulo_Atual;
    
    Referencia_Velocidade_Motor_1 = (Referencia_Anterior_Motor_1 + (Kp_Angulo_Motor_1*Erro_Angulo_Atual_Motor_1) - (Kp_Angulo_Motor_1*Alpha_Angulo_Motor_1*Erro_Angulo_Anterior_Motor_1));

    Erro_Angulo_Anterior_Motor_1 = Erro_Angulo_Atual_Motor_1;
    Referencia_Anterior_Motor_1 = Referencia_Velocidade_Motor_1;

    Controle_Velocidade(Index);

  }else if (Index == 2){

      float Angulo_Atual = 0;

      Erro_Angulo_Atual_Motor_2 = Referencia_Angulo_Motor_2 -Angulo_Atual;
      
      Referencia_Velocidade_Motor_2 = (Referencia_Anterior_Motor_2 + (Kp_Angulo_Motor_1*Erro_Angulo_Atual_Motor_2) - (Kp_Angulo_Motor_2*Alpha_Angulo_Motor_2*Erro_Angulo_Anterior_Motor_2));

      Erro_Angulo_Anterior_Motor_2 = Erro_Angulo_Atual_Motor_2;
      Referencia_Anterior_Motor_2 = Referencia_Velocidade_Motor_2;

    Controle_Velocidade(Index);
    }
}

#if TESTE == true
// TIMER PARA TESTE
void IRAM_ATTR onTimer(void* arg) {
  Controle_Angulo(1); // Controle do Motor 1
  Controle_Angulo(2); // Controle do Motor 2
}
#endif