///////////////////////// BIBLIOTECAS /////////////////////////
#include <Arduino.h>
#include <Wire.h>

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

}

/////////////////////////// LOOP ////////////////////////////
void loop() {
  Serial.println(Status_Remoto ? "Motor Ligado!" : "Motor Desligado");
  delay(1000);
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