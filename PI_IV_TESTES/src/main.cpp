///////////////////////// BIBLIOTECAS /////////////////////////
#include <Arduino.h>
#include <Wire.h>
#include "esp_timer.h"

///////////////////////// DIRETRIZES //////////////////////////
#define ACIONAMENTO true
#define SETUPINICIAL true
#define BNO055 true
#define MOTOR_1 true
#define MOTOR_2 true
#define ENCODER true
#define CONTROLE true
#define TIMER true
#define TESTE false

/////////////////////////// DEFINES ///////////////////////////
#if ACIONAMENTO == true // Pinos para Acionamento
#define Pino_Botao_Emergencia 15
#define Pino_Chave_ON_OFF 16
#define Pino_Rele_Seguranca 27
#endif

#if BNO055 == true // Registradores para Sensor BNO055
#define BNO055_ADDRESS 0x28
#define OPR_MODE_ADDR 0x3D
#define PWR_MODE_ADDR 0x3E
#define SYS_TRIGGER_ADDR 0x3F
#define AXIS_MAP_CONFIG_ADDR 0x41
#define AXIS_MAP_SIGN_ADDR 0x42

// Endereços para os offsets de calibração
#define ACC_OFFSET_X_LSB_ADDR 0x55
#define MAG_OFFSET_X_LSB_ADDR 0x5B
#define GYR_OFFSET_X_LSB_ADDR 0x61

// Registros de dados Euler
#define EUL_HEADING_LSB_ADDR 0x1A
#define EUL_ROLL_LSB_ADDR 0x1C
#define EUL_PITCH_LSB_ADDR 0x1E
#endif

#if MOTOR_1 == true // Pinos para o motor 1
#define Pino_Motor_1 25
#if ENCODER == true // Pinos para o encoder do motor 1
#define Encoder_Canal_A_Motor_1 5 
#define Encoder_Canal_B_Motor_1 17
#endif
#endif

#if MOTOR_2 == true
#define Pino_Motor_2 26
#if ENCODER == true // Pinos para o encoder do motor 1
#define Encoder_Canal_A_Motor_2 4 
#define Encoder_Canal_B_Motor_2 2
#endif
#endif

////////////////////////// VARIAVEIS //////////////////////////
#if ACIONAMENTO == true // Variáveis para Acionamento
volatile bool Botao_Emergencia_Pressionado = false; 
volatile bool Chave_ON_OFF_Anterior = false; 
volatile bool Robo_Ok = false;
#endif

#if BNO055 == true // Variavel para o sensor BNO055
// OFFSETS
int16_t acc_offset_x = 20; 
int16_t acc_offset_y = 5; 
int16_t acc_offset_z = 3; 
int16_t mag_offset_x = 43; 
int16_t mag_offset_y = 246; 
int16_t mag_offset_z = -1; 
int16_t gyr_offset_x = -2; 
int16_t gyr_offset_y = -3; 
int16_t gyr_offset_z = -2; 

// Leitura do sensor
volatile float Angulo_Atual = 0;
volatile float Referencia_Angulo = 0;
volatile float Erro_Angulo_Atual = 0;

volatile int i = 1;
#endif

#if MOTOR_1 == true // Variaveis para o motor 1
volatile int PWM_Motor_1 = 3103;
#if ENCODER == true // Variveis par o encoder do motor 1
// Variáveis de redução
const double Pulsos_Por_Revolucao_Encoder = 1000;
const double Reducao_Encoder_Motor = 3;
const double Reducao_Motor_Roda = 32;

// Variáveis para contar pulsos do encoder
volatile long Contador_Pulsos_Canal_A_Motor_1 = 0;
volatile long Contador_Pulsos_Canal_B_Motor_1 = 0;
#endif
#if CONTROLE == true
volatile double RPS_Motor_1 = 0; // Valor em m/s do motor 1
volatile long Aux_Convert_PWM_Motor_1 = 0; // Variável auxiliar para conversão do valor do PWM de 10 p/ 8 bits

// CONTROLE
// VELOCIDADE
volatile float Controle_Velocidade_Anterior_Motor_1 = 3103;
volatile float Controle_Velocidade_Atual_Motor_1 = 3103;
volatile float Kp_Velocidade_Motor_1 = 283.19;
volatile float Alpha_Velocidade_Motor_1 = 0.05924;
volatile float Erro_Velocidade_Atual_Motor_1 = 0;
volatile float Erro_Velocidade_Anterior_Motor_1 = 0;

// Referencia
volatile float Referencia_Velocidade_Motor_1 = 0;
volatile float referencia_atual_motor1 = 0.48;
volatile float Referencia_Velocidade_Desejada_Motor_1 = 1;

#endif
#endif

#if MOTOR_2 == true
volatile int PWM_Motor_2 = 3103;
#if ENCODER == true // Variveis par o encoder do motor 1
// Variáveis para contar pulsos do encoder
volatile long Contador_Pulsos_Canal_A_Motor_2 = 0;
volatile long Contador_Pulsos_Canal_B_Motor_2 = 0;
#endif
#if CONTROLE == true
volatile double RPS_Motor_2 = 0; // Valor em m/s do motor 1
volatile long Aux_Convert_PWM_Motor_2 = 0; // Variável auxiliar para conversão do valor do PWM de 10 p/ 8 bits

// CONTROLE
// VELOCIDADE
volatile float Controle_Velocidade_Anterior_Motor_2 = 3103;
volatile float Controle_Velocidade_Atual_Motor_2 = 3103;
volatile float Kp_Velocidade_Motor_2 = 283.19;
volatile float Alpha_Velocidade_Motor_2 = 0.05924;
volatile float Erro_Velocidade_Atual_Motor_2 = 0;
volatile float Erro_Velocidade_Anterior_Motor_2 = 0;

// Referencia
volatile float Referencia_Velocidade_Motor_2 = 0;
volatile float referencia_atual_motor2 = 0.48;
volatile float Referencia_Velocidade_Desejada_Motor_2 = 1;
#endif
#endif

#if TIMER == true
esp_timer_handle_t Timer_Controle;
#endif

//////////////////// DECLARAÇÃO DE FUNÇÕES ///////////////////
#if ACIONAMENTO == true // Funções para o Acionamento
void Acionamento();
#endif

#if SETUPINICIAL == true // Funções para carregar valor inicial das variaveis
void SetupInicial();     
#endif

#if BNO055 == true // Funções para leitura da sensor BNO055
void writeRegister(byte addr, byte reg, byte value);
void setMode(byte mode);
int16_t readEulerData(byte reg);
void writeOffset(byte reg, int16_t value);
void Angulo();
void Gerar_Referencia_Angulo();
#endif

#if MOTOR_1 == true
#if ENCODER == true
void Incrementar_Pulsos_Canal_A_Motor_1();
void Incrementar_Pulsos_Canal_B_Motor_1();
#endif
#if CONTROLE == true
void IRAM_ATTR Controle_Velocidade(void *arg);
#endif
#endif

#if MOTOR_2 == true
#if ENCODER == true
void Incrementar_Pulsos_Canal_A_Motor_2();
void Incrementar_Pulsos_Canal_B_Motor_2();
#endif
#endif

//////////////////////////// SETUP ///////////////////////////
void setup(){
  Serial.begin(115200);

  #if BNO055 == true // Configuração incial para leitura do sensor BNO055
  //CONFIGURAÇÕES BNO055
  Wire.begin(); //Initialize I2C communication

  setMode(0x00);  // Coloca o sensor em modo de configuração
  // Grava os offsets
  writeOffset(ACC_OFFSET_X_LSB_ADDR, acc_offset_x);
  writeOffset(ACC_OFFSET_X_LSB_ADDR + 2, acc_offset_y);
  writeOffset(ACC_OFFSET_X_LSB_ADDR + 4, acc_offset_z);
  writeOffset(MAG_OFFSET_X_LSB_ADDR, mag_offset_x);
  writeOffset(MAG_OFFSET_X_LSB_ADDR + 2, mag_offset_y);
  writeOffset(MAG_OFFSET_X_LSB_ADDR + 4, mag_offset_z);
  writeOffset(GYR_OFFSET_X_LSB_ADDR, gyr_offset_x);
  writeOffset(GYR_OFFSET_X_LSB_ADDR + 2, gyr_offset_y);
  writeOffset(GYR_OFFSET_X_LSB_ADDR + 4, gyr_offset_z);
  
  setMode(0x08);  // Muda para o modo M4G
  #endif

  #if ACIONAMENTO == true // Configurações inicial para o acionamento
  // Confiduração dos modos para os pinos de acionamento
  pinMode(Pino_Botao_Emergencia, INPUT_PULLDOWN); // Define pino do botão de mergência como entrada
  pinMode(Pino_Chave_ON_OFF, INPUT_PULLDOWN); // Define pino da chave liga e desliga com entrada
  
  pinMode(Pino_Rele_Seguranca, OUTPUT); // Define pino do relé de segurança como saída
  digitalWrite(Pino_Rele_Seguranca, HIGH); // Força o Relé para desligaddo

  // Configuração dos pinos de acionamento para atuarem com interrução
  attachInterrupt(digitalPinToInterrupt(Pino_Botao_Emergencia),Acionamento, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pino_Chave_ON_OFF), Acionamento, CHANGE);
  #endif

  #if MOTOR_1 == true // Configuração do motor 1
  // Configurando pino do do motor 1 como saída
  pinMode(Pino_Motor_1, OUTPUT);
  #if ENCODER == true
  attachInterrupt(digitalPinToInterrupt(Encoder_Canal_A_Motor_1), Incrementar_Pulsos_Canal_A_Motor_1, FALLING);
  attachInterrupt(digitalPinToInterrupt(Encoder_Canal_B_Motor_1), Incrementar_Pulsos_Canal_B_Motor_1, FALLING);
  #endif
  #endif

  #if MOTOR_2 == true // Configuração do motor 1
  // Configurando pino do do motor 1 como saída
  pinMode(Pino_Motor_2, OUTPUT);
  #if ENCODER == true
  attachInterrupt(digitalPinToInterrupt(Encoder_Canal_A_Motor_2), Incrementar_Pulsos_Canal_A_Motor_2, FALLING);
  attachInterrupt(digitalPinToInterrupt(Encoder_Canal_B_Motor_2), Incrementar_Pulsos_Canal_B_Motor_2, FALLING);
  #endif
  #endif

  #if TIMER == true
  const esp_timer_create_args_t timer_args = {
    .callback = &Controle_Velocidade,  // Função de callback
    .name = "Timer_Controle"      // Nome do timer (para fins de depuração)
  };

  // Criar o timer
  esp_timer_create(&timer_args, &Timer_Controle);

  // Iniciar o timer com intervalo de 100 ms (100000 microssegundos)
  esp_timer_start_periodic(Timer_Controle, 100000);

  #endif

}

/////////////////////////// LOOP ////////////////////////////
void loop() {

  #if TESTE == true
  if (Robo_Ok){
    if(i==1){
      Gerar_Referencia_Angulo();
      i++;
    }

  }
  #endif
}


///////////////////////// FUNÇÕES ///////////////////////////
#if ACIONAMENTO == true // Função de Acionamento
void Acionamento (){
  Wire.end();
  bool Emergencia_Atual = digitalRead(Pino_Botao_Emergencia); // Lê status atual do botão de emergência

   if (Emergencia_Atual == LOW && Botao_Emergencia_Pressionado == LOW) {
    Robo_Ok = LOW; 
    Botao_Emergencia_Pressionado = LOW;
    digitalWrite(Pino_Rele_Seguranca, HIGH); // Desaciona o Relé
  }else if (Emergencia_Atual == LOW && Botao_Emergencia_Pressionado == HIGH) {
    Botao_Emergencia_Pressionado = LOW;
    digitalWrite(Pino_Rele_Seguranca, HIGH); // Desaciona o Relé
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
  Wire.begin();
}
#endif

#if SETUPINICIAL == true // Função para carregar as configurações inicial das variaveis
void SetupInicial(){
  #if CONTROLE == true
  volatile double RPS_Motor_1 = 0; // Valor em m/s do motor 1
  volatile long Aux_Convert_PWM_Motor_1 = 0; // Variável auxiliar para conversão do valor do PWM de 10 p/ 8 bits

  // CONTROLE
  // VELOCIDADE
  volatile float Controle_Velocidade_Anterior_Motor_1 = 3103;
  volatile float Controle_Velocidade_Atual_Motor_1 = 3103;
  volatile float Kp_Velocidade_Motor_1 = 283.19;
  volatile float Alpha_Velocidade_Motor_1 = 0.05924;
  volatile float Erro_Velocidade_Atual_Motor_1 = 0;
  volatile float Erro_Velocidade_Anterior_Motor_1 = 0;
  
  // Referencia
  volatile float Referencia_Velocidade_Motor_1 = 0;
  volatile float referencia_atual_motor1 = 0.48;
  volatile float Referencia_Velocidade_Desejada_Motor_1 = 1;
  #endif

  #if CONTROLE == true
  volatile double RPS_Motor_2 = 0; // Valor em m/s do motor 1
  volatile long Aux_Convert_PWM_Motor_2 = 0; // Variável auxiliar para conversão do valor do PWM de 10 p/ 8 bits

  // CONTROLE
  // VELOCIDADE
  volatile float Controle_Velocidade_Anterior_Motor_2 = 3103;
  volatile float Controle_Velocidade_Atual_Motor_2 = 3103;
  volatile float Kp_Velocidade_Motor_2 = 283.19;
  volatile float Alpha_Velocidade_Motor_2 = 0.05924;
  volatile float Erro_Velocidade_Atual_Motor_2 = 0;
  volatile float Erro_Velocidade_Anterior_Motor_2 = 0;
  
  // Referencia
  volatile float Referencia_Velocidade_Motor_2 = 0;
  volatile float referencia_atual_motor2 = 0.48;
  volatile float Referencia_Velocidade_Desejada_Motor_2 = 1;
  #endif
  Serial.println("Variveis Zeradas!");
}
#endif

#if BNO055 == true // Funções par leitura do sensor BNO055
void writeRegister(byte addr, byte reg, byte value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void setMode(byte mode) {
  writeRegister(BNO055_ADDRESS, OPR_MODE_ADDR, mode);
  delay(25);  // Tempo para estabilizar após mudança de modo
}

int16_t readEulerData(byte reg) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BNO055_ADDRESS, (byte)2);
  byte lsb = Wire.read();
  byte msb = Wire.read();
  return (int16_t)((msb << 8) | lsb);
}

void writeOffset(byte reg, int16_t value) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  Wire.write((byte)(value & 0xFF));
  Wire.write((byte)((value >> 8) & 0xFF));
  Wire.endTransmission();
}

void Angulo(){
  Angulo_Atual = readEulerData(EUL_HEADING_LSB_ADDR);
  Angulo_Atual /= 16.0000;
  //Serial.print("Angulo: ");
  
  Erro_Angulo_Atual = Angulo_Atual - Referencia_Angulo;

  // Ajusta o ângulo se ele passar de 360 graus
  if (Erro_Angulo_Atual > 180) {
      // Adiciona a diferença para a soma angular
      Erro_Angulo_Atual -= 360;
  } 
  else if(Erro_Angulo_Atual < -180){
      Erro_Angulo_Atual += 360;
  }
  Serial.print(Erro_Angulo_Atual);
}

void Gerar_Referencia_Angulo(){
  delay(1000);
  Angulo();
  Referencia_Angulo = Angulo_Atual;
  delay(100);
}
#endif

#if MOTOR_1 == true
#if ENCODER == true
void Incrementar_Pulsos_Canal_A_Motor_1(){
  Contador_Pulsos_Canal_A_Motor_1++;
}

void Incrementar_Pulsos_Canal_B_Motor_1(){
  Contador_Pulsos_Canal_B_Motor_1++;
}
#endif
#if CONTROLE == true
void IRAM_ATTR Controle_Velocidade(void *arg){

if(Robo_Ok){
  if(i==1){
      Gerar_Referencia_Angulo();
      i++;
    }
  Angulo();
  // Motor 1
  RPS_Motor_1 = 10;
  RPS_Motor_1 *= Reducao_Encoder_Motor*Contador_Pulsos_Canal_A_Motor_1;
  RPS_Motor_1 /= Reducao_Motor_Roda*Pulsos_Por_Revolucao_Encoder;

  Contador_Pulsos_Canal_A_Motor_1 = 0;
  Contador_Pulsos_Canal_B_Motor_1 = 0;

  if(referencia_atual_motor1 != Referencia_Velocidade_Desejada_Motor_1){
    referencia_atual_motor1 = referencia_atual_motor1 + ((Referencia_Velocidade_Desejada_Motor_1 - referencia_atual_motor1)/100);
    Referencia_Velocidade_Motor_1 = referencia_atual_motor1;
  }

  Erro_Velocidade_Atual_Motor_1 = Referencia_Velocidade_Motor_1 - RPS_Motor_1;
  
  Controle_Velocidade_Atual_Motor_1 = (Controle_Velocidade_Anterior_Motor_1 + (Kp_Velocidade_Motor_1*Erro_Velocidade_Atual_Motor_1) - (Kp_Velocidade_Motor_1*Alpha_Velocidade_Motor_1*Erro_Velocidade_Anterior_Motor_1));

  Erro_Velocidade_Anterior_Motor_1 = Erro_Velocidade_Atual_Motor_1;
  Controle_Velocidade_Anterior_Motor_1 = Controle_Velocidade_Atual_Motor_1;

  if(Controle_Velocidade_Atual_Motor_1 >= 3878){
    Controle_Velocidade_Atual_Motor_1 = 3878;
  }else if(Controle_Velocidade_Atual_Motor_1 <=3103){
    Controle_Velocidade_Atual_Motor_1 = 3103;
  }

  long Auxiliar_Convert_PWM_1 = map(Controle_Velocidade_Atual_Motor_1, 0, 4095, 0, 255);
  dacWrite(Pino_Motor_1, (int) Auxiliar_Convert_PWM_1);

  volatile int64_t time_us = esp_timer_get_time(); // Tempo em microsegundos
  Serial.print("  | ");
  Serial.print(time_us / 1000); // Converte para milissegundos
  //Serial.print(" ms");
  Serial.print("  | ");
  //Serial.print("Referencia:");
  Serial.print(Referencia_Velocidade_Motor_1);
  Serial.print("  | ");
  //Serial.print("Motor 1: ");
  Serial.print(RPS_Motor_1);

  // MOTOR 2
  RPS_Motor_2 = 10;
  RPS_Motor_2 *= Reducao_Encoder_Motor*Contador_Pulsos_Canal_A_Motor_2;
  RPS_Motor_2 /= Reducao_Motor_Roda*Pulsos_Por_Revolucao_Encoder;

  Contador_Pulsos_Canal_A_Motor_2 = 0;
  Contador_Pulsos_Canal_B_Motor_2 = 0;

  if(referencia_atual_motor2 != Referencia_Velocidade_Desejada_Motor_2){
    referencia_atual_motor2 = referencia_atual_motor2 + ((Referencia_Velocidade_Desejada_Motor_2 - referencia_atual_motor2)/100);
    Referencia_Velocidade_Motor_2 = referencia_atual_motor2;
  }

  Erro_Velocidade_Atual_Motor_2 = Referencia_Velocidade_Motor_2 - RPS_Motor_2;
  
  Controle_Velocidade_Atual_Motor_2 = (Controle_Velocidade_Anterior_Motor_2 + (Kp_Velocidade_Motor_2*Erro_Velocidade_Atual_Motor_2) - (Kp_Velocidade_Motor_2*Alpha_Velocidade_Motor_2*Erro_Velocidade_Anterior_Motor_2));

  Erro_Velocidade_Anterior_Motor_2 = Erro_Velocidade_Atual_Motor_2;
  Controle_Velocidade_Anterior_Motor_2 = Controle_Velocidade_Atual_Motor_2;

  if(Controle_Velocidade_Atual_Motor_2 >= 3878){
    Controle_Velocidade_Atual_Motor_2 = 3878;
  }else if(Controle_Velocidade_Atual_Motor_2 <=3103){
    Controle_Velocidade_Atual_Motor_2 = 3103;
  }

  long Auxiliar_Convert_PWM_2 = map(Controle_Velocidade_Atual_Motor_2, 0, 4095, 0, 255);
  dacWrite(Pino_Motor_2, (int) Auxiliar_Convert_PWM_2);

  Serial.print("  | ");
 //Serial.print("Motor 2: ");
  Serial.println(RPS_Motor_2);
}else{
  SetupInicial();
}
}
#endif
#endif

#if MOTOR_2 == true
#if ENCODER == true
void Incrementar_Pulsos_Canal_A_Motor_2(){
  Contador_Pulsos_Canal_A_Motor_2++;
}

void Incrementar_Pulsos_Canal_B_Motor_2(){
  Contador_Pulsos_Canal_B_Motor_2++;
}
#endif
#endif