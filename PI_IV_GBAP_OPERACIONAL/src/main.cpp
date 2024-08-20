///////////////////////// BIBLIOTECAS /////////////////////////
#include <Arduino.h>
#include <Wire.h>
#include "esp_timer.h"
#include <BluetoothSerial.h>

///////////////////////// DIRETRIZES //////////////////////////
#define ACIONAMENTO true
#define MOTOR_1 true
#define MOTOR_2 true
#define BNO055 true
#define TIMER true
#define TESTE false
#define PRINT false
#define PI false
#define PD false
#define PID true
#define BLUETOOTH true

////////////////////////// VARIAVEIS //////////////////////////

#if BLUETOOTH == true
// Cria uma instância do BluetoothSerial
BluetoothSerial SerialBT;
#endif

// <--- ACIONAMENTO --->
#if ACIONAMENTO == true
// Botão de Emergência
#define Pino_Bt_Emergencia 15 

volatile bool Bt_Emergencia_Pressionado = false; 

// Chave Liga/Desliga
#define Pino_Chave_Liga_Desliga 16 

volatile bool Chave_Liga_Desliga_Pressionado = false; 
volatile bool Chave_Liga_Desliga_Anterior = false; 

// Acionamento Remoto
#define Pino_Acionamento_Remoto 14
volatile bool Bt_Acionamento_Remoto = false;

// Flag de Acionamento do Robô
volatile bool robo_ok = false;

// Saída de Acioanmento do Relé de Segurança
#define Pino_Rele_Seguranca 27 //Porta D27 para habilitar o relé de estado sólido, para acionar os motores um tempo após ligar o ESP
#endif

// <--- MOTOR 1 --->
#if MOTOR_1 == true // Início do bloco Motor 1
// Saída de Acioanmento do Motor 1
#define Pino_Motor_1 25

// Valor inicial do motor 1 (Velocidade em 0 m/s)
volatile int Valor_PWM_Motor_1 = 3103;

// <--- ENCODERS --->
#define Canal_A_Encoder_Motor_1 5 
#define Canal_B_Encoder_Motor_1 17

volatile long Contador_Pulsos_Encoder_Canala_A_Motor_1 = 0;
volatile long Contador_Pulsos_Encoder_Canala_B_Motor_1 = 0;

// <--- CONTROLE VELOCIDADE --->
volatile double RPS_Motor_1 = 0;

// Controlador PI
volatile float Kp_Motor_1 = 300.19;
volatile float Alpha_Motor_1 = 0.05924;
volatile float Controle_Atual_Motor_1 = 3103;
volatile float Controle_Anterior_Motor_1 = 3103;
volatile float Erro_Atual_Motor_1 = 0;
volatile float Erro_Anterior_Motor_1 = 0;
volatile float Referencia_Motor_1 = 1;
volatile float Referencia_Atual_Motor_1 = 0.48;
volatile float Referencia_Desejada_Motor_1 = 1;

// <--- CONTROLE ANGULO --->
volatile float Referencia_Correcao_Angulo_Motor_1 = 0;

#endif // Fim do bloco Motor 1

// <--- MOTOR 2 --->
#if MOTOR_2 == true // Início do bloco Motor 2
// Saída de Acioanmento do Motor 2
#define Pino_Motor_2 26

// Valor inicial do motor 2 (Velocidade em 0 m/s)
volatile int Valor_PWM_Motor_2 = 3103;

// <--- ENCODERS --->
#define Canal_A_Encoder_Motor_2 4 
#define Canal_B_Encoder_Motor_2 2

volatile long Contador_Pulsos_Encoder_Canala_A_Motor_2 = 0;
volatile long Contador_Pulsos_Encoder_Canala_B_Motor_2 = 0;

// <--- CONTROLE --->
double RPS_Motor_2 = 0;

// Controlador PI
float Kp_Motor_2 = 300.19;   //283.19
float Alpha_Motor_2 = 0.05924;
float Controle_Atual_Motor_2 = 3103;
float Controle_Anterior_Motor_2 = 3103;
float Erro_Atual_Motor_2 = 0;
float Erro_Anterior_Motor_2 = 0;
float Referencia_Motor_2 = 1;
float Referencia_Atual_Motor_2 = 0.48;
float Referencia_Desejada_Motor_2 = 1;

// <--- CONTROLE ANGULO --->
float Referencia_Correcao_Angulo_Motor_2 = 0;

#endif // Fim do bloco do Motor 2

// <--- ENCODER --->
volatile const double ENCODER_PULSOS_VOLTA = 1000; //Número de pulsos por volta
volatile const double REDUCAO_ENCODER_MOTOR = 3; //Redução entre o encoder e o eixo do motor
volatile const double REDUCAO_MOTOR_RODA = 32; //Valor da redução entre o motor e a roda

// <--- BNO055 --->
#if BNO055 == true

// Valores de offsets
int16_t acc_offset_x = 20; 
int16_t acc_offset_y = 5; 
int16_t acc_offset_z = 3; 
int16_t mag_offset_x = 43; 
int16_t mag_offset_y = 246; 
int16_t mag_offset_z = -1; 
int16_t gyr_offset_x = -2; 
int16_t gyr_offset_y = -3; 
int16_t gyr_offset_z = -2; 

// Registradores
#define BNO055_ADDRESS 0x28
#define OPR_MODE_ADDR 0x3D
#define PWR_MODE_ADDR 0x3E
#define SYS_TRIGGER_ADDR 0x3F
#define AXIS_MAP_CONFIG_ADDR 0x41
#define AXIS_MAP_SIGN_ADDR 0x42
#define ACC_OFFSET_X_LSB_ADDR 0x55
#define MAG_OFFSET_X_LSB_ADDR 0x5B
#define GYR_OFFSET_X_LSB_ADDR 0x61

// Registros de dados Euler
#define EUL_HEADING_LSB_ADDR 0x1A
#define EUL_ROLL_LSB_ADDR 0x1C
#define EUL_PITCH_LSB_ADDR 0x1E
#endif

// <--- CONTROLE ANGULO --->
volatile float Referencia_Angulo = 0;
volatile float Angulo_Atual = 0;
volatile float Delta_Angulo = 0;
volatile float Delta_Anterior = 0;
volatile float Delta_Anterior_2 = 0; 

volatile float constante_dinamica_prefiltro = 100;
volatile float coeficiente_angulo = 0.05284;

#if PID == true
volatile float kp = 42.0251;
volatile float ki = 71.4388;
volatile float kd = 4.8416;

///////////////////////////
volatile float Kd_Discreto = (kd*2)/0.1;
volatile float Ki_Discreto = (ki*2)/0.1;

volatile float P = 0;
volatile float I = 0;
volatile float D = 0;

volatile float integral = 0;
volatile float Max_Integral = 70;
#endif

//////////////////////
volatile float Controle_Angulo = 0;
volatile float Controle_Angulo_Anterior = 0;
volatile float Kp = 0.1;
volatile float Alpha = 0.01;
volatile float Delta_Anterior = 0; 
/////////////////////

// <--- OPERAÇÃO --->
#define TEMPO_AMOSTRAGEM 100000 //Valor em us
volatile int i = 1; // Contador para gerar referêcia de ângulo

// <--- TIMER --->
#if TIMER == true
esp_timer_handle_t Timer_Controle;
#endif

//////////////////// DECLARAÇÃO DE FUNÇÕES ///////////////////

// <--- ACIONAMENTO --->
#if ACIONAMENTO == true
void Acionamento();
void Remoto();
void Setup_Inicial();
#endif

// <--- MOTOR 1 --->
#if MOTOR_1 == true
void contador_canalA_motor1();
void contador_canalB_motor1();
#endif

// <--- MOTOR 2 --->
#if MOTOR_2 == true
void contador_canalA_motor2();
void contador_canalB_motor2();
#endif

// <--- BNO055 --->
#if BNO055 == true
void writeRegister(byte addr, byte reg, byte value);
void setMode(byte mode);
int16_t readEulerData(byte reg);
void writeOffset(byte reg, int16_t value);
#endif

// <--- CONTROLE --->
void Controle_Velocidade();

void Gerar_Referencia_Angulo();

#if TIMER == true
void IRAM_ATTR Angulo(void *arg);
#endif

#if TESTE == true
void Angulo();
#endif

// <--- LEITURA RPS --->
double leitura_rotacao(long pulsos);

//////////////////////////// SETUP ///////////////////////////
void setup() {
  Serial.begin(115200);

#if BLUETOOTH == true
// Inicializa a comunicação Bluetooth
SerialBT.begin("ESP32_Bluetooth"); // Nome do dispositivo Bluetooth

// Imprime uma mensagem indicando que o Bluetooth foi iniciado
Serial.println("Bluetooth iniciado. Dispositivo pronto para parear.");
#endif

// <--- BNO055 --->
#if BNO055 == true
  //CONFIGURAÇÕES BNO055
  Wire.begin(); //Initialize I2C communication

  ////////////////////////////Offset_Gyro///////////
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

// <--- ACIONAMENTO --->
#if ACIONAMENTO == true
    // Configura Interrupçao para o botão de acioanmento
  attachInterrupt(digitalPinToInterrupt(Pino_Bt_Emergencia), Acionamento, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pino_Chave_Liga_Desliga), Acionamento, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pino_Acionamento_Remoto), Remoto, CHANGE);

  // Configura entradas e saídas do botões e do relé de segurança
  pinMode(Pino_Chave_Liga_Desliga, INPUT_PULLDOWN);
  pinMode(Pino_Bt_Emergencia, INPUT_PULLDOWN);  
  pinMode(Pino_Rele_Seguranca, OUTPUT);
  pinMode(Pino_Acionamento_Remoto, INPUT_PULLDOWN);
  digitalWrite(Pino_Rele_Seguranca, HIGH);
#endif

// <--- MOTOR 1 --->
#if MOTOR_1 == true
  // Configura pino como saída
  pinMode(Pino_Motor_1, OUTPUT);

  // Configura entradas do Encoder
  pinMode(Canal_A_Encoder_Motor_1, INPUT_PULLUP);
  pinMode(Canal_B_Encoder_Motor_1, INPUT_PULLUP);

  // Configura interrupção do Encoder
  attachInterrupt(digitalPinToInterrupt(Canal_A_Encoder_Motor_1), contador_canalA_motor1, FALLING);
  attachInterrupt(digitalPinToInterrupt(Canal_B_Encoder_Motor_1), contador_canalB_motor1, FALLING);
#endif

// <--- MOTOR 2 --->
#if MOTOR_2 == true
  // Configura pino como saída
  pinMode(Pino_Motor_2, OUTPUT);

  // Configura entradas do Encoder
  pinMode(Canal_A_Encoder_Motor_2, INPUT_PULLUP); 
  pinMode(Canal_B_Encoder_Motor_2, INPUT_PULLUP);

  // Configura interrupção do Encoder
  attachInterrupt(digitalPinToInterrupt(Canal_A_Encoder_Motor_2), contador_canalA_motor2, FALLING);
  attachInterrupt(digitalPinToInterrupt(Canal_B_Encoder_Motor_2), contador_canalB_motor2, FALLING);
#endif

#if TIMER == true
  const esp_timer_create_args_t timer_args = {
    .callback = &Angulo,  // Função de callback
    .name = "Timer_Controle"      // Nome do timer (para fins de depuração)
  };

  // Criar o timer
  esp_timer_create(&timer_args, &Timer_Controle);

  // Iniciar o timer com intervalo de 100 ms (100000 microssegundos)
  esp_timer_start_periodic(Timer_Controle, TEMPO_AMOSTRAGEM);

  #endif
}

/////////////////////////// LOOP ////////////////////////////
void loop(){

#if TESTE == true
  if (robo_ok){
     if(i == 1){
      Gerar_Referencia_Angulo();
      i++; 
     }
      Angulo(); 
      Contador_Pulsos_Encoder_Canala_A_Motor_1 = 0; //Zera a variáveis de pulsos após os calculos
      Contador_Pulsos_Encoder_Canala_B_Motor_2 = 0; //Zera a variáveis de pulsos após os calculos

        Serial.print("Motor 1 : ");
        Serial.print(RPS_Motor_1);
        Serial.print(" | ");


        Serial.print("         Motor 2 : ");
        Serial.print(RPS_Motor_2);
        Serial.print(" | ");


        Serial.print("        Ref1: ");
        Serial.print(Referencia_Motor_1);

        Serial.print("        Ref2: ");
        Serial.print(Referencia_Motor_2);

        Serial.print("        delta: ");
        Serial.print(Delta_Angulo);

        
        Serial.print("        Ángulo ref: ");
        Serial.print(Referencia_Angulo);
        Serial.print(" | Angulo Atual ");
        Serial.println(Angulo_Atual );
        delay(TEMPO_AMOSTRAGEM*1000);
    }
    else{
      Setup_Inicial();
    }

#endif

}

///////////////////////// FUNÇÕES ///////////////////////////

// <--- ACIONAMENTO --->
#if ACIONAMENTO == true
void Acionamento() {

    // Verifica se o botão de emergência foi pressionado
    bool emergenciaAtual = digitalRead(Pino_Bt_Emergencia);
    Bt_Acionamento_Remoto = digitalRead(Pino_Acionamento_Remoto);

    // Se o botão de emergência estiver pressionado, desliga o robô
    if (emergenciaAtual == LOW) {
        robo_ok = LOW; // Desliga o motor
        Bt_Emergencia_Pressionado = LOW;
        digitalWrite(Pino_Rele_Seguranca, HIGH); // Desliga a porta
    }

    // Verifica a borda de subida no botão de liga/desliga
    bool ligaDesligaAtual = digitalRead(Pino_Chave_Liga_Desliga);

    if (ligaDesligaAtual == HIGH && Chave_Liga_Desliga_Anterior == LOW) {
        // Se a chave foi girada e o botão de emergência não está pressionado
        if (emergenciaAtual == HIGH) {
            robo_ok = HIGH; // Liga o robô
            digitalWrite(Pino_Rele_Seguranca, LOW); // Liga a porta
        }
    } 
    else if (ligaDesligaAtual == LOW && Chave_Liga_Desliga_Anterior == HIGH) {
        // Se a chave foi girada para desligar, desliga o robô
        robo_ok = LOW;
        digitalWrite(Pino_Rele_Seguranca, HIGH); // Desliga a porta
    }

    // Atualiza o estado da chave para o próximo ciclo
    Chave_Liga_Desliga_Anterior = ligaDesligaAtual;

    
}

void Remoto(){

  // Verifica se o botão de emergência foi pressionado
    bool emergenciaAtual = digitalRead(Pino_Bt_Emergencia);
    Bt_Acionamento_Remoto = digitalRead(Pino_Acionamento_Remoto);

  // Lógica de acionamento remoto com menor prioridade
    if (Bt_Acionamento_Remoto == HIGH && emergenciaAtual == HIGH && robo_ok == LOW) {
        robo_ok = HIGH; // Liga o robô
        digitalWrite(Pino_Rele_Seguranca, LOW); // Liga a porta
    }
    if (Bt_Acionamento_Remoto == LOW && emergenciaAtual == HIGH && robo_ok == HIGH) {
        robo_ok = LOW; // Liga o robô
        digitalWrite(Pino_Rele_Seguranca, HIGH); // Liga a porta
    }

}

void Setup_Inicial(){

    Controle_Atual_Motor_1 = 3103;
    Controle_Anterior_Motor_1 = 3103;
    Erro_Atual_Motor_1 = 0;
    Erro_Anterior_Motor_1 = 0;
    Referencia_Motor_1 = 1;
    Referencia_Atual_Motor_1 = 0.48;
    Referencia_Desejada_Motor_1 = 1;

    Controle_Atual_Motor_2 = 3103;
    Controle_Anterior_Motor_2 = 3103;
    Erro_Atual_Motor_2 = 0;
    Erro_Anterior_Motor_2 = 0;
    Referencia_Motor_2 = 1;
    Referencia_Atual_Motor_2 = 0.48;
    Referencia_Desejada_Motor_2 = 1;

    Referencia_Angulo = 0;
    Angulo_Atual = 0;
    Delta_Angulo = 0;
    Delta_Anterior = 0;
    Delta_Anterior_2 = 0;
    integral = 0; 

    Referencia_Correcao_Angulo_Motor_1 = 0;
    Referencia_Correcao_Angulo_Motor_2 = 0;

    Controle_Angulo = 0;
    Controle_Angulo_Anterior = 0;
    Kp = 0.1;
    Alpha = 0.01;


    dacWrite(Pino_Motor_1, 199);
    dacWrite(Pino_Motor_2, 198);

    i = 1;
    Serial.println("Variaveis Zeradas");
}

#endif

// <--- MOTOR 1 --->
#if MOTOR_1 == true
void contador_canalA_motor1(){
  Contador_Pulsos_Encoder_Canala_A_Motor_1++;
}

void contador_canalB_motor1(){
  Contador_Pulsos_Encoder_Canala_B_Motor_1++;
}
#endif

// <--- MOTOR 2 --->
#if MOTOR_2 == true
void contador_canalA_motor2(){
  Contador_Pulsos_Encoder_Canala_A_Motor_2++;
}

void contador_canalB_motor2(){
  Contador_Pulsos_Encoder_Canala_B_Motor_2++;
}
#endif

// <--- CONTROLE --->
void Controle_Velocidade(){

    // <--- MOTOR 1 --->
    RPS_Motor_1 = leitura_rotacao(Contador_Pulsos_Encoder_Canala_A_Motor_1);

    if(Referencia_Atual_Motor_1 != Referencia_Correcao_Angulo_Motor_1){
        Referencia_Atual_Motor_1 = Referencia_Atual_Motor_1 + ((Referencia_Correcao_Angulo_Motor_1 - Referencia_Atual_Motor_1) / constante_dinamica_prefiltro);
    }
    Referencia_Motor_1 = Referencia_Atual_Motor_1;

    //Cálculo do erro atual
    Erro_Atual_Motor_1 = (Referencia_Motor_1 - RPS_Motor_1); 

    //Cálculo do Controle
    Controle_Atual_Motor_1 = (Controle_Anterior_Motor_1 + (Kp_Motor_1 * Erro_Atual_Motor_1) - (Kp_Motor_1 * Alpha_Motor_1 * Erro_Anterior_Motor_1)); //Calculo da ação de controle atual
    
    //Atualiza as variáveis 
    Erro_Anterior_Motor_1 = Erro_Atual_Motor_1; 
    Controle_Anterior_Motor_1 = Controle_Atual_Motor_1;

    Valor_PWM_Motor_1 = Controle_Atual_Motor_1;
  
    if(Valor_PWM_Motor_1 >= 3878)
    {
        Valor_PWM_Motor_1 = 3878;
    }

    if(Valor_PWM_Motor_1 <= 3103)
    {
        Valor_PWM_Motor_1 = 3103;
    }
    volatile float Auxiliar_Converte_PWM_1 = map(Valor_PWM_Motor_1, 0, 4095, 0, 255);
    dacWrite(Pino_Motor_1, (int) Auxiliar_Converte_PWM_1); //Envia sinal para a ponte H

    // <--- MOTOR 2 --->
    RPS_Motor_2 = leitura_rotacao(Contador_Pulsos_Encoder_Canala_B_Motor_2);

    if(Referencia_Atual_Motor_2 != Referencia_Correcao_Angulo_Motor_2){
        Referencia_Atual_Motor_2 = Referencia_Atual_Motor_2 + ((Referencia_Correcao_Angulo_Motor_2 - Referencia_Atual_Motor_2) / constante_dinamica_prefiltro);
    }
    Referencia_Motor_2 = Referencia_Atual_Motor_2;

    //Cálculo do erro atual
    Erro_Atual_Motor_2 = (Referencia_Motor_2 - RPS_Motor_2); 

    //Cálculo do Controle
    Controle_Atual_Motor_2 = (Controle_Anterior_Motor_2 + (Kp_Motor_2 * Erro_Atual_Motor_2) - (Kp_Motor_2 * Alpha_Motor_2 * Erro_Anterior_Motor_2)); //Calculo da ação de controle atual
    
    //Atualiza as variáveis 
    Erro_Anterior_Motor_2 = Erro_Atual_Motor_2; 
    Controle_Anterior_Motor_2 = Controle_Atual_Motor_2;

    Valor_PWM_Motor_2 = Controle_Atual_Motor_2;
  
    if(Valor_PWM_Motor_2 >= 3878)
    {
        Valor_PWM_Motor_2 = 3878;
    }

    if(Valor_PWM_Motor_2 <= 3103)
    {
        Valor_PWM_Motor_2 = 3103;
    }
    volatile float Auxiliar_Converte_PWM_2 = map(Valor_PWM_Motor_2, 0, 4095, 0, 255);
    dacWrite(Pino_Motor_2, (int) Auxiliar_Converte_PWM_2); //Envia sinal para a ponte H
}

double leitura_rotacao(long pulsos){
  double rotacao = 10; 
  rotacao *= REDUCAO_ENCODER_MOTOR * pulsos; //Converte para rotação no motor
  rotacao /= (REDUCAO_MOTOR_RODA * ENCODER_PULSOS_VOLTA); //Converte para rotação na roda
  
  return rotacao; //Retorna RPS da roda
}

void Gerar_Referencia_Angulo(){
    Serial.println("Gerando Referencia . . .");
    Referencia_Angulo = readEulerData(EUL_HEADING_LSB_ADDR);
    delay(10);
     Referencia_Angulo = readEulerData(EUL_HEADING_LSB_ADDR);
    Referencia_Angulo /= 16.000;
}

#if TIMER == true
void IRAM_ATTR Angulo(void *arg){
    if(robo_ok){
        if(i == 1){
            Gerar_Referencia_Angulo();
            i++;
        }
        Angulo_Atual = readEulerData(EUL_HEADING_LSB_ADDR);
        Angulo_Atual /= 16.0000;
        Delta_Angulo = Referencia_Angulo -Angulo_Atual;

        // Ajusta o ângulo se ele passar de 360 graus
        if (Delta_Angulo > 180) {
        // Adiciona a diferença para a soma angular
        Delta_Angulo -= 360;
        } 
        else if(Delta_Angulo < -180){
        Delta_Angulo += 360;
        }

        #if PI == true
        ////////////////////////////////////////////////
        Controle_Angulo = (Controle_Angulo_Anterior+(Delta_Angulo*Kp*0.00786)-(Kp*Alpha*Delta_Anterior));
        Controle_Angulo_Anterior = Controle_Angulo;
        Delta_Anterior = Delta_Angulo;

         if(Controle_Angulo >= 0.1){
            Controle_Angulo = 0.2;
         }

         volatile float Dif = abs(Referencia_Angulo-Referencia_Correcao_Angulo_Motor_1);
        if (Dif >= 0.3){
          Referencia_Correcao_Angulo_Motor_1 = Referencia_Desejada_Motor_1 + Controle_Angulo;
          Referencia_Correcao_Angulo_Motor_2 = Referencia_Desejada_Motor_2 - Controle_Angulo;
        }
        #endif

        #if PD == true
        Controle_Angulo = 0.05932*(Delta_Angulo)-0.01883*Delta_Anterior-0.000852*Controle_Angulo_Anterior;
        Controle_Angulo_Anterior = Controle_Angulo;
        Delta_Anterior = Delta_Angulo;
        #endif

        #if PID == true
        P = kp*(Delta_Angulo-Delta_Anterior);
        I = Ki_Discreto*(Delta_Angulo+Delta_Anterior);
        D = Kd_Discreto*(Delta_Angulo-Delta_Anterior+Delta_Anterior_2);

        // Filtro anti-windup
        integral += ki*Delta_Angulo;
        if (integral > Max_Integral){
          I = Max_Integral;
        }

        Controle_Angulo = Controle_Angulo_Anterior + P + I + D;

        Serial.print("Erro: ");
        Serial.print(Delta_Angulo);
        Serial.print("  |   ");
        Serial.print("Integral: ");
        Serial.print(integral);
        Serial.print("|");
        Serial.print(I);
        Serial.print("  | ");
        Serial.print("Controle: ");
        Serial.println(Controle_Angulo);
        #endif

        Delta_Anterior = Delta_Angulo;
        Delta_Anterior_2 = Delta_Anterior;
        Controle_Angulo_Anterior = Controle_Angulo;

        ///////////////////////////////////////////////
        Referencia_Correcao_Angulo_Motor_1 = Referencia_Desejada_Motor_1 - Controle_Angulo;
        Referencia_Correcao_Angulo_Motor_2 = Referencia_Desejada_Motor_2 + Controle_Angulo;

        Controle_Velocidade();
        Contador_Pulsos_Encoder_Canala_A_Motor_1 = 0; //Zera a variáveis de pulsos após os calculos
        Contador_Pulsos_Encoder_Canala_B_Motor_2 = 0; //Zera a variáveis de pulsos após os calculos
    #if PRINT == true
        Serial.print("Ref1: ");
        Serial.print(Referencia_Motor_1);

        Serial.print("        Motor 1 : ");
        Serial.print(RPS_Motor_1);
        Serial.print(" | ");

        Serial.print("        Ref2: ");
        Serial.print(Referencia_Motor_2);
        
        Serial.print("         Motor 2 : ");
        Serial.print(RPS_Motor_2);
        Serial.print(" | ");

        Serial.print("        delta: ");
        Serial.print(Delta_Angulo);

        Serial.print("        Ángulo ref: ");
        Serial.print(Referencia_Angulo);
        Serial.print(" | Angulo Atual ");
        Serial.println(Angulo_Atual );
    #endif
    #if BLUETOOTH == true
        //SerialBT.print("Ref1: ");
        //SerialBT.print(Referencia_Motor_1);

        //SerialBT.print("        Motor 1 : ");
        //SerialBT.print(RPS_Motor_1);
        //SerialBT.print(" | ");

        //SerialBT.print("        Ref2: ");
        //SerialBT.print(Referencia_Motor_2);
        
        //SerialBT.print("         Motor 2 : ");
        //SerialBT.print(RPS_Motor_2);
        //SerialBT.print(" | ");

        //SerialBT.print("        delta: ");
        SerialBT.print(Delta_Angulo);

        //SerialBT.print("        Ángulo ref: ");
        //SerialBT.print(Referencia_Angulo);
        //SerialBT.print(" | Angulo Atual ");
        //SerialBT.println(Angulo_Atual );
    #endif

    }else{
        Setup_Inicial();
    }
}
#endif

// <--- BNO055 --->
#if BNO055 == true
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
#endif

#if TESTE == true
void Angulo(){
    if(robo_ok){
        if(i == 1){
            Gerar_Referencia_Angulo();
            i++;
        }
        Angulo_Atual = readEulerData(EUL_HEADING_LSB_ADDR);
        Angulo_Atual /= 16.0000;
        Delta_Angulo = Referencia_Angulo -Angulo_Atual;

        // Ajusta o ângulo se ele passar de 360 graus
        if (Delta_Angulo > 180) {
        // Adiciona a diferença para a soma angular
        Delta_Angulo -= 360;
        } 
        else if(Delta_Angulo < -180){
        Delta_Angulo += 360;
        }
        Referencia_Correcao_Angulo_Motor_1 = Referencia_Desejada_Motor_1 - (Delta_Angulo * coeficiente_angulo);
        Referencia_Correcao_Angulo_Motor_2 = Referencia_Desejada_Motor_2 + (Delta_Angulo * coeficiente_angulo);

        Controle_Velocidade();
    #if PRINT == true
        Serial.print("Ref1: ");
        Serial.print(Referencia_Motor_1);

        Serial.print("        Motor 1 : ");
        Serial.print(RPS_Motor_1);
        Serial.print(" | ");

        Serial.print("        Ref2: ");
        Serial.print(Referencia_Motor_2);
        
        Serial.print("         Motor 2 : ");
        Serial.print(RPS_Motor_2);
        Serial.print(" | ");

        Serial.print("        delta: ");
        Serial.print(Delta_Angulo);

        Serial.print("        Ángulo ref: ");
        Serial.print(Referencia_Angulo);
        Serial.print(" | Angulo Atual ");
        Serial.println(Angulo_Atual );
    #endif

    }else{
        Setup_Inicial();
    }

}

#endif

