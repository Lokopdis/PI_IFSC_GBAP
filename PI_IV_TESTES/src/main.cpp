///////////////////////// BIBLIOTECAS /////////////////////////
#include <Arduino.h>

///////////////////////// DIRETRIZES //////////////////////////
#define ACIONAMENTO true
#define SETUPINICIAL true

/////////////////////////// DEFINES ///////////////////////////
#if ACIONAMENTO == true // Pinos para Acionamento
#define Pino_Botao_Emergencia 15
#define Pino_Chave_ON_OFF 16
#define Pino_Rele_Seguranca 27
#endif

////////////////////////// VARIAVEIS //////////////////////////
#if ACIONAMENTO == true // Variáveis para Acionamento
bool Botao_Emergencia_Pressionado = false; 
bool Chave_ON_OFF_Anterior = false; 
bool Robo_Ok = false;
#endif

//////////////////// DECLARAÇÃO DE FUNÇÕES ///////////////////
#if ACIONAMENTO == true // Funções para o Acionamento
void Acionamento();
#if SETUPINICIAL == true
void SetupInicial();     
#endif
#endif

//////////////////////////// SETUP ///////////////////////////
void setup(){
  Serial.begin(115200);

  #if ACIONAMENTO == true
  attachInterrupt(digitalPinToInterrupt(Pino_Botao_Emergencia),Acionamento, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pino_Chave_ON_OFF), Acionamento, CHANGE);

  #endif

}

/////////////////////////// LOOP ////////////////////////////
void loop() {
  Serial.println(Robo_Ok ? "Motor Ligado!" : "Motor Desligado!");  
  delay(100);
}


///////////////////////// FUNÇÕES ///////////////////////////
#if ACIONAMENTO == true // Função de Acionamento
void Acionamento (){

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
    Serial.println(Botao_Emergencia_Pressionado);
    if (!Botao_Emergencia_Pressionado) { // Verifica se o botão de emergência não está pressionado
      Robo_Ok = !Robo_Ok; // Altera o Estado de acionamento do robô LOW --> HIGH
      digitalWrite(Pino_Rele_Seguranca, LOW); // Aciona o Relé
      #if SETUPINICIAL == true
      SetupInicial();
      #endif
    }
  }
  if (Chave_ON_OFF_Atual == LOW) {
      Robo_Ok = LOW;
      digitalWrite(Pino_Rele_Seguranca, HIGH); // Desaciona o Relé
  }

  Chave_ON_OFF_Anterior = Chave_ON_OFF_Atual;

}
#endif

#if SETUPINICIAL == true
void SetupInicial(){
  dacWrite(25, 240);
  dacWrite(26, 198);
}
#endif