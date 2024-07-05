///////////////////////// BIBLIOTECAS /////////////////////////
#include <Arduino.h>
#include <Wire.h>

/////////////////////////// DEFINES ///////////////////////////
// LÓGICA DE ACIONAMENTO
#define Pino_Botao_Emergencia 15
#define Pino_Chave_ON_OFF 16
#define Pino_Rele_Seguranca 27

////////////////////////// VARIAVEIS //////////////////////////
// LÓGICA DE ACIONAMENTO
bool Botao_Emergencia_Pressionado = false; // Verrifica botão de emergência precionado
bool Chave_ON_OFF_Anterior = false; // Salva status anterior da chave liga e desliga
bool Robo_Ok = false; // Variável de acionamento do robô

//////////////////// DECLARAÇÃO DE FUNÇÕES ///////////////////
// LÓGICA DE ACIONAMENTO
void StatusAcionamento();

//////////////////////////// SETUP ///////////////////////////
void setup() {
  Serial.begin(115200);

  // LÓGICA DE ACIONAMENTO
  pinMode(Pino_Botao_Emergencia, INPUT_PULLDOWN); // Define pino do botão de mergência como entrada
  pinMode(Pino_Chave_ON_OFF, INPUT_PULLDOWN); // Define pino da chave liga e desliga com entrada
  pinMode(Pino_Rele_Seguranca, OUTPUT); // Define pino do relé de segurança como saída

  digitalWrite(Pino_Rele_Seguranca, HIGH); // Garante que o relé estará desacionado

  // INTERRUPÇÕES
  // LÓGICA DE ACIOANEMNTO
  attachInterrupt(digitalPinToInterrupt(Pino_Botao_Emergencia), StatusAcionamento, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pino_Chave_ON_OFF), StatusAcionamento, CHANGE);

}

/////////////////////////// LOOP ////////////////////////////
void loop() {
}

///////////////////////// FUNÇÕES ///////////////////////////
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