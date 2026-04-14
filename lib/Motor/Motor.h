#pragma once
#include <Arduino.h>

// Estrutura que guarda as configurações do motor
struct Motor
{
  uint8_t pinoPWM;
  uint8_t pinoDir;
  uint8_t canalLEDC; // Necessário no ESP32 para vincular o PWM (0 a 15)

  int pwmAtual;
  int pwmAlvo;
  int passoMaximo; // O máximo de PWM que pode mudar por vez

  uint32_t ultimoTempoMs;
  uint32_t intervaloMs; // A cada quantos milissegundos o incremento acontece
};

// Função que cria e inicializa uma nova struct motor
Motor criarMotor(uint8_t pinoPWM, uint8_t pinoDir, uint8_t canalLEDC);

// Função que define a velocidade e a direção (-1023 a 1023)
void moverMotor(Motor *motor, int velocidade);

// Função para emitir som usando as bobinas do motor (como buzzer) sem rodar
void tocarSomMotor(Motor *motor, uint32_t frequencia, uint32_t duracao_ms);