#pragma once
#include <Arduino.h>

#define FREQ_PWM_ALTA 25000
#define RESOLUCAO_PWM 10

// CORREÇÃO: Usando deslocamento de bits (Shift Left) para calcular 2^10 - 1 = 1023
#define PID_MAX_PWM ((1 << RESOLUCAO_PWM) - 1)
#define PID_MIN_PWM -PID_MAX_PWM

#define PASSO_MAXIMO 50
#define INTERVALO_MS 10

// Estrutura que guarda as configurações do motor
struct Motor
{
  uint8_t pinoPWM;
  uint8_t pinoDir;

  // canalLEDC removido, pois no ESP32 Core 3.x o gerenciamento de canais é automático

  int pwmAtual;
  int pwmAlvo;
  int passoMaximo; // O máximo de PWM que pode mudar por vez

  uint32_t ultimoTempoMs;
  uint32_t intervaloMs; // A cada quantos milissegundos o incremento acontece
};

// Função que cria e inicializa uma nova struct motor
Motor criarMotor(uint8_t pinoPWM, uint8_t pinoDir);

// Função que define a velocidade e a direção
void moverMotor(Motor *motor, int velocidade);

// Função para emitir som usando as bobinas do motor (como buzzer) sem rodar
void tocarSomMotor(Motor *motor, uint32_t frequencia, uint32_t duracao_ms);