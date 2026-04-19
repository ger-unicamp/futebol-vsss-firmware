#pragma once
#include <Arduino.h>

#define FREQ_PWM_ALTA 25000
#define RESOLUCAO_PWM 10

// CORREÇÃO: Usando deslocamento de bits (Shift Left) para calcular 2^10 - 1 = 1023
#define PID_MAX_PWM ((1 << RESOLUCAO_PWM) - 1)
#define PID_MIN_PWM -PID_MAX_PWM

// Estrutura que guarda as configurações do motor
struct Motor
{
  uint8_t pinoPWM;
  uint8_t pinoDir;

  uint16_t passoMaximo;
  uint16_t pwmAtual;
};

// Função que cria e inicializa uma nova struct motor
Motor criarMotor(uint8_t pinoPWM, uint8_t pinoDir, uint16_t passoMaximo);

// Função que define a velocidade e a direção
void moverMotor(Motor *motor, int16_t pwm);

// Função para emitir som usando as bobinas do motor (como buzzer) sem rodar
void tocarSomMotor(Motor *motor, uint32_t frequencia, uint32_t duracao_ms);