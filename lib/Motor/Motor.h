#pragma once
#include <Arduino.h>

#define FREQ_PWM_ALTA 25000
#define RESOLUCAO_PWM 10

// CORREÇÃO: Usando deslocamento de bits (Shift Left) para calcular 2^10 - 1 = 1023
#define PID_MAX_PWM ((1 << RESOLUCAO_PWM) - 1)
#define PID_MIN_PWM -PID_MAX_PWM

// Estrutura que guarda as configurações do motor
struct motor_t
{
  uint8_t pin_pwm;
  uint8_t pin_dir;

  uint16_t passo_maximo_pwm;
  uint16_t pwm_atual;
};

// Função que cria e inicializa uma nova struct motor
motor_t criar_motor(uint8_t pinoPWM, uint8_t pinoDir, uint16_t passo_maximo_pwm);

// Função que define a velocidade e a direção
void mover_motor(motor_t *motor, int16_t pwm);

// Função para emitir som usando as bobinas do motor (como buzzer) sem rodar
void tocar_som_motor(motor_t *motor, uint32_t frequencia, uint32_t duracao_ms);