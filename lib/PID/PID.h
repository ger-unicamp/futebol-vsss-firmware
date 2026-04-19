#pragma once
#include <Arduino.h>

// Struct principal com os parâmetros e estado do PID
typedef struct
{
    float kp;
    float ki;
    float kd;
    float kf; // Ganho de Feedforward (Avanço de sinal)

    float erro_anterior;
    float integral;
    float limite_min;
    float limite_max;

    unsigned long ultimo_tempo_us;
} pid_config_t;

// Struct para manter o estado da máquina de estados do Autotune
typedef struct
{
    bool ativo;
    int ciclos_completos;
    int ciclos_desejados;

    float pico_max;
    float pico_min;

    unsigned long ultimo_cruzamento_us;
    unsigned long soma_periodos_us;

    bool estado_rele;
    float saida_rele_max;
    float saida_rele_min;
} pid_autotune_t;

// Inicializa a struct. Para 10bits bidirecional, use limites -1023 a 1023
void pid_iniciar(pid_config_t *pid, float kp, float ki, float kd, float kf, float limite_min, float limite_max);
void pid_resetar(pid_config_t *pid);
float pid_computar(pid_config_t *pid, float setpoint_ticks, float valor_medido_ticks);

// O autotune de motor precisa ser rápido. saídas max/min devem ser valores de PWM que o motor consiga girar bem
void autotune_iniciar(pid_autotune_t *tune, float pwm_teste_max, float pwm_teste_min, int ciclos);
float autotune_computar(pid_autotune_t *tune, pid_config_t *pid, float setpoint_ticks, float valor_medido_ticks);