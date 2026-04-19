#pragma once
#include <Arduino.h>

// Struct para armazenar o estado do encoder
typedef struct encoder
{
  uint8_t pin_a;          // Pino de interrupção (Pulso)
  uint8_t pin_b;          // Pino de direção
  volatile int32_t ticks; // Contador global de pulsos (Volatile pois muda na ISR)
  int32_t last_ticks;     // Memória do último ciclo
  int32_t delta_ticks;    // Variação de pulsos (Variável de Processo para o PID)
} encoder_t;

// Inicializa os pinos e zera os valores da struct
void inicializar_encoder(encoder_t *enc, uint8_t pA, uint8_t pB);

// Função executada dentro da interrupção
void IRAM_ATTR logica_encoder(encoder_t *enc);

// Função para calcular a variação de ticks no loop de controle
void atulaizar_delta_ticks(encoder_t *enc);

// Macro mágica para gerar as funções ISR na main.cpp de forma genérica
#define CRIAR_ISR_ENCODER(nome_funcao, instancia_encoder) \
  void IRAM_ATTR nome_funcao() { logica_encoder(&instancia_encoder); }