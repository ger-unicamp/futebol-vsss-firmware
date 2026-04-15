#pragma once
#include <Arduino.h>

// Struct para armazenar o estado do encoder
typedef struct encoder
{
  uint8_t pinA;        // Pino de interrupção (Pulso)
  uint8_t pinB;        // Pino de direção
  volatile long ticks; // Contador global de pulsos (Volatile pois muda na ISR)
  long lastTicks;      // Memória do último ciclo
  long delta_ticks;    // Variação de pulsos (Variável de Processo para o PID)
} Encoder;

// Inicializa os pinos e zera os valores da struct
void inicializarEncoder(Encoder *enc, uint8_t pA, uint8_t pB);

// Função executada dentro da interrupção
void IRAM_ATTR logicaEncoder(Encoder *enc);

// Função para calcular a variação de ticks no loop de controle
void atualizarDeltaTicks(Encoder *enc);

// Macro mágica para gerar as funções ISR na main.cpp de forma genérica
#define CRIAR_ISR_ENCODER(nome_funcao, instancia_encoder) void IRAM_ATTR nome_funcao() { logicaEncoder(&instancia_encoder); }