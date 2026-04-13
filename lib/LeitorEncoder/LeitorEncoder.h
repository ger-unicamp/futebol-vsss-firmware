#pragma once
#include <Arduino.h>

#define CIRCUNFERENCIA_RODA 21.0
#define PULSOS_POR_REVOLUCAO 7.0

typedef struct encoder {
  uint8_t pinA;
  uint8_t pinB;
  volatile long ticks;
  long lastTicks;
  float velocidade_cm_s;
} Encoder;

void IRAM_ATTR logicaEncoder(Encoder *enc);
void calcularVelocidadeEncoder(Encoder *enc, unsigned long deltaTime_ms);