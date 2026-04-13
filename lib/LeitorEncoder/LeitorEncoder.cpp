#include "LeitorEncoder.h"

void IRAM_ATTR logicaEncoder(Encoder *enc) {
  if (digitalRead(enc->pinB) == LOW) {
    enc->ticks++;
  } else {
    enc->ticks--;
  }
}

void calcularVelocidadeEncoder(Encoder *enc, unsigned long deltaTime_ms) {
  if (deltaTime_ms == 0) return;
  
  long currentTicks = enc->ticks;
  long deltaTicks = currentTicks - enc->lastTicks;
  enc->lastTicks = currentTicks;

  float distancia_cm = (deltaTicks * CIRCUNFERENCIA_RODA) / PULSOS_POR_REVOLUCAO;
  float tempo_s = deltaTime_ms / 1000.0;
  enc->velocidade_cm_s = distancia_cm / tempo_s;
}