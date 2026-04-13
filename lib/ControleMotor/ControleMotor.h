#pragma once
#include <Arduino.h>

/* Suaviza a transicao de velocidades, ( 0-> muda instantaneamente, 1-> nunca muda) */
#define ALPHA 0.6 

typedef struct motor {
  short pin0;
  short pin1;
  int velocidadeAtual;
  int velocidadeAlvo;
} Motor;

void atualizarVelocidade(Motor *motor);