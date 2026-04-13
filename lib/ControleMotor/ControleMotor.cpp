#include "ControleMotor.h"

void atualizarVelocidade(Motor *motor) {
  /* Suaviza a mudança de velocidade até chegar à velocidade alvo */
  motor->velocidadeAtual = ALPHA * motor->velocidadeAtual + (1 - ALPHA) * motor->velocidadeAlvo;

  /* Valores positivos de Velocidade fazem o motor girar numa direção e negativos noutra */
  if (motor->velocidadeAtual > 0) {
    analogWrite(motor->pin0, motor->velocidadeAtual);
    digitalWrite(motor->pin1, LOW);
  } else if (motor->velocidadeAtual == 0) {
    digitalWrite(motor->pin0, LOW);
    digitalWrite(motor->pin1, LOW);
    analogWrite(motor->pin0, 1);
    analogWrite(motor->pin1, 1);
  } else {
    digitalWrite(motor->pin0, LOW);
    analogWrite(motor->pin1, -motor->velocidadeAtual);
  }
}