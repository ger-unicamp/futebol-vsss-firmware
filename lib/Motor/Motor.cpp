#include "Motor.h"

Motor criarMotor(uint8_t pinoPWM, uint8_t pinoDir, uint16_t passoMaximo)
{
  Motor m;
  m.pinoPWM = pinoPWM;
  m.pinoDir = pinoDir;
  m.pwmAtual = 0;
  m.passoMaximo = passoMaximo;

  // O pino de direção só cuida da polaridade
  pinMode(m.pinoDir, OUTPUT);
  digitalWrite(m.pinoDir, LOW);

  // O pino de PWM vai para o periférico LEDC
  ledcAttach(m.pinoPWM, FREQ_PWM_ALTA, RESOLUCAO_PWM);
  ledcWrite(m.pinoPWM, m.pwmAtual);

  return m;
}

void moverMotor(Motor *motor, int16_t pwmDesejado)
{
  // 1. Limita o alvo aos limites físicos do PWM (agora vai até -1023 e 1023 corretamente)
  if (pwmDesejado > PID_MAX_PWM)
    pwmDesejado = PID_MAX_PWM;
  if (pwmDesejado < PID_MIN_PWM)
    pwmDesejado = PID_MIN_PWM;

  // 2. Aplica o passo máximo para evitar mudanças bruscas
  int16_t delta = pwmDesejado - motor->pwmAtual;
  if (abs(delta) > motor->passoMaximo)
  {
    if (delta > 0)
      pwmDesejado = motor->pwmAtual + motor->passoMaximo;
    else
      pwmDesejado = motor->pwmAtual - motor->passoMaximo;
  }

  // 3. Aplica o sinal físico aos pinos baseado no pwmAtual seguro
  if (pwmDesejado >= 0)
  {
    digitalWrite(motor->pinoDir, LOW);
  }
  else
  {
    digitalWrite(motor->pinoDir, HIGH);
    pwmDesejado = pwmDesejado + PID_MAX_PWM;
  }

  ledcWrite(motor->pinoPWM, pwmDesejado);
}

void tocarSomMotor(Motor *motor, uint32_t frequencia, uint32_t duracao_ms)
{
  // Muda a frequência e toca o som
  ledcWriteTone(motor->pinoPWM, frequencia);
  delay(duracao_ms);

  // CORREÇÃO: Desanexa o timer usado para a nota musical antes de voltar a ser PWM de motor
  ledcDetach(motor->pinoPWM);

  // Reconecta com a frequência e resolução do motor
  ledcAttach(motor->pinoPWM, FREQ_PWM_ALTA, RESOLUCAO_PWM);
  ledcWrite(motor->pinoPWM, 0);

  // Reseta o estado para evitar que o motor tente "compensar" uma diferença irreal
  motor->pwmAtual = 0;
}