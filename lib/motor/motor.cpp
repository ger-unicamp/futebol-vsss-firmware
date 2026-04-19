#include "motor.h"

motor_t criar_motor(uint8_t pinoPWM, uint8_t pinoDir, uint16_t passoMaximo)
{
  motor_t m;
  m.pin_pwm = pinoPWM;
  m.pin_dir = pinoDir;
  m.pwm_atual = 0;
  m.passo_maximo_pwm = passoMaximo;

  // O pino de direção só cuida da polaridade
  pinMode(m.pin_dir, OUTPUT);
  digitalWrite(m.pin_dir, LOW);

  // O pino de PWM vai para o periférico LEDC
  ledcAttach(m.pin_pwm, FREQ_PWM_ALTA, RESOLUCAO_PWM);
  ledcWrite(m.pin_pwm, m.pwm_atual);

  return m;
}

void mover_motor(motor_t *motor, int16_t pwmDesejado)
{
  // 1. Limita o alvo aos limites físicos do PWM (agora vai até -1023 e 1023 corretamente)
  if (pwmDesejado > PID_MAX_PWM)
    pwmDesejado = PID_MAX_PWM;
  if (pwmDesejado < PID_MIN_PWM)
    pwmDesejado = PID_MIN_PWM;

  // 2. Aplica o passo máximo para evitar mudanças bruscas
  int16_t delta = pwmDesejado - motor->pwm_atual;
  if (abs(delta) > motor->passo_maximo_pwm)
  {
    if (delta > 0)
      pwmDesejado = motor->pwm_atual + motor->passo_maximo_pwm;
    else
      pwmDesejado = motor->pwm_atual - motor->passo_maximo_pwm;
  }

  // 3. Aplica o sinal físico aos pinos baseado no pwmAtual seguro
  if (pwmDesejado >= 0)
  {
    digitalWrite(motor->pin_dir, LOW);
  }
  else
  {
    digitalWrite(motor->pin_dir, HIGH);
    pwmDesejado = pwmDesejado + PID_MAX_PWM;
  }

  ledcWrite(motor->pin_pwm, pwmDesejado);
}

void tocar_som_motor(motor_t *motor, uint32_t frequencia, uint32_t duracao_ms)
{
  // Muda a frequência e toca o som
  ledcWriteTone(motor->pin_pwm, frequencia);
  delay(duracao_ms);

  // CORREÇÃO: Desanexa o timer usado para a nota musical antes de voltar a ser PWM de motor
  ledcDetach(motor->pin_pwm);

  // Reconecta com a frequência e resolução do motor
  ledcAttach(motor->pin_pwm, FREQ_PWM_ALTA, RESOLUCAO_PWM);
  ledcWrite(motor->pin_pwm, 0);

  // Reseta o estado para evitar que o motor tente "compensar" uma diferença irreal
  motor->pwm_atual = 0;
}