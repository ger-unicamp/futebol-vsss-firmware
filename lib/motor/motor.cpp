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
  // 1. Limita o comando aos limites absolutos do sistema (-1023 a 1023)
  pwmDesejado = constrain(pwmDesejado, PID_MIN_PWM, PID_MAX_PWM);

  // 2. Rampa de Aceleração: Limita a variação em relação ao estado atual
  int16_t limite_inferior = motor->pwm_atual - motor->passo_maximo_pwm;
  int16_t limite_superior = motor->pwm_atual + motor->passo_maximo_pwm;

  pwmDesejado = constrain(pwmDesejado, limite_inferior, limite_superior);

  // 3. Atualiza o estado lógico do motor na memória
  motor->pwm_atual = pwmDesejado;

  // 4. Traduz a lógica para o hardware (Ponte H)
  uint32_t pwm_fisico;

  if (pwmDesejado >= 0)
  {
    digitalWrite(motor->pin_dir, LOW);
    pwm_fisico = pwmDesejado;
  }
  else
  {
    digitalWrite(motor->pin_dir, HIGH);
    // Como pwmDesejado é negativo, somar ao MAX equivale a: MAX - abs(pwmDesejado)
    // Necessário para drivers onde HIGH no pino de direção inverte a lógica do PWM
    pwm_fisico = pwmDesejado + PID_MAX_PWM;
  }

  // 5. Aciona os pinos
  ledcWrite(motor->pin_pwm, pwm_fisico);
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