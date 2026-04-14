#include "Motor.h"

#define FREQ_PWM_ALTA 20000
#define RESOLUCAO_PWM 10

#define PASSO_MAXIMO 50
#define INTERVALO_MS 10

Motor criarMotor(uint8_t pinoPWM, uint8_t pinoDir, uint8_t canalLEDC)
{
  Motor m;
  m.pinoPWM = pinoPWM;
  m.pinoDir = pinoDir;
  m.canalLEDC = canalLEDC;
  m.pwmAtual = 0;
  m.pwmAlvo = 0;
  m.passoMaximo = PASSO_MAXIMO;
  m.ultimoTempoMs = 0;
  m.intervaloMs = INTERVALO_MS;

  pinMode(m.pinoDir, OUTPUT);
  digitalWrite(m.pinoDir, LOW);

  ledcSetup(m.canalLEDC, FREQ_PWM_ALTA, RESOLUCAO_PWM);
  ledcAttachPin(m.pinoPWM, m.canalLEDC);
  ledcWrite(m.canalLEDC, 0);

  return m;
}

void moverMotor(Motor *motor, int pwmDesejado)
{
  // 1. Limita o alvo aos limites físicos do PWM
  if (pwmDesejado > 1023)
    pwmDesejado = 1023;
  if (pwmDesejado < -1023)
    pwmDesejado = -1023;

  motor->pwmAlvo = pwmDesejado;

  // 2. Verifica se já passou o tempo necessário para dar o próximo passo
  uint32_t tempoAtual = millis();
  if (tempoAtual - motor->ultimoTempoMs >= motor->intervaloMs)
  {

    int diferenca = motor->pwmAlvo - motor->pwmAtual;

    // Troca máxima instantânea permitida
    if (diferenca > motor->passoMaximo)
    {
      motor->pwmAtual += motor->passoMaximo;
    }
    else if (diferenca < -motor->passoMaximo)
    {
      motor->pwmAtual -= motor->passoMaximo;
    }
    else
    {
      motor->pwmAtual = motor->pwmAlvo;
    }

    // Atualiza o relógio interno do motor
    motor->ultimoTempoMs = tempoAtual;
  }

  // 3. Aplica o sinal físico aos pinos baseado no pwmAtual seguro
  int velocidadeAplicada = motor->pwmAtual;

  if (velocidadeAplicada >= 0)
  {
    digitalWrite(motor->pinoDir, HIGH);
  }
  else
  {
    digitalWrite(motor->pinoDir, LOW);
    velocidadeAplicada = -velocidadeAplicada;
  }

  ledcSetup(motor->canalLEDC, FREQ_PWM_ALTA, RESOLUCAO_PWM);
  ledcWrite(motor->canalLEDC, velocidadeAplicada);
}

void tocarSomMotor(Motor *motor, uint32_t frequencia, uint32_t duracao_ms)
{
  ledcSetup(motor->canalLEDC, frequencia, RESOLUCAO_PWM);
  ledcWrite(motor->canalLEDC, 50);
  delay(duracao_ms);
  ledcWrite(motor->canalLEDC, 0);
  ledcSetup(motor->canalLEDC, FREQ_PWM_ALTA, RESOLUCAO_PWM);

  // Reseta o estado para evitar que o motor tente "compensar" uma diferença irreal ao voltar a girar
  motor->pwmAtual = 0;
  motor->pwmAlvo = 0;
  motor->ultimoTempoMs = millis();
}