#include "Motor.h"

Motor criarMotor(uint8_t pinoPWM, uint8_t pinoDir)
{
  Motor m;
  m.pinoPWM = pinoPWM;
  m.pinoDir = pinoDir;
  m.pwmAtual = 0;
  m.pwmAlvo = 0;
  m.passoMaximo = PASSO_MAXIMO;
  m.ultimoTempoMs = 0;
  m.intervaloMs = INTERVALO_MS;

  // CORREÇÃO: Desanexa qualquer configuração fantasma do pino para evitar falta de Timers
  // ledcDetach(m.pinoPWM);

  // O pino de direção só cuida da polaridade
  pinMode(m.pinoDir, OUTPUT);
  digitalWrite(m.pinoDir, LOW);

  // O pino de PWM vai para o periférico LEDC
  ledcAttach(m.pinoPWM, FREQ_PWM_ALTA, RESOLUCAO_PWM);
  ledcWrite(m.pinoPWM, 0);

  return m;
}

void moverMotor(Motor *motor, int pwmDesejado)
{
  // 1. Limita o alvo aos limites físicos do PWM (agora vai até -1023 e 1023 corretamente)
  if (pwmDesejado > PID_MAX_PWM)
    pwmDesejado = PID_MAX_PWM;
  if (pwmDesejado < PID_MIN_PWM)
    pwmDesejado = PID_MIN_PWM;

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

  ledcWrite(motor->pinoPWM, velocidadeAplicada);
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
  motor->pwmAlvo = 0;
  motor->ultimoTempoMs = millis();
}