#include "Encoder.h"

void inicializar_encoder(encoder_t *enc, uint8_t pA, uint8_t pB)
{
  enc->pin_a = pA;
  enc->pin_b = pB;
  enc->ticks = 0;
  enc->last_ticks = 0;
  enc->delta_ticks = 0;

  // Configura os pinos como entrada com pull-up interno
  pinMode(enc->pin_a, INPUT_PULLUP);
  pinMode(enc->pin_b, INPUT_PULLUP);
}

// A lógica de leitura usando 1 pino de interrupção e 1 de direção.
// Presumindo que a interrupção seja RISING no canal A.
// void IRAM_ATTR logica_encoder(Encoder *enc)
// {
//   // Lê o canal B para determinar a direção
//   if (digitalRead(enc->pin_b) == HIGH)
//     enc->ticks = enc->ticks + 1;
//   else
//     enc->ticks = enc->ticks - 1;
// }
void IRAM_ATTR logica_encoder(encoder_t *enc)
{
  // Lê o estado dos dois pinos
  bool estadoA = digitalRead(enc->pin_a);
  bool estadoB = digitalRead(enc->pin_b);

  // Na leitura 2X, comparamos se os estados são iguais ou diferentes
  if (estadoA == estadoB)
    enc->ticks = enc->ticks + 1;
  else
    enc->ticks = enc->ticks - 1;
}

// Essa função deve ser chamada em uma frequência fixa (ex: a cada 10ms ou 20ms)
void atulaizar_delta_ticks(encoder_t *enc)
{
  // Pausa as interrupções rapidamente para copiar a variável volatile com segurança
  // (evita que a variável mude exatamente no meio da leitura de 32 bits)
  noInterrupts();
  long currentTicks = enc->ticks;
  interrupts();

  // Calcula a diferença de passos desde a última leitura
  enc->delta_ticks = currentTicks - enc->last_ticks;

  // Atualiza a memória
  enc->last_ticks = currentTicks;
}