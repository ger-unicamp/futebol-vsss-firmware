# Biblioteca Encoder

A biblioteca `encoder` fornece funcionalidades para leitura e processamento de dados de encoders de quadratura, utilizados para medir a rotação e direção dos motores.

## Funcionalidades

- **Gestão de Estado do Encoder:** Armazena pinos, contagem de pulsos (ticks) e variação (delta).
- **Leitura via Interrupção:** Utiliza interrupções nos pinos para garantir precisão na contagem.
- **Cálculo de Delta Ticks:** Calcula a mudança na contagem de pulsos desde a última medição, essencial para sistemas de controle como PID.
- **Macro para ISR:** Facilita a criação de funções de interrupção para múltiplas instâncias de encoder.

## Dependências

- `Arduino.h`

## Estruturas de Dados

### `encoder_t`
```cpp
typedef struct encoder {
    uint8_t pin_a, pin_b;
    volatile int32_t ticks;
    int32_t last_ticks;
    int32_t delta_ticks;
} encoder_t;
```

## Funções Principais

- `void inicializar_encoder(encoder_t *enc, uint8_t pA, uint8_t pB)`: Inicializa a estrutura e configura os pinos com pull-up interno.
- `void IRAM_ATTR logica_encoder(encoder_t *enc)`: Lógica executada na interrupção para atualizar a contagem de ticks.
- `void atualizar_delta_ticks(encoder_t *enc)`: Calcula o `delta_ticks` atual e atualiza o estado anterior de forma segura (protegida contra interrupções).

## Exemplo de Uso

```cpp
#include <encoder.h>

encoder_t encoderEsq;

void IRAM_ATTR isr_encoder() {
    logica_encoder(&encoderEsq);
}

void setup() {
    inicializar_encoder(&encoderEsq, 34, 35);
    attachInterrupt(digitalPinToInterrupt(34), isr_encoder, CHANGE);
}

void loop() {
    atualizar_delta_ticks(&encoderEsq);
    int32_t velocidade = encoderEsq.delta_ticks;
    delay(10);
}
```
