# Biblioteca Motor

A biblioteca `motor` fornece uma interface de alto nível para controle de motores DC usando pontes-H e o periférico LEDC (PWM) do ESP32.

## Funcionalidades

- **Controle de Velocidade e Direção:** Suporta valores de PWM sinalizados (ex: -255 a 255).
- **Suavização de Movimento (Ramping):** Implementa um limite de passo máximo (`passo_maximo_pwm`) para evitar picos de corrente e desgaste mecânico.
- **Feedback Sonoro:** Permite usar o motor como um buzzer para emitir bips de status ou erro.

## Dependências

- `Arduino.h`

## Estruturas de Dados

### `motor_t`
```cpp
typedef struct motor_t {
    uint8_t pin_pwm, pin_dir;
    uint16_t passo_maximo_pwm;
    uint16_t pwm_atual;
} motor_t;
```

## Funções Principais

- `motor_t criar_motor(uint8_t pinoPWM, uint8_t pinoDir, uint16_t passo_maximo_pwm)`: Configura os pinos e o periférico PWM.
- `void mover_motor(motor_t *motor, int16_t pwmDesejado)`: Aplica a velocidade desejada respeitando o limite de aceleração.
- `void tocar_som_motor(motor_t *motor, uint32_t frequencia, uint32_t duracao_ms)`: Altera a frequência do PWM para gerar som.

## Exemplo de Uso

```cpp
#include <motor.h>

motor_t motorDir;

void setup() {
    motorDir = criar_motor(12, 13, 20); // Pinos 12 e 13, rampa de 20 unidades por ciclo
}

void loop() {
    mover_motor(&motorDir, 255); // Vai para frente
    delay(1000);
    mover_motor(&motorDir, -255); // Vai para trás
    delay(1000);
}
```
