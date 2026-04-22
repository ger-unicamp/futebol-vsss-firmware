# Biblioteca PID

A biblioteca `pid` fornece uma implementação robusta de controle Proporcional-Integral-Derivativo (PID) e um mecanismo de ajuste automático (Autotune) baseado no método de relé de Ziegler-Nichols.

## Funcionalidades

- **Controlador PID Completo:** Inclui termos Proporcional, Integral, Derivativo e Feedforward (Kf).
- **Anti-Windup:** Proteção para evitar que o erro integral cresça indefinidamente quando a saída está saturada.
- **Sintonização Automática (Autotune):** Implementa o método de oscilação para calcular os ganhos Kp e Ki ideais para o sistema.

## Dependências

- `Arduino.h`

## Estruturas de Dados

### `pid_config_t`
Armazena os ganhos (`kp`, `ki`, `kd`, `kf`), histórico de erros, soma integral e limites de saída.

### `pid_autotune_t`
Armazena o estado do processo de calibração automática.

## Funções Principais

- `void pid_iniciar(...)`: Inicializa a estrutura do PID com os ganhos e limites.
- `float pid_computar(pid_config_t *pid, float setpoint, float medido, float dt)`: Calcula a saída do controle.
- `void autotune_iniciar(...)`: Inicia o processo de calibração.
- `float autotune_computar(...)`: Executa a lógica de oscilação e retorna os ganhos ao finalizar.

## Exemplo de Uso

```cpp
#include <pid.h>

pid_config_t pidEsq;

void setup() {
    pid_iniciar(&pidEsq, 1.5, 0.5, 0.1, 0.05, -255, 255);
}

void loop() {
    float erro = setpoint - leitura;
    float saida = pid_computar(&pidEsq, setpoint, leitura, 0.01);
    mover_motor(&motor, saida);
}
```
