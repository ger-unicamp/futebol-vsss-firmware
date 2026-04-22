# Biblioteca Despachante

A biblioteca `despachante` é responsável por gerenciar o envio de mensagens de forma assíncrona, principalmente para a comunicação ESP-NOW e logs de depuração. Ela utiliza filas do FreeRTOS para evitar o bloqueio de operações críticas de tempo.

## Funcionalidades

- **Envio Assíncrono de Mensagens:** Enfileira estruturas `mensagem_t` para transmissão posterior via ESP-NOW, evitando bloqueios em ISRs ou callbacks.
- **Log de Depuração Assíncrono:** Fornece a função `despachante_printf` que enfileira strings de log, processando-as e enviando-as via ESP-NOW (se pareado) no loop principal.
- **Segmentação de Mensagens:** Divide strings de log longas em múltiplas mensagens `COMANDO_PRINT` para respeitar o limite de carga útil da `mensagem_t`.

## Dependências

- `Arduino.h`
- `esp_now.h`
- `mensagem.h` (Definição do protocolo)
- `memoria.h` (Configurações e status de pareamento)
- `freertos/FreeRTOS.h`
- `freertos/queue.h`

## Estruturas de Dados

A biblioteca utiliza as filas `fila_tx` (para `mensagem_t`) e `fila_logs` (para strings de log).

## Funções Principais

- `void despachante_init()`: Inicializa as filas do FreeRTOS.
- `void despachante_enfileirar(mensagem_t msg)`: Adiciona uma mensagem à fila de transmissão.
- `void despachante_printf(const char *formato, ...)`: Formata e enfileira uma string de log.
- `void despachante_processar_loop()`: Processa as filas, realizando o envio efetivo via ESP-NOW. Deve ser chamada no `loop()`.

## Exemplo de Uso

```cpp
#include <despachante.h>

void setup() {
    despachante_init();
}

void loop() {
    despachante_processar_loop();
    
    if (algo_aconteceu) {
        despachante_printf("Evento detectado: %d", valor);
    }
}
```
