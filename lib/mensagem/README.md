# Biblioteca Mensagem

A biblioteca `mensagem` define o protocolo de comunicação e as estruturas de dados padrão para a troca de informações entre o transmissor (PC/TX) e os robôs (RX).

## Funcionalidades

- **Definição de Comandos:** Enumeração `tipo_comando` com todos os comandos suportados (Movimento, PID, Telemetria, Pareamento, etc.).
- **Protocolo Binário:** Estrutura `mensagem_t` compactada (`__attribute__((packed))`) para garantir consistência entre diferentes dispositivos e linguagens (ex: Python e C++).

## Dependências

- `Arduino.h`
- `stdint.h`

## Estruturas de Dados

### `mensagem_t`
Estrutura principal de transporte:
- `uint8_t tipo`: Tipo do comando.
- `uint8_t indice_destino`: ID do destinatário.
- `uint8_t indice_remetente`: ID de quem enviou.
- `union Payload`: União que contém as structs específicas para cada comando (ex: `Movimento`, `Telemetria`, `PIDConfig`).

### Comandos Principais
- `COMANDO_MOVIMENTO`: Envia velocidades para as rodas.
- `COMANDO_TELEMETRIA`: Robô envia status (bateria, RSSI, erros).
- `COMANDO_PAREAMENTO`: Processo de autenticação inicial.
- `COMANDO_AUTOTUNE_PID`: Inicia o ajuste automático de ganhos.

## Uso

Esta biblioteca é puramente de definição (`header-only` na prática de uso). Ela deve ser incluída em todos os módulos que participam da comunicação.

```cpp
#include <mensagem.h>

mensagem_t msg;
msg.tipo = COMANDO_MOVIMENTO;
msg.payload.movimento.v_roda_esquerda = 50;
msg.payload.movimento.v_roda_direita = -50;
```
