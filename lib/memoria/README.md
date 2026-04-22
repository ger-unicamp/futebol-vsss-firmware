# Biblioteca Memória

A biblioteca `memoria` fornece armazenamento persistente para dados de configuração no ESP32 utilizando a API NVS (Non-Volatile Storage). Isso permite que configurações como o índice do robô, parâmetros PID e status de pareamento sejam mantidos após reinicializações.

## Funcionalidades

- **Inicialização do NVS:** Gerencia a inicialização da partição flash, lidando com erros ou falta de espaço.
- **Persistência de Dados:** Salva e carrega a estrutura `memoria_t` como um bloco binário (blob).
- **Configurações Abrangentes:** Armazena parâmetros de controle (PID), MAC do transmissor, canais Wi-Fi e períodos de telemetria.

## Dependências

- `nvs_flash.h`
- `nvs.h` (ESP-IDF NVS API)
- `stdint.h`, `stdbool.h`

## Estruturas de Dados

### `memoria_t`
Contém todos os parâmetros persistentes do robô, incluindo:
- `uint8_t indice`: ID do robô (1 a 6).
- `uint8_t mac_esp_principal[6]`: MAC do transmissor pareado.
- `pid_params_t pid[2]`: Ganhos Kp, Ki, Kd e Kf para os motores.
- `bool pareado`: Indica se o robô está autenticado.
- Períodos de controle, TTL e telemetria.

## Funções Principais

- `bool inicializar_memoria()`: Inicializa o sistema de arquivos NVS.
- `bool salvar_config(const memoria_t *config)`: Grava as configurações atuais na flash.
- `bool carregar_config(memoria_t *config)`: Lê as configurações da flash para a memória RAM.

## Exemplo de Uso

```cpp
#include <memoria.h>

memoria_t config;

void setup() {
    if (inicializar_memoria()) {
        if (!carregar_config(&config)) {
            // Se falhar ao carregar, define valores padrão
            config.indice = 255;
            salvar_config(&config);
        }
    }
}
```
