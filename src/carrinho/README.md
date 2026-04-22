# Firmware do Carrinho (RX)

Este é o firmware principal executado nos robôs da categoria VSSS. Ele é responsável por receber comandos de movimento, controlar os motores usando PID e encoders, e reportar telemetria.

## Funcionalidades

- **Controle de Movimento:** Recebe velocidades desejadas via ESP-NOW e utiliza controladores PID para manter a velocidade das rodas.
- **Pareamento e Segurança:** Valida se as mensagens vêm de um transmissor autorizado.
- **Telemetria:** Envia periodicamente dados como RSSI (sinal), velocidade das rodas e velocidade desejada.
- **Fail-safe (TTL):** Para os motores automaticamente se não receber comandos por um período determinado (Time-To-Live).
- **Configuração Remota:** Permite atualizar ganhos PID e ID do robô via rádio, salvando-os na memória flash.

## Estrutura do Código

- `main.cpp`: Integração de todas as bibliotecas (`motor`, `encoder`, `pid`, `memoria`, `despachante`).
- `on_data_recv`: Callback do ESP-NOW que processa diferentes tipos de comandos.
- `loop`: Executa o ciclo de controle de 10ms (ou conforme configurado) e gerencia tarefas de fundo.

## Dependências de Hardware

- ESP32C3 (S3 ou similar).
- Ponte-H para motores DC.
- Encoders de quadratura.
