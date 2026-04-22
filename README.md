# futebol-vsss-firmware

Firmware para robôs da categoria VSSS (Very Small Size Soccer) utilizando ESP32 e protocolo ESP-NOW.

# Documentação de Arquitetura: Sistema de Comunicação VSSS (Python <-> ESP32)

## 1. Visão Geral do Sistema
O sistema implementa uma infraestrutura de comunicação bidirecional de baixa latência. A arquitetura é dividida em três camadas principais:
1. **Host (PC - Python):** Executa a lógica da IA, processa pacotes via interface CFFI, gerencia a rede lógica e analisa a telemetria em tempo real.
2. **Transmissor (TX - ESP32 Bridge):** Atua como uma ponte entre o PC (via Serial/USB) e os robôs. Converte comandos seriais para o protocolo ESP-NOW e vice-versa, mantendo uma lista de confiança de robôs ativos.
3. **Robôs (RX - ESP32 Carrinhos):** Recebem comandos de movimento e configuração, executam o controle PID dos motores via encoders e devolvem dados de telemetria.

---

## 2. Regras de Endereçamento (IDs)
O roteamento dos pacotes é lógico, permitindo a substituição rápida de robôs sem fixar endereços MAC no código da IA.
* **`0` (`ID_TRANSMISSOR`)**: ID exclusivo do Transmissor (PC/TX).
* **`1` a `6`**: IDs válidos dos robôs em campo.
* **`255` (`ID_BROADCAST`)**: ID de Broadcast Lógico. Usado para comandos globais ou para configurar robôs recém-ligados.

---

## 3. Estrutura de Dados (`mensagem.h`)
O pacote de dados trafega de ponta a ponta usando a mesma struct binária empacotada (`__attribute__((packed))`), garantindo que o `payload` tenha o mesmo layout na memória tanto no firmware quanto no Python.

**Campos principais da `mensagem_t`:**
* `tipo`: Identificador do comando baseado no enum `tipo_comando`.
* `indice_destino`: ID do destinatário da mensagem.
* `indice_remetente`: ID de quem gerou a mensagem.
* `payload`: Uma `union` contendo estruturas específicas para cada comando (ex: `movimento`, `telemetria`, `pareamento`, `set_id`).

---

## 4. Comandos e Mecânicas Principais

### A. Telemetria e Diagnóstico (`COMANDO_TELEMETRIA`)
Os robôs enviam periodicamente (ou sob demanda) pacotes de telemetria contendo:
* **MAC Address:** Para identificação física.
* **Ticks dos Encoders:** Variação real (`delta_ticks_atual`) vs desejada (`delta_ticks_target`).
* **Sinal (RSSI):** Qualidade do sinal medida tanto no robô quanto no transmissor.
* **Contadores:** Total de pacotes recebidos e pacotes válidos (para aferição de Packet Loss).

### B. Configuração Dinâmica de ID (`COMANDO_ID`)
Para evitar conflitos de IDs, o sistema permite alterar o ID de um robô em tempo real:
* O PC envia um `COMANDO_ID` contendo o **MAC Address** alvo e o **novo_id**.
* O robô que reconhecer seu MAC salva o novo ID na memória Flash (NVS) e passa a responder por ele imediatamente.

### C. Pareamento e Segurança (`COMANDO_PAREAMENTO`)
Implementa uma camada de segurança simples para evitar interferências:
* O robô só aceita comandos de movimento se o `indice_remetente` for o transmissor pareado.
* O pareamento é feito via senha e o robô armazena o MAC do transmissor como "chave" de segurança na Flash.

### D. Controle de Motores e PID (`COMANDO_PID`, `COMANDO_MOVIMENTO`)
* **Movimento:** Envia o *setpoint* de velocidade (ticks por ciclo) para cada roda.
* **PID:** Permite ajustar os ganhos `Kp`, `Ki`, `Kd` e `Kf` via rádio para sintonia fina sem necessidade de recompilação.

---

## 5. Estrutura do Projeto

O projeto está organizado em bibliotecas modulares e aplicações específicas:

### Aplicativos (`src/`)
- [**Carrinho**](./src/carrinho/README.md): Firmware para os robôs (RX). Controle PID, encoders e telemetria.
- [**Transmissor**](./src/transmissor/README.md): Firmware para a ponte TX (USB <-> Rádio).

### Bibliotecas (`lib/`)
- [**Despachante**](./lib/despachante/README.md): Gerenciamento assíncrono de mensagens e logs.
- [**Encoder**](./lib/encoder/README.md): Leitura de encoders de quadratura.
- [**Memória**](./lib/memoria/README.md): Armazenamento persistente (NVS) de configurações.
- [**Mensagem**](./lib/mensagem/README.md): Definição do protocolo de comunicação.
- [**Motor**](./lib/motor/README.md): Controle de drivers de motor com rampa de aceleração.
- [**PID**](./lib/pid/README.md): Controlador PID e Autotune.

---

