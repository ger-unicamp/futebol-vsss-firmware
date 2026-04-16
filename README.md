# futebol-vsss-firmware
Guarda o firmware dos carrinhos do futebol
# Documentação de Arquitetura: Sistema de Comunicação VSSS (Python <-> ESP32)

## 1. Visão Geral do Sistema
O sistema implementa a infraestrutura de comunicação bidirecional para a categoria VSSS (Very Small Size Soccer). A arquitetura é composta por três camadas:
1. **Host (PC - Python):** Roda a inteligência artificial, processa pacotes via interface CFFI, gerencia a rede (DHCP customizado) e afere telemetria (Ping/RTT).
2. **Transmissor (TX - ESP32 Bridge):** Conectado via USB ao PC. Recebe dados via Serial (com cabeçalhos de sincronização) e converte para protocolo sem fio ESP-NOW via Broadcast. Mantém uma lista volátil (RAM) de robôs ativos.
3. **Robôs (RX - ESP32 Carrinhos):** Recebem comandos via ESP-NOW, validam remetente/destino, controlam motores via PID, lêem encoders e respondem chamadas de ECHO e PAREAMENTO.

---

## 2. Regras de Endereçamento (IDs)
O roteamento dos pacotes é lógico e não depende de pareamento fixo de MAC Address do ESP-NOW (todas as mensagens usam MAC de Broadcast `FF:FF:FF:FF:FF:FF`).
* **`0`**: ID exclusivo do Transmissor (PC/TX).
* **`1` a `6`**: IDs válidos dos robôs em campo.
* **`255`**: ID de Broadcast Lógico. Usado por robôs "virgens" (sem ID definido) ou pelo TX para enviar comandos globais (ex: PING global).

---

## 3. Estrutura de Dados (`Mensagens.h`)
O pacote de dados trafega de ponta a ponta (PC -> Ar -> Robô) usando a mesma struct binária empacotada (`__attribute__((packed))`). Tamanho fixo, validado na recepção.

**Campos principais:**
* `tipo`: Baseado no enum `TipoComando`.
* `indice_destino`: Quem deve processar o pacote.
* `indice_remetente`: Quem gerou o pacote (assina a mensagem).
* `payload`: Uma `union` contendo structs específicas para cada comando (`movimento`, `echo`, `pareamento`, `set_id`).

---

## 4. Comandos e Mecânicas Principais

### A. Auto-Discovery e Telemetria (`COMANDO_ECHO` = 0x03)
* **Fluxo:** O PC envia um Echo com destino `255` (Broadcast). Todos os robôs respondem para o destino `0`.
* **Carga:** O robô anexa seu próprio MAC Address e o `RSSI` (sinal) da mensagem recebida.
* **Ping (RTT):** O script Python afere o tempo exato (em milissegundos) da saída do comando até a chegada da resposta.

### B. DHCP Customizado (`COMANDO_SET_ID` = 0x04)
* **Problema:** Evitar IDs duplicados ou robôs presos no ID `255`.
* **Solução:** O Python analisa as respostas do `ECHO`. Se detectar conflito ou ID `255`, seleciona o primeiro ID livre (1 a 6) e despacha o `COMANDO_SET_ID` tendo como payload o **MAC Address** do robô alvo. O robô que reconhecer seu próprio MAC altera seu ID na memória Flash/EEPROM interna e responde com um novo ECHO confirmando.

### C. Pareamento (`COMANDO_PAREAMENTO` = 0x99)
* Autenticação via senha string. Faz o robô salvar o MAC do transmissor como uma "chave de segurança" para aceitar comandos de movimento, ignorando rádios invasores.

### D. Controle de Segurança (Whitelist em RAM no TX)
* Para poupar a Flash, o TX não salva configurações fixas. Ao ligar, ele aprende quem são os robôs da rede "escutando" as respostas de ECHO. Somente pacotes vindos de MACs conhecidos e pareados são repassados do TX para o PC (evitando injeção de dados pela serial).

---

## 5. Próximo Passo: Interface Gráfica (GUI)
*(Aviso para a IA atuante)*: O próximo marco de desenvolvimento é criar uma Interface Gráfica no PC (ex: PyQt, Tkinter ou DearPyGui) integrada a essa ponte Python. 

A GUI deverá consumir os dados do dicionário gerado pelo DHCP (`robos_ativos`) para exibir um **Painel de Sinal** contendo:
* MAC e ID atual do robô.
* Indicador visual de RTT (Ping em ms).
* Indicador visual de força de sinal (RSSI em dBm).
* Botões rápidos para "Enviar PING" e "Organizar Rede (DHCP)".