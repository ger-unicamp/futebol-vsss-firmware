# Firmware do Transmissor (TX)

O transmissor atua como uma ponte (bridge) de comunicação entre o computador (via Serial/USB) e os robôs em campo (via ESP-NOW).

## Funcionalidades

- **Ponte Serial -> ESP-NOW:** Recebe pacotes binários do PC, valida a sincronização e os retransmite via broadcast ESP-NOW.
- **Ponte ESP-NOW -> Serial:** Recebe respostas dos robôs (Prints, Telemetria) e as envia para o PC via Serial com cabeçalhos específicos.
- **Gestão de Lista de Confiança:** Mantém em memória RAM uma lista de robôs pareados e ativos, ignorando mensagens de dispositivos desconhecidos.
- **Injeção de Metadados:** Adiciona informações de RSSI e ruído local aos pacotes de telemetria antes de enviá-los ao PC.

## Protocolo Serial

A comunicação com o PC utiliza um cabeçalho de sincronização (`0x0F 0xF0`) seguido pelo tamanho do pacote e o payload (struct `mensagem_t`).

## Fluxo de Operação

1. **Inicialização:** Configura o rádio no mesmo canal dos robôs.
2. **Loop Principal:** 
   - Monitora a porta Serial em busca de novos comandos.
   - Dequeues mensagens recebidas via rádio de uma fila do FreeRTOS e as envia para o Serial.
3. **Segurança:** Apenas robôs que realizaram o processo de pareamento (`COMANDO_PAREAMENTO`) com a senha correta são adicionados à lista de confiança e têm suas mensagens repassadas ao PC.
