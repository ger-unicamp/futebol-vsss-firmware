#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "mensagem.h"

#define CANAL 11
#define LED 8

// Cabeçalhos de Sincronização (Python -> ESP32)
#define SYNC_1_TX 0xAA
#define SYNC_2_TX 0x55

// Cabeçalhos de Sincronização (ESP32 -> Python)
#define SYNC_1_RX 0xBB
#define SYNC_2_RX 0x66

typedef enum
{
  STATE_WAIT_SYNC1,
  STATE_WAIT_SYNC2,
  STATE_WAIT_LENGTH,
  STATE_READ_PAYLOAD
} estado_serial_t;

typedef struct
{
  uint8_t mac[6];
  bool ativo;
  uint32_t ultimo_contato;
} robo_confiavel_t;

// Fila para mensagens que chegam do rádio e devem ir para o PC
static QueueHandle_t fila_rx_pc;

// Variáveis da Máquina de Estados Serial
estado_serial_t estado_serial = STATE_WAIT_SYNC1;
uint8_t buffer_payload[250];
uint8_t tamanho_payload = 0;
uint8_t indice_buffer = 0;

// Variáveis de Timeout Serial unificadas
uint32_t ultimo_byte_rx_tempo = 0;
const uint32_t TIMEOUT_SERIAL_MS = 50;

volatile robo_confiavel_t lista_confianca[TAMANHO_VETOR_ROBOS] = {0};

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Busca linear com otimização de comparação reversa (MAC[5] -> MAC[0])
int buscar_mac(const uint8_t *mac_alvo)
{
  for (int i = 0; i < TAMANHO_VETOR_ROBOS; i++)
  {
    if (!lista_confianca[i].ativo)
      continue; // Pula slots vazios

    bool bateu = true;
    // Compara de trás para frente para falhar o mais rápido possível (OUI da Espressif é igual no início)
    for (int j = 5; j >= 0; j--)
    {
      if (lista_confianca[i].mac[j] != mac_alvo[j])
      {
        bateu = false;
        break; // Não é esse carrinho, sai do loop interno
      }
    }

    if (bateu)
      return i; // Achou! Retorna o índice
  }
  return -1; // Não encontrou em nenhum slot
}

// Registro com substituição por LRU (Menos Recentemente Usado)
void registrar_mac(const uint8_t *mac)
{
  if (buscar_mac(mac) != -1)
    return; // Já é confiável, não faz nada

  int pos_alvo = -1;
  uint32_t mais_antigo = 0xFFFFFFFF;

  // Varre a lista procurando um buraco ou o mais antigo
  for (int i = 0; i < TAMANHO_VETOR_ROBOS; i++)
  {
    if (!lista_confianca[i].ativo)
    {
      pos_alvo = i;
      break; // Achou espaço vazio, prioridade máxima, para a busca
    }
    // Se está ativo, verifica se é o contato mais velho até agora
    if (lista_confianca[i].ultimo_contato < mais_antigo)
    {
      mais_antigo = lista_confianca[i].ultimo_contato;
      pos_alvo = i;
    }
  }

  // Copia os 6 bytes do MAC para a posição alvo
  for (int j = 0; j < 6; j++)
  {
    lista_confianca[pos_alvo].mac[j] = mac[j];
  }
  lista_confianca[pos_alvo].ativo = true;
  lista_confianca[pos_alvo].ultimo_contato = millis();
}

void on_data_recv(const esp_now_recv_info_t *info_pacote, const uint8_t *dados, int tamanho)
{
  if (tamanho != sizeof(mensagem_t))
    return;

  mensagem_t pacote;
  memcpy(&pacote, dados, sizeof(mensagem_t));

  // Ignora se não for para o transmissor
  if (pacote.indice_destino != ID_TRANSMISSOR)
    return;

  uint8_t id = pacote.indice_remetente;

  // 1. Lógica de Pareamento
  if (pacote.tipo == COMANDO_PAREAMENTO)
  {
    if (strncmp(pacote.payload.pareamento.senha, SENHA_PAREAMENTO, sizeof(pacote.payload.pareamento.senha)) == 0)
      registrar_mac(info_pacote->src_addr); // Confiança estabelecida!
    else
      return; // Senha errada, ignora o intruso sumariamente!
  }
  // 2. Lógica de Comandos Normais (Validação estrita)
  else
  {
    int pos = buscar_mac(info_pacote->src_addr);
    if (pos == -1)
    {
      return; // Desconhecido fingindo ser o robô!
    }

    // É de confiança. Atualiza o relógio de sobrevivência na tabela
    lista_confianca[pos].ultimo_contato = millis();
  }

  if (pacote.tipo == COMANDO_TELEMETRIA)
  {
    // RSSI e SNR medidos pelo receptor (Transmissor) agora
    pacote.payload.telemetria.rssi_transmissor = info_pacote->rx_ctrl->rssi;
    pacote.payload.telemetria.noise_floor_transmissor = info_pacote->rx_ctrl->noise_floor;
  }

  // Se passou por tudo, repassa a struct via Serial para o Python ler o MAC
  xQueueSend(fila_rx_pc, &pacote, 0);
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("A iniciar o Transmissor...");

  // Configura o Wi-Fi e o ESP-NOW
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_channel(CANAL, WIFI_SECOND_CHAN_NONE));
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84));
  if (esp_now_init() != ESP_OK)
    return;

  fila_rx_pc = xQueueCreate(15, sizeof(mensagem_t));

  if (fila_rx_pc == NULL)
  {
    Serial.println("Erro ao criar a fila RX!");
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(on_data_recv)); // Registra a função de callback para receber dados via ESP-NOW

  // configura o peer de broadcast
  esp_now_peer_info_t peer_info;
  memset(&peer_info, 0, sizeof(peer_info));
  memcpy(peer_info.peer_addr, broadcastAddress, 6);
  if (esp_now_add_peer(&peer_info) != ESP_OK)
  {
    Serial.println("Falha ao adicionar o peer de broadcast");
  }

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  Serial.println("Tamanho da mensagem: " + String(sizeof(mensagem_t)) + " bytes");
  Serial.println("Transmissor iniciado com sucesso!");
  // Envio de pareamento automático para popular a lista de robos
}

void loop()
{
  mensagem_t msg_para_pc;
  if (xQueueReceive(fila_rx_pc, &msg_para_pc, 0) == pdTRUE)
  {
    // Agora sim, em segurança fora da interrupção, enviamos para o Serial
    Serial.write(SYNC_1_RX);
    Serial.write(SYNC_2_RX);
    Serial.write(sizeof(mensagem_t));
    Serial.write((uint8_t *)&msg_para_pc, sizeof(mensagem_t));
  }

  if (estado_serial != STATE_WAIT_SYNC1)
  {
    if (millis() - ultimo_byte_rx_tempo > TIMEOUT_SERIAL_MS)
    {
      estado_serial = STATE_WAIT_SYNC1;
      indice_buffer = 0;
      tamanho_payload = 0;
    }
  }

  // Lê a porta Serial montando a mensagem guiada pelo cabeçalho
  while (Serial.available() > 0)
  {
    uint8_t byte_lido = Serial.read();
    ultimo_byte_rx_tempo = millis();
    switch (estado_serial)
    {
    case STATE_WAIT_SYNC1:
      if (byte_lido == SYNC_1_TX)
        estado_serial = STATE_WAIT_SYNC2;
      break;

    case STATE_WAIT_SYNC2:
      if (byte_lido == SYNC_2_TX)
        estado_serial = STATE_WAIT_LENGTH;
      else
        estado_serial = STATE_WAIT_SYNC1; // Falso positivo, reseta
      break;

    case STATE_WAIT_LENGTH:
      tamanho_payload = byte_lido;
      indice_buffer = 0;
      // Previne buffers zerados ou maiores que o ESP-NOW suporta
      if (tamanho_payload > 0 && tamanho_payload <= 250)
      {
        estado_serial = STATE_READ_PAYLOAD;
      }
      else
      {
        estado_serial = STATE_WAIT_SYNC1; // Tamanho inválido, reseta
      }
      break;

    case STATE_READ_PAYLOAD:
      buffer_payload[indice_buffer++] = byte_lido;

      // Se leu todos os bytes esperados, joga pro ar!
      if (indice_buffer >= tamanho_payload)
      {
        esp_now_send(broadcastAddress, buffer_payload, tamanho_payload);
        estado_serial = STATE_WAIT_SYNC1; // Reseta a máquina de estados para a próxima msg
      }
      break;
    }
  }
}