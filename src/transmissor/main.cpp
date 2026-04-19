#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "Mensagens.h"

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
} EstadoSerial;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;
Mensagem msg;
EstadoSerial estado_serial = STATE_WAIT_SYNC1;
uint8_t tamanho_payload = 0;
uint8_t buffer_payload[250]; // 250 é o limite seguro do ESP-NOW
uint8_t indice_buffer = 0;

uint8_t mac_robos[MAX_ROBOS + 1][6];       // Guarda os MACs (Índices 1 a 6)
bool robo_online[MAX_ROBOS + 1] = {false}; // Status de conexão

void OnDataRecv(const esp_now_recv_info_t *info_pacote, const uint8_t *dados, int tamanho)
{
  if (tamanho != sizeof(Mensagem))
    return;

  Mensagem pacote;
  memcpy(&pacote, dados, sizeof(Mensagem));

  // Ignora se não for para o transmissor
  if (pacote.indice_destino != ID_TRANSMISSOR)
    return;

  uint8_t id = pacote.indice_remetente;

  // Se o carrinho enviou Pareamento ou Echo, atualizamos a lista de confiáveis na RAM!
  if (pacote.tipo == COMANDO_PAREAMENTO || pacote.tipo == COMANDO_ID)
  {
    if (id > 0 && id <= MAX_ROBOS)
    {
      memcpy(mac_robos[id], info_pacote->src_addr, 6);
      robo_online[id] = true;
    }
  }
  else
  {
    // Se for outro comando (ex: resposta de telemetria), valida a segurança
    if (id == 0 || (id > MAX_ROBOS && id != 255) || !robo_online[id])
      return;

    for (int i = 0; i < 6; i++)
    {
      if (info_pacote->src_addr[i] != mac_robos[id][i] && id != 255) // Se o MAC não bater com o que temos registrado para aquele ID (e não for broadcast), ignora
        return;                                                      // Desconhecido fingindo ser o robô!
    }
  }

  if (pacote.tipo == COMANDO_TELEMETRIA)
  {
    // RSSI e SNR medidos pelo receptor (Transmissor) agora
    pacote.payload.telemetria.rssi_transmissor = info_pacote->rx_ctrl->rssi;
    pacote.payload.telemetria.noise_floor_transmissor = info_pacote->rx_ctrl->noise_floor;

    // Atualizamos o buffer de dados com os novos valores injetados
    memcpy((void *)dados, &pacote, sizeof(Mensagem));
  }

  // Se passou por tudo, repassa a struct via Serial para o Python ler o MAC
  Serial.write(SYNC_1_RX);
  Serial.write(SYNC_2_RX);
  Serial.write(tamanho);
  Serial.write(dados, tamanho);
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

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv)); // Registra a função de callback para receber dados via ESP-NOW

  // configura o peer de broadcast
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Falha ao adicionar o peer de broadcast");
  }

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  Serial.println("Tamanho da mensagem: " + String(sizeof(Mensagem)) + " bytes");
  Serial.println("Transmissor iniciado com sucesso!");
}

void loop()
{
  // Lê a porta Serial montando a mensagem guiada pelo cabeçalho
  while (Serial.available() > 0)
  {
    uint8_t byte_lido = Serial.read();

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