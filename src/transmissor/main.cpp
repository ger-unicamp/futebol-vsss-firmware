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

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *dados, int tamanho)
{
  // Envia os dados pela Serial para o Python usando o cabeçalho RX
  Serial.write(SYNC_1_RX);
  Serial.write(SYNC_2_RX);
  Serial.write((uint8_t)tamanho);
  Serial.write(dados, tamanho);
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("A iniciar o Transmissor...");

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_channel(CANAL, WIFI_SECOND_CHAN_NONE));
  esp_wifi_set_max_tx_power(84);

  if (esp_now_init() != ESP_OK)
    return;

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Falha ao adicionar o peer de broadcast");
  }

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

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