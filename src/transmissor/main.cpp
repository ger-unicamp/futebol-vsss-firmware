#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "Mensagens.h"

#define CANAL 11
#define LED 8
#define NUMERO_MAX_CARRINHOS 6

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;

Mensagem msg;
Mensagem_serial msg_serial;
uint8_t buffer_serial[10];

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  char buffer[100];
  memcpy(&buffer, incomingData, sizeof(buffer));
  Serial.print(buffer);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("A iniciar o Rádio...");

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_channel(CANAL, WIFI_SECOND_CHAN_NONE));
  esp_wifi_set_max_tx_power(84);

  if (esp_now_init() != ESP_OK) return;

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Falha ao adicionar o peer de broadcast");
  }

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  /* Manda todos os carrinhos pararem ao iniciar */
  for (int i = 0; i < NUMERO_MAX_CARRINHOS; i++) {
    strcpy(msg.checkger, "GERVSS");
    msg.indice_carrinho = i;
    msg.velocidadeMotores[0] = 0;
    msg.velocidadeMotores[1] = 0;
    esp_now_send(broadcastAddress, (uint8_t *) &msg, sizeof(msg));
  }
}

void loop() {
  int n = Serial.readBytesUntil(128, buffer_serial, sizeof(buffer_serial));
  
  if (n) {
    memcpy(&msg_serial, buffer_serial, sizeof(msg_serial));
    
    if (msg_serial.indice_carrinho < NUMERO_MAX_CARRINHOS) {
      strcpy(msg.checkger, "GERVSS");
      msg.indice_carrinho = msg_serial.indice_carrinho;
      msg.velocidadeMotores[0] = (int) ((msg_serial.velocidades[0]) << 1);
      msg.velocidadeMotores[1] = (int) ((msg_serial.velocidades[1]) << 1);
      
      esp_now_send(broadcastAddress, (uint8_t *) &msg, sizeof(msg));
      
      digitalWrite(LED, HIGH);
      delay(3);
      digitalWrite(LED, LOW);
    }
  }
}