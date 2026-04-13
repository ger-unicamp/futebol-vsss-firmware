#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "Mensagens.h"
#include "ControleMotor.h"
#include "LeitorEncoder.h"

#define DEBUG
#define MY_INDEX 0
#define CANAL 11

#define PH_IN1 2
#define PH_IN2 2
#define PH_IN3 4
#define PH_IN4 3

#define ENC0_PINA 0
#define ENC0_PINB 5
#define ENC1_PINA 6
#define ENC1_PINB 7

#define FREQ_ATT 60
#define DELAY 1000/FREQ_ATT
#define TTL_TIME 100

uint8_t mac_esp_principal[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
esp_now_peer_info_t peerInfo;

Motor motor0 = {PH_IN1, PH_IN2, 0, 0};
Motor motor1 = {PH_IN3, PH_IN4, 0, 0};
Encoder encoder0 = {ENC0_PINA, ENC0_PINB, 0, 0, 0.0};
Encoder encoder1 = {ENC1_PINA, ENC1_PINB, 0, 0, 0.0};

Mensagem msg;
Mensagem_serial msg_serial;

unsigned long millis_ttl = 0;
unsigned long millis_att = 0;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&msg, incomingData, sizeof(msg));
  if (strcmp(msg.checkger, "GERVSS") == 0 && msg.indice_carrinho == MY_INDEX) {
      motor0.velocidadeAlvo = msg.velocidadeMotores[0];
      motor1.velocidadeAlvo = msg.velocidadeMotores[1];
      millis_ttl = millis();
  }
}

void IRAM_ATTR isr_encoder0() { logicaEncoder(&encoder0); }
void IRAM_ATTR isr_encoder1() { logicaEncoder(&encoder1); }

void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  pinMode(motor0.pin0, OUTPUT);
  pinMode(motor0.pin1, OUTPUT);
  pinMode(motor1.pin0, OUTPUT);
  pinMode(motor1.pin1, OUTPUT);

  pinMode(encoder0.pinA, INPUT_PULLUP);
  pinMode(encoder0.pinB, INPUT_PULLUP);
  pinMode(encoder1.pinA, INPUT_PULLUP);
  pinMode(encoder1.pinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder0.pinA), isr_encoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder1.pinA), isr_encoder1, RISING);

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_channel(CANAL, WIFI_SECOND_CHAN_NONE));

  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  millis_ttl = millis();
  millis_att = millis();
}

void loop() {
  unsigned long millis_atual = millis();

  if (millis_atual - millis_att > DELAY) {
    atualizarVelocidade(&motor0);
    atualizarVelocidade(&motor1);
    millis_att = millis_atual;
  }

  if (millis_atual - millis_ttl > TTL_TIME) {
    motor0.velocidadeAlvo = 0;
    motor1.velocidadeAlvo = 0;
    millis_ttl = millis_atual;
  }
} 