#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "Mensagens.h"
#include "Motor.h"
#include "Encoder.h"
#include "Memoria.h"

#define DEBUG
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
#define DELAY 1000 / FREQ_ATT
#define TTL_TIME 100

uint8_t mac_esp_principal;
esp_now_peer_info_t peerInfo;

Motor motor0 = {0};
Motor motor1 = {0};
Encoder encoder0 = {0};
Encoder encoder1 = {0};

Mensagem msg;
bool pareado = false;

unsigned long millis_ttl = 0;
unsigned long millis_att = 0;

DadosConfig memoria;

CRIAR_ISR_ENCODER(isr0, encoder0)
CRIAR_ISR_ENCODER(isr1, encoder1)

// Função auxiliar para garantir que o rádio está na lista de transmissores permitidos
void registrar_peer_radio(const uint8_t *mac_radio)
{
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo)); // Zera a memória da struct
  memcpy(peerInfo.peer_addr, mac_radio, 6);
  peerInfo.channel = CANAL;
  peerInfo.encrypt = false;

  // Só adiciona se o peer ainda não existir
  if (!esp_now_is_peer_exist(mac_radio))
  {
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
      Serial.println("Falha ao registrar o rádio como Peer para envio.");
    }
  }
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len)
{
  // Verificação básica de tamanho
  if (len != sizeof(Mensagem))
    return;
  u_int8_t *mac = recv_info->src_addr;
  Mensagem pacote;
  memcpy(&pacote, incomingData, sizeof(pacote));

  // ---------------------------------------------------------
  // 1. TRATAMENTO DO COMANDO DE PAREAMENTO
  // ---------------------------------------------------------
  if (pacote.tipo == COMANDO_PAREAMENTO)
  {
    // Compara a senha recebida com a senha hardcoded
    if (strncmp(pacote.payload.pareamento.senha, SENHA_PAREAMENTO, sizeof(pacote.payload.pareamento.senha)) == 0)
    {

      // Senha correta! Copia o MAC do remetente para a struct da memória
      memcpy(memoria.mac_esp_principal, mac, 6);

      // Salva na memória flash usando sua função do Memoria.cpp
      if (salvar_config(&memoria))
      {
        pareado = true;
        Serial.println("Pareamento concluído com sucesso e salvo na memória!");
      }
      else
      {
        Serial.println("Erro ao tentar salvar o MAC na memória.");
      }
    }
    return; // Para a execução aqui, pois o comando era só para parear
  }

  // ---------------------------------------------------------
  // 2. FILTRO DE SEGURANÇA PARA COMANDOS COMUNS (MOVIMENTO)
  // ---------------------------------------------------------
  // Se ainda não estiver pareado, ignora qualquer outro comando
  if (!pareado)
    return;

  // Verifica se o MAC de quem enviou é o mesmo que está salvo na memória
  for (int i = 0; i < 6; i++)
  {
    if (mac[i] != memoria.mac_esp_principal[i])
    {
      return; // MAC não autorizado, ignora o pacote
    }
  }

  // ---------------------------------------------------------
  // 3. EXECUÇÃO DOS COMANDOS AUTORIZADOS
  // ---------------------------------------------------------
  switch (pacote.tipo)
  {
  case COMANDO_MOVIMENTO:
    // motor0.velocidadeAlvo = pacote.payload.movimento.vel_esq;
    // motor1.velocidadeAlvo = pacote.payload.movimento.vel_dir;
    millis_ttl = millis();
    break;

  case COMANDO_MOVIMENTO_GLOBAL:
    if (memoria.indice < MAX_ROBOS)
    {
      // motor0.velocidadeAlvo = pacote.payload.movimento_global.vel_esq[memoria.indice];
      // motor1.velocidadeAlvo = pacote.payload.movimento_global.vel_dir[memoria.indice];
      millis_ttl = millis();
    }
    break;
  case COMANDO_ECHO:
  {
    // Prepara uma mensagem de texto simples avisando que está vivo
    char resposta[50];
    snprintf(resposta, sizeof(resposta), "ECHO_OK - Carrinho ID: %d\n", memoria.indice);

    // Envia de volta EXATAMENTE para o MAC do rádio salvo na memória
    esp_now_send(memoria.mac_esp_principal, (uint8_t *)resposta, strlen(resposta) + 1); // +1 para incluir o \0 final
    break;
  }
  }
}

void IRAM_ATTR isr_encoder0() { logicaEncoder(&encoder0); }
void IRAM_ATTR isr_encoder1() { logicaEncoder(&encoder1); }

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("A inicializar o carrinho...");

  // inicia memoria e tenta carregar o MAC do rádio principal para pareamento automático
  if (inicializar_memoria() && carregar_config(&memoria))
  {
    pareado = true;
    Serial.println("Dados carregados!");
  }
  else
  {
    pareado = false;
    Serial.println("Falha na Flash ou Configuração não encontrada. Definindo novos valores...");
    uint8_t mac_novo[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(memoria.mac_esp_principal, mac_novo, 6);
    memoria.indice = 99;
    if (salvar_config(&memoria))
    {
      Serial.println("Dados salvos com sucesso!");
    }
    else
    {
      Serial.println("Falha ao salvar.");
    }
  }
  Serial.printf("MAC do transmissor: %02X:%02X:%02X:%02X:%02X:%02X\n",
                memoria.mac_esp_principal[0], memoria.mac_esp_principal[1],
                memoria.mac_esp_principal[2], memoria.mac_esp_principal[3],
                memoria.mac_esp_principal[4], memoria.mac_esp_principal[5]);
  Serial.printf("ID: %d\n", memoria.indice);
  Serial.println("Pareado: " + String(pareado));

  // Configura os motores
  motor0 = criarMotor(PH_IN1, PH_IN2, 0);
  motor1 = criarMotor(PH_IN3, PH_IN4, 1);
  tocarSomMotor(&motor0, 1000, 500);
  tocarSomMotor(&motor1, 1000, 500);

  // Configura os encoders e as interrupções
  inicializarEncoder(&encoder0, ENC0_PINA, ENC0_PINB);
  inicializarEncoder(&encoder1, ENC1_PINA, ENC1_PINB);
  attachInterrupt(digitalPinToInterrupt(encoder0.pinA), isr0, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder1.pinA), isr1, RISING);

  // Configura o Wi-Fi e o ESP-NOW
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_channel(CANAL, WIFI_SECOND_CHAN_NONE));
  if (esp_now_init() != ESP_OK)
    return;
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // Incicializa os timers
  millis_ttl = millis();
  millis_att = millis();
  Serial.println("Carrinho iniciado com sucesso!");
}

void loop()
{
  unsigned long millis_atual = millis();

  if (millis_atual - millis_att > DELAY)
  {
    // atualizarVelocidade(&motor0);
    // atualizarVelocidade(&motor1);
    millis_att = millis_atual;
  }

  if (millis_atual - millis_ttl > TTL_TIME)
  {
    // motor0.velocidadeAlvo = 0;
    // motor1.velocidadeAlvo = 0;
    millis_ttl = millis_atual;
  }
}