#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "Mensagens.h"
#include "Motor.h"
#include "Encoder.h"
#include "Memoria.h"
#include "PID.h"

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

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
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

// Instancia os PIDs para cada roda
PidConfig pidMotor0;
PidConfig pidMotor1;

// Variáveis alvo (Ticks por ciclo de controle de 16ms)
float target_ticks_0 = 0; 
float target_ticks_1 = 0;

void OnDataRecv(const esp_now_recv_info_t *info_pacote, const uint8_t *dados, int tamanho)
{
  // Verificação básica de tamanho
  if (tamanho != sizeof(Mensagem))
    return;
  u_int8_t *mac = info_pacote->src_addr;
  Mensagem pacote;
  memcpy(&pacote, dados, sizeof(pacote));

  // ---------------------------------------------------------
  // 1. TRATAMENTO DO COMANDO DE PAREAMENTO
  // ---------------------------------------------------------
  if (pacote.tipo == COMANDO_PAREAMENTO && pacote.indice_remetente == ID_TRANSMISSOR)
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
    {
    target_ticks_0 = pacote.payload.movimento.vel_esq;
    target_ticks_1 = pacote.payload.movimento.vel_dir;
    millis_ttl = millis();
    break;
    }
  case COMANDO_MOVIMENTO_GLOBAL:
    if (memoria.indice < MAX_ROBOS)
    {
      target_ticks_0 = pacote.payload.movimento_global.vel_esq[memoria.indice];
      target_ticks_1 = pacote.payload.movimento_global.vel_dir[memoria.indice];
      millis_ttl = millis();
    }
    break;
  case COMANDO_ECHO:
  {
    if(pacote.indice_destino != ID_BROADCAST && pacote.indice_destino != memoria.indice)
      return; // Se o comando de echo não for para broadcast nem para mim, ignora

    // Prepara uma mensagem de texto simples avisando que está vivo
    Mensagem resposta;
    // snprintf(resposta, sizeof(resposta), "ECHO_OK - Carrinho ID: %d\n", memoria.indice);
    resposta.indice_remetente = memoria.indice; // Pode ser útil para o transmissor identificar quem respondeu
    resposta.tipo = COMANDO_ECHO; // Define o tipo como ECHO para o transmissor reconhecer a resposta
    resposta.indice_destino = ID_TRANSMISSOR; // Responde diretamente para o transmissor
    resposta.payload.echo.rssi = (uint8_t) info_pacote->rx_ctrl->rssi; // Inclui o RSSI da mensagem recebida como parte do payload
    esp_now_send(broadcastAddress,(u_int8_t*) &resposta, sizeof(resposta)); // Envia 
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
    uint8_t mac_novo[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    memcpy(memoria.mac_esp_principal, mac_novo, 6);
    memoria.indice = 255;
    memoria.pid0 = {2.5f, 0.8f, 0.01f, 5.0f}; 
    memoria.pid1 = {2.5f, 0.8f, 0.01f, 5.0f};
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
  motor0 = criarMotor(PH_IN1, PH_IN2);
  motor1 = criarMotor(PH_IN3, PH_IN4);
  tocarSomMotor(&motor0, 8000, 500);
  tocarSomMotor(&motor1, 8000, 500);

  // Configura os encoders e as interrupções
  inicializarEncoder(&encoder0, ENC0_PINA, ENC0_PINB);
  inicializarEncoder(&encoder1, ENC1_PINA, ENC1_PINB);
  attachInterrupt(digitalPinToInterrupt(encoder0.pinA), isr0, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder1.pinA), isr1, RISING);

  // Limites do PWM: no header Motor.h
  // Configura com os valores guardados na memoria
  pid_iniciar(&pidMotor0, memoria.pid0.kp, memoria.pid0.ki, memoria.pid0.kd, memoria.pid0.kf, PID_MIN_PWM, PID_MAX_PWM);
  pid_iniciar(&pidMotor1, memoria.pid1.kp, memoria.pid1.ki, memoria.pid1.kd, memoria.pid1.kf, PID_MIN_PWM, PID_MAX_PWM);

  // Configura o Wi-Fi e o ESP-NOW
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

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv)); // Registra a função de callback para receber dados via ESP-NOW

  // adiciona o peer de broadcast para garantir que pode enviar respostas para o transmissor
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Falha ao adicionar o peer de broadcast");
  }

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
    // 1. Atualiza a leitura de Variação (Delta) do Encoder
    atualizarDeltaTicks(&encoder0);
    atualizarDeltaTicks(&encoder1);

    // 2. Calcula o PID (saída será um valor de PWM entre -1023 e 1023)
    float pwm_saida_0 = pid_computar(&pidMotor0, target_ticks_0, encoder0.delta_ticks);
    float pwm_saida_1 = pid_computar(&pidMotor1, target_ticks_1, encoder1.delta_ticks);

    // 3. Aplica nos motores (assumindo que a sua struct Motor ou biblioteca espera valor com sinal)
    moverMotor(&motor0, (int) pwm_saida_0);
    moverMotor(&motor1, (int) pwm_saida_1);

    millis_att = millis_atual;
  }

  if (millis_atual - millis_ttl > TTL_TIME)
  {
    target_ticks_0 = 0;
    target_ticks_1 = 0;
    millis_ttl = millis_atual;
  }
}