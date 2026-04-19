#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <stdarg.h>

#include "Mensagens.h"
#include "Motor.h"
#include "Encoder.h"
#include "Memoria.h"
#include "PID.h"

// Comente esta linha ou mude para 0 para desativar o modo Debug
#define DEBUG_ENABLE

#ifdef DEBUG_ENABLE
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#define DEBUG_BEGIN(x) Serial.begin(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(...)
#define DEBUG_BEGIN(x)
#endif

// Canal do Wifi
#define CANAL 11

// conexoes dos motores e encoders
#define PH_IN1 2
#define PH_IN2 1
#define PH_IN3 3
#define PH_IN4 4

#define ENC0_PINA 7
#define ENC0_PINB 6
#define ENC1_PINA 0
#define ENC1_PINB 5

// Estrutura para guardar os parâmetros na memoria Flash
DadosConfig memoria;

// Variaveis de rede e esp now
uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Nunca muda por isso hardcoded
uint8_t ultimo_rssi;
uint8_t ultimo_noise_floor;
uint32_t cnt_pacotes_totais;
uint32_t cnt_pacotes_validos;

// Variáveis de controle de Tasks
bool periodo_ttl_ativo;
uint32_t millis_atual;
uint32_t millis_ttl;
uint32_t millis_telemetria;

typedef float (*FuncaoControle)(int i_roda, float setpoint, float medido);

Motor motor[2];
Encoder encoder[2];
PidConfig pidMotor[2];
float delta_ticks_target[2];
PidAutotune tuneMotor[2];
FuncaoControle calcular_motor[2];

CRIAR_ISR_ENCODER(isr0, encoder[0])
CRIAR_ISR_ENCODER(isr1, encoder[1])

float controle_pid(int i_roda, float setpoint, float medido)
{
  return pid_computar(&pidMotor[i_roda], setpoint, medido);
}

float controle_autotune(int i_roda, float setpoint, float medido)
{
  float pwm = autotune_computar(&tuneMotor[i_roda], &pidMotor[i_roda], setpoint, medido);

  // Se o autotune acabou nesta iteração:
  if (!tuneMotor[i_roda].ativo)
  {
    DEBUG_PRINTLN("Autotune Motor 0 Finalizado!");

    // 1. Devolve o controle pro PID normal!
    calcular_motor[i_roda] = controle_pid;
    periodo_ttl_ativo = true;

    // 2. Envia a mensagem de aviso pro Python
    Mensagem msgFim;
    msgFim.tipo = COMANDO_PID;
    msgFim.indice_destino = ID_TRANSMISSOR;
    msgFim.indice_remetente = memoria.indice;
    msgFim.payload.pidconfig.roda = i_roda;
    msgFim.payload.pidconfig.kp = pidMotor[i_roda].kp;
    msgFim.payload.pidconfig.ki = pidMotor[i_roda].ki;
    msgFim.payload.pidconfig.kd = pidMotor[i_roda].kd;
    msgFim.payload.pidconfig.kf = pidMotor[i_roda].kf;

    memoria.pid[i_roda].kp = pidMotor[i_roda].kp;
    memoria.pid[i_roda].ki = pidMotor[i_roda].ki;
    memoria.pid[i_roda].kd = pidMotor[i_roda].kd;
    memoria.pid[i_roda].kf = pidMotor[i_roda].kf;
    esp_now_send(broadcastAddress, (uint8_t *)&msgFim, sizeof(msgFim));
  }

  return pwm;
}

// Função para enviar mensagens de texto de debug formatadas para o Python via ESP-NOW
void esp_printf(const char *formato, ...)
{
  Mensagem msg;
  msg.tipo = COMANDO_PRINT;
  msg.indice_destino = ID_TRANSMISSOR; // Envia para o PC
  msg.indice_remetente = memoria.indice;

  // Lógica para formatar a string (como um printf)
  va_list args;
  va_start(args, formato);
  vsnprintf(msg.payload.print.texto, sizeof(msg.payload.print.texto), formato, args);
  va_end(args);

  // Envia via ESP-NOW
  esp_now_send(broadcastAddress, (uint8_t *)&msg, sizeof(msg));
}

// Callback para receber mensagens via ESP-NOW
void OnDataRecv(const esp_now_recv_info_t *info_pacote, const uint8_t *dados, int tamanho)
{
  cnt_pacotes_totais = cnt_pacotes_totais + 1;
  // Verificação básica de tamanho
  if (tamanho != sizeof(Mensagem))
    return; // tamanho inesperado, ignora o pacote$

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
      cnt_pacotes_validos = cnt_pacotes_validos + 1;

      // Senha correta! Copia o MAC do remetente para a struct da memória (apenas na RAM)
      memcpy(memoria.mac_esp_principal, info_pacote->src_addr, 6);

      // Habilita a comunicação sem gastar a memória Flash
      memoria.pareado = true;
      DEBUG_PRINTLN("Pareamento concluído na RAM! Lembre-se de usar o COMANDO_SALVAR para persistir.");

      // Responde ao Python
      Mensagem resposta;
      resposta.tipo = COMANDO_PAREAMENTO;
      resposta.indice_destino = ID_TRANSMISSOR;
      resposta.indice_remetente = memoria.indice;
      esp_wifi_get_mac(WIFI_IF_STA, resposta.payload.pareamento.mac); // Coleta o próprio MAC
      delay(100);
      DEBUG_PRINTF("Respondendo para o transmissor com meu MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   resposta.payload.pareamento.mac[0], resposta.payload.pareamento.mac[1],
                   resposta.payload.pareamento.mac[2], resposta.payload.pareamento.mac[3],
                   resposta.payload.pareamento.mac[4], resposta.payload.pareamento.mac[5]);
      esp_now_send(broadcastAddress, (u_int8_t *)&resposta, sizeof(resposta));
    }
    return; // Para a execução aqui, pois o comando era só para parear
  }

  // ---------------------------------------------------------
  // 2. FILTRO DE SEGURANÇA PARA COMANDOS COMUNS (MOVIMENTO)
  // ---------------------------------------------------------
  // Se ainda não estiver pareado, ignora qualquer outro comando
  if (!memoria.pareado)
    return; // Se não estiver pareado, ignora qualquer comando que não seja de pareamento

  if (pacote.indice_remetente != ID_TRANSMISSOR)
    return; // Se o comando não veio do transmissor, ignora

  if (pacote.indice_destino != memoria.indice && pacote.indice_destino != ID_BROADCAST)
    return; // Se o comando não for para mim nem para broadcast, ignora

  // Verifica se o MAC de quem enviou é o mesmo que está salvo na memória
  for (int i = 0; i < 6; i++)
  {
    if (info_pacote->src_addr[i] != memoria.mac_esp_principal[i])
    {
      return; // MAC não autorizado, ignora o pacote
    }
  }

  cnt_pacotes_validos = cnt_pacotes_validos + 1; // Se passou por tudo, é um pacote válido!

  // ---------------------------------------------------------
  // 3. EXECUÇÃO DOS COMANDOS AUTORIZADOS
  // ---------------------------------------------------------

  if (memoria.periodo_telemetria_ms) // se a telemetria estiver ativada, guarda o rssi e snr do transmissor e do carrinho para enviar junto com a telemetria na próxima vez
  {
    ultimo_rssi = info_pacote->rx_ctrl->rssi;
    ultimo_noise_floor = info_pacote->rx_ctrl->noise_floor;
  }

  switch (pacote.tipo)
  {
  case COMANDO_MOVIMENTO:
  {
    for (int i = 0; i < 2; i++)
      delta_ticks_target[i] = pacote.payload.movimento.target_ticks[i];
    millis_ttl = millis();
    periodo_ttl_ativo = true;
    break;
  }
  case COMANDO_MOVIMENTO_GLOBAL:
  {
    if (memoria.indice < MAX_ROBOS)
    {
      for (int i = 0; i < 2; i++)
        delta_ticks_target[i] = pacote.payload.movimento_global.target_ticks[i][memoria.indice];
      millis_ttl = millis();
      periodo_ttl_ativo = true;
    }
    break;
  }
  case COMANDO_ID:
  {
    if (pacote.is_set)
    {
      // Pega o próprio MAC físico para comparar
      uint8_t meu_mac[6];
      esp_wifi_get_mac(WIFI_IF_STA, meu_mac);
      bool mac_eh_meu = true;
      for (int i = 0; i < 6; i++)
      {
        if (pacote.payload.set_id.mac_alvo[i] != meu_mac[i])
        {
          mac_eh_meu = false;
          break;
        }
      }

      // Se o comando foi direcionado para o MAC deste carrinho físico
      if (mac_eh_meu)
      {
        memoria.indice = pacote.payload.set_id.novo_id; // Atualiza o ID
      }
    }
    // Prepara uma mensagem de texto simples avisando que está vivo
    Mensagem resposta;
    DEBUG_PRINTF("ECHO_OK - Carrinho ID: %d\n", memoria.indice);
    resposta.indice_remetente = memoria.indice;                              // Pode ser útil para o transmissor identificar quem respondeu
    resposta.tipo = COMANDO_ID;                                              // Define o tipo como ECHO para o transmissor reconhecer a resposta
    resposta.indice_destino = ID_TRANSMISSOR;                                // Responde diretamente para o transmissor
    resposta.payload.echo.rssi = (uint8_t)info_pacote->rx_ctrl->rssi;        // Inclui o RSSI da mensagem recebida como parte do payload
    esp_wifi_get_mac(WIFI_IF_STA, resposta.payload.echo.mac);                // Coleta o próprio MAC para enviar de volta
    esp_now_send(broadcastAddress, (u_int8_t *)&resposta, sizeof(resposta)); // Envia
    break;
  }
  case COMANDO_PID:
  {
    if (pacote.is_set)
    {
      int i_roda = pacote.payload.pidconfig.roda;
      // muda valores na Memoria
      memoria.pid[i_roda].kp = pacote.payload.pidconfig.kp;
      memoria.pid[i_roda].ki = pacote.payload.pidconfig.ki;
      memoria.pid[i_roda].kd = pacote.payload.pidconfig.kd;
      memoria.pid[i_roda].kf = pacote.payload.pidconfig.kf;

      // muda os valores na RAM
      pidMotor[i_roda].kp = memoria.pid[i_roda].kp;
      pidMotor[i_roda].ki = memoria.pid[i_roda].ki;
      pidMotor[i_roda].kd = memoria.pid[i_roda].kd;
      pidMotor[i_roda].kf = memoria.pid[i_roda].kf;
      pid_resetar(&pidMotor[i_roda]); // Zera o erro acumulado para não dar tranco
    }
    // manda duas mensagens de resposta, uma para cada roda, com os parâmetros atuais
    Mensagem resposta;
    resposta.tipo = COMANDO_PID;
    resposta.indice_destino = ID_TRANSMISSOR;
    resposta.indice_remetente = memoria.indice;
    for (int i = 0; i < 2; i++)
    {
      resposta.payload.pidconfig.roda = i;
      resposta.payload.pidconfig.kp = pidMotor[i].kp;
      resposta.payload.pidconfig.ki = pidMotor[i].ki;
      resposta.payload.pidconfig.kd = pidMotor[i].kd;
      resposta.payload.pidconfig.kf = pidMotor[i].kf;
      esp_now_send(broadcastAddress, (uint8_t *)&resposta, sizeof(resposta));
    }
    break;
  }
  case COMANDO_AUTOTUNE_PID:
  {
    periodo_ttl_ativo = false; // Desativa o TTL para não zerar os alvos durante o autotune
    int i_roda = pacote.payload.autotune.roda;
    autotune_iniciar(&tuneMotor[i_roda], pacote.payload.autotune.pwm_teste_max, pacote.payload.autotune.pwm_teste_min, pacote.payload.autotune.ciclos);
    delta_ticks_target[i_roda] = pacote.payload.autotune.target_ticks; // Define o alvo de ticks para o autotune
    calcular_motor[i_roda] = controle_autotune;                        // Muda a função
    break;
  }
  case COMANDO_SALVAR:
  {
    if (salvar_config(&memoria))
    {
      DEBUG_PRINTLN("Parâmetros de PID salvos na flash com sucesso!");
      esp_printf("Salvo!");
    }
    else
    {
      DEBUG_PRINTLN("Erro ao tentar salvar os parâmetros na flash.");
      esp_printf("Erro na Flash.");
    }
    break;
  }
  case COMANDO_CONFIG_SISTEMA:
  {
    if (pacote.is_set)
    {
      memoria.passo_maximo_pwm = pacote.payload.config_sistema.passo_maximo_pwm;
      memoria.periodo_ttl_ms = pacote.payload.config_sistema.periodo_ttl_ms;
      for (int i = 0; i < 2; i++)
        motor[i].passoMaximo = memoria.passo_maximo_pwm;
      DEBUG_PRINTLN("Configurações Físicas atualizadas na RAM!");
    }
    // RETORNAR MSG
    break;
  }
  case COMANDO_TELEMETRIA:
  {
    if (pacote.is_set)
    {
      memoria.periodo_telemetria_ms = pacote.payload.telemetria.periodo_telemetria_ms;
      if (memoria.periodo_telemetria_ms > 0)
      {
        DEBUG_PRINTLN("Ativando telemetria... {periodo: " + String(memoria.periodo_telemetria_ms) + "ms}");
        esp_printf("Tele:on %u ms", memoria.periodo_telemetria_ms);
      }
      else
      {
        DEBUG_PRINTLN("Desativando telemetria...");
        esp_printf("Tele:off");
      }
    }
    break;
  }
  }
}

void setup()
{
  DEBUG_BEGIN(115200);
  delay(500);
  DEBUG_PRINTLN("A inicializar o carrinho...");

  // inicia memoria e tenta carregar o MAC do rádio principal para pareamento automático
  if (inicializar_memoria() && carregar_config(&memoria))
  {
    DEBUG_PRINTLN("Dados carregados!");
  }
  else
  {
    DEBUG_PRINTLN("Falha na Flash ou Configuração não encontrada. Definindo novos valores...");
    uint8_t mac_novo[6] = {0};
    memset(memoria.mac_esp_principal, 0, 6);
    memcpy(memoria.mac_esp_principal, mac_novo, 6);
    memoria.indice = 255;
    for (int i = 0; i < 2; i++)
      memoria.pid[i] = {2.5f, 0.8f, 0.01f, 5.0f};
    memoria.pareado = false;
    memoria.periodo_ttl_ms = 500;
    memoria.passo_maximo_pwm = 100;
    memoria.periodo_controle_ms = 20;

    if (salvar_config(&memoria))
      DEBUG_PRINTLN("Dados salvos com sucesso!");
    else
      DEBUG_PRINTLN("Falha ao salvar.");
  }
  DEBUG_PRINTF("MAC do transmissor: %02X:%02X:%02X:%02X:%02X:%02X\n",
               memoria.mac_esp_principal[0], memoria.mac_esp_principal[1],
               memoria.mac_esp_principal[2], memoria.mac_esp_principal[3],
               memoria.mac_esp_principal[4], memoria.mac_esp_principal[5]);
  DEBUG_PRINTF("ID: %d\n", memoria.indice);
  DEBUG_PRINTLN("Pareado: " + String(memoria.pareado));
  DEBUG_PRINTLN("Configurações de PID carregadas:");
  for (int i = 0; i < 2; i++)
  {
    DEBUG_PRINTF("Motor %d - KP: %.2f, KI: %.2f, KD: %.2f, KF: %.2f\n", i, memoria.pid[i].kp, memoria.pid[i].ki, memoria.pid[i].kd, memoria.pid[i].kf);
  }

  // Configura os motores
  motor[0] = criarMotor(PH_IN1, PH_IN2, memoria.passo_maximo_pwm);
  motor[1] = criarMotor(PH_IN3, PH_IN4, memoria.passo_maximo_pwm);

  // Configura os encoders e as interrupções
  inicializarEncoder(&encoder[0], ENC0_PINA, ENC0_PINB);
  inicializarEncoder(&encoder[1], ENC1_PINA, ENC1_PINB);
  attachInterrupt(digitalPinToInterrupt(encoder[0].pinA), isr0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder[1].pinA), isr1, CHANGE);

  for (int i = 0; i < 2; i++)
  {
    pid_iniciar(&pidMotor[i], memoria.pid[i].kp, memoria.pid[i].ki, memoria.pid[i].kd, memoria.pid[i].kf, PID_MIN_PWM, PID_MAX_PWM);
    calcular_motor[i] = controle_pid;
  }

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

  // adiciona o peer de broadcast para garantir que pode enviar respostas para o transmissor
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    DEBUG_PRINTLN("Falha ao adicionar o peer de broadcast");
  }

  periodo_ttl_ativo = true;

  // Incicializa os timers
  millis_ttl = millis();
  millis_atual = millis();
  DEBUG_PRINTLN("Carrinho iniciado com sucesso!");
  // tocarSomMotor(&motor0, 8000, 500);
  // tocarSomMotor(&motor1, 8000, 500);
}

void loop()
{
  unsigned long millis_atual = millis();

  if (millis_atual - millis_atual > memoria.periodo_controle_ms)
  {
    // 1. Atualiza a leitura de Variação (Delta) do Encoder
    for (int i = 0; i < 2; i++)
      atualizarDeltaTicks(&encoder[i]);

    // 2. Calcula o PID ou Autotune
    float pwm_saida[2];
    for (int i = 0; i < 2; i++)
      pwm_saida[i] = calcular_motor[i](i, delta_ticks_target[i], encoder[i].delta_ticks);

    // 3. Aplica nos motores (assumindo que a sua struct Motor ou biblioteca espera valor com sinal)
    for (int i = 0; i < 2; i++)
      moverMotor(&motor[i], (int)pwm_saida[i]);

    millis_atual = millis_atual;
  }

  if (periodo_ttl_ativo && millis_atual - millis_ttl > memoria.periodo_ttl_ms)
  {
    for (int i = 0; i < 2; i++)
      delta_ticks_target[i] = 0;
    millis_ttl = millis_atual;
  }

  if (memoria.periodo_telemetria_ms != 0 && (millis_atual - millis_telemetria > memoria.periodo_telemetria_ms)) // Envia a cada intervalo_telemetria_ms ms
  {
    millis_telemetria = millis_atual;

    Mensagem msgTelemetria;
    msgTelemetria.tipo = COMANDO_TELEMETRIA;
    msgTelemetria.indice_destino = ID_TRANSMISSOR;
    msgTelemetria.indice_remetente = memoria.indice; // Ou seu ID local
    msgTelemetria.is_set = true;

    // Preenche os dados atuais (Delta dos Encoders) e os alvos (Target)
    for (int i = 0; i < 2; i++)
    {
      msgTelemetria.payload.telemetria.delta_ticks_atual[i] = encoder[i].delta_ticks;
      msgTelemetria.payload.telemetria.delta_ticks_target[i] = delta_ticks_target[i];
    }
    msgTelemetria.payload.telemetria.rssi_carrinho = ultimo_rssi;
    msgTelemetria.payload.telemetria.noise_floor_carrinho = ultimo_noise_floor;
    msgTelemetria.payload.telemetria.cnt_pacotes_totais = cnt_pacotes_totais;
    msgTelemetria.payload.telemetria.cnt_pacotes_validos = cnt_pacotes_validos;
    cnt_pacotes_totais = 0;
    cnt_pacotes_validos = 0;
    // Envia para o Transmissor
    esp_now_send(broadcastAddress, (uint8_t *)&msgTelemetria, sizeof(msgTelemetria));
  }
}