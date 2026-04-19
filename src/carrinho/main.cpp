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
#define DEBUG_BEGIN(x) Serial.begin(x)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_BEGIN(x)
#define DEBUG_PRINTF(...)
#endif

// conexoes dos motores e encoders
#define PH_IN1 2
#define PH_IN2 1
#define PH_IN3 3
#define PH_IN4 4

#define ENC0_PINA 7
#define ENC0_PINB 6
#define ENC1_PINA 0
#define ENC1_PINB 5

#define NUMERO_RODAS 2
#define DIR 0 // indice roda direita
#define ESQ 1 // indice roda esquerda
#define BROADCAST_ADDRESS (uint8_t[]){0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

// Estrutura para guardar os parâmetros na memoria Flash
DadosConfig memoria;

// Variaveis de rede e esp now
uint8_t ultimo_rssi;
uint8_t ultimo_noise_floor;
uint32_t cnt_pacotes_totais;
uint32_t cnt_pacotes_validos;

// Variável para ativar/desativar o envio de logs via ESP-NOW
bool esp_print_ativo = true;
// Variáveis de controle de Tasks
bool periodo_ttl_ativo;
uint32_t millis_atual;
uint32_t millis_ttl;
uint32_t millis_telemetria;
uint32_t millis_controle;

typedef float (*FuncaoControle)(Roda *roda, float setpoint, float medido);

typedef struct
{
  uint8_t indice_roda;
  Motor motor;
  Encoder encoder;
  PidConfig pidMotor;
  float delta_ticks_target;
  PidAutotune tuneMotor;
  FuncaoControle calcular_motor;
} Roda;

Roda rodas[NUMERO_RODAS];

CRIAR_ISR_ENCODER(isr0, rodas[DIR].encoder)
CRIAR_ISR_ENCODER(isr1, rodas[ESQ].encoder)

// Função para enviar mensagens de texto de debug formatadas para o Python via ESP-NOW
void esp_printf(const char *formato, ...)
{
  Mensagem msg;
  msg.tipo = COMANDO_PRINT;
  msg.indice_destino = ID_TRANSMISSOR; // Envia para o PC
  msg.indice_remetente = memoria.indice;

  // 1. Formata a string (como um printf) diretamente dentro do payload da mensagem
  va_list args;
  va_start(args, formato);
  vsnprintf(msg.payload.print.texto, sizeof(msg.payload.print.texto), formato, args);
  va_end(args);

  // 2. Sempre imprime na porta Serial (se o DEBUG_ENABLE estiver ativado)
  // Nota: o formato não recebe "\n" automático aqui para manter o comportamento exato do printf
  DEBUG_PRINTF("%s", msg.payload.print.texto);

  // 3. Envia via ESP-NOW apenas se o print de rede estiver ativado e o carrinho pareado
  if (esp_print_ativo && memoria.pareado)
  {
    esp_now_send(BROADCAST_ADDRESS, (uint8_t *)&msg, sizeof(msg));
  }
}

float controle_pid(Roda *roda, float setpoint, float medido)
{
  return pid_computar(&(*roda).pidMotor, setpoint, medido);
}

float controle_autotune(Roda *roda, float setpoint, float medido)
{
  float pwm = autotune_computar(&roda->tuneMotor, &roda->pidMotor, setpoint, medido);

  // Se o autotune acabou nesta iteração:
  if (!roda->tuneMotor.ativo)
  {
    esp_printf("tune m%d feito!\n", roda->indice_roda);

    // 1. Devolve o controle pro PID normal!
    roda->calcular_motor = controle_pid;
    periodo_ttl_ativo = true;

    // 2. Envia a mensagem de aviso pro Python
    Mensagem msgFim;
    msgFim.tipo = COMANDO_PID;
    msgFim.indice_destino = ID_TRANSMISSOR;
    msgFim.indice_remetente = memoria.indice;
    msgFim.payload.pidconfig.roda = roda->indice_roda;
    msgFim.payload.pidconfig.kp = roda->pidMotor.kp;
    msgFim.payload.pidconfig.ki = roda->pidMotor.ki;
    msgFim.payload.pidconfig.kd = roda->pidMotor.kd;
    msgFim.payload.pidconfig.kf = roda->pidMotor.kf;

    memoria.pid[roda->indice_roda].kp = roda->pidMotor.kp;
    memoria.pid[roda->indice_roda].ki = roda->pidMotor.ki;
    memoria.pid[roda->indice_roda].kd = roda->pidMotor.kd;
    memoria.pid[roda->indice_roda].kf = roda->pidMotor.kf;
    esp_now_send(BROADCAST_ADDRESS, (uint8_t *)&msgFim, sizeof(msgFim));
  }

  return pwm;
}

void tratar_comando_movimento(const Mensagem *pacote)
{
  for (int i = 0; i < NUMERO_RODAS; i++)
    rodas[i].delta_ticks_target = pacote->payload.movimento.target_ticks[i];
  millis_ttl = millis();
  periodo_ttl_ativo = true;
}

void tratar_comando_movimento_global(const Mensagem *pacote)
{
  if (memoria.indice < MAX_ROBOS)
  {
    for (int i = 0; i < NUMERO_RODAS; i++)
      rodas[i].delta_ticks_target = pacote->payload.movimento_global.target_ticks[i][memoria.indice];
    millis_ttl = millis();
    periodo_ttl_ativo = true;
  }
}

void tratar_comando_id(const Mensagem *pacote, const esp_now_recv_info_t *info_pacote)
{
  if (pacote->is_set)
  {
    uint8_t meu_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, meu_mac);
    bool mac_eh_meu = true;
    for (int i = 0; i < 6; i++)
    {
      if (pacote->payload.set_id.mac_alvo[i] != meu_mac[i])
      {
        mac_eh_meu = false;
        break;
      }
    }
    if (mac_eh_meu)
    {
      memoria.indice = pacote->payload.set_id.novo_id;
    }
  }

  Mensagem resposta;
  esp_printf("ECHO - ID: %d\n", memoria.indice);
  resposta.indice_remetente = memoria.indice;
  resposta.tipo = COMANDO_ID;
  resposta.indice_destino = ID_TRANSMISSOR;
  resposta.payload.echo.rssi = (uint8_t)info_pacote->rx_ctrl->rssi;
  esp_wifi_get_mac(WIFI_IF_STA, resposta.payload.echo.mac);
  esp_now_send(BROADCAST_ADDRESS, (uint8_t *)&resposta, sizeof(resposta));
}

void tratar_comando_pid(const Mensagem *pacote)
{
  if (pacote->is_set)
  {
    int i_roda = pacote->payload.pidconfig.roda;

    memoria.pid[i_roda].kp = pacote->payload.pidconfig.kp;
    memoria.pid[i_roda].ki = pacote->payload.pidconfig.ki;
    memoria.pid[i_roda].kd = pacote->payload.pidconfig.kd;
    memoria.pid[i_roda].kf = pacote->payload.pidconfig.kf;

    // Correção: Usando ponteiro para modificar a roda global e não uma cópia
    Roda *roda = &rodas[i_roda];
    roda->pidMotor.kp = memoria.pid[i_roda].kp;
    roda->pidMotor.ki = memoria.pid[i_roda].ki;
    roda->pidMotor.kd = memoria.pid[i_roda].kd;
    roda->pidMotor.kf = memoria.pid[i_roda].kf;
    pid_resetar(&roda->pidMotor);
  }

  Mensagem resposta;
  resposta.tipo = COMANDO_PID;
  resposta.indice_destino = ID_TRANSMISSOR;
  resposta.indice_remetente = memoria.indice;
  for (int i = 0; i < NUMERO_RODAS; i++)
  {
    resposta.payload.pidconfig.roda = i;
    resposta.payload.pidconfig.kp = rodas[i].pidMotor.kp;
    resposta.payload.pidconfig.ki = rodas[i].pidMotor.ki;
    resposta.payload.pidconfig.kd = rodas[i].pidMotor.kd;
    resposta.payload.pidconfig.kf = rodas[i].pidMotor.kf;
    esp_now_send(BROADCAST_ADDRESS, (uint8_t *)&resposta, sizeof(resposta));
  }
}

void tratar_comando_autotune_pid(const Mensagem *pacote)
{
  periodo_ttl_ativo = false;
  Roda *roda = &rodas[pacote->payload.autotune.roda];

  autotune_iniciar(&roda->tuneMotor, pacote->payload.autotune.pwm_teste_max, pacote->payload.autotune.pwm_teste_min, pacote->payload.autotune.ciclos);
  roda->delta_ticks_target = pacote->payload.autotune.target_ticks;
  roda->calcular_motor = controle_autotune;
}

void tratar_comando_salvar()
{
  if (salvar_config(&memoria))
    esp_printf("Salvo!\n");
  else
    esp_printf("Erro na Flash.\n");
}

void tratar_comando_config_sistema(const Mensagem *pacote)
{
  if (pacote->is_set)
  {
    memoria.passo_maximo_pwm = pacote->payload.config_sistema.passo_maximo_pwm;
    memoria.periodo_ttl_ms = pacote->payload.config_sistema.periodo_ttl_ms;
    for (int i = 0; i < NUMERO_RODAS; i++)
    {
      rodas[i].motor.passoMaximo = memoria.passo_maximo_pwm;
    }
    esp_print("Recebido.\n");
  }
}

void tratar_comando_telemetria(const Mensagem *pacote)
{
  if (pacote->is_set)
  {
    memoria.periodo_telemetria_ms = pacote->payload.telemetria.periodo_telemetria_ms;
    if (memoria.periodo_telemetria_ms > 0)
      esp_printf("Tele:on %u ms\n", memoria.periodo_telemetria_ms);
    else
      esp_printf("Tele:off\n");
  }
}

bool pacote_eh_seguro(const Mensagem *pacote, const uint8_t *mac_remetente)
{
  // 1. O carrinho já deve ter concluído o processo de pareamento
  if (!memoria.pareado)
    return false;

  // 2. O comando deve obrigatoriamente vir do transmissor reconhecido
  if (pacote->indice_remetente != ID_TRANSMISSOR)
    return false;

  // 3. O destino precisa ser este carrinho específico ou um envio em massa (Broadcast)
  if (pacote->indice_destino != memoria.indice && pacote->indice_destino != ID_BROADCAST)
    return false;

  // 4. Autenticação física: O MAC do remetente deve ser idêntico ao salvo na memória
  for (int i = 0; i < 6; i++)
  {
    if (mac_remetente[i] != memoria.mac_esp_principal[i])
      return false;
  }

  // Se sobreviveu a todas as validações, o pacote é legítimo
  return true;
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
      cnt_pacotes_validos++;

      // Senha correta! Copia o MAC do remetente para a struct da memória (apenas na RAM)
      memcpy(memoria.mac_esp_principal, info_pacote->src_addr, 6);

      // Habilita a comunicação sem gastar a memória Flash
      memoria.pareado = true;
      esp_print("Pareado. use SALVAR.\n");

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
      esp_now_send(BROADCAST_ADDRESS, (u_int8_t *)&resposta, sizeof(resposta));
    }
    return; // Para a execução aqui, pois o comando era só para parear
  }

  // ---------------------------------------------------------
  // 2. FILTRO DE SEGURANÇA PARA COMANDOS COMUNS
  // ---------------------------------------------------------

  if (!pacote_eh_seguro(&pacote, info_pacote->src_addr))
    return;              // Early Return: Se não for seguro, descarta imediatamente sem processar
  cnt_pacotes_validos++; // Se passou por tudo, é um pacote válido!

  // ---------------------------------------------------------
  // 3. EXECUÇÃO DOS COMANDOS AUTORIZADOS
  // ---------------------------------------------------------

  if (memoria.periodo_telemetria_ms)
  {
    ultimo_rssi = info_pacote->rx_ctrl->rssi;
    ultimo_noise_floor = info_pacote->rx_ctrl->noise_floor;
  }

  switch (pacote.tipo)
  {
  case COMANDO_MOVIMENTO:
    tratar_comando_movimento(&pacote);
    break;

  case COMANDO_MOVIMENTO_GLOBAL:
    tratar_comando_movimento_global(&pacote);
    break;

  case COMANDO_ID:
    tratar_comando_id(&pacote, info_pacote);
    break;

  case COMANDO_PID:
    tratar_comando_pid(&pacote);
    break;

  case COMANDO_AUTOTUNE_PID:
    tratar_comando_autotune_pid(&pacote);
    break;

  case COMANDO_SALVAR:
    tratar_comando_salvar();
    break;

  case COMANDO_CONFIG_SISTEMA:
    tratar_comando_config_sistema(&pacote);
    break;

  case COMANDO_TELEMETRIA:
    tratar_comando_telemetria(&pacote);
    break;
  }
}

void setup()
{
  DEBUG_BEGIN(115200);
  delay(500);
  DEBUG_PRINTF("A inicializar o carrinho...\n");

  // inicia memoria e tenta carregar o MAC do rádio principal para pareamento automático
  if (inicializar_memoria() && carregar_config(&memoria))
  {
    DEBUG_PRINTF("Dados carregados!\n");
  }
  else
  {
    DEBUG_PRINTF("Falha na Flash ou Configuração não encontrada. Definindo novos valores...\n");
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
    memoria.canal_wifi = 11;

    if (salvar_config(&memoria))
      DEBUG_PRINTF("Dados salvos com sucesso!\n");
    else
      DEBUG_PRINTF("Falha ao salvar.\n");
  }
  DEBUG_PRINTF("MAC do transmissor: %02X:%02X:%02X:%02X:%02X:%02X\n",
               memoria.mac_esp_principal[0], memoria.mac_esp_principal[1],
               memoria.mac_esp_principal[2], memoria.mac_esp_principal[3],
               memoria.mac_esp_principal[4], memoria.mac_esp_principal[5]);
  DEBUG_PRINTF("ID: %d\n", memoria.indice);
  DEBUG_PRINTF("Pareado: %s\n", String(memoria.pareado));
  DEBUG_PRINTF("Configurações de PID carregadas:\n");
  for (int i = 0; i < 2; i++)
  {
    DEBUG_PRINTF("Motor %d - KP: %.2f, KI: %.2f, KD: %.2f, KF: %.2f\n", i, memoria.pid[i].kp, memoria.pid[i].ki, memoria.pid[i].kd, memoria.pid[i].kf);
  }

  // Configura os motores
  rodas[DIR].motor = criarMotor(PH_IN1, PH_IN2, memoria.passo_maximo_pwm);
  rodas[ESQ].motor = criarMotor(PH_IN3, PH_IN4, memoria.passo_maximo_pwm);

  // Configura os encoders e as interrupções
  inicializarEncoder(&rodas[DIR].encoder, ENC0_PINA, ENC0_PINB);
  inicializarEncoder(&rodas[ESQ].encoder, ENC1_PINA, ENC1_PINB);
  attachInterrupt(digitalPinToInterrupt(rodas[DIR].encoder.pinA), isr0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rodas[ESQ].encoder.pinA), isr1, CHANGE);

  for (int i = 0; i < NUMERO_RODAS; i++)
  {
    pid_iniciar(&rodas[i].pidMotor, memoria.pid[i].kp, memoria.pid[i].ki, memoria.pid[i].kd, memoria.pid[i].kf, PID_MIN_PWM, PID_MAX_PWM);
    rodas[i].calcular_motor = controle_pid;
  }

  // Configura o Wi-Fi e o ESP-NOW
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_channel(memoria.canal_wifi, WIFI_SECOND_CHAN_NONE));
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84));
  if (esp_now_init() != ESP_OK)
    return;

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv)); // Registra a função de callback para receber dados via ESP-NOW

  // adiciona o peer de broadcast para garantir que pode enviar respostas para o transmissor
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, BROADCAST_ADDRESS, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    DEBUG_PRINTF("Falha ao adicionar o peer de broadcast\n");
  }

  periodo_ttl_ativo = true;

  // Incicializa os timers
  millis_atual = millis();
  millis_ttl = millis_atual;
  millis_controle = millis_atual;
  millis_telemetria = millis_atual;
  DEBUG_PRINTF("Carrinho iniciado com sucesso!\n");
  // tocarSomMotor(&motor0, 8000, 500);
  // tocarSomMotor(&motor1, 8000, 500);
}

void loop()
{
  unsigned long millis_atual = millis();

  if (millis_atual - millis_controle > memoria.periodo_controle_ms)
  {
    millis_controle = millis_atual;
    // 1. Atualiza a leitura de Variação (Delta) do Encoder
    for (int i = 0; i < NUMERO_RODAS; i++)
      atualizarDeltaTicks(&rodas[i].encoder);

    // 2. Calcula o PID ou Autotune
    float pwm_saida[2];
    for (int i = 0; i < NUMERO_RODAS; i++)
      pwm_saida[i] = rodas[i].calcular_motor(&rodas[i], rodas[i].delta_ticks_target, rodas[i].encoder.delta_ticks);

    // 3. Aplica nos motores (assumindo que a sua struct Motor ou biblioteca espera valor com sinal)
    for (int i = 0; i < NUMERO_RODAS; i++)
      moverMotor(&rodas[i].motor, (int)pwm_saida[i]);
  }

  if (periodo_ttl_ativo && millis_atual - millis_ttl > memoria.periodo_ttl_ms)
  {
    millis_ttl = millis_atual;
    for (int i = 0; i < NUMERO_RODAS; i++)
      rodas[i].delta_ticks_target = 0;
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
      msgTelemetria.payload.telemetria.delta_ticks_atual[i] = rodas[i].encoder.delta_ticks;
      msgTelemetria.payload.telemetria.delta_ticks_target[i] = rodas[i].delta_ticks_target;
    }
    msgTelemetria.payload.telemetria.rssi_carrinho = ultimo_rssi;
    msgTelemetria.payload.telemetria.noise_floor_carrinho = ultimo_noise_floor;
    msgTelemetria.payload.telemetria.cnt_pacotes_totais = cnt_pacotes_totais;
    msgTelemetria.payload.telemetria.cnt_pacotes_validos = cnt_pacotes_validos;
    cnt_pacotes_totais = 0;
    cnt_pacotes_validos = 0;
    // Envia para o Transmissor
    esp_now_send(BROADCAST_ADDRESS, (uint8_t *)&msgTelemetria, sizeof(msgTelemetria));
  }
}