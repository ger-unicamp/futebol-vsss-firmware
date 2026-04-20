#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <stdarg.h>

#include "mensagem.h"
#include "motor.h"
#include "encoder.h"
#include "memoria.h"
#include "pid.h"
#include "despachante.h"

// Comente esta linha ou mude para 0 para desativar o modo Debug
#define DEBUG_ENABLE

#ifdef DEBUG_ENABLE
#define DEBUG_BEGIN(x) Serial.begin(x)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_BEGIN(x)
#define DEBUG_PRINTF(...)
#endif

// Conexões dos motores e encoders
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

// Estrutura para guardar os parâmetros na memoria Flash
memoria_t memoria;

// Variáveis de rede e ESP-NOW
uint8_t meu_mac[6];
volatile int8_t ultimo_rssi;
volatile int8_t ultimo_noise_floor;
volatile uint32_t cnt_pacotes_totais;
volatile uint32_t cnt_pacotes_validos;

// Variável para ativar/desativar o envio de logs via ESP-NOW
bool esp_print_ativo = true;
// Variáveis de controle de Tasks
volatile bool periodo_ttl_ativo;
uint32_t millis_atual;
volatile uint32_t millis_ttl;
uint32_t millis_telemetria;
uint32_t millis_controle;

struct roda_t;
typedef float (*funcao_controle)(roda_t *roda, float setpoint, float medido);
struct roda_t
{
  uint8_t indice_roda;
  motor_t motor;
  encoder_t encoder;
  pid_config_t pid_motor;
  float delta_ticks_target;
  pid_autotune_t tune_motor;
  funcao_controle calcular_motor;
};

roda_t rodas[NUMERO_RODAS];

CRIAR_ISR_ENCODER(isr0, rodas[DIR].encoder)
CRIAR_ISR_ENCODER(isr1, rodas[ESQ].encoder)

// Função para enviar mensagens de texto de debug formatadas para o Python via ESP-NOW
void espnow_printf(const char *formato, ...)
{
  mensagem_t msg = {0};
  msg.tipo = COMANDO_PRINT;

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
    despachante_enfileirar(msg);
}

float controle_pid(roda_t *roda, float setpoint, float medido)
{
  return pid_computar(&roda->pid_motor, setpoint, medido);
}

float controle_autotune(roda_t *roda, float setpoint, float medido)
{
  float pwm = autotune_computar(&roda->tune_motor, &roda->pid_motor, setpoint, medido);

  // Se o autotune acabou nesta iteração:
  if (!roda->tune_motor.ativo)
  {
    espnow_printf("tune m%d feito!\n", roda->indice_roda);

    // 1. Devolve o controle para o PID normal!
    roda->calcular_motor = controle_pid;
    periodo_ttl_ativo = true;
    memoria.pid[roda->indice_roda].kp = roda->pid_motor.kp;
    memoria.pid[roda->indice_roda].ki = roda->pid_motor.ki;
    memoria.pid[roda->indice_roda].kd = roda->pid_motor.kd;
    memoria.pid[roda->indice_roda].kf = roda->pid_motor.kf;

    // 2. Envia a mensagem de aviso para o Python
    mensagem_t msg = {0};
    msg.tipo = COMANDO_PID;
    msg.payload.pid_config.roda = roda->indice_roda;
    msg.payload.pid_config.kp = roda->pid_motor.kp;
    msg.payload.pid_config.ki = roda->pid_motor.ki;
    msg.payload.pid_config.kd = roda->pid_motor.kd;
    msg.payload.pid_config.kf = roda->pid_motor.kf;
    despachante_enfileirar(msg);
  }

  return pwm;
}

void tratar_comando_movimento(const mensagem_t *pacote)
{
  for (int i = 0; i < NUMERO_RODAS; i++)
    rodas[i].delta_ticks_target = pacote->payload.movimento.target_ticks[i];
  millis_ttl = millis();
  periodo_ttl_ativo = true;
}

void tratar_comando_movimento_global(const mensagem_t *pacote)
{
  if (memoria.indice < MAX_ROBOS)
  {
    for (int i = 0; i < NUMERO_RODAS; i++)
      rodas[i].delta_ticks_target = pacote->payload.movimento_global.target_ticks[i][memoria.indice];
    millis_ttl = millis();
    periodo_ttl_ativo = true;
  }
}

void tratar_comando_id(const mensagem_t *pacote, const esp_now_recv_info_t *info_pacote)
{
  if (pacote->is_set)
  {
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

  mensagem_t msg = {0};
  msg.tipo = COMANDO_ID;
  msg.payload.echo.rssi = info_pacote->rx_ctrl->rssi;
  memcpy(msg.payload.echo.mac, meu_mac, 6);
  despachante_enfileirar(msg);
}

void tratar_comando_pid(const mensagem_t *pacote)
{
  if (pacote->is_set)
  {
    int i_roda = pacote->payload.pid_config.roda;
    if (i_roda >= NUMERO_RODAS)
      return; // Proteção contra estouro de memória!

    memoria.pid[i_roda].kp = pacote->payload.pid_config.kp;
    memoria.pid[i_roda].ki = pacote->payload.pid_config.ki;
    memoria.pid[i_roda].kd = pacote->payload.pid_config.kd;
    memoria.pid[i_roda].kf = pacote->payload.pid_config.kf;

    // Correção: Usando ponteiro para modificar a roda global e não uma cópia
    roda_t *roda = &rodas[i_roda];
    roda->pid_motor.kp = memoria.pid[i_roda].kp;
    roda->pid_motor.ki = memoria.pid[i_roda].ki;
    roda->pid_motor.kd = memoria.pid[i_roda].kd;
    roda->pid_motor.kf = memoria.pid[i_roda].kf;
    pid_resetar(&roda->pid_motor);
  }

  mensagem_t msg = {0};
  msg.tipo = COMANDO_PID;
  for (int i = 0; i < NUMERO_RODAS; i++)
  {
    msg.payload.pid_config.roda = i;
    msg.payload.pid_config.kp = rodas[i].pid_motor.kp;
    msg.payload.pid_config.ki = rodas[i].pid_motor.ki;
    msg.payload.pid_config.kd = rodas[i].pid_motor.kd;
    msg.payload.pid_config.kf = rodas[i].pid_motor.kf;
    despachante_enfileirar(msg);
  }
}

void tratar_comando_autotune_pid(const mensagem_t *pacote)
{
  if (pacote->payload.autotune.roda >= NUMERO_RODAS)
    return;
  periodo_ttl_ativo = false;
  roda_t *roda = &rodas[pacote->payload.autotune.roda];

  autotune_iniciar(&roda->tune_motor, pacote->payload.autotune.pwm_teste_max, pacote->payload.autotune.pwm_teste_min, pacote->payload.autotune.ciclos);
  roda->delta_ticks_target = pacote->payload.autotune.target_ticks;
  roda->calcular_motor = controle_autotune;
}

void tratar_comando_salvar()
{
  if (salvar_config(&memoria))
    espnow_printf("Salvo!\n");
  else
    espnow_printf("Erro na Flash.\n");
}

void tratar_comando_config_sistema(const mensagem_t *pacote)
{
  if (pacote->is_set)
  {
    memoria.passo_maximo_pwm = pacote->payload.config_sistema.passo_maximo_pwm;
    memoria.periodo_ttl_ms = pacote->payload.config_sistema.periodo_ttl_ms;
    for (int i = 0; i < NUMERO_RODAS; i++)
    {
      rodas[i].motor.passo_maximo_pwm = memoria.passo_maximo_pwm;
    }
    espnow_printf("Recebido.\n");
  }
}

void tratar_comando_telemetria(const mensagem_t *pacote)
{
  if (pacote->is_set)
  {
    memoria.periodo_telemetria_ms = pacote->payload.telemetria.periodo_telemetria_ms;
    if (memoria.periodo_telemetria_ms > 0)
      espnow_printf("Tele:on %u ms\n", memoria.periodo_telemetria_ms);
    else
      espnow_printf("Tele:off\n");
  }
}

bool pacote_eh_seguro(const mensagem_t *pacote, const uint8_t *mac_remetente)
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
void on_data_recv(const esp_now_recv_info_t *info_pacote, const uint8_t *dados, int tamanho)
{
  cnt_pacotes_totais = cnt_pacotes_totais + 1;
  // Verificação básica de tamanho
  if (tamanho != sizeof(mensagem_t))
    return; // tamanho inesperado, ignora o pacote

  mensagem_t pacote;
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
      espnow_printf("Pareado. use SALVAR.\n");

      // Responde ao Python
      mensagem_t msg = {0};
      msg.tipo = COMANDO_PAREAMENTO;
      memcpy(msg.payload.pareamento.senha, SENHA_PAREAMENTO, sizeof(msg.payload.pareamento.senha));
      memcpy(msg.payload.pareamento.mac, meu_mac, 6);
      DEBUG_PRINTF("Respondendo para o transmissor com meu MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   msg.payload.pareamento.mac[0], msg.payload.pareamento.mac[1],
                   msg.payload.pareamento.mac[2], msg.payload.pareamento.mac[3],
                   msg.payload.pareamento.mac[4], msg.payload.pareamento.mac[5]);
      despachante_enfileirar(msg);
    }
    return; // Para a execução aqui, pois o comando era só para parear
  }

  // ---------------------------------------------------------
  // 2. FILTRO DE SEGURANÇA PARA COMANDOS COMUNS
  // ---------------------------------------------------------

  if (!pacote_eh_seguro(&pacote, info_pacote->src_addr))
    return;                                      // Early Return: Se não for seguro, descarta imediatamente sem processar
  cnt_pacotes_validos = cnt_pacotes_validos + 1; // Se passou por tudo, é um pacote válido!

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

  // Inicia a memória e tenta carregar o MAC do rádio principal para pareamento automático
  if (inicializar_memoria() && carregar_config(&memoria))
  {
    DEBUG_PRINTF("Dados carregados!\n");
  }
  else
  {
    DEBUG_PRINTF("Falha na Flash ou Configuração não encontrada. Definindo novos valores...\n");
    memset(memoria.mac_esp_principal, 0, 6);
    memoria.indice = 255;
    for (int i = 0; i < NUMERO_RODAS; i++)
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
  DEBUG_PRINTF("Pareado: %s\n", memoria.pareado ? "Sim" : "Nao");
  DEBUG_PRINTF("Configurações de PID carregadas:\n");
  for (int i = 0; i < NUMERO_RODAS; i++)
  {
    DEBUG_PRINTF("Motor %d - KP: %.2f, KI: %.2f, KD: %.2f, KF: %.2f\n", i, memoria.pid[i].kp, memoria.pid[i].ki, memoria.pid[i].kd, memoria.pid[i].kf);
  }

  // Configura os motores
  rodas[DIR].motor = criar_motor(PH_IN1, PH_IN2, memoria.passo_maximo_pwm);
  rodas[ESQ].motor = criar_motor(PH_IN3, PH_IN4, memoria.passo_maximo_pwm);
  rodas[DIR].indice_roda = DIR;
  rodas[ESQ].indice_roda = ESQ;

  // Configura os encoders e as interrupções
  inicializar_encoder(&rodas[DIR].encoder, ENC0_PINA, ENC0_PINB);
  inicializar_encoder(&rodas[ESQ].encoder, ENC1_PINA, ENC1_PINB);
  attachInterrupt(digitalPinToInterrupt(rodas[DIR].encoder.pin_a), isr0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rodas[ESQ].encoder.pin_a), isr1, CHANGE);

  for (int i = 0; i < NUMERO_RODAS; i++)
  {
    pid_iniciar(&rodas[i].pid_motor, memoria.pid[i].kp, memoria.pid[i].ki, memoria.pid[i].kd, memoria.pid[i].kf, PID_MIN_PWM, PID_MAX_PWM);
    rodas[i].calcular_motor = controle_pid;
  }

  despachante_init();

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

  esp_now_register_recv_cb(esp_now_recv_cb_t(on_data_recv)); // Registra a função de callback para receber dados via ESP-NOW

  // adiciona o peer de broadcast para garantir que pode enviar respostas para o transmissor
  esp_now_peer_info_t peer_info;
  memset(&peer_info, 0, sizeof(peer_info));
  memcpy(peer_info.peer_addr, BROADCAST_ADDRESS, 6);
  if (esp_now_add_peer(&peer_info) != ESP_OK)
  {
    DEBUG_PRINTF("Falha ao adicionar o peer de broadcast\n");
  }

  esp_wifi_get_mac(WIFI_IF_STA, meu_mac);
  periodo_ttl_ativo = true;

  // Inicializa os timers
  millis_atual = millis();
  millis_ttl = millis_atual;
  millis_controle = millis_atual;
  millis_telemetria = millis_atual;
  DEBUG_PRINTF("Carrinho iniciado com sucesso!\n");
  if (memoria.pareado)
  {
    // manda mensagem para parear se apresentando ao transmissor
  }
}

void loop()
{
  despachante_processar_loop();
  millis_atual = millis();

  if (millis_atual - millis_controle > memoria.periodo_controle_ms)
  {
    millis_controle = millis_atual;
    // 1. Atualiza a leitura de Variação (Delta) do Encoder
    for (int i = 0; i < NUMERO_RODAS; i++)
      atualizar_delta_ticks(&rodas[i].encoder);

    // 2. Calcula o PID ou Autotune
    float pwm_saida[NUMERO_RODAS];
    for (int i = 0; i < NUMERO_RODAS; i++)
      pwm_saida[i] = rodas[i].calcular_motor(&rodas[i], rodas[i].delta_ticks_target, rodas[i].encoder.delta_ticks);

    // 3. Aplica nos motores (assumindo que a sua struct Motor ou biblioteca espera valor com sinal)
    for (int i = 0; i < NUMERO_RODAS; i++)
      mover_motor(&rodas[i].motor, (int)pwm_saida[i]);
  }

  if (periodo_ttl_ativo && millis_atual - millis_ttl > memoria.periodo_ttl_ms)
  {
    millis_ttl = millis_atual;
    for (int i = 0; i < NUMERO_RODAS; i++)
      rodas[i].delta_ticks_target = 0;
  }

  if (memoria.periodo_telemetria_ms != 0 && (millis_atual - millis_telemetria > memoria.periodo_telemetria_ms))
  {
    millis_telemetria = millis_atual;

    mensagem_t msg = {0};
    msg.tipo = COMANDO_TELEMETRIA;

    // Preenche os dados atuais (Delta dos Encoders) e os alvos (Target)
    for (int i = 0; i < NUMERO_RODAS; i++)
    {
      msg.payload.telemetria.delta_ticks_atual[i] = rodas[i].encoder.delta_ticks;
      msg.payload.telemetria.delta_ticks_target[i] = rodas[i].delta_ticks_target;
    }
    msg.payload.telemetria.rssi_carrinho = ultimo_rssi;
    msg.payload.telemetria.noise_floor_carrinho = ultimo_noise_floor;
    msg.payload.telemetria.cnt_pacotes_totais = cnt_pacotes_totais;
    msg.payload.telemetria.cnt_pacotes_validos = cnt_pacotes_validos;
    despachante_enfileirar(msg);
  }
}