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

// Frequência de controle do PID (em Hz)
#define FREQ_ATT 60
#define DELAY 1000 / FREQ_ATT

// Comando para broadcast (Todos os carrinhos)
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t mac_esp_principal;
esp_now_peer_info_t peerInfo;

// Variáveis de controle TIME TO LIVE (TTL) para os comandos de movimento
uint32_t tempo_ttl_ms = 500; // Tempo de vida do comando em ms (TTL)
bool ttl_ativo = false;

// Criar as instâncias dos Motores
Motor motor0 = {0};
Motor motor1 = {0};
Encoder encoder0 = {0};
Encoder encoder1 = {0};
Mensagem msg;
bool pareado = false;
unsigned long millis_ttl = 0;
unsigned long millis_att = 0;
DadosConfig memoria;

// Prototipação das funções de controle (PID e Autotune)
CRIAR_ISR_ENCODER(isr0, encoder0)
CRIAR_ISR_ENCODER(isr1, encoder1)

// Instancia os PIDs para cada roda
PidConfig pidMotor0;
PidConfig pidMotor1;

// Variáveis alvo (Ticks por ciclo de controle de 16ms)
float target_ticks_0 = 0;
float target_ticks_1 = 0;

PidAutotune tuneMotor0 = {0};
PidAutotune tuneMotor1 = {0};

unsigned long cnt_pacotes_totais = 0;
unsigned long cnt_pacotes_validos = 0;

typedef float (*FuncaoControle)(float setpoint, float medido);
FuncaoControle calcular_motor0;
FuncaoControle calcular_motor1;

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

float controle_pid_m0(float setpoint, float medido)
{
  return pid_computar(&pidMotor0, setpoint, medido);
}

float controle_autotune_m0(float setpoint, float medido)
{
  float pwm = autotune_computar(&tuneMotor0, &pidMotor0, setpoint, medido);

  // Se o autotune acabou nesta iteração:
  if (!tuneMotor0.ativo)
  {
    DEBUG_PRINTLN("Autotune Motor 0 Finalizado!");

    // 1. Devolve o controle pro PID normal!
    calcular_motor0 = controle_pid_m0;
    ttl_ativo = true;

    // 2. Envia a mensagem de aviso pro Python
    Mensagem msgFim;
    msgFim.tipo = COMANDO_PID;
    msgFim.indice_destino = ID_TRANSMISSOR;
    msgFim.indice_remetente = memoria.indice;
    msgFim.payload.pidconfig.roda = 0;
    msgFim.payload.pidconfig.kp = pidMotor0.kp;
    msgFim.payload.pidconfig.ki = pidMotor0.ki;
    msgFim.payload.pidconfig.kd = pidMotor0.kd;
    msgFim.payload.pidconfig.kf = pidMotor0.kf;
    esp_now_send(broadcastAddress, (uint8_t *)&msgFim, sizeof(msgFim));
  }

  return pwm;
}

float controle_pid_m1(float setpoint, float medido)
{
  return pid_computar(&pidMotor1, setpoint, medido);
}

float controle_autotune_m1(float setpoint, float medido)
{
  float pwm = autotune_computar(&tuneMotor1, &pidMotor1, setpoint, medido);

  // Se o autotune acabou nesta iteração:
  if (!tuneMotor1.ativo)
  {
    DEBUG_PRINTLN("Autotune Motor 1 Finalizado!");

    // 1. Devolve o controle pro PID normal!
    calcular_motor1 = controle_pid_m1;
    ttl_ativo = true;

    // 2. Envia a mensagem de aviso pro Python
    Mensagem msgFim;
    msgFim.tipo = COMANDO_PID;
    msgFim.indice_destino = ID_TRANSMISSOR;
    msgFim.indice_remetente = memoria.indice;
    msgFim.payload.pidconfig.roda = 1;
    msgFim.payload.pidconfig.kp = pidMotor1.kp;
    msgFim.payload.pidconfig.ki = pidMotor1.ki;
    msgFim.payload.pidconfig.kd = pidMotor1.kd;
    msgFim.payload.pidconfig.kf = pidMotor1.kf;
    esp_now_send(broadcastAddress, (uint8_t *)&msgFim, sizeof(msgFim));
  }

  return pwm;
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
      pareado = true;
      DEBUG_PRINTLN("Pareamento concluído na RAM! Lembre-se de usar o COMANDO_SALVAR para persistir.");

      // Responde ao Python
      Mensagem resposta;
      resposta.tipo = COMANDO_PAREAMENTO;
      resposta.indice_destino = ID_TRANSMISSOR;
      resposta.indice_remetente = memoria.indice;
      esp_wifi_get_mac(WIFI_IF_STA, resposta.payload.pareamento.mac); // Coleta o próprio MAC
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
  if (!pareado)
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
  switch (pacote.tipo)
  {
  case COMANDO_MOVIMENTO:
  {
    target_ticks_0 = pacote.payload.movimento.vel_esq;
    target_ticks_1 = pacote.payload.movimento.vel_dir;
    millis_ttl = millis();
    break;
    // retornar msg com deltatiks algo e lidos
  }
  case COMANDO_MOVIMENTO_GLOBAL:
  {
    if (memoria.indice < MAX_ROBOS)
    {
      target_ticks_0 = pacote.payload.movimento_global.vel_esq[memoria.indice];
      target_ticks_1 = pacote.payload.movimento_global.vel_dir[memoria.indice];
      millis_ttl = millis();
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
      // muda os valores na RAM
      PidConfig *pidAlvo = (pacote.payload.pidconfig.roda == 0) ? &pidMotor0 : &pidMotor1;
      pidAlvo->kp = pacote.payload.pidconfig.kp;
      pidAlvo->ki = pacote.payload.pidconfig.ki;
      pidAlvo->kd = pacote.payload.pidconfig.kd;
      pidAlvo->kf = pacote.payload.pidconfig.kf;
      // muda os valores para salvar na flash
      PIDParams *config = (pacote.payload.pidconfig.roda == 0) ? &memoria.pid0 : &memoria.pid1;
      config->kp = pidAlvo->kp;
      config->ki = pidAlvo->ki;
      config->kd = pidAlvo->kd;
      config->kf = pidAlvo->kf;
      pid_resetar(pidAlvo); // Zera o erro acumulado para não dar tranco
    }
    // manda duas mensagens de resposta, uma para cada roda, com os parâmetros atuais
    Mensagem resposta;
    resposta.tipo = COMANDO_PID;
    resposta.indice_destino = ID_TRANSMISSOR;
    resposta.indice_remetente = memoria.indice;

    resposta.payload.pidconfig.roda = 0;
    resposta.payload.pidconfig.kp = pidMotor0.kp;
    resposta.payload.pidconfig.ki = pidMotor0.ki;
    resposta.payload.pidconfig.kd = pidMotor0.kd;
    resposta.payload.pidconfig.kf = pidMotor0.kf;
    esp_now_send(broadcastAddress, (uint8_t *)&resposta, sizeof(resposta));

    resposta.payload.pidconfig.roda = 1;
    resposta.payload.pidconfig.kp = pidMotor1.kp;
    resposta.payload.pidconfig.ki = pidMotor1.ki;
    resposta.payload.pidconfig.kd = pidMotor1.kd;
    resposta.payload.pidconfig.kf = pidMotor1.kf;
    esp_now_send(broadcastAddress, (uint8_t *)&resposta, sizeof(resposta));
    break;
  }
  case COMANDO_AUTOTUNE_PID:
  {
    ttl_ativo = false; // Desativa o TTL para não zerar os alvos durante o autotune
    if (pacote.payload.pidconfig.roda == 0)
    {
      autotune_iniciar(&tuneMotor0, pacote.payload.autotune.pwm_teste_max, pacote.payload.autotune.pwm_teste_min, pacote.payload.autotune.ciclos);
      target_ticks_0 = pacote.payload.autotune.targert_ticks; // Define o alvo de ticks para o autotune
      calcular_motor0 = controle_autotune_m0;
    }
    else
    {
      autotune_iniciar(&tuneMotor1, pacote.payload.autotune.pwm_teste_max, pacote.payload.autotune.pwm_teste_min, pacote.payload.autotune.ciclos);
      target_ticks_1 = pacote.payload.autotune.targert_ticks; // Define o alvo de ticks para o autotune
      calcular_motor1 = controle_autotune_m1;
    }
    break;
  }
  case COMANDO_SALVAR:
  {
    if (salvar_config(&memoria))
    {
      DEBUG_PRINTLN("Parâmetros de PID salvos na flash com sucesso!");
      // Usa sua função existente para mandar um feedback legal pro terminal Python
      esp_printf("Sucesso: PIDs salvos na memoria Flash!");
    }
    else
    {
      DEBUG_PRINTLN("Erro ao tentar salvar os parâmetros na flash.");
      esp_printf("Erro: Falha ao salvar na Flash.");
    }
    break;
  }
  case COMANDO_CONFIG_SISTEMA:
  {
    if (pacote.is_set)
    {
      memoria.passo_maximo_pwm = pacote.payload.config_sistema.passo_maximo_pwm;
      memoria.intervalo_rampa_ms = pacote.payload.config_sistema.intervalo_rampa_ms;
      memoria.tempo_ttl_ms = pacote.payload.config_sistema.tempo_ttl_ms;

      // Atualiza os motores em tempo real!
      motor0.passoMaximo = memoria.passo_maximo_pwm;
      motor0.intervaloMs = memoria.intervalo_rampa_ms;
      motor1.passoMaximo = memoria.passo_maximo_pwm;
      motor1.intervaloMs = memoria.intervalo_rampa_ms;
      DEBUG_PRINTLN("Configurações Físicas atualizadas na RAM!");
    }
    // RETORNAR MSG
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
    pareado = true;
    DEBUG_PRINTLN("Dados carregados!");
  }
  else
  {
    pareado = false;
    DEBUG_PRINTLN("Falha na Flash ou Configuração não encontrada. Definindo novos valores...");
    uint8_t mac_novo[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    memcpy(memoria.mac_esp_principal, mac_novo, 6);
    memoria.indice = 255;
    memoria.pid0 = {2.5f, 0.8f, 0.01f, 5.0f};
    memoria.pid1 = {2.5f, 0.8f, 0.01f, 5.0f};
    if (salvar_config(&memoria))
    {
      DEBUG_PRINTLN("Dados salvos com sucesso!");
    }
    else
    {
      DEBUG_PRINTLN("Falha ao salvar.");
    }
  }
  DEBUG_PRINTF("MAC do transmissor: %02X:%02X:%02X:%02X:%02X:%02X\n",
               memoria.mac_esp_principal[0], memoria.mac_esp_principal[1],
               memoria.mac_esp_principal[2], memoria.mac_esp_principal[3],
               memoria.mac_esp_principal[4], memoria.mac_esp_principal[5]);
  DEBUG_PRINTF("ID: %d\n", memoria.indice);
  DEBUG_PRINTLN("Pareado: " + String(pareado));
  DEBUG_PRINTLN("Configurações de PID carregadas:");
  DEBUG_PRINTF("Motor 0 - KP: %.2f, KI: %.2f, KD: %.2f, KF: %.2f\n", memoria.pid0.kp, memoria.pid0.ki, memoria.pid0.kd, memoria.pid0.kf);
  DEBUG_PRINTF("Motor 1 - KP: %.2f, KI: %.2f, KD: %.2f, KF: %.2f\n", memoria.pid1.kp, memoria.pid1.ki, memoria.pid1.kd, memoria.pid1.kf);

  // Configura os motores
  motor0 = criarMotor(PH_IN1, PH_IN2);
  motor1 = criarMotor(PH_IN3, PH_IN4);

  // Configura os encoders e as interrupções
  inicializarEncoder(&encoder0, ENC0_PINA, ENC0_PINB);
  inicializarEncoder(&encoder1, ENC1_PINA, ENC1_PINB);
  attachInterrupt(digitalPinToInterrupt(encoder0.pinA), isr0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1.pinA), isr1, CHANGE);

  // Limites do PWM: no header Motor.h
  // Configura com os valores guardados na memoria
  pid_iniciar(&pidMotor0, memoria.pid0.kp, memoria.pid0.ki, memoria.pid0.kd, memoria.pid0.kf, PID_MIN_PWM, PID_MAX_PWM);
  pid_iniciar(&pidMotor1, memoria.pid1.kp, memoria.pid1.ki, memoria.pid1.kd, memoria.pid1.kf, PID_MIN_PWM, PID_MAX_PWM);
  // pid_iniciar(&pidMotor0, 10.0f, 0.8f, 0.01f, 5.0f, PID_MIN_PWM, PID_MAX_PWM);
  // pid_iniciar(&pidMotor1, 10.0f, 0.8f, 0.01f, 5.0f, PID_MIN_PWM, PID_MAX_PWM);
  // pid_iniciar(&pidMotor0, 30.0f, 0.8f, 0.01f, 10.0f, PID_MIN_PWM, PID_MAX_PWM);
  // pid_iniciar(&pidMotor1, 30.0f, 0.8f, 0.01f, 10.0f, PID_MIN_PWM, PID_MAX_PWM);

  calcular_motor0 = controle_pid_m0;
  calcular_motor1 = controle_pid_m1;

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
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    DEBUG_PRINTLN("Falha ao adicionar o peer de broadcast");
  }

  // Incicializa os timers
  millis_ttl = millis();
  millis_att = millis();
  DEBUG_PRINTLN("Carrinho iniciado com sucesso!");
  // tocarSomMotor(&motor0, 8000, 500);
  // tocarSomMotor(&motor1, 8000, 500);
}

void loop()
{
  unsigned long millis_atual = millis();

  if (millis_atual - millis_att > DELAY)
  {
    // 1. Atualiza a leitura de Variação (Delta) do Encoder
    atualizarDeltaTicks(&encoder0);
    atualizarDeltaTicks(&encoder1);

    // 2. Calcula o PID ou Autotune
    float pwm_saida_0 = calcular_motor0(target_ticks_0, encoder0.delta_ticks);
    float pwm_saida_1 = calcular_motor1(target_ticks_1, encoder1.delta_ticks);

    // 3. Aplica nos motores (assumindo que a sua struct Motor ou biblioteca espera valor com sinal)
    moverMotor(&motor0, (int)pwm_saida_0);
    moverMotor(&motor1, (int)pwm_saida_1);

    // DEBUG_PRINTF("Target Ticks: [%.2f, %.2f] | Medidos: [%ld, %ld] | PWM Saída: [%.2f, %.2f]\n",
    //               target_ticks_0, target_ticks_1, encoder0.delta_ticks, encoder1.delta_ticks, pwm_saida_0, pwm_saida_1);

    millis_att = millis_atual;
  }

  if (ttl_ativo && millis_atual - millis_ttl > tempo_ttl_ms)
  {
    target_ticks_0 = 0;
    target_ticks_1 = 0;
    millis_ttl = millis_atual;
  }
}