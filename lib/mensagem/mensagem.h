#pragma once
#include <Arduino.h>

#define MAX_ROBOS 6
#define TAMANHO_VETOR_ROBOS MAX_ROBOS + 1 // indice 0 é transmissor
#define SENHA_PAREAMENTO "GERVSSS"
#define ID_TRANSMISSOR 0
#define ID_BROADCAST 255

enum tipo_comando
{
  COMANDO_PAREAMENTO = 0x01,
  COMANDO_ID = 0x02,
  COMANDO_MOVIMENTO = 0x03,
  COMANDO_MOVIMENTO_GLOBAL = 0x04,
  COMANDO_PID = 0x05,
  COMANDO_AUTOTUNE_PID = 0x06,
  COMANDO_PRINT = 0x07,
  COMANDO_SALVAR = 0x08,
  COMANDO_CONFIG = 0x09,
  COMANDO_TELEMETRIA = 0x0A,
  COMANDO_PING = 0x0B,
};

typedef struct __attribute__((packed)) mensagem_t
{
  uint8_t tipo;             // O código do tipo_comando (1 byte)
  uint8_t indice_destino;   // Indice do carrinho (1 byte)
  uint8_t indice_remetente; // Indice do carrinho remetente, útil para respostas e comandos globais (1 byte)

  union Payload
  {
    struct __attribute__((packed))
    {
      int16_t target_ticks[2];
    } movimento;

    struct __attribute__((packed))
    {
      int16_t target_ticks[2][TAMANHO_VETOR_ROBOS];
    } movimento_global;

    struct __attribute__((packed))
    {
      uint8_t mac[6];
    } ping;

    struct __attribute__((packed))
    {
      char senha[8];
      uint8_t mac[6];
    } pareamento;

    struct __attribute__((packed))
    {
      uint8_t mac_alvo[6]; // O MAC físico do carrinho que precisa mudar de ID
      uint8_t novo_id;     // O novo ID válido (1 a 6) que ele deve assumir
    } set_id;

    struct __attribute__((packed))
    {
      uint8_t roda; // 0 para Esquerda, 1 para Direita
      float pwm_teste_max;
      float pwm_teste_min;
      uint16_t ciclos;
      int16_t target_ticks;
    } autotune;

    struct __attribute__((packed))
    {
      uint8_t is_set;
      uint8_t roda;
      float kp;
      float ki;
      float kd;
      float kf;
    } pid_config;

    struct __attribute__((packed))
    {
      char texto[28]; // teto da mensagem atual
    } print;

    struct __attribute__((packed))
    {
      uint8_t is_set;
      uint16_t passo_maximo_pwm;
      uint16_t periodo_telemetria_ms;
      uint16_t periodo_ttl_ms;
      uint16_t periodo_controle_ms;
    } config_sistema;

    struct __attribute__((packed))
    {
      uint8_t mac[6];
      int16_t delta_ticks_atual[2];
      int16_t delta_ticks_target[2];
      int8_t rssi_transmissor;
      int8_t noise_floor_transmissor;
      int8_t rssi_carrinho;
      int8_t noise_floor_carrinho;
      uint32_t cnt_pacotes_totais;
      uint32_t cnt_pacotes_validos;
    } telemetria;

  } payload;
} mensagem_t;