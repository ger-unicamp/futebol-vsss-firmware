#pragma once
#include <Arduino.h>

#define MAX_ROBOS 6
#define SENHA_PAREAMENTO "GERVSSS"
#define ID_TRANSMISSOR 0
#define ID_BROADCAST 255

void esp_print(const char *formato, ...);

enum TipoComando : uint8_t
{
  COMANDO_PAREAMENTO = 0x01,
  COMANDO_ID = 0x02,
  COMANDO_MOVIMENTO = 0x03,
  COMANDO_MOVIMENTO_GLOBAL = 0x04,
  COMANDO_PID = 0x05,
  COMANDO_AUTOTUNE_PID = 0x06,
  COMANDO_PRINT = 0x07,
  COMANDO_SALVAR = 0x08,
  COMANDO_CONFIG_SISTEMA = 0x09,
  COMANDO_TELEMETRIA = 0x0A,
};

typedef struct __attribute__((packed)) Mensagem
{
  uint8_t tipo;             // O código do TipoComando (1 byte)
  uint8_t indice_destino;   // Indice do carrinho (1 byte)
  uint8_t indice_remetente; // Indice do carrinho remetente, útil para respostas e comandos globais (1 byte)
  uint8_t is_set;

  union Payload
  {
    struct __attribute__((packed))
    {
      int16_t target_ticks[2];
    } movimento;

    struct __attribute__((packed))
    {
      int16_t target_ticks[2][MAX_ROBOS];
    } movimento_global;

    struct __attribute__((packed))
    {
      char senha[8];
      uint8_t mac[6];
    } pareamento;

    struct __attribute__((packed))
    {
      uint8_t rssi;
      uint8_t mac[6];
    } echo;

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
      int ciclos;
      int target_ticks;
    } autotune;

    struct __attribute__((packed))
    {
      uint8_t roda;
      float kp;
      float ki;
      float kd;
      float kf;
    } pidconfig;

    struct __attribute__((packed))
    {
      char texto[24]; // para nao aumentar o tamnho da mensagem, limitamos a 24 chars
    } print;

    struct __attribute__((packed))
    {
      int passo_maximo_pwm;
      uint32_t periodo_ttl_ms;
    } config_sistema;

    struct __attribute__((packed))
    {
      uint16_t periodo_telemetria_ms;
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
} Mensagem;