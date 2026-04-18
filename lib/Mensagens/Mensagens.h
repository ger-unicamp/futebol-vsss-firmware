#pragma once
#include <Arduino.h>

#define MAX_ROBOS 6
#define SENHA_PAREAMENTO "GERVSSS"
#define ID_TRANSMISSOR 0
#define ID_BROADCAST 255

void esp_print(const char *formato, ...);

enum TipoComando : uint8_t
{
  COMANDO_MOVIMENTO = 0x01,
  COMANDO_MOVIMENTO_GLOBAL = 0x02,
  COMANDO_ECHO = 0x03,
  COMANDO_SET_ID = 0x04,
  COMANDO_PID_AUTOTUNE = 0x05,
  COMANDO_SET_PID = 0x06,
  COMANDO_GET_PID = 0x07,
  COMANDO_PRINT = 0x08,
  COMANDO_SALVAR = 0x09,
  COMANDO_SET_CONFIG_SISTEMA = 0x0A,
  COMANDO_PAREAMENTO = 0x99
};

typedef struct __attribute__((packed)) Mensagem
{
  uint8_t tipo;             // O código do TipoComando (1 byte)
  uint8_t indice_destino;   // Indice do carrinho (1 byte)
  uint8_t indice_remetente; // Indice do carrinho remetente, útil para respostas e comandos globais (1 byte)

  union Payload
  {
    struct __attribute__((packed))
    {
      int16_t vel_esq;
      int16_t vel_dir;
    } movimento;

    struct __attribute__((packed))
    {
      int16_t vel_esq[MAX_ROBOS];
      int16_t vel_dir[MAX_ROBOS];
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
      float pwm_teste;
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
      char texto[64];
    } print;

    struct __attribute__((packed))
    {
      int passo_maximo_pwm;
      int intervalo_rampa_ms;
      uint32_t tempo_ttl_ms;
    } config_sistema;

  } payload;
} Mensagem;