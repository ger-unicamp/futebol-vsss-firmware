#pragma once
#include <Arduino.h>

#define MAX_ROBOS 6
#define SENHA_PAREAMENTO "GERVSSS"
#define ID_TRANSMISSOR 0
#define ID_BROADCAST 255

enum TipoComando : uint8_t
{
  COMANDO_MOVIMENTO = 0x01,
  COMANDO_MOVIMENTO_GLOBAL = 0x02,
  COMANDO_ECHO = 0x03,
  COMANDO_PAREAMENTO = 0x99
  // Pode adicionar CMD_TUNING_PID, etc.
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
    } pareamento;

    struct __attribute__((packed))
    {
      uint8_t rssi;
    } echo;

  } payload;
} Mensagem;