#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include "mensagem.h"
#include "memoria.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#define BROADCAST_ADDRESS (uint8_t[]){0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
#define MAX_LOG_SIZE 256

// Prepara a fila na memória
void despachante_init();

// Chamado dentro da receção (Rápido e sem bloqueios!)
void despachante_enfileirar(mensagem_t msg);

// Substitui o seu printf tradicional. Seguro para callbacks.
void despachante_printf(const char *formato, ...);

// Chamado dentro do loop() (Faz o trabalho pesado)
void despachante_processar_loop();