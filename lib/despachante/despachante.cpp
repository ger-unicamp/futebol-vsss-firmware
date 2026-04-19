#include "despachante.h"

// Cria uma fila capaz de guardar 10 mensagens em espera
static QueueHandle_t fila_tx;

// Puxa as suas variáveis globais que já existem no main.cpp
extern memoria_t memoria;

void despachante_init()
{
    // Inicializa a fila definindo o tamanho e o tipo de dados
    fila_tx = xQueueCreate(10, sizeof(mensagem_t));
}

void despachante_enfileirar(mensagem_t msg)
{
    // 1. PREENCHE O QUE NUNCA MUDA!
    msg.indice_destino = ID_TRANSMISSOR;
    msg.indice_remetente = memoria.indice;
    msg.is_set = false;

    // 2. Envia para a fila usando a função segura para Tasks (tempo de espera = 0)
    // Se a fila estiver cheia (10 pacotes), ele descarta este para não travar o robô
    xQueueSend(fila_tx, &msg, 0);
}

void despachante_processar_loop()
{
    mensagem_t msg_para_enviar;

    // 3. Tenta tirar 1 mensagem da fila. Se não houver nada, sai em 0 milissegundos!
    if (xQueueReceive(fila_tx, &msg_para_enviar, 0) == pdTRUE)
    {
        // O trabalho pesado e demorado de rádio acontece aqui, em segurança.
        esp_now_send(BROADCAST_ADDRESS, (uint8_t *)&msg_para_enviar, sizeof(msg_para_enviar));
    }
}