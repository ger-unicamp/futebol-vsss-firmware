#include "despachante.h"

// Cria uma fila capaz de guardar 10 mensagens em espera
static QueueHandle_t fila_tx;

// Puxa as suas variáveis globais que já existem no main.cpp
extern memoria_t memoria;

// Nova fila para armazenar os textos de debug antes do fatiamento
static QueueHandle_t fila_logs;

void despachante_init()
{
    // Inicializa a fila definindo o tamanho e o tipo de dados
    fila_tx = xQueueCreate(10, sizeof(mensagem_t));

    fila_logs = xQueueCreate(5, MAX_LOG_SIZE);
}

void despachante_enfileirar(mensagem_t msg)
{
    // 1. PREENCHE O QUE NUNCA MUDA!
    msg.indice_destino = ID_TRANSMISSOR;
    msg.indice_remetente = memoria.indice;

    // 2. Envia para a fila usando a função segura para Tasks (tempo de espera = 0)
    // Se a fila estiver cheia (10 pacotes), ele descarta este para não travar o robô
    xQueueSend(fila_tx, &msg, 0);
}

void despachante_printf(const char *formato, ...)
{
    char buffer_local[MAX_LOG_SIZE];

    // 1. Resolve as variáveis (...) na string e guarda no buffer local
    va_list args;
    va_start(args, formato);
    int tamanho = vsnprintf(buffer_local, sizeof(buffer_local), formato, args);
    va_end(args);

    if (tamanho <= 0)
        return;

    // Proteção de segurança
    if (tamanho > sizeof(buffer_local) - 1)
    {
        tamanho = sizeof(buffer_local) - 1;
    }

    // 2. Imprime na Serial normalmente
    // (Serial.printf ou DEBUG_PRINTF, use a sua macro preferida)
    Serial.printf("%s", buffer_local);

    // 3. Joga o texto BRUTO na fila para ser processado depois
    if (memoria.pareado)
    {
        // Usa xQueueSend com tempo 0. Extremamente rápido e não bloqueia callbacks.
        xQueueSend(fila_logs, buffer_local, 0);
    }
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

    // ---------------------------------------------------------
    // ETAPA 2: Esvaziar a fila de logs, fatiar e enviar
    // ---------------------------------------------------------
    char buffer_log[MAX_LOG_SIZE];

    // Tenta puxar um texto grande da fila de logs
    if (xQueueReceive(fila_logs, buffer_log, 0) == pdTRUE)
    {
        int tamanho_total = strlen(buffer_log);
        int bytes_enviados = 0;

        // Pega o teto matemático definido no seu header automaticamente (agora 28 bytes - 1 pro \0)
        int max_chars = sizeof(msg_para_enviar.payload.print.texto) - 1;

        while (bytes_enviados < tamanho_total)
        {
            // Monta o pacote de print
            memset(&msg_para_enviar, 0, sizeof(mensagem_t));
            msg_para_enviar.tipo = COMANDO_PRINT;
            msg_para_enviar.indice_destino = ID_TRANSMISSOR;
            msg_para_enviar.indice_remetente = memoria.indice;

            // Calcula o tamanho deste pedaço específico
            int restante = tamanho_total - bytes_enviados;
            int copiar_agora = (restante > max_chars) ? max_chars : restante;

            // Transfere o pedaço de texto para o pacote
            memcpy(msg_para_enviar.payload.print.texto, buffer_log + bytes_enviados, copiar_agora);
            msg_para_enviar.payload.print.texto[copiar_agora] = '\0'; // Garante o terminador

            // Dispara o rádio diretamente
            esp_now_send(BROADCAST_ADDRESS, (uint8_t *)&msg_para_enviar, sizeof(msg_para_enviar));

            bytes_enviados += copiar_agora;

            // Pequeno delay(1) para dar tempo de o rádio físico limpar o buffer interno
            // e evitar perdas de pacote em rajadas de texto grandes.
            // delay(1);
        }
    }
}