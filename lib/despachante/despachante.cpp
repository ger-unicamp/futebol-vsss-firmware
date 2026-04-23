#include "despachante.h"

// Cria uma fila capaz de guardar 10 mensagens em espera
static QueueHandle_t fila_tx;

// Puxa as suas variáveis globais que já existem no main.cpp
extern memoria_t memoria;

// Nova fila para armazenar os textos de debug antes do fatiamento
static QueueHandle_t fila_logs;

static bool processando_log = false;        // Estado atual (true = fatiando, false = livre)
static char buffer_log_atual[MAX_LOG_SIZE]; // Guarda a string que está sendo fatiada no momento
static int tamanho_total_log = 0;           // Tamanho total da string atual
static int bytes_enviados_log = 0;          // Contador de quantos bytes já foram enviados

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

    // ---------------------------------------------------------
    // ETAPA 1: Processar a fila de rádio padrão (comandos/telemetria)
    // ---------------------------------------------------------
    // Tenta tirar 1 mensagem da fila. Se não houver nada, sai em 0 milissegundos.
    if (xQueueReceive(fila_tx, &msg_para_enviar, 0) == pdTRUE)
    {
        // O trabalho pesado e demorado de rádio acontece aqui, em segurança.
        esp_now_send(BROADCAST_ADDRESS, (uint8_t *)&msg_para_enviar, sizeof(msg_para_enviar));
    }

    // ---------------------------------------------------------
    // ETAPA 2: Máquina de Estados para fatiar e enviar Logs (1 fatia por ciclo!)
    // ---------------------------------------------------------

    // ESTADO 1: Ocioso. Vamos verificar se tem texto novo na fila de logs.
    if (!processando_log)
    {
        // Puxa um log da fila para a variável estática (memória da máquina de estados)
        if (xQueueReceive(fila_logs, buffer_log_atual, 0) == pdTRUE)
        {
            tamanho_total_log = strlen(buffer_log_atual);
            bytes_enviados_log = 0;
            processando_log = true; // Muda o estado: começaremos a fatiar
        }
    }

    // ESTADO 2: Fatiando. Vamos enviar APENAS UM PEDAÇO por execução desta função.
    if (processando_log)
    {
        // Pega o teto matemático definido no seu header automaticamente (agora 28 bytes - 1 pro \0)
        int max_chars = sizeof(msg_para_enviar.payload.print.texto) - 1;

        // Monta o cabeçalho do pacote
        memset(&msg_para_enviar, 0, sizeof(mensagem_t));
        msg_para_enviar.tipo = COMANDO_PRINT;
        msg_para_enviar.indice_destino = ID_TRANSMISSOR;
        msg_para_enviar.indice_remetente = memoria.indice;

        // Calcula o tamanho do pedaço a ser enviado neste ciclo do loop
        int restante = tamanho_total_log - bytes_enviados_log;
        int copiar_agora = (restante > max_chars) ? max_chars : restante;

        // Transfere o pedaço de texto para o pacote
        memcpy(msg_para_enviar.payload.print.texto, buffer_log_atual + bytes_enviados_log, copiar_agora);
        msg_para_enviar.payload.print.texto[copiar_agora] = '\0'; // Garante o terminador

        // Dispara o rádio diretamente
        esp_now_send(BROADCAST_ADDRESS, (uint8_t *)&msg_para_enviar, sizeof(msg_para_enviar));

        // Avança o cursor do fatiamento
        bytes_enviados_log += copiar_agora;

        // Verifica se terminamos de enviar a string atual
        if (bytes_enviados_log >= tamanho_total_log)
        {
            // Volta para o estado ocioso. No próximo ciclo do loop(),
            // ele vai tentar puxar a próxima mensagem inteira da fila.
            processando_log = false;
        }
    }
}