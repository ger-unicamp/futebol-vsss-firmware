#include "memoria.h"
#include "nvs_flash.h"
#include "nvs.h"

// Nome do "arquivo" e da "chave" interna na memória flash do ESP32
#define ESPACO_NOME "Memoria"
#define CHAVE_DADOS "GERVSSS"

bool inicializar_memoria(void)
{
    esp_err_t erro = nvs_flash_init();

    // Se a memória estiver corrompida ou com versão incompatível, formata e tenta de novo
    if (erro == ESP_ERR_NVS_NO_FREE_PAGES || erro == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        erro = nvs_flash_init();
    }

    return (erro == ESP_OK);
}

bool salvar_config(const memoria_t *config)
{
    nvs_handle_t nvs_handle;
    esp_err_t erro;

    // Abre o espaço de memória para Leitura e Escrita
    erro = nvs_open(ESPACO_NOME, NVS_READWRITE, &nvs_handle);
    if (erro != ESP_OK)
        return false;

    // Salva a struct inteira como um "blob" (Binary Large Object)
    erro = nvs_set_blob(nvs_handle, CHAVE_DADOS, config, sizeof(memoria_t));

    if (erro == ESP_OK)
    {
        // Confirma a gravação física na memória flash
        erro = nvs_commit(nvs_handle);
    }

    // Fecha o acesso à memória
    nvs_close(nvs_handle);

    return (erro == ESP_OK);
}

bool carregar_config(memoria_t *config)
{
    nvs_handle_t nvs_handle;
    esp_err_t erro;

    // Abre o espaço de memória apenas para Leitura
    erro = nvs_open(ESPACO_NOME, NVS_READONLY, &nvs_handle);
    if (erro != ESP_OK)
        return false; // Provavelmente ainda não existe nada salvo

    // Verifica o tamanho e lê os dados
    size_t tamanho_necessario = sizeof(memoria_t);
    erro = nvs_get_blob(nvs_handle, CHAVE_DADOS, config, &tamanho_necessario);

    // Fecha o acesso à memória
    nvs_close(nvs_handle);

    return (erro == ESP_OK);
}