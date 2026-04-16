#pragma once
#include <stdint.h>
#include <stdbool.h>


typedef struct {
    float kp;
    float ki;
    float kd;
    float kf;
} PIDParams;

// Defina aqui a sua struct com as variáveis que deseja salvar
typedef struct {
    uint8_t indice;
    uint8_t mac_esp_principal[6];
    PIDParams pid0;
    PIDParams pid1;
} DadosConfig;

// Inicializa a memória flash (deve ser chamada no setup/app_main)
bool inicializar_memoria(void);

// Salva a struct na memória
// Retorna true se salvou com sucesso, false em caso de erro
bool salvar_config(const DadosConfig *config);

// Lê a struct da memória
// Retorna true se leu com sucesso, false se não encontrou ou deu erro
bool carregar_config(DadosConfig *config);