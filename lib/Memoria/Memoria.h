#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float kf;
} PIDParams;

// Defina aqui a sua struct com as variáveis que deseja salvar
typedef struct
{
    uint8_t indice;
    uint8_t mac_esp_principal[6];
    PIDParams pid[2];
    bool pareado;
    uint16_t canal_wifi;
    uint32_t passo_maximo_pwm;
    uint32_t periodo_controle_ms;
    uint32_t periodo_ttl_ms;
    uint32_t periodo_telemetria_ms;
} DadosConfig;

// Inicializa a memória flash (deve ser chamada no setup/app_main)
bool inicializar_memoria(void);

// Salva a struct na memória
// Retorna true se salvou com sucesso, false em caso de erro
bool salvar_config(const DadosConfig *config);

// Lê a struct da memória
// Retorna true se leu com sucesso, false se não encontrou ou deu erro
bool carregar_config(DadosConfig *config);