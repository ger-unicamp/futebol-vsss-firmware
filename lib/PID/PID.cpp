#include "PID.h"

// ==========================================
// FUNÇÕES DO PID
// ==========================================

void pid_iniciar(PidConfig *pid, float kp, float ki, float kd, float kf, float limite_min, float limite_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kf = kf;
    pid->limite_min = limite_min;
    pid->limite_max = limite_max;
    pid_resetar(pid);
}

void pid_resetar(PidConfig *pid) {
    pid->erro_anterior = 0.0f;
    pid->integral = 0.0f;
    pid->ultimo_tempo_us = micros();
}

float pid_computar(PidConfig *pid, float setpoint_ticks, float valor_medido_ticks) {
    unsigned long agora_us = micros();
    // Converte de microssegundos para segundos
    float dt = (agora_us - pid->ultimo_tempo_us) / 1000000.0f; 
    
    // Evita divisão por zero e estalos na inicialização
    if (dt <= 0.0f || dt > 0.5f) {
        pid->ultimo_tempo_us = agora_us;
        return 0.0f; 
    }

    float erro = setpoint_ticks - valor_medido_ticks;
    
    // Feedforward: Ação preditiva baseada no setpoint desejado
    float FF = pid->kf * setpoint_ticks;

    // Proporcional
    float P = pid->kp * erro;
    
    // Integral (com Anti-Windup)
    // Só acumula integral se a saída não estiver saturada, ou se o erro for no sentido de dessaturar
    pid->integral += erro * dt;
    float I = pid->ki * pid->integral;
    
    if (I > pid->limite_max) pid->integral = pid->limite_max / pid->ki;
    else if (I < pid->limite_min) pid->integral = pid->limite_min / pid->ki;
    
    // Derivada
    float derivada = (erro - pid->erro_anterior) / dt;
    float D = pid->kd * derivada;
    
    // Soma tudo (PID + Feedforward)
    float saida = P + I + D + FF;
    
    // Aplica limites na saída final do PWM (-1023 a 1023)
    if (saida > pid->limite_max) saida = pid->limite_max;
    else if (saida < pid->limite_min) saida = pid->limite_min;
    
    // Atualiza estados para a próxima iteração
    pid->erro_anterior = erro;
    pid->ultimo_tempo_us = agora_us;
    
    return saida;
}

// ==========================================
// FUNÇÕES DE AUTOTUNE (RELÉ PARA MOTORES)
// ==========================================

void autotune_iniciar(PidAutotune *tune, float pwm_teste_max, float pwm_teste_min, int ciclos) {
    tune->ativo = true;
    tune->ciclos_completos = 0;
    tune->ciclos_desejados = ciclos;
    tune->pico_max = -99999.0f;
    tune->pico_min = 99999.0f;
    tune->ultimo_cruzamento_us = micros();
    tune->soma_periodos_us = 0;
    tune->estado_rele = true;
    tune->saida_rele_max = pwm_teste_max;
    tune->saida_rele_min = pwm_teste_min;
}

float autotune_computar(PidAutotune *tune, PidConfig *pid, float setpoint_ticks, float valor_medido_ticks) {
    if (!tune->ativo) return 0.0f;

    unsigned long agora_us = micros();

    if (valor_medido_ticks > tune->pico_max) tune->pico_max = valor_medido_ticks;
    if (valor_medido_ticks < tune->pico_min) tune->pico_min = valor_medido_ticks;

    // Lógica do Relé Bang-Bang
    if (tune->estado_rele && valor_medido_ticks > setpoint_ticks) {
        tune->estado_rele = false;
    } 
    else if (!tune->estado_rele && valor_medido_ticks < setpoint_ticks) {
        tune->estado_rele = true;
        
        unsigned long periodo_atual = agora_us - tune->ultimo_cruzamento_us;
        tune->ultimo_cruzamento_us = agora_us;
        
        if (tune->ciclos_completos > 0) {
            tune->soma_periodos_us += periodo_atual;
        }
        tune->ciclos_completos++;
        
        if (tune->ciclos_completos > tune->ciclos_desejados) {
            float amplitude_oscilacao = (tune->pico_max - tune->pico_min) / 2.0f;
            float amplitude_rele = (tune->saida_rele_max - tune->saida_rele_min) / 2.0f;
            
            // Tu em segundos
            float Tu = (tune->soma_periodos_us / (float)tune->ciclos_desejados) / 1000000.0f;
            float Ku = (4.0f * amplitude_rele) / (PI * amplitude_oscilacao);
            
            // Regras de PI (Ziegler-Nichols modificado para ruído menor - Sem D)
            // Motores costumam ser muito ruidosos em D se a resolução do encoder for baixa.
            pid->kp = 0.45f * Ku;
            pid->ki = (0.54f * Ku) / Tu;
            pid->kd = 0.0f; // Comece com D zerado para motores. Adicione manualmente se oscilar.
            
            tune->ativo = false;
            pid_resetar(pid); 
            return true; 
        }
        
        tune->pico_max = setpoint_ticks;
        tune->pico_min = setpoint_ticks;
    }

    return tune->estado_rele ? tune->saida_rele_max : tune->saida_rele_min;
}