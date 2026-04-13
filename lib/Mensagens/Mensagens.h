#pragma once
#include <Arduino.h>

/* Define o tipo mensagem 
Importante: deve ser igual para quem envia e recebe
( o outro esp tem que usar a mesma struct de mensagem )
*/
typedef struct mensagem {
  char checkger[7];         // String para checar integridade das mensagens (conteudo sempre eh GERVSS\0)
  int indice_carrinho;      // Indice do carrinho destino/origem
  int velocidadeMotores[2]; // Array com a velocidade alvo dos dois motores
} Mensagem;

/* struct para guardar dados vindos da comunicacao serial */
typedef struct mensagem_serial {
  uint8_t indice_carrinho;  // Indice do carrinho destino/origem
  int8_t velocidades[2];    // Array com as velocidades enviadas via serial
} Mensagem_serial;