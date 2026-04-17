import serial
import struct
import threading
import time
import re
from cffi import FFI

class PonteESP32:
    def __init__(self, porta, baudrate, header_path):
        # 1. Analisa e carrega as definições do Mensagens.h
        self.ffi, self.defines, self.enums = self._parse_header(header_path)
        
        # 2. Inicia a conexão Serial
        self.serial = serial.Serial(porta, baudrate, timeout=0.1)
        
        # Cabeçalhos de Sincronização (Python -> ESP32)
        self.SYNC_1_TX = 0xAA
        self.SYNC_2_TX = 0x55
        
        # Cabeçalhos de Sincronização (ESP32 -> Python)
        self.SYNC_1_RX = 0xBB
        self.SYNC_2_RX = 0x66

        self.MAX_ROBOS = 6
        self.robos_ativos = {} # Formato: {"MAC_STR": {"mac_bytes": [...], "id": 1}}
        self.timestamp_ultimo_echo = 0.0
        
        # 3. Inicia a máquina de estados receptora em uma thread separada
        self.rodando = True
        self.thread_rx = threading.Thread(target=self._maquina_estados_rx, daemon=True)
        self.thread_rx.start()
        print(f"[*] Conectado na porta {porta}. Aguardando sincronização...")

    def _parse_header(self, header_path):
        """Lê o arquivo .h e o prepara para a biblioteca CFFI."""
        with open(header_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # ========================================================
        # 1. REMOÇÃO DE COMENTÁRIOS
        # ========================================================
        # Remove comentários de bloco (/* ... */)
        content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
        # Remove comentários de linha (// ...)
        content = re.sub(r'//.*', '', content)

        # ========================================================
        # 2. EXTRAÇÃO DE DEFINES E ENUMS PRO PYTHON
        # ========================================================
        # Extrair #defines para variáveis normais no Python
        defines = {}
        for match in re.finditer(r'#define\s+(\w+)\s+([^\n\r]+)', content):
            val = match.group(2).strip('"').strip()
            try: val = int(val, 0) # Converte se for número
            except ValueError: pass
            defines[match.group(1)] = val

        # Extrair valores do enum TipoComando para o Python
        enums = {}
        enum_block = re.search(r'enum\s+TipoComando.*?\{(.*?)\}', content, re.DOTALL)
        if enum_block:
            for line in enum_block.group(1).split(','):
                if '=' in line:
                    k, v = line.split('=')
                    # Agora v.strip() conterá apenas o número, pois os comentários sumiram
                    enums[k.strip()] = int(v.strip(), 0)

        # ========================================================
        # 3. LIMPEZA E FORMATAÇÃO PARA O CFFI
        # ========================================================
        # Limpar o código C para a compreensão estrita do CFFI
        content = re.sub(r'#.*', '', content) # Remove macros
        content = content.replace('__attribute__((packed))', '') 
        content = re.sub(r'enum\s+TipoComando\s*:\s*uint8_t', 'enum TipoComando', content)

        # Substitui as macros pelos seus valores numéricos reais no texto
        for nome_macro, valor_macro in defines.items():
            content = re.sub(rf'\b{nome_macro}\b', str(valor_macro), content)

        ffi = FFI()
        
        declaracoes_base = """
            typedef signed char int8_t;
            typedef unsigned char uint8_t;
            typedef short int16_t;
            typedef unsigned short uint16_t;
            typedef int int32_t;
            typedef unsigned int uint32_t;
        """
        
        # Carrega as estruturas com pack=1
        ffi.cdef(declaracoes_base + content, pack=1)
        
        return ffi, defines, enums

    def criar_mensagem(self):
        """Retorna uma nova instância vazia da struct Mensagem."""
        return self.ffi.new("Mensagem *")

    def enviar_mensagem(self, msg_cdata):
        """Empacota a struct compilada e envia via Serial seguindo o protocolo TX."""
        # Converte a struct de volta para uma cadeia crua de bytes
        payload_bytes = bytes(self.ffi.buffer(msg_cdata))
        tamanho = len(payload_bytes)

        if tamanho > 250:
            raise ValueError("O Payload excede o limite de 250 bytes do ESP-NOW.")

        # Constrói o cabeçalho: 0xAA, 0x55, TAMANHO, PAYLOAD
        pacote = struct.pack('BB B', self.SYNC_1_TX, self.SYNC_2_TX, tamanho) + payload_bytes
        self.serial.write(pacote)

    def organizar_ids(self):
        """
        Analisa os robôs que responderam ao ECHO.
        Identifica IDs duplicados ou não configurados (255) e despacha novos IDs.
        """
        print("\n--- [DHCP VSSS] Analisando Conflitos de Rede ---")
        
        ids_ocupados_validos = set()
        macs_precisando_id = []

        # 1. Varre todos os robôs conhecidos para separar quem tá OK de quem precisa de ID
        for mac_str, info in self.robos_ativos.items():
            id_atual = info['id']
            
            # Se for ID de fábrica (255), ID inválido (0) ou maior que o permitido
            if id_atual == 255 or id_atual == 0 or id_atual > self.MAX_ROBOS:
                macs_precisando_id.append((mac_str, info['mac_bytes']))
            
            # Se o ID for válido, mas JÁ ESTÁ na lista de ocupados (Duplicata!)
            elif id_atual in ids_ocupados_validos:
                macs_precisando_id.append((mac_str, info['mac_bytes']))
            
            # Se o ID for válido e for o primeiro a reivindicá-lo, tá liberado
            else:
                ids_ocupados_validos.add(id_atual)

        # Se ninguém precisa de ID novo, encerra aqui
        if not macs_precisando_id:
            print("Todos os robôs estão com IDs únicos e válidos. Nenhuma ação necessária.")
            return

        # 2. Descobre quais IDs de 1 a MAX_ROBOS estão sobrando
        ids_livres = [i for i in range(1, self.MAX_ROBOS + 1) if i not in ids_ocupados_validos]

        # 3. Atribui os IDs livres para a fila de MACs problemáticos
        for mac_str, mac_bytes in macs_precisando_id:
            if not ids_livres:
                print(f"[ALERTA] Acabaram os IDs livres! O robô {mac_str} ficará sem ID.")
                break
                
            novo_id = ids_livres.pop(0) # Pega o primeiro ID livre da lista
            
            # Chama a função que já criamos para disparar o comando
            self.enviar_set_id(mac_bytes, novo_id)
            
            # Atualiza o dicionário local para refletir a mudança
            self.robos_ativos[mac_str]['id'] = novo_id 
            print(f"[*] Robô {mac_str} reconfigurado para o ID {novo_id}")
            
        print("------------------------------------------------\n")


    # ==========================================
    # MÉTODOS AUXILIARES DE COMANDOS
    # ==========================================
    def enviar_pareamento(self):
        """Envia o comando de pareamento usando a senha definida no Mensagens.h."""
        msg = self.criar_mensagem()
        msg.tipo = self.enums['COMANDO_PAREAMENTO']
        msg.indice_destino = 255 # Indiferente para broadcast de pareamento
        
        # Pega a senha do header (ou usa o padrão se não encontrar) e injeta na struct
        senha = self.defines.get('SENHA_PAREAMENTO', 'GERVSSS')
        msg.payload.pareamento.senha = senha.encode('utf-8')
        
        print(f"\n[--> SEND] Enviando Comando de PAREAMENTO (Senha: {senha})...")
        self.enviar_mensagem(msg)

    def enviar_echo(self, indice_alvo=255):
        """Envia o comando de ECHO para verificar quais robôs estão vivos."""
        msg = self.criar_mensagem()
        msg.tipo = self.enums['COMANDO_ECHO']
        msg.indice_destino = indice_alvo

        # Marca o tempo em segundos (com precisão de microsegundos) ANTES de enviar
        self.timestamp_ultimo_echo = time.time()
        
        print(f"\n[--> SEND] Enviando Comando de ECHO...")
        self.enviar_mensagem(msg)

    def enviar_movimento(self, indice, vel_esq, vel_dir):
        """Envia o comando de Movimento simples."""
        msg = self.criar_mensagem()
        msg.tipo = self.enums['COMANDO_MOVIMENTO']
        msg.indice_destino = indice
        msg.payload.movimento.vel_esq = vel_esq
        msg.payload.movimento.vel_dir = vel_dir
        
        print(f"\n[--> SEND] Enviando MOVIMENTO (Robô {indice}) | Esq: {vel_esq}, Dir: {vel_dir}...")
        self.enviar_mensagem(msg)

    def enviar_set_id(self, mac_alvo_bytes, novo_id):
        """
        Envia o comando para um carrinho específico (identificado pelo MAC) 
        assumir um novo ID.
        """
        msg = self.criar_mensagem()
        msg.tipo = self.enums.get('COMANDO_SET_ID', 0x04)
        msg.indice_remetente = 0   # É o TX enviando
        msg.indice_destino = 255   # Broadcast lógico (todos ouvem, mas só o alvo obedece)

        # Copia o array de bytes do MAC para a struct
        for i in range(6):
            msg.payload.set_id.mac_alvo[i] = mac_alvo_bytes[i]
            
        msg.payload.set_id.novo_id = novo_id

        print(f"[--> SEND] Mandando MAC {':'.join([f'{b:02X}' for b in mac_alvo_bytes])} assumir ID {novo_id}")
        self.enviar_mensagem(msg)

    def _maquina_estados_rx(self):
        """Máquina de estados idêntica a do main.cpp, focada na recepção (RX)."""
        ESTADO_WAIT_SYNC1 = 0
        ESTADO_WAIT_SYNC2 = 1
        ESTADO_WAIT_LENGTH = 2
        ESTADO_READ_PAYLOAD = 3

        estado = ESTADO_WAIT_SYNC1
        tamanho_payload = 0
        buffer_payload = bytearray()

        while self.rodando:
            if self.serial.in_waiting > 0:
                byte_lido = self.serial.read(1)[0]

                if estado == ESTADO_WAIT_SYNC1:
                    if byte_lido == self.SYNC_1_RX:
                        estado = ESTADO_WAIT_SYNC2

                elif estado == ESTADO_WAIT_SYNC2:
                    if byte_lido == self.SYNC_2_RX:
                        estado = ESTADO_WAIT_LENGTH
                    else:
                        estado = ESTADO_WAIT_SYNC1

                elif estado == ESTADO_WAIT_LENGTH:
                    tamanho_payload = byte_lido
                    buffer_payload.clear()
                    if 0 < tamanho_payload <= 250:
                        estado = ESTADO_READ_PAYLOAD
                    else:
                        estado = ESTADO_WAIT_SYNC1

                elif estado == ESTADO_READ_PAYLOAD:
                    buffer_payload.append(byte_lido)
                    if len(buffer_payload) >= tamanho_payload:
                        # Passa os bytes brutos para o CFFI reconstruir a struct
                        self._processar_mensagem_recebida(bytes(buffer_payload))
                        estado = ESTADO_WAIT_SYNC1
            else:
                time.sleep(0.001) # Evita travamento da CPU se a serial estiver vazia

    def _processar_mensagem_recebida(self, raw_bytes):
        """Mapeia os bytes recebidos. Agora tudo usa a struct Mensagem de forma padronizada."""
        try:
            # Transforma os bytes brutos novamente em objeto usando a struct em C
            msg = self.ffi.from_buffer("Mensagem *", raw_bytes)
            
            # Trata a resposta do comando de ECHO
            if msg.tipo == self.enums.get('COMANDO_ECHO', 0x03):
                # 1. Para o cronômetro e calcula o RTT em milissegundos
                tempo_atual = time.time()
                ping_ms = (tempo_atual - self.timestamp_ultimo_echo) * 1000.0
                
                # 2. Extrai os dados que já havíamos configurado
                mac_bytes = msg.payload.echo.mac
                mac_str = ":".join([f"{b:02X}" for b in mac_bytes])
                id_recebido = msg.indice_remetente
                rssi = msg.payload.echo.rssi - 256

                # 3. Salva no "DHCP"
                self.robos_ativos[mac_str] = {
                    "mac_bytes": list(mac_bytes),
                    "id": id_recebido
                }
                
                # 4. Imprime o log com o novo medidor de Ping!
                print(f"[<-- RECV ECHO] MAC: {mac_str} | ID: {id_recebido} | RSSI: {rssi} dBm | Ping: {ping_ms:.1f} ms")
                return

            # 2. Trata comandos de Movimento (se houver alguma confirmação)
            elif msg.tipo == self.enums.get('COMANDO_MOVIMENTO', 0x01):
                print(f"\n[<-- RECV] Mensagem do Robô {msg.indice} | Tipo Comando: 0x{msg.tipo:02X}")
                print(f"    Velocidade Esq: {msg.payload.movimento.vel_esq}")
                print(f"    Velocidade Dir: {msg.payload.movimento.vel_dir}")

            
                
        except Exception as e:
            print(f"Erro ao processar pacote recebido: {e} | Dados brutos: {raw_bytes}")

    def fechar(self):
        self.rodando = False
        self.thread_rx.join()
        self.serial.close()


# ==========================================
# Exemplo de Uso
# ==========================================
if __name__ == "__main__":
    # Ajuste para a porta COM correta (ex: 'COM3' no Windows ou '/dev/ttyUSB0' no Linux)
    PORTA_SERIAL = '/dev/ttyACM0' 
    BAUD_RATE = 115200
    CAMINHO_HEADER = './lib/Mensagens/Mensagens.h'

    # Inicia a ponte
    esp = PonteESP32(PORTA_SERIAL, BAUD_RATE, CAMINHO_HEADER)
    time.sleep(2) # Tempo para o ESP32 resetar ao abrir a Serial

    try:    
        # 1. Testa Pareamento
        esp.enviar_pareamento()
        time.sleep(1)

        # 2. Testa Echo
        esp.enviar_echo()
        time.sleep(1.5)

        esp.organizar_ids()

        # 3. Testa Movimento
        esp.enviar_movimento(indice=1, vel_esq=255, vel_dir=-255)

        # Mantém vivo para escutar as respostas
        while True:
            esp.enviar_echo()
            time.sleep(0.016666)

    except KeyboardInterrupt:
        print("\nFechando conexão...")
        esp.fechar()