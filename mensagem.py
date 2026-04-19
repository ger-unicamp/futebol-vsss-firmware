import serial
import struct
import threading
import time
import re
from cffi import FFI
import serial.tools.list_ports

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

    def _enviar_mensagem(self, tipo_comando, destino, is_set, payload_setter=None):
        msg = self.ffi.new("Mensagem *")
        
        # 1. Preenche o cabeçalho
        msg.tipo = self.enums[tipo_comando]
        msg.indice_destino = destino
        msg.indice_remetente = self.defines.get('ID_TRANSMISSOR', 0)
        
        # 2. A Mágica: Define se é operação de Gravar (True) ou Ler (False)
        msg.is_set = is_set 

        # 3. Preenche o payload específico, se houver
        if payload_setter:
            payload_setter(msg.payload)

        # 4. Empacota e envia via Serial
        msg_bytes = bytes(self.ffi.buffer(msg))
        tamanho = len(msg_bytes)

        if tamanho > 250:
            raise ValueError("O Mensagem excede o limite de 250 bytes do ESP-NOW.")

        sync_bytes = struct.pack('BBB', self.SYNC_1_TX, self.SYNC_2_TX, tamanho)
        
        try:
            self.serial.write(sync_bytes + msg_bytes)
        except serial.SerialException:
            print("Erro ao enviar dados pela serial.")

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
        def set_payload(payload):
            senha = self.defines.get('SENHA_PAREAMENTO', b'GERVSSS')
            if isinstance(senha, str):
                senha = senha.encode('utf-8')
            for i, b in enumerate(senha[:8]):
                payload.pareamento.senha[i] = bytes([b])
                
        print("\n[--> SEND] Enviando Comando de PAREAMENTO...")
        self._enviar_mensagem('COMANDO_PAREAMENTO', self.defines.get('ID_BROADCAST', 255), False, set_payload)

    def enviar_echo(self):
        # Apenas leitura (GET). Não precisa de payload.
        self.timestamp_ultimo_echo = time.time() # Marca o tempo antes de enviar para medir o RTT depois
        self._enviar_mensagem('COMANDO_ID', self.defines.get('ID_BROADCAST', 255), False)

    def enviar_movimento(self, indice, vel_esq, vel_dir):
        """Envia o comando de Movimento simples."""
        def set_payload(payload):
            payload.movimento.vel_esq = vel_esq
            payload.movimento.vel_dir = vel_dir
            
        print(f"\n[--> SEND] Enviando MOVIMENTO (Robô {indice}) | Esq: {vel_esq}, Dir: {vel_dir}...")
        self._enviar_mensagem('COMANDO_MOVIMENTO', indice, True, set_payload)

    def enviar_set_id(self, mac_bytes, novo_id):
        # Escrita (SET). Preenche o MAC alvo e o novo ID.
        def set_payload(payload):
            for i in range(6):
                payload.set_id.mac_alvo[i] = mac_bytes[i]
            payload.set_id.novo_id = novo_id
            
        self._enviar_mensagem('COMANDO_ID', self.defines.get('ID_BROADCAST', 255), True, set_payload)

    def enviar_get_pid(self, robo_id, roda):
        # Leitura (GET). Informa apenas a roda desejada.
        def set_payload(payload):
            payload.pidconfig.roda = roda
            
        self._enviar_mensagem('COMANDO_PID', robo_id, False, set_payload)

    def enviar_set_pid(self, robo_id, roda, kp, ki, kd, kf=0.0):
        # Escrita (SET). Informa a roda e todos os ganhos.
        def set_payload(payload):
            payload.pidconfig.roda = roda
            payload.pidconfig.kp = kp
            payload.pidconfig.ki = ki
            payload.pidconfig.kd = kd
            payload.pidconfig.kf = kf
            
        self._enviar_mensagem('COMANDO_PID', robo_id, True, set_payload)

    def enviar_get_config(self, robo_id):
        # Leitura (GET). Sem payload.
        self._enviar_mensagem('COMANDO_CONFIG_SISTEMA', robo_id, False)

    def enviar_set_config(self, robo_id, passo_max, intervalo, ttl):
        # Escrita (SET). Preenche os parâmetros da rampa.
        def set_payload(payload):
            payload.config_sistema.passo_maximo_pwm = passo_max
            payload.config_sistema.intervalo_rampa_ms = intervalo
            payload.config_sistema.tempo_ttl_ms = ttl
        self._enviar_mensagem('COMANDO_CONFIG_SISTEMA', robo_id, True, set_payload)


    def enviar_autotune(self, robo_id, roda, pwm_max, pwm_min, ciclos, target_ticks):
        # Ação/Gatilho. O "is_set" é ignorado pelo ESP32 (mando False por padrão).
        def set_payload(payload):
            payload.autotune.roda = roda
            payload.autotune.pwm_teste_max = pwm_max
            payload.autotune.pwm_teste_min = pwm_min
            payload.autotune.ciclos = ciclos
            payload.autotune.target_ticks = target_ticks
            
        self._enviar_mensagem('COMANDO_AUTOTUNE_PID', robo_id, False, set_payload)

    def enviar_config_telemetria(self, robo_id, intervalo_ms):
        """
        Liga ou desliga a telemetria. 
        intervalo_ms = 0 desliga.
        """
        def set_payload(payload):
            payload.telemetria.intervalo = intervalo_ms
        self._enviar_mensagem('COMANDO_TELEMETRIA', robo_id, True, set_payload)
        status = f"em {intervalo_ms}ms" if intervalo_ms > 0 else "DESLIGADA"
        print(f"[*] Solicitando Telemetria {status} para o Robô {robo_id}")

    def enviar_salvar(self, robo_id):
        self._enviar_mensagem('COMANDO_SALVAR', robo_id, False)

    def _maquina_estados_rx(self):
        """Máquina de estados idêntica a do main.cpp, focada na recepção (RX)."""
        ESTADO_WAIT_SYNC1 = 0
        ESTADO_WAIT_SYNC2 = 1
        ESTADO_WAIT_LENGTH = 2
        ESTADO_READ_PAYLOAD = 3

        estado = ESTADO_WAIT_SYNC1
        tamanho_payload = 0
        buffer_payload = bytearray()


        print("[*] Thread de RX iniciada em background!")
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
                        # print(f"\n[RX DEBUG] Pacote RAW recebido ({tamanho_payload} bytes): {bytes(buffer_payload).hex()}")
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
            if msg.tipo in (self.enums.get('COMANDO_ID'), self.enums.get('COMANDO_PAREAMENTO')):
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

            elif msg.tipo == self.enums.get('COMANDO_PID'):
                cfg = msg.payload.pidconfig
                print(f"\n[<-- RECV PID] Robô {msg.indice_remetente} | Roda {cfg.roda} | Kp: {cfg.kp:.3f}, Ki: {cfg.ki:.3f}, Kd: {cfg.kd:.3f}, Kf: {cfg.kf:.3f}")

            # --- 2. RECEBIMENTO DO PRINT (MENSAGENS DE TEXTO) ---
            elif msg.tipo == self.enums.get('COMANDO_PRINT'):
                # Pega a mensagem inteira como bytes crus e pula os 4 bytes do cabeçalho
                raw_bytes = bytes(self.ffi.buffer(msg))[4:]
            
                # Corta a string no primeiro '\x00' (Null Byte) e transforma em texto
                texto = raw_bytes.split(b'\x00')[0].decode('utf-8', errors='ignore')
            
                print(f"\n[ROBÔ {msg.indice_remetente} LOG]: {texto}")

            elif msg.tipo == self.enums.get('COMANDO_MOVIMENTO'):
                # O CFFI já converte os int16_t do C para inteiros normais no Python!
                vel_esq = msg.payload.movimento.vel_esq
                vel_dir = msg.payload.movimento.vel_dir
            
                # Avisa se é um retorno de estado (is_set falso) ou um espelho de comando enviado
                tipo_acao = "Feedback Atual" if not msg.is_set else "Comando Espelhado"
            
                print(f"\n[ROBÔ {msg.indice_remetente} MOVIMENTO] {tipo_acao} -> Roda Esq: {vel_esq} | Roda Dir: {vel_dir}")

            elif msg.tipo == self.enums.get('COMANDO_TELEMETRIA'):
                esq_atual = msg.payload.telemetria.vel_esq_atual
                dir_atual = msg.payload.telemetria.vel_dir_atual
                esq_target = msg.payload.telemetria.vel_esq_target
                dir_target = msg.payload.telemetria.vel_dir_target
            
                print(f"[ROBÔ {msg.indice_remetente} TELEMETRIA] Esq: {esq_atual}/{esq_target} | Dir: {dir_atual}/{dir_target}")
            else:
                print(f"\n[<-- RECV] Mensagem do Robô {msg.indice_remetente} | Tipo Comando: 0x{msg.tipo:02X} | Payload bruto: {raw_bytes.hex()}")

            
                
        except Exception as e:
            print(f"Erro ao processar pacote recebido: {e} | Dados brutos: {raw_bytes}")

    def fechar(self):
        self.rodando = False
        self.thread_rx.join()
        self.serial.close()


def localizar_transmissor():
    """Procura automaticamente pela porta serial do ESP32 Transmissor"""
    portas = serial.tools.list_ports.comports()
    
    # Lista de padrões comuns de descrição/ID para ESP32
    # CP210x, CH340 e o JTAG nativo do ESP32-S3/C3 (ACM)
    # padroes = ["CP210", "CH340", "USB Serial", "S3", "ACM", "UART"]
    padroes = ["ACM"]
    
    for p in portas:
        for padrao in padroes:
            if padrao.upper() in p.description.upper() or padrao.upper() in p.device.upper():
                print(f"[*] Transmissor encontrado: {p.device} ({p.description})")
                return p.device
    return None

# ==========================================
# Terminal Interativo
# ==========================================
if __name__ == "__main__":
    # --- AUTO DETECÇÃO DA PORTA ---
    PORTA_SERIAL = localizar_transmissor()
    
    if not PORTA_SERIAL:
        print("❌ Erro: Nenhum ESP32 encontrado. Verifique o cabo USB!")
        exit(1)
    # ------------------------------
    print(f"[*] Conectando ao Transmissor na porta {PORTA_SERIAL}...")
    BAUD_RATE = 115200
    CAMINHO_HEADER = './lib/Mensagens/Mensagens.h'

    esp = PonteESP32(PORTA_SERIAL, BAUD_RATE, CAMINHO_HEADER)
    time.sleep(2)

    print("\n" + "="*40)
    print("🚗 TERMINAL VSSS INICIADO")
    print("Comandos disponíveis:")
    print(" - echo")
    print(" - parear")
    print(" - get_pid <robo_id> <roda_0_ou_1>")
    print(" - set_pid <robo_id> <roda> <kp> <ki> <kd>")
    print(" - autotune <robo_id> <roda> <pwm_max> <pwm_min> <ciclos> <target_ticks>")
    print(" - salvar <robo_id>") # Nova linha
    print(" - sair")
    print("="*40 + "\n")

    try:
        while True:
            comando_raw = input("vsss> ").strip().split()
            if not comando_raw:
                continue

            cmd = comando_raw[0].lower()

            if cmd == "sair":
                break
            elif cmd == "echo":
                esp.enviar_echo()
            elif cmd == "parear":
                esp.enviar_pareamento()
            elif cmd == "get_pid":
                if len(comando_raw) >= 3:
                    esp.enviar_get_pid(int(comando_raw[1]), int(comando_raw[2]))
                else:
                    print("Uso: get_pid <robo_id> <roda>")
            elif cmd == "set_pid":
                if len(comando_raw) >= 6:
                    robo, roda, p, i, d = map(float, comando_raw[1:6])
                    esp.enviar_set_pid(int(robo), int(roda), p, i, d)
                    print(f"[*] PID enviado para o Robô {int(robo)}.")
                else:
                    print("Uso: set_pid <robo_id> <roda> <kp> <ki> <kd>")
            elif cmd == "autotune":
                if len(comando_raw) >= 7:
                    robo = int(comando_raw[1])
                    roda = int(comando_raw[2])
                    pwm_max = float(comando_raw[3])
                    pwm_min = float(comando_raw[4])
                    ciclos = int(comando_raw[5])
                    target_ticks = int(comando_raw[6])
                    
                    esp.enviar_autotune(robo, roda, pwm_max, pwm_min, ciclos, target_ticks)
                    print(f"[*] Iniciando Autotune na roda {roda} com PWM máximo {pwm_max}, mínimo {pwm_min}, {ciclos} ciclos e target de ticks {target_ticks}.")
                else:
                    print("Uso: autotune <robo_id> <roda> <pwm_max> <pwm_min> <ciclos> <target_ticks>")
            elif cmd == "salvar":
                if len(comando_raw) >= 2:
                    robo_id = int(comando_raw[1])
                    esp.enviar_salvar(robo_id)
                    print(f"[*] Mandando Robô {robo_id} salvar configurações na Flash...")
                else:
                    print("Uso: salvar <robo_id>")
            elif cmd == "telemetria":
                if len(comando_raw) >= 3:
                    robo_id = int(comando_raw[1])
                    ms = int(comando_raw[2])
                    esp.enviar_config_telemetria(robo_id, ms)
                else:
                    print("Uso: telemetria <id_robo> <intervalo_ms (0 para desligar)>")
            else:
                print(f"Comando desconhecido: {cmd}")

    except KeyboardInterrupt:
        print("\nSaindo...")
    finally:
        esp.fechar()
        print("\nTerminal encerrado e porta fechada com segurança.")