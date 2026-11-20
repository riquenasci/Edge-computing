/*
 * FLUXO SEGURO - Monitoramento preditivo de alagamentos
 * Global Solution FIAP 2026.2 (CitizenTech) - Equipe Core (1ESPA)
 *
 * Integrantes:
 *   Eduardo Mororo - RM575543
 *   Flavia         - RM575681
 *   Leandro        - RM569915
 *   Rick           - RM576619
 *   Vitor          - RM575857
 *
 * O ESP32 mede a altura da agua no bueiro (HC-SR04), decide o nivel de risco
 * e avisa no proprio local (LEDs, buzzer e OLED), mesmo sem internet.
 * Com Wi-Fi, tambem manda os dados para o servidor do site.
 *
 * Estados (% da altura util do bueiro; justificativa em "Niveis", abaixo):
 *   NORMAL      menos de 40%  LED verde
 *   ATENCAO     40% a 60%     LED amarelo pisca + bip (1 vez a cada 3 s)
 *   PERIGO      60% a 90%     LED vermelho piscando + buzzer
 *   EMERGENCIA  90% ou mais   LED vermelho rapido + buzzer rapido
 *   FALHA       sensor sem resposta: LEDs apagados e aviso no OLED
 * Filtros: media movel de 5 leituras, descarte de picos e confirmacao em 3.
 * Previsao: velocidade de subida da agua + tabela de tempo ate alagar.
 * Chuva e pos-chuva:
 *   na chuva, se a previsao e de alagamento, pede manutencao para a central;
 *   depois da chuva (menu "Chuva: Nao"), mede o lixo que ficou no bueiro
 *   e pede limpeza a partir de 25% (LED amarelo pisca 1 vez a cada 3 s).
 * Botao (com o menu fechado): 1 clique abre o menu (chuva e idioma),
 *   2 cliques trocam portugues/ingles e 3 cliques ligam/desligam a chuva.
 *   No menu: clique muda o valor; segurar 1 s desce a linha.
 * Idiomas da tela: portugues, espanhol e ingles (o site recebe sempre em portugues).
 *
 * Ligacoes: HC-SR04 TRIG D23 e ECHO D22 (com divisor de tensao, ECHO e 5 V)
 *           OLED SDA D4 e SCL D15 | LEDs D5, D18, D19 | buzzer D21
 *           botao GPIO17 (TX2) com pull-up interno
 * Bibliotecas: Adafruit SSD1306, Adafruit GFX, Adafruit BusIO
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

// ============================================================================
//  CONFIGURACOES
// ============================================================================

// false = Wokwi (simulador) | true = maquete impressa em 3D
const bool MAQUETE_3D = false;

// Distancia do sensor ate a agua, em cm (Wokwi). Na maquete 3D: 12, 4 e 1,5 (veja o setup)
float distanciaVaziaCm = 100.0;           // bueiro vazio
float distanciaTransbordaCm = 20.0;       // agua na boca do bueiro
float toleranciaCalibracaoCm = 5.0;

// Ao ligar, mede o bueiro vazio e usa no lugar de distanciaVaziaCm (se estiver perto)
const bool CALIBRAR_AO_LIGAR = true;

// Niveis, em % da altura util. Por que estes limites (tabela de previsao, abaixo):
//   40% -> ~15 min para alagar: da tempo de avisar (abaixo disso, sem alarme a toa)
//   60% -> ~10 min: ainda da para trocar de caminho
//   90% -> menos de 1 min: a agua esta saindo do bueiro, ninguem passa
const float LIMITE_ATENCAO = 40.0;
const float LIMITE_PERIGO = 60.0;
const float LIMITE_EMERGENCIA = 90.0;

// Pos-chuva: depois que a agua escoa, o que continua no bueiro e lixo, terra ou
// agua parada (o bueiro esta entupindo). Com 25% ocupado ja pede limpeza: na
// proxima chuva sobram so 3/4 do espaco e o bueiro enche mais rapido.
const float LIMITE_MANUTENCAO = 25.0;
const float LIMITE_MANUTENCAO_OK = 20.0;  // so volta a "limpo" abaixo de 20% (nao fica trocando)
const unsigned long PISCA_AMARELO_MS = 3000;       // atencao e limpeza: o amarelo pisca 1 vez a cada 3 s
const unsigned long PISCA_AMARELO_ACESO_MS = 500;  // e fica aceso 0,5 s em cada piscada

// Leituras e filtros
const int INTERVALO_LEITURA_MS = 250;     // 1 leitura a cada 250 ms
const int LEITURAS_MEDIA = 5;             // media movel das 5 ultimas leituras boas
const int LEITURAS_CONFIRMACAO = 3;       // 3 seguidas: o LED nao fica trocando na fronteira
const int LEITURAS_FALHA = 3;             // 3 invalidas seguidas = falha do sensor
const float VARIACAO_BRUSCA_PCT = 15.0;   // salto > 15% em 250 ms nao e agua, e objeto passando

// Previsao
const int TAM_HISTORICO = 31;             // altura guardada a cada 1 s: 31 valores = 30 s
const int MIN_AMOSTRAS_PREVISAO = 6;      // precisa de pelo menos 5 s de dados
const float SUBIDA_MINIMA_PCT_MIN = 1.0;  // abaixo de 1% por minuto: "Nivel estavel"

// Tabela de previsao (chuva forte): agua no bueiro (%) -> minutos ate alagar
const int PONTOS_TABELA = 6;
float tabelaAgua[6]    = { 0, 40, 60, 75, 85, 90};
float tabelaMinutos[6] = {30, 15, 10,  5,  2,  0};

// Botao e menu
const int TEMPO_SEGURAR_MS = 1000;        // segurar 1 s = proxima linha do menu
const int MENU_FECHA_MS = 10000;          // o menu fecha sozinho apos 10 s parado
const int JANELA_CLIQUES_MS = 450;        // cliques com menos de 0,45 s entre eles contam juntos (2 ou 3 cliques)

// Som: volume fixo pelo PWM (0 a 1023; 512 e o mais alto)
const int VOLUME_ALARME = 512;            // alarmes sempre no volume maximo
const int VOLUME_BAIXINHO = 20;           // so o "plim" da abertura
const bool SOM_ATIVO = true;              // false = sem buzzer (testes em sala)
const bool ANIMACAO_INICIAL = true;       // false = pula a abertura

// Internet: manda os dados para o servidor e o site.
// Sem Wi-Fi ou sem servidor, o sistema continua funcionando normalmente.
const bool USAR_INTERNET = true;
String WIFI_NOME = "Wokwi-GUEST";         // Wokwi: "Wokwi-GUEST" (sem senha)
String WIFI_SENHA = "";                   // Maquete: Wi-Fi do celular (rede 2,4 GHz)
String SERVIDOR_URL = "https://leandror.pythonanywhere.com/api/leituras";
String CHAVE_SERVIDOR = "fluxo-seguro-2026";   // a mesma chave do servidor (flask_app.py)
String SENSOR_ID = "bueiro-01";
String LOCAL_SENSOR = "Av. Paulista, 1106 - Bela Vista";
String RUA_NA_TELA = "Av. Paulista, 1106";       // rua de exemplo (mostrada na abertura)
String BAIRRO_NA_TELA = "Bela Vista - SP";
const int INTERVALO_ENVIO_MS = 2000;      // manda o dado de agora a cada 2 s

// ============================================================================
//  PINOS
// ============================================================================
const int PINO_TRIG = 23;
const int PINO_ECHO = 22;           // recebe o ECHO pelo divisor de tensao
const int PINO_LED_VERDE = 5;
const int PINO_LED_AMARELO = 18;
const int PINO_LED_VERMELHO = 19;
const int PINO_BUZZER = 21;
const int PINO_BOTAO = 17;          // pino TX2 da placa
const int PINO_SDA = 4;             // OLED
const int PINO_SCL = 15;            // OLED

Adafruit_SSD1306 display(128, 64, &Wire, -1);   // OLED 128 x 64
Preferences memoria;                            // guarda o idioma no ESP32

// ============================================================================
//  NIVEIS E TEXTOS DA TELA em 3 idiomas: 0 portugues, 1 espanhol, 2 ingles
//  (o display nao tem acentos; cada texto cabe na tela nos 3 idiomas)
// ============================================================================
const int SEM_NIVEL = -1;
const int NORMAL = 0;
const int ATENCAO = 1;
const int PERIGO = 2;
const int EMERGENCIA = 3;
const int FALHA = 4;                   // usado so no envio para o site

const int EVENTO_LEITURA = 0;          // envio para o site: dado de agora (a cada 2 s)
const int EVENTO_ALERTA = 1;           // o nivel mudou
const int EVENTO_FALHA = 2;            // o sensor parou

String nomeNivel[3][4] = {
  {"NORMAL", "ATENCAO",  "PERIGO",  "EMERGENCIA"},
  {"NORMAL", "ATENCION", "PELIGRO", "EMERGENCIA"},
  {"NORMAL", "WARNING",  "DANGER",  "EMERGENCY"}
};
String mensagemNivel[3][4] = {         // para quem esta na rua (ate 9 letras)
  {"VIA LIVRE", "CUIDADO!", "DESVIE!", "NAO PASSE"},
  {"VIA LIBRE", "CUIDADO!", "DESVIE!", "NO PASE!"},
  {"ALL CLEAR", "CAUTION!", "DETOUR!", "DO NOT GO"}
};
String nomeIdioma[3] = {"Portugues", "Espanol", "English"};

String txtAgua[3]          = {"Agua", "Agua", "Water"};
String txtSemPrevisao[3]   = {"Sem previsao", "Sin prevision", "No forecast"};
String txtEstavel[3]       = {"Nivel estavel", "Nivel estable", "Stable level"};
String txtLimite[3]        = {"Limite critico!", "Limite critico!", "Critical limit!"};
String txtAlagaEm[3]       = {"Alaga em", "Inunda en", "Flood in"};
String txtMinutos[3]       = {"minutos", "min", "minutes"};
String txtMinuto[3]        = {"minuto", "min", "minute"};
String txtMenos1[3]        = {"Menos de 1 minuto!", "Menos de 1 minuto!", "Under 1 minute!"};
String txtFalha[3]         = {"FALHA", "FALLA", "FAILURE"};
String txtSemSensor[3]     = {"Sensor sem resposta", "Sensor sin respuesta", "No sensor response"};
String txtMonitoramento[3] = {"Monitoramento", "Monitoreo", "Monitoring"};
String txtIndisponivel[3]  = {"indisponivel", "no disponible", "unavailable"};
String txtIniciando[3]     = {"Iniciando leituras...", "Iniciando lecturas...", "Starting readings..."};
String txtCalibrando[3]    = {"Calibrando...", "Calibrando...", "Calibrating..."};
String txtNivelAgua[3]     = {"Nivel de agua", "Nivel del agua", "Water level"};
String txtRecusada[3]      = {"Recusada: reinicie", "Rechazada: reinicie", "Rejected: restart"};
String txtMenu[3]          = {"CONFIGURACOES", "CONFIGURACION", "SETTINGS"};
String txtIdioma[3]        = {"Idioma:", "Idioma:", "Language:"};
String txtSair[3]          = {"Sair", "Salir", "Exit"};
String txtAjudaClique[3]   = {"Clique: muda valor", "Clic: cambia valor", "Click: change value"};
String txtAjudaSegure[3]   = {"Segure: proximo item", "Mantener: siguiente", "Hold: next item"};
String txtSubtitulo[3]     = {"Alerta de alagamento", "Alerta de inundacion", "Flood alert"};
String txtAnimEscudo[3]    = {"Protegendo a cidade", "Protegiendo la ciudad", "Protecting the city"};
String txtAnimGota[3]      = {"Monitorando a agua", "Monitoreando el agua", "Watching the water"};
String txtAnimPronto[3]    = {"Sistema pronto!", "Sistema listo!", "System ready!"};
String txtPrevChuva[3]     = {"Previsao de chuva", "Prevision de lluvia", "Rain forecast"};
String txtSistIniciando[3] = {"Sistema iniciando", "Sistema iniciando", "System starting"};
String txtRuaMonitorada[3] = {"Rua monitorada:", "Calle monitoreada:", "Monitored street:"};
String txtChuva[3]         = {"Chuva:", "Lluvia:", "Rain:"};
String txtSimNao[3][2]     = {{"Sim", "Nao"}, {"Si", "No"}, {"Yes", "No"}};
String txtChuvaAcabou[3]   = {"A chuva acabou", "La lluvia paro", "Rain is over"};
String txtChuvaComecou[3]  = {"A chuva comecou", "Empezo a llover", "Rain started"};
String txtVerificando[3]   = {"Verificando bueiro", "Revisando sumidero", "Checking the drain"};
String txtPosChuva[3]      = {"POS-CHUVA", "POS-LLUVIA", "AFTER RAIN"};
String txtLimpar[3]        = {"LIMPAR", "LIMPIAR", "CLEAN UP"};
String txtLimpo[3]         = {"LIMPO", "LIMPIO", "CLEAN"};
String txtLixo[3]          = {"Lixo", "Sucio", "Trash"};
String txtLimpeza[3]       = {"Limpeza necessaria", "Limpieza necesaria", "Cleaning needed"};
String txtProntoChuva[3]   = {"Pronto para chuva", "Listo para lluvia", "Ready for rain"};
String txtManutPedida[3]   = {"Manutencao pedida", "Mantenim. pedido", "Crew requested"};

// ============================================================================
//  VARIAVEIS DO SISTEMA
// ============================================================================
int idioma = 0;                    // 0 portugues, 1 espanhol, 2 ingles (muda no menu)
float alturaUtilCm = 80.0;         // vazia - transborda (calculada no setup)

// Media movel: as 5 ultimas leituras boas
float leituras[LEITURAS_MEDIA];
int qtdLeituras = 0;               // quantas ja tem (no maximo 5)
int posicaoLeitura = 0;            // onde entra a proxima

// Filtro de picos
bool temSuspeita = false;
float leituraSuspeita = 0;

// Falha do sensor
int invalidasSeguidas = 0;
bool falhaSensor = false;

// Resultado de cada leitura
float distanciaMediaCm = 0;
float alturaAguaCm = 0;
float nivelPct = 0;

// Confirmacao tripla
int nivelConfirmado = SEM_NIVEL;   // o nivel que vale (LEDs, buzzer e tela)
int nivelCandidato = SEM_NIVEL;    // o nivel que esta "querendo entrar"
int contagemCandidato = 0;         // quantas leituras seguidas ele apareceu

// Previsao: altura da agua a cada 1 s nos ultimos 30 s
float historicoAltura[TAM_HISTORICO];
unsigned long historicoTempo[TAM_HISTORICO];
int qtdHistorico = 0;
int posicaoHistorico = 0;

const int PREV_SEM_DADOS = 0;      // "Sem previsao"
const int PREV_ESTAVEL = 1;        // "Nivel estavel"
const int PREV_MINUTOS = 2;        // "Alaga em X minutos"
const int PREV_ATINGIDO = 3;       // "Limite critico!"
int tipoPrevisao = PREV_SEM_DADOS;
float velocidadeCmMin = 0;
float previsaoMin = 0;
bool aguaSubindo = false;

// Chuva e pos-chuva
bool chovendo = true;              // true = modo chuva; false = pos-chuva (tempo seco)
float lixoPct = 0;                 // o que ficou no bueiro depois da ultima chuva (% da altura)
float lixoParado = 0;              // pos-chuva: ultima medida sem a agua subindo
bool manutencaoPedida = false;     // true = o bueiro precisa de manutencao (o site foi avisado)

// Buzzer
int frequenciaAtual = 0;
int volumeAtual = -1;              // volume (PWM) que esta tocando agora
int frequenciaBip = 0;
unsigned long bipAte = 0;          // ate quando o bip curto toca
unsigned long inicioNivelMs = 0;   // quando o nivel atual foi confirmado

// Botao e menu
bool botaoApertado = false;        // estado ja filtrado (sem tremido)
bool leituraBotaoAnterior = false;
bool segurarJaFeito = false;
unsigned long tempoMudancaBotao = 0;
unsigned long tempoApertou = 0;
bool menuAberto = false;
int linhaMenu = 0;                 // 0 chuva, 1 idioma, 2 sair
unsigned long tempoUltimoUsoMenu = 0;
int cliquesSeguidos = 0;           // menu fechado: 1 = abre o menu, 2 = idioma, 3 = chuva
unsigned long tempoUltimoClique = 0;
unsigned long avisoIdiomaAte = 0;  // ate quando a tela mostra o idioma novo (2 cliques)

// Ultima vez (em millis) que cada tarefa do loop rodou
unsigned long tempoLeitura = 0;
unsigned long tempoHistorico = 0;
unsigned long tempoTela = 0;
unsigned long tempoSerial = 0;
unsigned long tempoEnvio = 0;
unsigned long tempoStatusInternet = 0;

// ============================================================================
//  BUZZER (PWM: a frequencia e o tom, a largura do pulso e o volume)
// ============================================================================
void iniciarBuzzer() {
  ledcAttach(PINO_BUZZER, 2000, 10);      // PWM no pino do buzzer, 10 bits (0 a 1023)
  ledcWrite(PINO_BUZZER, 0);              // comeca em silencio
}

void tocarNoVolume(int frequencia, int volumePwm) {
  if (!SOM_ATIVO) {
    frequencia = 0;
  }
  if (frequencia == frequenciaAtual && volumePwm == volumeAtual) {
    return;                               // ja esta tocando isso
  }
  frequenciaAtual = frequencia;
  volumeAtual = volumePwm;
  if (frequencia == 0) {
    ledcWrite(PINO_BUZZER, 0);            // silencio
  } else {
    ledcWriteTone(PINO_BUZZER, frequencia);    // muda o tom
    ledcWrite(PINO_BUZZER, volumePwm);         // aplica o volume
  }
}

// Alarmes e bips: sempre no volume maximo
void tocar(int frequencia) {
  tocarNoVolume(frequencia, VOLUME_ALARME);
}

// So o "plim" da abertura, baixinho
void tocarBaixinho(int frequencia) {
  tocarNoVolume(frequencia, VOLUME_BAIXINHO);
}

// Bip curto que nao trava o programa (o loop desliga quando o tempo acaba)
void bip(int frequencia, int duracaoMs) {
  frequenciaBip = frequencia;
  bipAte = millis() + duracaoMs;
}

// ============================================================================
//  SENSOR HC-SR04
// ============================================================================

// Mede a distancia em cm. Devolve -1 se a leitura for invalida.
float medirDistanciaCm() {
  digitalWrite(PINO_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PINO_TRIG, HIGH);          // pulso de 10 us: "mede agora"
  delayMicroseconds(10);
  digitalWrite(PINO_TRIG, LOW);
  long duracao = pulseIn(PINO_ECHO, HIGH, 30000);   // tempo de ida e volta (ate ~5 m)
  if (duracao == 0) {
    return -1;                            // sem eco: sensor sem resposta
  }
  float cm = duracao / 58.0;              // tempo (us) / 58 = distancia em cm
  if (cm > 400) {
    return -1;                            // fora do alcance do sensor
  }
  if (cm < 2) {
    cm = 2;                               // abaixo de 2 cm o sensor nao mede: agua encostando
  }
  return cm;
}

// Media das leituras guardadas
float calcularMedia() {
  float soma = 0;
  for (int i = 0; i < qtdLeituras; i++) {
    soma = soma + leituras[i];
  }
  return soma / qtdLeituras;
}

// Guarda uma leitura na media (quando enche, a nova entra no lugar da mais velha)
void guardarLeitura(float cm) {
  leituras[posicaoLeitura] = cm;
  posicaoLeitura = posicaoLeitura + 1;
  if (posicaoLeitura == LEITURAS_MEDIA) {
    posicaoLeitura = 0;
  }
  if (qtdLeituras < LEITURAS_MEDIA) {
    qtdLeituras = qtdLeituras + 1;
  }
}

void apagarMedia() {
  qtdLeituras = 0;
  posicaoLeitura = 0;
}

// ============================================================================
//  FALHA DO SENSOR (RN02)
// ============================================================================
void entrarEmFalha() {
  falhaSensor = true;
  apagarMedia();
  temSuspeita = false;
  nivelConfirmado = SEM_NIVEL;            // nao mostra nivel antigo como se fosse atual
  nivelCandidato = SEM_NIVEL;
  contagemCandidato = 0;
  qtdHistorico = 0;
  posicaoHistorico = 0;
  Serial.println("[FALHA] 3 leituras invalidas seguidas: monitoramento indisponivel.");
  avisarSite(EVENTO_FALHA);
}

// ============================================================================
//  NIVEIS E CONFIRMACAO TRIPLA (RN01)
// ============================================================================
int nivelPeloPercentual(float pct) {
  if (pct >= LIMITE_EMERGENCIA) {
    return EMERGENCIA;
  } else if (pct >= LIMITE_PERIGO) {
    return PERIGO;
  } else if (pct >= LIMITE_ATENCAO) {
    return ATENCAO;
  } else {
    return NORMAL;
  }
}

void classificar() {
  int nivel = nivelPeloPercentual(nivelPct);
  if (nivel == nivelCandidato) {
    contagemCandidato = contagemCandidato + 1;    // mais uma leitura igual
  } else {
    nivelCandidato = nivel;                       // mudou: comeca a contar de novo
    contagemCandidato = 1;
  }
  if (contagemCandidato >= LEITURAS_CONFIRMACAO && nivel != nivelConfirmado) {
    nivelConfirmado = nivel;                      // agora o nivel muda de verdade
    inicioNivelMs = millis();
    Serial.println("[NIVEL] Confirmado apos 3 leituras seguidas: " + nomeNivel[0][nivel]);
    if (chovendo) {                               // no pos-chuva quem decide e a manutencao
      avisarSite(EVENTO_ALERTA);
      if (nivel >= PERIGO && menuAberto) {
        fecharMenu("alerta de perigo");           // o alerta vem primeiro
      }
    }
  }
}

// ============================================================================
//  PROCESSAMENTO DE CADA LEITURA (RF01 / US03)
// ============================================================================
void processarLeitura() {
  float cm = medirDistanciaCm();

  // Leitura invalida: conta e, na 3a seguida, entra em falha
  if (cm < 0) {
    invalidasSeguidas = invalidasSeguidas + 1;
    if (invalidasSeguidas >= LEITURAS_FALHA && !falhaSensor) {
      entrarEmFalha();
    }
    return;
  }
  invalidasSeguidas = 0;
  if (falhaSensor) {
    falhaSensor = false;
    Serial.println("[OK] Sensor voltou a responder.");
  }

  // Filtro de picos: leitura muito longe da media fica "em analise"
  if (qtdLeituras >= 3) {
    float media = calcularMedia();
    float limite = alturaUtilCm * VARIACAO_BRUSCA_PCT / 100.0;
    float diferenca = cm - media;
    if (diferenca < 0) {
      diferenca = -diferenca;             // so interessa o tamanho do salto
    }
    if (diferenca > limite) {
      float diferencaSuspeita = cm - leituraSuspeita;
      if (diferencaSuspeita < 0) {
        diferencaSuspeita = -diferencaSuspeita;
      }
      if (temSuspeita && diferencaSuspeita <= limite / 2) {
        // Duas leituras seguidas no mesmo valor: a mudanca e real
        apagarMedia();
        guardarLeitura(leituraSuspeita);
        temSuspeita = false;
        Serial.println("[FILTRO] Variacao brusca confirmada: mudanca real aceita.");
      } else {
        // Primeira leitura estranha: nao entra na media
        temSuspeita = true;
        leituraSuspeita = cm;
        Serial.println("[FILTRO] Leitura fora do padrao (" + String(cm, 1) + " cm) em analise.");
        return;
      }
    } else {
      temSuspeita = false;
    }
  }

  guardarLeitura(cm);

  // Media movel e nivel em %
  distanciaMediaCm = calcularMedia();
  alturaAguaCm = distanciaVaziaCm - distanciaMediaCm;
  if (alturaAguaCm < 0) {
    alturaAguaCm = 0;
  }
  nivelPct = alturaAguaCm / alturaUtilCm * 100.0;

  classificar();
}

// ============================================================================
//  PREVISAO DO TEMPO ATE ALAGAR (RF04 / US01)
// ============================================================================

// Guarda a altura da agua (chamada a cada 1 s)
void guardarHistorico(unsigned long agora) {
  if (falhaSensor || qtdLeituras < 3) {
    return;
  }
  historicoAltura[posicaoHistorico] = alturaAguaCm;
  historicoTempo[posicaoHistorico] = agora;
  posicaoHistorico = posicaoHistorico + 1;
  if (posicaoHistorico == TAM_HISTORICO) {
    posicaoHistorico = 0;
  }
  if (qtdHistorico < TAM_HISTORICO) {
    qtdHistorico = qtdHistorico + 1;
  }
}

// Minutos ate alagar pela tabela (conta proporcional entre dois pontos)
float minutosPelaTabela(float pct) {
  if (pct <= tabelaAgua[0]) {
    return tabelaMinutos[0];
  }
  for (int i = 1; i < PONTOS_TABELA; i++) {
    if (pct <= tabelaAgua[i]) {
      float parte = (pct - tabelaAgua[i - 1]) / (tabelaAgua[i] - tabelaAgua[i - 1]);
      return tabelaMinutos[i - 1] + parte * (tabelaMinutos[i] - tabelaMinutos[i - 1]);
    }
  }
  return 0;
}

void calcularPrevisao() {
  if (falhaSensor || qtdLeituras == 0) {
    tipoPrevisao = PREV_SEM_DADOS;
    return;
  }

  // a) Velocidade de subida: compara a altura de agora com a de ate 30 s atras
  bool temDados = qtdHistorico >= MIN_AMOSTRAS_PREVISAO;
  aguaSubindo = false;
  velocidadeCmMin = 0;
  if (temDados) {
    int atual = posicaoHistorico - 1;             // a mais nova
    if (atual < 0) {
      atual = atual + TAM_HISTORICO;
    }
    int antiga = posicaoHistorico - qtdHistorico; // a mais antiga
    if (antiga < 0) {
      antiga = antiga + TAM_HISTORICO;
    }
    float minutos = (historicoTempo[atual] - historicoTempo[antiga]) / 60000.0;
    if (minutos > 0) {
      velocidadeCmMin = (historicoAltura[atual] - historicoAltura[antiga]) / minutos;
    }
    if (velocidadeCmMin >= alturaUtilCm * SUBIDA_MINIMA_PCT_MIN / 100.0) {
      aguaSubindo = true;
    }
  }

  // b) Tempo ate alagar pela tabela
  if (nivelPct >= LIMITE_EMERGENCIA) {
    tipoPrevisao = PREV_ATINGIDO;
  } else if (aguaSubindo || nivelPct >= LIMITE_ATENCAO) {
    previsaoMin = minutosPelaTabela(nivelPct);
    tipoPrevisao = PREV_MINUTOS;
  } else if (!temDados) {
    tipoPrevisao = PREV_SEM_DADOS;
  } else {
    tipoPrevisao = PREV_ESTAVEL;
  }
}

// ============================================================================
//  CHUVA E POS-CHUVA (manutencao do bueiro)
//  Na chuva: se a previsao e de alagamento, pede manutencao para a central.
//  Pos-chuva: mede o que ficou no bueiro (lixo, terra, agua parada) e pede
//  limpeza a partir de 25%, antes da proxima chuva.
//  O sensor mede so a altura do que tem no bueiro: ele nao separa agua de lixo.
//  Por isso o lixo e medido no tempo seco (a agua ja escoou) e, na chuva,
//  o que passar dessa altura e agua.
// ============================================================================

// Chamada a cada 1 s pelo loop
void verificarManutencao() {
  if (falhaSensor || nivelConfirmado == SEM_NIVEL) {
    return;                               // sem leitura confiavel: nao muda nada
  }
  bool precisa = false;
  if (chovendo) {
    if (manutencaoPedida) {
      precisa = nivelConfirmado >= ATENCAO;     // continua pedida ate a agua voltar ao normal
    } else {
      // previsao de alagamento: agua subindo em ATENCAO, ou ja em PERIGO/EMERGENCIA
      precisa = nivelConfirmado >= PERIGO || (nivelConfirmado == ATENCAO && aguaSubindo);
    }
  } else {
    if (!aguaSubindo) {
      lixoParado = nivelPct;              // ultima medida com a agua parada
    } else if (nivelConfirmado >= PERIGO) {
      // A agua subiu rapido ate PERIGO: e chuva de novo (o alerta de alagamento vem primeiro)
      Serial.println("[CHUVA] A agua voltou a subir: voltando para o modo chuva.");
      lixoPct = lixoParado;               // o lixo e o de antes da agua subir
      mudarModo(true);
      return;
    }
    lixoPct = nivelPct;                   // tempo seco: tudo o que esta no bueiro e sujeira
    if (manutencaoPedida) {
      precisa = lixoPct >= LIMITE_MANUTENCAO_OK;
    } else {
      precisa = lixoPct >= LIMITE_MANUTENCAO;
    }
  }

  if (precisa != manutencaoPedida) {
    manutencaoPedida = precisa;
    if (precisa && chovendo) {
      Serial.println("[MANUTENCAO] Previsao de alagamento: manutencao pedida para a central.");
    } else if (precisa) {
      bip(1200, 150);
      Serial.println("[MANUTENCAO] Bueiro " + String(lixoPct, 0) + "% ocupado: limpeza pedida.");
    } else {
      Serial.println("[MANUTENCAO] Pedido encerrado.");
    }
    avisarSite(EVENTO_ALERTA);
  }
}

// Troca entre chuva (true) e pos-chuva (false). Chamada pelo menu.
void mudarModo(bool chuva) {
  chovendo = chuva;
  manutencaoPedida = false;
  qtdHistorico = 0;                       // a previsao recomeca do zero no modo novo
  posicaoHistorico = 0;
  aguaSubindo = false;
  if (chuva) {
    Serial.println("[CHUVA] A chuva comecou. Lixo da ultima medida: " + String(lixoPct, 1) + "%");
  } else {
    if (!falhaSensor && qtdLeituras > 0) {
      lixoPct = nivelPct;                 // primeira medida do que ficou no bueiro
      lixoParado = nivelPct;
    }
    Serial.println("[POS-CHUVA] A chuva acabou: medindo o que ficou no bueiro.");
  }
  animacaoModo(chuva);
  inicioNivelMs = millis();
  avisarSite(EVENTO_ALERTA);
}

// ============================================================================
//  ALERTAS: LEDs E BUZZER (RF03 / US02)
// ============================================================================

// Troca entre ligado e desligado a cada 500 ms (perigo) ou 150 ms (emergencia)
bool piscaDevagar(unsigned long agora) {
  return (agora / 500) % 2 == 0;
}
bool piscaRapido(unsigned long agora) {
  return (agora / 150) % 2 == 0;
}

// Atencao: LED amarelo e bip juntos, 0,5 s ligados a cada 3 s (o 1o toca na hora)
bool pulsoAtencao(unsigned long agora) {
  if (agora < inicioNivelMs) {
    return false;
  }
  return (agora - inicioNivelMs) % PISCA_AMARELO_MS < PISCA_AMARELO_ACESO_MS;
}

void atualizarLeds(unsigned long agora) {
  bool verde = false;
  bool amarelo = false;
  bool vermelho = false;
  if (!falhaSensor && !chovendo && nivelConfirmado != SEM_NIVEL) {
    if (manutencaoPedida) {
      amarelo = (agora % PISCA_AMARELO_MS) < PISCA_AMARELO_ACESO_MS;  // pos-chuva: 1 piscada a cada 3 s = limpar
    } else {
      verde = true;                       // pos-chuva: bueiro limpo
    }
  } else if (!falhaSensor) {
    if (nivelConfirmado == NORMAL) {
      verde = true;
    } else if (nivelConfirmado == ATENCAO) {
      amarelo = pulsoAtencao(agora);      // pisca junto com o bip
    } else if (nivelConfirmado == PERIGO) {
      vermelho = piscaDevagar(agora);
    } else if (nivelConfirmado == EMERGENCIA) {
      vermelho = piscaRapido(agora);
    }
  }
  digitalWrite(PINO_LED_VERDE, verde);
  digitalWrite(PINO_LED_AMARELO, amarelo);
  digitalWrite(PINO_LED_VERMELHO, vermelho);
}

void atualizarBuzzer(unsigned long agora) {
  int frequencia = 0;
  if (!chovendo) {                        // pos-chuva: sem alarme de alagamento, so bips curtos
    if (agora < bipAte) {
      frequencia = frequenciaBip;
    }
    tocar(frequencia);
    return;
  }
  if (!falhaSensor && nivelConfirmado == EMERGENCIA) {
    if (piscaRapido(agora)) {
      frequencia = 1400;                  // bips rapidos e agudos
    }
  } else if (!falhaSensor && nivelConfirmado == PERIGO) {
    if (piscaDevagar(agora)) {
      frequencia = 1000;                  // bips lentos, junto com o LED
    }
  } else if (!falhaSensor && nivelConfirmado == ATENCAO && pulsoAtencao(agora)) {
    frequencia = 1000;
  } else if (agora < bipAte) {
    frequencia = frequenciaBip;           // bip curto do menu
  }
  tocar(frequencia);
}

// ============================================================================
//  BOTAO E MENU (RF05 / US04)
// ============================================================================
void salvarConfiguracao() {
  memoria.begin("fluxo", false);
  memoria.putInt("idioma", idioma);
  memoria.end();
}

void carregarConfiguracao() {
  memoria.begin("fluxo", true);
  idioma = memoria.getInt("idioma", 0);
  memoria.end();
  if (idioma < 0 || idioma > 2) {
    idioma = 0;
  }
}

void abrirMenu(unsigned long agora) {
  menuAberto = true;
  linhaMenu = 0;
  tempoUltimoUsoMenu = agora;
  bip(1200, 60);
  Serial.println("[MENU] Aberto.");
}

void fecharMenu(String motivo) {
  menuAberto = false;
  salvarConfiguracao();
  Serial.println("[MENU] Fechado (" + motivo + "). Idioma: " + nomeIdioma[idioma]);
}

// Clique curto
void aoClicar(unsigned long agora) {
  if (!menuAberto) {
    abrirMenu(agora);
    return;
  }
  tempoUltimoUsoMenu = agora;
  if (linhaMenu == 0) {
    fecharMenu("chuva");
    mudarModo(!chovendo);                 // Sim -> Nao (a chuva acabou) ou Nao -> Sim
  } else if (linhaMenu == 1) {
    idioma = (idioma + 1) % 3;            // proximo idioma (depois do ingles volta ao portugues)
    bip(1000, 80);
    Serial.println("[MENU] Idioma: " + nomeIdioma[idioma]);
  } else {
    bip(800, 60);
    fecharMenu("Sair");
  }
}

// Cliques rapidos com o menu fechado: 1 abre o menu, 2 trocam portugues/ingles,
// 3 ligam/desligam a chuva (o mesmo que "Chuva: Sim/Nao" no menu)
void aoCliquesSeguidos(int cliques, unsigned long agora) {
  if (cliques == 1) {
    abrirMenu(agora);
  } else if (cliques == 2) {
    if (idioma == 2) {
      idioma = 0;                         // ingles -> portugues
    } else {
      idioma = 2;                         // portugues (ou espanhol) -> ingles
    }
    salvarConfiguracao();
    avisoIdiomaAte = agora + 1200;        // a tela mostra o idioma novo por 1,2 s
    bip(1000, 80);
    Serial.println("[BOTAO] 2 cliques: idioma " + nomeIdioma[idioma]);
  } else {
    if (chovendo) {
      Serial.println("[BOTAO] 3 cliques: a chuva acabou.");
    } else {
      Serial.println("[BOTAO] 3 cliques: a chuva comecou.");
    }
    mudarModo(!chovendo);
  }
}

// Segurou 1 segundo
void aoSegurar(unsigned long agora) {
  if (!menuAberto) {
    abrirMenu(agora);
    return;
  }
  tempoUltimoUsoMenu = agora;
  linhaMenu = (linhaMenu + 1) % 3;        // proxima linha do menu
  bip(800, 40);
}

// Le o botao sem travar o programa e ignora o "tremido" do contato (30 ms)
void lerBotao(unsigned long agora) {
  bool leitura = (digitalRead(PINO_BOTAO) == LOW);   // apertado = LOW (pull-up)
  if (leitura != leituraBotaoAnterior) {
    leituraBotaoAnterior = leitura;
    tempoMudancaBotao = agora;
  }
  if (agora - tempoMudancaBotao < 30) {
    return;                               // ainda tremendo: espera estabilizar
  }

  if (leitura != botaoApertado) {
    botaoApertado = leitura;
    if (botaoApertado) {
      tempoApertou = agora;               // acabou de apertar
      segurarJaFeito = false;
    } else if (!segurarJaFeito) {        // soltou rapido = clique
      if (menuAberto) {
        aoClicar(agora);                  // no menu: cada clique vale na hora
      } else {
        cliquesSeguidos = cliquesSeguidos + 1;   // menu fechado: espera para ver se vem outro clique
        tempoUltimoClique = agora;
      }
    }
  } else if (botaoApertado && !segurarJaFeito && agora - tempoApertou >= TEMPO_SEGURAR_MS) {
    segurarJaFeito = true;                // continua apertado ha 1 s = segurar
    cliquesSeguidos = 0;
    aoSegurar(agora);
  }

  // Menu fechado: 0,45 s depois do ultimo clique, ve quantos foram (1, 2 ou 3)
  if (cliquesSeguidos > 0 && !botaoApertado && agora - tempoUltimoClique >= JANELA_CLIQUES_MS) {
    int cliques = cliquesSeguidos;
    if (cliques > 3) {
      cliques = 3;
    }
    cliquesSeguidos = 0;
    aoCliquesSeguidos(cliques, agora);
  }

  if (menuAberto && agora - tempoUltimoUsoMenu >= MENU_FECHA_MS) {
    fecharMenu("tempo esgotado");
  }
}

// ============================================================================
//  DISPLAY OLED (RF02)
//  Linhas 0 a 15 amarelas (mensagem) e 16 a 63 azuis (nivel, agua e previsao).
//  No Wokwi tudo aparece branco.
// ============================================================================

// Escreve um texto centralizado na linha y (cada letra tem 6 pontos x tamanho)
void escreverCentralizado(String texto, int y, int tamanho) {
  int largura = texto.length() * 6 * tamanho;
  display.setTextSize(tamanho);
  display.setCursor((128 - largura) / 2, y);
  display.print(texto);
}

// Logo: uma gota d'agua (bolinha embaixo + triangulo em cima)
void desenharLogo(int x, int y, int cor) {
  display.fillCircle(x + 6, y + 9, 5, cor);
  display.fillTriangle(x + 1, y + 8, x + 11, y + 8, x + 6, y, cor);
}

// Icone de gota pequena (nivel da agua)
void desenharIconeGota(int x, int y) {
  display.fillCircle(x + 3, y + 5, 2, SSD1306_WHITE);
  display.fillTriangle(x + 1, y + 4, x + 5, y + 4, x + 3, y, SSD1306_WHITE);
}

// Icone de relogio pequeno (previsao)
void desenharIconeRelogio(int x, int y) {
  display.drawCircle(x + 3, y + 3, 3, SSD1306_WHITE);
  display.drawLine(x + 3, y + 3, x + 3, y + 1, SSD1306_WHITE);
  display.drawLine(x + 3, y + 3, x + 5, y + 3, SSD1306_WHITE);
}

// Icone de chave de boca (manutencao)
void desenharIconeChave(int x, int y) {
  display.drawLine(x, y + 6, x + 4, y + 2, SSD1306_WHITE);   // cabo
  display.drawCircle(x + 5, y + 1, 1, SSD1306_WHITE);         // boca da chave
}

// Icone de lixeira (pos-chuva)
void desenharIconeLixo(int x, int y) {
  display.drawPixel(x + 3, y, SSD1306_WHITE);                 // alca
  display.drawFastHLine(x, y + 1, 7, SSD1306_WHITE);          // tampa
  display.drawRect(x + 1, y + 2, 5, 5, SSD1306_WHITE);        // lata
}

// Icone de "certo" (bueiro limpo)
void desenharIconeOk(int x, int y) {
  display.drawLine(x, y + 3, x + 2, y + 5, SSD1306_WHITE);
  display.drawLine(x + 2, y + 5, x + 6, y + 1, SSD1306_WHITE);
}

// Faixa amarela: logo + mensagem grande (ex.: VIA LIVRE).
// piscando = fundo aceso e letras apagadas (usado na emergencia).
void faixaAmarela(String mensagem, bool piscando) {
  int cor = SSD1306_WHITE;
  if (piscando) {
    display.fillRect(0, 0, 128, 16, SSD1306_WHITE);
    cor = SSD1306_BLACK;
  }
  desenharLogo(0, 1, cor);
  display.setTextColor(cor);
  int largura = mensagem.length() * 12;
  if (largura <= 112) {
    display.setTextSize(2);                       // letra grande
    display.setCursor(16 + (112 - largura) / 2, 1);
  } else {
    display.setTextSize(1);                       // texto comprido: letra pequena
    display.setCursor(18, 4);
  }
  display.print(mensagem);
  display.setTextColor(SSD1306_WHITE);
}

// Faixa amarela com a marca FLUXO SEGURO (telas de inicio)
void faixaMarca() {
  desenharLogo(0, 1, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(18, 0);
  display.print("FLUXO");
  display.setCursor(18, 8);
  display.print("SEGURO");
}

// Barra do bueiro (0 a 100%): o lixo fica embaixo, pontilhado, e a agua em cima,
// cheia. Marquinhas em 40%, 60% e 90% (chuva) ou em 25% (pos-chuva).
void desenharBarraAgua(float pct, float lixo) {
  if (pct < 0) {
    pct = 0;
  }
  if (pct > 100) {
    pct = 100;
  }
  if (lixo > pct) {
    lixo = pct;                           // o lixo nunca passa do total medido
  }
  display.drawRect(44, 38, 58, 8, SSD1306_WHITE);
  int cheio = 54 * pct / 100.0 + 0.5;
  int parteLixo = 54 * lixo / 100.0 + 0.5;
  for (int x = 0; x < parteLixo; x++) {   // lixo: um ponto aceso, um apagado
    for (int y = 0; y < 4; y++) {
      if ((x + y) % 2 == 0) {
        display.drawPixel(46 + x, 40 + y, SSD1306_WHITE);
      }
    }
  }
  display.fillRect(46 + parteLixo, 40, cheio - parteLixo, 4, SSD1306_WHITE);   // agua
  if (chovendo) {
    display.drawFastVLine(46 + 54 * LIMITE_ATENCAO / 100, 47, 2, SSD1306_WHITE);
    display.drawFastVLine(46 + 54 * LIMITE_PERIGO / 100, 47, 2, SSD1306_WHITE);
    display.drawFastVLine(46 + 54 * LIMITE_EMERGENCIA / 100, 47, 2, SSD1306_WHITE);
  } else {
    display.drawFastVLine(46 + 54 * LIMITE_MANUTENCAO / 100, 47, 2, SSD1306_WHITE);
  }
}

// Numero da porcentagem, alinhado na direita da linha da barra
void escreverPorcentagem(float valor) {
  int pct = valor + 0.5;
  if (pct > 100) {
    pct = 100;
  }
  String texto = String(pct) + "%";
  display.setTextSize(1);
  display.setCursor(128 - texto.length() * 6, 38);
  display.print(texto);
}

// Previsao em uma linha: "Alaga em 9 minutos", "Nivel estavel"...
void escreverPrevisao(int x, int y) {
  display.setTextSize(1);
  display.setCursor(x, y);
  if (tipoPrevisao == PREV_SEM_DADOS) {
    display.print(txtSemPrevisao[idioma]);
  } else if (tipoPrevisao == PREV_ESTAVEL) {
    display.print(txtEstavel[idioma]);
  } else if (tipoPrevisao == PREV_ATINGIDO) {
    display.print(txtLimite[idioma]);
  } else if (previsaoMin < 1.0) {
    display.print(txtMenos1[idioma]);
  } else {
    int minutos = previsaoMin + 0.5;              // arredonda
    if (minutos == 1) {
      display.print(txtAlagaEm[idioma] + " 1 " + txtMinuto[idioma]);
    } else {
      display.print(txtAlagaEm[idioma] + " " + String(minutos) + " " + txtMinutos[idioma]);
    }
  }
}

// Linha pontilhada que separa as partes da tela
void linhaPontilhada(int y) {
  for (int x = 0; x < 128; x = x + 2) {
    display.drawPixel(x, y, SSD1306_WHITE);
  }
}

void desenharMenu() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 16, SSD1306_WHITE);       // titulo na faixa amarela
  display.setTextColor(SSD1306_BLACK);
  escreverCentralizado(txtMenu[idioma], 4, 1);
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  for (int i = 0; i < 3; i++) {
    display.setCursor(0, 18 + i * 10);                  // linhas 18, 28 e 38
    if (linhaMenu == i) {
      display.print("> ");                              // linha marcada
    } else {
      display.print("  ");
    }
    if (i == 0) {
      if (chovendo) {
        display.print(txtChuva[idioma] + " " + txtSimNao[idioma][0]);
      } else {
        display.print(txtChuva[idioma] + " " + txtSimNao[idioma][1]);
      }
    } else if (i == 1) {
      display.print(txtIdioma[idioma] + " " + nomeIdioma[idioma]);
    } else {
      display.print(txtSair[idioma]);
    }
  }

  display.drawFastHLine(0, 47, 128, SSD1306_WHITE);
  display.setCursor(0, 49);
  display.print(txtAjudaClique[idioma]);
  display.setCursor(0, 57);
  display.print(txtAjudaSegure[idioma]);
  display.display();
}

void atualizarTela(unsigned long agora) {
  if (menuAberto) {
    desenharMenu();
    return;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (agora < avisoIdiomaAte) {           // 2 cliques: mostra o idioma novo
    faixaMarca();
    escreverCentralizado(txtIdioma[idioma], 22, 1);
    escreverCentralizado(nomeIdioma[idioma], 36, 2);
    display.display();
    return;
  }

  if (falhaSensor) {
    faixaAmarela(txtFalha[idioma], false);
    escreverCentralizado(txtMonitoramento[idioma], 19, 1);
    escreverCentralizado(txtIndisponivel[idioma], 28, 1);
    linhaPontilhada(40);
    escreverCentralizado(txtSemSensor[idioma], 49, 1);
    display.display();
    return;
  }

  if (nivelConfirmado == SEM_NIVEL) {
    faixaMarca();
    escreverCentralizado(txtIniciando[idioma], 36, 1);
    display.display();
    return;
  }

  if (!chovendo) {
    telaPosChuva(agora);
    return;
  }

  // Amarelo: mensagem grande (na emergencia a faixa pisca)
  bool pisca = false;
  if (nivelConfirmado == EMERGENCIA && (agora / 300) % 2 == 0) {
    pisca = true;
  }
  faixaAmarela(mensagemNivel[idioma][nivelConfirmado], pisca);

  // Azul 1: nome do nivel, grande e centralizado
  escreverCentralizado(nomeNivel[idioma][nivelConfirmado], 18, 2);
  linhaPontilhada(34);

  // Azul 2: agua (icone, nome, barra e numero)
  desenharIconeGota(0, 38);
  display.setTextSize(1);
  display.setCursor(11, 38);
  display.print(txtAgua[idioma]);
  desenharBarraAgua(nivelPct, lixoPct);   // lixo da ultima medida + agua
  escreverPorcentagem(nivelPct);

  // Azul 3: previsao (com manutencao pedida, troca a cada 2 s com o aviso)
  if (manutencaoPedida && (agora / 2000) % 2 == 1) {
    desenharIconeChave(0, 52);
    display.setTextSize(1);
    display.setCursor(11, 52);
    display.print(txtManutPedida[idioma]);
  } else {
    desenharIconeRelogio(0, 52);
    escreverPrevisao(11, 52);
  }

  display.display();
}

// Tela do pos-chuva: quanto ficou no bueiro e se precisa limpar
void telaPosChuva(unsigned long agora) {
  faixaAmarela(txtPosChuva[idioma], false);

  // Azul 1: LIMPAR ou LIMPO, grande
  if (manutencaoPedida) {
    escreverCentralizado(txtLimpar[idioma], 18, 2);
  } else {
    escreverCentralizado(txtLimpo[idioma], 18, 2);
  }
  linhaPontilhada(34);

  // Azul 2: lixo (no tempo seco, tudo o que esta no bueiro e sujeira)
  desenharIconeLixo(0, 38);
  display.setTextSize(1);
  display.setCursor(11, 38);
  display.print(txtLixo[idioma]);
  desenharBarraAgua(lixoPct, lixoPct);
  escreverPorcentagem(lixoPct);

  // Azul 3: o que fazer
  display.setCursor(11, 52);
  if (manutencaoPedida) {
    desenharIconeChave(0, 52);
    if ((agora / 2000) % 2 == 0) {
      display.print(txtLimpeza[idioma]);
    } else {
      display.print(txtManutPedida[idioma]);
    }
  } else {
    desenharIconeOk(0, 52);
    display.print(txtProntoChuva[idioma]);
  }
  display.display();
}

// Uma linha por segundo no Monitor Serial (sempre em portugues)
void escreverNoSerial() {
  if (falhaSensor) {
    Serial.println("Sensor sem resposta | Monitoramento indisponivel");
    return;
  }
  if (!chovendo) {
    String pos = "Pos-chuva | Dist: " + String(distanciaMediaCm, 1) + " cm | Lixo: " + String(lixoPct, 1) + "% | ";
    if (manutencaoPedida) {
      pos = pos + "Limpeza pedida";
    } else {
      pos = pos + "Bueiro limpo";
    }
    Serial.println(pos);
    return;
  }
  String linha = "Dist: " + String(distanciaMediaCm, 1) + " cm | Agua: " + String(nivelPct, 1) + "% | Nivel: ";
  if (nivelConfirmado == SEM_NIVEL) {
    linha = linha + "...";
  } else {
    linha = linha + nomeNivel[0][nivelConfirmado];
  }
  linha = linha + " | Previsao: ";
  if (tipoPrevisao == PREV_SEM_DADOS) {
    linha = linha + "sem previsao";
  } else if (tipoPrevisao == PREV_ESTAVEL) {
    linha = linha + "nivel estavel";
  } else if (tipoPrevisao == PREV_ATINGIDO) {
    linha = linha + "limite critico atingido";
  } else {
    linha = linha + String(previsaoMin, 2) + " min (subida de " + String(velocidadeCmMin, 1) + " cm/min)";
  }
  if (lixoPct > 0) {
    linha = linha + " | Lixo: " + String(lixoPct, 1) + "%";
  }
  if (manutencaoPedida) {
    linha = linha + " | Manutencao pedida";
  }
  Serial.println(linha);
}

// ============================================================================
//  CALIBRACAO (bueiro vazio)
// ============================================================================
void calibrar() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  faixaMarca();
  escreverCentralizado(txtCalibrando[idioma], 26, 1);
  escreverCentralizado(txtNivelAgua[idioma], 38, 1);
  display.display();

  float soma = 0;
  int validas = 0;
  for (int i = 0; i < 10; i++) {          // 10 leituras
    float cm = medirDistanciaCm();
    if (cm > 0) {
      soma = soma + cm;
      validas = validas + 1;
    }
    delay(60);
  }
  if (validas < 3) {
    Serial.println("[CALIBRACAO] Sensor sem resposta. Usando a medida da configuracao.");
    return;
  }
  float medido = soma / validas;
  float diferenca = medido - distanciaVaziaCm;
  if (diferenca < 0) {
    diferenca = -diferenca;
  }
  if (diferenca <= toleranciaCalibracaoCm) {
    distanciaVaziaCm = medido;
    alturaUtilCm = distanciaVaziaCm - distanciaTransbordaCm;
    Serial.println("[CALIBRACAO] Bueiro vazio medido: " + String(distanciaVaziaCm, 1) + " cm");
  } else {
    Serial.println("[CALIBRACAO] Recusada: leitura de " + String(medido, 1) +
                   " cm, longe do esperado. Esvazie o bueiro e reinicie.");
    escreverCentralizado(txtRecusada[idioma], 52, 1);
    display.display();
    delay(2500);
  }
}

// ============================================================================
//  ABERTURA (uns 9 s): "Previsao de chuva, sistema iniciando", o escudo e
//  desenhado, uma gota cai dentro dele, o escudo vira a logo, aparece
//  FLUXO SEGURO e, por fim, a rua de exemplo.
//  Os LEDs acendem junto. Apertar o botao pula. Desligar: ANIMACAO_INICIAL = false
// ============================================================================

// Mostra o quadro e espera. Devolve true se o botao foi apertado (pula a abertura).
bool mostrarQuadro(int esperaMs) {
  display.display();
  delay(esperaMs);
  return digitalRead(PINO_BOTAO) == LOW;
}

// Desenha um pedaco de uma linha: parte 0 = nada, parte 1 = a linha inteira
void linhaParcial(int x1, int y1, int x2, int y2, float parte) {
  if (parte <= 0) {
    return;
  }
  if (parte > 1) {
    parte = 1;
  }
  int xFim = x1 + (x2 - x1) * parte;
  int yFim = y1 + (y2 - y1) * parte;
  display.drawLine(x1, y1, xFim, yFim, SSD1306_WHITE);
}

// Contorno do escudo. cx = centro, topo = linha de cima,
// progresso = de 0 a 5 (cada numero inteiro e um lado pronto)
void desenharContornoEscudo(int cx, int topo, float progresso) {
  int esquerda = cx - 18;
  int direita = cx + 18;
  int meio = topo + 22;                   // onde os lados viram a ponta
  int ponta = topo + 40;
  linhaParcial(esquerda, topo, direita, topo, progresso);        // 1: em cima
  linhaParcial(direita, topo, direita, meio, progresso - 1);     // 2: lado direito
  linhaParcial(direita, meio, cx, ponta, progresso - 2);         // 3: ponta, lado direito
  linhaParcial(cx, ponta, esquerda, meio, progresso - 3);        // 4: ponta, lado esquerdo
  linhaParcial(esquerda, meio, esquerda, topo, progresso - 4);   // 5: lado esquerdo
}

// Gota de agua: bolinha embaixo + triangulo em cima. cy = centro da bolinha
void desenharGota(int cx, int cy, int cor) {
  display.fillCircle(cx, cy, 7, cor);
  display.fillTriangle(cx - 6, cy - 3, cx + 6, cy - 3, cx, cy - 15, cor);
}

// Gotas de chuva caindo. quadro = numero do quadro (cada quadro desce 5 pontos)
void desenharChuva(int quadro) {
  for (int i = 0; i < 12; i++) {
    int x = 6 + i * 10;
    int y = 14 + (i * 13 + quadro * 5) % 36;    // cada gota comeca numa altura diferente
    display.drawLine(x, y, x - 2, y + 4, SSD1306_WHITE);
  }
}

// Sol que cresce e depois ganha raios que "brilham"
void desenharSol(int cx, int cy, int quadro) {
  int raio = 3 + quadro;
  if (raio > 9) {
    raio = 9;
  }
  display.fillCircle(cx, cy, raio, SSD1306_WHITE);
  if (quadro >= 6) {
    int tamanho = 4 + (quadro % 2) * 2;         // raio curto, raio longo, curto...
    int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};    // as 8 direcoes dos raios
    int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    for (int i = 0; i < 8; i++) {
      int inicio = raio + 3;
      display.drawLine(cx + dx[i] * inicio, cy + dy[i] * inicio,
                       cx + dx[i] * (inicio + tamanho), cy + dy[i] * (inicio + tamanho), SSD1306_WHITE);
    }
  }
}

// Animacao curta (uns 1,5 s) quando muda entre chuva e pos-chuva
void animacaoModo(bool chuva) {
  tocar(0);                               // nada de som preso durante a animacao
  for (int quadro = 0; quadro < 20; quadro++) {
    display.clearDisplay();
    if (chuva) {
      escreverCentralizado(txtChuvaComecou[idioma], 4, 1);
      desenharChuva(quadro);
      escreverCentralizado(txtAnimGota[idioma], 54, 1);       // "Monitorando a agua"
    } else {
      escreverCentralizado(txtChuvaAcabou[idioma], 4, 1);
      desenharSol(64, 34, quadro);
      escreverCentralizado(txtVerificando[idioma], 54, 1);    // "Verificando bueiro"
    }
    display.display();
    delay(70);
  }
}

// Logo do Fluxo Seguro: escudo cheio com a gota recortada no meio
void desenharEscudoCheio(int cx, int topo) {
  display.fillRect(cx - 18, topo, 37, 23, SSD1306_WHITE);
  display.fillTriangle(cx - 18, topo + 22, cx + 18, topo + 22, cx, topo + 40, SSD1306_WHITE);
  desenharGota(cx, topo + 22, SSD1306_BLACK);
}

void ligarLeds(bool verde, bool amarelo, bool vermelho) {
  digitalWrite(PINO_LED_VERDE, verde);
  digitalWrite(PINO_LED_AMARELO, amarelo);
  digitalWrite(PINO_LED_VERMELHO, vermelho);
}

void animacaoAbertura() {
  bool pulou = false;

  // 0) Previsao de chuva: gotas caindo e "Sistema iniciando..."
  String iniciando = txtSistIniciando[idioma];
  int xIniciando = (128 - (iniciando.length() + 3) * 6) / 2;   // o lugar dos "..." ja conta
  for (int quadro = 0; quadro < 30 && !pulou; quadro++) {
    display.clearDisplay();
    escreverCentralizado(txtPrevChuva[idioma], 4, 1);
    desenharChuva(quadro);
    String pontos = "";
    for (int i = 0; i < (quadro / 5) % 4; i++) {
      pontos = pontos + ".";              // . .. ... (um ponto a mais a cada 5 quadros)
    }
    display.setTextSize(1);
    display.setCursor(xIniciando, 54);
    display.print(iniciando + pontos);
    pulou = mostrarQuadro(60);
  }

  // 1) O contorno do escudo e desenhado, lado por lado (LED verde)
  ligarLeds(true, false, false);
  for (int quadro = 0; quadro <= 25 && !pulou; quadro++) {
    display.clearDisplay();
    escreverCentralizado(txtAnimEscudo[idioma], 4, 1);
    desenharContornoEscudo(64, 20, quadro / 5.0);
    pulou = mostrarQuadro(30);
  }

  // 2) Uma gota cai do alto e para no meio do escudo (LED amarelo)
  ligarLeds(false, true, false);
  for (int y = 0; y <= 42 && !pulou; y = y + 3) {
    display.clearDisplay();
    if (y >= 30) {
      escreverCentralizado(txtAnimGota[idioma], 4, 1);   // o texto aparece quando a gota chega
    }
    desenharContornoEscudo(64, 20, 5);
    desenharGota(64, y, SSD1306_WHITE);
    pulou = mostrarQuadro(30);
  }

  // 3) O escudo enche e vira a logo, com um clarao (LED vermelho)
  if (!pulou) {
    ligarLeds(false, false, true);
    display.clearDisplay();
    escreverCentralizado(txtAnimPronto[idioma], 4, 1);
    desenharEscudoCheio(64, 20);
    display.invertDisplay(true);          // clarao
    pulou = mostrarQuadro(80);
    display.invertDisplay(false);
    if (mostrarQuadro(400)) {
      pulou = true;
    }
  }

  // 4) A logo vai para a esquerda e FLUXO SEGURO aparece letra por letra
  for (int passo = 0; passo <= 10 && !pulou; passo++) {
    display.clearDisplay();
    desenharEscudoCheio(64 - passo * 4, 20);    // o centro anda de 64 ate 24
    pulou = mostrarQuadro(30);
  }
  String palavra1 = "FLUXO";
  String palavra2 = "SEGURO";
  for (int letras = 1; letras <= 11 && !pulou; letras++) {
    display.clearDisplay();
    desenharEscudoCheio(24, 20);
    display.setTextSize(2);
    display.setCursor(50, 22);
    display.print(palavra1.substring(0, letras));          // primeiro FLUXO
    if (letras > 5) {
      display.setCursor(50, 42);
      display.print(palavra2.substring(0, letras - 5));    // depois SEGURO
    }
    pulou = mostrarQuadro(60);
  }

  // 5) Subtitulo, os 3 LEDs acendem um por um e um "plim" baixinho
  if (!pulou) {
    escreverCentralizado(txtSubtitulo[idioma], 4, 1);     // na faixa amarela
    display.display();
    ligarLeds(true, false, false);
    delay(200);
    ligarLeds(true, true, false);
    delay(200);
    ligarLeds(true, true, true);
    tocarBaixinho(1568);
    delay(70);
    tocarBaixinho(2093);
    delay(110);
    tocar(0);
    delay(1000);
  }

  // 6) A rua de exemplo que o sistema vai monitorar
  if (!pulou) {
    ligarLeds(false, false, false);
    display.clearDisplay();
    faixaMarca();
    escreverCentralizado(txtRuaMonitorada[idioma], 22, 1);
    escreverCentralizado(RUA_NA_TELA, 36, 1);
    escreverCentralizado(BAIRRO_NA_TELA, 48, 1);
    pulou = mostrarQuadro(1800);
  }

  display.invertDisplay(false);
  ligarLeds(false, false, false);
  while (digitalRead(PINO_BOTAO) == LOW) {
    delay(10);                            // espera soltar o botao (se pulou)
  }
}

// ============================================================================
//  INTERNET: manda os dados para o servidor (banco de dados + site).
//  O envio roda numa tarefa separada do ESP32 (como um segundo programa
//  rodando junto), para a internet nunca atrasar LEDs, buzzer e tela.
// ============================================================================
// Dados para o site: o loop escreve aqui e a tarefa da internet le e envia
bool temDadoParaEnviar = false;
bool avisarInicio = false;         // "Monitoramento ativo" ainda nao chegou no site
int dadoEvento = EVENTO_LEITURA;
int dadoNivel = SEM_NIVEL;         // numero do nivel (texto nao: as duas tarefas mexem aqui)
float dadoPct = 0;
float dadoDistancia = -1;
float dadoPrevisao = -1;
bool dadoSubindo = false;
bool dadoChovendo = true;
float dadoLixo = 0;
bool dadoManutencao = false;

// Situacao da internet (para a linha de status no Serial)
bool internetLigada = false;       // so manda depois do fim do setup
bool wifiConectado = false;
bool enviando = false;             // true enquanto um envio esta acontecendo
int enviosOk = 0;
int enviosErro = 0;
int ultimoCodigo = 0;              // resposta do ultimo envio (201 = deu certo)
int ultimoNivelEnviado = SEM_NIVEL;
float ultimoPctEnviado = 0;
unsigned long ultimoTempoEnvioMs = 0;

WiFiClientSecure conexao;          // conexao segura (HTTPS) com o servidor
HTTPClient http;

// Nome do nivel do jeito que o servidor entende
String nomeParaServidor(int nivel) {
  if (nivel == FALHA) {
    return "FALHA";
  } else if (nivel == SEM_NIVEL) {
    return "INICIANDO";
  } else {
    return nomeNivel[0][nivel];         // o site entende os nomes em portugues
  }
}

// Chamada pelo loop: guarda o dado de agora para a tarefa da internet enviar
void avisarSite(int evento) {
  if (!USAR_INTERNET || !internetLigada) {
    return;
  }
  calcularPrevisao();
  dadoEvento = evento;
  if (falhaSensor) {
    dadoNivel = FALHA;
  } else {
    dadoNivel = nivelConfirmado;
  }
  if (falhaSensor || qtdLeituras == 0) {
    dadoPct = 0;
    dadoDistancia = -1;
  } else {
    dadoPct = nivelPct;
    dadoDistancia = distanciaMediaCm;
  }
  if (tipoPrevisao == PREV_MINUTOS) {
    dadoPrevisao = previsaoMin;
  } else if (tipoPrevisao == PREV_ATINGIDO) {
    dadoPrevisao = 0;
  } else {
    dadoPrevisao = -1;
  }
  dadoSubindo = aguaSubindo;
  dadoChovendo = chovendo;
  dadoLixo = lixoPct;
  dadoManutencao = manutencaoPedida;
  temDadoParaEnviar = true;               // o dado novo toma o lugar do que nao saiu
}

void conectarWifi() {
  if (WIFI_NOME == "Wokwi-GUEST") {
    WiFi.begin(WIFI_NOME.c_str(), WIFI_SENHA.c_str(), 6);   // canal 6: conecta rapido no Wokwi
  } else {
    WiFi.begin(WIFI_NOME.c_str(), WIFI_SENHA.c_str());
  }
}

// Manda o dado guardado para o servidor. Devolve true se chegou.
bool enviarParaServidor() {
  // Copia o dado (o loop pode escrever um novo enquanto este e enviado)
  String evento = "leitura";
  if (avisarInicio) {
    evento = "inicio";
  } else if (dadoEvento == EVENTO_ALERTA) {
    evento = "alerta";
  } else if (dadoEvento == EVENTO_FALHA) {
    evento = "falha";
  }
  String nivel = nomeParaServidor(dadoNivel);
  float pct = dadoPct;
  String modo = "chuva";
  if (!dadoChovendo) {
    modo = "pos_chuva";
  }

  // Monta o JSON (o formato que o servidor entende)
  String json = "{";
  json = json + "\"sensor_id\":\"" + SENSOR_ID + "\",";
  json = json + "\"local\":\"" + LOCAL_SENSOR + "\",";
  json = json + "\"evento\":\"" + evento + "\",";
  json = json + "\"nivel\":\"" + nivel + "\",";
  json = json + "\"nivel_pct\":" + String(pct, 1) + ",";
  json = json + "\"distancia_cm\":" + String(dadoDistancia, 1) + ",";
  json = json + "\"previsao_min\":" + String(dadoPrevisao, 1) + ",";
  json = json + "\"modo\":\"" + modo + "\",";
  json = json + "\"lixo_pct\":" + String(dadoLixo, 1) + ",";
  if (dadoManutencao) {
    json = json + "\"manutencao\":true,";
  } else {
    json = json + "\"manutencao\":false,";
  }
  if (dadoSubindo) {
    json = json + "\"subindo\":true}";
  } else {
    json = json + "\"subindo\":false}";
  }

  unsigned long inicio = millis();
  enviando = true;
  http.begin(conexao, SERVIDOR_URL);
  http.setTimeout(8000);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Chave", CHAVE_SERVIDOR);     // sem a chave certa o servidor recusa
  int codigo = http.POST(json);                  // envia; 201 = guardado no banco
  if (codigo > 0) {
    http.getString();                     // le a resposta toda
  }
  http.end();                             // com setReuse(true) a conexao continua aberta
  if (codigo <= 0) {
    conexao.stop();                       // deu erro: fecha; o proximo envio abre outra
  }
  ultimoTempoEnvioMs = millis() - inicio;
  ultimoCodigo = codigo;
  enviando = false;

  bool deuCerto = (codigo == 200 || codigo == 201);
  if (deuCerto) {
    enviosOk = enviosOk + 1;
    ultimoNivelEnviado = dadoNivel;
    ultimoPctEnviado = pct;
    if (evento == "inicio") {
      avisarInicio = false;
    }
  } else {
    enviosErro = enviosErro + 1;
  }
  // As leituras aparecem na linha de status; aqui so os eventos e os erros
  if (!deuCerto || evento != "leitura") {
    Serial.println("[INTERNET] Enviado \"" + evento + "\" (" + nivel + ") -> resposta " +
                   String(codigo) + " em " + String(ultimoTempoEnvioMs) + " ms");
  }
  return deuCerto;
}

// A tarefa da internet: repete para sempre, separada do loop
void tarefaInternet(void* parametro) {
  WiFi.mode(WIFI_STA);
  conectarWifi();
  unsigned long ultimaTentativa = millis();
  conexao.setInsecure();                  // nao confere o certificado (suficiente para o prototipo)
  conexao.setHandshakeTimeout(30);        // desiste da conexao segura apos 30 s (e tenta de novo)
  http.setConnectTimeout(10000);          // desiste de achar o servidor apos 10 s
  http.setReuse(true);                    // mantem a conexao aberta entre os envios

  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      if (wifiConectado) {
        wifiConectado = false;
        Serial.println("[INTERNET] Sem Wi-Fi (tentando de novo).");
        conexao.stop();
      }
      if (millis() - ultimaTentativa > 15000) {     // tenta reconectar a cada 15 s
        WiFi.disconnect();
        conectarWifi();
        ultimaTentativa = millis();
      }
    } else {
      if (!wifiConectado) {
        wifiConectado = true;
        Serial.println("[INTERNET] Wi-Fi conectado. IP: " + WiFi.localIP().toString());
      }
      if (temDadoParaEnviar) {
        temDadoParaEnviar = false;
        bool deuCerto = enviarParaServidor();
        if (!deuCerto) {
          temDadoParaEnviar = true;       // tenta de novo (com o dado mais novo)
          delay(2000);
        }
      }
    }
    delay(200);                           // dentro da tarefa, o delay nao atrasa o loop
  }
}

void iniciarInternet() {
  if (!USAR_INTERNET) {
    return;
  }
  // Cria a tarefa da internet: (funcao, nome, memoria 24 KB, -, prioridade 1, -, nucleo 1).
  // Nucleo 1 e o mesmo do loop: no nucleo 0 o HTTPS demorava e o "vigia" do ESP32
  // (watchdog) achava que ele tinha travado e reiniciava.
  xTaskCreatePinnedToCore(tarefaInternet, "internet", 24576, NULL, 1, NULL, 1);
}

// Uma linha a cada 5 s no Monitor Serial dizendo como esta a internet
void escreverStatusInternet() {
  if (!USAR_INTERNET) {
    return;
  }
  if (!wifiConectado) {
    Serial.println("[INTERNET] Sem Wi-Fi ainda (rede: " + WIFI_NOME + ")");
    return;
  }
  String linha = "[INTERNET] Wi-Fi OK | enviados: " + String(enviosOk) + " | erros: " + String(enviosErro);
  if (enviosOk > 0) {
    linha = linha + " | no site agora: " + nomeParaServidor(ultimoNivelEnviado) + " " + String(ultimoPctEnviado, 1) + "%";
  }
  linha = linha + " | ultima resposta: " + String(ultimoCodigo);
  if (ultimoCodigo == 0 && enviando) {
    linha = linha + " (conectando no servidor: a 1a vez leva ate 1 minuto)";
  } else if (ultimoCodigo == 0) {
    linha = linha + " (ainda nao enviou)";
  } else if (ultimoCodigo == 200 || ultimoCodigo == 201) {
    linha = linha + " = OK em " + String(ultimoTempoEnvioMs) + " ms";
  } else if (ultimoCodigo == 401) {
    linha = linha + " = chave errada";
  } else if (ultimoCodigo == 404) {
    linha = linha + " = endereco errado";
  } else {
    linha = linha + " = erro de conexao";
  }
  Serial.println(linha);
}

// ============================================================================
//  SETUP E LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  if (MAQUETE_3D) {                       // medidas da maquete impressa
    distanciaVaziaCm = 12.0;
    distanciaTransbordaCm = 4.0;
    toleranciaCalibracaoCm = 1.5;
  }
  alturaUtilCm = distanciaVaziaCm - distanciaTransbordaCm;
  iniciarInternet();                      // o Wi-Fi conecta enquanto roda a abertura

  pinMode(PINO_TRIG, OUTPUT);
  pinMode(PINO_ECHO, INPUT);
  pinMode(PINO_LED_VERDE, OUTPUT);
  pinMode(PINO_LED_AMARELO, OUTPUT);
  pinMode(PINO_LED_VERMELHO, OUTPUT);
  pinMode(PINO_BOTAO, INPUT_PULLUP);      // solto = HIGH, apertado = LOW
  iniciarBuzzer();
  carregarConfiguracao();

  Wire.begin(PINO_SDA, PINO_SCL);
  // "false" no fim: o Wire ja foi ligado acima, nos pinos do OLED (nao liga de novo)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, true, false)) {
    Serial.println("[ERRO] Display OLED nao encontrado (endereco 0x3C).");
  }
  display.setTextColor(SSD1306_WHITE);    // texto branco (sem isso nenhum texto aparece na abertura)
  if (ANIMACAO_INICIAL) {
    animacaoAbertura();
  } else {
    ligarLeds(true, false, false);        // so testa os LEDs
    delay(200);
    ligarLeds(true, true, false);
    delay(200);
    ligarLeds(true, true, true);
    delay(200);
    ligarLeds(false, false, false);
  }
  if (CALIBRAR_AO_LIGAR) {
    calibrar();
  }

  Serial.println("FLUXO SEGURO iniciado.");
  Serial.println("Altura util: " + String(alturaUtilCm, 1) + " cm");
  Serial.println("Idioma da tela: " + nomeIdioma[idioma]);
  Serial.println("Botao: 1 clique abre o menu | 2 cliques: portugues/ingles | 3 cliques: liga/desliga a chuva.");
  Serial.println("No menu: clique muda o valor e segurar 1 s desce a linha.");
  Serial.println("Menu, 1a linha \"Chuva\": Nao = a chuva acabou (mede o lixo), Sim = chovendo.");

  internetLigada = true;
  avisarInicio = true;
  avisarSite(EVENTO_LEITURA);             // o site mostra "Monitoramento ativo"
}

void loop() {
  unsigned long agora = millis();

  lerBotao(agora);

  if (agora - tempoLeitura >= INTERVALO_LEITURA_MS) {   // a cada 250 ms
    tempoLeitura = agora;
    processarLeitura();
  }
  if (agora - tempoHistorico >= 1000) {                 // a cada 1 s
    tempoHistorico = agora;
    guardarHistorico(agora);
    calcularPrevisao();
    verificarManutencao();                              // chuva e pos-chuva
  }
  if (agora - tempoTela >= 200) {                       // a cada 200 ms
    tempoTela = agora;
    atualizarTela(agora);
  }
  if (agora - tempoSerial >= 1000) {                    // a cada 1 s
    tempoSerial = agora;
    escreverNoSerial();
  }
  if (agora - tempoEnvio >= INTERVALO_ENVIO_MS) {       // a cada 2 s
    tempoEnvio = agora;
    avisarSite(EVENTO_LEITURA);
  }
  if (agora - tempoStatusInternet >= 5000) {            // a cada 5 s
    tempoStatusInternet = agora;
    escreverStatusInternet();
  }

  atualizarLeds(agora);                                 // sempre: o pisca fica no ritmo
  atualizarBuzzer(agora);
  delay(1);   // pausa de 1 ms: sobra processador para a internet (e o Wokwi roda mais rapido)
}