# Fluxo Seguro — alerta de alagamento direto na rua

**Global Solution 2026.2 · CitizenTech · Edge Computing & Computer Systems**
Equipe Core · 1ESPA

## Ficha da CitizenTech

| | |
|---|---|
| **Nome** | Fluxo Seguro |
| **Cidade / bairro** | São Paulo – Bela Vista (local de exemplo: Av. Paulista, 1106) |
| **Cidadão beneficiado** | Pedestres, motoristas e moradores de ruas que alagam; equipe de manutenção de bueiros |
| **Problema (1 frase)** | A rua alaga em poucos minutos e quem passa só percebe quando já está dentro da água. |
| **Como funciona** | Um sensor no bueiro mede a altura da água, o ESP32 decide o nível de risco e avisa na hora, num painel na calçada (LEDs, buzzer e tela), mesmo sem internet. Com Wi-Fi, também manda os dados para um site. |
| **O que cada parte faz** | Sensor: mede · ESP32: decide · LEDs, buzzer e tela: avisam quem passa · botão: chuva/pós-chuva e idioma · site: histórico e pedido de manutenção |
| **Indicador de impacto** | Minutos de antecedência do aviso (ATENÇÃO ~15 min e PERIGO ~10 min antes de alagar) e bueiros limpos antes da próxima chuva (limpeza pedida a partir de 25% de lixo) |
| **ODS** | ODS 11 – Cidades e comunidades sustentáveis (meta 11.5) · ODS 13 – Ação contra a mudança global do clima (meta 13.1) |
| **Repositório** | _(colar o link deste repositório)_ |

## Links

- **Simulação no Wokwi:** https://wokwi.com/projects/480151390850541569
- **Vídeo (YouTube):** 
- **Banco de dados ao vivo (painel do site):** https://leandror.pythonanywhere.com/
- **Link do repositório:**https://github.com/riquenasci/Edge-computing
## Equipe

| Nome | RM |
|---|---|
| Eduardo Mororó | RM575543 |
| Flávia | RM575681 |
| Henrique | RM576619 |
| Leandro | RM569915 |
| Vitor | RM575857 |

## Circuito

| Componente | Pinos | Para que serve |
|---|---|---|
| ESP32 DevKit | — | O "cérebro": lê o sensor, decide o nível e comanda o resto (mais pinos que o Arduino e Wi-Fi) |
| Sensor ultrassônico HC-SR04 | TRIG D23 · ECHO D22 | Mede a distância até a água pelo tempo do eco |
| Divisor de tensão (3 × 10 kΩ) | no ECHO | Baixa o sinal de 5 V do sensor para os 3,3 V do ESP32 |
| Tela OLED 128×64 (I2C) | SDA D4 · SCL D15 | Mostra o nível, a % de água e em quantos minutos vai alagar |
| LEDs verde, amarelo e vermelho + 300 Ω | D5 · D18 · D19 | Cores do semáforo: todo mundo entende na hora |
| Buzzer + 300 Ω | D21 | Alerta sonoro para quem não está olhando |
| Botão (pull-up interno) | GPIO17 | Menu e atalhos |

## Estados e limites

Nível = % da altura útil do bueiro. Os limites vêm da tabela de previsão do código (% de água → minutos até alagar).

| Estado | Água | LED | Buzzer | Tela | Por que este limite |
|---|---|---|---|---|---|
| NORMAL | < 40% | verde aceso | — | VIA LIVRE | abaixo disso, sem alarme à toa |
| ATENÇÃO | 40–60% | amarelo pisca 1× a cada 3 s | bip junto | CUIDADO! | ~15 min para alagar: dá tempo de avisar |
| PERIGO | 60–90% | vermelho pisca (0,5 s) | bips lentos | DESVIE! | ~10 min: ainda dá para trocar de caminho |
| EMERGÊNCIA | ≥ 90% | vermelho rápido (0,15 s) | bips rápidos | NÃO PASSE | < 1 min: a água está saindo do bueiro |
| FALHA | 3 leituras inválidas | apagados | — | Monitoramento indisponível | não mostra dado velho como se fosse atual |

**Pós-chuva:** quando a chuva acaba, o sistema mede o lixo e a água parada que ficaram no bueiro. Com 25% ou mais, pede limpeza (tela LIMPAR, amarelo piscando) e só volta a LIMPO abaixo de 20%.

## Lógica de borda (tudo no próprio ESP32)

- **Média móvel** das 5 últimas leituras (1 leitura a cada 250 ms).
- **Filtro de picos:** salto de mais de 15% em 250 ms fica "em análise" e só entra se repetir (objeto passando não vira alarme).
- **Confirmação tripla:** o estado só muda depois de 3 leituras seguidas iguais.
- **Leitura inválida:** sem eco ou acima de 400 cm conta como inválida; 3 seguidas = FALHA.
- **Previsão:** velocidade de subida da água nos últimos 30 s + tabela → "Alaga em X minutos".
- **millis():** leitura, tela, LEDs, buzzer e botão rodam juntos, sem travar.
- **Standalone:** o alerta funciona sem internet; o envio ao site roda numa tarefa separada e nunca atrasa o aviso.
- **Monitor Serial:** uma linha por segundo com distância, %, nível e previsão.

## Botão

| Ação (menu fechado) | O que faz |
|---|---|
| 1 clique | Abre o menu (Chuva · Idioma · Sair) |
| 2 cliques | Troca português ↔ inglês |
| 3 cliques | Liga/desliga a chuva (chuva ↔ pós-chuva) |
| Segurar 1 s (no menu) | Desce para a próxima linha |

## Como testar no Wokwi

1. Abra o link e clique em ▶. Deixe o sensor em **100 cm** ao ligar (bueiro vazio, usado na calibração).
2. Clique no sensor e mude a distância:

   | Distância | Estado |
   |---|---|
   | 100 cm | NORMAL |
   | 64 cm | ATENÇÃO |
   | 48 cm | PERIGO |
   | 26 cm | EMERGÊNCIA |

3. **Pós-chuva:** 3 cliques no botão e o sensor em 76 cm (~30% de lixo) → LIMPAR; volte para 100 cm → LIMPO.
4. Acompanhe pelo Monitor Serial e pelo painel do site.

**Maquete física:** o mesmo código com `MAQUETE_3D = true` (vazio 12 cm, transborda 4 cm): ATENÇÃO a 8,8 cm, PERIGO a 7,2 cm e EMERGÊNCIA a 4,8 cm, batendo com as marcas impressas no bueiro.

## Arquivos

- `sketch.ino` — código do ESP32, comentado
- `diagram.json` — circuito do Wokwi
- Bibliotecas: Adafruit SSD1306, Adafruit GFX e Adafruit BusIO

## Uso de IA

Usamos IA (Claude) como copiloto no desenvolvimento e na revisão do código, nos testes e na edição do vídeo. Os prompts estão registrados em `prompts.md`.
