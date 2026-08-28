#include <math.h>
#include <QTRSensors.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_TCS34725.h>
#include <MPU6050_light.h>
#include <Adafruit_VL53L0X.h>

// ========================================================
// PARÂMETROS DE THRESHOLD E CALIBRAÇÃO
// ========================================================
const uint16_t QTR_TH_PRETO = 650;  // Valor do QTR para considerar fita/ar (>)
const uint16_t QTR_TH_LINHA = 200;  // Valor do QTR para considerar linha central (>)

// Thresholds de cor e altura do TCS
const uint16_t TCS_CLEAR_MIN = 400;      // Intensidade (C) mínima: abaixo disso é ar/redutor
const uint16_t TCS_CLEAR_MAX = 3200;     // Intensidade (C) máxima: acima disso é fundo branco
const float TCS_FATOR_VERDE_R = 1.30;    // G precisa ser 30% maior que R
const float TCS_FATOR_VERDE_B = 1.20;    // G precisa ser 20% maior que B
const uint16_t TCS_VERDE_G_MINIMO = 90;  // Valor absoluto mínimo do canal G para ser verde
const float TCS_FATOR_VERMELHO_G = 1.40;
const float TCS_FATOR_VERMELHO_B = 1.30;
const uint16_t TCS_VERMELHO_R_MINIMO = 90;

// ========================================================
// DEFINIÇÕES DE PINOS MOTORES
// ========================================================
#define ENA 7
#define IN1 10
#define IN2 11
#define ENB 6
#define IN3 9
#define IN4 8

// ========================================================
// PINOS ENCODER, CALIBRAÇÃO E LED
// ========================================================
#define EncoderAmarelo 3
#define EncoderAzul 4
#define PINO_CALIBRACAO 12
#define PINO_LED 13

// ========================================================
//  PINOS Ultrasônico
// ========================================================

#define trigger A12
#define echo A11

// ========================================================
// VARIÁVEIS DO ENCODER
// ========================================================
#define DIAMETRO_RODA 3.0
#define PULSOS_POR_VOLTA 226
#define DISTANCIA_ENTRE_RODAS 19.5
volatile long degraus;

// ========================================================
// CONFIGURAÇÃO DO POLOLU
// ========================================================
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// ========================================================
// VARIÁVEIS DOS SENSORES DE COR
// ========================================================
bool viuVerdeEsq = false;
bool viuVerdeDir = false;

// ========================================================
// VARIÁVEIS DO PID E VELOCIDADE
// ========================================================
float Kp = 0.06;
float Kd = 1.7;
int erroAnterior = 0;

int velocidadeBase = 50;
int velocidadeMaxima = 150;

// ========================================================
// CONFIGS DISPLAY E MULTIPLEXADOR
// ========================================================
#define largura_display 128
#define altura_display 64
#define oled_reset -1
#define endereco_display 0x3c
Adafruit_SSD1306 display(largura_display, altura_display, &Wire, oled_reset);

#define TCAADDR 0x70

// Objetos dos Sensores TCS34725
Adafruit_TCS34725 tcsEsq = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 tcsDir = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);

// Giro
MPU6050 mpu(Wire);

// Infra
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#define CANAL_GIROSCOPIO 7
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}


// ========================================================
// SETUP
// ========================================================
void setup() {
  Serial.begin(9600);
  Wire.begin();

  tcaselect(2);
  if (!display.begin(SSD1306_SWITCHCAPVCC, endereco_display)) {
    Serial.println(("Falha no SSD1306"));
  }
  escreverDisplay("Iniciando...");

  tcaselect(0);
  if (!tcsEsq.begin()) Serial.println("Erro TCS Esquerdo!");

  tcaselect(1);
  if (!tcsDir.begin()) Serial.println("Erro TCS Direito!");

  tcaselect(CANAL_GIROSCOPIO);
  mpu.begin();
  mpu.calcOffsets();

  tcaselect(6);
  if (!lox.begin()) {
    Serial.println(F("Erro no VL53L0X lateral!"));
  } else {
    Serial.println(F("VL53L0X iniciado."));
  }

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(PINO_LED, OUTPUT);

  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);

  pinMode(EncoderAmarelo, INPUT_PULLUP);
  pinMode(EncoderAzul, INPUT_PULLUP);
  degraus = 0;
  attachInterrupt(digitalPinToInterrupt(EncoderAmarelo), contarDegraus, CHANGE);

  pinMode(PINO_CALIBRACAO, INPUT_PULLUP);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A4, A5, A6, A7, A8, A9 }, SensorCount);

  if (digitalRead(PINO_CALIBRACAO) == LOW) {
    escreverDisplay("Calibrando :3 ");
    digitalWrite(PINO_LED, HIGH);

    for (uint16_t i = 0; i < 400; i++) {
      qtr.calibrate();
    }

    digitalWrite(PINO_LED, LOW);
    int endereco = 0;
    for (uint8_t i = 0; i < SensorCount; i++) {
      EEPROM.put(endereco, qtr.calibrationOn.minimum[i]);
      endereco += sizeof(uint16_t);
    }
    for (uint8_t i = 0; i < SensorCount; i++) {
      EEPROM.put(endereco, qtr.calibrationOn.maximum[i]);
      endereco += sizeof(uint16_t);
    }
    escreverDisplay("Pronto p/ correr");
    for (int i = 0; i < 6; i++) {
      digitalWrite(PINO_LED, !digitalRead(PINO_LED));
      delay(200);
    }
  } else {
    escreverDisplay("Carregando EEPROM");
    qtr.calibrate();
    int endereco = 0;
    for (uint8_t i = 0; i < SensorCount; i++) {
      EEPROM.get(endereco, qtr.calibrationOn.minimum[i]);
      endereco += sizeof(uint16_t);
    }
    for (uint8_t i = 0; i < SensorCount; i++) {
      EEPROM.get(endereco, qtr.calibrationOn.maximum[i]);
      endereco += sizeof(uint16_t);
    }
    escreverDisplay("Pronto  :)");
  }
}

// ========================================================
// LOOP PRINCIPAL
// ========================================================
void loop() {
  seguirLinhaPID();
  //testarVerdeDebug();
  //virarDireita(120, 90);
  //girarGraus(120, 90, 'D');
  //delay(2000);
}

int compensarZonaMorta(int vel) {
  int minPWM = 95;
  if (vel == 0) return 0;
  if (vel > 0) return map(vel, 1, 255, minPWM, 255);
  else return map(abs(vel), 1, 255, minPWM, 255) * -1;
}

// ========================================================
// LÓGICA DO PID COM FILTRO DE REDUTOR
// ========================================================
void seguirLinhaPID() {

  // ----------------------------------------------------
  // CHECAGEM DE OBSTÁCULO (Executa a cada 60ms)
  // ----------------------------------------------------
  static unsigned long tempoUltimoUS = 0;
  if (millis() - tempoUltimoUS > 60) {
    tempoUltimoUS = millis();
    float dist = lerDistanciaCM();

    if (dist <= 10.0) {  // Obstáculo detectado a 10cm ou menos
      escreverDisplay("OBSTACULO!");
      parar(300);
      desviarObstaculo();

      return;
    }
  }
  //
  uint16_t position = qtr.readLineBlack(sensorValues);


  // Conta quantos sensores estão lendo preto ao mesmo tempo
  uint8_t contagemPreto = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > QTR_TH_PRETO) {
      contagemPreto++;
    }
  }

  // Se 5 ou 6 sensores viram preto, é a frente levantando no redutor!
  if (contagemPreto >= 5) {
    escreverDisplay("Redutor!");
    controlarMotoresPID(velocidadeBase + 30, velocidadeBase + 30);
    escreverDisplay("avanca");
    controlarMotoresPID(velocidadeBase + 30, velocidadeBase + 30);
    escreverDisplay("consegui");
    controlarMotoresPID(velocidadeBase + 30, velocidadeBase + 30);
    delay(250);
    return;  // Aborta cruzamento e empurra o robô para subir
  }

  // 2. VERIFICAÇÃO DE CRUZAMENTO
  bool marcaEsq = (sensorValues[0] > QTR_TH_PRETO);
  bool marcaDir = (sensorValues[5] > QTR_TH_PRETO);

  if (marcaEsq || marcaDir) {
    moverCentimetros(velocidadeBase, 0.8);
    parar(0);
    delay(30);

    checarQuadradosVerdes();  // verifica qual lado viu verde

    if (viuVerdeEsq && viuVerdeDir) {
      escreverDisplay("180 Graus");
      moverCentimetros(120, -1);
      virarDireita(120, 150);
      direitaAtePreto();
    } else if (viuVerdeEsq) {
      escreverDisplay("Verde Dir");
      virarDireita(120, 60);
      direitaAtePreto();
      moverCentimetros(120, 4);
    } else if (viuVerdeDir) {
      escreverDisplay("Verde Esq");
      moverCentimetros(120, 1.5);
      virarEsquerda(120, 60);
      esquerdaAtePreto();
      moverCentimetros(120, 4);
    } else {
      moverCentimetros(velocidadeBase, 0.7);  // Avança a linha e procura se é um cruzamento
      qtr.readLineBlack(sensorValues);

      bool linhaContinua = (sensorValues[2] > QTR_TH_LINHA || sensorValues[3] > QTR_TH_LINHA);

      if (linhaContinua) {
        escreverDisplay("Cruzamento Reto");
        moverCentimetros(velocidadeBase + 30, 2.0);
      } else {
        if (marcaEsq) {
          escreverDisplay("Curva 90 Esq");
          virarEsquerda(120, 46);
          esquerdaAtePreto();
        } else if (marcaDir) {
          escreverDisplay("Curva 90 Dir");
          virarDireita(120, 46);
          direitaAtePreto();
        }
      }
    }

    viuVerdeEsq = false;
    viuVerdeDir = false;
    return;
  }

  // GAP ou Perda de Linha
  bool perdeuLinha = true;
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > QTR_TH_LINHA) {
      perdeuLinha = false;
      break;
    }
  }

  if (perdeuLinha) {  // Ve se não parou por conta do vermelho
    uint16_t r, g, b, c;
    tcaselect(0);
    tcsEsq.getRawData(&r, &g, &b, &c);
    bool noChaoEsq = (c >= TCS_CLEAR_MIN && c <= TCS_CLEAR_MAX);
    bool eVermelhoEsq = noChaoEsq && (r >= TCS_VERMELHO_R_MINIMO) && (r > g * TCS_FATOR_VERMELHO_G) && (r > b * TCS_FATOR_VERMELHO_B);

    tcaselect(1);
    tcsDir.getRawData(&r, &g, &b, &c);
    bool noChaoDir = (c >= TCS_CLEAR_MIN && c <= TCS_CLEAR_MAX);
    bool eVermelhoDir = noChaoDir && (r >= TCS_VERMELHO_R_MINIMO) && (r > g * TCS_FATOR_VERMELHO_G) && (r > b * TCS_FATOR_VERMELHO_B);

    // Se qualquer um dos lados vir vermelho (ou exigir os dois, dependendo da pista)
    if (eVermelhoEsq || eVermelhoDir) {
      escreverDisplay("FIM DE PISTA!");
      parar(0);
      while (true) {
        // Trava o robô num loop infinito no final da pista
        digitalWrite(PINO_LED, HIGH);
        delay(500);
        digitalWrite(PINO_LED, LOW);
        delay(500);
      }
    }
    if (abs(erroAnterior) < 1000) {
      controlarMotoresPID(velocidadeBase, velocidadeBase);
      return;
    } else if (erroAnterior < 0) {
      controlarMotoresPID(-80, 80);
      return;
    } else {
      controlarMotoresPID(80, -80);
      return;
    }
  }

  // PID NORMAL
  int erro = position - 2500;
  int P = erro;
  int D = erro - erroAnterior;
  erroAnterior = erro;

  int calculoPID = (Kp * P) + (Kd * D);
  int velDir = velocidadeBase + calculoPID;
  int velEsq = velocidadeBase - calculoPID;

  velEsq = constrain(velEsq, -velocidadeMaxima, velocidadeMaxima);
  velDir = constrain(velDir, -velocidadeMaxima, velocidadeMaxima);

  controlarMotoresPID(velEsq, velDir);
}

void controlarMotoresPID(int velEsq, int velDir) {
  int pwmEsq = compensarZonaMorta(velEsq);
  int pwmDir = compensarZonaMorta(velDir);

  if (pwmEsq >= 0) motor_esq(pwmEsq);
  else motor_esqTras(abs(pwmEsq));

  if (pwmDir >= 0) motor_dir(pwmDir);
  else motor_dirTras(abs(pwmDir));
}

// ========================================================
// ENCODERS E MOTORES
// ========================================================
void contarDegraus() {
  if (digitalRead(EncoderAmarelo) == digitalRead(EncoderAzul)) degraus++;
  else degraus--;
}

long lerDegraus() {
  long contagemSegura;
  noInterrupts();
  contagemSegura = degraus;
  interrupts();
  return contagemSegura;
}

void girarGraus(int forca, float grausAlvo, char direcao) {
  tcaselect(CANAL_GIROSCOPIO);
  mpu.update();

  float anguloInicial = mpu.getAngleZ();
  float diferenca = 0;

  // 1. Compensação de Inércia
  float inercia = 5.0;
  float alvoReal = grausAlvo - inercia;

  if (alvoReal < 0) alvoReal = 1.0;  // Prevenção de segurança

  // Aciona os motores
  if (direcao == 'D' || direcao == 'd') {
    motor_esqTras(forca);
    motor_dir(forca);
  } else if (direcao == 'E' || direcao == 'e') {
    motor_esq(forca);
    motor_dirTras(forca);
  }

  // Fica preso no while enquanto a diferença for menor que o alvo
  while (diferenca < alvoReal) {
    tcaselect(CANAL_GIROSCOPIO);
    mpu.update();  // Atualiza a leitura

    diferenca = abs(mpu.getAngleZ() - anguloInicial);

    // 2. Filtro anti-ruído:
    // Um micro-respiro para o Multiplexador e o Sensor não travarem
    delay(2);
  }

  parar(200);  // Freia os motores
}

void virarGraus(int vel, float graus) {
  float grausAbs = abs(graus);

  // 1. Calcula quantas voltas a roda/esteira precisa dar para aquele ângulo
  float voltasRoda = (DISTANCIA_ENTRE_RODAS * grausAbs) / (360.0 * DIAMETRO_RODA);

  // 2. Converte as voltas em pulsos do encoder
  long pulsosAlvo = voltasRoda * PULSOS_POR_VOLTA;

  // 3. Reseta o contador do encoder
  noInterrupts();
  degraus = 0;
  interrupts();

  // 4. Aciona os motores em sentidos opostos (Giro no próprio eixo)
  if (graus > 0) {
    // Virar para a DIREITA (Esteira esq frente, esteira dir trás)
    motor_esq(vel);
    motor_dirTras(vel);
  } else {
    // Virar para a ESQUERDA (Esteira esq trás, esteira dir frente)
    motor_esqTras(vel);
    motor_dir(vel);
  }

  // 5. Aguarda atingir os pulsos (com timeout de segurança de 3 segundos)
  unsigned long tempoInicio = millis();
  while (abs(lerDegraus()) < pulsosAlvo && (millis() - tempoInicio < 3000)) {}

  // 6. Freia os motores
  parar(0);
}

// Funções auxiliares (facilitam a leitura do código)
void virarEsquerda(int vel, float graus) {
  virarGraus(vel, abs(graus));
}

void virarDireita(int vel, float graus) {
  virarGraus(vel, -abs(graus));
}

void motor_dir(int forca) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, forca);
}

void motor_dirTras(int forca) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, forca);
}

void motor_esq(int forca) {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, forca);
}

void motor_esqTras(int forca) {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, forca);
}

void motor_frente(int forca) {
  motor_dir(forca);
  motor_esq(forca);
}

void motor_tras(int forca) {
  motor_dirTras(forca);
  motor_esqTras(forca);
}

void parar(int tempo) {
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(tempo);
}

void direitaAtePreto() {
  qtr.readLineBlack(sensorValues);
  while (sensorValues[0] <= QTR_TH_LINHA && sensorValues[1] <= QTR_TH_LINHA) {
    motor_dir(120);
    motor_esqTras(120);
    qtr.readLineBlack(sensorValues);
  }
  parar(0);
}

void esquerdaAtePreto() {
  qtr.readLineBlack(sensorValues);
  while (sensorValues[5] <= QTR_TH_LINHA && sensorValues[4] <= QTR_TH_LINHA) {
    motor_esq(120);
    motor_dirTras(120);
    qtr.readLineBlack(sensorValues);
  }
  parar(0);
}

void moverDist(int vel, long G) {
  noInterrupts();
  degraus = 0;
  interrupts();

  if (G < 0) motor_tras(vel);
  else motor_frente(vel);

  unsigned long tempoInicio = millis();
  while (abs(lerDegraus()) < abs(G) && (millis() - tempoInicio < 2000)) {}
  parar(0);
}

void moverCentimetros(int vel, float distancia_cm) {
  float circunferencia = DIAMETRO_RODA * PI;
  float voltas = distancia_cm / circunferencia;
  long pulsos_alvo = voltas * PULSOS_POR_VOLTA;
  moverDist(vel, pulsos_alvo);
}

void frenteAtePreto(int vel) {
  qtr.readLineBlack(sensorValues);

  // Enquanto nenhum dos 6 sensores encontrar a linha preta:
  while (sensorValues[0] <= QTR_TH_LINHA && sensorValues[1] <= QTR_TH_LINHA && sensorValues[2] <= QTR_TH_LINHA && sensorValues[3] <= QTR_TH_LINHA && sensorValues[4] <= QTR_TH_LINHA && sensorValues[5] <= QTR_TH_LINHA) {
    motor_frente(vel);
    qtr.readLineBlack(sensorValues);
  }

  parar(0);  // Para assim que cruzar a fita
}
// ========================================================
// Obstaculo
// ========================================================

void desviarObstaculo() {
  escreverDisplay("Desviando...");

  // 1. Vira 90° para a DIREITA
  girarGraus(120, 90.0, 'D');
  parar(200);

  // 2. Liga os motores para ir para frente
  motor_frente(100);

  // 3. ETAPA A: "PROCURAR A CAIXA"
  long tempoSeguranca = millis();
  while (lerDistanciaLateral() > 250) {
    if (millis() - tempoSeguranca > 2500) break;
    delay(10);
  }

  motor_frente(100);
  delay(300);
  parar(200);

  motor_frente(100);

  // 4. ETAPA B: "ACOMPANHAR A CAIXA"
  while (lerDistanciaLateral() < 250) {
    delay(10);
  }

  // 5. Saiu do loop! 
  // CORREÇÃO: Trocado moverCentimetros por tempo para não travar
  motor_frente(100);
  delay(400); // Ajuste esse delay para simular os seus 7.5 cm antigos
  parar(200);

  // 6. Vira 90° para a ESQUERDA (Fica paralelo à linha original)
  girarGraus(120, 90.0, 'E');
  parar(200);
  
  // 7. Vai em direção à fita preta
  bool achouReta = moverAteLinhaOuDistancia(100, 60.0);

  if (achouReta) {
    escreverDisplay("Linha Encontrada");

    // CORREÇÃO: Trocado moverCentimetros por tempo
    motor_frente(100);
    delay(200); // Ajuste para simular os 3.0 cm
    parar(100);

    // Gira para a Esquerda para alinhar com a pista
    girarGraus(120, 90.0, 'D');

    // Trava o alinhamento
    direitaAtePreto();
    
    // Ré de assentamento
    motor_tras(100);
    delay(300);
    parar(100);
    
  } else {

    // Tratamento caso a pista dobre durante o obstáculo (Dobrou para a Esquerda)
    escreverDisplay("Pista dobrou!");
    girarGraus(120, 90.0, 'E');
    
    bool linha2 = moverAteLinhaOuDistancia(100, 50.0);
    
    if (linha2) {
      // === ELE ACHOU A LINHA 2! A AÇÃO ACONTECE AQUI DENTRO ===
      escreverDisplay("Linha 2 Encontrada!");
      
      // Avança o eixo da roda (simulando os 3.0 cm)
      motor_frente(100);
      delay(200); 
      parar(100);
      
      // Gira para a DIREITA para alinhar com a nova pista
      girarGraus(120, 90.0, 'D'); 
      
      // Trava o alinhamento
      direitaAtePreto();
      
      // Ré de assentamento
      motor_tras(100);
      delay(300);
      parar(100);
      
    } else {
      motor_frente(100);
      delay(200); 
      parar(0);
      girarGraus(120, 90.0, 'E');
      frenteAtePreto(120);
      girarGraus(120, 90.0, 'D');

      direitaAtePreto();
      
      // Ré de assentamento
      motor_tras(100);
      delay(300);
      parar(100); 
    }
  }
}

// ========================================================
// LEITURA DO ULTRASSÔNICO
// ========================================================
float lerDistanciaCM() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  // pulseIn com timeout de 10.000 microssegundos (~1.7 metros)
  long duracao = pulseIn(echo, HIGH, 10000);

  if (duracao == 0) {
    return 999.0;  // Sem obstáculo no alcance seguro
  }

  return (duracao * 0.0343) / 2.0;
}


bool moverAteLinhaOuDistancia(int vel, float cm) {
  float circunferencia = DIAMETRO_RODA * 3.14159;
  float voltas = cm / circunferencia;
  long pulsosAlvo = abs(voltas * PULSOS_POR_VOLTA);
  degraus = 0;


  if (cm < 0) motor_tras(vel);
  else motor_frente(vel);

  while (abs(lerDegraus()) < pulsosAlvo) {
    qtr.readLineBlack(sensorValues);
    // Se qualquer um dos sensores centrais ou laterais detectar a linha
    if (sensorValues[1] > QTR_TH_LINHA || sensorValues[2] > QTR_TH_LINHA || sensorValues[3] > QTR_TH_LINHA || sensorValues[4] > QTR_TH_LINHA) {
      parar(0);
      return true;  // Achou a linha antes do limite!
    }
  }
  parar(0);
  return false;  // Chegou ao fim da distância sem achar linha
}

int lerDistanciaLateral() {
  tcaselect(6);
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  // O RangeStatus 4 indica que o sensor não encontrou obstáculo dentro do limite
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;  // Retorna em milímetros (ex: 150 = 15 cm)
  } else {
    return 9999;  // Vazio absoluto
  }
}

// =======================================================
// LÓGICA TCS COM PARÂMETROS GLOBAIS
// =======================================================
void checarQuadradosVerdes() {
  uint16_t r, g, b, c;

  // ESQUERDO (Porta 0)
  tcaselect(0);
  tcsEsq.getRawData(&r, &g, &b, &c);
  bool noChaoEsq = (c >= TCS_CLEAR_MIN && c <= TCS_CLEAR_MAX);
  bool eVerdeEsq = (g >= TCS_VERDE_G_MINIMO) && (g > r * TCS_FATOR_VERDE_R) && (g > b * TCS_FATOR_VERDE_B);
  viuVerdeEsq = (noChaoEsq && eVerdeEsq);

  // DIREITO (Porta 1)
  tcaselect(1);
  tcsDir.getRawData(&r, &g, &b, &c);
  bool noChaoDir = (c >= TCS_CLEAR_MIN && c <= TCS_CLEAR_MAX);
  bool eVerdeDir = (g >= TCS_VERDE_G_MINIMO) && (g > r * TCS_FATOR_VERDE_R) && (g > b * TCS_FATOR_VERDE_B);
  viuVerdeDir = (noChaoDir && eVerdeDir);
}

void testarVerdeDebug() {
  parar(0);
  uint16_t rEsq, gEsq, bEsq, cEsq;
  uint16_t rDir, gDir, bDir, cDir;

  tcaselect(0);
  tcsEsq.getRawData(&rEsq, &gEsq, &bEsq, &cEsq);
  bool vEsq = (cEsq >= TCS_CLEAR_MIN && cEsq <= TCS_CLEAR_MAX) && (gEsq >= TCS_VERDE_G_MINIMO) && (gEsq > rEsq * TCS_FATOR_VERDE_R) && (gEsq > bEsq * TCS_FATOR_VERDE_B);

  tcaselect(1);
  tcsDir.getRawData(&rDir, &gDir, &bDir, &cDir);
  bool vDir = (cDir >= TCS_CLEAR_MIN && cDir <= TCS_CLEAR_MAX) && (gDir >= TCS_VERDE_G_MINIMO) && (gDir > rDir * TCS_FATOR_VERDE_R) && (gDir > bDir * TCS_FATOR_VERDE_B);

  Serial.print("ESQ -> C:");
  Serial.print(cEsq);
  Serial.print(" G:");
  Serial.print(gEsq);
  Serial.print(vEsq ? " [VERDE!]" : " [----]");

  Serial.print(" | DIR -> C:");
  Serial.print(cDir);
  Serial.print(" G:");
  Serial.print(gDir);
  Serial.println(vDir ? " [VERDE!]" : " [----]");

  tcaselect(2);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("--- DEBUG TCS ---");
  display.print("Esq C=");
  display.print(cEsq);
  display.println(vEsq ? " VERDE" : " ---");
  display.print("Dir C=");
  display.print(cDir);
  display.println(vDir ? " VERDE" : " ---");
  display.display();

  delay(300);
}

void testarVermelhoDebug() {
  parar(0);
  uint16_t rEsq, gEsq, bEsq, cEsq;
  uint16_t rDir, gDir, bDir, cDir;

  // Lê sensor Esquerdo
  tcaselect(0);
  tcsEsq.getRawData(&rEsq, &gEsq, &bEsq, &cEsq);
  bool vEsq = (cEsq >= TCS_CLEAR_MIN && cEsq <= TCS_CLEAR_MAX) && (rEsq >= TCS_VERMELHO_R_MINIMO) && (rEsq > gEsq * TCS_FATOR_VERMELHO_G) && (rEsq > bEsq * TCS_FATOR_VERMELHO_B);

  // Lê sensor Direito
  tcaselect(1);
  tcsDir.getRawData(&rDir, &gDir, &bDir, &cDir);
  bool vDir = (cDir >= TCS_CLEAR_MIN && cDir <= TCS_CLEAR_MAX) && (rDir >= TCS_VERMELHO_R_MINIMO) && (rDir > gDir * TCS_FATOR_VERMELHO_G) && (rDir > bDir * TCS_FATOR_VERMELHO_B);

  Serial.print("ESQ -> C:");
  Serial.print(cEsq);
  Serial.print(" R:");
  Serial.print(rEsq);
  Serial.print(vEsq ? " [VERMELHO!]" : " [--------]");

  Serial.print(" | DIR -> C:");
  Serial.print(cDir);
  Serial.print(" R:");
  Serial.print(rDir);
  Serial.println(vDir ? " [VERMELHO!]" : " [--------]");

  // Mostra no display
  tcaselect(2);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("--- DEBUG VERMELHO ---");
  display.print("Esq R=");
  display.print(rEsq);
  display.println(vEsq ? " RED" : " ---");
  display.print("Dir R=");
  display.print(rDir);
  display.println(vDir ? " RED" : " ---");
  display.display();

  delay(300);
}

void escreverDisplay(String escrito) {
  tcaselect(2);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(escrito);
  display.display();
}