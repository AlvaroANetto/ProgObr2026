#include <math.h>
#include <QTRSensors.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_TCS34725.h>

// ========================================================
// PARÂMETROS DE THRESHOLD E CALIBRAÇÃO 
// ========================================================
const uint16_t QTR_TH_PRETO = 700;       // Valor do QTR para considerar fita/ar (>)
const uint16_t QTR_TH_LINHA = 200;       // Valor do QTR para considerar linha central (>)

// Thresholds de cor e altura do TCS
const uint16_t TCS_CLEAR_MIN = 400;      // Intensidade (C) mínima: abaixo disso é ar/redutor
const uint16_t TCS_CLEAR_MAX = 3200;     // Intensidade (C) máxima: acima disso é fundo branco
const float TCS_FATOR_VERDE_R = 1.30;    // G precisa ser 30% maior que R
const float TCS_FATOR_VERDE_B = 1.20;    // G precisa ser 20% maior que B
const uint16_t TCS_VERDE_G_MINIMO = 90;  // Valor absoluto mínimo do canal G para ser verde

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
// VARIÁVEIS DO ENCODER
// ========================================================
#define DIAMETRO_RODA 3.0
#define PULSOS_POR_VOLTA 226
#define DISTANCIA_ENTRE_RODAS 15
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

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(PINO_LED, OUTPUT);

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
  // testarVerdeDebug();
  //virarDireita(120, 90);
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
  uint16_t position = qtr.readLineBlack(sensorValues);

  // 
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
    moverCentimetros(velocidadeBase, 2);
    escreverDisplay("consegui");
    controlarMotoresPID(velocidadeBase + 30, velocidadeBase + 30);
    delay(250);
    return; // Aborta cruzamento e empurra o robô para subir
  }

  // 2. VERIFICAÇÃO DE CRUZAMENTO REAL
  bool marcaEsq = (sensorValues[0] > QTR_TH_PRETO);
  bool marcaDir = (sensorValues[5] > QTR_TH_PRETO);

  if (marcaEsq || marcaDir) {
    moverCentimetros(velocidadeBase, 0.8);
    parar(0);
    delay(30);

    checarQuadradosVerdes();

    if (viuVerdeEsq && viuVerdeDir) {
      escreverDisplay("180 Graus");
      moverCentimetros(120, -1);
      virarDireita(120, 150);
      direitaAtePreto();
    } else if (viuVerdeEsq) {
      escreverDisplay("Verde Dir");
      moverCentimetros(120, 0);
      parar(0);
      virarDireita(120, 80);
      direitaAtePreto();
      moverCentimetros(120, 3);
    } else if (viuVerdeDir) {
      escreverDisplay("Verde Esq");
      moverCentimetros(120, 0);
      virarEsquerda(120, 80);
      esquerdaAtePreto();
      moverCentimetros(120, 3);
    } else {
      moverCentimetros(velocidadeBase, 0.7);
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

  if (perdeuLinha) {
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

// =======================================================
// LÓGICA TCS COM PARÂMETROS GLOBAIS
// =======================================================
void checarQuadradosVerdes() {
  uint16_t r, g, b, c;

  // ESQUERDO (Porta 0)
  tcaselect(0);
  tcsEsq.getRawData(&r, &g, &b, &c);
  bool noChaoEsq = (c >= TCS_CLEAR_MIN && c <= TCS_CLEAR_MAX);
  bool eVerdeEsq = (g >= TCS_VERDE_G_MINIMO) && 
                   (g > r * TCS_FATOR_VERDE_R) && 
                   (g > b * TCS_FATOR_VERDE_B);
  viuVerdeEsq = (noChaoEsq && eVerdeEsq);

  // DIREITO (Porta 1)
  tcaselect(1);
  tcsDir.getRawData(&r, &g, &b, &c);
  bool noChaoDir = (c >= TCS_CLEAR_MIN && c <= TCS_CLEAR_MAX);
  bool eVerdeDir = (g >= TCS_VERDE_G_MINIMO) && 
                   (g > r * TCS_FATOR_VERDE_R) && 
                   (g > b * TCS_FATOR_VERDE_B);
  viuVerdeDir = (noChaoDir && eVerdeDir);
}

void testarVerdeDebug() {
  parar(0);
  uint16_t rEsq, gEsq, bEsq, cEsq;
  uint16_t rDir, gDir, bDir, cDir;

  tcaselect(0);
  tcsEsq.getRawData(&rEsq, &gEsq, &bEsq, &cEsq);
  bool vEsq = (cEsq >= TCS_CLEAR_MIN && cEsq <= TCS_CLEAR_MAX) &&
              (gEsq >= TCS_VERDE_G_MINIMO) &&
              (gEsq > rEsq * TCS_FATOR_VERDE_R) && 
              (gEsq > bEsq * TCS_FATOR_VERDE_B);

  tcaselect(1);
  tcsDir.getRawData(&rDir, &gDir, &bDir, &cDir);
  bool vDir = (cDir >= TCS_CLEAR_MIN && cDir <= TCS_CLEAR_MAX) &&
              (gDir >= TCS_VERDE_G_MINIMO) &&
              (gDir > rDir * TCS_FATOR_VERDE_R) && 
              (gDir > bDir * TCS_FATOR_VERDE_B);

  Serial.print("ESQ -> C:"); Serial.print(cEsq);
  Serial.print(" G:"); Serial.print(gEsq);
  Serial.print(vEsq ? " [VERDE!]" : " [----]");

  Serial.print(" | DIR -> C:"); Serial.print(cDir);
  Serial.print(" G:"); Serial.print(gDir);
  Serial.println(vDir ? " [VERDE!]" : " [----]");

  tcaselect(2);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("--- DEBUG TCS ---");
  display.print("Esq C="); display.print(cEsq); display.println(vEsq ? " VERDE" : " ---");
  display.print("Dir C="); display.print(cDir); display.println(vDir ? " VERDE" : " ---");
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