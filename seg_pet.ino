// ============================================================
//              SEGUIDOR DE LINHA - ARDUINO NANO
// ============================================================
//
// 8 sensores IR analógicos
// Calibração automática
// Normalização
// Controle PD
// Correção não linear
// Correção das pontas
// Memória da direção da curva
// Recuperação girando no próprio eixo
//
// TRATAMENTO ESPECIAL:
// Se todos os sensores ficarem pretos:
// - Se vinha claramente fazendo curva -> continua a curva
// - Se vinha centralizado -> não assume que é curva
//   (protege cruzamentos)
//
// NOVO:
// Se perder a linha LOGO DEPOIS de uma faixa larga de preto,
// era curva fechada (não cruzamento) -> gira para o lado
// onde a linha foi vista por último.
//
// ============================================================


// ============================================================
// 1. SENSORES
// ============================================================

const int sensores[8] = {
  A1,
  A2,
  A3,
  A4,
  A5,
  A0,
  A6,
  A7
};


// ============================================================
// 2. PESOS DOS SENSORES
// ============================================================

const int pesos[8] = {
  -3500,
  -2500,
  -1500,
  -500,
   500,
   1500,
   2500,
   3500
};


// ============================================================
// 3. TB6612FNG
// ============================================================

const int PWMA = 3;
const int AIN2 = 4;
const int AIN1 = 5;

const int PWMB = 6;
const int BIN1 = 7;
const int BIN2 = 8;

const int STBY = 2;


// ============================================================
// 4. LED
// ============================================================

const int LED = LED_BUILTIN;


// ============================================================
// 5. CONTROLE PD
// ============================================================

float Kp = 190.0;

float Ki = 0.0;

float Kd = 0.5;


// ============================================================
// 6. VELOCIDADE
// ============================================================

int velocidadeBase = 165;

int velocidadeMinima = 30;

float reducaoCurva = 85.0;


// ============================================================
// 7. CORREÇÃO NÃO LINEAR
// ============================================================

float ganhoLinearCurva = 0.80;

float ganhoQuadraticoCurva = 0.80;


// ============================================================
// 8. CORREÇÃO DAS PONTAS
// ============================================================

float ganhoPonta = 15.0;


// ============================================================
// 9. RECUPERAÇÃO
// ============================================================

// Velocidade do giro no próprio eixo
int velocidadeGiro = 170;

// Tempo máximo de recuperação
unsigned long tempoMaximoRecuperacao = 3500;

bool recuperando = false;

unsigned long inicioRecuperacao = 0;


// ============================================================
// 10. MEMÓRIA DA DIREÇÃO
// ============================================================
//
// -1 = esquerda
//  1 = direita
//  0 = nenhuma
//
// ============================================================

int ultimaDirecao = 0;

unsigned long tempoUltimaDirecao = 0;

unsigned long validadeDirecao = 1000;

// Direção congelada durante recuperação
int direcaoRecuperacao = 0;

bool direcaoTravada = false;


// ============================================================
// 11. DETECÇÃO DE TODOS OS SENSORES PRETOS
// ============================================================

bool todosPretos = false;

const float LIMIAR_CURVA_TUDO_PRETO = 0.30;


// ============================================================
// 11B. NOVO - FAIXA LARGA DE PRETO (CURVA x CRUZAMENTO)
// ============================================================
//
// Cruzamento: depois da faixa preta a linha continua no centro.
// Curva fechada: depois da faixa preta a linha some.
//
// ============================================================

bool emFaixaPreta = false;          // está sobre uma faixa larga de preto

unsigned long fimFaixaPreta = 0;    // quando saiu dela

const unsigned long JANELA_POS_PRETO = 300;   // ms


// ============================================================
// 12. CALIBRAÇÃO
// ============================================================

float branco[8];

float preto[8];


// ============================================================
// 13. VARIÁVEIS
// ============================================================

float valoresNormalizados[8];

float erro = 0.0;

float ultimoErro = 0.0;

float posicaoLinha = 0.0;

unsigned long ultimoTempo = 0;


// ============================================================
// 14. SETUP
// ============================================================

void setup() {

  // Sensores
  for (int i = 0; i < 8; i++) {
    pinMode(sensores[i], INPUT);
  }

  // TB6612FNG
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // Motores parados
  parar();

  // Calibração
  calibrarBranco();
  calibrarPreto();

  // Inicialização
  erro = 0.0;
  ultimoErro = 0.0;
  ultimoTempo = micros();

  ultimaDirecao = 0;
  direcaoRecuperacao = 0;
  direcaoTravada = false;
  recuperando = false;
  todosPretos = false;

  // NOVO
  emFaixaPreta = false;
  fimFaixaPreta = 0;

  // Aguarda 2 segundos para alinhar
  delay(2000);

  ultimoTempo = micros();
}


// ============================================================
// 15. LOOP
// ============================================================

void loop() {

  bool encontrouLinha = calcularErro();


  // ==========================================================
  // NOVO - DETECTA FAIXA LARGA DE PRETO E QUANDO ELA TERMINA
  // ==========================================================

  int qtdPretos = 0;

  for (int i = 0; i < 8; i++) {
    if (valoresNormalizados[i] >= 0.70) {
      qtdPretos++;
    }
  }

  if (encontrouLinha && qtdPretos >= 6) {
    emFaixaPreta = true;
  }
  else if (emFaixaPreta) {
    emFaixaPreta = false;
    fimFaixaPreta = millis();
  }


  // ==========================================================
  // CASO ESPECIAL: TODOS OS SENSORES PRETOS
  // ==========================================================

  if (encontrouLinha && todosPretos) {

    // Vinha claramente para a esquerda
    if (ultimoErro < -LIMIAR_CURVA_TUDO_PRETO && ultimaDirecao < 0) {
      iniciarRecuperacaoPorCurva(-1);
      return;
    }

    // Vinha claramente para a direita
    if (ultimoErro > LIMIAR_CURVA_TUDO_PRETO && ultimaDirecao > 0) {
      iniciarRecuperacaoPorCurva(1);
      return;
    }

    // Provavelmente cruzamento: NÃO força giro
    controlePD();
    return;
  }


  // ==========================================================
  // LINHA NORMALMENTE ENCONTRADA
  // ==========================================================

  if (encontrouLinha) {

    // Se estava recuperando e encontrou novamente
    if (recuperando) {
      recuperando = false;
      ultimoErro = erro;
      ultimoTempo = micros();
    }

    // Direção travada
    if (direcaoTravada) {

      float esqPonta = max(valoresNormalizados[0], valoresNormalizados[1]);
      float dirPonta = max(valoresNormalizados[6], valoresNormalizados[7]);

      // Destrava somente quando voltar suficientemente para o centro
      if (fabs(erro) < 0.25 && esqPonta < 0.3 && dirPonta < 0.3) {
        direcaoTravada = false;
      }
    }
    else {
      atualizarDirecao();
    }

    // Controle normal
    controlePD();
    return;
  }


  // ==========================================================
  // LINHA REALMENTE PERDIDA
  // ==========================================================


  // ----------------------------------------------------------
  // NOVO - Perdeu a linha LOGO DEPOIS de uma faixa preta:
  // não era cruzamento, era curva fechada.
  // Gira para o lado onde a linha foi vista por último.
  // ----------------------------------------------------------

  if (
    !recuperando &&
    fimFaixaPreta != 0 &&
    millis() - fimFaixaPreta < JANELA_POS_PRETO
  ) {

    recuperando = true;
    inicioRecuperacao = millis();

    direcaoRecuperacao = (erro < 0) ? -1 : 1;
    direcaoTravada = true;

    fimFaixaPreta = 0;

    recuperarLinha();
    return;
  }


  if (!recuperando) {

    recuperando = true;
    inicioRecuperacao = millis();

    // Congela a direção no momento em que perdeu
    if (!direcaoTravada) {

      if (
        ultimaDirecao != 0 &&
        millis() - tempoUltimaDirecao <= validadeDirecao
      ) {
        direcaoRecuperacao = ultimaDirecao;
        direcaoTravada = true;
      }
      else {
        direcaoRecuperacao = 0;
      }
    }
  }

  recuperarLinha();
}


// ============================================================
// 16. INICIA RECUPERAÇÃO POR CURVA
// ============================================================

void iniciarRecuperacaoPorCurva(int direcao) {

  recuperando = true;
  inicioRecuperacao = millis();

  direcaoRecuperacao = direcao;
  direcaoTravada = true;

  recuperarLinha();
}


// ============================================================
// 17. ATUALIZA DIREÇÃO
// ============================================================

void atualizarDirecao() {

  float esq = max(valoresNormalizados[0], valoresNormalizados[1]);
  float dir = max(valoresNormalizados[6], valoresNormalizados[7]);

  // Curva para esquerda pelas pontas
  if (esq > 0.6 && dir < 0.3) {
    ultimaDirecao = -1;
    tempoUltimaDirecao = millis();
  }

  // Curva para direita pelas pontas
  else if (dir > 0.6 && esq < 0.3) {
    ultimaDirecao = 1;
    tempoUltimaDirecao = millis();
  }

  // Erro forte para esquerda
  else if (erro < -0.25) {
    ultimaDirecao = -1;
    tempoUltimaDirecao = millis();
  }

  // Erro forte para direita
  else if (erro > 0.25) {
    ultimaDirecao = 1;
    tempoUltimaDirecao = millis();
  }
}


// ============================================================
// 18. CONTROLE PD
// ============================================================

void controlePD() {

  unsigned long agora = micros();

  float dt = (agora - ultimoTempo) / 1000000.0;

  ultimoTempo = agora;

  if (dt < 0.001) {
    dt = 0.001;
  }

  // P
  float termoP = Kp * erro;

  // I
  float termoI = Ki * 0.0;

  // D
  float derivada = (erro - ultimoErro) / dt;
  float termoD = Kd * derivada;

  // Correção PD
  float correcao = termoP + termoI + termoD;

  // Correção não linear
  float erroAbsoluto = abs(erro);

  float fatorCorrecao =
    1.0
    + ganhoLinearCurva * erroAbsoluto
    + ganhoQuadraticoCurva * erroAbsoluto * erroAbsoluto;

  correcao *= fatorCorrecao;

  // Correção das pontas
  float esquerdaPontas = (valoresNormalizados[0] + valoresNormalizados[1]) / 2.0;
  float direitaPontas  = (valoresNormalizados[6] + valoresNormalizados[7]) / 2.0;

  float correcaoPontas = (direitaPontas - esquerdaPontas) * ganhoPonta;

  correcao += correcaoPontas;

  // Guarda o erro atual para a próxima iteração
  ultimoErro = erro;

  // Velocidade adaptativa
  int velocidadeAtual = velocidadeBase - erroAbsoluto * reducaoCurva;

  velocidadeAtual = constrain(velocidadeAtual, velocidadeMinima, velocidadeBase);

  // Motores
  int motorEsquerdo = velocidadeAtual - correcao;
  int motorDireito  = velocidadeAtual + correcao;

  motorEsquerdo = constrain(motorEsquerdo, -50, 255);
  motorDireito  = constrain(motorDireito, -50, 255);

  mover(motorEsquerdo, motorDireito);
}


// ============================================================
// 19. RECUPERAÇÃO DA LINHA
// ============================================================
//
// direcaoRecuperacao < 0 -> esquerda
// direcaoRecuperacao > 0 -> direita
//
// ============================================================

void recuperarLinha() {

  unsigned long agora = millis();

  // Tempo máximo
  if (agora - inicioRecuperacao >= tempoMaximoRecuperacao) {
    parar();
    return;
  }

  // Sem direção conhecida
  if (direcaoRecuperacao == 0) {
    mover(40, 40);
    return;
  }

  // Giro para esquerda
  if (direcaoRecuperacao < 0) {
    mover(velocidadeGiro, -velocidadeGiro);
    return;
  }

  // Giro para direita
  mover(-velocidadeGiro, velocidadeGiro);
}


// ============================================================
// 20. CALIBRAÇÃO DO BRANCO
// ============================================================

void calibrarBranco() {

  parar();

  for (int i = 0; i < 6; i++) {
    digitalWrite(LED, HIGH);
    delay(250);
    digitalWrite(LED, LOW);
    delay(250);
  }

  digitalWrite(LED, HIGH);

  for (int i = 0; i < 8; i++) {
    branco[i] = 1023;
  }

  unsigned long inicio = millis();

  while (millis() - inicio < 2000) {
    for (int i = 0; i < 8; i++) {
      int leitura = analogRead(sensores[i]);
      if (leitura < branco[i]) {
        branco[i] = leitura;
      }
    }
  }

  digitalWrite(LED, LOW);
}


// ============================================================
// 21. CALIBRAÇÃO DO PRETO
// ============================================================

void calibrarPreto() {

  parar();

  for (int i = 0; i < 6; i++) {
    digitalWrite(LED, HIGH);
    delay(250);
    digitalWrite(LED, LOW);
    delay(250);
  }

  digitalWrite(LED, HIGH);

  for (int i = 0; i < 8; i++) {
    preto[i] = 0;
  }

  unsigned long inicio = millis();

  while (millis() - inicio < 2000) {
    for (int i = 0; i < 8; i++) {
      int leitura = analogRead(sensores[i]);
      if (leitura > preto[i]) {
        preto[i] = leitura;
      }
    }
  }

  digitalWrite(LED, LOW);
}


// ============================================================
// 22. CÁLCULO DA POSIÇÃO E DO ERRO
// ============================================================

bool calcularErro() {

  float somaPesos = 0.0;
  float somaValores = 0.0;
  int sensoresPretos = 0;

  for (int i = 0; i < 8; i++) {

    int leitura = analogRead(sensores[i]);

    // Normalização
    float diferenca = preto[i] - branco[i];

    if (diferenca < 1.0) {
      diferenca = 1.0;
    }

    float normalizado = (leitura - branco[i]) / diferenca;

    normalizado = constrain(normalizado, 0.0, 1.0);

    valoresNormalizados[i] = normalizado;

    // Considera sensor preto
    if (normalizado >= 0.70) {
      sensoresPretos++;
    }

    // Soma ponderada
    somaPesos += normalizado * pesos[i];
    somaValores += normalizado;
  }

  // Verifica todos os sensores pretos
  todosPretos = (sensoresPretos == 8);

  // Nenhum sensor encontrou a linha
  if (somaValores < 0.05) {
    todosPretos = false;
    return false;
  }

  // Posição da linha
  posicaoLinha = somaPesos / somaValores;

  // Erro normalizado
  erro = posicaoLinha / 3500.0;
  erro = constrain(erro, -1.0, 1.0);

  return true;
}


// ============================================================
// 23. MOVIMENTO DOS MOTORES
// ============================================================
//
// positivo = frente
// negativo = ré
//
// ============================================================

void mover(int esquerdo, int direito) {

  esquerdo = constrain(esquerdo, -255, 255);
  direito  = constrain(direito, -255, 255);

  // Motor esquerdo
  if (esquerdo >= 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, esquerdo);
  }
  else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, -esquerdo);
  }

  // Motor direito
  if (direito >= 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, direito);
  }
  else {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, -direito);
  }
}


// ============================================================
// 24. PARA OS MOTORES
// ============================================================

void parar() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}
