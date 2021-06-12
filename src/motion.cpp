#include <inttypes.h>
#include <math.h>

#define ENCODER_TICKS_PER_CM 10L  // TODO Cantidad de ticks de encoder por centímetro recorrido

#define CM_TO_TICKS(cm) (int32_t)(cm * ENCODER_TICKS_PER_CM)
#define CMS_TO_TMS(cms) (int32_t)(cms * ENCODER_TICKS_PER_CM / 1000)  // Macro que convierte velocidades de cm/s a ticks de encoder por milisegundo (recibe double)

#define SEGMENTS 10                         // TODO cantidad de segmentos
const uint16_t segmentDistances[SEGMENTS];  // TODO distancias en ticks (usar macros)
const double segmentSpeeds[SEGMENTS];       // TODO velocidades en ticks por milisegundo (usar macros)
uint8_t actualSegment = 0;

uint32_t remainingDistance = 0;  // distancia de segmento

// velocidades en ticks por milisegundo
int32_t realSpeed;
int32_t currentSpeed = 0;
int32_t targetSpeed = 0;

const double speedKp = 0.0;  // TODO
const double speedKd = 0.0;  // TODO
int16_t oldSpeedError = 0;
int16_t speedCorrection = 0;

const double acceleration = CMS_TO_TMS(50.0);  // TODO aceleracion maxima en ticks por milisegundo cuadrado (usar macros)
const double deceleration = CMS_TO_TMS(50.0);  // TODO desaceleracion maxima en ticks por milisegundo cuadrado (usar macros)

// TODO calibracion
#define SENSORES 9
uint16_t rawReadings[SENSORES];
double calibrationSlope[SENSORES];      // guarda el valor de min/(max-min)
double calibrationIntercept[SENSORES];  // guarda el valor de 1000+1000*slope (slope es lo de arriba)
double calibratedData[SENSORES];

const uint16_t positionSetpoint = 400;
const double positionKp = 0.0;  // TODO
const double positionKd = 0.0;  // TODO

uint16_t position;
int16_t oldPositionError = 0;
int16_t positionCorrection = 0;  // valores negativos es hacia la izquierda

////////////////////////
// CALCULOS VELOCIDAD //
////////////////////////

void updateSpeedStatus(void) {
    static uint32_t leftEncoderOld = 0;
    static uint32_t rightEncoderOld = 0;

    //    static uint32_t leftEncoderCount = 0;
    //    static uint32_t rightEncoderCount = 0;

    uint32_t leftEncoder = 0;   // TODO read left encoder
    uint32_t rightEncoder = 0;  // TODO read right encoder

    // Calcular la velocidad de cada rueda en ticks por milisegundo
    // (es el cambio de los encoders y esto se ejecuta cada un milisegundo)
    uint32_t leftEncoderChange = leftEncoder - leftEncoderOld;
    uint32_t rightEncoderChange = rightEncoder - rightEncoderOld;

    // Asumimos que el promedio de velocidades es la velocidad real del robot
    realSpeed = (leftEncoderChange + rightEncoderChange) / 2;

    leftEncoderOld = leftEncoder;
    rightEncoderOld = rightEncoder;

    /*
    // Conteo general de distancia avanzada por cada rueda
    leftEncoderCount += leftEncoderChange;
    rightEncoderCount += rightEncoderChange;

    // Asumimos que el promedio de las distancias avanzadas es la distancia real avanzada del robot
    uint32_t encoderDistance = (leftEncoderCount + rightEncoderCount) / 2;
    */

    remainingDistance -= realSpeed;
}

// Función para actualizar las velocidades (aplicar aceleración/desaceleración)
void updateCurrentSpeed(void) {
    if (currentSpeed < targetSpeed) {                                // Fase de aceleración
        currentSpeed += acceleration;                                // La velocidad X es el doble ya que después se compara con la suma de los cambios (saltea división para promedio)
        if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;  // Limita la velocidad al target
    } else if (currentSpeed > targetSpeed) {                         // Fase de desaceleración
        currentSpeed -= deceleration;
        if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
    }
}

void calculateSpeedCorrection(void) {
    int16_t error = realSpeed - currentSpeed;  //(2 * realSpeed) - currentSpeed; // No recuerdo por que pero en algun lado lei que habia que duplicarla. // TODO PROBAR.
    speedCorrection = speedKp * error + speedKd * (error - oldSpeedError);
    oldSpeedError = error;
}

// Función que calcula la desaceleración requerida para llegar de la velocidad curSpeed
// a la velocidad endSpeed en cierta distancia. En ticks por milisegundo cuadrado
uint32_t requiredDeceleration(int32_t dist, int16_t curSpeed, int16_t endSpeed) {
    if (curSpeed < 0) curSpeed = -curSpeed;
    if (endSpeed < 0) endSpeed = -endSpeed;
    if (dist <= 0) dist = 1;  // Prevenir la división por 0

    return abs((curSpeed * curSpeed - endSpeed * endSpeed) / dist / 4 / 2);
    // Cosas medio mágicas pero básicamente aplico la fórmula de física básica 2aS=Vt^2-V0^2
    // despejada a a=(Vt^2-V0^2)/2S y agregando una división por 4 (=2^2) porque las velocidades
    // son la suma de la velocidad de cada lado (para evitar promedios). El cuadrado se distribuye.
}

void applySpeedProfile(void) {
    updateSpeedStatus();
    updateCurrentSpeed();
    calculateSpeedCorrection();
}

///////////////////////
// CALCULOS POSICION //
///////////////////////

// Arma la data calibrada de los sensores y la guarda en calibratedData[]
void readCalibratedSensorData(void) {
    // antes de ejecutar: obtener lecturas en orden de izquierda a derecha y guardarlas en rawReadings[]
    for (int i = 0; i < SENSORES; i++)
        calibratedData[i] = calibrationIntercept[i] - rawReadings[i] * calibrationSlope[i];
}

void updateSegmentStatus(void) {
    // antes de ejecutar: ejecutar readCalibratedSensorData()
    static uint8_t state = 0;                                                     // ya vengo viendo blanco
    uint8_t aux = calibratedData[0] > 500 || calibratedData[SENSORES - 1] > 500;  // alguno de los auxiliares esta viendo blanco

    if (!state) actualSegment += aux;  // si no venia viendo blanco, sumo 1 si ahora si
    state = aux;                       // si veo blanco, lo dejo anotado para no volver a sumar
}

void updatePositionStatus(void) {
    uint32_t sum = 0;
    uint32_t weights = 0;

    for (uint8_t i = 1; i < SENSORES - 1; i++) {
        sum += i * 100 * calibratedData[i];
        weights += calibratedData[i];
    }

    position = sum / weights;
}

void calculatePositionCorrection(void) {
    int16_t error = position - positionSetpoint;
    positionCorrection = positionKp * error + positionKd * (error - oldPositionError);
    oldPositionError = error;
}

void applyPositionProfile(void) {
    readCalibratedSensorData();
    updateSegmentStatus();
    updatePositionStatus();
    calculatePositionCorrection();
}

///////////////////////
//     GENERALES     //
///////////////////////

void resetMotionProfile(void) {
    currentSpeed = 0;
    targetSpeed = 0;
    oldSpeedError = 0;
    oldPositionError = 0;
    actualSegment = 0;

    //TODO resetear valores de encoders
}

void run() {  // correr cada 1ms (systick)
}
