#include "HX711.h"
HX711 scale;
#define calibration_factor 2121130
#define max_peaks 2 // si en isr se detectan 2 peaks la persona tiene taquicardia, una condición que generalmente se da antes del paro cardiorrespiratorio
#define ecgok  1000 // 400 
#define k 0.00001
#define duty_cycle 0.9
#define min_duty 0.5
//-7050.0 //This value is obtained using the SparkFun_HX711_Calibration sketch
//float calibration_factor = 48100; // this calibration factor is adjusted according to my load cell
float units;
// 2. Adjustment settings
const long LOADCELL_OFFSET = 50682624;
const long LOADCELL_DIVIDER = 5895655;

int t = 640; // hardcodeado para microsteps, es el ancho de pulso, 640 es para /32 microstep
int l = 1000; //800, 200*5 son steps para cinco vueltas

// 200 steps para 360 grados
// 360 grados hacen una vuelta, que en el tornillo es 0.6 cm
//
volatile int t1 = 0;
volatile int t2 = 0;

const int heartPin = A1;
const int scale_clk = 18;
const int scale_data = 19;

volatile int heartValue = 0;
//volatile int fasea = 0;
//volatile int faseb = 0;
const int faseA = 3;
const int faseB = 2;
long zero_factor = 0;
volatile int encoder = 0;
volatile int lastencoder = 0;
volatile int aState;
volatile int aLastState;
volatile int steps = 0; //steps reales del motor
volatile int degree = 0; // angulo del motor
float duty_cycle_incr = 0.1; // para hacer la rampa se aumenta el duty cycle
int start_encoder = 0; //se debe guardar el step del encoder para saber cuando se avanza al siguiente
int zero_steps = 0; // da la posicion de comienzo del encoder en steps para contar hasta l y llegar a los 5 cm

int program = 0; // programa de rcp para el motor
volatile int breakbone = 0; // cuenta cuantos ciclos de lectura de encoder se ha mantenido sin movimiento el eje axial (se está rompiendo hueso)
volatile int breakcount = 0; // cuenta cuandos ciclos de encoder se deben pasar para poner uno mas en breakbone
volatile int heart_count = 0; //muestras del electrocardiograma
volatile float heart_measure = 0; //suma de puntos del ECG
volatile float u_heart = 0; // almacena el promedio de mediciones del ECG
volatile int reg_peaks = 0; // almacena la cantidad de peaks que ocurre en el intervalo de heart_count
int save_person = 0; // indica si se puede bypasear al sensor de presión y romper hueso para salvar la vida
int death_warning = 0; // si hay una taquicardia

int start_compress = 0; // indica si se ha encontrado el esternon

volatile int delta = 0; // se usa para hacer control de posicion con el encoder
volatile float cal_factor_sum = 0; // suma de factores de calibracion hx711
volatile int cal_factor_count = 0; // cuenta de factores de calibracion hx711
float cal_factor = 0; // factor de calibracion hx711 final
volatile long reading = 0; // lectura raw de hx711
volatile long readit = 0; // offset de hx711
bool calibrar = true; // por defecto se debe calibrar el sensor de presion
bool measure_offset = false; // luego de calibrar se debe medir el offset del hx711
int weight = 0; // esta variable guarda la fuerza de la compresion en kg
int i = 0; // para contar pulsos en compresiones

void setup() {
  // put your setup code here, to run once:

  pinMode(A1, INPUT); // heart rate monitor
  pinMode(2, INPUT); // encoder A
  pinMode(3, INPUT); // encoder B
  pinMode(scale_clk, INPUT); // CLK HX711
  pinMode(scale_data, INPUT); // Data HX711

  pinMode(faseB, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(faseB), encoder_read, CHANGE);

  pinMode(scale_clk, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(scale_clk), scale_read, CHANGE);

  pinMode(13, OUTPUT); //pin de steps
  pinMode(12, OUTPUT); //pin de dirección
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT); //pin de enable (0 V es enable, 5 V es disable)
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);

  digitalWrite(13, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(11, HIGH);
  digitalWrite(10, HIGH);
  digitalWrite(9, LOW);
  digitalWrite(8, LOW);

  Serial.begin(115200);
  Serial.println(encoder);
  aLastState = digitalRead(faseA);

  scale.begin(scale_data, scale_clk);

  scale.set_scale();
  scale.tare();  //Reset the scale to 0

  zero_factor = scale.read_average(); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor); // el zero factor es el offset de fabrica del sensor hx711
}

void loop() {
  // put your main code here, to run repeatedly:
  // medir empiricamente los parametros

  if ((program > 0) && (start_compress > 0)) { // si se ha encontrado el pecho
    for (int compresiones = 0; compresiones == 30 ; compresiones++) {
      if (program > 0) {
        duty_cycle_incr = min_duty;
        zero_steps = steps;
        delta = abs(steps - zero_steps);
        i = 0;
        while (delta < l) {
          delta = abs(steps - zero_steps);
          i++;
          duty_cycle_incr = min_duty + (l - delta) * k; // control proporcional es dar mas corriente con duty cycle
          if (duty_cycle_incr > duty_cycle) { // cada 1% de vuelta
            duty_cycle_incr = duty_cycle;
          }
          if (duty_cycle_incr < min_duty) {
            duty_cycle_incr = min_duty;
          }
          start_encoder = steps;
          while (abs(steps - start_encoder) < 1) { // asegurar el paso de 1 step
            t1 = (int) t * (duty_cycle_incr);
            digitalWrite(13, HIGH); // duty cycle es (t1/(t1 + t2))
            delayMicroseconds(t1);
            digitalWrite(13, LOW);
            t2 = (int) t * (1 - duty_cycle_incr);
            delayMicroseconds(t2);
          }
        } // forward
        digitalWrite(10, HIGH); // para un movimiento mas limpio se hace disable antes de cambiar direccion
        digitalWrite(12, LOW); // direction change
        digitalWrite(10, LOW); // enable nuevamente para mover el motor

        if (death_warning >= 2) { // si el ecg es peligrosamente bajo se permite romper hueso (consideracion etica dada por medicos de los hospitales)
          save_person = 1;
        } else {
          save_person = 0;
        }

        if ((breakbone >= 5)||(weight > 30)) { // si por más de 50 steps, o 90 grados, no se ha podido mover el motor, o el peso ejercido supera el maximo para un niño
          if (save_person == 0) {
            delta = (int) delta * 2;
          }
          breakbone = 0;
        }

        duty_cycle_incr = min_duty;
        zero_steps = steps;
        delta = abs(steps - zero_steps);
        i = 0;
        while (delta < l) {
          delta = abs(steps - zero_steps);
          i++;
          duty_cycle_incr = min_duty + (l - delta) * k; // control proporcional es dar mas corriente con duty cycle
          if (duty_cycle_incr > duty_cycle) { // cada 1% de vuelta
            duty_cycle_incr = duty_cycle;
          }
          if (duty_cycle_incr < min_duty) {
            duty_cycle_incr = min_duty;
          }
          start_encoder = steps;
          while (abs(steps - start_encoder) < 1) { // asegurar el paso de 1 step
            t1 = (int) t * (duty_cycle_incr);
            digitalWrite(13, HIGH); // duty cycle es (t1/(t1 + t2))
            delayMicroseconds(t1);
            digitalWrite(13, LOW);
            t2 = (int) t * (1 - duty_cycle_incr);
            delayMicroseconds(t2);
          }
        } // backward
        digitalWrite(10, HIGH);
        digitalWrite(12, HIGH);
        digitalWrite(10, LOW);

        if (death_warning >= 2) { // si el ecg es peligrosamente bajo se permite romper hueso (consideracion etica dada por medicos de los hospitales)
          save_person = 1;
        } else {
          save_person = 0;
        }

        if ((breakbone >= 5)||(weight > 30)) { // si por más de 50 steps, o 90 grados, no se ha podido mover el motor, o el peso ejercido supera el maximo para un niño
          if (save_person == 0) {
            delta = (int) delta * 2;
          }
          breakbone = 0;
        }
      }
    }
  } else if (start_compress == 0) { // si no se ha encontrado hay que llegar a el a media velocidad
    digitalWrite(10, LOW);
    digitalWrite(12, LOW);

    for (int i = 0; i < 5 ; i++) {
      digitalWrite(13, HIGH);
      delayMicroseconds(t * 2);
      digitalWrite(13, LOW);
      delayMicroseconds(t);
    }
  }

  // medicion biometrica
  heartValue = analogRead(heartPin); // leer el ecg del sensor de pulso cardiaco
  heart_count++;
  heart_measure = heart_measure + heartValue;
  u_heart = heart_measure / heart_count; // promedio del ecg para detectar peaks
  if (heartValue >= u_heart) {
    reg_peaks++;
  }
  if ((reg_peaks >= max_peaks)||(u_heart < 200)) { // si hay tarquicardia o la persona tiene pulso insuficiente
    death_warning++;
  }

  if (weight >= 3) { //si el sensor de presión detecta una compresión inequivoca
    start_compress = 1;
  } else {
    start_compress = 0;
  }

  if (u_heart >= ecgok) // el sensor biometrico ve si la persona se recupera, y si es asi, para el cpr
  {
    program = 0;
  } else {
    program = 1;
  }

  heart_count = 0;
  heart_measure = 0;
  reg_peaks = 0;
  death_warning = 0;

  delay(1500); // duracion de la ventilacion

}

void scale_read() {
  if (scale.wait_ready_timeout(1000)) {
    reading = scale.get_units(10);
    if (reading < 0)
    {
      reading = 0.00;
    }
    if (measure_offset) {
      readit = reading;
      scale.set_offset(zero_factor);
      measure_offset = false;
      Serial.print("--- Offset complete! ---: ");
      Serial.println(zero_factor);
      Serial.println(readit);
    }
    if (calibrar) {
      readit = scale.read_average();
      cal_factor_sum = cal_factor_sum + readit;
      cal_factor_count++;
      if (cal_factor_count == 10) {
        cal_factor = cal_factor_sum / cal_factor_count;
        float m = (3328341 - cal_factor) / (1.6); //pendiente con 1.6 kg del stepper
        scale.set_scale((long) m);
        //scale.tare();
        Serial.print("--- Calibration complete! ---: ");
        Serial.println(m);
        calibrar = false;
        cal_factor_sum = 0;
        cal_factor_count = 0;
        measure_offset = true;
      }
    }
    // Serial.print("HX711 reading: ");
    // Serial.println(reading);
    weight = reading - readit;
    Serial.print("Weight: ");
    Serial.println(weight);
  } else {
    Serial.println("HX711 not found.");
  }
  Serial.println();
  //  if (scale.wait_ready_timeout(1000)) {
  //    long reading = scale.get_units(10);
  //    Serial.print("Weight: ");
  //    Serial.print(reading, 2); //scale.get_units() returns a float
  //    Serial.print(" lbs"); //You can change this to kg but you'll need to refactor the calibration_factor
  //    Serial.println();
  //  } else {
  //    Serial.println("HX711 not found.");
  //  }
}

void encoder_read() {
  delay(3);
  //fasea = digitalRead(faseA);
  //faseb = digitalRead(faseB);
  //Serial.println(' ------- fase A ----- ');
  //Serial.println(fasea);
  //Serial.println(' ------- fase B ----- ');
  //Serial.println(faseb);

  aState = digitalRead(faseA); // Reads the "current" state of the outputA
  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState != aLastState) {
    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (digitalRead(faseB) != aState) {
      encoder ++;
    } else {
      encoder --;
    }
    //Serial.print("Position: ");
    //Serial.println(encoder);
  }
  aLastState = aState;
  steps = (encoder / 3); // (encoder/600)*200
  degree = 0.6 * encoder; // 360*(encoder/600)

  if ((breakcount >= 10) && (abs(encoder - lastencoder) <= 1)) {
    breakbone++;
  }
  if (breakcount < 9) {
    breakcount++;
  } else {
    lastencoder = encoder;
    breakcount = 0;
  }

  //   if (fasea == HIGH) {   // found a low-to-high on channel A
  //   if (faseb == LOW) {  // check channel B to see which way
  //                                            // encoder is turning
  //     encoder = encoder - 1;         // CCW
  //   }
  //   else {
  //     encoder = encoder + 1;         // CW
  //   }
  // }
  // else                                        // found a high-to-low on channel A
  // {
  //   if (faseb == LOW) {   // check channel B to see which way
  //                                             // encoder is turning
  //     encoder = encoder + 1;          // CW
  //   }
  //   else {
  //     encoder = encoder - 1;          // CCW
  //   }
  //  Serial.println(encoder);
  // }

  //Serial.println(encoder);
}
