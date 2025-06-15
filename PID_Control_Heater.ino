#include <stdio.h>
#include <math.h>
#define N 10  // Jumlah sampel untuk moving average
#include <LiquidCrystal.h>

float eksponen;
float temperatur;
int dataIndex = 1;
float dataBuffer[N];  // Buffer untuk menyimpan nilai ADC
int bufferIndex = 0;
const int buttonPin_increment = 7;
const int buttonPin_column = 8;
const int HEATER_PIN = 13;
int counter = 0;
const int rs = 12, en = 10, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
int setpoint = 0;
int digits[2] = {0, 0};
int currentColumn = 0;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
float histeresis = 0.5;
static bool lastIncrementState = HIGH;  // Pengatur nilai kenaikan
static bool lastColumnState = HIGH;     // Pengatur kolom yang akan diatur
bool incrementState = digitalRead(buttonPin_increment);
bool columnState = digitalRead(buttonPin_column);
int adcValue;
float movingAverage;
String status;
double Kp = 40.0;	   // dapat disesuaikan nilainya
double Ki = 0.0;   // dapat disesuaikan nilainya
double Kd = 27.5;   // dapat disesuaikan nilainya
double error, prevError = 0;
double integral = 0;
double output;
double dt = 1.0; // sampling rate (dalam detik)
unsigned long cycleTime = 1000;  // Durasi 1 siklus TPO dalam milidetik
unsigned long windowStartTime = 0;

void readTemp() {
  // Membaca nilai ADC
  adcValue = analogRead(A0);
   
  // Menyimpan nilai ke dalam buffer
  dataBuffer[bufferIndex] = adcValue;
  bufferIndex = (bufferIndex + 1) % N; // Rotasi indeks

  // Menghitung Moving Average
  float sum = 0;
  for (int i = 0; i < N; i++) {
      sum += dataBuffer[i];
  }  
  movingAverage = sum / N;

  // Menghitung nilai temperatur
  eksponen = exp(0.0019*movingAverage);
  temperatur = 25.616*eksponen;

  //temperatur = 0.0001*(adcValue*adcValue)+0.0278*adcValue+27.386;
}

void buttonLogic() {
  // Increment Button Logic
  if (lastIncrementState == HIGH && incrementState == LOW && millis() - lastDebounceTime > debounceDelay) {
      digits[currentColumn]++;
      if (digits[currentColumn] > 9) digits[currentColumn] = 0;
      displaySetpoint();
      lastDebounceTime = millis();
  }

  // Column Button Logic
  if (lastColumnState == HIGH && columnState == LOW && millis() - lastDebounceTime > debounceDelay) {
      currentColumn--;
      if (currentColumn < 0) currentColumn = 1;
      displaySetpoint();
      lastDebounceTime = millis();
  }
  lastIncrementState = incrementState;
  lastColumnState = columnState;
}

void displaySetpoint() {
  setpoint = digits[0] * 10 + digits[1];
  lcd.setCursor(0, 1);
  lcd.print("Set : ");

  // Highlight active digit
  for (int i = 0; i < 2; i++) {
    if (i == currentColumn) {
      lcd.print(digits[i]);
    } else {
      lcd.print(digits[i]);
    }
  }
}

void onoff_logic() {
  if (temperatur < setpoint-histeresis) {
    digitalWrite(HEATER_PIN, HIGH);
    status = "On";
  } else if (temperatur > setpoint+histeresis) {
    digitalWrite(HEATER_PIN, LOW);
    status = "Off";
  }

  lcd.setCursor(9,1);
  lcd.print(status);
  lcd.print("   ");
}

void PID_control() {
  error = setpoint - temperatur;

  integral += error*dt;
  integral = constrain(integral, -50, 50);

  double derivative = (error - prevError) / dt;

  output = Kp*error + Ki*integral + Kd*derivative;
  output = constrain(output, 0, 100);
}

void TPO() {
  unsigned long now = millis();
  if (now - windowStartTime > cycleTime) {
    windowStartTime += cycleTime;
  }

  double onTime = (output/100.0) * cycleTime;

  if ((now - windowStartTime) < onTime) {
    digitalWrite(HEATER_PIN, LOW);
  } else {
    digitalWrite(HEATER_PIN, HIGH);
  }
  prevError = error;
}

void setup() {
  pinMode(buttonPin_increment, INPUT);
  pinMode(buttonPin_column, INPUT);
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);  // Matikan heater pada awal nyala- Heater Active HIGH
  windowStartTime = millis();
  Serial.begin(9600);
  lcd.begin(16, 2);
  delay(200);
  lcd.clear();
}

void loop() {
  // Panggil fungsi pembacaan temperatur
  readTemp();

  // Print ke serial monitor untuk pengawasan
  Serial.print(setpoint);     // Data 1 (ADC Value)
  Serial.print(",");
  Serial.print(adcValue); // Data 2 (Moving Average)
  Serial.print(",");
  Serial.println(temperatur);

  // Print ke LCD  
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print("Temp:           ");
  lcd.setCursor(6, 0);
  lcd.print(temperatur, 3);
  // lcd.setCursor(uint8_t, uint8_t)

  incrementState = digitalRead(buttonPin_increment);
  columnState = digitalRead(buttonPin_column);

  // Panggil fungsi logika button untuk atur setpoint
  buttonLogic();

  // Panggil fungsi on-off
  // onoff_logic();

  // Panggil fungsi PID dan kontrol output
  PID_control();
  TPO();

  // Delay total sistem
  delay(200);
}
