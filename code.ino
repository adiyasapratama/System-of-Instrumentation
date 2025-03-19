#include <stdio.h>
#include <math.h>
#define N 10  // Jumlah sampel untuk moving average

float eksponen;
int temperatur;
int dataIndex = 1;
float dataBuffer[N];  // Buffer untuk menyimpan nilai ADC
int bufferIndex = 0;

void setup() {
    Serial.begin(115200);
}

void loop() {
    // Membaca nilai ADC
    int adcValue = analogRead(A0);
    
    // Menyimpan nilai ke dalam buffer
    dataBuffer[bufferIndex] = adcValue;
    bufferIndex = (bufferIndex + 1) % N; // Rotasi indeks

    // Menghitung Moving Average
    float sum = 0;
    for (int i = 0; i < N; i++) {
        sum += dataBuffer[i];
    }
    float movingAverage = sum / N;

    eksponen = exp(0.0019*adcValue);
    temperatur = 25.616*eksponen;

    temperatur = 0.0001*(adcValue**2)+0.0278*adcValue+27.386

    Serial.print(adcValue);     // Data 1 (ADC Value)
    Serial.print("\t");
    Serial.println(movingAverage); // Data 2 (Moving Average)
    Serial.print("\t");
    Serial.println(temperatur);

    delay(1000);
}
