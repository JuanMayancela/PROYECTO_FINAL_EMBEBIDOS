#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

BluetoothSerial SerialBT;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // Address 0x3C for 128x64

#define POT_PIN     34  // Pin analógico para el potenciómetro
#define SIGNAL_PIN  35  // Pin analógico para la señal
#define BUTTON_PIN  32  // Pin donde está conectado el botón

#define NUM_POINTS  128

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int signalValues[NUM_POINTS];
#define SINE_WAVE 1
#define SQUARE_WAVE 2
int signalType = SINE_WAVE;

float voltageRange = 3.3; // Voltaje máximo para el ADC

int generateSignal() {
    static float angle = 0;
    switch (signalType) {
        case SINE_WAVE:
            angle += 0.1;
            if (angle >= 2 * PI) angle = 0;
            return (int)(2047 * (sin(angle) + 1)); // +1 to make it 0-4095
        case SQUARE_WAVE:
            angle += 0.1;
            if (angle >= 2 * PI) angle = 0;
            return angle < PI ? 4095 : 0;
        default:
            return 0;
    }
}

void readSensorsTask(void *pvParameters) {
    (void)pvParameters;
    while (true) {
        int valorPotenciometro = analogRead(POT_PIN);
        SerialBT.println(valorPotenciometro);

        int buttonState = digitalRead(BUTTON_PIN);
        if (buttonState == LOW) {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            if (digitalRead(BUTTON_PIN) == LOW) {
                if (signalType == SINE_WAVE) {
                    signalType = SQUARE_WAVE;
                } else {
                    signalType = SINE_WAVE;
                }
                while (digitalRead(BUTTON_PIN) == LOW) {
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void updateDisplayTask(void *pvParameters) {
    (void)pvParameters;
    while (true) {
        int scaleValue = analogRead(POT_PIN);
        float scaleFactor = map(scaleValue, 0, 4095, 1, 10) / 10.0;
        int signalValue = generateSignal();

        for (int i = 0; i < NUM_POINTS - 1; i++) {
            signalValues[i] = signalValues[i + 1];
        }
        signalValues[NUM_POINTS - 1] = signalValue;

        display.clearDisplay();

        // Draw X and Y axes
        display.drawLine(0, SCREEN_HEIGHT - 1, SCREEN_WIDTH, SCREEN_HEIGHT - 1, SSD1306_WHITE); // X-axis
        display.drawLine(0, SCREEN_HEIGHT - 1, 0, 0, SSD1306_WHITE); // Y-axis

        // Draw axis labels with smallest text size
        display.setTextSize(1);

        // X-axis labels
        for (int i = 0; i <= SCREEN_WIDTH; i += 20) {
            display.setCursor(i, SCREEN_HEIGHT - 10);
            int xLabel = map(i, 0, SCREEN_WIDTH, 0, NUM_POINTS / 2);
            if (xLabel % 2 == 0) { // Print labels only for multiples of 2
                display.print(xLabel);
            }
        }

        // Y-axis labels (from bottom to top)
        for (int i = 0; i <= SCREEN_HEIGHT; i += 10) {
            float voltageLabel = map(i, SCREEN_HEIGHT, 0, 0, voltageRange * 1000) / 1000.0; // Convert to voltage
            display.setCursor(2, SCREEN_HEIGHT - i - 2); // Position of the label
            display.print(voltageLabel, 2); // Print with 2 decimal places
        }

        // Draw the signal
        for (int i = 0; i < NUM_POINTS - 1; i++) {
            int y1 = map(signalValues[i] * scaleFactor, 0, 4095, SCREEN_HEIGHT - 1, 0);
            int y2 = map(signalValues[i + 1] * scaleFactor, 0, 4095, SCREEN_HEIGHT - 1, 0);
            display.drawLine(i, y1, i + 1, y2, SSD1306_WHITE);
        }

        display.display();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void setup() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Serial.begin(115200);
    SerialBT.begin("ESP32_BT");

    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;);
    }
    delay(2000);
    display.clearDisplay();

    // Smallest text size
    display.setTextSize(1);
    display.setTextColor(WHITE);

    for (int i = 0; i < NUM_POINTS; i++) {
        signalValues[i] = 0;
    }

    xTaskCreate(readSensorsTask, "ReadSensors", 2048, NULL, 1, NULL);
    xTaskCreate(updateDisplayTask, "UpdateDisplay", 2048, NULL, 1, NULL);
}

void loop() {
    // No need to put code here, as FreeRTOS tasks handle the functionality.
}
