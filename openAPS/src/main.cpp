#include <Arduino.h>
#include <vector>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "arduino_secrets.h"
#include <ArduinoJson.h>

using namespace std;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "mqtt-dev.precise.seas.upenn.edu";
int port = 1883;

// Task initialization
TaskHandle_t taskHandle1 = NULL;
TaskHandle_t taskHandle2 = NULL;

struct InsulinTreatment {
    long time;      // time insulin was administered
    float amount;  // amount of inuslin delivered
    int duration;   // how long insulin will be active

    //Insulin treatment constructor
    InsulinTreatment(long t, float a, int d) : time(t), amount(a), duration(d) {}
};

class OpenAPS {
public:
    float ISF = 5;
    int DIA = 90;
    int target_BG = 100;
    int threshold_BG = 50;
    std::vector<InsulinTreatment> insulin_treatments;
    float prev_BG = -1;
    float prev_basal_rate = 0.0;

    OpenAPS(std::vector<InsulinTreatment> bolus_insulins) 
        : insulin_treatments(bolus_insulins) {}

    void clearInsulinTreatments() { insulin_treatments.clear(); }
    
    void addInsulinTreatment(const InsulinTreatment& treatment) { insulin_treatments.push_back(treatment); }

    std::pair<float, float> insulin_calculations(long t) {
        float total_activity = 0.0;
        float total_IOB = 0.0;

        for (const auto& treatment : insulin_treatments) {
            long elapsed_time = t - treatment.time;
            
            if (elapsed_time < treatment.duration) {
                float DIA = treatment.duration;
                float peak_time = DIA * 75 / 180.0;

                float activity = 0.0;
                if (elapsed_time < peak_time) {
                    activity = (treatment.amount * elapsed_time) / peak_time;
                } else {
                    float time_left = DIA - elapsed_time;
                    activity = (treatment.amount * time_left) / (DIA - peak_time);
                }

                float IOB = (treatment.amount * (DIA - elapsed_time)) / DIA;

                total_activity += activity;
                total_IOB += IOB;
            }
        }

        return std::make_pair(total_activity, total_IOB);
    }

    std::pair<float, float> get_BG_forecast(float current_BG, float activity, float IOB) {
        float naive_eventual_BG = current_BG - (IOB * ISF);
        float predBGI = (activity * ISF * 5);
        float delta_BG = current_BG - prev_BG;
        float deviation = (30.0 / 5.0) * (delta_BG - predBGI);
        float eventual_BG = naive_eventual_BG + deviation;
        return std::make_pair(naive_eventual_BG, eventual_BG);
    }

    float get_basal_rate(long t, float current_BG) {
        auto [activity, IOB] = insulin_calculations(t);

        auto [naive_BG, eventual_BG] = get_BG_forecast(current_BG, activity, IOB);

        float basal_rate = 0.0;
        if (eventual_BG < threshold_BG) {
            basal_rate = 0.0;
        } else if (eventual_BG < target_BG) {
            if (naive_BG < 40) {
                basal_rate = 0.0;
            } else {
                float insulinReq = (eventual_BG - target_BG) / ISF;
                basal_rate = prev_basal_rate + (insulinReq / DIA);
            }
        } else if (eventual_BG > target_BG) {
            float insulinReq = (eventual_BG - target_BG) / ISF;
            basal_rate = prev_basal_rate + (insulinReq / DIA);
        }
        if (basal_rate < 0) basal_rate = 0.0;

        prev_basal_rate = basal_rate;
        InsulinTreatment new_treatment(t, basal_rate, DIA);
        addInsulinTreatment(new_treatment);

        return basal_rate;
    }
};

// Global variables
OpenAPS* openAPS;
volatile float current_BG = 0.0;
volatile long current_time = 0;
volatile bool newBGData = false;
volatile bool newInsulinTreatment = false;
volatile bool attributeReceived = false;

void onMqttMessage(int messageSize) {
    String topic = mqttClient.messageTopic();

    char message[messageSize + 1];
    mqttClient.readBytes(message, messageSize);
    message[messageSize] = '\0';

    if (topic.indexOf("cis541-2024/socoobo/cgm-openaps") >= 0) {
        const char* glucoseStr = strstr(message, "\"Glucose\":");
        const char* timeStr = strstr(message, "\"time\":");
        if (glucoseStr && timeStr) {
            float glucose = atof(glucoseStr + 10);
            long timestamp = atol(timeStr + 7);
            openAPS->prev_BG = current_BG;
            current_BG = glucose;
            current_time = timestamp;
            newBGData = true;
        }
    } else if (topic.indexOf("tb-proxy/h59wc1djkhxy8z4v342m/attributes") >= 0) {
        const char* bolusStr = strstr(message, "\"bolus_insulins\":");
        if (bolusStr) {
            const char* treatmentStr = strstr(bolusStr, "{");
            while (treatmentStr) {
                const char* startTimeStr = strstr(treatmentStr, "\"start_time\":");
                const char* doseStr = strstr(treatmentStr, "\"dose\":");
                const char* durationStr = strstr(treatmentStr, "\"duration\":");
                if (startTimeStr && doseStr && durationStr) {
                    long start_time = atol(startTimeStr + 13);
                    float dose = atof(doseStr + 7);
                    int duration = atoi(durationStr + 11);
                    openAPS->addInsulinTreatment(InsulinTreatment(start_time, dose, duration));
                }
                treatmentStr = strstr(treatmentStr + 1, "{");
            }
            newInsulinTreatment = true;
        }
    }
}

void TaskMQTT(void *pvParameters) {
    while (true) {
        mqttClient.poll();
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void TaskOpenAPS(void *pvParameters) {
    while (true) {
        if (newBGData) {
            float basal_rate = openAPS->get_basal_rate(current_time, current_BG);

            String output = "{\"insulin_rate\": " + String(basal_rate) + "}";
            if (mqttClient.beginMessage("cis541-2024/socoobo/insulin-pump-openaps")) {
                mqttClient.print(output);
                mqttClient.endMessage();
                Serial.println("Message published to MQTT: " + output);
            } else {
                Serial.println("Failed to begin MQTT message.");
            } 

            // update the newBGData flag 
            newBGData = false;
        }
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.print("Connecting to Wi-Fi: ");
    Serial.println(ssid);
    while (WiFi.begin(ssid, password) != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi.");

    Serial.print("Connecting to MQTT broker: ");
    Serial.println(broker);
    mqttClient.setUsernamePassword("cis541-2024", "cukwy2-geNwit-puqced");
    while (!mqttClient.connect(broker, port)) {
        Serial.println("Connecting to MQTT...");
        delay(1000);
    }
    Serial.println("Connected to MQTT broker.");

    mqttClient.onMessage(onMqttMessage);
    
    mqttClient.subscribe("cis541-2024/socoobo/cgm-openaps", 1);
    mqttClient.subscribe("tb-proxy/h59wc1djkhxy8z4v342m/attributes", 1);
    mqttClient.subscribe("tb-proxy/h59wc1djkhxy8z4v342m/attributes/response/+", 1);

    Serial.println("Subscribed to topics.");

    openAPS = new OpenAPS({});

    xTaskCreate(TaskMQTT, "TaskMQTT", 1024, NULL, 1, &taskHandle1);
    xTaskCreate(TaskOpenAPS, "TaskOpenAPS", 1024, NULL, 1, &taskHandle2);

    vTaskStartScheduler();

    Serial.println("Failed to start scheduler.");
}

void loop() {
    // Empty. Tasks are handled by FreeRTOS
}