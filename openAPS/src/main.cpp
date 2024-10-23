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

// WiFi and MQTT configuration
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient); 

// broker, port
const char* broker = "tcp://mqtt-dev.precise.seas.upenn.edu";
const int port = 1883;

// Task initialization
SemaphoreHandle_t dataMutex;


struct InsulinTreatment {
    long time;      // time insulin was administered
    float amount;  // amount of inuslin delivered
    int duration;   // how long insulin will be active

    //Insulin treatment constructor
    InsulinTreatment(long t, float a, int d) : time(t), amount(a), duration(d) {}
};

class OpenAPS {
private:
    float ISF = 5;
    int DIA = 90;
    int target_BG = 100;
    int threshold_BG = 50;
    std::vector<InsulinTreatment> insulin_treatments;
    float prev_BG = -1;
    float prev_basal_rate = 0.0;

public:
    OpenAPS(std::vector<InsulinTreatment> bolus_insulins) 
        : insulin_treatments(bolus_insulins) {}

    void clearInsulinTreatments() { insulin_treatments.clear(); }
    
    void addInsulinTreatment(const InsulinTreatment& treatment) { insulin_treatments.push_back(treatment); }

    std::pair<float, float> insulin_calculations(long t) {
        // TODO: Implement insulin calculations
        // Return pair of total_activity and total_iob
        float total_IOB = 0.0; // IOB is insulin that has been adminisered but not done working
        float total_activity = 0.0;


        for (const auto& insulin_treatment : insulin_treatments) {
            long time_elapsed = t - insulin_treatment.time;

            if (time_elapsed >= 0 && time_elapsed <= insulin_treatment.duration) {
                // calculatae active insulin percentage (IOB) 
                float iob_percentage = 1.0 - ((float)(time_elapsed/insulin_treatment.time));
                float insulin_on_board = insulin_treatment.amount * iob_percentage;

                // Total IOB
                total_IOB += insulin_on_board;


                // calculate total insulin activity
                float activity = insulin_on_board * ISF;
                total_activity += activity;
            }
        }
        return pair<float, float>(total_activity, total_IOB);
    }

    std::pair<float, float> get_BG_forecast(float current_BG, float activity, float IOB) {
        // TODO: Implement blood glucose forecasting
        // Return pair of naive_eventual_BG and eventual_BG
        float naive_eventual_BG = current_BG + (activity / ISF); // activity/ISF glucose level reduced for every unit of insulin

        //  Consider administered insulin that has not yet become active
        float eventual_BG = naive_eventual_BG + (IOB * ISF);

        return pair<float, float>(naive_eventual_BG, eventual_BG);

    }

    float get_basal_rate(long t, float current_BG) {
        // TODO: Implement basal rate calculation
        // Use insulin_calculations and get_BG_forecast
        // Apply control logic based on BG levels
        // Update prev_BG, prev_basal_rate, and add new insulin treatment
        // Return calculated basal_rate

        // Step1: call insulin_calculation to get activity and IOB
        pair<float, float> insulin_data = insulin_calculations(t);
        float activity = insulin_data.first;
        float IOB = insulin_data.second;

        // Step2: call get_BG_forecast
        pair<float, float> bg_forecast = get_BG_forecast(current_BG, activity, IOB);
        float naive_eventual_BG = bg_forecast.first;
        float eventual_BG = bg_forecast.second;

        //Step3: BG control logic
        float basal_rate = prev_basal_rate;

        // if bg is too high
        if (eventual_BG > threshold_BG) {
            basal_rate += 0.5;  // increase rate of insulin to reduce BG
        } else if (eventual_BG < target_BG) {
            basal_rate -= 0.5;  // DEcrease rate of insulin to increase BG
        }

        // Step4: Safety measure, ensure rate is within constraints
        // if (basal_rate < min_basal_rate) {
        //     basal_rate = min_basal_rate;
        // } else if (basal_rate > max_basal_rate) {
        //     basal_rate = max_basal_rate;
        // }

        // Step5: update variables
        prev_BG = current_BG;
        prev_basal_rate = basal_rate;

        // Step6:Add new insulin treatment
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
    // TODO: Implement MQTT message callback
    // Handle attribute updates and CGM data
    // Update openAPS, current_BG, current_time, and flags as needed

    // Get MQTT topic
    String topic = mqttClient.messageTopic();

    // read incoming messages while there is something available from topic
    String payload = "";
    while(mqttClient.available()) {
       char c = (char)mqttClient.read();
       payload += c; 
    }

    // check that topic contains attrubute and access token 
    if (topic.indexOf("tb-proxy/h59wc1djkhxy8z4v342m/attribute") >= 0) {
            DynamicJsonDocument doc(1024);
            deserializeJson(doc, payload);

        // check that payload contains "Patient Profile" and that attributeReceived is false
        if (payload.indexOf("Patient Profile") >= 0 && !attributeReceived) {

            // parse bolus insulin data
            JsonArray bolus_array = doc["bolus_insulins"].as<JsonArray>();

            // parsing bolus insulin data
            for (JsonObject bolus_item : bolus_array) {
                long time = bolus_item["time"];
                float dose = bolus_item["dose"];
                int duration = bolus_item["duration"];

                // Create InsulinTreatment object on this
                InsulinTreatment new_treatment(time, dose, duration);
                openAPS->addInsulinTreatment(new_treatment);
            }

            // Update flags (to prevent reprocessing already seen data)
            attributeReceived = true;
            newInsulinTreatment = true;

            // Unsubscribe from the attribute topic to avoid redundant updates
            mqttClient.unsubscribe("tb-proxy/h59wc1djkhxy8z4v342m/attribute");


        }
    }

    // CGM blood glucose updates
    if (topic.indexOf("/cis541-2024/75837946/cgm-openaps") >= 0) {
        DynamicJsonDocument doc(512);
        deserializeJson(doc, payload);

        // parse glucose and timestamp from the payload
        float glucose = doc["Glucose"];
        long timestamp = doc["time"];

        // Update current blood glucose and time in OpenAPS system
        current_BG = glucose;
        current_time = timestamp;

        // Update flag to indicate new BG data is available for processing
        newBGData = true;

    }


}

void TaskMQTT(void *pvParameters) {
    // TODO: Implement MQTT task
    // Continuously poll for MQTT messages

    // keep looping for MQTT poll
    for(;;) {
        mqttClient.poll();

        // delay to manage task switching
        vTaskDelay(10 / portTICK_PERIOD_MS);


    }
}

void TaskOpenAPS(void *pvParameters) {
    // TODO: Implement OpenAPS task
    // Process new data, calculate basal rate, and publish to MQTT
    String topic = "cis541-2024/75837946/insulin-pump-openaps";

    // keep looping to get openAPS data
    for (;;) {
        // if there is new BG data pull
        if (newBGData) {
            // Lock the mutex to ensure data synchronization
            xSemaphoreTake(dataMutex, portMAX_DELAY);

            pair<float, float> insulinData = openAPS->insulin_calculations(current_time);

            // get basal rate
            float basal_rate = openAPS->get_basal_rate(current_time, current_BG);

            // Unlock mutex once data is  processed
            xSemaphoreGive(dataMutex);

            // // Format and publish the insulin rate to the insulin pump topic
            // DynamicJsonDocument doc(128);
            // doc["insulin_rate"] = basal_rate;
            // String output;
            // serializeJson(doc, output);
            // Construct the JSON payload
            String output = "{\"insulin_rate\": " + String(basal_rate) + "}";
            if (mqttClient.beginMessage(topic)) {
                // Print the message content (JSON payload)
                mqttClient.print(output);

                // End the message (publish it)
                mqttClient.endMessage();

                Serial.println("Message published to MQTT!");
            } else {
                Serial.println("Failed to begin MQTT message.");
            } 

            // update the newBGData flag 
            newBGData = false;

        }

        //  add delay to manage task swithcing
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// // Connect to WiFi
// void setup_wifi() {
//     delay(10);
//     Serial.println();
//     Serial.print("Connecting to ");
//     Serial.println(ssid);

//     WiFi.begin(ssid, password);

//     while (WiFi.status() != WL_CONNECTED) {
//         delay(1000);
//         Serial.print(".");
//     }
//     Serial.println("");
//     Serial.println("WiFi connected");
//     Serial.println("IP address: ");
//     Serial.println(WiFi.localIP());
// }

void setup() {
    // TODO: Implement setup function
    // Initialize Serial, WiFi, MQTT, OpenAPS, mutex, and tasks
    // Subscribe to necessary MQTT topics
    // Request virtual patient profile

    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Connect to WiFi
    Serial.print("Connecting to Wi-Fi: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi!");




    // Initialize MQTT client and connect to the broker
    Serial.print("Connecting to MQTT broker: ");
    Serial.println(broker);

    while (!mqttClient.connect(broker, port)) {
        Serial.println("Connecting to MQTT...");
        delay(1000);
    }

    Serial.println("Connected to MQTT broker!");
    
    // Subscribe to the necessary topics
    mqttClient.subscribe("cis541-2024/75837946/cgm-openaps");
    mqttClient.subscribe("tb-proxy/h59wc1djkhxy8z4v342m/attribute");
    // mqttClient.begin("mqtt-dev.precise.seas.upenn.edu", wifiClient);
    // while (!mqttClient.connect(ssid, username, password)) {
    //     Serial.println("Connecting to MQTT broker...");
    //     delay(1000);
    // }
    // Serial.println("Connected to MQTT broker!");

    // // Subscribe to necessary openAPS topics
    // mqttClient.subscribe("cis541-2024/75837946/cgm-openaps");  // 
    // mqttClient.subscribe("tb-proxy/h59wc1djkhxy8z4v342m/attribute");

    // Create the mutex for data synchronization
    dataMutex = xSemaphoreCreateMutex();

    // Set up FreeRTOS tasks
    xTaskCreate(TaskMQTT, "TaskMQTT", 2048, NULL, 1, NULL);  // Create MQTT polling task
    xTaskCreate(TaskOpenAPS, "TaskOpenAPS", 2048, NULL, 1, NULL);   // OpenAPS processing tasK
}

void loop() {
    // Empty. Tasks are handled by FreeRTOS
}