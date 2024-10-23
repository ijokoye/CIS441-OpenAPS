#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <mqtt/async_client.h>

using namespace std;
using namespace std::chrono;

const string ADDRESS { "tcp://mqtt-dev.precise.seas.upenn.edu" };
const string USERNAME { "cis541-2024" };
const string PASSWORD { "cukwy2-geNwit-puqced" };

const int QOS = 1;

// communication between Virtual Compenent and Virtual Patient
const string INSULIN_TOPIC { "cis541-2024/socoobo/insulin-pump" };
const string CGM_TOPIC { "cis541-2024/socoobo/cgm" };

// communication between Virtual Compenent and OpenAPS
const string OA_INSULIN_TOPIC { "cis541-2024/socoobo/insulin-pump-openaps" };
const string OA_CGM_TOPIC { "cis541-2024/socoobo/cgm-openaps" };

// Separate callback class inheriting from mqtt::callback
class MessageRelayCallback : public virtual mqtt::callback {
    mqtt::async_client& client_;

public:
    MessageRelayCallback(mqtt::async_client& client)
        : client_(client) {}

    // subscribe mqtt topics
    void connected(const string& cause) override {
        cout << "Connected to the broker. Subscribing to topics..." << endl;
        client_.subscribe(CGM_TOPIC, QOS);
        client_.subscribe(OA_INSULIN_TOPIC, QOS);
    }

    // message handler
    void message_arrived(mqtt::const_message_ptr msg) override {
        string topic = msg->get_topic();
        string payload = msg->to_string();

        if (topic == CGM_TOPIC) {
            on_message_cgm(payload);
        } else if (topic == OA_INSULIN_TOPIC) {
            on_message_insulin(payload);
        }
    }

    // handle cgm message
    void on_message_cgm(const string& payload) {
        cout << "Processing CGM message: " << payload << endl;

        mqtt::message_ptr pubmsg = mqtt::make_message(OA_CGM_TOPIC, payload);
        pubmsg->set_qos(QOS);
        client_.publish(pubmsg)->wait_for_completion();
        
        cout << "CGM data relayed to OpenAPS." << endl;
    }

    // handle insulin message
    void on_message_insulin(const string& payload) {
        cout << "Processing insulin message: " << payload << endl;

        mqtt::message_ptr pubmsg = mqtt::make_message(INSULIN_TOPIC, payload);
        pubmsg->set_qos(QOS);
        client_.publish(pubmsg)->wait_for_completion();
        
        cout << "Insulin data relayed to virtual patient." << endl;
    }

};


// The main MQTTClientHandler class to manage the connection and the callback
class MQTTClientHandler {
    mqtt::async_client client_;
    MessageRelayCallback callback_;

public:
    MQTTClientHandler(const string& host, const string& username, const string& password)
        : client_(host, ""), callback_(client_) {
        mqtt::connect_options connOpts;
        connOpts.set_user_name(username);
        connOpts.set_password(password);

        client_.set_callback(callback_);

        try {
            client_.connect(connOpts)->wait();
            cout << "Connected to the MQTT broker at " << host << endl;
        } catch (const mqtt::exception& e) {
            cerr << "Error connecting to the broker: " << e.what() << endl;
            exit(1);
        }   
    }

    // This keeps the program running indefinitely, which ensures that the MQTT client remains active and ready to receive and send messages.
    void inject_loop() {
        cout << "Press Ctrl+C to exit the mqtt handler." << endl;
        while (true) {
            this_thread::sleep_for(seconds(1));
        }
    }
};

int main() {
    MQTTClientHandler mqtt_handler(ADDRESS, USERNAME, PASSWORD);
    mqtt_handler.inject_loop();

    return 0;
}
