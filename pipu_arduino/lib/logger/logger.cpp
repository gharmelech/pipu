#include "logger.h"
#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <HTTPClient.h>

Logger::Logger()
{}

Logger::~Logger()
{
    SSID.end();
    free(samples);
}

void Logger::initLogger(bool clear)
{
    samples = (int32_t*)ps_malloc((MAX_NUM_OF_SAMPLES + 10)* sizeof(int32_t));
    if (samples == NULL)
        Serial.println("sample buffer allocation failed");
    else
        Serial.println("sample buffer allocation success!");
    sampleCount = 0;
    eventJson.reserve(MAX_NUM_OF_SAMPLES * MAX_NUM_OF_CHARS + 1000);

    SSID.begin("cred", false);
    if (clear)
        SSID.clear();
    if (!SSID.isKey("ssid"))
        set_creds();
    uint8_t fail_count = 0;
    Serial.println("Connecting to WiFi...");
    WiFi.begin(SSID.getString("ssid").c_str(), SSID.getString("pass").c_str());
    while (WiFi.status() != WL_CONNECTED)
    {
        if (fail_count++ > 60)
        {
            Serial.println("Could not connect to WiFi with provided credentials");
            Serial.println("Reset device or clear memory to continue");
        }
        delay(500);
    }
    Serial.printf("Connected to: %s, credentials saved to NVM!\n", SSID.getString("ssid").c_str());
}

void Logger::event_catID(String catID)
{
    this->catID = catID;
}

void Logger::event_catWeight(String weight)
{
    catWeight = weight;
}

void Logger::event_depositWeight(String weight)
{
    depositWeight = weight;
}

void Logger::event_type(String type)
{
    eventType = type;
}

void Logger::event_sample(int32_t sample)
{
    // Serial.printf("Logging sample! sample: %d, current sample count: %d\n", sample, sampleCount);
    // delay(500);
    samples[sampleCount] = sample;
    // Serial.println("logged!");
    // delay(500);
    sampleCount++;
    // Serial.println("counter advanced");
    // delay(500);
    if (sampleCount == MAX_NUM_OF_SAMPLES)
    {
        eventType = "max samples";
        event_send();
    }
}

void Logger::event_send()
{
    if (sampleCount)
    {
        eventJson = "{\"Type\":\"" + eventType + "\"";
        if (catID.length() > 1)
            eventJson.concat(", \"Cat ID\":\"" + catID + "\"");
        if (catWeight.length() > 1)
            eventJson.concat(", \"Cat Weight\":" + catWeight);
        if (depositWeight.length() > 1)
            eventJson.concat(", \"Deposit Weight\":" + depositWeight);
        eventJson.concat(", \"Samples\":[" + String(samples[0]));
        for (int i = 1; i < sampleCount; i++)
            eventJson.concat("," + String(samples[i]));
        eventJson.concat("]}");
        post_event(eventJson);
    }
    sampleCount = 0;
    catID.clear();
    catWeight.clear();
    depositWeight.clear();
    eventType.clear();
    eventJson.clear();
}

void Logger::post_event(String json)
{
    Serial.println("posting event: " + json);
    HTTPClient http;
    http.begin("http://192.168.88.187:5000/log"); // your URL
    http.addHeader("Content-Type", "application/json"); // header

    int httpResponseCode = http.POST(json);
    if (httpResponseCode > 0)
    {
      String response = http.getString();
      Serial.print("Response code: ");
      Serial.println(httpResponseCode);
      Serial.print("Response: ");
      Serial.println(response);
    } 
    else
    {
      Serial.print("Error on POST: ");
      Serial.println(httpResponseCode);
    }

    http.end(); // free resources
}

int32_t Logger::remote_setting(rcommand cmd, int32_t arg)
{
    String cmd_string;
    switch (cmd)
    {
    case start:
        cmd_string = "start";
        break;
    case req_tare:
        cmd_string = "req_tare";
        break;
    case ack_tare:
        cmd_string = "ack_tare";
        break;
    case req_calib:
        cmd_string = "req_calib";
        break;
    case ack_calib:
        cmd_string = "ack_calib";
        break;
    default:
        cmd_string = "null";
        break;
    }
    HTTPClient http;
    http.begin("http://192.168.88.187:5000/cmd"); // your URL
    http.addHeader("Content-Type", "application/json"); // header
    String json = "{\"cmd\":\"" + cmd_string + "\", \"arg\":\"" + String(arg) + "\"}";
    Serial.print("Posting JSON: ");
    Serial.println(json);
    http.POST(json);
    String response = http.getString();
    http.end();

    return response.toInt();
}

void Logger::set_creds()
{
    Serial.println("Please enter SSID to use.");
    while(true)
    {
        if (Serial.available())
        {
            String ssid = Serial.readStringUntil('\n');
            SSID.putString("ssid", ssid);
            Serial.println("Please enter password for ssid \"" + ssid + "\".");
            while(true)
            {
                if (Serial.available())
                {
                    String pass = Serial.readStringUntil('\n');
                    SSID.putString("pass", pass);
                    Serial.println("Credantials saved!");
                    return;
                }
            }
        }
    }
}
