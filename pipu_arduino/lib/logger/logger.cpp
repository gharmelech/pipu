#include "logger.h"
#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <HTTPClient.h>

Logger::Logger()
{
    samples = (int16_t*)ps_malloc(MAX_NUM_OF_SAMPLES * sizeof(int16_t));
    sampleCount = 0;
    eventJson.reserve(180000);
}

Logger::~Logger()
{
    SSID.end();
    free(samples);
}

void Logger::initLogger(bool clear)
{
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

void Logger::event_type(String type)
{
    eventType = type;
}

void Logger::event_sample(int16_t sample)
{
    samples[sampleCount++] = sample;
    if (sampleCount >= 180 * 100)
        event_send();
}

void Logger::event_send()
{
    if (sampleCount)
    {
        eventJson = "{\"Type\":\"" + eventType + "\"";
        if (catID.length() > 1)
            eventJson.concat(", \"Cat ID\":\"" + catID + "\"");
        eventJson.concat(", \"Samples\":[" + String(samples[0]) + "\"");
        for (int i = 1; i < sampleCount; i++)
            eventJson.concat(", \"" + String(samples[i]) + "\"");
        eventJson.concat("]}");
        post_event(eventJson);
    }
    sampleCount = 0;
    catID.clear();
    eventType.clear();
    eventJson.clear();
}

void Logger::post_event(String json)
{
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

void Logger::set_creds()
{
    Serial.println("Please enter SSID to use.");
    while(true)
    {
        if (Serial.available())
        {
            String ssid = Serial.readStringUntil('\n');
            SSID.putString("ssid", ssid);
            Serial.println("Please enter password for ssid" + ssid + ".");
            while(true)
            {
                if (Serial.available())
                {
                    String pass = Serial.readStringUntil('\n');
                    SSID.putString("pass", pass);
                    break;
                }
            }
        }
    }
}
