#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <Preferences.h>

#define MAX_NUM_OF_SAMPLES (100 * 180)

enum rcommand
{
    start,
    req_tare,
    ack_tare,
    req_calib,
    ack_calib
};

class Logger
{
    private:
        Preferences SSID;
        String catID, catWeight, depositWeight, eventType, eventJson;
        int32_t *samples;
        uint32_t sampleCount = 0;
        void set_creds();

    public:
        Logger();
        ~Logger();
        void initLogger(bool clear);
        void post_event(String json);
        void event_catID(String catID);          // add a catID to event
        void event_catWeight(String weight);     // add a cat weight to event
        void event_depositWeight(String weight); // add a deposit weight to event
        void event_type(String type);            // add a type to event
        void event_sample(int32_t sample);       // add a sample to event
        void event_send();                       // end event and send POST
        int32_t remote_setting(rcommand cmd, int32_t arg = 0);    // remote commands
};

#endif