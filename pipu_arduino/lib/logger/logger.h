#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <Preferences.h>

#define MAX_NUM_OF_SAMPLES 100 * 180 + 1000


class Logger
{
    private:
        Preferences SSID;
        String catID, eventType, eventJson;
        int16_t *samples;
        uint16_t sampleCount = 0;
        void set_creds();

    public:
        Logger();
        ~Logger();
        void initLogger(bool clear);
        void post_event(String json);
        void event_catID(String catID); //  add a catID to event
        void event_type(String type); //  add a type to event
        void event_sample(int16_t sample); // add a sample to event
        void event_send(); // end event and send POST
};

#endif