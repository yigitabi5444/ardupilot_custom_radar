#include "AP_RangeFinder_Vband.h"

#if AP_RANGEFINDER_VBAND_ENABLED

#define CONFIDENCE_MIN 1000
#define CONFIDENCE_MAX 1000000000000
#define DISTANCE_MIN 0
#define DISTANCE_MAX 1000

/*
V-Band Radar data output sample

Everyting is sent as an ascii string,
separated by commas, terminated by a newline and a carriage return.
Format:
BIN, I, Q ,J, K

Distance = BIN * 25cm
Confidence (like intensity) = sqrt(I^2+ Q^2 + J^2 + K^2)

322,-8426,-19048,1596,-2416
239,-4786,-993,-3576,-2408
130,-1100,13,-2903,-2396
411,-1182,-154,-2498,-2321
142,-674,-533,1602,2317
413,1617,59,258,-439
*/

// get_reading
bool AP_RangeFinder_VBand::get_reading(float &reading_m)
{
    bool got_reading = false;
    uint8_t nbytes = uart->available();

    while (nbytes-- > 0)
    {
        int16_t b = uart->read();

        // start of the string
        if (b == 10)
        {
            _stringBuffer_index = 0;
            comma_count = 0;
            BIN = 0;
            I = 0;
            Q = 0;
            J = 0;
            K = 0;
        }

        // read the string until the comma
        if (b != 44 || b != 13 || b != 10)
        {
            _stringBuffer[_stringBuffer_index] = b;
            _stringBuffer_index++;
            if (_stringBuffer_index > 15)
                _stringBuffer_index = 0; // prevent buffer overflow
        }

        if (b == 44)
        {
            _stringBuffer[_stringBuffer_index] = '\0'; // terminate the string
            switch (comma_count)
            {
            case 0:
                BIN = atoi(_stringBuffer);
                break;
            case 1:
                I = atoi(_stringBuffer);
                break;
            case 2:
                Q = atoi(_stringBuffer);
                break;
            case 3:
                J = atoi(_stringBuffer);
                break;
            }
            _stringBuffer_index = 0;
            comma_count++;
        }

        // end of the string
        if (b == 13)
        {
            _stringBuffer[_stringBuffer_index] = '\0'; // terminate the string
            K = atoi(_stringBuffer);

            float v_confidence = (I * I) + (Q * Q) + (J * J) + (K * K);
            float v_distance = BIN * 0.25f;

            if (v_confidence > CONFIDENCE_MIN && v_confidence < CONFIDENCE_MAX && v_distance > DISTANCE_MIN && v_distance < DISTANCE_MAX)
            {
                reading_m = v_distance;
                got_reading = true;
            }
        }
    }

    return got_reading;
}

#endif // AP_RANGEFINDER_VBAND_ENABLED