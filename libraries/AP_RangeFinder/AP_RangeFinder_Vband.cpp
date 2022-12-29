#include "AP_RangeFinder_Vband.h"

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

void AP_RangeFinder_VBand::_requestData(void)
{
    uart->write(0x59);
}

// get_reading
bool AP_RangeFinder_VBand::get_reading(float &reading_m)
{
    bool got_reading = false;
    uint8_t nbytes = uart->available();

    if (nbytes == 0)
    {
        _requestData();
    }
    else
        while (nbytes-- > 0)
        {
            char b = uart->read();

            // start of the data
            if (b == 10)
            {
                _stringBuffer_index = 0;
                comma_count = 0;
                BIN = 8;
                I = 0;
                Q = 0;
                J = 0;
                K = 0;
                memset(_confidence, 0, sizeof(_confidence));
                memset(_stringBuffer, 0, sizeof(_stringBuffer));
            }

            // read the string
            if (b != 44 && b != 13 && b != 10)
            {
                _stringBuffer[_stringBuffer_index] = b;
                _stringBuffer_index++;
                if (_stringBuffer_index > 14)
                    _stringBuffer_index = 0; // prevent buffer overflow
            }

            if (b == 44) // comma
            {
                _stringBuffer[_stringBuffer_index] = '\0'; // terminate the string
                switch (comma_count % 4)
                {
                case 0:
                    I = atoi(_stringBuffer);
                    break;
                case 1:
                    J = atoi(_stringBuffer);
                    break;
                case 2:
                    K = atoi(_stringBuffer);
                    break;
                case 3:
                    Q = atoi(_stringBuffer);
                    if (BIN < 128)
                    {
                        _confidence[BIN] = sqrtF((I * I) + (Q * Q) + (J * J) + (K * K));
                        BIN++;
                    }
                    break;
                }
                _stringBuffer_index = 0;
                comma_count++;
            }

            // end of the string
            if (b == 13)
            {
                // find the maximum confidence
                uint32_t max_confidence = 0;
                for (uint8_t i = 8; i < 128; i++)
                {
                    if (_confidence[i] > max_confidence)
                    {
                        max_confidence = _confidence[i];
                        BIN = i;
                    }
                }

                float v_distance = BIN * 0.25f;

                reading_m = v_distance;
                got_reading = true;
            }
        }

    return got_reading;
}