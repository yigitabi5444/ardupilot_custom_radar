#include "AP_RangeFinder_Vband.h"

void AP_RangeFinder_VBand::_requestData(void)
{
    uart->write('a');
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
        while (nbytes-- > 0) // read and process bytes from uart one by one while data is available
        {
            char b = uart->read(); // Read one byte from the serial port

            // read the string if it is not a comma or a newline
            if (b != ',' && b != '\n' && b != '\r')
            {
                _stringBuffer[_stringBuffer_index] = b;
                _stringBuffer_index++;
                if (_stringBuffer_index > 14)
                    _stringBuffer_index = 0; // prevent buffer overflow
            }

            // if it is a comma, then we have a new value
            if (b == ',')
            {
                _stringBuffer[_stringBuffer_index] = '\0'; // terminate the number string
                switch (comma_count % 4)
                {
                case 0:
                    // if the recieved bin value is smaller than the last one, then we have a new set of data
                    if (atoi(_stringBuffer) < BIN)
                    {
                        // We set the flag to true, so the distance gets calculated from the last compelete set
                        _set_done = true;
                    }
                    BIN = atoi(_stringBuffer);
                    break;
                case 1:
                    I = atoi(_stringBuffer);
                    break;
                case 2:
                    J = atoi(_stringBuffer);
                    break;
                case 3:
                    K = atoi(_stringBuffer);
                    break;
                case 4:
                    Q = atoi(_stringBuffer);
                    break;
                }
                _stringBuffer_index = 0;
                comma_count++;
            }

            if (b == '\n') // end of the line
            {
                if (BIN < 128) // Check if BIN is in range
                {
                    _confidence[BIN] = sqrtF((I * I) + (Q * Q) + (J * J) + (K * K));
                }

                // reset the data buffers
                _stringBuffer_index = 0;
                comma_count = 0;
                I = 0;
                Q = 0;
                J = 0;
                K = 0;
                memset(_stringBuffer, 0, sizeof(_stringBuffer));
            }

            if (_set_done) // if a complete set of data has been recieved:
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

                float v_distance = BIN * 0.25f; // distance in meters

                reading_m = v_distance;
                got_reading = true;

                _set_done = false;                           // unset the flag
                memset(_confidence, 0, sizeof(_confidence)); // reset the confidence array
            }
        }

    return got_reading;
}