#ifndef POLYLINE_H
#define POLYLINE_H

#include <Arduino.h>

class PolylineEncoder
{
public:
    void addPoint(double latitude, double longitude);
    String Result(void) { return(Encoded); };
    void clear(void) { LastLAT = LastLON = 0.0; Encoded = ""; };

private:
    //! Encodes a single value according to the compression algorithm.
    String encode(double value);
    String Encoded;
    double LastLAT, LastLON;
};

#endif // POLYLINE_H
