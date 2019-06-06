#include "polylineencoder.h"

static const double s_presision   = 100000.0;
static const int    s_chunkSize   = 5;
static const int    s_asciiOffset = 63;
static const int    s_5bitMask    = 0x1f; // 0b11111 = 31
static const int    s_6bitMask    = 0x20; // 0b100000 = 32

void PolylineEncoder::addPoint(double latitude, double longitude)
{
  Encoded += encode(latitude - LastLAT);
  Encoded += encode(longitude - LastLON);
  
  LastLAT = latitude;
  LastLON = longitude;
}

String PolylineEncoder::encode(double value)
{
    int32_t e5 = round(value * s_presision); // (2)
    e5 <<= 1;                                // (4)
    if (value < 0) e5 = ~e5;                 // (5)

    bool hasNextChunk = false;
    String result = "";

    // Split the value into 5-bit chunks and convert each of them to integer
    do 
    {
        int32_t nextChunk = (e5 >> s_chunkSize);   // (6), (7) - start from the left 5 bits.
        hasNextChunk = nextChunk > 0;
        int charVar = e5 & s_5bitMask;             // 5-bit mask (0b11111 == 31). Extract the left 5 bits.
        if (hasNextChunk) charVar |= s_6bitMask;   // (8)
        charVar += s_asciiOffset;                  // (10)
        result += (char)charVar;                   // (11)
        e5 = nextChunk;
    } while (hasNextChunk);

    return(result);
}

