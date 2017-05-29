#ifndef SERIALIZE_H
#define SERIALIZE_H

#include <stdint.h>
#include <cstring>

/* Serializes numeric data into a char array (big endian)
 */
template <typename T>
inline void serialize(char *arr, T data) {
  int max = sizeof(T) - 1;
  for (int i = 0; i <= max; i++) {
    arr[max - i] = (data >> (i * 8)) & 0xFF;
  }
}

/* Deseralizes numeric data from a char array to its original value (big endian)
 */
template <typename T>
inline T deserialize(const char *arr) {
  T result = 0;
  for (unsigned int i = 0; i < sizeof(T); i++) {
    result = (result << 8) + reinterpret_cast<const unsigned char&>(arr[i]);
  }
  return result;
}

/* Serializes a float or double as an IEEE754 64-bit representation
 */
void serializeF(char *arr, double f)
{
    long double fnorm;
    int shift;
    long long sign, exp, significand;
    unsigned significandbits = 52; // -1 for sign bit

    if (f == 0.0)
    {
        serialize<uint64_t>(arr, 0);
    }
    else
    {
        // check sign and begin normalization
        if (f < 0) { sign = 1; fnorm = -f; }
        else { sign = 0; fnorm = f; }

        // get the normalized form of f and track the exponent
        shift = 0;
        while(fnorm >= 2.0) { fnorm /= 2.0; shift++; }
        while(fnorm < 1.0) { fnorm *= 2.0; shift--; }
        fnorm = fnorm - 1.0;

        // calculate the binary form (non-float) of the significand data
        significand = fnorm * ((1LL<<significandbits) + 0.5f);

        // get the biased exponent
        exp = shift + ((1<<(10)) - 1); // shift + bias

        // serialize the final answer
        uint64_t result = (sign<<63) | (exp<<52) | significand;
        serialize<uint64_t>(arr, result);
    }
}

/* Deserializes a float or double from a 64-bit IEEE754 representation back
 * into its original value
 */
double deserializeF(const char *arr)
{
    uint64_t i = deserialize<uint64_t>(arr);
    long double result;
    long long shift;
    unsigned bias;
    unsigned significandbits = 52; // -1 for sign bit

    if (i == 0) return 0.0;

    // pull the significand
    result = (i&((1LL<<significandbits)-1)); // mask
    result /= (1LL<<significandbits); // convert back to float
    result += 1.0f; // add the one back on

    // deal with the exponent
    bias = (1<<(10)) - 1;
    shift = ((i>>significandbits)&((1LL<<11)-1)) - bias;
    while(shift > 0) { result *= 2.0; shift--; }
    while(shift < 0) { result /= 2.0; shift++; }

    // sign it
    result *= (i>>(63))&1? -1.0: 1.0;

    return result;
}

#endif
