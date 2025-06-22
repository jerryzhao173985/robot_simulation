#include "mac_drand48r.h"

// Implementation of drand48_r for macOS compatibility

int srand48_r(long int seedval, struct drand48_data *buffer) {
    buffer->__x[0] = 0x330e;
    buffer->__x[1] = seedval & 0xffff;
    buffer->__x[2] = seedval >> 16;
    buffer->__old_x[0] = 0;
    buffer->__old_x[1] = 0;
    buffer->__old_x[2] = 0;
    buffer->__c = 0xb;
    buffer->__init = 1;
    buffer->__a = 0x5deece66dLL;
    return 0;
}

int drand48_r(struct drand48_data *buffer, double *result) {
    unsigned long long int X;
    unsigned long long int a;
    unsigned short int c;

    // Linear congruential generator
    X = (unsigned long long int)buffer->__x[2] << 32 | 
        (unsigned long long int)buffer->__x[1] << 16 | 
        (unsigned long long int)buffer->__x[0];
    
    a = buffer->__a;
    c = buffer->__c;
    
    X = a * X + c;
    
    buffer->__x[0] = X & 0xffff;
    buffer->__x[1] = (X >> 16) & 0xffff;
    buffer->__x[2] = (X >> 32) & 0xffff;
    
    // Convert to double in [0, 1)
    union ieee754_double temp;
    temp.ieee.negative = 0;
    temp.ieee.exponent = IEEE754_DOUBLE_BIAS;
    temp.ieee.mantissa0 = (buffer->__x[2] << 4) | (buffer->__x[1] >> 12);
    temp.ieee.mantissa1 = ((buffer->__x[1] & 0xfff) << 20) | (buffer->__x[0] << 4);
    
    *result = temp.d - 1.0;
    return 0;
}