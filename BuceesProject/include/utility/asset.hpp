/*
ima be so fr this was ripped right out of lemlib
sry cant find my own solution to text file reading hehehehehehehe
(shout out to lemlib though)
*/
#pragma once

#include <stdint.h>
#include <cstddef>

extern "C" {

typedef struct __attribute__((__packed__)) _asset {
        uint8_t* buf;
        size_t size;
} asset;
}

#define ASSET(x)                                                                                                       \
    extern "C" {                                                                                                       \
    extern uint8_t _binary_static_##x##_start[], _binary_static_##x##_size[];                                          \
    static asset x = {_binary_static_##x##_start, (size_t)_binary_static_##x##_size};                                  \
    }