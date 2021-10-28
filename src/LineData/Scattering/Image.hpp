#pragma once
#include <stdint.h>

typedef unsigned char byte;

union Pixel {
    struct {
        // NOTE(Felix): This is the correct memory order, this is not a mistake
        byte b;
        byte g;
        byte r;
        byte a;
    };
    byte     bytes[4];
    int32_t  s32;
};

struct Image {
    uint32_t width;
    uint32_t height;
    Pixel*   pixels;

    void allocate();
    void free();
    void save_as_bmp(const char* file_name);
};
