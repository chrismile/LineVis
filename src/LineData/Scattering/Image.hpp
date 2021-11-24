#pragma once
#include <stdint.h>
#include <stdlib.h>

typedef unsigned char byte;

union Pixel {
    struct {
        byte r;
        byte g;
        byte b;
        byte a;
    };
    byte     bytes[4];
    int32_t  s32;
};

struct Image {
    uint32_t width;
    uint32_t height;
    Pixel*   pixels;

    void allocate() {
        this->pixels = (Pixel*)calloc(1, this->width * this->height * sizeof(Pixel));
    }
    void free() {
        ::free(this->pixels);
        this->pixels = nullptr;
    }
};
