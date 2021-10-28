#include "Image.hpp"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#pragma pack(push, 1)
struct BMP_Header {
    uint16_t FileType;
    uint32_t FileSize;
    uint16_t Reserved1;
    uint16_t Reserved2;
    uint32_t BitmapOffset;
    uint32_t Size;
    int32_t  Width;
    int32_t  Height;
    uint16_t Planes;
    uint16_t BitsPerPixel;
    uint32_t Compression;
    uint32_t SizeOfBitmap;
    int32_t  HorzResolution;
    int32_t  VertResolution;
    uint32_t ColorsUsed;
    uint32_t ColorsImportant;
};
#pragma pack(pop)

void Image::allocate() {
    this->pixels = (Pixel*)calloc(1, this->width * this->height * sizeof(Pixel));
}


void Image::free() {
    ::free(this->pixels);
    this->pixels = nullptr;
}

void Image::save_as_bmp(const char *file_name) {
    uint32_t pixels_byte_size = this->width * this->height * sizeof(Pixel);

    BMP_Header Header = {};
    Header.FileType = 0x4D42;
    Header.FileSize = sizeof(Header) + pixels_byte_size;
    Header.BitmapOffset = sizeof(Header);
    Header.Size = sizeof(Header) - 14;
    Header.Width = this->width;
    Header.Height = this->height;
    Header.Planes = 1;
    Header.BitsPerPixel = 32;
    Header.Compression = 0;
    Header.SizeOfBitmap = pixels_byte_size;
    Header.HorzResolution = 0;
    Header.VertResolution = 0;
    Header.ColorsUsed = 0;
    Header.ColorsImportant = 0;

    FILE *OutFile = fopen(file_name, "wb");
    if(OutFile) {
        fwrite(&Header, sizeof(Header), 1, OutFile);
        fwrite(this->pixels, pixels_byte_size, 1, OutFile);
        fclose(OutFile);
    } else {
        fprintf(stderr, "[ERROR] Unable to write output file %s.\n", file_name);
    }
}
