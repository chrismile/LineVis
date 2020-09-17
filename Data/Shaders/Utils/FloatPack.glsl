// Packs, e.g., a normalized depth value with 24-bit precision and some other normalized attribute with 8-bit precision
// into one uint value. The floating point values are assumed to lie in the range [0,1].
void packFloat24Float8(out uint packedValue, in float val24, in float val8) {
    packedValue = uint(round(clamp(val8, 0.0, 1.0) * 255.0)) & 0xFFu;
    packedValue |= (uint(round(clamp(val24, 0.0, 1.0) * 16777215.0)) & 0xFFFFFFu) << 8u;
}

// Decompression equivalent to function above.
void unpackFloat24Float8(in uint packedValue, out float val24, out float val8) {
    val8 = float(packedValue & 0xFFu) / 255.0;
    val24 = float((packedValue >> 8u)  & 0xFFFFFFu) / 16777215.0;
}

// Decompression only of 8-bit float of function above.
float unpackFloat8(in uint packedValue) {
    return float(packedValue & 0xFFu) / 255.0;
}


// Packs, e.g., a normalized floating point depth value with 24-bit precision and some other unsigned int attribute with
// 8-bit precision into one uint value. The floating point values are assumed to lie in the range [0,1].
void packFloat24Uint8(out uint packedValue, in float val24, in uint val8) {
    packedValue = val8 & 0xFFu;
    packedValue |= (uint(round(clamp(val24, 0.0, 1.0) * 16777215.0)) & 0xFFFFFFu) << 8u;
}

// Decompression equivalent to function above.
void unpackFloat24Uint8(in uint packedValue, out float val24, out uint val8) {
    val8 = packedValue & 0xFFu;
    val24 = float((packedValue >> 8u)  & 0xFFFFFFu) / 16777215.0;
}


// Converts normalized float in range [0,1] to 32-bit uint.
uint convertNormalizedFloatToUint32(float valueFloat) {
    return uint(round(clamp(valueFloat, 0.0, 1.0) * 4294967295.0));
}

// Decompression equivalent to function above.
float convertUint32ToNormalizedFloat(uint valueUint) {
    return float(valueUint) / 4294967295.0;
}


// Packs, e.g., a normalized depth value with 22-bit precision and some other normalized attribute with 10-bit precision
// into one uint value. The floating point values are assumed to lie in the range [0,1].
void packFloat22Float10(out uint packedValue, in float val22, in float val10) {
    packedValue = uint(round(clamp(val10, 0.0, 1.0) * 1023.0)) & 0x3FFu;
    packedValue |= (uint(round(clamp(val22, 0.0, 1.0) * 4194303.0)) & 0x3FFFFFu) << 10u;
}

// Decompression equivalent to function above.
void unpackFloat22Float10(in uint packedValue, out float val22, out float val10) {
    val10 = float(packedValue & 0x3FFu) / 1023.0;
    val22 = float((packedValue >> 10u)  & 0x3FFFFFu) / 4194303.0;
}

// Decompression only of 8-bit float of function above.
float unpackFloat10(in uint packedValue) {
    return float(packedValue & 0x3FFu) / 1023.0;
}


// Converts normalized float in range [0,1] to 32-bit uint.
uint convertNormalizedFloatToUint16(float valueFloat) {
    return uint(round(clamp(valueFloat, 0.0, 1.0) * 65535.0)) & 0xFFFFu;
}

// Decompression equivalent to function above.
float convertUint16ToNormalizedFloat(uint valueUint) {
    return float(valueUint & 0xFFFFu) / 65535.0;
}


// Converts normalized float in range [0,1] to 32-bit uint.
uint convertNormalizedFloatToUint8(float valueFloat) {
    return uint(round(clamp(valueFloat, 0.0, 1.0) * 255.0)) & 0xFFu;
}

// Decompression equivalent to function above.
float convertUint8ToNormalizedFloat(uint valueUint) {
    return float(valueUint & 0xFFu) / 255.0;
}
