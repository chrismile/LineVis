// Compresses RGBA color into 32-bit unsigned integer (8 bits per channel)
uint packColorRGBA(vec4 vecColor)
{
    uint packedColor;
    packedColor = uint(round(clamp(vecColor.r, 0.0, 1.0) * 255.0)) & 0xFFu;
    packedColor |= (uint(round(clamp(vecColor.g, 0.0, 1.0) * 255.0)) & 0xFFu) << 8;
    packedColor |= (uint(round(clamp(vecColor.b, 0.0, 1.0) * 255.0)) & 0xFFu) << 16;
    packedColor |= (uint(round(clamp(vecColor.a, 0.0, 1.0) * 255.0)) & 0xFFu) << 24;
    return packedColor;
}

// Decompression equivalent to function above
vec4 unpackColorRGBA(uint packedColor)
{
    vec4 vecColor;
    vecColor.r = float(packedColor & 0xFFu) / 255.0;
    vecColor.g = float((packedColor >> 8)  & 0xFFu) / 255.0;
    vecColor.b = float((packedColor >> 16) & 0xFFu) / 255.0;
    vecColor.a = float((packedColor >> 24) & 0xFFu) / 255.0;
    return vecColor;
}

// Decompression equivalent (only alpha!) to function above
float unpackColorAlpha(uint packedColor)
{
    return float((packedColor >> 24) & 0xFFu) / 255.0;
}

// Replace alpha with new value. Mainly used in Adaptive Transparency
// to update alpha of packed color without decompressing.
void updatePackedColorAlpha(inout uint packedColor, in float newAlpha)
{
    packedColor &= 0xFFFFFFu;
    packedColor |= (uint(round(clamp(newAlpha, 0.0, 1.0) * 255.0)) & 0xFFu) << 24;
}



// --- Functions below used for Hybrid Transparency algorithm ---

// Compresses HDR RGB color with 10 bits per channel (last two bits unused)
uint packColor30bit(vec4 vecColor)
{
    uint packedColor;
    packedColor = uint(round(vecColor.r * 255.0)) & 0x3FFu;
    packedColor |= (uint(round(vecColor.g * 255.0)) & 0x3FFu) << 10;
    packedColor |= (uint(round(vecColor.b * 255.0)) & 0x3FFu) << 20;
    return packedColor;
}

// Decompression equivalent to function above
vec4 unpackColor30bit(uint packedColor)
{
    vec4 vecColor;
    vecColor.r = float(packedColor & 0x3FFu) / 255.0;
    vecColor.g = float((packedColor >> 10)  & 0x3FFu) / 255.0;
    vecColor.b = float((packedColor >> 20) & 0x3FFu) / 255.0;
    vecColor.a = 1.0;
    return vecColor;
}

// Packs accumulated alpha into lower two bytes, fragment count into higher two bytes
uint packAccumAlphaAndFragCount(float accumAlpha, uint fragCount)
{
    uint packedValue = uint(round(accumAlpha * 255.0)) & 0xFFFFu;
    packedValue |= (fragCount & 0xFFFFu) << 16;
    return packedValue;
}

// Decompression equivalent to function above
void unpackAccumAlphaAndFragCount(in uint packedValue, out float accumAlpha, out uint fragCount)
{
    accumAlpha = float(packedValue & 0xFFFFu) / 255.0;
    fragCount = (packedValue >> 16) & 0xFFFFu;
}