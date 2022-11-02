#include "texture.h"
#include <framework/image.h>
#include <iostream>
#include <cmath>

int getIndex(Image image, const glm::vec2& coord)
{
    int x = (int)(coord.x * image.width - 0.5);
    int y = (int)((1 - coord.y) * image.height - 0.5);

    auto index = y * image.width + x;
    return index;
}

int billinearIndex(Image image, int x, int y)
{
    int bilinearX = fmax(fmin(image.width - 1, x), 0);
    int bilinearY = fmax(fmin(image.height - 1, y), 0);
    return bilinearY * image.width + bilinearX;
}


glm::vec3 bilinearInterpolation(Image image, const glm::vec2 texCoord)
{
    float u_p = texCoord.x;
    float v_p = (1 - texCoord.y);
    float iu0 = floor(u_p);
    float iv0 = floor(v_p);
    float iu1 = iu0 + 1;
    float iv1 = iv0 + 1;
    float a_u = iu1 - u_p;
    float a_v = iv1 - v_p;
    float b_u = 1 - a_u;
    float b_v = 1 - a_v;

    glm::vec3 t_iu0_iv0 = image.pixels[billinearIndex(image, iu0, iv0)];
    glm::vec3 t_iu0_iv1 = image.pixels[billinearIndex(image, iu0, iv1)];
    glm::vec3 t_iu1_iv0 = image.pixels[billinearIndex(image, iu1, iv0)];
    glm::vec3 t_iu1_iv1 = image.pixels[billinearIndex(image, iu1, iv1)];
    return a_u * a_v * t_iu0_iv0
        + a_u * b_v * t_iu0_iv1
        + b_u * a_v * t_iu1_iv0
        + b_u * b_v * t_iu1_iv1;
}


glm::vec3 acquireTexel(const Image& image, const glm::vec2& texCoord, const Features& features)
{
    // TODO: implement this function.
    // Given texcoords, return the corresponding pixel of the image
    // The pixel are stored in a 1D array of row major order
    // you can convert from position (i,j) to an index using the method seen in the lecture
    // Note, the center of the first pixel is at image coordinates (0.5, 0.5)
    if (features.extra.enableBilinearTextureFiltering) {
        return bilinearInterpolation(image, texCoord);
    } else {
        int index = getIndex(image, texCoord);
        return image.pixels[index];
    }
}

glm::vec3 getEnvironmentTexel(const Image& image, const glm::vec3& rayDirection)
{
    float x_abs = abs(rayDirection.x);
    float y_abs = abs(rayDirection.y);
    float z_abs = abs(rayDirection.z);

    float maxDir = std::max(x_abs, std::max(y_abs, z_abs));

    if (maxDir < 0.00001f) {
        return glm::vec3(0.0f);
    }

    float x = rayDirection.x / maxDir;
    float y = rayDirection.y / maxDir;
    float z = rayDirection.z / maxDir;

    int helper = -1;

    float hor = 0.0f;
    float ver = 0.0f;

    if (x_abs >= y_abs && x_abs >= z_abs) {
        if (x > 0.0f) {
            helper = 0;
            hor = -z;
            ver = -y;
        } else {
            helper = 1;
            hor = z;
            ver = -y;
        }
    } else if (y_abs >= x_abs && y_abs >= z_abs) {
        if (y > 0.0f) {
            helper = 2;
            hor = x;
            ver = z;
        } else {
            helper = 3;
            hor = x;
            ver = -z;
        }
    } else if (z_abs >= x_abs && z_abs >= y_abs) {
        if (z > 0.0f) {
            helper = 4;
            hor = x;
            ver = -y;
        } else {
            helper = 5;
            hor = -x;
            ver = -y;
        }
    }

    hor = (hor + 1.0f) / 2.0f;
    ver = (ver + 1.0f) / 2.0f;

    float width = image.width / 4.0f;
    float height = image.height / 3.0f;

    float i;
    float j;

    switch (helper) {
    case 0: {
        i = (2 + hor) * width;
        j = (1 + ver) * height;
    } break;
    case 1: {
        i = (2 + hor) * width;
        j = (1 + ver) * height;
    } break;
    case 2: {
        i = hor * width;
        j = (1 + ver) * height;
    } break;
    case 3: {
        i = (1 + hor) * width;
        j = (2 + ver) * height;
    } break;
    case 4: {
        i = (1 + hor) * width;
        j = (1 + ver) * height;
    } break;
    case 5: {
        i = (3 + hor) * width;
        j = (1 + ver) * height;
    } break;
    default:
        i = 0.0f;
        j = 0.0f;
    }

    int k = floor(i);
    int l = floor(j);

    return image.pixels[image.width * l + k];
}