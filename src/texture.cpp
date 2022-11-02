#include "texture.h"
#include <cmath>
#include <framework/image.h>
#include <iostream>

// Method for getting textures if only texture mapping is enabled
int getIndex(Image image, const glm::vec2& coord)
{
    int x = (int)(coord.x * image.width - 0.5);
    int y = (int)((1 - coord.y) * image.height - 0.5);

    auto index = y * image.width + x;
    return index;
}

// Method for getting index for bilinear interpolation
int bilinearIndex(Image image, int x, int y)
{
    int bilinearX = fmax(fmin(image.width - 1, x), 0);
    int bilinearY = fmax(fmin(image.height - 1, y), 0);
    return bilinearY * image.width + bilinearX;
}

glm::vec3 bilinearInterpolation(Image image, const glm::vec2 texCoord)
{
    // Compute indexes
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

    // Get colors at indexes
    glm::vec3 t_iu0_iv0 = image.pixels[bilinearIndex(image, iu0, iv0)];
    glm::vec3 t_iu0_iv1 = image.pixels[bilinearIndex(image, iu0, iv1)];
    glm::vec3 t_iu1_iv0 = image.pixels[bilinearIndex(image, iu1, iv0)];
    glm::vec3 t_iu1_iv1 = image.pixels[bilinearIndex(image, iu1, iv1)];

    // Compute interpolated color
    return a_u * a_v * t_iu0_iv0
        + a_u * b_v * t_iu0_iv1
        + b_u * a_v * t_iu1_iv0
        + b_u * b_v * t_iu1_iv1;
}

// Get texture pixel
glm::vec3 acquireTexel(const Image& image, const glm::vec2& texCoord, const Features& features)
{
    if (features.extra.enableBilinearTextureFiltering) {
        return bilinearInterpolation(image, texCoord);
    } else {
        int index = getIndex(image, texCoord);
        return image.pixels[index];
    }
}

// Get color from the cube_map (for the surrounding)
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

    //Check on which face is the point
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


    //Width/Height of the face
    float measure = image.width / 4.0f;

    float i;
    float j;


    //Calculate texel based on the #face we are on
    switch (helper) {
    case 0: {
        i = (2 + hor) * measure;
        j = (1 + ver) * measure;
    } break;
    case 1: {
        i = (2 + hor) * measure;
        j = (1 + ver) * measure;
    } break;
    case 2: {
        i = hor * measure;
        j = (1 + ver) * measure;
    } break;
    case 3: {
        i = (1 + hor) * measure;
        j = (2 + ver) * measure;
    } break;
    case 4: {
        i = (1 + hor) * measure;
        j = (1 + ver) * measure;
    } break;
    case 5: {
        i = (3 + hor) * measure;
        j = (1 + ver) * measure;
    } break;
    default:
        i = 0.0f;
        j = 0.0f;
    }

    int k = floor(i);
    int l = floor(j);

    return image.pixels[image.width * l + k];
}