#include "texture.h"
#include <framework/image.h>
#include <iostream>
#include <cmath>

int getIndex(Image image, const glm::vec2& texCoord)
{
    auto x = (int)(texCoord.x * image.width);
    auto y = (int)((1 - texCoord.y) * image.height);

    auto index = y * image.width + x;
    return index;
}

int billinearIndex(Image image, int x, int y)
{
    return y * image.width + x;
}

glm::vec3 acquireTexel(const Image& image, const glm::vec2& texCoord, const Features& features)
{
    // TODO: implement this function.
    // Given texcoords, return the corresponding pixel of the image
    // The pixel are stored in a 1D array of row major order
    // you can convert from position (i,j) to an index using the method seen in the lecture
    // Note, the center of the first pixel is at image coordinates (0.5, 0.5)
    if (features.extra.enableBilinearTextureFiltering) {
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
    } else {
        int index = getIndex(image, texCoord);
        return image.pixels[index];
    }
}