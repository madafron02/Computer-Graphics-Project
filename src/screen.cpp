#include "screen.h"
#include "gaussian.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/common.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb/stb_image_write.h>
DISABLE_WARNINGS_POP()
#include <algorithm>
#include <framework/opengl_includes.h>
#include <iostream>
#include <string>

float BLOOM_FILTER_SCALE;
float BLOOM_FILTER_SIGMA;
float BLOOM_FILTER_THRESHOLD;

Screen::Screen(const glm::ivec2& resolution, bool presentable)
    : m_presentable(presentable)
    , m_resolution(resolution)
    , m_textureData(size_t(resolution.x * resolution.y), glm::vec3(0.0f))
{
    // Create OpenGL texture if we want to present the screen.
    if (m_presentable) {
        // Generate texture
        glGenTextures(1, &m_texture);
        glBindTexture(GL_TEXTURE_2D, m_texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glBindTexture(GL_TEXTURE_2D, 0);
    }
}

void Screen::clear(const glm::vec3& color)
{
    std::fill(std::begin(m_textureData), std::end(m_textureData), color);
}

void Screen::setPixel(int x, int y, const glm::vec3& color)
{
    // In the window/camera class we use (0, 0) at the bottom left corner of the screen (as used by GLFW).
    // OpenGL / stbi like the origin / (-1,-1) to be at the TOP left corner so transform the y coordinate.
    const int i = (m_resolution.y - 1 - y) * m_resolution.x + x;
    m_textureData[i] = glm::vec4(color, 1.0f);
}

void Screen::writeBitmapToFile(const std::filesystem::path& filePath)
{
    std::vector<glm::u8vec4> textureData8Bits(m_textureData.size());
    std::transform(std::begin(m_textureData), std::end(m_textureData), std::begin(textureData8Bits),
        [](const glm::vec3& color) {
            const glm::vec3 clampedColor = glm::clamp(color, 0.0f, 1.0f);
            return glm::u8vec4(glm::vec4(clampedColor, 1.0f) * 255.0f);
        });

    std::string filePathString = filePath.string();
    stbi_write_bmp(filePathString.c_str(), m_resolution.x, m_resolution.y, 4, textureData8Bits.data());
}

void Screen::draw()
{
    if (m_presentable) {
        glPushAttrib(GL_ALL_ATTRIB_BITS);

        glBindTexture(GL_TEXTURE_2D, m_texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, m_resolution.x, m_resolution.y, 0, GL_RGB, GL_FLOAT, m_textureData.data());

        glDisable(GL_LIGHTING);
        glDisable(GL_LIGHT0);
        glDisable(GL_COLOR_MATERIAL);
        glDisable(GL_NORMALIZE);
        glColor3f(1.0f, 1.0f, 1.0f);

        glEnable(GL_TEXTURE_2D);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_texture);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();

        glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 1.0f);
        glVertex3f(-1.0f, -1.0f, 0.0f);
        glTexCoord2f(1.0f, 1.0f);
        glVertex3f(+1.0f, -1.0f, 0.0f);
        glTexCoord2f(1.0f, 0.0f);
        glVertex3f(+1.0f, +1.0f, 0.0f);
        glTexCoord2f(0.0f, 0.0f);
        glVertex3f(-1.0f, +1.0f, 0.0f);
        glEnd();

        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();

        glPopAttrib();
    } else {
        std::cerr << "Screen::draw() called on non-presentable screen" << std::endl;
    }
}

glm::ivec2 Screen::resolution() const
{
    return m_resolution;
}

const std::vector<glm::vec3>& Screen::pixels() const
{
    return m_textureData;
}

std::vector<glm::vec3>& Screen::pixels()
{
    return m_textureData;
}

float Screen::boxFilter(const std::vector<glm::vec3>& source, int i, int j, int col, int filterSize)
{
    if (filterSize < 1)
        filterSize = 1;

    float sum = 0;
    for (int x = -filterSize; x < filterSize + 1; ++x) {
        for (int y = -filterSize; y < filterSize + 1; ++y) {
            try {
                sum += source.at(indexAt(i + x, j + y))[col];
            } 
            catch (const std::out_of_range& e) {
                sum += 0.0f; // out of range: assume color is black.
            }
        }
    }

    return sum / ((2 * filterSize + 1) * (2 * filterSize + 1));
}

int Screen::indexAt(int x, int y) const
{
    return (m_resolution.y - 1 - y) * m_resolution.x + x;
}

void Screen::applyBloomFilter(const std::filesystem::path& filePath)
{
    // m_textureData
    // 1. Threshold values
    std::vector<glm::vec3> pixels(m_textureData.size());

    std::transform(std::begin(m_textureData), std::end(m_textureData), std::begin(pixels),
        [](const glm::vec3& color) {
            return color.x >= BLOOM_FILTER_THRESHOLD || color.y >= BLOOM_FILTER_THRESHOLD || color.z >= BLOOM_FILTER_THRESHOLD ? color : glm::vec3 { 0.0f, 0.0f, 0.0f };
            //return color.x >= threshold || color.y >= threshold || color.z >= threshold ? color : color;
        });

    // [+] DEBUG: render thresholded values to an image
    auto helper = m_textureData;
    m_textureData = pixels;

    std::filesystem::path debugPath { filePath };
    writeBitmapToFile(debugPath.replace_filename(filePath.stem().string() + "_debug.bmp"));

    m_textureData = helper;
    // 2. Apply box filter 

    //std::vector<glm::vec3> pixels_boxFilter(m_textureData.size());
    using clock = std::chrono::high_resolution_clock;
    const auto start = clock::now();

    std::vector<glm::vec3> withGauss(m_textureData.size());
    Gaussian::gaussBlur(*this, pixels, withGauss, BLOOM_FILTER_SIGMA);
    
    const auto end = clock::now();
    std::cout << "Time to RENDER the FILTER: only " << std::chrono::duration<float, std::milli>(end - start).count() << " milliseconds :)\n" << std::endl;

    // 4.  Scale the bloom and add final bloom to the image :)
    glm::vec3 scaler { BLOOM_FILTER_SCALE };
    for (int i = 0; i < m_textureData.size(); ++i) {
        m_textureData.at(i) += withGauss.at(i) * scaler;
    }
}