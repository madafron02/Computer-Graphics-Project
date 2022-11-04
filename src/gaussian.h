#include "screen.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>
#include <vector>

// Reference: https://blog.ivank.net/fastest-gaussian-blur.html
namespace Gaussian {

    std::vector<int> boxesForGauss(float sigma, int n)
    {
        auto wIdeal = std::sqrt((12 * sigma * sigma / n) + 1); 
        int wl = floor(wIdeal);
        if (wl % 2 == 0)
            wl--;
        int wu = wl + 2;

        auto mIdeal = (12 * sigma * sigma - n * wl * wl - 4 * n * wl - 3 * n) / (-4 * wl - 4);
        int m = round(mIdeal);

        std::vector<int> sizes(n);
        for (auto i = 0; i < n; i++)
            sizes[i] = i < m ? wl : wu;
        return sizes;
    }

    void boxFilterVertical(const Screen& screen, const std::vector<glm::vec3>& source, std::vector<glm::vec3>& dest, int filterSize)
    {
        if (filterSize < 1)
            filterSize = 1;

        auto resolution = screen.resolution();

        for (int i = 0; i < resolution.x; ++i) {
            for (int j = 0; j < resolution.y; ++j) {
                for (int col = 0; col < 3; ++col) {

                    float sum = 0;

                    for (int y = -filterSize; y < filterSize + 1; ++y) {
                        try {
                            sum += source.at(screen.indexAt(i, j + y))[col];

                        } catch (const std::out_of_range& e) {
                            sum += 0.0f; // out of range: assume color is black.
                        }
                    }

                    dest.at(screen.indexAt(i, j))[col] = sum / (2 * filterSize + 1);
                }
            }
        } 
    }

    void boxFilterTotal(const Screen& screen, const std::vector<glm::vec3>& source, std::vector<glm::vec3>& dest, int filterSize)
    {
        if (filterSize < 1)
            filterSize = 1;

        auto resolution = screen.resolution();

        for (int i = 0; i < resolution.x; ++i) {
            for (int j = 0; j < resolution.y; ++j) {
                for (int col = 0; col < 3; ++col) {

                    float sum = 0;

                    for (int x = -filterSize; x < filterSize + 1; ++x) {
                        try {
                            sum += source.at(screen.indexAt(i + x, j))[col];
                        } catch (const std::out_of_range& e) {
                            sum += 0.0f; // out of range: assume color is black.
                        }
                    }

                    
                    dest.at(screen.indexAt(i, j))[col] = sum / (2 * filterSize + 1);
                }
            }
        }
    }

    void boxFilter(const Screen& screen, const std::vector<glm::vec3>& source, std::vector<glm::vec3>& dest, int filterSize)
    {
        std::vector<glm::vec3> copy(source.size());

        auto resolution = screen.resolution();

        boxFilterVertical(screen, source, copy, filterSize);
        boxFilterTotal(screen, copy, dest, filterSize);
    }

    void gaussBlur(const Screen& screen, const std::vector<glm::vec3>& source, std::vector<glm::vec3>& dest, float sigma)
    {       
        auto x = Gaussian::boxesForGauss(sigma, 3);
        std::vector<glm::vec3> pixels1(source.size());
   
        boxFilter(screen, source, dest, x.at(0));
        boxFilter(screen, dest, pixels1, x.at(1));
        boxFilter(screen, pixels1, dest, x.at(2));
    }
}