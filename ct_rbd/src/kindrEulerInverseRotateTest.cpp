/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#include <chrono>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-value"
#include <kindr/Core>
#pragma GCC diagnostic pop

int main()
{
    static const size_t nTests = 1000000;
    std::vector<kindr::EulerAnglesXyzD, Eigen::aligned_allocator<kindr::EulerAnglesXyzD>> angles(nTests);

    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> test_vector_t;
    test_vector_t vectors(nTests);
    test_vector_t vectorsRotatedNormal(nTests);
    test_vector_t vectorsRotatedMatrix(nTests);

    std::cout << "running " << nTests << " rotation tests" << std::endl;

    for (size_t i = 0; i < nTests; i++)
    {
        angles[i].setRandom();
        vectors[i].setRandom();
    }

    std::cout << "timing angle.inverseRotate(vector)" << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        vectorsRotatedNormal[i] = angles[i].inverseRotate(vectors[i]);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto diff = end - start;
    size_t msTotal = std::chrono::duration<double, std::milli>(diff).count();
    std::cout << "Total time: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    std::cout << "timing angle.toRotationMatrix().inverted().rotate(vector)" << std::endl;
    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        vectorsRotatedMatrix[i] = kindr::RotationMatrixD(angles[i]).inverted().rotate(vectors[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::milli>(diff).count();
    std::cout << "Total time: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    double error = 0.0;
    double maxError = 0.0;
    size_t maxErrorIndex = 0;
    for (size_t i = 0; i < nTests; i++)
    {
        double err = (vectorsRotatedNormal[i] - vectorsRotatedMatrix[i]).norm();
        if (err > maxError)
        {
            maxError = err;
            maxErrorIndex = i;
        }
        error += err;
    }
    std::cout << "average error: " << error / double(nTests) << std::endl;
    std::cout << "max error " << maxError << " for pair: " << std::endl;
    std::cout << "euler angles: " << angles[maxErrorIndex].toImplementation().transpose() << std::endl;
    std::cout << "vector to rotate: " << vectors[maxErrorIndex].transpose() << std::endl;
    std::cout << "vector inverse rotated (direct): " << vectorsRotatedNormal[maxErrorIndex].transpose() << std::endl;
    std::cout << "vector inverse rotated (rotation matrix transpose): "
              << vectorsRotatedMatrix[maxErrorIndex].transpose() << std::endl;
}
