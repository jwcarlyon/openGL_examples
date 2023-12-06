#ifndef TRACKLAYOUT_H_INCLUDED
#define TRACKLAYOUT_H_INCLUDED

#include <glm/glm.hpp>
#include <mutex>

class ThreadSafe_matrix4
{
public:

    ThreadSafe_matrix4(const glm::mat4& matrix_ref) : matrix(matrix_ref) {}

    ~ThreadSafe_matrix4(){}

    glm::mat4 get()
    {
        std::unique_lock<std::mutex> lock(matrixMutex);
        return matrix; // return a copy of the matrix
    }

    void set(const glm::mat4& matrix_ref)
    {
        std::unique_lock<std::mutex> lock(matrixMutex);
        matrix = matrix_ref;
    }

private:

    std::mutex matrixMutex;
    glm::mat4 matrix;
};

class TrackLayout
{
public:

    ThreadSafe_matrix4 turnOne;
    ThreadSafe_matrix4 turnTwo;
    ThreadSafe_matrix4 turnThree;
    ThreadSafe_matrix4 turnFour;

    TrackLayout() :
        turnOne(glm::mat4(1.f)),
        turnTwo(glm::mat4(1.f)),
        turnThree(glm::mat4(1.f)),
        turnFour(glm::mat4(1.f))
    {}

    ~TrackLayout(){}
};

#endif // TRACKLAYOUT_H_INCLUDED
