#ifndef WIREFRAMECOLLECTOR_H_INCLUDED
#define WIREFRAMECOLLECTOR_H_INCLUDED

#include <mutex>

struct Line
{
public:
    glm::vec3 startPoint;
    glm::vec3 endPoint;
    glm::vec3 lineColor;

    Line(glm::vec3 start, glm::vec3 end, glm::vec3 color) : startPoint(start), endPoint(end), lineColor(color) {}
};

class WireframeCollector
{
public:
    void drawLines(
        const float* linePointArray_floatPtr,
        const int linePointArraySize_int,
        const float* colorArray_floatPtr
    ) {
        std::cout << "WireframeCollector - draw wireframe given block of lines with size: " << linePointArraySize_int << std::endl;
        // float rotation_fl = M_PI / 2.f;
        // objectShader_ref.use();
        // model_m4 = glm::rotate(glm::mat4(1.0f), rotation_fl, glm::vec3(0.f, 1.f, 0.f));
        glm::vec3 color_vec3(colorArray_floatPtr[0], colorArray_floatPtr[1], colorArray_floatPtr[2]);
        for(int i = 0; ((i * 6) + 5) < linePointArraySize_int; i++)
        {
            int currentStridePosition = i * 6;
            glm::vec3 fromVertex_vec3(
                linePointArray_floatPtr[currentStridePosition],
                linePointArray_floatPtr[currentStridePosition + 1],
                linePointArray_floatPtr[currentStridePosition + 2]
            );
            glm::vec3 toVertex_vec3(
                linePointArray_floatPtr[currentStridePosition + 3],
                linePointArray_floatPtr[currentStridePosition + 4],
                linePointArray_floatPtr[currentStridePosition + 5]
            );
            // bool isRed = ((i % 3) == 0); bool isBlue = ((i % 3) == 1); bool isGreen = ((i % 3) == 2);
            Line line(
                fromVertex_vec3,
                toVertex_vec3,
                color_vec3
            );
            std::unique_lock<std::mutex> lock(collectorMutex);
            lineCache_vector.push_back(line);
        }
    }

    std::vector<Line>& getLineVector()
    {
        std::unique_lock<std::mutex> lock(collectorMutex);
        return lineCache_vector;
        // writeable_lineVector.clear();
        // writeable_lineVector.reserve(lineCache_vector.size());
        // for(auto line : lineCache_vector) { writeable_lineVector.push_back(line); }
    }

    void clearLineVector() {
        std::unique_lock<std::mutex> lock(collectorMutex);
        lineCache_vector.clear();
    }

private:
        std::mutex collectorMutex;
        std::vector<Line> lineCache_vector;
};

#endif // WIREFRAMECOLLECTOR_H_INCLUDED
