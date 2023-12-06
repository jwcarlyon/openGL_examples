#include <glm/glm.hpp>

class LineShader_CLASS
{
public:
    glm::vec3 startPoint;
    glm::vec3 endPoint;
    glm::vec3 lineColor;

    Line(glm::vec3 start, glm::vec3 end, glm::vec3 color) : startPoint(start), endPoint(end), lineColor(color) {}
  // {
  //     vertices = (float*)(malloc(sizeof(float) * 6));
  //
  //     vertices[0] = start.x;
  //     vertices[1] = start.y;
  //     vertices[2] = start.z;
  //     vertices[3] = end.x;
  //     vertices[4] = end.y;
  //     vertices[5] = end.z;
  // }

    // {
        // startPoint = start;
        // endPoint = end;
        // lineColor = glm::vec3(1,1,1);
        // MVP = glm::mat4(1.0f);
        //
        // const char *vertexShaderSource = "#version 330 core\n"
        //     "layout (location = 0) in glm::vec3 aPos;\n"
        //     "uniform glm::mat4 MVP;\n"
        //     "void main()\n"
        //     "{\n"
        //     "   gl_Position = MVP * vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
        //     "}\0";
        // const char *fragmentShaderSource = "#version 330 core\n"
        //     "out vec4 FragColor;\n"
        //     "uniform glm::vec3 color;\n"
        //     "void main()\n"
        //     "{\n"
        //     "   FragColor = vec4(color, 1.0f);\n"
        //     "}\n\0";
        //
        // // vertex shader
        // std::cout << "line with all local data" << std::endl;
        // int vertexShader = glCreateShader(GL_VERTEX_SHADER);
        // glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
        // glCompileShader(vertexShader);
        // // check for shader compile errors
        //
        // // fragment shader
        // int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
        // glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
        // glCompileShader(fragmentShader);
        // // check for shader compile errors
        //
        // // link shaders
        // shaderProgram = glCreateProgram();
        // glAttachShader(shaderProgram, vertexShader);
        // glAttachShader(shaderProgram, fragmentShader);
        // glLinkProgram(shaderProgram);
        // // check for linking errors
        //
        // glDeleteShader(vertexShader);
        // glDeleteShader(fragmentShader);
        //
        // vertices = {
        //      start.x, start.y, start.z,
        //      end.x, end.y, end.z,
        //
        // };
        //
        // glGenVertexArrays(1, &VAO);
        // glGenBuffers(1, &VBO);
        // glBindVertexArray(VAO);
        //
        // glBindBuffer(GL_ARRAY_BUFFER, VBO);
        // glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices.data(), GL_STATIC_DRAW);
        //
        // glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        // glEnableVertexAttribArray(0);
        //
        // glBindBuffer(GL_ARRAY_BUFFER, 0);
        // glBindVertexArray(0);

    // }

    // int setMVP(glm::mat4 mvp) {
    //     MVP = mvp;
    //     return 1;
    // }

    // int setColor(glm::vec3 color) {
    //     lineColor = color;
    //     return 1;
    // }

    // int draw() {
    //     glUseProgram(shaderProgram);
    //     glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "MVP"), 1, GL_FALSE, &MVP[0][0]);
    //     glUniform3fv(glGetUniformLocation(shaderProgram, "color"), 1, &lineColor[0]);
    //
    //     glBindVertexArray(VAO);
    //     glDrawArrays(GL_LINES, 0, 2);
    //     return 1;
    // }

    // ~Line()
    // {
    //     glDeleteVertexArrays(1, &VAO);
    //     glDeleteBuffers(1, &VBO);
    //     glDeleteProgram(shaderProgram);
    // }
};
