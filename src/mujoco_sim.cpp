#include <iostream>
#include <string>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

namespace go2_control {

class MujocoSim{
public:
    MujocoSim(const std::string& model_path)
    {
        model_path_ = model_path.c_str();

        model_ = mj_loadXML(model_path_,nullptr,error_msg,1000);
        data_ = mj_makeData(model_);

    }


    ~MujocoSim(){};
    void Start_Sim(){
        glfwInit();
        GLFWwindow* window = glfwCreateWindow(1200,900,"go2",nullptr,nullptr);
        

    }

private:
    const char* model_path_;
    mjModel* model_;
    mjData* data_;
    char error_msg[1000];
};

}

