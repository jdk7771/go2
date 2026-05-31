#include <mujoco/mujoco.h>
#include <iostream>
#include <GLFW/glfw3.h>


int main()
{
    const char* model_path = "/home/jiang/tools/unitree_mujoco/unitree_robots/go2/scene.xml";
    
    char error_msg[1000] = "Could not load model";

    mjModel *model = mj_loadXML(model_path,nullptr,error_msg,1000);

    mjData*data = mj_makeData(model);


    std::cout<<model->nq<<" "<<"广义位置"<<std::endl;
    std::cout<<model->nv<<" "<<"广义速度"<<std::endl;
    std::cout<<model->nu<<" "<<"控制数量"<<std::endl;
    
    if(!glfwInit())
    {
        std::cerr<<"GLFW初始化失败！"<<std::endl;
        return 1;
    }

    GLFWwindow* window = glfwCreateWindow(1200,900,"Go2仿真控制台",NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // 开启垂直同步

// 3. 初始化 MuJoCo 的渲染数据结构 (相机、选项、场景、GPU上下文)
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;
    mjvPerturb pert;
    
    bool button_left = false;
    bool button_middle = false;
    bool button_right = false;
    double lastx = 0,lasty = 0;

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // 将物理模型映射到渲染场景中
    mjv_makeScene(model, &scn, 2000);
    mjr_makeContext(model, &con, mjFONTSCALE_150);

    std::cout << "========== 可视化仿真启动 ==========" << std::endl;

    // 4. 主循环：只要窗口没被关掉，就一直运行
    while (!glfwWindowShouldClose(window)) {
        
        // 物理推演循环：让物理时间追赶上真实渲染时间 (假设显示器 60Hz)
        mjtNum simstart = data->time;
        
        while (data->time - simstart < 1.0 / 60.0) {
            mj_step(model, data);
            
            // 这里就是你未来写入 WBC 控制逻辑的注入点！
            // 例如：data->ctrl[0] = 你的计算结果;
        }

        // 渲染准备：获取当前窗口大小
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // 更新场景并渲染到屏幕
        mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // 交换缓冲区，处理鼠标键盘事件
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 5. 优雅地释放所有资源
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(data);
    mj_deleteModel(model);
    glfwTerminate();

    return 0;

}