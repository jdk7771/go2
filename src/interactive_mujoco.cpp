#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <iostream>

namespace {

mjModel* g_model = nullptr;
mjData* g_data = nullptr;
mjvCamera g_cam;
mjvOption g_opt;
mjvScene g_scn;
mjrContext g_con;
mjvPerturb g_pert;

bool g_button_left = false;
bool g_button_middle = false;
bool g_button_right = false;
double g_lastx = 0.0;
double g_lasty = 0.0;

void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    (void)mods;

    g_button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    g_button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    g_button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &g_lastx, &g_lasty);

    if (act == GLFW_PRESS && button == GLFW_MOUSE_BUTTON_LEFT) {
        static double last_click_time = 0.0;
        const double current_time = glfwGetTime();

        if (current_time - last_click_time < 0.25) {
            int width = 0;
            int height = 0;
            glfwGetFramebufferSize(window, &width, &height);

            mjtNum selpnt[3] = {0, 0, 0};
            int geomid = -1;
            int flexid = -1;
            int skinid = -1;

            const int selbody = mjv_select(
                g_model,
                g_data,
                &g_opt,
                static_cast<mjtNum>(width) / static_cast<mjtNum>(height),
                static_cast<mjtNum>(g_lastx) / static_cast<mjtNum>(width),
                static_cast<mjtNum>(height - g_lasty) / static_cast<mjtNum>(height),
                &g_scn,
                selpnt,
                &geomid,
                &flexid,
                &skinid);

            g_pert.select = selbody;
            g_pert.active = 0;

            if (selbody >= 0) {
                mjv_initPerturb(g_model, g_data, &g_scn, &g_pert);
                std::cout << "selected body id: " << selbody << std::endl;
            } else {
                std::cout << "selection cleared" << std::endl;
            }
        }

        last_click_time = current_time;
    }
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    if (!g_button_left && !g_button_middle && !g_button_right) {
        return;
    }

    const double dx = xpos - g_lastx;
    const double dy = ypos - g_lasty;
    g_lastx = xpos;
    g_lasty = ypos;

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    if (height == 0) {
        return;
    }

    const bool mod_shift =
        glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    const bool mod_ctrl =
        glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;

    if (mod_ctrl && g_pert.select >= 0) {
        const int action = g_button_right
                               ? mjMOUSE_MOVE_H
                               : (g_button_left ? mjMOUSE_MOVE_V : mjMOUSE_ROTATE_V);
        g_pert.active = action;
        mjv_movePerturb(
            g_model,
            g_data,
            action,
            static_cast<mjtNum>(dx) / static_cast<mjtNum>(height),
            static_cast<mjtNum>(dy) / static_cast<mjtNum>(height),
            &g_scn,
            &g_pert);
    } else {
        int action = mjMOUSE_ZOOM;
        if (g_button_right) {
            action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        } else if (g_button_left) {
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        }

        mjv_moveCamera(
            g_model,
            action,
            static_cast<mjtNum>(dx) / static_cast<mjtNum>(height),
            static_cast<mjtNum>(dy) / static_cast<mjtNum>(height),
            &g_scn,
            &g_cam);
    }
}

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    (void)window;
    (void)xoffset;

    mjv_moveCamera(g_model, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &g_scn, &g_cam);
}

}  // namespace

int main() {
    const char* model_path = "/home/jiang/tools/unitree_mujoco/unitree_robots/go2/scene.xml";
    char error_msg[1000] = "Could not load model";

    g_model = mj_loadXML(model_path, nullptr, error_msg, sizeof(error_msg));
    if (!g_model) {
        std::cerr << error_msg << std::endl;
        return 1;
    }

    g_data = mj_makeData(g_model);
    if (!g_data) {
        std::cerr << "mj_makeData failed" << std::endl;
        mj_deleteModel(g_model);
        return 1;
    }

    std::cout << g_model->nq << " 广义位置" << std::endl;
    std::cout << g_model->nv << " 广义速度" << std::endl;
    std::cout << g_model->nu << " 控制数量" << std::endl;

    if (!glfwInit()) {
        std::cerr << "GLFW初始化失败" << std::endl;
        mj_deleteData(g_data);
        mj_deleteModel(g_model);
        return 1;
    }

    GLFWwindow* window = glfwCreateWindow(1200, 900, "Go2 MuJoCo Interaction", nullptr, nullptr);
    if (!window) {
        std::cerr << "GLFW窗口创建失败" << std::endl;
        glfwTerminate();
        mj_deleteData(g_data);
        mj_deleteModel(g_model);
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    mjv_defaultCamera(&g_cam);
    mjv_defaultFreeCamera(g_model, &g_cam);
    mjv_defaultOption(&g_opt);
    mjv_defaultScene(&g_scn);
    mjr_defaultContext(&g_con);
    mjv_defaultPerturb(&g_pert);

    mjv_makeScene(g_model, &g_scn, 2000);
    mjr_makeContext(g_model, &g_con, mjFONTSCALE_150);

    std::cout << "双击左键选中刚体，按住 Ctrl + 左键/右键拖拽施加扰动。" << std::endl;

    while (!glfwWindowShouldClose(window)) {
        const mjtNum simstart = g_data->time;

        while (g_data->time - simstart < 1.0 / 60.0) {
            mjv_applyPerturbPose(g_model, g_data, &g_pert, 0);
            mjv_applyPerturbForce(g_model, g_data, &g_pert);
            mj_step(g_model, g_data);
        }

        mjrRect viewport{0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        mjv_updateScene(g_model, g_data, &g_opt, nullptr, &g_cam, mjCAT_ALL, &g_scn);
        mjr_render(viewport, &g_scn, &g_con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    mjv_freeScene(&g_scn);
    mjr_freeContext(&g_con);
    glfwDestroyWindow(window);
    glfwTerminate();
    mj_deleteData(g_data);
    mj_deleteModel(g_model);

    return 0;
}
