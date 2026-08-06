#pragma once

#include <iostream>
#include <unitree/dds_wrapper/common/unitree_joystick.hpp>
#include <GLFW/glfw3.h>
#include <memory>
#include <map>

/**
 * @brief Keyboard-based joystick emulation for unitree_mujoco
 * @author Zhang Zhen (zhangzhen@cmhi.chinamobile.com)
 * 
 * Keyboard mapping:
 * - IJKL: Left stick (lx, ly) - movement control
 * - U/O: Right stick rx (left/right turn)
 * - A/B/X/Y: A/B/X/Y buttons
 * - LCtrl/LShift: L1/L2
 * - RCtrl/RShift: R1/R2
 * - Arrow keys: D-pad (up/left/down/right)
 * - 1: F1 | 2: F2
 * - Tab: Start | Esc: Select
 */
class KeyboardJoystick : public unitree::common::UnitreeJoystick
{
public:
    KeyboardJoystick(GLFWwindow* window)
        : unitree::common::UnitreeJoystick(), window_(window)
    {
        if (!window_) {
            std::cout << "Error: GLFW window is null." << std::endl;
            exit(1);
        }
        std::cout << "[KeyboardJoystick] Initialized. Use keyboard to control:" << std::endl;
        std::cout << "  IJKL: Move (left stick)" << std::endl;
        std::cout << "  U/O: Turn left/right (right stick rx)" << std::endl;
        std::cout << "  A/B/X/Y: A/B/X/Y buttons" << std::endl;
        std::cout << "  LCtrl/LShift: L1/L2 | RCtrl/RShift: R1/R2" << std::endl;
        std::cout << "  Arrow keys: D-pad (up/left/down/right)" << std::endl;
        std::cout << "  1: F1 | 2: F2 | Tab: Start | Esc: Select" << std::endl;
        std::cout << "  3: Lock Stand (L2+Up) | 4: Walk (R2+A)" << std::endl;
    }

    void update() override
    {
        const bool macro_l2_up = (glfwGetKey(window_, GLFW_KEY_3) == GLFW_PRESS);
        const bool macro_r2_a = (glfwGetKey(window_, GLFW_KEY_4) == GLFW_PRESS);

        // ABXY buttons (A/B/X/Y)
        A((glfwGetKey(window_, GLFW_KEY_A) == GLFW_PRESS) || macro_r2_a);
        B(glfwGetKey(window_, GLFW_KEY_B) == GLFW_PRESS);
        X(glfwGetKey(window_, GLFW_KEY_X) == GLFW_PRESS);
        Y(glfwGetKey(window_, GLFW_KEY_Y) == GLFW_PRESS);
        
        // Shoulder buttons: L1=LCtrl, L2=LShift, R1=RCtrl, R2=RShift
        LB(glfwGetKey(window_, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS);   // L1
        LT((glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) || macro_l2_up);    // L2
        RB(glfwGetKey(window_, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS);  // R1
        RT((glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS) || macro_r2_a);   // R2
        
        // D-pad (Arrow keys)
        up((glfwGetKey(window_, GLFW_KEY_UP) == GLFW_PRESS) || macro_l2_up);
        left(glfwGetKey(window_, GLFW_KEY_LEFT) == GLFW_PRESS);
        down(glfwGetKey(window_, GLFW_KEY_DOWN) == GLFW_PRESS);
        right(glfwGetKey(window_, GLFW_KEY_RIGHT) == GLFW_PRESS);
        
        // Function keys: F1 = 1, F2 = 2
        F1(glfwGetKey(window_, GLFW_KEY_1) == GLFW_PRESS);
        F2(glfwGetKey(window_, GLFW_KEY_2) == GLFW_PRESS);
        
        // Other
        back(glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS);
        start(glfwGetKey(window_, GLFW_KEY_TAB) == GLFW_PRESS);
        
        // Left stick (IJKL for movement)
        double lx_val = 0.0;
        double ly_val = 0.0;
        
        if (glfwGetKey(window_, GLFW_KEY_J) == GLFW_PRESS) lx_val -= 1.0;
        if (glfwGetKey(window_, GLFW_KEY_L) == GLFW_PRESS) lx_val += 1.0;
        if (glfwGetKey(window_, GLFW_KEY_I) == GLFW_PRESS) ly_val += 1.0;
        if (glfwGetKey(window_, GLFW_KEY_K) == GLFW_PRESS) ly_val -= 1.0;
        
        // Normalize diagonal movement
        if (lx_val != 0.0 && ly_val != 0.0) {
            double norm = std::sqrt(lx_val * lx_val + ly_val * ly_val);
            lx_val /= norm;
            ly_val /= norm;
        }
        
        lx(lx_val);
        ly(ly_val);
        
        // Right stick (Arrow keys for rotation/camera)
        double rx_val = 0.0;
        double ry_val = 0.0;
        
        if (glfwGetKey(window_, GLFW_KEY_U) == GLFW_PRESS) rx_val -= 1.0;
        if (glfwGetKey(window_, GLFW_KEY_O) == GLFW_PRESS) rx_val += 1.0;
        
        // Normalize diagonal movement
        if (rx_val != 0.0 && ry_val != 0.0) {
            double norm = std::sqrt(rx_val * rx_val + ry_val * ry_val);
            rx_val /= norm;
            ry_val /= norm;
        }
        
        rx(rx_val);
        ry(ry_val);
    }

private:
    GLFWwindow* window_;
};
