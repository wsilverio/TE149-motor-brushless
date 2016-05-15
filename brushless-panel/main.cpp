#include <imgui.h>
#include "imgui_impl_sdl.h"
#include <stdio.h>
#include <SDL.h>
#include <SDL_opengl.h>

#define IM_ARRAYSIZE(_ARR)  ((int)(sizeof(_ARR)/sizeof(*_ARR)))

float MinMaxElement(const float* mvec, const int len, float& min, float& max) {
    min = max = mvec[0];

    for (unsigned int i = 1; i < len; ++i) {
        if (mvec[i] < min) min = mvec[i];
        else if (mvec[i] > max) max = mvec[i];
    }
}

int main(int argc, char const *argv[]){
    // Setup SDL
    if (SDL_Init(SDL_INIT_VIDEO|SDL_INIT_TIMER) != 0){
        printf("Error: %s\n", SDL_GetError());
        return -1;
    }

    // Setup window
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    SDL_DisplayMode current;
    SDL_GetCurrentDisplayMode(0, &current);
    SDL_Window *window = SDL_CreateWindow("Brushless Panel", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1024, 500, SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE);
    SDL_GLContext glcontext = SDL_GL_CreateContext(window);

    // Setup ImGui binding
    ImGui_ImplSdl_Init(window);

    ImVec4 clear_color = ImColor(58, 58, 58);

    // Main loop
    bool done = false;
    while (!done){
        SDL_Event event;
        while (SDL_PollEvent(&event)){
            ImGui_ImplSdl_ProcessEvent(&event);
            if (event.type == SDL_QUIT)
                done = true;
        }

        ImGui_ImplSdl_NewFrame(window);

        // non 'static' window
        bool plot_window = true;
        bool serial_window = true;
        bool control_window = true;

        if (plot_window){
            float values[90] = {666};
            static int values_offset = 0;

            float min, max;
            MinMaxElement(values, IM_ARRAYSIZE(values), min, max);

            ImGui::SetNextWindowSize(ImVec2(700, 450), ImGuiSetCond_FirstUseEver);
            ImGui::Begin("Plot Window", &plot_window);
            
            ImGui::PushStyleColor(ImGuiCol_PlotLines, ImVec4(0.90f, 0.70f, 0.00f, 1.00f));
            ImGui::PlotLines("##rpm", values, IM_ARRAYSIZE(values), values_offset, "rpm", min, max, ImVec2(0.97f*ImGui::GetWindowWidth(),0.9f*ImGui::GetWindowHeight()));
            ImGui::PopStyleColor();
            
            ImGui::End();
        }

        static bool serial_opened = false;
        if (serial_window){
            static char serial_name[128] = "/dev/ttyACM";

            static bool serial_opened_last = serial_opened;
            static int bps = 11;
            const int serial_bps[] = {300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200};
            const char* serial_bps_str[] = {"300", "600", "1200", "2400", "4800", "9600", "14400", "19200", "28800", "38400", "57600", "115200"};

            ImGui::SetNextWindowSize(ImVec2(700, 450), ImGuiSetCond_FirstUseEver);
            ImGui::Begin("Serial", &serial_window);

            ImGui::Text("serial port:");
            ImGui::InputText("##serial port", serial_name, IM_ARRAYSIZE(serial_name));
            ImGui::Text("baudrate:");
            ImGui::Combo("##baudrate", &bps, serial_bps_str, IM_ARRAYSIZE(serial_bps_str));
            ImGui::Checkbox("open", &serial_opened);

            // verifica alteracao no toggle serial
            bool serial_changed = false;
            if(serial_opened_last != serial_opened){
                serial_changed = true;
                serial_opened_last = serial_opened;
            }

            if (serial_changed){
                if(serial_opened){
                    // printf("conectado\n");
                }else{
                    // printf("desconectado\n");
                }
            }

            ImGui::End();
        }

        if (control_window){
            static int pulse_width = 1000;
            ImGui::SetNextWindowSize(ImVec2(700, 450), ImGuiSetCond_FirstUseEver);
            ImGui::Begin("Control", &control_window);
            ImGui::Text("pulse width:");
            ImGui::SliderInt("##pulse width", &pulse_width, 1000, 2000);
            ImGui::SameLine();
            if(ImGui::Button("set") && serial_opened){
                // printf("send pulse\n");
            }

            // ImGui::Separator();
            ImGui::Spacing();
            ImGui::Spacing();
            ImGui::Spacing();

            ImGui::Text("SNR:"); ImGui::SameLine(); ImGui::TextColored(ImVec4(1.0f,1.0f,0.0f,1.0f), "%d", 123);

            ImGui::End();
        }

        // Rendering
        glViewport(0, 0, (int)ImGui::GetIO().DisplaySize.x, (int)ImGui::GetIO().DisplaySize.y);
        glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui::Render();
        SDL_GL_SwapWindow(window);
    }

    // Cleanup
    ImGui_ImplSdl_Shutdown();
    SDL_GL_DeleteContext(glcontext);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
