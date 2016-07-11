#include <stdint.h>
#include <vector>
#include <pthread.h>
#include <signal.h>
#include "serial_port.h"
#include <imgui.h>
#include "imgui_impl_sdl.h"
#include <stdio.h>
#include <SDL.h>
#include <SDL_opengl.h>
#include <string>

#define IM_ARRAYSIZE(_ARR)((int)(sizeof(_ARR)/sizeof(*_ARR)))

#define VECTOR_LEN 512

void* start_brushless_interface_read_thread(void *args);
void* start_brushless_interface_write_thread(void *args);

class BrushlessSerial{
public:

    BrushlessSerial(Serial_Port *serial_port_){
        serial_values.reserve(VECTOR_LEN);
        dataReset();
        serial_port = serial_port_;
        time_to_exit   = false;
    }

    ~BrushlessSerial(){

    }

    void read_messages(){
        return;
    }
    int write_message(int msg){
        // do the write
        int len = serial_port->write_message(std::to_string(msg));
        // Done!
        return len;
    }

    void start(){
        if(not serial_port->status == 1){ // SERIAL_PORT_OPEN{
            fprintf(stderr,"ERROR: serial port not open\n");
            throw 1;
        }
        int result;
        result = pthread_create( &read_tid, NULL, &start_brushless_interface_read_thread, this);
        if (result) throw result;
        result = pthread_create( &write_tid, NULL, &start_brushless_interface_write_thread, this);
        if (result) throw result;
        return;
    }

    void stop(){
        printf("CLOSE THREADS\n");
        // signal exit
        time_to_exit = true;
        // wait for exit
        pthread_join(read_tid , NULL);
        pthread_join(write_tid, NULL);
        // now the read and write threads are closed
        printf("\n");
        // still need to close the serial_port separately
    }

    void start_read_thread(){
        if ( reading_status != 0 ){
            fprintf(stderr,"read thread already running\n");
            return;
        }else{
            read_thread();
            return;
        }
    }

    void start_write_thread(){
        if ( not writing_status == false ){
            fprintf(stderr,"write thread already running\n");
            return;
        }else{
            write_thread();
            return;
        }
    }

    void handle_quit(){
        try {
            stop();
        }
        catch (int error) {
            fprintf(stderr,"Warning, could not stop autopilot interface\n");
        }
    }

    void dataReset(){
        index = 0;
        std::fill(serial_values.begin(), serial_values.end(), 0);
    }
    
    uint16_t index;
    std::vector<int16_t> serial_values;

private:
    void read_thread(){
        reading_status = true;
        while( not time_to_exit ){
            read_messages();
            // usleep(100000); // Read batches at 10Hz
        }
        reading_status = false;
        return;
    }

    void write_thread(){
        return;
    }

    Serial_Port *serial_port;

    char reading_status;
    char writing_status;
    bool time_to_exit;
    
    pthread_t read_tid;
    pthread_t write_tid;
    pthread_mutex_t lock;
};

void* start_brushless_interface_read_thread(void *args){
    BrushlessSerial *brushless_interface = (BrushlessSerial *)args;
    brushless_interface->start_read_thread();
    return NULL;
}

void* start_brushless_interface_write_thread(void *args){
    BrushlessSerial *brushless_interface = (BrushlessSerial *)args;
    brushless_interface->start_write_thread();
    return NULL;
}

struct ExampleAppLog{
    ImGuiTextBuffer Buf;
    ImGuiTextFilter Filter;
    ImVector<int> LineOffsets; // Index to lines offset
    bool ScrollToBottom;

    void Clear() { Buf.clear(); LineOffsets.clear(); }

    void AddLog(const char* fmt, ...) IM_PRINTFARGS(2){
        int old_size = Buf.size();
        va_list args;
        va_start(args, fmt);
        Buf.appendv(fmt, args);
        va_end(args);
        for (int new_size = Buf.size(); old_size < new_size; old_size++)
            if (Buf[old_size] == '\n')
                LineOffsets.push_back(old_size);
        ScrollToBottom = true;
    }

    void Draw(const char* title){
        ImGui::Begin(title);
        if(ImGui::Button("clear")){
            Clear();
        }
        ImGui::SameLine();
        bool copy = ImGui::Button("copy");
        ImGui::Separator();
        ImGui::BeginChild("scrolling", ImVec2(0,0), false, ImGuiWindowFlags_HorizontalScrollbar);
        
        if(copy){
            ImGui::LogToClipboard();
        }

        ImGui::TextUnformatted(Buf.begin());

        if(ScrollToBottom){
            ImGui::SetScrollHere(1.0f);
        }

        ScrollToBottom = false;

        ImGui::EndChild();
        ImGui::End();
    }
};

static void ShowExampleAppLog()
{
    static ExampleAppLog log;

    // Demo fill
    static float last_time = -1.0f;
    float time = ImGui::GetTime();
    if (time - last_time >= 0.3f){
        const char* random_words[] = { "system", "info", "warning", "error", "fatal", "notice", "log" };
        log.AddLog("[%s] Hello, time is %.1f, rand() %d\n", random_words[rand() % IM_ARRAYSIZE(random_words)], time, (int)rand());
        last_time = time;
    }

    log.Draw("Log");
}

int main(int argc, char const *argv[]){
    Serial_Port *serial_port;
    BrushlessSerial b_serial(serial_port);

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
    SDL_Window *window = SDL_CreateWindow("Brushless Panel", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1250, 660, SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE);
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

            int values_offset = 0;
            std::vector<float> serial_values(VECTOR_LEN, 0);
            
            ImGui::SetNextWindowSize(ImVec2(700, 450), ImGuiSetCond_FirstUseEver);
            ImGui::Begin("Plot Window", &plot_window);
            
            ImGui::PushStyleColor(ImGuiCol_PlotLines, ImVec4(0.90f, 0.70f, 0.00f, 1.00f));
            ImGui::PlotLines("##rpm", &serial_values[0], VECTOR_LEN, values_offset, "rpm", 1200, 6600, ImVec2(0.97f*ImGui::GetWindowWidth(),0.9f*ImGui::GetWindowHeight()));
            ImGui::PopStyleColor();
            
            ImGui::End();
        }

        static bool serial_opened = false;
        if (serial_window){
            static char serial_name[128] = "/dev/ttyUSB0";

            static bool serial_opened_last = serial_opened;
            static int bps = 5;
            const int serial_bps[] = {9600, 19200, 38400, 57600, 115200, 460800};
            const char* serial_bps_str[] = {"9600", "19200", "38400", "57600", "115200", "460800"};

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
                    b_serial.start();
                }else{
                    serial_port->handle_quit();
                }
            }

            ImGui::End();
        }

        if (control_window){
            static int setRPM = 1000;
            ImGui::SetNextWindowSize(ImVec2(700, 450), ImGuiSetCond_FirstUseEver);
            ImGui::Begin("Control", &control_window);
            ImGui::Text("rpm:");
            ImGui::SliderInt("##rpm", &setRPM, 2000, 6000);
            ImGui::SameLine();
            if(ImGui::Button("set") && serial_opened){
                // printf("send pulse\n");
            }

            // ImGui::Separator();
            ImGui::Spacing();
            ImGui::Spacing();
            ImGui::Spacing();

            // ImGui::Text("SNR:"); ImGui::SameLine(); ImGui::TextColored(ImVec4(1.0f,1.0f,0.0f,1.0f), "%d", 123);

            ShowExampleAppLog();

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

    try {
        b_serial.handle_quit();
    }
    catch (int error){}

    try {
        serial_port->handle_quit();
    }
    catch (int error){}
    delete serial_port;



    return 0;
}
