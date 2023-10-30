#ifndef __CFORGE_DIALOG_HPP__
#define __CFORGE_DIALOG_HPP__

#include "ExampleSceneBase.hpp"
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_glfw.h>
#include <GLFW/glfw3.h>
#include "DialogGraph.hpp"

using namespace std;

namespace CForge {

	class Dialog : public ExampleSceneBase {
	public:
        ImVec2 size;
        ImVec2 pos;
        ImVec2 pivot;
        ImGuiWindowFlags windowFlags;
        bool hasFinished;
        Dialoggraph dialogGraph;
        ImGuiContext* ctx;
        ImGuiStyle style;
        vector<int> conversationProgress;
        Dialoggraph currentDialog;
        string currentFile;

        void init() {
            IMGUI_CHECKVERSION();
            ctx = ImGui::CreateContext();

            ImGuiIO& io = ImGui::GetIO();
            (void)io;

            io.Fonts->AddFontFromFileTTF(
                "Assets/Fonts/NotoSerif/NotoSerif-Regular.ttf",
                24.0f,
                NULL,
                NULL
            );

            // setup platform/renderer bindings
            if (!ImGui_ImplGlfw_InitForOpenGL(glfwGetCurrentContext(), true)) {
                std::cout << "Failed to init imGUI for window" << std::endl;
            }
            if (!ImGui_ImplOpenGL3_Init()) {
                std::cout << "Failed to init imGUI for OpenGL" << std::endl;
            }

            //dialogGraph.init("Assets/Dialogs/conversation.json");
        }

		void setWindowStyle(float width, float height) {
            ImGui::SetCurrentContext(ctx);
            size = { width / 3.0f, height / 5.0f };
            pos = { width / 2.0f, height * 4.0f / 5.0f };
            pivot = { 0.5, 0.5 };
            windowFlags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize;
            //style = ImGui::GetStyle();
            style.FrameRounding = 2.0f;
            style.WindowRounding = 3.0f;
            style.WindowPadding = { 4.0f, 10.0f };
            style.ItemSpacing = { 5.0f, 7.0f };
            ImGui::StyleColorsLight();
            style.Colors[ImGuiCol_WindowBg] = { 255.0f, 255.0f, 255.0f, 0.8f };
		};

        bool showDialog(string filepath) {
            ImGui::SetCurrentContext(ctx);
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();

            if (filepath != currentFile)
            {
                dialogGraph.init(filepath);
                currentFile = filepath;
            }
            
            bool test = true;
            hasFinished = false;
            
            currentDialog = dialogGraph;
            for (int selected : conversationProgress) {
                currentDialog = currentDialog.answers[selected];
                if (currentDialog.playerSpeaking && !currentDialog.answers.empty()) {
                    currentDialog = currentDialog.answers[0];
                }
            }
            ImGui::NewFrame();
            ImGui::SetNextWindowSize(size);
            ImGui::SetNextWindowPos(pos, 0, pivot);
            ImGui::Begin("test", &test, windowFlags);
            float win_width = ImGui::GetWindowSize().x;
            float text_width = ImGui::CalcTextSize(currentDialog.text.c_str()).x;
            ImGui::SetCursorPosX((win_width - text_width) * 0.5f);
            ImGui::Text(currentDialog.text.c_str());
            for (int i = 0; i < currentDialog.answers.size(); i++) {
                float button_width = ImGui::CalcTextSize(currentDialog.answers[i].text.c_str()).x;
                ImGui::SetCursorPosX((win_width - button_width) * 0.5f);
                if (ImGui::Button(currentDialog.answers[i].text.c_str())) {
                    conversationProgress.push_back(i);
                }
            }
            if (currentDialog.answers.empty()) {
                hasFinished = true;
                conversationProgress.clear();
            }
            ImGui::End();
            ImGui::EndFrame();
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            return hasFinished;
        }
	};
}
#endif