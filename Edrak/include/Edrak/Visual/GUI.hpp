#ifndef EDRAK_INCLUDE_EDRAK_VISUAL_GUI
#define EDRAK_INCLUDE_EDRAK_VISUAL_GUI
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <math.h> // sqrtf, powf, cosf, sinf, floorf, ceilf

#include <stdio.h>
#include <string>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#if defined(_MSC_VER) && (_MSC_VER >= 1900) &&                                 \
    !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

static void glfw_error_callback(int error, const char *description) {
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

namespace Edrak {
namespace GUI {
/**
 * @brief Inherit from this calls to create a custom window
 * Override the run function to render your window.
 *
 */
class Window {
private:
  GLFWwindow *window;
  bool success = false;

public:
  Window(int width = 1280, int height = 720,
         const std::string &title = "Edrak") {
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
      return;

      // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    const char *glsl_version = "#version 100";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    // GL 3.2 + GLSL 150
    const char *glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);           // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char *glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+
    // only glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // 3.0+ only
#endif

    window = glfwCreateWindow(width, height, title.c_str(), NULL, NULL);
    if (window == NULL) {
      success = false;
      return;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable
    // Keyboard Controls io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad; //
    // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    // ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
  }
  virtual void Run() {

    // Our state
    bool show_another_window = true;
    bool edrak_settings_active = true;
    bool show_pointcloud = false;
    ImVec4 clear_color = ImVec4(0.073f, 0.000f, 0.137f, 1.000f);

    // Main loop
    while (!glfwWindowShouldClose(window)) {
      // Poll and handle events (inputs, window resize, etc.)
      // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to
      // tell if dear imgui wants to use your inputs.
      // - When io.WantCaptureMouse is true, do not dispatch mouse input data to
      // your main application, or clear/overwrite your copy of the mouse data.
      // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input
      // data to your main application, or clear/overwrite your copy of the
      // keyboard data. Generally you may always pass all inputs to dear imgui,
      // and hide them from your application based on those two flags.
      glfwPollEvents();

      // Start the Dear ImGui frame
      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();

      // 2. Show a simple window that we create ourselves. We use a Begin/End
      // pair to created a named window.
      {
        static float f = 0.0f;
        static int counter = 0;

        ImGui::Begin("Edrak Settings",
                     &edrak_settings_active); // Create a window called "Hello,
                                              // world!" and append into it.

        ImGui::Text("Welcome To Edrak ToolBox."); // Display some text (you can
                                                  // use a format strings too)

        // open/close state
        ImGui::Checkbox("PointCloud", &show_pointcloud);

        ImGui::SliderFloat(
            "float", &f, 0.0f,
            1.0f); // Edit 1 float using a slider from 0.0f to 1.0f
        ImGui::ColorEdit3(
            "clear color",
            (float *)&clear_color); // Edit 3 floats representing a color

        if (ImGui::Button(
                "Button")) // Buttons return true when clicked (most widgets
                           // return true when edited/activated)
          counter++;
        ImGui::SameLine();
        ImGui::Text("counter = %d", counter);

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                    1000.0f / ImGui::GetIO().Framerate,
                    ImGui::GetIO().Framerate);
        ImGui::End();
      }

      if (show_pointcloud) {

        ImGui::Begin("PointCloud", &show_pointcloud);
        static ImVector<ImVec2> points;
        static ImVec2 scrolling(0.0f, 0.0f);
        static bool opt_enable_grid = true;
        static bool opt_enable_context_menu = true;
        static bool adding_line = false;

        ImGui::Checkbox("Enable grid", &opt_enable_grid);
        ImGui::Checkbox("Enable context menu", &opt_enable_context_menu);
        ImGui::Text("Mouse Left: drag to add lines,\nMouse Right: drag to "
                    "scroll, click for context menu.");

        // Typically you would use a BeginChild()/EndChild() pair to benefit
        // from a clipping region + own scrolling. Here we demonstrate that this
        // can be replaced by simple offsetting + custom drawing +
        // PushClipRect/PopClipRect() calls. To use a child window instead we
        // could use, e.g:
        //      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        //      // Disable padding ImGui::PushStyleColor(ImGuiCol_ChildBg,
        //      IM_COL32(50, 50, 50, 255));  // Set a background color
        //      ImGui::BeginChild("canvas", ImVec2(0.0f, 0.0f), true,
        //      ImGuiWindowFlags_NoMove); ImGui::PopStyleColor();
        //      ImGui::PopStyleVar();
        //      [...]
        //      ImGui::EndChild();

        // Using InvisibleButton() as a convenience 1) it will advance the
        // layout cursor and 2) allows us to use IsItemHovered()/IsItemActive()
        ImVec2 canvas_p0 = ImGui::GetCursorScreenPos(); // ImDrawList API uses
                                                        // screen coordinates!
        ImVec2 canvas_sz =
            ImGui::GetContentRegionAvail(); // Resize canvas to what's available
        if (canvas_sz.x < 50.0f)
          canvas_sz.x = 50.0f;
        if (canvas_sz.y < 50.0f)
          canvas_sz.y = 50.0f;
        ImVec2 canvas_p1 =
            ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

        // Draw border and background color
        ImGuiIO &io = ImGui::GetIO();
        ImDrawList *draw_list = ImGui::GetWindowDrawList();
        draw_list->AddRectFilled(canvas_p0, canvas_p1,
                                 IM_COL32(50, 50, 50, 255));
        draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(255, 255, 255, 255));

        // This will catch our interactions
        ImGui::InvisibleButton("canvas", canvas_sz,
                               ImGuiButtonFlags_MouseButtonLeft |
                                   ImGuiButtonFlags_MouseButtonRight);
        const bool is_hovered = ImGui::IsItemHovered(); // Hovered
        const bool is_active = ImGui::IsItemActive();   // Held
        const ImVec2 origin(canvas_p0.x + scrolling.x,
                            canvas_p0.y + scrolling.y); // Lock scrolled origin
        const ImVec2 mouse_pos_in_canvas(io.MousePos.x - origin.x,
                                         io.MousePos.y - origin.y);

        // Add first and second point
        if (is_hovered && !adding_line &&
            ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
          points.push_back(mouse_pos_in_canvas);
          points.push_back(mouse_pos_in_canvas);
          adding_line = true;
        }
        if (adding_line) {
          points.back() = mouse_pos_in_canvas;
          if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
            adding_line = false;
        }

        // Pan (we use a zero mouse threshold when there's no context menu)
        // You may decide to make that threshold dynamic based on whether the
        // mouse is hovering something etc.
        const float mouse_threshold_for_pan =
            opt_enable_context_menu ? -1.0f : 0.0f;
        if (is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Right,
                                                mouse_threshold_for_pan)) {
          scrolling.x += io.MouseDelta.x;
          scrolling.y += io.MouseDelta.y;
        }

        // Context menu (under default mouse threshold)
        ImVec2 drag_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
        if (opt_enable_context_menu && drag_delta.x == 0.0f &&
            drag_delta.y == 0.0f)
          ImGui::OpenPopupOnItemClick("context",
                                      ImGuiPopupFlags_MouseButtonRight);
        if (ImGui::BeginPopup("context")) {
          if (adding_line)
            points.resize(points.size() - 2);
          adding_line = false;
          if (ImGui::MenuItem("Remove one", NULL, false, points.Size > 0)) {
            points.resize(points.size() - 2);
          }
          if (ImGui::MenuItem("Remove all", NULL, false, points.Size > 0)) {
            points.clear();
          }
          ImGui::EndPopup();
        }

        // Draw grid + all lines in the canvas
        draw_list->PushClipRect(canvas_p0, canvas_p1, true);
        if (opt_enable_grid) {
          const float GRID_STEP = 64.0f;
          for (float x = fmodf(scrolling.x, GRID_STEP); x < canvas_sz.x;
               x += GRID_STEP)
            draw_list->AddLine(ImVec2(canvas_p0.x + x, canvas_p0.y),
                               ImVec2(canvas_p0.x + x, canvas_p1.y),
                               IM_COL32(200, 200, 200, 40));
          for (float y = fmodf(scrolling.y, GRID_STEP); y < canvas_sz.y;
               y += GRID_STEP)
            draw_list->AddLine(ImVec2(canvas_p0.x, canvas_p0.y + y),
                               ImVec2(canvas_p1.x, canvas_p0.y + y),
                               IM_COL32(200, 200, 200, 40));
        }
        for (int n = 0; n < points.Size; n += 2)
          draw_list->AddLine(
              ImVec2(origin.x + points[n].x, origin.y + points[n].y),
              ImVec2(origin.x + points[n + 1].x, origin.y + points[n + 1].y),
              IM_COL32(255, 255, 0, 255), 2.0f);
        draw_list->PopClipRect();
        draw_list->AddCircle(ImVec2(canvas_p0.x + 20, canvas_p0.y + 20), 10,
                             IM_COL32(255, 255, 0, 255));
        ImGui::End();
      }

      // Rendering
      ImGui::Render();
      int display_w, display_h;
      glfwGetFramebufferSize(window, &display_w, &display_h);
      glViewport(0, 0, display_w, display_h);
      glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w,
                   clear_color.z * clear_color.w, clear_color.w);
      glClear(GL_COLOR_BUFFER_BIT);
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

      glfwSwapBuffers(window);
    }
  }
  virtual ~Window() {
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
  }
};

} // namespace GUI

} // namespace Edrak

#endif /* EDRAK_INCLUDE_EDRAK_VISUAL_GUI */
