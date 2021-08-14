#include "utils.h"
#include "GL/glwrapper.h"
#include "ImGuizmo/ImGuizmo.h"

#include <Eigen/Dense>
#include <boost/program_options.hpp>
#include <thread>

glm::vec2 clicked_point;
float rot_x =0.0f;
float rot_y =0.0f;
bool drawing_buffer_dirty = true;
glm::vec3 view_translation{ 0,0,-30 };

void cursor_calback(GLFWwindow* window, double xpos, double ypos)
{
    ImGuiIO& io = ImGui::GetIO();
    if(!io.WantCaptureMouse) {
        const glm::vec2 p{-xpos, ypos};
        const auto d = clicked_point - p;
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS) {
            rot_x += 0.01 * d[1];
            rot_y += 0.01 * d[0];
        }
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_2) == GLFW_PRESS) {
            view_translation[2] += 0.02 * d[1];
        }
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_3) == GLFW_PRESS) {
            view_translation[1] += 0.01 * d[1];
            view_translation[0] -= 0.01 * d[0];
        }
        clicked_point = p;
    }
}
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}


int main(int argc, char **argv) {

    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "produce help message")
            ("mirror_config", po::value<std::string>()->required(), "path to mirror mesh PLY.");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return false;
    }


    bool gl_dirty = false;

    // Loading mirrrors initial params
    std::vector<catoptric_livox::Mirror> mirrors = catoptric_livox::loadMirrorFromPLY(vm["mirror_config"].as<std::string>());

    std::vector<std::pair<double,double>> angle_mask;

    GLFWwindow *window;
    const char *glsl_version = "#version 130";
    if (!glfwInit())
        return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    window = glfwCreateWindow(960, 540, "Simulation", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetCursorPosCallback(window, cursor_calback);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    glfwSwapInterval(1);
    if (glewInit() != GLEW_OK) { return -1; }

    GLCall(glClearColor(1.0, 1.0, 1.0, 1));

    Renderer renderer;

    Shader shader(shader_simple_v, shader_simple_f);

    VertexBufferLayout layout;
    layout.Push<float>(3);
    layout.Push<float>(3);

    VertexArray va_co;
    VertexBuffer vb_co(gl_primitives::coordinate_system_vertex.data(),
                       gl_primitives::coordinate_system_vertex.size() * sizeof(float));
    va_co.AddBuffer(vb_co, layout);
    IndexBuffer ib_co(gl_primitives::coordinate_system_indices.data(), gl_primitives::coordinate_system_indices.size());

    std::vector<float> mirrors_vertices;
    std::vector<unsigned int> mirrors_indices;
    catoptric_livox::updateDrawingBuffer(mirrors, mirrors_vertices, mirrors_indices);
    VertexArray va_mirror;
    VertexBuffer vb_mirror(mirrors_vertices.data(),
                           mirrors_vertices.size() * sizeof(float));
    va_mirror.AddBuffer(vb_mirror, layout);
    IndexBuffer ib_mirror(mirrors_indices.data(), mirrors_indices.size());

    std::vector<float> ray_vertices;
    std::vector<unsigned int> ray_indicies;
    float all_points = 0;
    float classified = 0;

    for (float i = -38.4/2; i <0; i+=1)
    {
        for (float j = 0;j < 360; j+=0.25) {
            Eigen::Vector3f  color{0,0,0};
            Eigen::Affine3d mat1(Eigen::Matrix4d::Identity());
            Eigen::Affine3d mat2(Eigen::Matrix4d::Identity());

            mat1.rotate(Eigen::AngleAxisd(M_PI * i / 180.0, Eigen::Vector3d::UnitZ()));
            mat2.rotate(Eigen::AngleAxisd(M_PI * j / 180.0, Eigen::Vector3d::UnitX()));

            Eigen::Affine3d mat3 = mat1 * mat2;


            Eigen::Vector3d p_local{10*mat3(0, 0), 10*mat3(0, 1), 10*mat3(0, 2)};
            double p_local_l = p_local.norm();
            Eigen::Vector3d p_local_norm = p_local / p_local_l;

            int mirror_id = -1;
            for (int m = 0; m < mirrors.size();m++)
            {
                if (mirrors[m].checkIfRayIntersectMirror(Eigen::Vector3d::Zero(),p_local)) {
                    mirror_id = m;
                }
            }
            all_points++;
            if (mirror_id==-1) {continue;}

            classified++;
            color = catoptric_livox::mirror_colors[mirror_id];
            color = color *0.5;
            ray_vertices.push_back(0);
            ray_vertices.push_back(0);
            ray_vertices.push_back(0);
            ray_vertices.push_back(color.x());
            ray_vertices.push_back(color.y());
            ray_vertices.push_back(color.z());

            const Eigen::Vector4d plane = mirrors[mirror_id].getABCDofPlane();

            Eigen::Vector3d p_glob = catoptric_livox::getMirroredRayIntersection(p_local_norm, p_local_l,plane);

            ray_vertices.push_back(p_glob.x());
            ray_vertices.push_back(p_glob.y());
            ray_vertices.push_back(p_glob.z());
            ray_vertices.push_back(color.x());
            ray_vertices.push_back(color.y());
            ray_vertices.push_back(color.z());

            ray_indicies.push_back(ray_indicies.size());
            ray_indicies.push_back(ray_indicies.size());

            ray_vertices.push_back(p_glob.x());
            ray_vertices.push_back(p_glob.y());
            ray_vertices.push_back(p_glob.z());
            ray_vertices.push_back(color.x());
            ray_vertices.push_back(color.y());
            ray_vertices.push_back(color.z());

            Eigen::Vector3d p_glob2 = catoptric_livox::getMirroredRay(p_local_norm, p_local_l,plane);

            ray_vertices.push_back(p_glob2.x());
            ray_vertices.push_back(p_glob2.y());
            ray_vertices.push_back(p_glob2.z());
            ray_vertices.push_back(color.x());
            ray_vertices.push_back(color.y());
            ray_vertices.push_back(color.z());


            ray_indicies.push_back(ray_indicies.size());
            ray_indicies.push_back(ray_indicies.size());
        }
    }
    std::cout << " classified to all points "  << classified/all_points <<std::endl;

    VertexArray va_ray;
    VertexBuffer vb_ray(ray_vertices.data(),
                               ray_vertices.size() * sizeof(float));
    va_ray.AddBuffer(vb_ray, layout);
    IndexBuffer ib_ray(ray_indicies.data(), ray_indicies.size());

    auto mat = Eigen::Affine3d::Identity();
    mat.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()));

    std::vector<float> nns_vertices;
    std::vector<unsigned int> nns_indices;
    VertexArray va_nns;
    VertexBuffer vb_nns(nns_vertices.data(),
                        nns_vertices.size() * sizeof(float));
    va_nns.AddBuffer(vb_nns, layout);
    IndexBuffer ib_nns(nns_indices.data(), nns_indices.size());

    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, false);
    ImGui_ImplOpenGL3_Init(glsl_version);
    glm::mat4 glm_test {glm::mat4(1.0f)};

    while (!glfwWindowShouldClose(window)) {

        GLCall(glEnable(GL_DEPTH_TEST));
        GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuizmo::BeginFrame();
        ImGuizmo::Enable(true);
        ImGuiIO &io = ImGui::GetIO();
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

        ImGuizmo::Enable(true);
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        glm::mat4 proj = glm::perspective(30.f, 1.0f * width / height, 0.05f, 100.0f);
        glm::mat4 model_translate = glm::translate(glm::mat4(1.0f), view_translation);
        glm::mat4 model_rotation_0 = glm::rotate(model_translate, float(-0.5 * M_PI), glm::vec3(0.0f, 0.0f, 1.0f));

        glm::mat4 model_rotation_1 = glm::rotate(model_rotation_0, rot_x, glm::vec3(0.0f, 1.0f, 0.0f));
        glm::mat4 model_rotation_2 = glm::rotate(model_rotation_1, rot_y, glm::vec3(1.0f, 0.0f, 0.0f));

        shader.Bind(); // bind shader to apply uniform
        // draw reference frame
        GLCall(glPointSize(1));
        GLCall(glLineWidth(1));

        shader.setUniformMat4f("u_MVP", proj * model_rotation_2);
        renderer.Draw(va_ray, ib_ray, shader, GL_LINES);
        renderer.Draw(va_co, ib_co, shader, GL_LINES);
        GLCall(glLineWidth(3));
        renderer.Draw(va_mirror, ib_mirror, shader, GL_LINES);
        GLCall(glLineWidth(1));
        renderer.Draw(va_nns, ib_nns, shader, GL_LINES);

        for (int i = 0; i < mirrors.size(); i++) {
            glm::mat4 tr = glm::mat4(1.0f);
            glm::mat4 sc = glm::scale(glm::mat4(1.0f), glm::vec3(0.01f));
            Eigen::Map<Eigen::Matrix4f> tr_e(&tr[0][0]);
            tr_e = mirrors[i].getTransformation().matrix().cast<float>();
            shader.setUniformMat4f("u_MVP", proj * model_rotation_2 * tr * sc);
            renderer.Draw(va_co, ib_co, shader, GL_LINES);
        }

        ImGui::Begin("Calibration Demo");
        ImGui::End();
        ImGui::Render();

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwTerminate();
    return 0;
}