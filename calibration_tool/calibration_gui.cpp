#include <GL/glwrapper.h>

#include "utils.h"
#include "utils_io.h"
#include "cost_fun.h"

#include "ImGuizmo/ImGuizmo.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <boost/program_options.hpp>

#include <Eigen/Dense>
#include <tbb/tbb.h>
#include <thread>

namespace clv=catoptric_livox;

glm::vec2 clicked_point;
float rot_x =0.0f;
float rot_y =0.0f;
bool drawing_buffer_dirty = true;
glm::vec3 view_translation{ 0,0,-30 };

struct found_nn{
    int scan_site1_id;
    int scan_site2_id;
    int scan1_id;
    int scan2_id;
    int scan1_point_id;
    int scan2_point_id;
    Eigen::Vector3d p1;
    Eigen::Vector3d p2;
    Eigen::Vector3d global_p1;
    Eigen::Vector3d global_p2;
};

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

std::vector<found_nn> findNNAgainstGT(const  std::vector<clv::scan_site> scans_sites,
                             const std::vector<catoptric_livox::Mirror> & mirrors,
                             const Sophus::SE3d& intruments_lever_arm,
                             const Sophus::SE3d& global_pose,
                             const pcl::KdTreeFLANN<pcl::PointXYZRGB>& groundtruth_kdtree,
                             const pcl::PointCloud<pcl::PointXYZRGB>& groundtruth_cloud,
                             const std::vector<std::pair<double,double>>& angle_mask,
                             float radius){
    if (groundtruth_cloud.empty()) return  std::vector<found_nn>();
    using PoinType=pcl::PointXYZL;
    struct cloudWithIds{
        pcl::PointCloud<PoinType>::Ptr cloud;
        int scan_id;
        int pose_id;
    };
    std::vector<cloudWithIds> clouds;
    clouds.reserve(scans_sites.size());
    for (int i =0; i < scans_sites.size(); i++)
    {
        for (int j=0; j < scans_sites[i].datastreams.size(); j++ ) {
            const auto &scan1 = scans_sites[i].datastreams[j];
            const auto &instrument_pose1 = scans_sites[i].intruments_poses_SE3.at(j);
            const auto &scan_site_pose1 = scans_sites[i].scan_site_pose_SE3;
            auto cloud_ptr = getPoinCloud(scan1, mirrors, instrument_pose1, intruments_lever_arm, global_pose, angle_mask,scan_site_pose1).makeShared();
            cloudWithIds cl{cloud_ptr};
            cl.scan_id = j;
            cl.pose_id = i;
            clouds.push_back(cl);
        }
    }
    std::vector<found_nn> total_nns;
    tbb::mutex mtx;
    tbb::parallel_for(
            tbb::blocked_range<size_t>(0,clouds.size()),
            [&](const tbb::blocked_range<size_t>& r) {
                std::vector<found_nn> local_nns;
                for (long i=r.begin();i<r.end();++i)
                {
                    const auto &p = clouds[i];
                    const auto &pointcloud1 = p.cloud;
                    int cloud1_id = p.scan_id;
                    int pose1_id = p.pose_id;

                    for (int cloud1_point_id = 0; cloud1_point_id < pointcloud1->size(); cloud1_point_id+=5) {
                        pcl::PointXYZRGB p1;
                        p1.getArray3fMap() = (*pointcloud1)[cloud1_point_id].getArray3fMap();
                        std::vector<int> pointIdxRadiusSearch;
                        std::vector<float> pointRadiusSquaredDistance;
                        if (groundtruth_kdtree.radiusSearch(p1, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                            for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
                                const auto p2 = groundtruth_cloud.at(pointIdxRadiusSearch[i]);
                                const int cloud1_point_id_org = (*pointcloud1)[cloud1_point_id].label;
                                local_nns.emplace_back(found_nn{pose1_id,0,cloud1_id, 0, cloud1_point_id_org, pointIdxRadiusSearch[i],
                                                                p1.getArray3fMap().cast<double>(),
                                                                p2.getArray3fMap().cast<double>()});

                                Eigen::Vector3f d = p1.getArray3fMap() - p2.getArray3fMap();
                                if (d.norm() > 1.0) {
                                    std::cerr << "error !!!" << d << std::endl;
                                }
                                break;
                            }
                        }
                    }
                }
                tbb::mutex::scoped_lock lck (mtx);
                total_nns.reserve(total_nns.size()+local_nns.size());
                std::move(std::begin(local_nns), std::end(local_nns), std::back_inserter(total_nns));
            });

    return total_nns;
}

std::vector<found_nn> findNN(const  std::vector<clv::scan_site> scans_sites,
                             const std::vector<catoptric_livox::Mirror> & mirrors,
                             const Sophus::SE3d& intruments_lever_arm,
                             const std::vector<std::pair<double,double>>& angle_mask,
                             float radius,
                             bool allow_nn_between_sites,
                             bool allow_nn_inside_sites){

    using PoinType=pcl::PointXYZL;
    if (!allow_nn_between_sites && !allow_nn_inside_sites){
        return std::vector<found_nn>();
    }
    // build kdtrees
    struct cloudWithKdtree{
        int scan_id;
        int pose_id;
        pcl::KdTreeFLANN<PoinType>::Ptr kdtree;
        pcl::PointCloud<PoinType>::Ptr cloud;
    };

    std::vector<std::vector<cloudWithKdtree>> clouds;
    std::vector<std::pair<cloudWithKdtree*,cloudWithKdtree*>> pairs_to_test;

    clouds.resize(scans_sites.size());
    for (int i =0; i < scans_sites.size(); i++)
    {
        clouds[i].resize(scans_sites[i].datastreams.size());
        for (int j=0; j < scans_sites[i].datastreams.size(); j++ ) {

            const auto &scan1 = scans_sites[i].datastreams[j];
            const auto &instrument_pose1 = scans_sites[i].intruments_poses_SE3.at(j);
            const auto &scan_site_pose1 = scans_sites[i].scan_site_pose_SE3;
            const Sophus::SE3d global_pose(Eigen::Matrix4d::Identity());
            auto cloud_ptr = getPoinCloud(scan1, mirrors, instrument_pose1, intruments_lever_arm, global_pose, angle_mask, scan_site_pose1).makeShared();
            clouds[i][j].cloud = cloud_ptr;
            clouds[i][j].scan_id = j;
            clouds[i][j].pose_id = i;
            clouds[i][j].kdtree = pcl::KdTreeFLANN<PoinType>::Ptr(new pcl::KdTreeFLANN<PoinType>);
            clouds[i][j].kdtree->setInputCloud(cloud_ptr);
        }
    }
    for (int i =0; i <clouds.size();i++){
        for (int j =0; j <clouds.size();j++){
            if (!allow_nn_between_sites && i != j )continue;
            if (!allow_nn_inside_sites  && i == j )continue;
            // here we allow to test the same points
            for (int k = 0; k < clouds[i].size(); k++){
                for (int l = 0; l < clouds[j].size(); l++){
                    if (i ==j && k==l ) continue;
                    pairs_to_test.emplace_back(std::make_pair(&clouds.at(i).at(k), &clouds.at(j).at(l)));
                }
            }
        }
    }
    std::vector<found_nn> total_nns;
    tbb::mutex mtx;
    tbb::parallel_for(
            tbb::blocked_range<size_t>(0,pairs_to_test.size()),
            [&](const tbb::blocked_range<size_t>& r) {
                std::vector<found_nn> local_nns;
                for (long i=r.begin();i<r.end();++i)
                {
                    const auto &p = pairs_to_test[i];
                    const auto &pointcloud2 = p.second->cloud;
                    const auto &pointcloud1 = p.first->cloud;
                    const auto &kdtree1 = p.first->kdtree;

                    int cloud1_id = p.first->scan_id;
                    int cloud2_id = p.second->scan_id;

                    int pose1_id = p.first->pose_id;
                    int pose2_id = p.second->pose_id;

                    for (int cloud2_point_id = 0; cloud2_point_id < pointcloud2->size(); cloud2_point_id+=100) {
                        const auto &p2 = (*pointcloud2)[cloud2_point_id];
                        std::vector<int> pointIdxRadiusSearch;
                        std::vector<float> pointRadiusSquaredDistance;
                        if (kdtree1->radiusSearch(p2, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                            for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
                                const auto p1 = pointcloud1->at(pointIdxRadiusSearch[i]);

                                const int cloud1_point_id_org = p1.label;
                                const int cloud2_point_id_org = p2.label;

                                local_nns.emplace_back(found_nn{pose1_id,pose2_id,cloud1_id, cloud2_id, cloud1_point_id_org, cloud2_point_id_org,
                                                                p1.getArray3fMap().cast<double>(),
                                                                p2.getArray3fMap().cast<double>()});
                                break;
                            }
                        }
                    }
                }
                tbb::mutex::scoped_lock lck (mtx);
                total_nns.reserve(total_nns.size()+local_nns.size());
                std::move(std::begin(local_nns), std::end(local_nns), std::back_inserter(total_nns));
            });

    return total_nns;
}

void getNNDrawings(const std::vector<found_nn> nns, std::vector<float>& nns_vertex,
                   std::vector<unsigned int>& nns_indices)
{
    nns_vertex.reserve(nns_vertex.size()+nns.size()*12);
    nns_indices.reserve(nns_indices.size()+nns.size()*2);
    for (const auto &nn : nns)
    {
        nns_vertex.push_back(nn.p1.x());
        nns_vertex.push_back(nn.p1.y());
        nns_vertex.push_back(nn.p1.z());
        nns_vertex.push_back(1);
        nns_vertex.push_back(1);
        nns_vertex.push_back(1);
        nns_indices.push_back(nns_indices.size());

        nns_vertex.push_back(nn.p2.x());
        nns_vertex.push_back(nn.p2.y());
        nns_vertex.push_back(nn.p2.z());
        nns_vertex.push_back(1);
        nns_vertex.push_back(1);
        nns_vertex.push_back(1);
        nns_indices.push_back(nns_indices.size());
    }
}

int main(int argc, char **argv) {

    struct imgui_params{
        int gizmed_scan = -1;
        float imgui_nn_len = 0.2;
        int imgui_iters = 5;

        Eigen::Matrix4f gimzed{ Eigen::Matrix4f::Identity()};
        float min_angle{0};
        float max_angle{20};
        bool block_mirror {false};                  //<Disables optimizitation of plane coefitiens of all mirrrors
        bool block_leverarm {false};                //<Disables optimizitation of laser SE3 agains turntable
        bool block_global_pose{true};

        bool block_pose_relative {false};           //<Disables optimization of scan sites SE3
        bool allow_nn_between_sites{true};          //<Creates observation between different scan poses
        bool allow_nn_inside_sites{true};           //<Creates observation inside one scan site
        bool allow_nn_groundtruth{true};            //<Creates observation between ground truth

        bool render_sparse{true};
        bool render_gt{true};

        bool operator==(const imgui_params &rhs) const {
            if (gizmed_scan != rhs.gizmed_scan)return false;
            if (gimzed != rhs.gimzed)return false;
            if (render_sparse != rhs.render_sparse)return false;

            return true;
        }
    }gui_data,gui_data_old;

    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "produce help message")
            ("dataset", po::value<std::vector<std::string>>()->multitoken()->required(), "path to dataset/datasets.")
            ("ground_truth", po::value<std::string>(), "path to groundtruth PCD.")
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
    const std::string mirror_cfg_fn{vm["mirror_config"].as<std::string>()};
    std::cout << "loading mirrors config from " << mirror_cfg_fn << std::endl;
    std::vector<clv::Mirror> mirrors = clv::loadMirrorFromPLY(mirror_cfg_fn);
    std::vector<std::pair<double,double>> angle_mask;

    // SE3 pose between the turntable's frame and the laser's scanner local frame
    Sophus::SE3d intruments_lever_arm_SE3(Eigen::Matrix4d::Identity());

    // SE3 pose against all sites and grounduth
    Sophus::SE3d global_pose_SE3(Eigen::Matrix4d::Identity());
    std::vector<clv::scan_site> scans_sites;

    std::vector<double> angles;
    for (int i =0; i < 350; i+=10)
    {
        angles.push_back(i);
    }

    // load ground truth
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundtruth_pointcloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    if (vm.count("ground_truth")) {
        const std::string gt_fn{vm["ground_truth"].as<std::string>()};
        std::cout << "loading ground truth PCD from " << gt_fn << std::endl;
        pcl::io::loadPCDFile<pcl::PointXYZRGB>(gt_fn,*groundtruth_pointcloud);
    }
    Eigen::Affine3f gt_rot (Eigen::Affine3f::Identity());
    gt_rot.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY()));
    pcl::transformPointCloud(*groundtruth_pointcloud, *groundtruth_pointcloud, gt_rot.matrix());
    pcl::KdTreeFLANN<pcl::PointXYZRGB> groundtruth_kdtree;
    groundtruth_kdtree.setInputCloud(groundtruth_pointcloud);

    // load datasets
    const std::vector<std::string> datasets_lib_fn{vm["dataset"].as<std::vector<std::string>>()};
    std::cout << "Loaidng dataset lib from : (it can take a while, 1st time)" <<std::endl;
    for (const auto &s : datasets_lib_fn){
        std::cout << " -> " << s << std::endl;
        scans_sites.push_back(clv::loadScanSite(s));
    }

    GLFWwindow *window;
    if (!glfwInit())
        return -1;
    window = glfwCreateWindow(960, 540, "Livox Mirror Calibration", NULL, NULL);
    if (!window) {glfwTerminate();return -1;}

    glfwMakeContextCurrent(window);
    glfwSetCursorPosCallback(window, cursor_calback);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    glfwSwapInterval(1);
    if (glewInit() != GLEW_OK) { return -1; }

    GLCall(glClearColor(0.4, 0.4, 0.4, 1));

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

    std::vector<float> pointcloud_vertices;
    std::vector<unsigned int> pointcloud_indices;

    std::vector<float> pointcloud_vertices_gt;
    std::vector<unsigned int> pointcloud_indices_gt;

    for (auto &s : scans_sites) {
        catoptric_livox::updateDrawingBuffer(s.datastreams, mirrors, s.intruments_poses_SE3, intruments_lever_arm_SE3,
                                             s.scan_site_pose_SE3,
                                             global_pose_SE3,
                                             angle_mask,
                                             catoptric_livox::LASER_WITH_MIRROR_COLOR,
                                             pointcloud_vertices, pointcloud_indices,
                                             gui_data.render_sparse ? 15 : 0);
    }
    VertexArray va_pointcloud;
    VertexBuffer vb_pointcloud(pointcloud_vertices.data(),
                               pointcloud_vertices.size() * sizeof(float));
    va_pointcloud.AddBuffer(vb_pointcloud, layout);
    IndexBuffer ib_pointcloud(pointcloud_indices.data(), pointcloud_indices.size());

    auto mat = Eigen::Affine3d::Identity();
    mat.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()));
    pointcloud_vertices_gt.reserve(groundtruth_pointcloud->size()*6);
    pointcloud_indices_gt.reserve(groundtruth_pointcloud->size());
    for (int i=0; i < groundtruth_pointcloud->size();i++)
    {
        const auto &p = (*groundtruth_pointcloud)[i];
        pointcloud_vertices_gt.push_back(p.x);
        pointcloud_vertices_gt.push_back(p.y);
        pointcloud_vertices_gt.push_back(p.z);
        pointcloud_vertices_gt.push_back(1.0*p.r/255);
        pointcloud_vertices_gt.push_back(1.0*p.g/255);
        pointcloud_vertices_gt.push_back(1.0*p.b/255);
        pointcloud_indices_gt.push_back(pointcloud_indices_gt.size());
    }

    VertexArray va_pointcloud_gt;
    VertexBuffer vb_pointcloud_gt(pointcloud_vertices_gt.data(),
                               pointcloud_vertices_gt.size() * sizeof(float));
    va_pointcloud_gt.AddBuffer(vb_pointcloud_gt, layout);
    IndexBuffer ib_pointcloud_gt(pointcloud_indices_gt.data(), pointcloud_indices_gt.size());

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
    ImGui_ImplOpenGL3_Init();

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
        glm::mat4 proj = glm::perspective(30.f, 1.0f*width/height, 0.05f, 100.0f);
        glm::mat4 model_translate = glm::translate(glm::mat4(1.0f), view_translation);
        glm::mat4 model_rotation_0 = glm::rotate(model_translate, float(-0.5*M_PI), glm::vec3(0.0f, 0.0f, 1.0f));

        glm::mat4 model_rotation_1 = glm::rotate(model_rotation_0, rot_x, glm::vec3(0.0f, 1.0f, 0.0f));
        glm::mat4 model_rotation_2 = glm::rotate(model_rotation_1, rot_y, glm::vec3(1.0f, 0.0f, 0.0f));
        shader.Bind(); // bind shader to apply uniform

        GLCall(glLineWidth(1));
        shader.setUniformMat4f("u_MVP", proj * model_rotation_2);
        GLCall(glPointSize(2));
        renderer.Draw(va_pointcloud, ib_pointcloud, shader, GL_POINTS);
        if (gui_data.render_gt){
            GLCall(glPointSize(1));
            renderer.Draw(va_pointcloud_gt, ib_pointcloud_gt, shader, GL_POINTS);
        }
        renderer.Draw(va_mirror, ib_mirror, shader, GL_LINES);
        renderer.Draw(va_nns, ib_nns, shader, GL_LINES);

        for (const auto &s:scans_sites) {
            glm::mat4 tl = glm::mat4(1.0f);
            Eigen::Map<Eigen::Matrix4f> tl_e(&tl[0][0]);
            tl_e = (s.scan_site_pose_SE3 ).matrix().cast<float>();
            shader.setUniformMat4f("u_MVP", proj * model_rotation_2 * tl );
            renderer.Draw(va_co, ib_co, shader, GL_LINES);

            for (int i = 0; i < s.intruments_poses_SE3.size(); i++) {
                glm::mat4 tr = glm::mat4(1.0f);
                glm::mat4 sc = glm::scale(glm::mat4(1.0f), glm::vec3(0.02f));
                Eigen::Map<Eigen::Matrix4f> tr_e(&tr[0][0]);
                tr_e = (s.scan_site_pose_SE3.matrix() * s.intruments_poses_SE3[i].matrix() *
                        intruments_lever_arm_SE3.matrix()).cast<float>();
                shader.setUniformMat4f("u_MVP", proj * model_rotation_2 * tr * sc);
                renderer.Draw(va_co, ib_co, shader, GL_LINES);
            }
        }
        for (int i =0; i< mirrors.size(); i++)
        {
            glm::mat4 tr = glm::mat4(1.0f);
            glm::mat4 sc = glm::scale(glm::mat4(1.0f), glm::vec3(0.02f));
            Eigen::Map<Eigen::Matrix4f> tr_e (&tr[0][0]);
            tr_e = mirrors[i].getTransformation().matrix().cast<float>();
            shader.setUniformMat4f("u_MVP", proj * model_rotation_2 * tr * sc);
            renderer.Draw(va_co, ib_co, shader, GL_LINES);
        }

        ImGui::Begin("Calibration Demo");
        ImGui::Checkbox("render sparse", &gui_data.render_sparse);
        ImGui::Checkbox("render ground truth", &gui_data.render_gt);
        ImGui::Text("calibration");
        ImGui::Checkbox("block mirror", &gui_data.block_mirror);
        ImGui::Checkbox("block leverarm", &gui_data.block_leverarm);
        ImGui::Checkbox("block pose relative", &gui_data.block_pose_relative);
        ImGui::Checkbox("block global pose", &gui_data.block_global_pose);
        ImGui::Checkbox("allow NN between_sites", &gui_data.allow_nn_between_sites);
        ImGui::Checkbox("allow NN inside_site", &gui_data.allow_nn_inside_sites);
        ImGui::Checkbox("allow NN groundtruth", &gui_data.allow_nn_groundtruth);

        ImGui::RadioButton("gizmo:None", &gui_data.gizmed_scan, -1);
        ImGui::RadioButton("gizmo:GlobalPose", &gui_data.gizmed_scan, -2);
        for (int i =1; i < scans_sites.size(); i++)
        {
            ImGui::RadioButton(("gizmo:"+std::to_string(i)).c_str(), &gui_data.gizmed_scan, i);
        }
        if(ImGui::Button("draw NN")){
            nns_vertices.clear();
            nns_indices.clear();
            auto nns = findNN(scans_sites, mirrors, intruments_lever_arm_SE3, angle_mask, gui_data.imgui_nn_len,
                              gui_data.allow_nn_between_sites, gui_data.allow_nn_inside_sites);
            getNNDrawings(nns, nns_vertices, nns_indices);

            std::vector<found_nn> nns_gt;
            if (gui_data.allow_nn_groundtruth) {
                // search of nn between scan sites and ground truth
                nns_gt = findNNAgainstGT(scans_sites, mirrors, intruments_lever_arm_SE3,global_pose_SE3,
                                         groundtruth_kdtree, *groundtruth_pointcloud,
                                         angle_mask,
                                         gui_data.imgui_nn_len);
            }
            getNNDrawings(nns_gt, nns_vertices, nns_indices);
            vb_nns.update(nns_vertices.data(), nns_vertices.size() * sizeof(float));
            ib_nns.update(nns_indices.data(), nns_indices.size());
        }

        if(ImGui::Button("reset")){
            intruments_lever_arm_SE3 = Sophus::SE3d(Eigen::Matrix4d::Identity());
            for (int i =0; i< mirrors.size(); i++){
                mirrors[i].reset();
            }
            for (int i=0; i < scans_sites.size(); i ++)
            {
                scans_sites[i].scan_site_pose_SE3 = scans_sites[i].scan_site_pose_SE3_initial;
                for (int j=0; j < scans_sites[i].intruments_poses_SE3.size(); j++)
                {
                    scans_sites[i].intruments_poses_SE3[j] = scans_sites[i].intruments_poses_SE3_initial[j];
                }
            }
            gl_dirty = true;
        }

        if(ImGui::Button("optimize"))
        {
            for (int iter = 0; iter < gui_data.imgui_iters ; iter++) {
                // search for nearest neighborhood
                const std::vector<found_nn> nns = findNN(scans_sites, mirrors, intruments_lever_arm_SE3, angle_mask,
                                                         gui_data.imgui_nn_len,
                                                         gui_data.allow_nn_between_sites,
                                                         gui_data.allow_nn_inside_sites);
                std::vector<found_nn> nns_gt;
                if (gui_data.allow_nn_groundtruth) {
                    // search of nn between scan sites and ground truth
                    nns_gt = findNNAgainstGT(scans_sites, mirrors, intruments_lever_arm_SE3,global_pose_SE3,
                                             groundtruth_kdtree, *groundtruth_pointcloud,
                                             angle_mask,
                                             gui_data.imgui_nn_len);
                }
                std::cout << "number of nns (inside and between)  : " << nns.size() << std::endl;
                std::cout << "number of nns (against groundtruth) : " << nns_gt.size() << std::endl;

                ceres::Problem problem;
                // initial parameters values
                std::vector<Eigen::Vector4d> mirror_plane_coef_init;
                const Sophus::Vector6d lever_arm_se3_init = intruments_lever_arm_SE3.log();
                const Sophus::Vector6d global_pose_se3_init = global_pose_SE3.log();
                std::vector<Sophus::Vector6d> scan_sites_poses_se3_init;
                for (int i = 0; i < scans_sites.size(); i++) {
                    scan_sites_poses_se3_init.push_back(scans_sites[i].scan_site_pose_SE3.log());
                }

                // optimization parameters setup
                for (int i=0; i <scans_sites.size(); i++)
                {
                    problem.AddParameterBlock(scans_sites[i].scan_site_pose_SE3.data(), Sophus::SE3d::num_parameters, new LocalParameterizationSE3());
                }
                for (int i = 0; i < mirrors.size(); i++) {
                    mirror_plane_coef_init.push_back(mirrors[i].getABCDofPlane());
                    problem.AddParameterBlock(mirrors[i].getABCDofPlane().data(), catoptric_livox::Mirror::getNumberOfParams(), new LocalParameterizationPlane());
                }
                for (int i = 0; i < scans_sites.size(); i++) {
                    for (int j = 0; j < scans_sites[i].intruments_poses_SE3.size(); j++){
                        problem.AddParameterBlock(scans_sites[i].intruments_poses_SE3[j].data(), Sophus::SE3d::num_parameters, new LocalParameterizationSE3());
                        problem.SetParameterBlockConstant(scans_sites[i].intruments_poses_SE3[j].data());
                    }
                }
                problem.AddParameterBlock(intruments_lever_arm_SE3.data(), Sophus::SE3d::num_parameters, new LocalParameterizationSE3());
                problem.AddParameterBlock(global_pose_SE3.data(), Sophus::SE3d::num_parameters, new LocalParameterizationSE3());

                // parameter blocking from GUI
                if (gui_data.block_mirror) {
                    for (int i = 0; i < mirrors.size(); i++) {
                        problem.SetParameterBlockConstant(mirrors[i].getABCDofPlane().data());
                    }
                }
                if (gui_data.block_leverarm) {
                    problem.SetParameterBlockConstant(intruments_lever_arm_SE3.data());
                }
                if (gui_data.block_pose_relative) {
                    for (int i = 0; i <  scans_sites.size(); i++) {
                        problem.SetParameterBlockConstant( scans_sites[i].scan_site_pose_SE3.data());
                    }
                }
                if (gui_data.block_global_pose){
                    problem.SetParameterBlockConstant(global_pose_SE3.data());
                }
                // always block 1-st site scan
                problem.SetParameterBlockConstant(scans_sites[0].scan_site_pose_SE3.data());

                // consume observations to ground truth
                for (const auto nn : nns_gt) {
                    const auto &scan1 = scans_sites[nn.scan_site1_id];
                    const auto pose1_id = nn.scan1_id;
                    const auto &ds1 = scan1.datastreams[pose1_id][nn.scan1_point_id];

                    const Eigen::Vector3d p1{ds1.X, ds1.Y, ds1.Z};
                    int mirror1_id{-1};
                    // get mirrors
                    for (int i = 0; i < mirrors.size(); i++) {
                        if (mirrors[i].checkIfRayIntersectMirror(Eigen::Vector3d(0, 0, 0), p1)) {
                            mirror1_id = i;
                            break;
                        }
                    }
                    if (mirror1_id == -1)continue;
                    ceres::LossFunction *loss = nullptr;//new ceres::CauchyLoss(gui_data.imgui_nn_len);
                    ceres::CostFunction *cost_function =
                            MirrorOprimizeABCDWithPoseWithSitePose::Create(p1, nn.p2,
                                                                           scans_sites[nn.scan_site1_id].intruments_poses_SE3[pose1_id]);
                    problem.AddResidualBlock(cost_function, loss,
                                             mirrors[mirror1_id].getABCDofPlane().data(),
                                             scans_sites[nn.scan_site1_id].intruments_poses_SE3[pose1_id].data(),
                                             intruments_lever_arm_SE3.data(),
                                             scans_sites[nn.scan_site1_id].scan_site_pose_SE3.data(),
                                             global_pose_SE3.data());

                }
                // consume observations to uni-view / multi-view
                for (const auto nn : nns) {
                    const auto &scan1 = scans_sites[nn.scan_site1_id];
                    const auto &scan2 = scans_sites[nn.scan_site2_id];

                    const auto pose1_id = nn.scan1_id;
                    const auto pose2_id = nn.scan2_id;

                    // case 1 - we have nn-pair from the same scan site
                    if (nn.scan_site1_id == nn.scan_site2_id) {
                        // hovewer we cannot add residum from the same measurment.
                        // at this state that should not happend, but be safe
                        if (pose1_id == pose2_id) continue;

                        const auto &ds1 = scan1.datastreams[pose1_id][nn.scan1_point_id];
                        const auto &ds2 = scan1.datastreams[pose2_id][nn.scan2_point_id];

                        const Eigen::Vector3d p1{ds1.X, ds1.Y, ds1.Z};
                        const Eigen::Vector3d p2{ds2.X, ds2.Y, ds2.Z};

                        int mirror1_id = -1;
                        int mirror2_id = -1;
                        // get mirrors
                        for (int i = 0; i < mirrors.size(); i++) {
                            if (mirrors[i].checkIfRayIntersectMirror(Eigen::Vector3d(0, 0, 0), p1)) {
                                mirror1_id = i;
                                break;
                            }
                        }
                        for (int i = 0; i < mirrors.size(); i++) {
                            if (mirrors[i].checkIfRayIntersectMirror(Eigen::Vector3d(0, 0, 0), p2)) {
                                mirror2_id = i;
                                break;
                            }
                        }
                        // uni-view
                        if (mirror1_id != mirror2_id && mirror1_id != -1 && mirror2_id != -1) {
                            const auto local_nnp2 = scans_sites[nn.scan_site1_id].scan_site_pose_SE3.inverse() * nn.p2;
                            ceres::LossFunction *loss = new ceres::CauchyLoss(gui_data.imgui_nn_len);
                            ceres::CostFunction *cost_function =
                                    MirrorOprimizeABCDWithPose::Create(p1, local_nnp2,
                                                                 scans_sites[nn.scan_site1_id].intruments_poses_SE3[pose1_id]);
                            problem.AddResidualBlock(cost_function, loss,
                                                     mirrors[mirror1_id].getABCDofPlane().data(),
                                                     scans_sites[nn.scan_site1_id].intruments_poses_SE3[pose1_id].data(),
                                                     intruments_lever_arm_SE3.data());
                        }
                    }else{
                        const auto &ds1 = scan1.datastreams[pose1_id][nn.scan1_point_id];
                        const auto &ds2 = scan1.datastreams[pose2_id][nn.scan2_point_id];
                        const Eigen::Vector3d p1{ds1.X, ds1.Y, ds1.Z};
                        const Eigen::Vector3d p2{ds2.X, ds2.Y, ds2.Z};

                        int mirror1_id = -1;
                        int mirror2_id = -1;
                        // get mirrors
                        for (int i = 0; i < mirrors.size(); i++) {
                            if (mirrors[i].checkIfRayIntersectMirror(Eigen::Vector3d(0, 0, 0), p1)) {
                                mirror1_id = i;
                                break;
                            }
                        }
                        for (int i = 0; i < mirrors.size(); i++) {
                            if (mirrors[i].checkIfRayIntersectMirror(Eigen::Vector3d(0, 0, 0), p2)) {
                                mirror2_id = i;
                                break;
                            }
                        }
                        // multi-view
                        if (/*mirror1_id != mirror2_id &&*/ mirror1_id != -1 && mirror2_id != -1) {
                            ceres::LossFunction *loss = new ceres::CauchyLoss(gui_data.imgui_nn_len);
                            ceres::CostFunction *cost_function =
                                    MirrorOprimizeABCDWithPoseWithSitePose::Create(p1, nn.p2,
                                                                                   scans_sites[nn.scan_site1_id].intruments_poses_SE3[pose1_id]);
                            problem.AddResidualBlock(cost_function, loss,
                                                     mirrors[mirror1_id].getABCDofPlane().data(),
                                                     scans_sites[nn.scan_site1_id].intruments_poses_SE3[pose1_id].data(),
                                                     intruments_lever_arm_SE3.data(),
                                                     global_pose_SE3.data(),
                                                     scans_sites[nn.scan_site1_id].scan_site_pose_SE3.data());
                        }
                    }
                }
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.minimizer_progress_to_stdout = true;
                options.max_num_iterations = 5;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
                std::cout << summary.FullReport() << "\n";

                // Printout deltas on parameters
                std::cout << "==========================" << std::endl;
                std::cout << "Deltas of all DOFs : " << std::endl;
                for (int i = 0; i < mirrors.size(); i++) {
                    std::cout << "mirror[" << i << "] : " <<
                        (mirror_plane_coef_init[i]-mirrors[i].getABCDofPlane()).transpose()<<std::endl;
                }
                std::cout << "lever_arm_se3   : " << (intruments_lever_arm_SE3.log()-lever_arm_se3_init).transpose() << std::endl;
                std::cout << "global_pose_se3 : " << (global_pose_SE3.log()-global_pose_se3_init).transpose() << std::endl;

                for (int i = 0; i < scan_sites_poses_se3_init.size(); i++) {
                    std::cout << "scan_sites_poses_se3 ["<<i<<"] : "<<
                        (scans_sites[i].scan_site_pose_SE3.log()-scan_sites_poses_se3_init[i]).transpose() << std::endl;
                }
                std::cout << "==========================" << std::endl;
            }
            // request update of GL drawing
            gl_dirty = true;
        }
        if(ImGui::Button("export scan sites poses"))
        {
            std::vector<Sophus::SE3d> scan_site_poses_SE3;
            for (const auto &s : scans_sites)
            {
                scan_site_poses_SE3.push_back(s.scan_site_pose_SE3);
            }
            catoptric_livox::saveCFG(scan_site_poses_SE3);
        }
        ImGui::SameLine();
        if(ImGui::Button("import scan sites poses"))
        {
            gui_data.gizmed_scan = -1;
            std::vector<Sophus::SE3d> scan_site_poses_SE3;
            catoptric_livox::loadCFG(scan_site_poses_SE3);
            for (int i =0; i < scan_site_poses_SE3.size(); i++){
                scans_sites[i].scan_site_pose_SE3 =  scan_site_poses_SE3[i];
            }
            gl_dirty = true;
        }

        if(ImGui::Button("export calib"))
        {
            catoptric_livox::saveCFG(mirrors, intruments_lever_arm_SE3);
        }

        ImGui::SameLine();
        if(ImGui::Button("import calib"))
        {
            catoptric_livox::loadCFG(mirrors, intruments_lever_arm_SE3);
            Eigen::Matrix<double, 6, 1> lever_arm_s3 =  Sophus::SE3d(intruments_lever_arm_SE3.matrix()).log();
            //for (auto & m : mirrors)m.updateTransformFromCoeef();
            gl_dirty = true;
        }

        if(ImGui::Button("export PCDs"))
        {
            pcl::PointCloud<pcl::PointXYZL> dataset;
            for (int i =0; i < scans_sites.size(); i++)
            {
                for (int j=0; j < scans_sites[i].datastreams.size(); j++ ) {

                    const auto &scan1 = scans_sites[i].datastreams[j];
                    const auto &instrument_pose1 = scans_sites[i].intruments_poses_SE3.at(j);
                    const auto &scan_site_pose1 = scans_sites[i].scan_site_pose_SE3;

                    const auto p = getPoinCloud(scan1, mirrors, instrument_pose1, intruments_lever_arm_SE3, global_pose_SE3,
                                                angle_mask, scan_site_pose1);
                    dataset+=p;
                }
            }
            pcl::io::savePCDFile("site_scan.pcd", dataset);
            pcl::io::savePCDFile("gt.pcd", *groundtruth_pointcloud);
        }

//        if(ImGui::Button("load angle mask"))
//        {
//            angle_mask.clear();
//            std::ifstream infile ("mask.txt");
//            std::string line;
//            while (std::getline(infile, line)){
//                std::istringstream iss(line);
//                float a, b;
//                if (!(iss >> a >> b)) { break; } // error
//                std::cout << "mask " << a << " " << b << std::endl;
//                angle_mask.push_back({std::min(a,b),std::max(b,a)});
//            }
//            gl_dirty = true;
//        }

        ImGui::SliderFloat("nnsize", &gui_data.imgui_nn_len, 0, 1.0f);
        ImGui::SliderInt("iters", &gui_data.imgui_iters, 1, 20);
        ImGui::End();

        if (gui_data.gizmed_scan!= -1)
        {
            ImGuizmo::Manipulate(&model_rotation_2[0][0], &proj[0][0], ImGuizmo::TRANSLATE | ImGuizmo::ROTATE_X,
                                 ImGuizmo::WORLD, gui_data.gimzed.data());
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
        glfwPollEvents();
        if (!(gui_data_old == gui_data)||gl_dirty){
            if (gui_data.gizmed_scan>=0) {
                if (gui_data.gizmed_scan != gui_data_old.gizmed_scan) {
                    gui_data.gimzed = scans_sites.at(gui_data.gizmed_scan).scan_site_pose_SE3.matrix().cast<float>();
                } else {
                    const auto m0 = Eigen::Affine3d(gui_data.gimzed.cast<double>());
                    const auto m1= catoptric_livox::orthogonize(m0);
                    Sophus::SE3d m2(m1.matrix());
                    scans_sites.at(gui_data.gizmed_scan).scan_site_pose_SE3 = m2;
                }
            }
            if (gui_data.gizmed_scan==-2){
                const auto m0 = Eigen::Affine3d(gui_data.gimzed.cast<double>());
                const auto m1= clv::orthogonize(m0);
                Sophus::SE3d m2(m1.matrix());
                global_pose_SE3 = m2;
            }

            clv::updateDrawingBuffer(mirrors, mirrors_vertices, mirrors_indices);
            vb_mirror.update(mirrors_vertices.data(),
                                   mirrors_vertices.size() * sizeof(float));
            ib_mirror.update(mirrors_indices.data(), mirrors_indices.size());
            pointcloud_vertices.clear();
            pointcloud_indices.clear();
            for (auto &s : scans_sites) {
                clv::updateDrawingBuffer(s.datastreams, mirrors, s.intruments_poses_SE3,
                                                     intruments_lever_arm_SE3,
                                                     s.scan_site_pose_SE3,
                                                     global_pose_SE3,
                                                     angle_mask,
                                                     clv::LASER_WITH_MIRROR_COLOR,
                                                     pointcloud_vertices, pointcloud_indices,
                                                     gui_data.render_sparse ? 15 : 0);
            }
            vb_pointcloud.update(pointcloud_vertices.data(),
                                 pointcloud_vertices.size() * sizeof(float));
            ib_pointcloud.update(pointcloud_indices.data(), pointcloud_indices.size());
            gl_dirty = false;
        }
        gui_data_old = gui_data;
    }
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwTerminate();
    return 0;
}