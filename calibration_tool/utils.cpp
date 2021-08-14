#include "utils.h"
#define TINYPLY_IMPLEMENTATION
#include "3rd/tinyply.h"

Eigen::Affine3d catoptric_livox::orthogonize(const Eigen::Affine3d& p ){
    Eigen::Affine3d pose_orto(Eigen::Affine3d::Identity());
    Eigen::Quaterniond q1(p.matrix().block<3,3>(0,0)); q1.normalize();
    pose_orto.translate(p.matrix().block<3,1>(0,3));
    pose_orto.rotate(q1);
    return pose_orto;
}
catoptric_livox::Mirror::Mirror(const std::vector<Eigen::Vector3d>& segments):
        segments(segments)
{
    centroid = Eigen::Vector3d::Zero();
    for (const auto & p : segments )
    {
        centroid+= p;
    }
    centroid = centroid / segments.size();

    // comput
    Eigen::Matrix3d covariance;
    for (int x = 0; x < 3; x ++)
    {
        for (int y = 0; y < 3; y ++)
        {
            double element =0;
            for (const auto pp : segments)
            {
                element += (pp(x) - centroid(x)) * (pp(y) - centroid(y));
            }
            covariance(x,y) = element / (segments.size()-1);
        }
    };

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullV|Eigen::ComputeFullU);
    transformation = Eigen::Matrix4d::Identity();
    if (std::abs(svd.singularValues()[2]) > 1e-12)
    {
        std::cerr << "mirrror is NOT flat" <<std::endl;
    }else
    {
        std::cout << "mirrror is flat" <<std::endl;
    }


    transformation.matrix().block<3,3>(0,0) = svd.matrixV();
    if (transformation.matrix().block<3,3>(0,0).determinant() < 0)
    {
        transformation.matrix().block<3,1>(0,0) = -svd.matrixV().block<3,1>(0,0);
        transformation.matrix().block<3,1>(0,1) = -svd.matrixV().block<3,1>(0,1);
        transformation.matrix().block<3,1>(0,2) = -svd.matrixV().block<3,1>(0,2);
    }
    transformation.matrix().block<3,1>(0,3) = centroid;

    transformation_inv = transformation.inverse();
    segments2D.resize(segments.size());
    for (int i=0; i< segments.size(); i++)
    {
        Eigen::Vector3d segment_flatten = transformation_inv * segments[i];
        if (std::abs(segment_flatten.z()) > 1e-6 ){
            std::cerr << "mirror " << i << " has non-flat vertex " << segment_flatten.z() << std::endl;
        }
        segments2D[i] = segment_flatten.head<2>();
    }

    plane_coeff = catoptric_livox::getPlaneCoefFromSE3(transformation.matrix());
    plane_coeff_init = plane_coeff;
//    plane_coeff.x() = transformation.rotation()(0,2);
//    plane_coeff.y() = transformation.rotation()(1,2);
//    plane_coeff.z() = transformation.rotation()(2,2);
//    plane_coeff.w() = -centroid.x() *  plane_coeff.x() - centroid.y() *  plane_coeff.y() - centroid.z() *  plane_coeff.z();


    for (int i =0; i < segments.size(); i++) {
        const double dot = segments[i].x() * plane_coeff.x() + segments[i].y() * plane_coeff.y() +
                           segments[i].z() * plane_coeff.z() + plane_coeff.w();
        //assert(std::abs(dot) < 1e-5);
    }
}
void catoptric_livox::Mirror::reset(){
    plane_coeff = catoptric_livox::getPlaneCoefFromSE3(transformation.matrix());
}

bool catoptric_livox::checkIfMasked(const Eigen::Vector3d& dir, const std::vector<std::pair<double,double>>& angle_mask)
{
    double bearing = std::atan2(dir.y(),dir.z());
    for (const auto p: angle_mask)
    {
        if (bearing>(p.first)*M_PI/180.0&&bearing<(p.second)*M_PI/180.0){
            return false;
        }
    }
    return true;
}
bool catoptric_livox::Mirror::checkIfRayIntersectMirror(const Eigen::Vector3d& origin,const Eigen::Vector3d& dir) const
{
    if (dir == origin) return false;
    //double radius = std::sqrt(dir.y()*dir.y()+dir.z()+dir.z());
    // project on plane
    Eigen::Vector3d dirn = dir/dir.norm();
    if (dirn.x() > std::cos(min_angle*M_PI/180.0)) return false;
    if (dirn.x() < std::cos(max_angle*M_PI/180.0)) return false;
//
//    double bearing = std::atan2(dirn.y(), dirn.z());
//
//    std::vector<double> vertices_bearings;
//    for (auto f:this->segments)
//    {
//        Eigen::Vector3d fn = f/f.norm();
//        double bearingf = std::atan2(dirn.y(), dirn.z());
//        vertices_bearings.push_back(bearingf);
//    }
//    std::sort(vertices_bearings.begin(),vertices_bearings.end());
//    if (bearing > vertices_bearings)

    Eigen::Vector3d intersection;
    const auto &p = plane_coeff_init;
    double a = p.x() * dirn.x() + p.y() * dirn.y() + p.z() * dirn.z();
    double dist = p.x() * origin.x() + p.y() * origin.y() + p.z() * origin.z() + p.w();

    intersection = origin - dirn * (dist/a);

    // project intersection on 2D
    Eigen::Vector3d intersection_2D = this->transformation_inv * intersection;
    Eigen::Vector2d point_2D = intersection_2D.head<2>();
//    assert(std::abs(intersection_2D.z()) < 1e-8);
    double angle = 0;
    for (int i =0; i < segments2D.size()-1; i++)
    {
        Eigen::Vector2d v1 = segments2D[i] - point_2D;
        Eigen::Vector2d v2 = segments2D[i+1] - point_2D;
        angle += acos(v1.dot(v2)/(v1.norm()*v2.norm()));
    }
    Eigen::Vector2d v1 = segments2D.back() - point_2D;
    Eigen::Vector2d v2 = segments2D.front() - point_2D;
    angle += acos(v1.dot(v2)/(v1.norm()*v2.norm()));

    if(std::abs(angle - 2*M_PI) < 1e-5){
        return true;
    }
    return false;
}

std::vector<catoptric_livox::Mirror> catoptric_livox::loadMirrorFromPLY(const std::string& filename){

    std::vector<catoptric_livox::Mirror> mirrors;
    // PLY mesh with mirrrors should contain quad faces with
    tinyply::PlyFile mirrors_mesh;
    std::ifstream file_stream(filename, std::ios::binary);
    if (file_stream.fail()) throw std::runtime_error("failed to open ");

    mirrors_mesh.parse_header(file_stream);

    std::shared_ptr<tinyply::PlyData> vertices, faces;

    vertices = mirrors_mesh.request_properties_from_element("vertex", {"x", "y", "z"});
    faces = mirrors_mesh.request_properties_from_element("face", {"vertex_indices"}, 3);

    mirrors_mesh.read(file_stream);

    std::vector<std::vector<Eigen::Vector3d>> segments;

    auto getVertFromBuffer = [](tinyply::PlyData &vertices, int i)
    {
        if (vertices.t == tinyply::Type::FLOAT32) {
            const float *const data = (float *) vertices.buffer.get();
            return Eigen::Vector3d  (data[3*i],data[3*i+1],data[3*i+2]);
        }else
        if (vertices.t == tinyply::Type::FLOAT64) {
            const double *const data = (double *) vertices.buffer.get();
            return Eigen::Vector3d  (data[3*i],data[3*i+1],data[3*i+2]);
        }else
        {
            throw ("PLY file has unsupported vertex type");
        }
    };

    if (faces->t == tinyply::Type::UINT32)
    {
        const u_int32_t *const data = (u_int32_t  *) faces->buffer.get();
        for (int i =0; i<faces->count; i++ ) {

            std::array<int,3> indices {{(int)data[3*i], (int)data[3*i+1],(int)data[3*i+2]/*,(int)data[4*i+3]*/}};
            std::array<Eigen::Vector3d, 3> verts;
            verts[0] = getVertFromBuffer(*vertices,indices[0]);
            verts[1] = getVertFromBuffer(*vertices,indices[1]);
            verts[2] = getVertFromBuffer(*vertices,indices[2]);
            //verts[3] = getVertFromBuffer(*vertices,indices[3]);
            //std::cout << verts[0].transpose() << ";" << verts[1].transpose()<< ";" << verts[2].transpose()<< /*";" << verts[3].transpose()*/ std::endl;
            mirrors.emplace_back(catoptric_livox::Mirror({verts[0],verts[1],verts[2]/*,verts[3]*/}));
        }
    }else{
        throw("usuported type for faces type");
    }


    file_stream.close();

    return mirrors;
}


void catoptric_livox::updateDrawingBuffer(const std::vector<catoptric_livox::Mirror> & mirrors,
                                          std::vector<float>& data_v, std::vector<unsigned int>& data_i )
{
    int lines_count = 0;
    for (const auto &m : mirrors){
        lines_count += m.getSegments().size();
    }

    data_v.resize(data_v.size()+lines_count*3*2);
    data_i.resize(data_i.size()+2*lines_count);

    int counter =0;
    for (int i=0; i< mirrors.size(); i++){
        const auto &m  =  mirrors[i];
        const auto color = mirror_colors[i];
        int loop_start = counter;
        for (Eigen::Vector3d s : m.getSegments())
        {
            data_v[6*counter] = s.x();
            data_v[6*counter+1] = s.y();
            data_v[6*counter+2] = s.z();
            data_v[6*counter+3] = color.x();
            data_v[6*counter+4] = color.y();
            data_v[6*counter+5] = color.z();
            data_i[2*counter] = counter;
            data_i[2*counter+1] = counter+1;

            counter++;
        }
        data_i[2*counter-1] = loop_start;
    }
}

void catoptric_livox::updateDrawingBuffer(const std::vector<std::vector<catoptric_livox::DataStream>> &data,
                         const std::vector<catoptric_livox::Mirror> & mirrors,
                         const std::vector<Sophus::SE3d> &instrument_poses,
                         const Sophus::SE3d &intruments_lever_arm,
                         const Sophus::SE3d &scan_pose,
                         const Sophus::SE3d &global_pose,
                         const std::vector<std::pair<double,double>>& angle_mask,
                         catoptric_livox::DrawType type,
                         std::vector<float>& data_v, std::vector<unsigned int>& data_i,
                         int skip )
{
    int count = 0;
    for (const auto & d :data)
    {
        count += d.size();
    }
    int counter = data_i.size();
    data_i.resize(data_i.size()+count);
    data_v.resize(data_v.size()+count*6);


    for (int k = 0; k<data.size(); k++ )
    {
        const auto& dds  = data[k];
        const auto& instrument_pose = instrument_poses[k];
        for (int i=0; i < dds.size(); i+=1+skip){
            const auto & ds = dds[i];
            int mirror_id =-1;
            if (catoptric_livox::checkIfMasked(Eigen::Vector3d(ds.X, ds.Y, ds.Z), angle_mask))
            {
                for (int i = 0; i < mirrors.size(); i++) {
                    if (mirrors[i].checkIfRayIntersectMirror(Eigen::Vector3d(0, 0, 0),
                                                             Eigen::Vector3d(ds.X, ds.Y, ds.Z))) {
                        mirror_id = i;
                        break;
                    }
                }
            }
            if (mirror_id!=-1) {
                Eigen::Vector3f c = catoptric_livox::mirror_colors[mirror_id];
                Eigen::Vector3d dir{ds.X, ds.Y, ds.Z};
                const double l = dir.norm();
                dir = dir / l;

//                Eigen::Vector4d plane = catoptric_livox::getPlaneCoefFromSE3(mirrors[mirror_id].getTransformationOptimized());
                Eigen::Vector4d plane = mirrors[mirror_id].getABCDofPlane();

                Eigen::Vector4d p = Eigen::Vector4d::Ones();
                p.head(3)= catoptric_livox::getMirroredRay(dir, l, plane);
                p = global_pose.matrix() * scan_pose.matrix()* instrument_pose.matrix() * intruments_lever_arm.matrix() * p;
                data_v[6 * counter] = p.x();
                data_v[6 * counter + 1] = p.y();
                data_v[6 * counter + 2] = p.z();

                data_v[6 * counter + 3] = c.x();
                data_v[6 * counter + 4] = c.y();
                data_v[6 * counter + 5] = c.z();
                data_i[counter] = counter;
                counter++;
            }else{
                Eigen::Vector4d p{ds.X, ds.Y, ds.Z, 1.0};
                p = global_pose.matrix() * scan_pose.matrix() * instrument_pose.matrix() * intruments_lever_arm.matrix() * p;

                data_v[6 * counter] = p.x();
                data_v[6 * counter + 1] = p.y();
                data_v[6 * counter + 2] = p.z();

                data_v[6 * counter + 3] = 0;
                data_v[6 * counter + 4] = 0;
                data_v[6 * counter + 5] = 0;
                data_i[counter] = counter;
                counter++;
            }
        }
    }
}

pcl::PointCloud<pcl::PointXYZL> catoptric_livox::getPoinCloud(const std::vector<catoptric_livox::DataStream> &data,
                                             const std::vector<catoptric_livox::Mirror> & mirrors,
                                             const Sophus::SE3d &instrument_pose,
                                             const Sophus::SE3d& intruments_lever_arm,
                                             const Sophus::SE3d& global_pose,
                                             const std::vector<std::pair<double,double>>& angle_mask,
                                             const Sophus::SE3d &scan_pose){
    pcl::PointCloud<pcl::PointXYZL> pc;
    pc.reserve(data.size());
    for (int i=0; i < data.size(); i++)
    {
        Eigen::Vector3d p_local{data[i].X,data[i].Y,data[i].Z};
        double p_local_l = p_local.norm();
        Eigen::Vector3d p_local_norm = p_local / p_local_l;

        int mirror_id = -1;
        if (catoptric_livox::checkIfMasked(p_local, angle_mask)) {
            for (int m = 0; m < mirrors.size(); m++) {
                if (mirrors[m].checkIfRayIntersectMirror(Eigen::Vector3d::Zero(), p_local)) {
                    mirror_id = m;
                }
            }
        }
        if (mirror_id==-1) {continue;}

        //const Eigen::Vector4d plane = catoptric_livox::getPlaneCoefFromSE3(mirrors[mirror_id].getTransformation().matrix());
        const Eigen::Vector4d plane = mirrors[mirror_id].getABCDofPlane();

        Eigen::Vector3d p_glob =global_pose * scan_pose * instrument_pose * intruments_lever_arm * catoptric_livox::getMirroredRay(p_local_norm, p_local_l,plane);
        pcl::PointXYZL p;
        p.getArray3fMap() = p_glob.cast<float>();
        p.label = i;
        pc.push_back(p);
    }
    return pc;
}