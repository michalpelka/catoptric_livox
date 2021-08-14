#pragma once

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Sophus/sophus/se3.hpp>

#include <fstream>
#include <iostream>
#include <vector>


namespace catoptric_livox{

    Eigen::Affine3d orthogonize(const Eigen::Affine3d& p );

    template <typename T> Eigen::Matrix<T,4,1>transformPlaneBySE3(const Eigen::Matrix<T,4,1>& plane, const Eigen::Matrix<T,4,4>& SE3){

        Eigen::Matrix<T,4,1> r = (SE3.inverse()).transpose() * plane;
        return r;
    }

    template <typename T> Eigen::Matrix<T,4,1>getPlaneCoefFromSE3(const Eigen::Matrix<T,4,4>& SE3){
        const T a = -SE3(0,2);
        const T b = -SE3(1,2);
        const T c = -SE3(2,2);
        const T d = -SE3(0,3) * a - SE3(1,3) * b - SE3(2,3) * c;
        return Eigen::Matrix<T,4,1> {a,b,c,d};
    }
        
    template <typename T> Eigen::Matrix<T,3,1>getMirroredRayIntersection(const Eigen::Matrix<T,3,1>& dir, T ray_length, const Eigen::Matrix<T,4,1>& plane)
    {

        Eigen::Matrix<T,3,1> np {plane.x(), plane.y(), plane.z()};
        np= np /np.norm();
        Eigen::Matrix<T,3,1> ndir = dir / dir.norm();
        const T a = np.x() * ndir.x() + np.y() * ndir.y() + np.z() * ndir.z();
        const Eigen::Matrix<T,3,1> intersection = - ndir *(plane.w()/a);
        return intersection;
    }


    template <typename T> Eigen::Matrix<T,3,1>getMirroredRay(const Eigen::Matrix<T,3,1>& dir, T ray_length, const Eigen::Matrix<T,4,1>& plane)
    {

        Eigen::Matrix<T,3,1> np {plane.x(), plane.y(), plane.z()};
        np= np /np.norm();
        Eigen::Matrix<T,3,1> ndir = dir / dir.norm();

        const T a = np.x() * ndir.x() + np.y() * ndir.y() + np.z() * ndir.z();
        const Eigen::Matrix<T,3,1> intersection = - ndir *(plane.w()/a);
        const Eigen::Matrix<T,3,1> rd= ndir - T(2.0)*(ndir.dot(np))*np;
        const T ll = ray_length - intersection.norm();
        return -intersection + rd * ll;
    }

    enum DrawType{
        LASER_WITH_MIRROR_COLOR=1
    };
    const std::vector<Eigen::Vector3f> mirror_colors{
            {1.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 1.0f},
            {1.0f, 0.0f, 1.0f},
            {1.0f, 1.0f, 0.0f},
            {0.0f, 1.0f, 1.0f},
    };

    struct DataStream{
        double X;
        double Y;
        double Z;
        double Reflectivity;

        double Version;
        double SlotID;
        double LiDAR_Index;
        double Rsvd;
        double ErrorCode;
        double TimestampType;
        double DataType;
        double Timestamp;

        double Tag;
        double Ori_x;
        double Ori_y;
        double Ori_z;
        double Ori_radius;
        double Ori_theta;
        double Ori_phi;
    };

    class Mirror{
    public:
        static unsigned int getNumberOfParams(){
            return 4;
        }

        Mirror()=default;
        Mirror(const Mirror& other)=default;
        Mirror& operator=(const Mirror& other )=default;
        Mirror(Mirror&& other) = default;
        Mirror& operator=(Mirror&& other) = default;

        Mirror(const std::vector<Eigen::Vector3d>& segments);

        bool checkIfRayIntersectMirror(const Eigen::Vector3d& origin,const Eigen::Vector3d& dir) const;

        const Eigen::Vector3d &getCentroid() const {
            return centroid;
        }

        const Eigen::Affine3d &getTransformation() const {
            return transformation;
        }

        const Eigen::Affine3d &getTransformationInv() const {
            return transformation_inv;
        }

        const std::vector<Eigen::Vector3d> getSegments() const {
            return segments;
        }

        const Eigen::Vector4d &getABCDofPlane() const {
            return plane_coeff;
        }
        const Eigen::Vector4d &getABCDofPlaneInit() const {
            return plane_coeff_init;
        }

        Eigen::Vector4d &getABCDofPlane()  {
            return plane_coeff;
        }

        void reset();

        float max_angle{45};
        float min_angle{0};
    private:
        Eigen::Vector4d plane_coeff;
        Eigen::Vector4d plane_coeff_init;
        Eigen::Vector3d centroid;
        Eigen::Affine3d transformation;
        Eigen::Affine3d transformation_inv;
        std::vector<Eigen::Vector3d> segments;
        std::vector<Eigen::Vector2d> segments2D;
    };

    std::vector<Mirror> loadMirrorFromPLY(const std::string& filename);

    void updateDrawingBuffer(const std::vector<Mirror> & mirrors, std::vector<float>& data_v, std::vector<unsigned int>& data_i  );

    void updateDrawingBuffer(const std::vector<std::vector<catoptric_livox::DataStream>> &data,
                             const std::vector<Mirror> & mirrors,
                             const std::vector<Sophus::SE3d> &instrument_poses,
                             const Sophus::SE3d &intruments_lever_arm,
                             const Sophus::SE3d &scan_pose,
                             const Sophus::SE3d &global_pose,
                             const std::vector<std::pair<double,double>>& angle_mask,
                             DrawType type,
                             std::vector<float>& data_v, std::vector<unsigned int>& data_i,
                             int skip);

    bool checkIfMasked(const Eigen::Vector3d& dir, const std::vector<std::pair<double,double>>& angle_mask);

    pcl::PointCloud<pcl::PointXYZL> getPoinCloud(const std::vector<catoptric_livox::DataStream> &data,
                                                 const std::vector<catoptric_livox::Mirror> & mirrors,
                                                 const Sophus::SE3d &instrument_pose,
                                                 const Sophus::SE3d& intruments_lever_arm,
                                                 const Sophus::SE3d& global_pose,
                                                 const std::vector<std::pair<double,double>>& angle_mask,
                                                 const Sophus::SE3d &scan_pose);

}