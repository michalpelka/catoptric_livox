#pragma once


#include <ceres/ceres.h>
#include <Sophus/sophus/se3.hpp>

class LocalParameterizationSE3 : public ceres::LocalParameterization {
// adopted from https://github.com/strasdat/Sophus/blob/master/test/ceres/local_parameterization_se3.hpp
public:
    virtual ~LocalParameterizationSE3() {}

    // SE3 plus operation for Ceres
    //
    //  T * exp(x)
    //
    virtual bool Plus(double const* T_raw, double const* delta_raw,
                      double* T_plus_delta_raw) const {
        Eigen::Map<Sophus::SE3d const> const T(T_raw);
        Eigen::Map<Sophus::Vector6d const> const delta(delta_raw);
        Eigen::Map<Sophus::SE3d> T_plus_delta(T_plus_delta_raw);
        T_plus_delta = T * Sophus::SE3d::exp(delta);
        return true;
    }

    // Jacobian of SE3 plus operation for Ceres
    //
    // Dx T * exp(x)  with  x=0
    //
    virtual bool ComputeJacobian(double const* T_raw,
                                 double* jacobian_raw) const {
        Eigen::Map<Sophus::SE3d const> T(T_raw);
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jacobian(
                jacobian_raw);
        jacobian = T.Dx_this_mul_exp_x_at_0();
        return true;
    }

    virtual int GlobalSize() const { return Sophus::SE3d::num_parameters; }

    virtual int LocalSize() const { return Sophus::SE3d::DoF; }
};

class LocalParameterizationPlane : public ceres::LocalParameterization {
public:
    virtual ~LocalParameterizationPlane() {}

    bool Plus(const double* x,
              const double* delta,
              double* x_plus_delta) const {
        x_plus_delta[0] = x[0] + delta[0];
        x_plus_delta[1] = x[1] + delta[1];
        x_plus_delta[2] = x[2] + delta[2];
        x_plus_delta[3] = x[3] + delta[3];
        Eigen::Map<Eigen::Matrix<double, 3, 1>> x_plus_deltap (x_plus_delta);
        x_plus_deltap = x_plus_deltap / x_plus_deltap.norm();
        return true;
    }
    virtual bool ComputeJacobian(double const* T_raw,
                                 double* jacobian_raw) const {
        ceres::MatrixRef(jacobian_raw, 4, 4) = ceres::Matrix::Identity(4, 4);
        return true;
    }

    virtual int GlobalSize() const { return 4; }

    virtual int LocalSize() const { return 4; }
};

template<typename T> Sophus::SE3<T>  getSEFromParams(const T* const params)
{
//    Eigen::Map<const Eigen::Matrix<T,6,1>> eigen_laser_params(params);
//    Sophus::SE3<T> TT = Sophus::SE3<T>::exp(eigen_laser_params);
    Eigen::Map<Sophus::SE3<T> const> const TT(params);

    return TT;
}

struct MirrorOprimizeSE3ithPoseSE3AgainstPlane{
    const Eigen::Vector3d point_1;
    const double x = -0.2;
    MirrorOprimizeSE3ithPoseSE3AgainstPlane(const Eigen::Vector3d& point_1 ):
            point_1(point_1){}

    template <typename T>
    bool operator()(const T* const mirror1_tan, const T* const pose1_tan,
                    T* residuals) const {

        Eigen::Map<Sophus::SE3<T> const> pose1(pose1_tan);
        Eigen::Map<Sophus::SE3<T> const> pose_mirror(mirror1_tan);

        const Eigen::Matrix<T,4,1> mirror_params_tt = catoptric_livox::getPlaneCoefFromSE3(pose_mirror.matrix());

        Eigen::Matrix<T,3,1> dir1 = point_1.cast<T>();
        const T l1 = dir1.norm();
        dir1 = dir1/l1;

        Eigen::Matrix<T,3,1> p1 = catoptric_livox::getMirroredRay<T>(dir1, l1, mirror_params_tt);
        Eigen::Matrix<T,4,1> p1_4; p1_4 << p1.x(), p1.y(), p1.z(), T(1.0);

        Eigen::Matrix<T,4,1> pg1 = pose1.matrix() * p1_4;

        residuals[0] = (pg1.x())-x;
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& point_1) {
        return (new ceres::AutoDiffCostFunction<MirrorOprimizeSE3ithPoseSE3AgainstPlane, 1, 6, 6>(
                new MirrorOprimizeSE3ithPoseSE3AgainstPlane(point_1)));
//        return (new ceres::NumericDiffCostFunction<MirrorOprimizeABCDWithPose, ceres::CENTRAL, 3, 4, 4, 6, 6>(
//                new MirrorOprimizeABCDWithPose(point_1,point_target)));

//        return (new ceres::NumericDiffCostFunction<PlaneAligmentError, ceres::CENTRAL, 4, 6, 6>(
//                new PlaneAligmentError(plane_1,plane_2,pose_1,pose_2)));
    }
};


struct MirrorOprimizeSE3ithPoseSE3{
    const Eigen::Vector3d point_1;
    const Eigen::Vector3d point_target;

    MirrorOprimizeSE3ithPoseSE3(const Eigen::Vector3d& point_1, const Eigen::Vector3d& point_target) :
            point_1(point_1), point_target(point_target){}

    template <typename T>
    bool operator()(const T* const mirror1_tan, const T* const pose1_tan,
                    T* residuals) const {

        const Eigen::Matrix<T,3,1> point_target_t = point_target.cast<T>();

        Eigen::Map<Sophus::SE3<T> const>  pose1(pose1_tan);
        Eigen::Map<Sophus::SE3<T> const>  pose_mirror(mirror1_tan);

        const Eigen::Matrix<T,4,1> mirror_params_tt = catoptric_livox::getPlaneCoefFromSE3(pose_mirror.matrix());

        Eigen::Matrix<T,3,1> dir1 = point_1.cast<T>();
        const T l1 = dir1.norm();
        dir1 = dir1/l1;

        Eigen::Matrix<T,3,1> p1 = catoptric_livox::getMirroredRay<T>(dir1, l1, mirror_params_tt);
        Eigen::Matrix<T,4,1> p1_4; p1_4 << p1.x(), p1.y(), p1.z(), T(1.0);

        Eigen::Matrix<T,4,1> pg1 = pose1.matrix() * p1_4;

        residuals[0] = (pg1.x()-point_target_t.x());
        residuals[1] = (pg1.y()-point_target_t.y());
        residuals[2] = (pg1.z()-point_target_t.z());
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& point_1, const Eigen::Vector3d& point_target) {
        return (new ceres::AutoDiffCostFunction<MirrorOprimizeSE3ithPoseSE3, 3, 6, 6>(
                new MirrorOprimizeSE3ithPoseSE3(point_1,point_target)));
//        return (new ceres::NumericDiffCostFunction<MirrorOprimizeABCDWithPose, ceres::CENTRAL, 3, 4, 4, 6, 6>(
//                new MirrorOprimizeABCDWithPose(point_1,point_target)));

//        return (new ceres::NumericDiffCostFunction<PlaneAligmentError, ceres::CENTRAL, 4, 6, 6>(
//                new PlaneAligmentError(plane_1,plane_2,pose_1,pose_2)));
    }
};


struct MirrorOprimizeABCDWithPose{
    const Eigen::Vector3d point_1;
    const Eigen::Vector3d point_target;
    const Sophus::SE3d instrument_pos;
    MirrorOprimizeABCDWithPose(const Eigen::Vector3d& point_1, const Eigen::Vector3d& point_target, const Sophus::SE3d &instrument_pos) :
            point_1(point_1), point_target(point_target), instrument_pos(instrument_pos){}

    template <typename T>
    bool operator()(const T* const mirror1params_data, const T* const pose1tan, const T* const lever_arm_tan,
                    T* residuals) const {

        const Eigen::Matrix<T,3,1> point_target_t = point_target.cast<T>();

        Eigen::Map<Sophus::SE3<T> const>  pose1(pose1tan);
        //Sophus::SE3<T> pose1 = instrument_pos.cast<T>();
        Eigen::Map<Sophus::SE3<T> const>  lever_arm(lever_arm_tan);


        Eigen::Map<const Eigen::Matrix<T,4,1>> mirror1params(mirror1params_data);

        Eigen::Matrix<T,3,1> dir1 = point_1.cast<T>();
        const T l1 = dir1.norm();
        dir1 = dir1/l1;

        Eigen::Matrix<T,3,1> p1 = catoptric_livox::getMirroredRay<T>(dir1, l1, mirror1params);
        Eigen::Matrix<T,4,1> p1_4; p1_4 << p1.x(), p1.y(), p1.z(), T(1.0);
        // TODO LeverArm
        Eigen::Matrix<T,4,1> pg1 = pose1.matrix()  * lever_arm.matrix() * p1_4;


        residuals[0] = (pg1.x()-point_target_t.x());
        residuals[1] = (pg1.y()-point_target_t.y());
        residuals[2] = (pg1.z()-point_target_t.z());
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& point_1, const Eigen::Vector3d& point_target,
                                       const Sophus::SE3d &instrument_pose) {
        return (new ceres::AutoDiffCostFunction<MirrorOprimizeABCDWithPose, 3, 4, Sophus::SE3d::num_parameters, Sophus::SE3d::num_parameters>(
                new MirrorOprimizeABCDWithPose(point_1,point_target,instrument_pose)));
//        return (new ceres::NumericDiffCostFunction<MirrorOprimizeABCDWithPose, ceres::CENTRAL, 3, 4, 4, 6, 6>(
//                new MirrorOprimizeABCDWithPose(point_1,point_target)));

//        return (new ceres::NumericDiffCostFunction<PlaneAligmentError, ceres::CENTRAL, 4, 6, 6>(
//                new PlaneAligmentError(plane_1,plane_2,pose_1,pose_2)));
    }
};

struct MirrorOprimizeABCDWithPoseWithSitePose{
    const Eigen::Vector3d point_1;
    const Eigen::Vector3d point_target;
    const Sophus::SE3d instrument_pos;
    MirrorOprimizeABCDWithPoseWithSitePose(const Eigen::Vector3d& point_1, const Eigen::Vector3d& point_target,
                                           const Sophus::SE3d& instrument_pos) :
            point_1(point_1), point_target(point_target), instrument_pos(instrument_pos){}

    template <typename T>
    bool operator()(const T* const mirror1params_data, const T* const pose1tan, const T* const lever_arm_tan,
                    const T* const scan_site1tan, const T* const global_pose_tan,
                    T* residuals) const {

        const Eigen::Matrix<T,3,1> point_target_t = point_target.cast<T>();

        Eigen::Map<Sophus::SE3<T> const>  pose1(pose1tan);
        //Sophus::SE3<T> pose1 = instrument_pos.cast<T>();
        Eigen::Map<Sophus::SE3<T> const>  site1(scan_site1tan);
        Eigen::Map<Sophus::SE3<T> const>  lever_arm(lever_arm_tan);
        Eigen::Map<Sophus::SE3<T> const>  global_pose(global_pose_tan);


        Eigen::Map<const Eigen::Matrix<T,4,1>> mirror1params(mirror1params_data);

        Eigen::Matrix<T,3,1> dir1 = point_1.cast<T>();
        const T l1 = dir1.norm();
        dir1 = dir1/l1;

        Eigen::Matrix<T,3,1> p1 = catoptric_livox::getMirroredRay<T>(dir1, l1, mirror1params);
        Eigen::Matrix<T,4,1> p1_4; p1_4 << p1.x(), p1.y(), p1.z(), T(1.0);
        // TODO LeverArm
        Eigen::Matrix<T,4,1> pg1 =global_pose.matrix() * site1.matrix() * pose1.matrix()  * lever_arm.matrix() * p1_4;


        residuals[0] = (pg1.x()-point_target_t.x());
        residuals[1] = (pg1.y()-point_target_t.y());
        residuals[2] = (pg1.z()-point_target_t.z());
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& point_1, const Eigen::Vector3d& point_target,
                                       const Sophus::SE3d& instrument_pos) {
        return (new ceres::AutoDiffCostFunction<MirrorOprimizeABCDWithPoseWithSitePose, 3, 4,
                Sophus::SE3d::num_parameters,
                Sophus::SE3d::num_parameters,
                Sophus::SE3d::num_parameters,
                Sophus::SE3d::num_parameters>(
                new MirrorOprimizeABCDWithPoseWithSitePose(point_1,point_target, instrument_pos)));
//        return (new ceres::NumericDiffCostFunction<MirrorOprimizeABCDWithPose, ceres::CENTRAL, 3, 4, 4, 6, 6>(
//                new MirrorOprimizeABCDWithPose(point_1,point_target)));

//        return (new ceres::NumericDiffCostFunction<PlaneAligmentError, ceres::CENTRAL, 4, 6, 6>(
//                new PlaneAligmentError(plane_1,plane_2,pose_1,pose_2)));
    }
};


struct RelativePose{
    const Sophus::SE3d odom1;
    const Sophus::SE3d odom2;
    const Sophus::SE3d icrement_pose_measured;

    RelativePose(const Eigen::Affine3d& _odom1, const Eigen::Affine3d& _odom2 ) :
            odom1(_odom1.rotation(), _odom1.translation()),odom2(_odom2.rotation(), _odom2.translation()),
            icrement_pose_measured(odom1.inverse()*odom2)
    {}

    template <typename T>
    bool operator()(const T* const odom1tan, const T* const odom2tan,
                    T* residuals) const {

        Sophus::SE3<T> icrement_pose_measured_sophus(icrement_pose_measured.matrix().cast<T>());

        Eigen::Map<Sophus::SE3<T> const>  odom1(odom1tan);
        Eigen::Map<Sophus::SE3<T> const>  odom2(odom2tan);

        Sophus::SE3<T> increment = (odom1.inverse()*odom2);
        Eigen::Map<Eigen::Matrix<T,6,1>> residuals_map(residuals);
        residuals_map = increment.log() - icrement_pose_measured_sophus.log();

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Affine3d& odom1,const Eigen::Affine3d& odom2) {
        return (new ceres::AutoDiffCostFunction<RelativePose, Sophus::SE3d::num_parameters,
                Sophus::SE3d::num_parameters, Sophus::SE3d::num_parameters >(
                new RelativePose(odom1, odom2)));
    }
};
