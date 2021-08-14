#include "utils_io.h"

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

bool catoptric_livox::getDataFromCSV(std::vector<catoptric_livox::DataStream> &data, const std::string & filename)
{
    std::cout << "opening " << filename << std::endl;
    data.clear();
    std::ifstream f;
    f.open(filename.c_str());
    if(f.is_open() && f.good()) {
        std::string s;
        //first line
        //indexTrajectoryPose,timestamp,indexPointBeginInclusive,indexPointEndExclusive,angleRotatingUnitRad,trajectoryX,trajectoryY,trajectoryZ,trajectoryXangleRad,trajectoryYangleRad,trajectoryZangleRad
        getline(f,s);
        int counter = 0;
        while(!f.eof())	{
            getline(f,s);
            std::vector<std::string> strs;
            boost::split(strs,s,boost::is_any_of(","));
            if(strs.size() == 19){
                DataStream ds;
                std::istringstream(strs[0]) >> ds.Version;
                std::istringstream(strs[1]) >> ds.SlotID;
                std::istringstream(strs[2]) >> ds.LiDAR_Index;
                std::istringstream(strs[3]) >> ds.Rsvd;
                std::istringstream(strs[4]) >> ds.ErrorCode;
                std::istringstream(strs[5]) >> ds.TimestampType;
                std::istringstream(strs[6]) >> ds.DataType;
                std::istringstream(strs[7]) >> ds.Timestamp;
                std::istringstream(strs[8]) >> ds.X;
                std::istringstream(strs[9]) >> ds.Y;
                std::istringstream(strs[10]) >> ds.Z;
                std::istringstream(strs[11]) >> ds.Reflectivity;
                std::istringstream(strs[12]) >> ds.Tag;
                std::istringstream(strs[13]) >> ds.Ori_x;
                std::istringstream(strs[14]) >> ds.Ori_y;
                std::istringstream(strs[15]) >> ds.Ori_z;
                std::istringstream(strs[15]) >> ds.Ori_radius;
                std::istringstream(strs[15]) >> ds.Ori_theta;
                std::istringstream(strs[15]) >> ds.Ori_phi;

                data.push_back(ds);
            }else{
                std::cout << "ERROR line " << counter << " file " << filename <<": record.length!=19 it is " <<  strs.size() << " check" << std::endl;
                std::cout << "Version,Slot ID,LiDAR Index,Rsvd,Error Code,Timestamp Type,Data Type,Timestamp,X,Y,Z,Reflectivity,Tag,Ori_x,Ori_y,Ori_z,Ori_radius,Ori_theta,Ori_phi" << std::endl;
                break;
            }
            counter++;
        }
        f.close();
    }else{
        std::cerr << "cannot open "<< filename <<std::endl;
        return false;
    }
    return true;
}

void catoptric_livox::saveDataToCSV(std::vector<catoptric_livox::DataStream> &data, const std::string & fn){
    std::ofstream f;
    f.open(fn);
    if(f.good()) {
        f << "Version,Slot ID,LiDAR Index,Rsvd,Error Code,Timestamp Type,Data Type,Timestamp,X,Y,Z,Reflectivity,Tag,Ori_x,Ori_y,Ori_z,Ori_radius,Ori_theta,Ori_phi" << std::endl;
        for (const auto& ds : data){
            f << ds.Version<< ",";
            f << ds.SlotID<< ",";
            f << ds.LiDAR_Index<< ",";
            f << ds.Rsvd<< ",";
            f << ds.ErrorCode<< ",";
            f << ds.TimestampType<< ",";
            f << ds.DataType<< ",";
            f << ds.Timestamp<< ",";
            f << ds.X<< ",";
            f << ds.Y<< ",";
            f << ds.Z<< ",";
            f << ds.Reflectivity<< ",";
            f << ds.Tag<< ",";
            f << ds.Ori_x<< ",";
            f << ds.Ori_y<< ",";
            f << ds.Ori_z<< ",";
            f << ds.Ori_radius<< ",";
            f << ds.Ori_theta<< ",";
            f << ds.Ori_phi;
            f << "\n";
        }
        f.flush();
        f.close();
    }else{
        throw("file is not opened");
    }
}

std::vector<catoptric_livox::DataStream> catoptric_livox::subsample(const std::vector<catoptric_livox::DataStream> & input, float leaf_size)
{
    pcl::PointCloud<pcl::PointXYZL>::Ptr pre_filteredcloud (new pcl::PointCloud<pcl::PointXYZL>);
    pcl::PointCloud<pcl::PointXYZL>::Ptr post_filteredcloud_m (new pcl::PointCloud<pcl::PointXYZL>);
    pcl::PointCloud<pcl::PointXYZL>::Ptr post_filteredcloud_m2 (new pcl::PointCloud<pcl::PointXYZL>);

    pcl::PointCloud<pcl::PointXYZL> post_filteredcloud;

    pre_filteredcloud->resize(input.size());
    for (int i =0; i < input.size(); i++){
        const auto & p1 = input[i];
        pcl::PointXYZL p2;
        p2.label = i;
        p2.label = i;
        p2.getArray3fMap() = Eigen::Vector3f{static_cast<float>(p1.X),
                                             static_cast<float>(p1.Y),
                                             static_cast<float>(p1.Z)};
        (*pre_filteredcloud)[i] = p2;
    }
    pcl::VoxelGrid<pcl::PointXYZL> sor0;
    sor0.setInputCloud (pre_filteredcloud);
    sor0.setLeafSize (0.05,0.05,0.025);
    sor0.filter (*post_filteredcloud_m);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZL> sor2;
    sor2.setMeanK (200);
    sor2.setStddevMulThresh (1.0);
    sor2.setInputCloud (post_filteredcloud_m);
    sor2.filter (*post_filteredcloud_m2);


    pcl::VoxelGrid<pcl::PointXYZL> sor1;
    sor1.setInputCloud (post_filteredcloud_m2);
    sor1.setLeafSize (leaf_size,leaf_size,leaf_size/2);
    sor1.filter (post_filteredcloud);

    std::vector<catoptric_livox::DataStream> ds2;
    ds2.resize(post_filteredcloud.size());

    for (int i =0; i < ds2.size(); i++){
        const auto & p1 = post_filteredcloud[i];
        const auto & p2 = input.at(p1.label);
        ds2[i] = p2;
    }
    return ds2;
}

catoptric_livox::scan_site catoptric_livox::loadScanSite(std::vector<std::string> & filenames, std::vector<double> angles){
    assert(filenames.size() == angles.size());
    catoptric_livox::scan_site site;
    for ( int i =0; i < filenames.size(); i++)
    {
        const std::string &fn = filenames[i];
        std::string fn_sub = filenames[i];
        fn_sub.replace(fn_sub.size()-4, 4, "_sub2.csv");
        std::vector<catoptric_livox::DataStream> ds;
        // try to open subsampled file first
        if (catoptric_livox::getDataFromCSV(ds, fn_sub))
        {
            catoptric_livox::getDataFromCSV(ds, fn_sub);
            site.datastreams.push_back(ds);
        }else{
            catoptric_livox::getDataFromCSV(ds, fn);
            std::vector<catoptric_livox::DataStream> ds_sub = subsample(ds, 0.2);
            site.datastreams.push_back(ds_sub);
            catoptric_livox::saveDataToCSV(ds_sub, fn_sub);
        }
        auto mat = Eigen::Affine3d::Identity();
        mat.rotate(Eigen::AngleAxisd(-M_PI*angles[i]/ 180.0, Eigen::Vector3d::UnitX()));
        site.intruments_poses_SE3.push_back(Sophus::SE3d(mat.matrix()));
    }
    site.intruments_poses_SE3_initial = site.intruments_poses_SE3;
    site.scan_site_pose_SE3_initial = site.scan_site_pose_SE3;
    return site;
}

catoptric_livox::scan_site catoptric_livox::loadScanSite(std::string lib_fn){
    std::ifstream f;
    std::vector<double> angles;
    std::vector<std::string> fns;
    std::vector<catoptric_livox::scan_site> scan_sites;
    f.open(lib_fn.c_str());
    auto p = boost::filesystem::path(lib_fn);
    if(f.is_open() && f.good()) {
        while(!f.eof()) {
            std::string line;
            getline(f, line);
            if (line.empty()) continue;
            if (line[0] == '#') continue;
            std::stringstream os(line);
            float angle;
            std::string fn;
            os >> angle;
            os >> fn;
            angles.emplace_back(angle);
            fns.emplace_back((p.parent_path()/fn).string());
        }
    }
    return catoptric_livox::loadScanSite(fns,angles);;
}

void catoptric_livox::saveCFG(const std::vector<catoptric_livox::Mirror> & mirrors,
                              const Sophus::SE3d& intruments_lever_arm)
{
    boost::property_tree::ptree pt;
    pt.put("mirrors_size", mirrors.size());
    for (int i =0; i < mirrors.size(); i++ )
    {
        pt.put("mirror_"+std::to_string(i)+"_a", mirrors[i].getABCDofPlane().x());
        pt.put("mirror_"+std::to_string(i)+"_b", mirrors[i].getABCDofPlane().y());
        pt.put("mirror_"+std::to_string(i)+"_c", mirrors[i].getABCDofPlane().z());
        pt.put("mirror_"+std::to_string(i)+"_d", mirrors[i].getABCDofPlane().w());
    }
    const auto lsed = intruments_lever_arm.log();
    pt.put("lever_arm_x", lsed(0));
    pt.put("lever_arm_y", lsed(1));
    pt.put("lever_arm_z", lsed(2));
    pt.put("lever_arm_rx", lsed(3));
    pt.put("lever_arm_ry", lsed(4));
    pt.put("lever_arm_rz", lsed(5));
    std::ofstream ofss("config.ini");
    boost::property_tree::write_ini(ofss, pt);
    ofss.close();
}
void catoptric_livox::loadCFG(std::vector<catoptric_livox::Mirror> & mirrors,
                              Sophus::SE3d& intruments_lever_arm)
{
    boost::property_tree::ptree pt;
    std::ifstream ifss("config.ini");
    boost::property_tree::read_ini(ifss, pt);
    for (int i =0; i < mirrors.size(); i++ )
    {
        mirrors[i].getABCDofPlane().x() = pt.get<double>("mirror_"+std::to_string(i)+"_a");
        mirrors[i].getABCDofPlane().y() = pt.get<double>("mirror_"+std::to_string(i)+"_b");
        mirrors[i].getABCDofPlane().z() = pt.get<double>("mirror_"+std::to_string(i)+"_c");
        mirrors[i].getABCDofPlane().w() = pt.get<double>("mirror_"+std::to_string(i)+"_d");
    }

    Eigen::Matrix<double,6,1> lsed;
    lsed(0) = pt.get<double>("lever_arm_x");
    lsed(1) = pt.get<double>("lever_arm_y");
    lsed(2) = pt.get<double>("lever_arm_z");
    lsed(3) = pt.get<double>("lever_arm_rx");
    lsed(4) = pt.get<double>("lever_arm_ry");
    lsed(5) = pt.get<double>("lever_arm_rz");
    intruments_lever_arm = Sophus::SE3d::exp(lsed);
}


void catoptric_livox::saveCFG(const std::vector<Sophus::SE3d>& scan_sites)
{
    boost::property_tree::ptree pt;
    pt.put("scan_sites_size", scan_sites.size());
    for (int i =0; i < scan_sites.size(); i++ )
    {
        const auto t = scan_sites[i].log();
        pt.put("scan_site_"+std::to_string(i)+"_x", t[0]);
        pt.put("scan_site_"+std::to_string(i)+"_y", t[1]);
        pt.put("scan_site_"+std::to_string(i)+"_z", t[2]);
        pt.put("scan_site_"+std::to_string(i)+"_rx", t[3]);
        pt.put("scan_site_"+std::to_string(i)+"_ry", t[4]);
        pt.put("scan_site_"+std::to_string(i)+"_rz", t[5]);
    }
    std::ofstream ofss("config_scan_sites.ini");
    boost::property_tree::write_ini(ofss, pt);
    ofss.close();
}

void catoptric_livox::loadCFG( std::vector<Sophus::SE3d>& scan_sites)
{
    boost::property_tree::ptree pt;
    std::ifstream ifss("config_scan_sites.ini");
    boost::property_tree::read_ini(ifss, pt);
    scan_sites.resize( pt.get<unsigned int>("scan_sites_size"));
    for (int i =0; i < scan_sites.size(); i++) {
        Eigen::Matrix<double, 6, 1> lsed;
        lsed(0) = pt.get<double>("scan_site_"+std::to_string(i)+"_x");
        lsed(1) = pt.get<double>("scan_site_"+std::to_string(i)+"_y");
        lsed(2) = pt.get<double>("scan_site_"+std::to_string(i)+"_z");
        lsed(3) = pt.get<double>("scan_site_"+std::to_string(i)+"_rx");
        lsed(4) = pt.get<double>("scan_site_"+std::to_string(i)+"_ry");
        lsed(5) = pt.get<double>("scan_site_"+std::to_string(i)+"_rz");
        scan_sites[i] = Sophus::SE3d::exp(lsed);
    }
}