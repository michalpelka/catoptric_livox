#pragma once
#include "utils.h"

namespace catoptric_livox{

    struct scan_site{
        std::vector<std::vector<catoptric_livox::DataStream>> datastreams;
        std::vector<Sophus::SE3d> intruments_poses_SE3;
        Sophus::SE3d scan_site_pose_SE3;
        std::vector<Sophus::SE3d> intruments_poses_SE3_initial;
        Sophus::SE3d scan_site_pose_SE3_initial;
    };


    bool getDataFromCSV(std::vector<catoptric_livox::DataStream> &data, const std::string & fn);

    void saveDataToCSV(std::vector<catoptric_livox::DataStream> &data, const std::string & fn);


/***
 * Subsample datasets from Livox Viewer
 */
    std::vector<catoptric_livox::DataStream> subsample(const std::vector<catoptric_livox::DataStream> & input, float leaf_size);

/***
 * Load scan sites from list of files and corespoding turntable's angles. First run cache subsampled data.
 * CSV data is exported pointcloud from Livox Viewer
 * @param filenames list of filename
 * @param angles list of angles
 * @return
 */
    scan_site loadScanSite(std::vector<std::string> & filenames, std::vector<double> angles);

/***
 * Parses a list of input calibration data. Input file name need to consists:
 *  - in first column a rotation angle in degrees.
 *  - in second column a filename to Livox Viewer's CSV
 *  Note : '#' means comented line.
 */
    scan_site loadScanSite(std::string lib_fn);

    void saveCFG(const std::vector<catoptric_livox::Mirror> & mirrors,
                 const Sophus::SE3d& intruments_lever_arm);

    void saveCFG(const std::vector<Sophus::SE3d>& scan_sites);

    void loadCFG(std::vector<catoptric_livox::Mirror> & mirrors,
                 Sophus::SE3d& intruments_lever_arm);

    void loadCFG( std::vector<Sophus::SE3d>& scan_sites );


}