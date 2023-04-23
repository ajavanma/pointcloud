/**
 * @file pcd_report.cpp
 *
 * @brief Process point cloud data (PCD) files and generates a report for each file for analyzing.
 *
 * This file contains a C++ implementation for processing .pcd files containing point cloud data.
 * Each point cloud consists of point coordinates (x, y, z), color values (r, g, b), a classification label,
 * and normal vectors (normal_x, normal_y, normal_z). The program reads each file, processes the data,
 * and generates a report including the number of zeros, NaNs, infinite values, and the min/max values
 * for each field.
 *
 * This code assumes the raw data are located in the "../data/raw" directory.
 * Report is saved in "../logs" directory.
 *
 * @author Arash javanmardi
 * @date 2023-04-20
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#if defined(__APPLE__)
    #include <malloc/malloc.h>
#endif
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;

/**
 * @brief Processes a single PCD file, generating a report with statistics.
 *
 * @param input_file Path to the input PCD file to be processed.
 * @param log_file_path Path where the generated report will be saved.
 */

void process_pcd_file(const fs::path &input_file, const fs::path &log_file_path)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(input_file.string(), *point_cloud) == -1)
    {
        std::ofstream log_file(log_file_path);
        log_file << "Error loading PCD file '" << input_file << "'\n";
        log_file.close();
        return;
    }

    const std::vector<std::string> field_names = {"x", "y", "z", "r", "g", "b", "normal_x", "normal_y", "normal_z"};
    int num_fields = field_names.size();

    std::vector<int> counter_zeros(num_fields, 0);
    std::vector<int> counter_nans(num_fields, 0);
    std::vector<int> counter_infs(num_fields, 0);
    Eigen::VectorXd min_values = Eigen::VectorXd::Constant(num_fields, std::numeric_limits<double>::quiet_NaN());
    Eigen::VectorXd max_values = Eigen::VectorXd::Constant(num_fields, std::numeric_limits<double>::quiet_NaN());

    for (const auto &point : point_cloud->points)
    {
        const std::vector<double> point_data = {point.x, point.y, point.z, point.r, point.g, point.b, point.normal_x, point.normal_y, point.normal_z};

        for (size_t i = 0; i < point_data.size(); ++i)
        {
            double value = point_data[i];

            if (value == 0.0)
            {
                counter_zeros[i]++;
            }

            if (std::isnan(value))
            {
                counter_nans[i]++;
            }
            else
            {
                if (std::isnan(min_values[i]))
                {
                    min_values[i] = value;
                }
                else
                {
                    min_values[i] = std::min(min_values[i], value);
                }

                if (std::isnan(max_values[i]))
                {
                    max_values[i] = value;
                }
                else
                {
                    max_values[i] = std::max(max_values[i], value);
                }
            }

            if (std::isinf(value))
            {
                counter_infs[i]++;
            }
        }
    }

    std::ofstream log_file(log_file_path);
    log_file << "PCD file information for '" << input_file << "':\n";
    log_file << point_cloud->size() << " points\n";

    for (size_t i = 0; i < field_names.size(); ++i)
    {
        log_file << "Number of zeros found in " << field_names[i] << ": " << counter_zeros[i] << '\n';
        log_file << "Number of NaNs found in " << field_names[i] << ": " << counter_nans[i] << '\n';
        log_file << "Number of infinite values found in " << field_names[i] << ": " << counter_infs[i] << '\n';
        if (!std::isnan(min_values[i]) && !std::isnan(max_values[i]))
        {
            log_file << "Min and max values for " << field_names[i] << ": " << min_values[i] << ", " << max_values[i] << '\n';
        }
    }
    log_file << "Done writing to file" << '\n';
    log_file.close();
}

int main()
{
    constexpr fs::path pcd_files_path = "/Users/arash/Documents/github/pointclouds/data/raw";
    constexpr fs::path output_logs_path = "/Users/arash/Documents/github/pointclouds/logs";
    for (const auto &entry : fs::directory_iterator(pcd_files_path))
    {
        if (entry.path().extension() == ".pcd")
        {
            fs::path input_file = entry.path();
            fs::path log_file_path = output_logs_path / (input_file.stem().string() + "_report.log");

            process_pcd_file(input_file, log_file_path);
        }
    }

    return 0;
}
