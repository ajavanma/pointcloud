/**
 * @file asc_report.cpp
 *
 * @brief Processes point cloud data from .asc files and generates a report for each file.
 *
 * This file contains a C++ implementation for processing .asc files containing point cloud data.
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
#include <Eigen/Dense>
#include <limits>
#include <cmath>

namespace fs = std::filesystem;

void process_asc_file(const fs::path &input_file, const fs::path &log_file_path)
{
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> colors;
    std::vector<int> labels;
    std::vector<Eigen::Vector3d> normals;

    std::ifstream file(input_file);
    std::string line;

    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        std::string column;
        std::vector<std::string> columns;

        while (std::getline(ss, column, ';'))
        {
            columns.push_back(column);
        }

        try
        {
            Eigen::Vector3d point(std::stod(columns[0]), std::stod(columns[1]), std::stod(columns[2]));
            Eigen::Vector3d color(std::stod(columns[3]), std::stod(columns[4]), std::stod(columns[5]));
            int label = std::stoi(columns[6]);
            Eigen::Vector3d normal(std::stod(columns[7]), std::stod(columns[8]), std::stod(columns[9]));

            points.push_back(point);
            colors.push_back(color);
            labels.push_back(label);
            normals.push_back(normal);
        }
        catch (const std::invalid_argument &e)
        {
            std::cerr << "Invalid argument: " << e.what() << " in line: " << line << std::endl;
        }
        catch (const std::out_of_range &e)
        {
            std::cerr << "Out of range: " << e.what() << " in line: " << line << std::endl;
        }
    }

    file.close();

    Eigen::Matrix<double, Eigen::Dynamic, 3> points_array(points.size(), 3);
    Eigen::Matrix<double, Eigen::Dynamic, 3> colors_array(colors.size(), 3);
    Eigen::Matrix<int, Eigen::Dynamic, 1> labels_array(labels.size(), 1);
    Eigen::Matrix<double, Eigen::Dynamic, 3> normals_array(normals.size(), 3);

    for (size_t i = 0; i < points.size(); ++i)
    {
        points_array.row(i) = points[i];
        colors_array.row(i) = colors[i];
        labels_array(i, 0) = labels[i];
        normals_array.row(i) = normals[i];
    }

    Eigen::Matrix<double, Eigen::Dynamic, 10> combined_array(points.size(), 10);
    combined_array << points_array, colors_array, labels_array.cast<double>(), normals_array;

    std::vector<int> counter_zeros(10, 0);
    std::vector<int> counter_nans(10, 0);
    std::vector<int> counter_infs(10, 0);
    Eigen::VectorXd min_values = Eigen::VectorXd::Constant(10, std::numeric_limits<double>::quiet_NaN());
    Eigen::VectorXd max_values = Eigen::VectorXd::Constant(10, std::numeric_limits<double>::quiet_NaN());

    for (size_t j = 0; j < combined_array.cols(); ++j)
    {
        Eigen::Array<bool, Eigen::Dynamic, 1> is_zero = combined_array.col(j).array() == 0.0;
        counter_zeros[j] = is_zero.count();

        Eigen::Array < bool, Eigen for (size_t j = 0; j < combined_array.cols(); ++j)
        {
            Eigen::Array<bool, Eigen::Dynamic, 1> is_zero = combined_array.col(j).array() == 0.0;
            counter_zeros[j] = is_zero.count();

            Eigen::Array<bool, Eigen::Dynamic, 1> is_nan = combined_array.col(j).array().isNaN();
            counter_nans[j] = is_nan.count();

            Eigen::Array<bool, Eigen::Dynamic, 1> is_inf = combined_array.col(j).array().isInf();
            counter_infs[j] = is_inf.count();

            min_values[j] = combined_array.col(j).minCoeff();
            max_values[j] = combined_array.col(j).maxCoeff();
        }

        // Write the report to a log file
        std::ofstream log_file(log_file_path);
        log_file << "File: " << input_file << std::endl;

        std::vector<std::string> field_names = {"x", "y", "z", "r", "g", "b", "Classification", "normal_x", "normal_y", "normal_z"};

        for (size_t i = 0; i < field_names.size(); ++i)
        {
            log_file << "Number of zeros found in " << field_names[i] << ": " << counter_zeros[i] << std::endl;
            log_file << "Number of NaNs found in " << field_names[i] << ": " << counter_nans[i] << std::endl;
            log_file << "Number of infinite values found in " << field_names[i] << ": " << counter_infs[i] << std::endl;
            if (!std::isnan(min_values[i]) && !std::isnan(max_values[i]))
            {
                log_file << "Min and max values for " << field_names[i] << ": " << min_values[i] << ", " << max_values[i] << std::endl;
            }
        }

        log_file << "Done writing to file" << std::endl;
        log_file.close();
    }

    int main()
    {
        constexpr fs::path asc_files_path = "../data/old";
        constexpr fs::path output_logs_path = "../logs";
        for (const auto &entry : fs::directory_iterator(asc_files_path))
        {
            if (entry.path().extension() == ".asc")
            {
                fs::path input_file = entry.path();
                fs::path log_file_path = output_logs_path / (input_file.stem().string() + "_asc_log.log");

                process_asc_file(input_file, log_file_path);
            }
        }

        return 0;
    }
