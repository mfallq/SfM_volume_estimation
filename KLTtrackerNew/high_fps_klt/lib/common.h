#pragma once
#include <string>
#include <vector>
//#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <memory>
#include "Matrix.h"
#include "pose.h"
//#include "vis/Visualization.h"

using uint = unsigned int;

namespace cvl {
	/*
    template<typename T, int Rows, int Cols>
    Eigen::Matrix<T, Rows, Cols> convert2Eigen(const cvl::Matrix<T, Rows, Cols> &a)
    {
        Eigen::Matrix<T, Rows, Cols> b;

        for (unsigned int row = 0; row < Rows; row++) {
            for (unsigned int col = 0; col < Cols; col++) {
                b(row, col) = a(row, col);
            }
        }
        return b;
    }

    template<typename T, int Rows, int Cols>
    cvl::Matrix<T, Rows, Cols> convert2Cvl(const Eigen::Matrix<T, Rows, Cols> &a)
    {
        cvl::Matrix<T, Rows, Cols> b;

        for (unsigned int row = 0; row < Rows; row++) {
            for (unsigned int col = 0; col < Cols; col++) {
                b(row, col) = a(row, col);
            }
        }
        return b;
    }
	*/
    template<class T>
    cv::Point_<T> convert2Point(const cvl::Matrix<T, 2, 1> &m)
    {
        return cv::Point_<T>(m(0), m(1));
    }

    template<class T>
    cv::Point_<T> convert2Point(const cvl::Matrix<T, 3, 1> &m)
    {
        return cv::Point_<T>(m(0), m(1), m(2));
    }

    template<class T>
    cvl::Matrix<T,2,1> convert2Cvl(const cv::Point_<T> &m)
    {
        return cvl::Matrix<T,2,1>(m.x, m.y);
    }

} // cvl::
