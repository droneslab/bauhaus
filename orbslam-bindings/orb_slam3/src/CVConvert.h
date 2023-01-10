#ifndef CVCONVERT_H
#define CVCONVERT_H

#include <vector>
#include<opencv2/core/core.hpp>

namespace orb_slam3
{
    struct Grid;
    struct DVKeyPoint;
    struct DVPoint2f;

    struct BindCVKeyPoints {
        std::vector<cv::KeyPoint> *ptr;
        std::vector<cv::KeyPoint> * operator->() { return ptr; }
        std::vector<cv::KeyPoint> & operator*() const { return *ptr; }
        std::vector<cv::KeyPoint> *as_ptr() { return ptr; }
    };

    struct BindCVKeyPointsRef {
        std::vector<cv::KeyPoint> **ptr;
        std::vector<cv::KeyPoint> * operator->() { return *ptr; }
        std::vector<cv::KeyPoint> & operator*() const { return **ptr; }
        std::vector<cv::KeyPoint> *as_ptr() { return *ptr; }
    };

    struct BindCVMat {
        cv::Mat *ptr;
        cv::Mat * operator->() { return ptr; }
        cv::Mat & operator*() const{ return *ptr; }
        cv::Mat *as_ptr() { return ptr; }
    };

    struct BindCVMatRef {
        cv::Mat **ptr;
        cv::Mat * operator->() { return *ptr; }
        cv::Mat & operator*() const { return **ptr; }
        cv::Mat *as_ptr() { return *ptr; }
    };

    struct BindCVVectorOfi32 {
        std::vector<int> *ptr;
        std::vector<int> * operator->() { return ptr; }
        std::vector<int> & operator*() const{ return *ptr; }
        std::vector<int> *as_ptr() { return ptr; }
    };

    struct BindCVVectorOfPoint2f {
        std::vector<cv::Point2f> *ptr;
        std::vector<cv::Point2f> * operator->() { return ptr; }
        std::vector<cv::Point2f> & operator*() const{ return *ptr; }
        std::vector<cv::Point2f> *as_ptr() { return ptr; }
    };

    struct BindCVVectorOfPoint2fRef {
        std::vector<cv::Point2f> **ptr;
        std::vector<cv::Point2f> * operator->() { return *ptr; }
        std::vector<cv::Point2f> & operator*() const { return **ptr; }
        std::vector<cv::Point2f> *as_ptr() { return *ptr; }
    };

    struct BindCVVectorOfPoint3f {
        std::vector<cv::Point3f> *ptr;
        std::vector<cv::Point3f> * operator->() { return ptr; }
        std::vector<cv::Point3f> & operator*() const{ return *ptr; }
        std::vector<cv::Point3f> *as_ptr() { return ptr; }
    };

    struct BindCVRawPtr {
        void *ptr;
    };
    inline const std::vector<cv::KeyPoint> * raw_ptr_as_keypoints(const BindCVRawPtr ptr) {
        return reinterpret_cast<const std::vector<cv::KeyPoint>*>(ptr.ptr);
    }
    inline const cv::Mat * raw_ptr_as_mat(const BindCVRawPtr ptr) {
        return reinterpret_cast<const cv::Mat *>(ptr.ptr);
    }
    inline std::vector<cv::Point2f> * raw_ptr_as_vecpoint2f(BindCVRawPtr ptr) {
        return reinterpret_cast<std::vector<cv::Point2f> *>(ptr.ptr);
    }
    inline std::vector<cv::Point2f> * raw_ptr_as_veci32(BindCVRawPtr ptr) {
        return reinterpret_cast<std::vector<cv::Point2f> *>(ptr.ptr);
    }
}//namespace ORB_SLAM

#endif
