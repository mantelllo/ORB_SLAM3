#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <Eigen/Dense>
#include <System.h>
#include <opencv2/opencv.hpp>
#include <Thirdparty/Sophus/sophus/se3.hpp>  // Include Sophus for SE(3) support

namespace py = pybind11;


// Convert a numpy array to cv::Mat for grayscale images
cv::Mat numpy_to_cv_grayscale(const py::array_t<uint8_t>& input) {
    return cv::Mat(input.shape(0), input.shape(1), CV_8UC1, (unsigned char*)input.data(), input.strides(0)); // Use strides for proper memory layout
}

// Convert a numpy array to cv::Mat for color images
cv::Mat numpy_to_cv_color(const py::array_t<uint8_t>& input) {
    return cv::Mat(input.shape(0), input.shape(1), CV_8UC3, (unsigned char*)input.data(), input.strides(0)); // Use strides for proper memory layout
}

cv::Mat numpy_to_cv_float(const py::array_t<float_t>& input) {
    return cv::Mat(input.shape(0), input.shape(1), CV_8UC1, (float*)input.data(), input.strides(0)); // Use strides for proper memory layout
}


py::array_t<float> EigenMatrixToArray(Eigen::Matrix4f pose){
    return py::array_t<float>(
        {4, 4},                    // shape
        {4 * sizeof(float), sizeof(float)},  // strides
        pose.data()                // data pointer
    );
}

py::array_t<float> EigenVectorToArray(Eigen::Vector3f pose){
    return py::array_t<float>(
        {3},                            // shape
        {3 * sizeof(float)},        // strides
        pose.data()                // data pointer
    );
}

cv::Mat NumpyImageToMat(const py::array_t<uint8_t>& image) {
    if (image.ndim() == 2)
        return numpy_to_cv_grayscale(image); // Grayscale image

    if (image.ndim() == 3) {
        auto shape = image.shape();
        if (shape[2] == 3)
            return numpy_to_cv_color(image); // Color image

        throw std::runtime_error("Color image must have 3 channels.");
    }

    throw std::runtime_error("Image must be either 2D (grayscale) or 3D (color).");
}


PYBIND11_MODULE(orbslam3, m) {
    py::enum_<ORB_SLAM3::System::eSensor>(m, "eSensor")
        .value("MONOCULAR", ORB_SLAM3::System::MONOCULAR)
        .value("STEREO", ORB_SLAM3::System::STEREO)
        .value("RGBD", ORB_SLAM3::System::RGBD)
        .export_values();

    py::class_<ORB_SLAM3::System>(m, "System")
        .def(py::init<const std::string &, const std::string &, ORB_SLAM3::System::eSensor, const bool &>(),
             py::arg("voc_file"), py::arg("settings_file"), py::arg("sensor_type"), py::arg("use_viewer") = true)
        .def("TrackMonocular",
            [](ORB_SLAM3::System &self, const py::array_t<uint8_t> &image, double timestamp) {
                cv::Mat img = NumpyImageToMat(image);

                Eigen::Matrix4f pose = self.TrackMonocular(img, timestamp).matrix();
                return EigenMatrixToArray(pose);
            },
            py::arg("image"), py::arg("timestamp"))
        .def("TrackStereo",
            [](ORB_SLAM3::System &self, const py::array_t<uint8_t> &image1, const py::array_t<uint8_t> &image2, double timestamp) {
                cv::Mat img1 = NumpyImageToMat(image1);
                cv::Mat img2 = NumpyImageToMat(image2);

                Eigen::Matrix4f pose = self.TrackStereo(img1, img2, timestamp).matrix();
                return EigenMatrixToArray(pose);
            },
            py::arg("front_left"), py::arg("front_right"), py::arg("timestamp"))
        .def("TrackRGBD",
            [](ORB_SLAM3::System &self, const py::array_t<uint8_t> &image, const py::array_t<uint8_t> &depth, double timestamp) {
                cv::Mat img = NumpyImageToMat(image);
                cv::Mat dep = numpy_to_cv_float(depth);

                Eigen::Matrix4f pose = self.TrackRGBD(img, dep, timestamp).matrix();
                return EigenMatrixToArray(pose);
            },
            py::arg("image"), py::arg("depth"), py::arg("timestamp"))
        .def("Shutdown", &ORB_SLAM3::System::Shutdown)
        .def("GetAllMapPoints",
            [](ORB_SLAM3::System &self) {
                std::vector<py::array_t<float>> point_cloud;
                for (const auto& mp : self.mpAtlas->GetAllMapPoints()) {
                    auto pos = mp->GetWorldPos();
                    point_cloud.push_back(EigenVectorToArray(pos));
                }
                return point_cloud;
            })
        .def("GetAllKeyFrames",
            [](ORB_SLAM3::System &self) {
                std::vector<py::array_t<float>> trajectory;
                for (const auto& kf : self.mpAtlas->GetAllKeyFrames()) {
                    auto mat = kf->GetPose().matrix();
                    trajectory.push_back(EigenMatrixToArray(mat));
                }
                return trajectory;
            })
        .def("GetOccupancyGrids",
            [](ORB_SLAM3::System &self) {
                return self.mpAtlas->GetOccupancyGrids();
            })
        .def("GenerateNewOccupancyGrid",
            [](ORB_SLAM3::System &self) -> std::shared_ptr<ORB_SLAM3::OccGrid> {
                self.mpAtlas->GenerateNewOccupancyGrid();
                auto grids = self.mpAtlas->GetOccupancyGrids();
                return grids[grids.size() - 1];
            });

    py::class_<ORB_SLAM3::OccGrid, std::shared_ptr<ORB_SLAM3::OccGrid>>(m, "OccupancyGrid")
        .def("SaveToFile", &ORB_SLAM3::OccGrid::SaveToFile, py::arg("filename"))
        .def("Entropy", &ORB_SLAM3::OccGrid::Entropy)
        .def("InformationGainOver", &ORB_SLAM3::OccGrid::InformationGainOver, py::arg("otherGC"));
}
