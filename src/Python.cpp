#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <Eigen/Dense>
#include <System.h>
#include <opencv2/opencv.hpp>
#include <pybind11/stl_bind.h>
#include <Thirdparty/Sophus/sophus/se3.hpp>  // Include Sophus for SE(3) support

#include "FrontierDetector.h"

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
        {sizeof(float), sizeof(float) * 4},  // strides
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


PYBIND11_MODULE(orbslam3_python, m) {
    py::class_<ORB_SLAM3::Frontier, std::shared_ptr<ORB_SLAM3::Frontier>>(m, "Frontier")
        .def_readonly("points", &ORB_SLAM3::Frontier::points)
        .def("point_count", [](ORB_SLAM3::Frontier& self) { return self.points.size(); })
        .def_readonly("center", &ORB_SLAM3::Frontier::center)
        .def("__repr__", [](const ORB_SLAM3::Frontier &f) {
            return "<Frontier point_count=" + std::to_string(f.points.size()) +
                   ", center=(" + std::to_string(f.center.x()) +
                   ", " + std::to_string(f.center.y()) +
                   ", " + std::to_string(f.center.z()) + ")>";
        });

    py::class_<octomath::Vector3>(m, "Vector3")
        .def(py::init<float, float, float>(), py::arg("x") = 0, py::arg("y") = 0, py::arg("z") = 0)
        .def("x", [](const octomath::Vector3 &v) { return v.x(); })  // Explicitly call the correct x()
        .def("y", [](const octomath::Vector3 &v) { return v.y(); })  // Explicitly call the correct y()
        .def("z", [](const octomath::Vector3 &v) { return v.z(); })  // Explicitly call the correct z()
        .def("__repr__", [](const octomath::Vector3 &v) {
            return "<Vector3 x=" + std::to_string(v.x()) +
                   ", y=" + std::to_string(v.y()) +
                   ", z=" + std::to_string(v.z()) + ">";
        });


    py::enum_<ORB_SLAM3::System::eSensor>(m, "eSensor")
        .value("MONOCULAR", ORB_SLAM3::System::MONOCULAR)
        .value("STEREO", ORB_SLAM3::System::STEREO)
        .value("RGBD", ORB_SLAM3::System::RGBD)
        .export_values();

    py::class_<ORB_SLAM3::System>(m, "System")
        .def(py::init<const std::string &, const std::string &, ORB_SLAM3::System::eSensor, const bool &>(),
             py::arg("voc_file"), py::arg("settings_file"), py::arg("sensor_type"), py::arg("use_viewer") = true)
        .def("TrackMonocular",
            [](ORB_SLAM3::System &self, const py::array_t<uint8_t> &image, double timestamp, bool inversePose, bool toNED) {
                cv::Mat img = NumpyImageToMat(image);

                Eigen::Matrix4f pose = self.TrackMonocular(img, timestamp).matrix();
                if (inversePose) pose = pose.inverse();
                if (toNED) pose = ORB_SLAM3::Converter::cvPinholeToNED(pose);

                return EigenMatrixToArray(pose);
            },
            py::arg("image"), py::arg("timestamp"), py::arg("inverse_pose"), py::arg("to_ned"))
        .def("TrackStereo",
            [](ORB_SLAM3::System &self, const py::array_t<uint8_t> &image1, const py::array_t<uint8_t> &image2, double timestamp,
                 bool inversePose, bool toNED) {
                cv::Mat img1 = NumpyImageToMat(image1);
                cv::Mat img2 = NumpyImageToMat(image2);

                Eigen::Matrix4f pose = self.TrackStereo(img1, img2, timestamp).matrix();
                if (inversePose) pose = pose.inverse();
                if (toNED) pose = ORB_SLAM3::Converter::cvPinholeToNED(pose);

                return EigenMatrixToArray(pose);
            },
            py::arg("front_left"), py::arg("front_right"), py::arg("timestamp"),
            py::arg("inverse_pose"), py::arg("to_ned"))
        .def("TrackRGBD",
            [](ORB_SLAM3::System &self, const py::array_t<uint8_t> &image, const py::array_t<uint8_t> &depth, double timestamp,
                bool inversePose, bool toNED) {
                cv::Mat img = NumpyImageToMat(image);
                cv::Mat dep = numpy_to_cv_float(depth);

                Eigen::Matrix4f pose = self.TrackRGBD(img, dep, timestamp).matrix();
                if (inversePose) pose = pose.inverse();
                if (toNED) pose = ORB_SLAM3::Converter::cvPinholeToNED(pose);

                return EigenMatrixToArray(pose);
            },
            py::arg("image"), py::arg("depth"), py::arg("timestamp"), py::arg("inverse_pose"),
            py::arg("to_ned"))
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
            [](ORB_SLAM3::System &self, const float octree_resolution,
                const int n_mappoint_obs_min, const int n_mappoint_max_dst) -> std::shared_ptr<ORB_SLAM3::OccGrid> {
                Py_BEGIN_ALLOW_THREADS
                self.mpAtlas->GenerateNewOccupancyGrid(octree_resolution, n_mappoint_obs_min, n_mappoint_max_dst);
                Py_END_ALLOW_THREADS

                auto grids = self.mpAtlas->GetOccupancyGrids();
                return grids[grids.size() - 1];
            }, py::arg("octree_resolution"), py::arg("n_mappoint_obs_min"), py::arg("n_mappoint_max_dst"))
        .def("GetMapID",
            [](ORB_SLAM3::System &self) {
                return self.mpAtlas->GetCurrentMap()->GetId();
            })

        .def("GetStablePointCloud", [](ORB_SLAM3::System &self, const float radius, const int n_mappoint_obs_min) -> py::array_t<float>  {
            // Get Points
            ORB_SLAM3::Map* pActiveMap = self.mpAtlas->GetCurrentMap();
            if(!pActiveMap)
                return {};
            const vector<ORB_SLAM3::MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();
            set<ORB_SLAM3::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
            std::vector<Sophus::Vector3f> selectedPoints;

            // Get Current Position
            ORB_SLAM3::KeyFrame* pKF = nullptr;
            if (self.mpAtlas) {
                auto keyframes = self.mpAtlas->GetAllKeyFrames();
                if (!keyframes.empty()) {
                    pKF = keyframes.back(); // Last added keyframe
                }
            }
            if (!pKF) {
                return {}; // Return empty array if no keyframe
            }
            Sophus::Vector3f position = pKF->GetPoseInverse().translation();

            // Select Good Points Within Radius
            for(auto vpMP : vpRefMPs)
            {
                if(vpMP->isBad() || vpMP->nObs < n_mappoint_obs_min) continue;
                Sophus::Vector3f point = vpMP->GetWorldPos();
                // Compute distance
                float distance = (position - point).norm();
                if (distance <= radius) {
                    Eigen::Vector3f pNed = ORB_SLAM3::Converter::cvPinholeToNED(point);
                    selectedPoints.push_back(pNed);
                }
            }

            // Return points in numpy array
            py::ssize_t num_points = static_cast<py::ssize_t>(selectedPoints.size());
            // cout << "Number of selected points:: " << num_points << endl;

            // Create an empty NumPy array (num_points, 3)
            py::array_t<float> numpy_array(py::array::ShapeContainer({num_points, 3}));

            // Fill NumPy array
            auto buf = numpy_array.mutable_unchecked<2>();
            Sophus::Vector3f pNed;
            for (py::ssize_t i = 0; i < num_points; ++i) {
                buf(i, 0) = selectedPoints[i].x();
                buf(i, 1) = selectedPoints[i].y();
                buf(i, 2) = selectedPoints[i].z();
            };

            return numpy_array;
        }, py::arg("radius"), py::arg("n_mappoint_obs_min"))
        .def("GetClosestPoints",
            [](ORB_SLAM3::System &self, const float radius, const int n_mappoint_obs_min) -> py::array_t<float> {

                // Get the most recent keyframe
                ORB_SLAM3::KeyFrame* pKF = nullptr;
                if (self.mpAtlas) {
                    auto keyframes = self.mpAtlas->GetAllKeyFrames();
                    if (!keyframes.empty()) {
                        pKF = keyframes.back(); // Last added keyframe
                    }
                }

                if (!pKF) {
                    return py::array_t<float>(); // Return empty array if no keyframe
                }

                // Get the current keyframe position
                Sophus::Vector3f positionKF = pKF->GetPoseInverse().translation();

                // Get all tracked map points
                auto points = self.GetTrackedMapPoints();
                std::vector<Sophus::Vector3f> closest_points;

                // Iterate over all map points
                for (auto* pMP : points) {
                    if (!pMP || pMP->isBad() || pMP->Observations() < n_mappoint_obs_min) continue;

                    Sophus::Vector3f positionMP = pMP->GetWorldPos();

                    // Compute distance
                    float distance = (positionKF - positionMP).norm();
                    if (distance <= radius) {
                        Eigen::Vector3f pNED = ORB_SLAM3::Converter::cvPinholeToNED(positionMP);
                        closest_points.push_back(pNED);
                    }
                }

                // Number of points
                py::ssize_t num_points = static_cast<py::ssize_t>(closest_points.size());

                // Create an empty NumPy array (num_points, 3)
                py::array_t<float> numpy_array(py::array::ShapeContainer({num_points, 3}));

                // Fill NumPy array
                auto buf = numpy_array.mutable_unchecked<2>();
                for (py::ssize_t i = 0; i < num_points; ++i) {
                    buf(i, 0) = closest_points[i].x();
                    buf(i, 1) = closest_points[i].y();
                    buf(i, 2) = closest_points[i].z();
                };

                return numpy_array;

            }, py::arg("radius"), py::arg("n_mappoint_obs_min"));



    py::class_<ORB_SLAM3::OccGrid, std::shared_ptr<ORB_SLAM3::OccGrid>>(m, "OccupancyGrid")
        .def("SaveToFile", &ORB_SLAM3::OccGrid::SaveToFile, py::arg("filename"), py::call_guard<py::gil_scoped_release>())
        .def("Entropy", &ORB_SLAM3::OccGrid::Entropy, py::call_guard<py::gil_scoped_release>())
        .def("InformationGainOver", &ORB_SLAM3::OccGrid::InformationGainOver, py::arg("otherGC"), py::call_guard<py::gil_scoped_release>())
        .def("PointCloud", [](ORB_SLAM3::OccGrid &self) {
            vector<point3d> pcd = *self.GetPointCloud();
            // Specify the shape of the NumPy array
            // Number of points in the point cloud
            pybind11::ssize_t num_points = static_cast<pybind11::ssize_t>(pcd.size());

            // Create a NumPy array with the desired shape (num_points, 3)
            pybind11::array_t<float> numpy_array(pybind11::array::ShapeContainer({num_points, 3}));

            // Access the raw buffer of the NumPy array for modification
            auto buf = numpy_array.mutable_unchecked<2>();
            for (pybind11::ssize_t i = 0; i < num_points; ++i) {
                buf(i, 0) = pcd[i].x();
                buf(i, 1) = pcd[i].y();
                buf(i, 2) = pcd[i].z();
            }

            return numpy_array;
        })
        .def("PointCloudWithProba", [](ORB_SLAM3::OccGrid &self) {
            const octomap::OcTree *octree = self.GetOcTree();  // Assuming a function to get the OctoMap instance
            if (!octree) {
                throw std::runtime_error("OctoMap is not initialized!");
            }

            std::vector<std::pair<octomap::point3d, float>> pcd_with_proba;

            // octree already uses NED coordinate system
            for (octomap::OcTree::leaf_iterator it = octree->begin(), end = octree->end(); it != end; ++it) {
                octomap::point3d point = it.getCoordinate();
                float probability = it->getOccupancy();  // Extract occupancy probability
                pcd_with_proba.emplace_back(point, probability);
            }

            pybind11::ssize_t num_points = static_cast<pybind11::ssize_t>(pcd_with_proba.size());

            // Create a NumPy array with shape (num_points, 4)
            pybind11::array_t<float> numpy_array(pybind11::array::ShapeContainer({num_points, 4}));

            // Access raw buffer
            auto buf = numpy_array.mutable_unchecked<2>();

            for (pybind11::ssize_t i = 0; i < num_points; ++i) {
                buf(i, 0) = pcd_with_proba[i].first.x();
                buf(i, 1) = pcd_with_proba[i].first.y();
                buf(i, 2) = pcd_with_proba[i].first.z();
                buf(i, 3) = pcd_with_proba[i].second;  // Probability
            }

            return numpy_array;
        });

    m.def("DetectFrontiers",
          [](const std::shared_ptr<ORB_SLAM3::OccGrid>& pOG) {
              vector<shared_ptr<ORB_SLAM3::Frontier>>* frontiers = ORB_SLAM3::FrontierDetector::DetectFrontiers(pOG);
              return frontiers;
          });

    m.def("ConvertVector3f",
        [](const Eigen::Vector3f& v) {
            return ORB_SLAM3::Converter::cvPinholeToNED(v);
        });
    m.def("ConvertVector4f",
        [](const Eigen::Vector4f& v) {
            return ORB_SLAM3::Converter::cvPinholeToNED(v);
        });
    m.def("ConvertMatrix4f",
        [](const Eigen::Matrix4f& v) {
            return ORB_SLAM3::Converter::cvPinholeToNED(v);
        });
}
