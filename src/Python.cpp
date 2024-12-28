#include <pybind11/pybind11.h>
#include <System.h>

namespace py = pybind11;


PYBIND11_MODULE(orbslam3, m) {
    py::enum_<ORB_SLAM3::System::eSensor>(m, "eSensor")
        .value("MONOCULAR", ORB_SLAM3::System::MONOCULAR)
        .value("STEREO", ORB_SLAM3::System::STEREO)
        .value("RGBD", ORB_SLAM3::System::RGBD)
        .export_values();

    py::class_<ORB_SLAM3::System>(m, "System")
        .def(py::init<const std::string &, const std::string &, ORB_SLAM3::System::eSensor, const bool &>(),
             py::arg("voc_file"), py::arg("settings_file"), py::arg("sensor_type"), py::arg("use_viewer")=true)
        .def("TrackMonocular", &ORB_SLAM3::System::TrackMonocular, 
             py::arg("image"), py::arg("timestamp"), py::arg("imu_data"), py::arg("debug_string"))
        .def("Shutdown", &ORB_SLAM3::System::Shutdown);
}
