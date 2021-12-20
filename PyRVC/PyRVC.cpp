// Copyright (c) RVBUST, Inc - All rights reserved.

#include <RVC/RVC.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <chrono>
#include <string>

#define DEVICE_MAX_NUM 16

using namespace RVC;

PYBIND11_MODULE(PyRVC, m) {
    namespace py = pybind11;

    m.def("GetVersion", &GetVersion);

    m.def("GetLastError", &GetLastError);

    py::class_<Size>(m, "Size")
        .def(py::init<>())
        .def(py::init<int, int>())
        .def_readwrite("width", &Size::width)
        .def_readwrite("height", &Size::height)
        .def_readwrite("cols", &Size::cols)
        .def_readwrite("rows", &Size::rows);

    py::enum_<RVC::CameraID>(m, "CameraID")
        .value("CameraID_NONE", RVC::CameraID::CameraID_NONE)
        .value("CameraID_0", RVC::CameraID::CameraID_0)
        .value("CameraID_1", RVC::CameraID::CameraID_1)
        .value("CameraID_2", RVC::CameraID::CameraID_2)
        .value("CameraID_Left", RVC::CameraID::CameraID_Left)
        .value("CameraID_Right", RVC::CameraID::CameraID_Right)
        .value("CameraID_Both", RVC::CameraID::CameraID_Both)
        .export_values();

    py::enum_<RVC::PortType>(m, "PortType")
        .value("PortType_NONE", RVC::PortType::PortType_NONE)
        .value("PortType_USB", RVC::PortType::PortType_USB)
        .value("PortType_GIGE", RVC::PortType::PortType_GIGE)
        .export_values();

    py::enum_<RVC::CameraTempSelector>(m, "CameraTempSelector")
        .value("CameraTempSelector_Camera", RVC::CameraTempSelector::CameraTempSelector_Camera)
        .value("CameraTempSelector_CoreBoard", RVC::CameraTempSelector::CameraTempSelector_CoreBoard)
        .value("CameraTempSelector_FpgaCore", RVC::CameraTempSelector::CameraTempSelector_FpgaCore)
        .value("CameraTempSelector_Framegrabberboard", RVC::CameraTempSelector::CameraTempSelector_Framegrabberboard)
        .value("CameraTempSelector_Sensor", RVC::CameraTempSelector::CameraTempSelector_Sensor)
        .value("CameraTempSelector_SensorBoard", RVC::CameraTempSelector::CameraTempSelector_SensorBoard)
        .export_values();

    py::enum_<RVC::ProjectorColor>(m, "ProjectorColor")
        .value("ProjectorColor_None", RVC::ProjectorColor::ProjectorColor_None)
        .value("ProjectorColor_Red", RVC::ProjectorColor::ProjectorColor_Red)
        .value("ProjectorColor_Green", RVC::ProjectorColor::ProjectorColor_Green)
        .value("ProjectorColor_Blue", RVC::ProjectorColor::ProjectorColor_Blue)
        .export_values();

    py::enum_<RVC::BalanceSelector>(m, "BalanceSelector")
        .value("BalanceSelector_None", RVC::BalanceSelector::BalanceSelector_None)
        .value("BalanceSelector_Red", RVC::BalanceSelector::BalanceSelector_Red)
        .value("BalanceSelector_Green", RVC::BalanceSelector::BalanceSelector_Green)
        .value("BalanceSelector_Blue", RVC::BalanceSelector::BalanceSelector_Blue)
        .export_values();

    py::enum_<RVC::NetworkType>(m, "NetworkType")
        .value("NetworkType_DHCP", RVC::NetworkType::NetworkType_DHCP)
        .value("NetworkType_STATIC", RVC::NetworkType::NetworkType_STATIC)
        .export_values();

    py::enum_<RVC::NetworkDevice>(m, "NetworkDevice")
        .value("NetworkDevice_LightMachine", RVC::NetworkDevice::NetworkDevice_LightMachine)
        .value("NetworkDevice_LeftCamera", RVC::NetworkDevice::NetworkDevice_LeftCamera)
        .value("NetworkDevice_RightCamera", RVC::NetworkDevice::NetworkDevice_RightCamera)
        .export_values();

    py::class_<RVC::DeviceInfo>(m, "DeviceInfo")
        .def_readonly("name", &RVC::DeviceInfo::name)
        .def_readonly("sn", &RVC::DeviceInfo::sn)
        .def_readonly("factroydate", &RVC::DeviceInfo::factroydate)
        .def_readonly("port", &RVC::DeviceInfo::port)
        .def_readonly("type", &RVC::DeviceInfo::type)
        .def_readonly("cameraid", &RVC::DeviceInfo::cameraid)
        .def_readonly("boardmodel", &RVC::DeviceInfo::boardmodel)
        .def_readonly("support_x2", &RVC::DeviceInfo::support_x2)
        .def_readonly("support_color", &RVC::DeviceInfo::support_color);

    py::class_<Device>(m, "Device")
        .def_static("Destroy", &Device::Destroy)
        .def("IsValid", &Device::IsValid)
        .def("Print", &Device::Print)
        .def("GetDeviceInfo",
             [](Device &self) {
                 DeviceInfo info;
                 bool ret = self.GetDeviceInfo(&info);
                 return std::make_pair(ret, info);
             })
        .def("SetNetworkConfig",
             [](Device &self, const RVC::NetworkDevice d, const RVC::NetworkType type, const char *ip,
                const char *netMask, const char *gateway) {
                 return self.SetNetworkConfig(d, type, ip, netMask, gateway) == 0 ? true : false;
             })
        .def("GetNetworkConfig", [](Device &self, const RVC::NetworkDevice d) {
            RVC::NetworkType type;
            char ip[16];
            char netmask[16];
            char gateway[16];
            memset(ip, 0, sizeof(char) * 16);
            memset(netmask, 0, sizeof(char) * 16);
            memset(gateway, 0, sizeof(char) * 16);
            int status = 0;
            int ret = self.GetNetworkConfig(d, &type, &ip[0], &netmask[0], &gateway[0], &status);
            return std::make_tuple(type, py::bytes(ip), py::bytes(netmask), py::bytes(gateway), status);
        });

    py::enum_<SystemListDeviceType::Enum>(m, "SystemListDeviceTypeEnum")
        .value("USB", SystemListDeviceType::USB)
        .value("GigE", SystemListDeviceType::GigE)
        .value("All", SystemListDeviceType::All)
        .export_values();

    m.def("SystemListDeviceTypeToString",
          [](const SystemListDeviceType::Enum &e) { return SystemListDeviceType::ToString(e); });

    m.def("SystemListDevices", [](SystemListDeviceType::Enum opt = SystemListDeviceType::USB) {
        std::vector<Device> devices(DEVICE_MAX_NUM);
        size_t actualsize = 0;
        int ret = SystemListDevices(&devices[0], DEVICE_MAX_NUM, &actualsize, opt);
        devices.resize(actualsize);
        return std::make_pair(ret, devices);
    });

    py::enum_<ImageType::Enum>(m, "ImageTypeEnum")
        .value("None", ImageType::None)
        .value("Mono8", ImageType::Mono8)
        .value("RGB8", ImageType::RGB8)
        .value("BGR8", ImageType::BGR8)
        .export_values();

    m.def("ImageTypeToString", [](const ImageType::Enum &e) { return ImageType::ToString(e); });
    m.def("ImageTypeGetPixelSize", [](const ImageType::Enum &e) { return ImageType::GetPixelSize(e); });

    m.def("ImageCreate", [](const ImageType::Enum it, const Size &sz) { return Image::Create(it, sz, nullptr, true); });

    m.def(
        "ImageDestroy", [](Image &img, bool no_reuse = true) { Image::Destroy(img, no_reuse); }, "Destroy the image",
        py::arg("img"), py::arg("no_reuse") = true);

    py::class_<Image>(m, "Image", py::buffer_protocol())
        .def("IsValid", &Image::IsValid)
        .def("GetSize", &Image::GetSize)
        .def("GetType", &Image::GetType)
        // construct a proper buffer object a.k.a numpy array for Image
        // If the image is mono type, it will return a buffer object with two dimension: shape == (rows, cols)
        // if the image is 2,3,4 channels, it will return a buffer object with three dimension: shape = (rows, cols, cn)
        .def_buffer([](Image &im) -> py::buffer_info {
            Size size = im.GetSize();
            ImageType::Enum it = im.GetType();
            size_t cn = it;  // directly to channel type
            size_t w = size.width, h = size.height;

            if (it == 0 || w <= 0 || h <= 0)
                return py::buffer_info();  // return empty buffer object.

            if (it == 1) {
                return py::buffer_info(im.GetDataPtr(), {h, w}, {sizeof(char) * w, sizeof(char)});
            } else {
                return py::buffer_info(im.GetDataPtr(), {h, w, cn},
                                       {sizeof(char) * w * cn, sizeof(char) * cn, sizeof(char)});
            }
        });

    py::class_<DepthMap>(m, "DepthMap", py::buffer_protocol())
        .def(py::init<>())
        .def("IsValid", &DepthMap::IsValid)
        .def("GetSize", &DepthMap::GetSize)
        .def_buffer([](DepthMap &self) -> py::buffer_info {
            Size sz = self.GetSize();
            double *p = self.GetDataPtr();
            const auto dsize = sizeof(double);

            return py::buffer_info(p, {sz.height, sz.width}, {dsize * sz.width * 1, dsize});
        });

    py::enum_<PointMapType::Enum>(m, "PointMapTypeEnum")
        .value("None", PointMapType::None)
        .value("PointsOnly", PointMapType::PointsOnly)
        .value("PointsNormals", PointMapType::PointsNormals)
        .export_values();

    m.def("PointMapTypeToString", [](const PointMapType::Enum &e) { return PointMapType::ToString(e); });

    m.def("PointMapCreate",
          [](const PointMapType::Enum it, const Size &sz) { return PointMap::Create(it, sz, nullptr, true); });

    m.def("PointMapDestroy", [](PointMap &img) { PointMap::Destroy(img); });

    py::class_<PointMap>(m, "PointMap", py::buffer_protocol())
        .def(py::init<>())
        .def("IsValid", &PointMap::IsValid)
        .def("GetSize", &PointMap::GetSize)

        // TODO: Implement and use protocol buffer
        .def("GetNormalDataPtr",
             [](PointMap &self) {
                 Size sz = self.GetSize();
                 double *p = self.GetNormalDataPtr();

                 // Here it copied... FIXME
                 auto result = py::array_t<double>(
                     {sz.rows, sz.cols, 3}, {sizeof(double) * sz.cols * 3, sizeof(double) * 3, sizeof(double)}, p);
                 return result;
             })

        .def_buffer([](PointMap &self) -> py::buffer_info {
            Size sz = self.GetSize();
            double *p = self.GetPointDataPtr();
            const auto dsize = sizeof(double);

            return py::buffer_info(p, {sz.height, sz.width, 3}, {dsize * sz.width * 3, dsize * 3, dsize});
        });

    m.def("SystemInit", &SystemInit, "Initialize system.");
    m.def("SystemIsInited", &SystemIsInited, "System Is Inited?");
    m.def("SystemShutdown", &SystemShutdown, "Shut down the system.");

    m.def(
        "SystemFindDevice", [](const char *serialNumber) { return SystemFindDevice(serialNumber); },
        "Find the device with serial number.", py::arg("serialNumber"));

    py::class_<X1::CaptureOptions>(m, "X1_CaptureOptions")
        .def(py::init<>())
        .def_readwrite("calc_normal", &X1::CaptureOptions::calc_normal)
        .def_readwrite("calc_normal_radius", &X1::CaptureOptions::calc_normal_radius)
        .def_readwrite("transform_to_camera", &X1::CaptureOptions::transform_to_camera)
        .def_readwrite("filter_range", &X1::CaptureOptions::filter_range)
        .def_readwrite("phase_filter_range", &X1::CaptureOptions::phase_filter_range)
        .def_readwrite("projector_brightness", &X1::CaptureOptions::projector_brightness)
        .def_readwrite("exposure_time_2d", &X1::CaptureOptions::exposure_time_2d)
        .def_readwrite("exposure_time_3d", &X1::CaptureOptions::exposure_time_3d)
        .def_readwrite("gain_2d", &X1::CaptureOptions::gain_2d)
        .def_readwrite("gain_3d", &X1::CaptureOptions::gain_3d)
        .def_readwrite("gamma_2d", &X1::CaptureOptions::gamma_2d)
        .def_readwrite("gamma_3d", &X1::CaptureOptions::gamma_3d)
        .def_readwrite("hdr_exposure_times", &X1::CaptureOptions::hdr_exposure_times)
        .def_readwrite("use_projector_capturing_2d_image", &X1::CaptureOptions::use_projector_capturing_2d_image)
        .def("GetHDRExposureTimeContent", [](X1::CaptureOptions &opt, int num) {
            int exp = -1;
            if (num > 0 && num < 4) {
                exp = opt.hdr_exposuretime_content[num - 1];
            }
            return exp;
        })
        .def("SetHDRExposureTimeContent", [](X1::CaptureOptions &opt, int num, int exposure_time) {
            if (num > 0 && num < 4) {
                if (exposure_time >= 3 && exposure_time <= 100) {
                    opt.hdr_exposuretime_content[num - 1] = exposure_time;
                    return true;
                }
            }
            return false;
        });


    py::class_<X1>(m, "X1")
        .def(py::init<>())
        .def_static(
            "Create", [](const Device &d, enum CameraID camid) { return X1::Create(d, camid); }, py::arg("device"),
            py::arg("camid") = RVC::CameraID_Left)
        .def_static("Destroy", &X1::Destroy)
        .def("Open", &X1::Open)
        .def("IsOpen", &X1::IsOpen)
        .def("Close", &X1::Close)
        .def("IsValid", &X1::IsValid)
        .def(
            "Capture",
            [](X1 &self, X1::CaptureOptions opts) {
                bool ret = false;
                Py_BEGIN_ALLOW_THREADS;
                ret = self.Capture(opts);
                Py_END_ALLOW_THREADS;
                return ret;
            },
            py::arg("opts") = X1::CaptureOptions())
        .def("SetBandwidth", &X1::SetBandwidth)
        .def("GetImage", &X1::GetImage)
        .def("GetExposureTimeRange",
             [](X1 &self) {
                 int min_v, max_v;
                 bool ret = self.GetExposureTimeRange(&min_v, &max_v);
                 return std::make_tuple(ret, min_v, max_v);
             })
        .def("GetGainRange",
             [](X1 &self) {
                 float min_v, max_v;
                 bool ret = self.GetGainRange(&min_v, &max_v);
                 return std::make_tuple(ret, min_v, max_v);
             })
        .def("GetGammaRange",
             [](X1 &self) {
                 float min_v, max_v;
                 bool ret = self.GetGammaRange(&min_v, &max_v);
                 return std::make_tuple(ret, min_v, max_v);
             })
        .def("GetDepthMap", &X1::GetDepthMap)
        .def("GetPointMap", &X1::GetPointMap)
        .def("SetBalanceRatio", &X1::SetBalanceRatio)
        .def("GetBalanceRatio",
             [](X1 &self, BalanceSelector selector) {
                 float value;
                 bool ret = self.GetBalanceRatio(selector, &value);
                 return std::make_tuple(ret, value);
             })
        .def("GetBalanceRange",
             [](X1 &self, BalanceSelector selector) {
                 float min_v, max_v;
                 bool ret = self.GetBalanceRange(selector, &min_v, &max_v);
                 return std::make_tuple(ret, min_v, max_v);
             })
        .def("AutoWhiteBalance", &X1::AutoWhiteBalance)
        .def("GetExtrinsicMatrix",
             [](X1 &self) {
                 std::vector<float> matrix = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
                 bool ret = self.GetExtrinsicMatrix(&matrix[0]);
                 return std::make_pair(ret, matrix);
             })
        .def("GetIntrinsicParameters",
             [](X1 &self) {
                 std::vector<float> intrinsic_matrix = {0, 0, 0, 0, 0, 0, 0, 0, 0};
                 std::vector<float> distortion = {0, 0, 0, 0, 0};
                 bool ret = self.GetIntrinsicParameters(&intrinsic_matrix[0], &distortion[0]);
                 return std::make_tuple(ret, intrinsic_matrix, distortion);
             })
        .def("SaveCaptureOptionParameters", &X1::SaveCaptureOptionParameters)
        .def("LoadCaptureOptionParameters", [](X1 &self) {
            X1::CaptureOptions opts;
            bool ret = self.LoadCaptureOptionParameters(opts);
            return std::make_pair(ret, opts);
        });


    py::class_<X2::CaptureOptions>(m, "X2_CaptureOptions")
        .def(py::init<>())
        .def_readwrite("transform_to_camera", &X2::CaptureOptions::transform_to_camera)
        .def_readwrite("projector_brightness", &X2::CaptureOptions::projector_brightness)
        .def_readwrite("calc_normal", &X2::CaptureOptions::calc_normal)
        .def_readwrite("calc_normal_radius", &X2::CaptureOptions::calc_normal_radius)
        .def_readwrite("light_contrast_threshold", &X2::CaptureOptions::light_contrast_threshold)
        .def_readwrite("edge_noise_reduction_threshold", &X2::CaptureOptions::edge_noise_reduction_threshold)
        .def_readwrite("exposure_time_2d", &X2::CaptureOptions::exposure_time_2d)
        .def_readwrite("exposure_time_3d", &X2::CaptureOptions::exposure_time_3d)
        .def_readwrite("gain_2d", &X2::CaptureOptions::gain_2d)
        .def_readwrite("gain_3d", &X2::CaptureOptions::gain_3d)
        .def_readwrite("hdr_exposure_times", &X2::CaptureOptions::hdr_exposure_times)
        .def_readwrite("use_projector_capturing_2d_image", &X2::CaptureOptions::use_projector_capturing_2d_image)
        .def("GetHDRExposureTimeContent", [](X2::CaptureOptions &opt, int num) {
            int exp = -1;
            if (num > 0 && num < 4) {
                exp = opt.hdr_exposuretime_content[num - 1];
            }
            return exp;
        })
        .def("SetHDRExposureTimeContent",
             [](X2::CaptureOptions &opt, int num, int exposure_time) {
                 if (num > 0 && num < 4) {
                     if (exposure_time >= 3 && exposure_time <= 100) {
                         opt.hdr_exposuretime_content[num - 1] = exposure_time;
                         return true;
                     }
                 }
                 return false;
             })
        .def_readwrite("gamma_2d", &X2::CaptureOptions::gamma_2d)
        .def_readwrite("gamma_3d", &X2::CaptureOptions::gamma_3d)
        // .def_readwrite("confidence_filter_threshold", &X2::CaptureOptions::confidence_filter_threshold)
        .def_readwrite("projector_color", &X2::CaptureOptions::projector_color);

    py::class_<X2>(m, "X2")
        .def(py::init<>())
        .def_static(
            "Create", [](const Device &d) { return X2::Create(d); }, py::arg("device"))
        .def_static("Destroy", &X2::Destroy)
        .def("Open", &X2::Open)
        .def("IsOpen", &X2::IsOpen)
        .def("Close", &X2::Close)
        .def("IsValid", &X2::IsValid)
        .def(
            "Capture",
            [](X2 &self, X2::CaptureOptions opts) {
                bool ret = false;
                Py_BEGIN_ALLOW_THREADS;
                ret = self.Capture(opts);
                Py_END_ALLOW_THREADS;
                return ret;
            },
            py::arg("opts") = X2::CaptureOptions())
        .def("SetBandwidth", &X2::SetBandwidth)
        .def("GetImage", &X2::GetImage)
        .def("GetPointMap", &X2::GetPointMap)
        .def("GetDepthMap", &X2::GetDepthMap)
        .def("AutoWhiteBalance", &X2::AutoWhiteBalance)
        .def("GetExtrinsicMatrix",
             [](X2 &self, CameraID cid) {
                 std::vector<float> matrix(16);
                 bool ret = self.GetExtrinsicMatrix(cid, matrix.data());
                 return std::make_pair(ret, matrix);
             })
        .def("GetIntrinsicParameters",
             [](X2 &self, CameraID cid) {
                 std::vector<float> intrinsic_matrix(9);
                 std::vector<float> distortion(5);
                 bool ret = self.GetIntrinsicParameters(cid, intrinsic_matrix.data(), distortion.data());
                 return std::make_tuple(ret, intrinsic_matrix, distortion);
             })
        .def("GetGainRange",
             [](X2 &self) {
                 float min_v, max_v;
                 bool ret = self.GetGainRange(&min_v, &max_v);
                 return std::make_tuple(ret, min_v, max_v);
             })
        .def("GetGammaRange",
             [](X2 &self) {
                 float min_v, max_v;
                 bool ret = self.GetGammaRange(&min_v, &max_v);
                 return std::make_tuple(ret, min_v, max_v);
             })
        .def("GetCameraTemperature",
             [](X2 &self, CameraID cid, CameraTempSelector sel) {
                 float temperature;
                 bool ret;
                 ret = self.GetCameraTemperature(cid, sel, temperature);
                 return std::make_pair(ret, temperature);
             })
        .def("SaveCaptureOptionParameters", &X2::SaveCaptureOptionParameters)
        .def("LoadCaptureOptionParameters", [](X2 &self) {
            X2::CaptureOptions opts;
            bool ret = self.LoadCaptureOptionParameters(opts);
            return std::make_pair(ret, opts);
        });
}
