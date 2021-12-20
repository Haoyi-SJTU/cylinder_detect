// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <string>
#include <vector>

// Define EXPORTED for any platform
#if defined _WIN32 || defined __CYGWIN__
#ifdef RVC_WIN_EXPORT
#ifdef __GNUC__
#define RVC_EXPORT __attribute__((dllexport))
#else
#define RVC_EXPORT __declspec(dllexport)  // Note: actually gcc seems to also supports this syntax.
#endif
#else
#ifdef __GNUC__
#define RVC_EXPORT __attribute__((dllimport))
#else
#define RVC_EXPORT __declspec(dllimport)  // Note: actually gcc seems to also supports this syntax.
#endif
#endif
#define RVC_NOT_EXPORT
#else
#if __GNUC__ >= 4
#define RVC_EXPORT __attribute__((visibility("default")))
#define RVC_NOT_EXPORT __attribute__((visibility("hidden")))
#else
#define RVC_EXPORT
#define RVC_NOT_EXPORT
#endif
#endif

#define EXPOSURE_TIME_2D_MIN 3
#define EXPOSURE_TIME_2D_MAX 100
#define EXPOSURE_TIME_3D_MIN 3
#define EXPOSURE_TIME_3D_MAX 100
#define EXPOSURE_TIME_3D_MAX_OLD 20

/**
 * @brief RVC namespace
 *
 */
namespace RVC {
/**
 * @brief Handle id 4 bytes long
 *
 */
typedef uint32_t HandleID;
/**
 * @brief RVC Handle
 *
 */
struct RVC_EXPORT Handle {
    /**
     * @brief Construct a new Handle object
     *
     */
    Handle() : sid(0), gid(0) {}
    /**
     * @brief Construct a new Handle object with sid and gid
     *
     * @param sid_
     * @param gid_
     */
    Handle(HandleID sid_, HandleID gid_) : sid(sid_), gid(gid_) {}
    /**
     * @brief Construct a new Handle object with another Handle
     *
     * @param rhs Another Handle
     */
    Handle(const Handle &rhs) : sid(rhs.sid), gid(rhs.gid) {}
    /**
     * @brief Assigns new Object to the Handle object,
     * replace its current parameters.
     *
     * @param rhs A Handle object of the same type
     * @return Handle& *this
     */
    Handle &operator=(const Handle &rhs) {
        sid = rhs.sid;
        gid = rhs.gid;
        return *this;
    }
    /**
     * @brief Compare with another Handle object
     *
     * @param rhs Another Handle object
     * @return true Equal
     * @return false Not equal
     */
    inline bool operator==(const Handle &rhs) { return sid == rhs.sid && gid == rhs.gid; }
    /**
     * @brief Compare with another Handle object
     *
     * @param rhs Another Handle object
     * @return true Not equal
     * @return false Equal
     */
    inline bool operator!=(const Handle &rhs) { return sid != rhs.sid || gid != rhs.gid; }

    HandleID sid;
    HandleID gid;
};

/**
 * @brief Image/PointMap Size
 *
 */
struct RVC_EXPORT Size {
    /**
     * @brief Construct a new Image/PointMap Size object
     *
     */
    Size() : width(0), height(0) {}
    /**
     * @brief Construct a new Image/PointMap Size object with width/cols and height/rows
     *
     * @param w Image width/PointMap cols
     * @param h Image height/PointMap rows
     */
    Size(int w, int h) : width(w), height(h) {}
    /**
     * @brief width or cols
     *
     */
    union {
        int32_t width, cols;
    };
    /**
     * @brief height or rows
     *
     */
    union {
        int32_t height, rows;
    };
};

/**
 * @brief Compare two Image size
 *
 * @param lhs one Image Size
 * @param rhs another Image Size
 * @return true equal
 * @return false not equal
 */
inline bool operator==(const Size &lhs, const Size &rhs) {
    return lhs.width == rhs.width && lhs.height == rhs.height;
}
/**
 * @brief Compare two Image size
 *
 * @param lhs one Image Size
 * @param rhs another Image Size
 * @return true not equal
 * @return false equal
 */
inline bool operator!=(const Size &lhs, const Size &rhs) {
    return lhs.width != rhs.width || lhs.height != rhs.height;
}
/**
 * @brief Image Type
 *
 *  All support type are *Mono8*, *RGB8*, *BGR8*
 */
struct RVC_EXPORT ImageType {
    enum Enum {
        None = 0,
        Mono8 = 1,
        RGB8 = 2,
        BGR8 = 3,
    };
    /**
     * @brief Print given ImageType
     *
     * @param type ImageType
     * @return const char* Name of ImageType
     */
    static const char *ToString(ImageType::Enum type);
    /**
     * @brief Get pixel size of ImageType
     *
     * @param type ImageType
     * @return size_t Pixel size
     */
    static size_t GetPixelSize(ImageType::Enum type);
};

/**
 * @brief RVC X Image object
 *
 */
struct RVC_EXPORT Image {
    /**
     * @brief Create Image object
     *
     * @param it image type
     * @param sz image size
     * @param data image data
     * @param own_data Image object
     * @return Image A Image object with given parameters
     */
    static Image Create(ImageType::Enum it, const Size sz, unsigned char *data = nullptr, bool own_data = true);
    /**
     * @brief Destroy Image object
     *
     * @param img Image object will be destroyed
     * @param no_reuse True for reuse current space of PointMap
     */
    static void Destroy(Image img, bool no_reuse = true);
    /**
     * @brief Check Image obejct is Valid or not.
     *
     * @return true Valid
     * @return false Not valid
     */
    bool IsValid();
    /**
     * @brief Get the Image Size object
     *
     * @return Size A Image Size object
     */
    Size GetSize();
    /**
     * @brief Get the ImageType object
     *
     * @return ImageType Image type
     */
    ImageType::Enum GetType();
    /**
     * @brief Get the Data Ptr of Image object
     *
     * @return unsigned* Data Ptr
     */
    unsigned char *GetDataPtr();
    /**
     * @brief Get the Data Const Ptr of Image object
     *
     * @return const unsigned* Data Ptr
     */
    const unsigned char *GetDataConstPtr();
    /**
     * @brief RVC Handle
     *
     */
    Handle m_handle;
};

/**
 * @brief RVC X DepthMap
 *
 */
struct RVC_EXPORT DepthMap {
    /**
     * @brief Create DepthMap object
     *
     * @param sz depthmap size
     * @param data depthmap data
     * @param own_data depthmap data
     * @return DepthMap object with given parameters
     */
    static DepthMap Create(const Size sz, double *data = nullptr, bool own_data = true);
    /**
     * @brief Destroy DepthMap object
     *
     * @param depthmap DepthMap object will be destroyed
     * @param no_reuse True for reuse current space of PointMap
     */
    static void Destroy(DepthMap depthmap, bool no_reuse = true);

    /**
     * @brief Check DepthMap obejct is Valid or not.
     *
     * @return true
     * @return false
     */
    bool IsValid();
    /**
     * @brief Get the DepthMap Size object
     *
     * @return Size
     */
    Size GetSize();
    /**
     * @brief Get the Data Ptr of DepthMap
     *
     * @return double* Data Ptr
     */
    double *GetDataPtr();
    /**
     * @brief Get the Data Const Ptr of DepthMap
     *
     * @return const double* Data Ptr
     */
    const double *GetDataConstPtr();

    /**
     * @brief DepthMap Handle
     *
     */
    Handle m_handle;
};

/**
 * @brief RVC X PointMap Type
 *
 */
struct RVC_EXPORT PointMapType {
    /**
     * @brief PointMap Type Support type *PointsOnly* and *PointsNormals*
     * @param PointsOnly which only contains points
     * @param PointsNormals which contains points with normals
     */
    enum Enum {
        None = 0,
        PointsOnly = 1,
        PointsNormals = 2,
    };
    /**
     * @brief print PointMap Type`s name
     *
     * @param e Pointmap Type
     * @return const char* Pointmap Type`s name
     */
    static const char *ToString(enum Enum e);
};

/**
 * @brief RVC X PointMap
 *
 */
struct RVC_EXPORT PointMap {
    /**
     * @brief Create PointMap object
     *
     * @param pmt PointMapType
     * @param size PointMap Size
     * @param data PointMap data
     * @param owndata True for malloc a new space for PointMap
     * @return PointMap A PointMap object
     */
    static PointMap Create(PointMapType::Enum pmt, const Size size, double *data = nullptr, bool owndata = true);
    /**
     * @brief Destroy a PointMap object
     *
     * @param pm PointMap object will be destroyed
     * @param no_reuse True for reuse current space of PointMap
     */
    static void Destroy(PointMap pm, bool no_reuse = true);
    /**
     * @brief Check PointMap is Valid or not
     *
     * @return true Valid
     * @return false Not valid
     */
    bool IsValid();
    /**
     * @brief Get the PointMap Size object
     *
     * @return Size PointMap Size
     */
    Size GetSize();
    /**
     * @brief Get the Point Data Ptr of PointMap object
     *
     * @return double* Point Data Ptr
     */
    double *GetPointDataPtr();
    /**
     * @brief Get the Normal Data Ptr oof PointMap bject
     *
     * @return double* Normal Data Ptr
     */
    double *GetNormalDataPtr();
    /**
     * @brief Get the Point Data Const Ptr of PointMap object
     *
     * @return const double* Point Data Ptr
     */
    const double *GetPointDataConstPtr();
    /**
     * @brief Get the Normal Const Data Ptr oof PointMap bject
     *
     * @return const double* Normal Data Ptr
     */
    const double *GetNormalDataConstPtr();
    /**
     * @brief RVC Handle
     *
     */
    Handle m_handle;
};

/**
 * @brief Get last error of RVC SDK
 *
 * @return int
 */
RVC_EXPORT int GetLastError();

/**
 * @brief Get the Version of RVC SDK
 *
 * @return string of version
 */
RVC_EXPORT const char *GetVersion();

/**
 * @brief CameraID, support *CameraID_Left* and *CameraID_Right*
 *
 * @enum CameraID
 *
 */
enum CameraID {
    CameraID_NONE = 0,
    CameraID_0 = 1 << 0,
    CameraID_1 = 1 << 1,
    CameraID_2 = 1 << 2,
    CameraID_3 = 1 << 3,
    CameraID_Left = CameraID_0,
    CameraID_Right = CameraID_1,
    CameraID_Both = CameraID_Left | CameraID_Right,
};

/**
 * @brief RVC supported Camera Port type
 *
 * All support type is *PortType_USB*, *PortType_GIGE*
 */
enum PortType {
    PortType_NONE = 0,
    PortType_USB = 1,
    PortType_GIGE = 2,
};

/**
 * @brief RVC supported Camera Network type
 *
 *  Only valid when Camera PortType is *PortType_GIGE*
 *
 * All support type is *NetworkType_DHCP*, *NetworkType_STATIC*
 */
enum NetworkType {
    NetworkType_DHCP = 0,
    NetworkType_STATIC = 1,
};

/**
 * @brief Projector color, support *ProjectorColor_Red*, *ProjectorColor_Green* and *ProjectorColor_Blue*
 *
 */
enum ProjectorColor {
    ProjectorColor_None = 0,
    ProjectorColor_Red = 1,
    ProjectorColor_Green = 2,
    ProjectorColor_Blue = 4,
};

/**
 * @brief White balance ratio selector, suport *BalanceSelector_Red*, *BalanceSelector_Green* and *BalanceSelector_Blue*
 *
 */
enum BalanceSelector {
    BalanceSelector_None = 0,
    BalanceSelector_Red,
    BalanceSelector_Green,
    BalanceSelector_Blue,
};

/**
 * @brief NetworkDevie, support *NetworkDevice_LightMachine*, *NetworkDevice_LeftCamera* and *NetworkDevice_RightCamera*
 *
 */
enum NetworkDevice {
    NetworkDevice_LightMachine = 0,
    NetworkDevice_LeftCamera = 1,
    NetworkDevice_RightCamera = 2,
};

/**

 * @brief RVC X Device info
 *
 * @param name Device name
 * @param sn Device serial number
 * @param factroydate Device manufacture date
 * @param port Port number
 * @param type Support *PortType_USB* and *PortType_GIGE*
 * @param cameraid Support *CameraID_Left*, *CameraID_Right* and *CameraID_Both*
 * @param boardmodel main borad model type
 * @param support_x2 support x2 or not
 * @param support_color supported projector color
 */
struct RVC_EXPORT DeviceInfo {
    char name[20];
    char sn[20];
    char factroydate[20];
    char port[20];
    enum PortType type;
    enum CameraID cameraid;
    int boardmodel;
    bool support_x2;
    enum ProjectorColor support_color;
};

/**
 * @brief Device struct
 *
 */
struct RVC_EXPORT Device {
    /**
     * @brief Destroy the device
     *
     * @param d Device to be destroyed
     */
    static void Destroy(Device d);
    /**
     * @brief Check device is valid or not
     *
     * @return true Valid
     * @return false Not valid
     */
    bool IsValid() const;
    /**
     * @brief Print device info
     *
     */
    void Print() const;
    /**
     * @brief Get the DeviceInfo object
     *
     * @param pinfo Current Device information
     * @return true Success
     * @return false Failed
     */
    bool GetDeviceInfo(DeviceInfo *pinfo);
    /**
     * @brief Set the Device network configuration
     *
     * @param d NetworkDevice
     * @param type NetworkType of Device DHCP/Static
     * @param ip New ip for Device, valid if NetworkType is *NetworkType_STATIC*
     * @param netMask New netmask for Device, valid if NetworkType is *NetworkType_STATIC*
     * @param gateway New gateway for Device, valid if NetworkType is *NetworkType_STATIC*
     * @return int status code
     */
    int SetNetworkConfig(enum NetworkDevice d, NetworkType type, const char *ip, const char *netMask,
                         const char *gateway);
    /**
     * @brief Get the device network configuration
     *
     * @param d NetworkDevice
     * @param type NetworkType of Device DHCP/Static
     * @param ip Current ip of Device
     * @param netMask Current netmask of Device
     * @param gateway Current gateway of Device
     * @param status Device`s status
     * @return int Status code
     */
    int GetNetworkConfig(enum NetworkDevice d, NetworkType *type, char *ip, char *netMask, char *gateway, int *status);
    /**
     * @brief RVC Handle
     *
     */
    Handle m_handle;
};

/**
 * @brief System list Device type
 *
 * All support type are *USB*, *GigE*, *ALL*
 */
struct RVC_EXPORT SystemListDeviceType {
    /**
     * @brief RVC camera type, support *USB*, *GigE* and *All*
     *
     */
    enum Enum {
        None = 0,
        USB = 1 << 0,
        GigE = 1 << 1,
        All = USB | GigE,
    };
    /**
     * @brief print SystemListDeviceType
     *
     * @param e List Device Type
     * @return const char* name of SystemListDeviceType
     */
    static const char *ToString(const SystemListDeviceType::Enum e);
};

/**
 * @brief SystemInit
 *
 * @return true Success
 * @return false Failed
 */
RVC_EXPORT bool SystemInit();
/**
 * @brief Check system is inited or not
 *
 * @return true Inited
 * @return false Not Inited
 */
RVC_EXPORT bool SystemIsInited();
/**
 * @brief SystemShutdown
 *
 */
RVC_EXPORT void SystemShutdown();


/**
 * @brief List the fix number devices
 *
 * @param pdevices device listed
 * @param size desire list devices number
 * @param actual_size actually list devices number
 * @param opt List DeviceType All/USB/GIGE optional
 * @return int Status code
 *
 * @snippet X1Test.cpp List USB Device using ptr
 */
RVC_EXPORT int SystemListDevices(Device *pdevices, size_t size, size_t *actual_size,
                                 SystemListDeviceType::Enum opt = SystemListDeviceType::All);

/**
 * @brief Return the device according to the serialNumber, if the device is valid
 *
 * @param serialNumber Device Struct serial number
 * @return Device The serialNumber  corresponding Device Struct
 */
RVC_EXPORT Device SystemFindDevice(const char *serialNumber);


enum CameraTempSelector {
    CameraTempSelector_Camera,
    CameraTempSelector_CoreBoard,
    CameraTempSelector_FpgaCore,
    CameraTempSelector_Framegrabberboard,
    CameraTempSelector_Sensor,
    CameraTempSelector_SensorBoard,
};

/**
 * @brief X1 struct
 *
 */
struct RVC_EXPORT X1 {
    /**
     * @brief Capture options
     *
     * @snippet X1Test.cpp Capture with options
     */
    struct CaptureOptions {
        /**
         * @brief Construct a new Capture Options object
         *
         */
        CaptureOptions() {
            calc_normal = false;
            transform_to_camera = true;
            // false
            filter_range = 2;
            phase_filter_range = 0;
            projector_brightness = 240;
            exposure_time_2d = 3;
            exposure_time_3d = 6;
            gain_2d = 0.f;
            gain_3d = 0.f;
            hdr_exposure_times = 0;
            hdr_exposuretime_content[0] = 1;
            hdr_exposuretime_content[1] = 2;
            hdr_exposuretime_content[2] = 4;
            // 3 6 12
            calc_normal_radius = 1;
            // 5
            gamma_2d = 1.f;
            gamma_3d = 1.f;
            use_projector_capturing_2d_image = false;
            // true
        }
        /**
         * @brief flag Whether calculate 3D points normal vector
         *
         */
        bool calc_normal;
        /**
         * @brief flag Whether transfrom 3D points from calibration board system to camera coordinate system
         *
         */
        bool transform_to_camera;
        /**
         * @brief Set the noise filter value. The larger the value, the greater the filtering degree
         *
         */
        int filter_range;
        /**
         * @brief Set the phase filter value. The larger the value, the greater the filtering degree 0~40
         *
         */
        int phase_filter_range;
        /**
         * @brief Set the exposure_time 2D value in milliseconds
         *
         */
        int exposure_time_2d;
        /**
         * @brief Set the exposure_time 3D value in milliseconds
         *
         */
        int exposure_time_3d;
        /**
         * @brief Set the projector brightness value
         *
         */
        int projector_brightness;
        /**
         * @brief Set the 2D gain value
         *
         */
        float gain_2d;
        /**
         * @brief Set the 3D gain value
         *
         */
        float gain_3d;
        /**
         * @brief Set the hdr exposure times value. 0,2,3
         *
         */
        int hdr_exposure_times;
        /**
         * @brief Set the hdr exposure time content 3 ~ 100
         *
         */
        int hdr_exposuretime_content[3];
        /**
         * @brief Neighborhood radius in pixel of calculating normal, > 0
         */
        unsigned int calc_normal_radius;
        /**
         * @brief Set the 2D gamma value
         *
         */
        float gamma_2d;
        /**
         * @brief Set the 3D gamma value
         *
         */
        float gamma_3d;
        /**
         * @brief Set 2D image whether use projector
         *
         */
        bool use_projector_capturing_2d_image;
    };

    /**
     * @brief Create a RVC X Camera and choose the CameraID which you desire before you use X1
     *
     * @param d One RVC X Camera
     * @param camid Choose *CameraID_Left* or *CameraID_Left*
     * @return X1 RVC X object
     *
     * @snippet X1Test.cpp Create RVC X1
     */
    static X1 Create(const Device &d, enum CameraID camid = CameraID_Left);
    /**
     * @brief Release all X1 resources after you use X1
     *
     * @param x RVC X object to be released
     *
     * @snippet X1Test.cpp Create RVC X1
     */
    static void Destroy(X1 &x);
    /**
     * @brief Check X1 is valid or not before use X1
     *
     * @return true Available
     * @return false Not available
     *
     * @snippet X1Test.cpp Create RVC X1
     */
    bool IsValid();
    /**
     * @brief Open X1 before you use it
     *
     * @return true Success
     * @return false Failed
     *
     * @snippet X1Test.cpp Create RVC X1
     */
    bool Open();
    /**
     * @brief Close X1 after you Open it
     *
     * @snippet X1Test.cpp Create RVC X1
     */
    void Close();
    /**
     * @brief Check X1 is open or not
     *
     * @return true Is open
     * @return false Not open
     *
     * @snippet X1Test.cpp Create RVC X1
     */
    bool IsOpen();
    /**
     * @brief Capture one point map and one image. This function will save capture options into camera.
     *
     * @param opts CaptureOptions
     * @return true Success
     * @return false Failed
     *
     * @snippet X1Test.cpp Capture with options
     */
    bool Capture(const CaptureOptions &opts = CaptureOptions());
    /**
     * @brief Set the camera band width ratio
     *
     * @param percent Band width ratio
     * @return true Success
     * @return false Failed
     */
    bool SetBandwidth(float percent);
    /**
     * @brief Get the Image object
     *
     * @return Image
     */
    Image GetImage();
    /**
     * @brief Get the DepthMap object
     *
     * @return DepthMap A DepthMap
     */
    DepthMap GetDepthMap();
    /**
     * @brief Get the Point Map object
     *
     * @return PointMap
     */
    PointMap GetPointMap();
    /**
     * @brief Get the extrinsic matrix
     *
     * @param matrix  extrinsic matrix
     * @return true Success
     * @return false Failed
     */
    bool GetExtrinsicMatrix(float *matrix);
    /**
     * @brief Get the intrinsic matrix
     *
     * @param instrinsic_matrix camera instrinsic matrix
     * @param distortion //TODO::
     * @return true Success
     * @return false Failed
     */
    bool GetIntrinsicParameters(float *instrinsic_matrix, float *distortion);

    /**
     * @brief Set the white balance ratio value
     *
     * @param selector support BalanceSelector_Red, BalanceSelector_Green and BalanceSelector_Blue
     * @param value  balance ratio value
     * @return true Success
     * @return false Failed
     */
    bool SetBalanceRatio(BalanceSelector selector, float value);
    /**
     * @brief Get the white balance ratio value
     *
     * @param selector support BalanceSelector_Red, BalanceSelector_Green and BalanceSelector_Blue
     * @param value balance ratio value
     * @return true Success
     * @return false Failed
     */
    bool GetBalanceRatio(BalanceSelector selector, float *value);

    /**
     * @brief Get the white balance ratio value range
     *
     * @param selector support BalanceSelector_Red, BalanceSelector_Green and BalanceSelector_Blue
     * @param min_value selector's minimum value
     * @param max_value selector's maximum value
     * @return true Success
     * @return false Failed
     */
    bool GetBalanceRange(BalanceSelector selector, float *min_value, float *max_value);

    /**
     * @brief Only Color Camera can use this function. This function can get suitable white balance paramters
     *
     * @param wb_times how many images used for calculate the white balance paramters
     * @return true
     * @return false
     */
    bool AutoWhiteBalance(int wb_times = 10);


    /**
     * @brief Get the Exposure Time Range
     *
     * @param min_value exposure time minimum value
     * @param max_value exposure time maxinum value
     * @return true Success
     * @return false Failed
     */
    bool GetExposureTimeRange(int *min_value, int *max_value);

    /**
     * @brief Get the Gain Range
     *
     * @param min_value gain minimum value
     * @param max_value gain minimum value
     * @return true Success
     * @return false Failed
     */
    bool GetGainRange(float *min_value, float *max_value);

    /**
     * @brief Get the Gamma Range
     *
     * @param min_value gamma minimum value
     * @param max_value gamma minimum value
     * @return true Success
     * @return false Failed
     */
    bool GetGammaRange(float *min_value, float *max_value);

    /**
     * @brief Save capture option parameters
     *
     * @param opts CaptureOptions
     * @return true Success
     * @return false Failed
     */
    bool SaveCaptureOptionParameters(const X1::CaptureOptions &opts);

    /**
     * @brief Get capture option parameters
     *
     * @param opts CaptureOptions
     * @return true Success
     * @return false Failed
     */
    bool LoadCaptureOptionParameters(X1::CaptureOptions &opts);

    /**
     * @brief RVC Handle
     *
     */
    Handle m_handle;
};


/**
 * @brief X2 struct
 *
 */
struct RVC_EXPORT X2 {
    /**
     * @brief Capture options
     *
     */
    struct CaptureOptions {
        /**
         * @brief Construct a new Capture Options object
         *
         */
        CaptureOptions() {
            transform_to_camera = CameraID_NONE;
            projector_brightness = 240;
            calc_normal = false;
            calc_normal_radius = 5;
            light_contrast_threshold = 3;
            edge_noise_reduction_threshold = 2;
            exposure_time_2d = 6;
            exposure_time_3d = 6;
            gain_2d = 0.f;
            gain_3d = 0.f;
            hdr_exposure_times = 0;
            hdr_exposuretime_content[0] = 3;
            hdr_exposuretime_content[1] = 6;
            hdr_exposuretime_content[2] = 12;
            gamma_2d = 1.f;
            gamma_3d = 1.f;
            projector_color = ProjectorColor_Blue;
            use_projector_capturing_2d_image = true;
        }
        /**
         * @brief CameraID_Left transfrom 3D points from calibration board system to left camera coordinate system
         * CameraID_Right transfrom 3D points from calibration board system to right camera coordinate system
         * other is do not transfrom
         */
        CameraID transform_to_camera;
        /**
         * @brief Set the projector brightness value
         *
         */
        int projector_brightness;
        /**
         * @brief flag Whether calculate 3D points normal vector
         *
         */
        bool calc_normal;

        /**
         * @brief Neighborhood radius in pixel of calculating normal, > 0
         */
        unsigned int calc_normal_radius;

        /**
         * @brief Light contrast trheshold, range in [0, 10]. The contrast of point less than this value will be treat
         * as invalid point and be removed.
         *
         */
        int light_contrast_threshold;
        /**
         * @brief edge control after point matching, range in [0, 10], default = 2. The big the value, the more edge
         * noise to be removed.
         *
         */
        int edge_noise_reduction_threshold;
        /**
         * @brief Set the exposure_time 2D value in milliseconds
         *
         */
        int exposure_time_2d;
        /**
         * @brief Set the exposure_time 3D value in milliseconds
         *
         */
        int exposure_time_3d;
        /**
         * @brief Set the 2D gain value in milliseconds
         *
         */
        float gain_2d;
        /**
         * @brief Set the 3D gain value in milliseconds
         *
         */
        float gain_3d;
        /**
         * @brief Set the hdr exposure times value. 0,2,3
         *
         */
        int hdr_exposure_times;
        /**
         * @brief Set the hdr exposure time content 3 ~ 100
         *
         */
        int hdr_exposuretime_content[3];
        /**
         * @brief Set the 2D gamma value in milliseconds
         *
         */
        float gamma_2d;
        /**
         * @brief Set the 3D gamma value in milliseconds
         *
         */
        float gamma_3d;
        /**
         * @brief  projector color
         */
        ProjectorColor projector_color;
        /**
         * @brief Set 2D image whether use projector
         *
         */
        bool use_projector_capturing_2d_image;
    };

    /**
     * @brief Create a RVC X Camera before you use x2
     *
     * @param d One RVC X Camera
     * @return X2 RVC X object
     */
    static X2 Create(const Device &d);
    /**
     * @brief Release all X2 resources after you use X2
     *
     * @param x RVC X2 object to be released
     */
    static void Destroy(X2 &x);

    /**
     * @brief Check X2 is valid or not before use X2
     *
     * @return true Available
     * @return false Not available
     */
    bool IsValid();
    /**
     * @brief Open X2 before you use it
     *
     * @return true Success
     * @return false Failed
     */
    bool Open();
    /**
     * @brief Close X2 after you Open it
     */
    void Close();
    /**
     * @brief Check X2 is open or not
     *
     * @return true Is open
     * @return false Not open
     */
    bool IsOpen();
    /**
     * @brief Capture one point map and one image.This function will save capture options into camera.
     *
     * @param opts CaptureOptions
     * @return true Success
     * @return false Failed
     */
    bool Capture(const CaptureOptions &opts = CaptureOptions());

    /**
     * @brief Set the camera band width ratio
     *
     * @param percent Band width ratio
     * @return true Success
     * @return false Failed
     */
    bool SetBandwidth(float percent);

    /**
     * @brief Get the Point Map object
     *
     * @return PointMap
     */
    PointMap GetPointMap();
    /**
     * @brief Get the Image object
     *
     * @return Image
     */
    Image GetImage(const CameraID cid);

    /**
     * @brief Get the DepthMap object
     *
     * @return DepthMap A DepthMap
     */
    DepthMap GetDepthMap();

    /**
     * @brief Get extrinsic matrix
     *
     * @param cid [in] option of {CameraID_Left, CameraID_Right}
     * @param matrix [out] the extrinsic matrix
     * @return true for success
     */
    bool GetExtrinsicMatrix(const CameraID cid, float *matrix);

    /**
     * @brief Get intrinsic parameters
     *
     * @param cid [in] option of {CameraID_Left, CameraID_Right}
     * @param instrinsicMatrix [out] intrinsic parameters of selected camera
     * @param distortion [out] distortion parameter of selected camera
     * @return true for success.
     */
    bool GetIntrinsicParameters(const CameraID cid, float *instrinsicMatrix, float *distortion);

    /**
     * @brief  GetCameraTemperature
     * @note   TODO:
     * @param  cid:
     * @param  seletor:
     * @param  &temperature:
     * @retval
     */
    bool GetCameraTemperature(const CameraID cid, CameraTempSelector seletor, float &temperature);
    /**
     * @brief Only Color Camera can use this function. This function can get suitable white balance paramters
     *
     * @param wb_times how many images used for calculate the white balance paramters
     * @return true
     * @return false
     */
    bool AutoWhiteBalance(int wb_times = 10);
    /**
     * @brief Get the Gain Range
     *
     * @param min_value gain minimum value
     * @param max_value gain minimum value
     * @return true Success
     * @return false Failed
     */
    bool GetGainRange(float *min_value, float *max_value);
    /**
     * @brief Get the Gamma Range
     *
     * @param min_value gamma minimum value
     * @param max_value gamma minimum value
     * @return true Success
     * @return false Failed
     */
    bool GetGammaRange(float *min_value, float *max_value);


    /**
     * @brief Save capture option parameters
     *
     * @param opts CaptureOptions
     * @return true Success
     * @return false Failed
     */
    bool SaveCaptureOptionParameters(const X2::CaptureOptions &opts);

    /**
     * @brief Get capture option parameters
     *
     * @param opts CaptureOptions
     * @return true Success
     * @return false Failed
     */
    bool LoadCaptureOptionParameters(X2::CaptureOptions &opts);
    Handle m_handle;
};

}  // namespace RVC
