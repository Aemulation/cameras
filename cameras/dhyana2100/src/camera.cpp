#include <TUCamApi.h>
#include <algorithm>
#include <cstdint>
#include <exception>
#define _CRT_SECURE_NO_WARNINGS

#include <TUDefine.h>
#include <array>
#include <cassert>
#include <fmt/core.h>
#include <format>
#include <mutex>
#include <stdexcept>
#include <string>

#include <camera.h>

std::vector<std::string> Camera::get_element_names() {
    std::lock_guard camera_lock(camera_mutex);
    std::vector<std::string> elems;

    TUCAM_ELEMENT node = {};
    char name[20] = "Root";
    node.pName = name;

    while (TUCAMRET_SUCCESS ==
           TUCAM_GenICam_ElementAttrNext(camera.hIdxTUCam, &node, node.pName)) {
        elems.push_back(node.pName);
    }

    return elems;
}

std::string Camera::get_enum_or_string_element(const char *element_name) {
    auto elem = get_element(element_name);

    std::lock_guard camera_lock(camera_mutex);

    if (elem.Type == TU_ElemString) {
        return elem.pTransfer;
    } else if (elem.Type == TU_ElemEnumeration) {
        return elem.pEntries[elem.nVal];
    }

    throw std::runtime_error(
        std::format("Incorrect elem type {}", element_name));
}

Camera::Camera(unsigned num_copy_threads_per_buffer)
    : init{}, camera{}, config{}, tucam_frame{},
      num_copy_threads_per_buffer(num_copy_threads_per_buffer) {
    config.num_buffer_frames = 128;
}

Camera::~Camera() {
    if (capturing) {
        try {
            stop();
        } catch (const std::exception &error) {
            fmt::println(stderr, "Failed to stop camera: {}", error.what());
        }
    }

    std::lock_guard camera_lock(camera_mutex);

    if (camera.hIdxTUCam) {
        if (TUCAM_Dev_Close(camera.hIdxTUCam) != TUCAMRET_SUCCESS) {
            fmt::println(stderr, "Failed to close camera device");
            return;
        };
        camera.hIdxTUCam = nullptr;

        if (TUCAM_Api_Uninit() != TUCAMRET_SUCCESS) {
            fmt::println(stderr, "Failed to uninit tucam API");
            return;
        };
    }
}

TUCAM_ELEMENT Camera::get_element(const char *element_name) {
    std::lock_guard camera_lock(camera_mutex);

    std::array<char, 128> name_buffer;
    strcpy(name_buffer.data(), element_name);
    TUCAM_ELEMENT element;

    if (TUCAM_GenICam_ElementAttr(camera.hIdxTUCam, &element,
                                  name_buffer.data()) != TUCAMRET_SUCCESS)
        throw std::runtime_error(std::format(
            "TUCAM_GenICam_ElementAttr failed for {}", element_name));

    if (TU_ElemString != element.Type)
        TUCAM_GenICam_GetElementValue(camera.hIdxTUCam, &element);

    return element;
}

int Camera::get_int_element(const char *name, int *min, int *max) {
    const auto element = get_element(name);

    if (min) {
        *min = static_cast<int>(element.nMin);
    }
    if (max) {
        *max = static_cast<int>(element.nMax);
    }

    return static_cast<int>(element.nVal);
}

int Camera::get_unsigned_element(const char *name, unsigned *min,
                                 unsigned *max) {
    const auto element = get_element(name);

    if (min) {
        *min = static_cast<unsigned>(element.nMin);
    }
    if (max) {
        *max = static_cast<unsigned>(element.nMax);
    }

    return static_cast<unsigned>(element.nVal);
}

float Camera::get_float_element(const char *name, float *min, float *max) {
    const auto element = get_element(name);

    if (max) {
        *max = static_cast<float>(element.dbMax);
    }
    if (min) {
        *min = static_cast<float>(element.dbMin);
    }

    return static_cast<float>(element.dbVal);
}

void Camera::set_enum_element(const char *name, const char *value) {
    auto element = get_element(name);

    std::lock_guard camera_lock(camera_mutex);
    for (unsigned element_id{}; element_id <= element.nMax; ++element_id) {
        if (strcmp(element.pEntries[element_id], value) == 0) {
            element.nVal = element_id;

            if (TUCAM_GenICam_SetElementValue(camera.hIdxTUCam, &element) !=
                TUCAMRET_SUCCESS)
                throw std::runtime_error(
                    std::format("Failed to set attrib {}", name));
            return;
        }
    }

    throw std::runtime_error(
        std::format("Invalid value {} for attribute {}", value, name));
}

void Camera::set_int_element(const char *name, const int value) {
    auto elem = get_element(name);
    elem.nVal =
        std::min(elem.nMax, std::max(elem.nMin, static_cast<INT64>(value)));

    std::lock_guard camera_lock(camera_mutex);
    if (TUCAM_GenICam_SetElementValue(camera.hIdxTUCam, &elem) !=
        TUCAMRET_SUCCESS) {
        throw std::runtime_error(
            std::format("Failed to set {} to int value {}", name, value));
    }
}

void Camera::set_float_element(const char *name, const float val) {
    auto elem = get_element(name);
    elem.dbVal =
        std::min(elem.dbMax, std::max(elem.dbMin, static_cast<DOUBLE>(val)));

    std::lock_guard camera_lock(camera_mutex);
    if (TUCAM_GenICam_SetElementValue(camera.hIdxTUCam, &elem) !=
        TUCAMRET_SUCCESS) {
        throw std::runtime_error(
            std::format("Failed to set {} to float value {}", name, val));
    }
}

void Camera::set_bool_element(const char *name, const bool value) {
    set_int_element(name, value ? 1 : 0);
}

void Camera::get_config(CameraConfig &config, CameraConfig &minvals,
                        CameraConfig &maxvals) {
    config = this->config;

    config.full_mode = get_enum_or_string_element("DeviceMode") == "FULL Mode";
    config.width =
        get_unsigned_element("Width", &minvals.width, &maxvals.width);
    config.height =
        get_unsigned_element("Height", &minvals.height, &maxvals.height);
    config.offsetx =
        get_unsigned_element("OffsetX", &minvals.offsetx, &maxvals.offsetx);
    config.offsety = get_int_element("OffsetY");

    config.analog_gain = get_int_element("AnalogGain");

    // Get SensorMode=FastBinning
    std::string sensorMode = get_enum_or_string_element("SensorMode");
    config.fast_binning = sensorMode == "FastBinning";

    config.exposure_ms = get_float_element("ExposureTime", &minvals.exposure_ms,
                                           &maxvals.exposure_ms);
    config.framerate = get_float_element(
        "AcquisitionFrameRate", &minvals.framerate, &maxvals.framerate);

    const std::string trigger_mode = get_enum_or_string_element("TriggerMode");
    const std::string trigger_source =
        get_enum_or_string_element("TriggerSource");
    if (trigger_mode == "On") {
        if (trigger_source == "ExternalTrigger")
            config.input_trigger = 1;
        else
            config.input_trigger = 2; // SoftTrigger
    } else {
        config.input_trigger = 0;
    }
    config.trigger_frames = get_int_element("TriggerFrameNum");

    const std::string pixel_format = get_enum_or_string_element("PixelFormat");
    if (pixel_format == "Mono 8")
        config.bits_per_pixel = BitsPerPixel::Bits8;
    else if (pixel_format == "Mono 10")
        config.bits_per_pixel = BitsPerPixel::Bits10;
    else if (pixel_format == "Mono 12")
        config.bits_per_pixel = BitsPerPixel::Bits12;
    else
        throw std::runtime_error(
            std::format("Unexpected pixelformat {}", pixel_format.c_str()));

    config.test_pattern =
        get_enum_or_string_element("TestPattern") == "SensorTestPattern";

    this->config = config;
}

void Camera::get_info(CameraInfo &info) {
    info.bytesPerPixel = bytes_per_pixel;
    info.maxFPS = static_cast<float>(get_element("AcquisitionFrameRate").dbMax);
    info.maxWidth = static_cast<unsigned>(get_element("Width").nMax);
    info.maxHeight = static_cast<unsigned>(get_element("Height").nMax);
}

void Camera::set_config(const CameraConfig &config) {
    set_enum_element("DeviceMode",
                     config.full_mode ? "FULL Mode" : "Base Mode");
    set_enum_element(
        "PixelFormat",
        std::format("Mono {}", static_cast<int>(config.bits_per_pixel))
            .c_str());
    set_enum_element("SensorMode",
                     config.fast_binning ? "FastBinning" : "Normal");
    // SetEnumElem("BinningAver", "Off");
    set_bool_element("BinningAver", false);
    set_int_element("Width", config.width);
    set_int_element("Height", config.height);
    set_int_element("OffsetX", config.offsetx);
    // offsety is auto-set

    // setup triggering
    TUCAM_TRIGGER_ATTR triggerAttr;
    if (config.input_trigger == DC_SW_TRIGGER) {
        set_enum_element("TriggerMode", "On");
        set_enum_element("TriggerSource", "SoftTrigger");
        set_enum_element("TriggerActivation", "Falling Edge");
        triggerAttr.nTgrMode = TUCCM_TRIGGER_SOFTWARE;
    } else if (config.input_trigger == DC_HW_TRIGGER) {
        set_enum_element("TriggerMode", "On");
        set_enum_element("TriggerSource", "ExternalTrigger");
        set_enum_element("TriggerActivation", "Falling Edge");
        triggerAttr.nTgrMode = TUCCM_TRIGGER_STANDARD;
    } else { // DC_NO_TRIGGER
        set_enum_element("TriggerMode", "Off");
        triggerAttr.nTgrMode = TUCCM_SEQUENCE;
    }

    // TODO: Should this be one?
    // It was one but it seems weird that `nFrames` != `config.trigger_frames`
    // in set_int_element("TriggerFrameNum", std::max(1,
    // config.trigger_frames));
    //
    triggerAttr.nFrames = 1;
    // triggerAttr.nFrames = config.trigger_frames;
    set_int_element("TriggerFrameNum", std::max(1, config.trigger_frames));

    set_enum_element("TestPattern",
                     config.test_pattern ? "SensorTestPattern" : "Off");

    set_float_element("ExposureTime", config.exposure_ms);
    set_float_element("AcquisitionFrameRate", config.framerate);

    set_int_element("AnalogGain", config.analog_gain);

    bytes_per_pixel = to_bytes_per_pixel(config.bits_per_pixel);

    triggerAttr.nDelayTm = 0;
    triggerAttr.nEdgeMode = 1; // rising
    triggerAttr.nExpMode = 0;  // exposure time

    capture_mode = triggerAttr.nTgrMode;

    std::lock_guard camera_lock(camera_mutex);
    if (TUCAM_Cap_SetTrigger(camera.hIdxTUCam, triggerAttr) !=
        TUCAMRET_SUCCESS) {
        throw std::runtime_error("TUCAM_Cap_SetTrigger failed");
    }

    this->config = config;
}

std::size_t Camera::get_frame_sizes(const Camera &camera) {
    return camera.config.height * camera.config.width *
           to_bytes_per_pixel(camera.config.bits_per_pixel);
}

void Camera::open_camera(const unsigned camera_index) {
    std::lock_guard camera_lock(camera_mutex);
    if (TUCAM_Api_Init(&init) != TUCAMRET_SUCCESS) {
        throw std::runtime_error("TUCAM_Api_Init failed");
    }

    if (camera_index >= init.uiCamCount) {
        throw std::runtime_error("Camera index out of range");
    }

    camera.hIdxTUCam = nullptr;
    camera.uiIdxOpen = camera_index;
    if (TUCAM_Dev_Open(&camera) != TUCAMRET_SUCCESS) {
        throw std::runtime_error("TUCAM_Dev_Open failed");
    }

    if (TUCAM_GenICam_BuffDataCallBack(
            camera.hIdxTUCam,
            reinterpret_cast<BUFFER_CALLBACK>(&Camera::frame_callback_caller),
            this) != TUCAMRET_SUCCESS) {
        throw std::runtime_error("TUCAM_GenICam_BuffDataCallBack failed");
    }
}

void Camera::start() {
    std::lock_guard camera_lock(camera_mutex);

    if (capturing) {
        return;
    }

    tucam_frame = {};
    tucam_frame.pBuffer = nullptr;
    // Looks like this should always be 1.
    tucam_frame.uiRsdSize = 1;
    tucam_frame.ucFormatGet = TUFRM_FMT_USUAL;

    if (TUCAM_Buf_Alloc(camera.hIdxTUCam, &tucam_frame) != TUCAMRET_SUCCESS) {
        throw std::runtime_error("TUCAM_Buf_Alloc failed");
    }

    fmt::println("Assumed size:   {}", 2560 * 2016 * 2);
    fmt::println("Actual size :   {}", tucam_frame.uiImgSize);

    fmt::println("Frame width:    {}", tucam_frame.usWidth);
    fmt::println("Frame height:   {}", tucam_frame.usHeight);
    fmt::println("Frame offset:   {}", tucam_frame.usOffset);
    fmt::println("Frame format:   {}", tucam_frame.ucFormat);
    fmt::println("Frame channels: {}", tucam_frame.ucChannels);
    fmt::println("Frame reserved: {}", tucam_frame.uiRsdSize);

    num_lost_frames = 0;
    num_captured_frames = 0;

    if (TUCAM_Cap_Start(camera.hIdxTUCam, capture_mode,
                        config.num_buffer_frames) != TUCAMRET_SUCCESS) {
        throw std::runtime_error("TUCAM_Cap_Start failed");
    }

    capturing = true;
}

void Camera::frame_callback_caller(Camera *camera) { camera->frame_callback(); }

void Camera::frame_callback() {
    TUCAM_CB_BUFHEADER frame_buffer;

    if (!capturing) {
        return;
    }

    std::lock_guard<std::mutex> camera_lock(camera_mutex);

    std::unique_lock empty_frame_buffers_lock(empty_frame_buffers_mutex);
    if (TUCAM_GenICam_GetBuffData(camera.hIdxTUCam, &frame_buffer) !=
        TUCAMRET_SUCCESS) {
        throw std::runtime_error("TUCAM_GenICam_GetBuffData failed");
    }

    if (empty_frame_buffers.empty()) {
        ++num_lost_frames;
        return;
    }

    FrameBuffer &empty_frame_buffer = empty_frame_buffers.front();

    if (frame_buffer.uiImgSize != empty_frame_buffer.frame_size) {
        ++num_lost_frames;
        return;
    }
    empty_frame_buffer.copier->queue_copy(
        empty_frame_buffer.data +
            empty_frame_buffer.frame_size * empty_frame_buffer.current_frame_id,
        frame_buffer.pImgData, frame_buffer.uiImgSize);

    ++num_captured_frames;
    ++empty_frame_buffer.current_frame_id;

    if (empty_frame_buffer.current_frame_id == empty_frame_buffer.max_frames) {
        /* This buffer is full, so we take it off the empty buffer queue, and
         * move it to the full buffer queue. */
        empty_frame_buffer.current_frame_id = 0;
        std::unique_lock full_frame_buffers_lock(full_frame_buffers_mutex);

        full_frame_buffers.emplace_back(std::move(empty_frame_buffer));
        empty_frame_buffers.pop_front();

        empty_frame_buffers_lock.unlock();
        full_frame_buffers_lock.unlock();

        frame_condition_variable.notify_all();
    }
}

void Camera::stop() {
    if (!capturing) {
        return;
    }

    capturing = false;

    std::lock_guard camera_lock(camera_mutex);

    if (TUCAM_Cap_Stop(camera.hIdxTUCam) != TUCAMRET_SUCCESS) {
        throw std::runtime_error("TUCAM_Cap_Stop failed");
    }

    if (tucam_frame.pBuffer) {
        TUCAM_Buf_Release(camera.hIdxTUCam);
        tucam_frame.pBuffer = nullptr;
    }
}

std::uint64_t Camera::get_lost_frames() { return num_lost_frames; }

std::uint64_t Camera::get_captured_frames() { return num_captured_frames; }

uint8_t *Camera::get_next_buffer() {
    std::unique_lock full_frame_buffers_lock(full_frame_buffers_mutex);

    if (full_frame_buffers.empty()) {
        frame_condition_variable.wait(full_frame_buffers_lock, [this] {
            return !full_frame_buffers.empty();
        });
    }

    FrameBuffer buffer = std::move(full_frame_buffers.front());
    full_frame_buffers.pop_front();
    full_frame_buffers_lock.unlock();

    buffer.copier->wait_all();

    std::unique_lock empty_frame_buffers_lock(empty_frame_buffers_mutex);
    empty_frame_buffers.emplace_back(std::move(buffer));

    return buffer.data;
}

void Camera::get_temperature(float &device_temperature,
                             float &sensor_temperature) {
    device_temperature = get_float_element("DeviceTemperature");
    sensor_temperature = get_float_element("SensorBoardTemperature");
}

void Camera::set_temperature_control(const bool enable_fan,
                                     const bool enable_TEC) {
    set_enum_element("FanOperationMode", enable_fan ? "Temperature" : "Off");
    set_enum_element("TECOperationMode", enable_TEC ? "On" : "Off");
    set_int_element("TECDutyFactor", 100);
}

std::uint64_t Camera::get_frame_size() {
    return static_cast<std::uint64_t>(config.width) * config.height *
           bytes_per_pixel;
}

void Camera::add_buffer(unsigned char *const buffer,
                        const std::size_t frame_size,
                        const std::size_t number_of_frames) {
    std::unique_lock empty_frame_buffers_lock(empty_frame_buffers_mutex);

    empty_frame_buffers.push_back(
        FrameBuffer(buffer, frame_size, number_of_frames, 4));
}

// // ----------------------- C API -----------------------
//
// // CDLL_EXPORT Camera* DC_Open(int cameraIndex)
// // {
// // 	Camera* inst = new Camera();
// // 	inst->OpenCamera(cameraIndex);
// // 	return inst;
// // }
// //
// // CDLL_EXPORT void DC_Close(Camera* inst)
// // {
// // 	delete inst;
// // }
// //
// //
// // CDLL_EXPORT bool DC_GetConfig(Camera* inst, DC_Config& config,
// // DC_Config& minvals, DC_Config& maxvals)
// // {
// // 	return inst->GetConfig(config, minvals, maxvals);
// // }
// //
// // CDLL_EXPORT bool DC_GetInfo(Camera* inst, DC_Info& info)
// // {
// // 	return inst->GetInfo(info);
// // }
// //
// // CDLL_EXPORT bool DC_Configure(Camera* inst, const DC_Config&
// config)
// // {
// // 	return inst->Configure(config);
// // }
// //
// // CDLL_EXPORT const char* DC_GetError(Camera* inst)
// // {
// // 	if (inst->hasError)
// // 		return inst->error.c_str();
// // 	return 0;
// // }
// //
// // CDLL_EXPORT void DC_StartAcquisition(Camera* inst)
// // {
// // 	inst->Start();
// // }
// //
// // CDLL_EXPORT void DC_StopAcquisition(Camera* inst)
// // {
// // 	inst->Stop();
// // }
// //
// //
// // CDLL_EXPORT int DC_GetFrameSize(Camera* inst)
// // {
// // 	return inst->GetFrameSize();
// // }
// //
// // CDLL_EXPORT int DC_GetNextFrame(Camera* inst, uint8_t*& data)
// // {
// // 	return inst->GetNextFrame(data);
// // }
// //
// //
// // CDLL_EXPORT int DC_CopyNextFrame(Camera* inst, int newest,
// uint8_t*
// // data)
// // {
// // 	return inst->CopyNextFrame(data, newest);
// // }
// //
// // CDLL_EXPORT int DC_PollFrames(Camera* inst)
// // {
// // 	return inst->PollFrames();
// // }
// //
// // CDLL_EXPORT int DC_GetLostFrames(Camera* inst)
// // {
// // 	return inst->GetLostFrames();
// // }
// //
// // CDLL_EXPORT void DC_ClearQueue(Camera* inst)
// // {
// // 	inst->ClearQueue();
// // }
// //
// // CDLL_EXPORT void DC_GetTemperature(Camera* inst, float&
// deviceTemp,
// // float& sensorTemp)
// // {
// // 	inst->GetTemperatures(deviceTemp, sensorTemp);
// // }
// //
// //
// // CDLL_EXPORT void DC_SoftwareTrigger(Camera* inst)
// // {
// // 	/// dhyana software trigger is basically broken.
// // 	inst->SoftwareTrigger();
// // }
// //
// // CDLL_EXPORT void DC_SetTempControl(Camera* inst, bool enableFan,
// bool
// // enableTEC)
// // {
// // 	inst->SetTempControl(enableFan, enableTEC);
// // }
//
// const char *TU_ElementToString(TUELEM_TYPE element) {
//     switch (element) {
//     case TU_ElemValue:
//         return "TU_ElemValue";
//     case TU_ElemBase:
//         return "TU_ElemBase";
//     case TU_ElemInteger:
//         return "TU_ElemInteger";
//     case TU_ElemBoolean:
//         return "TU_ElemBoolean";
//     case TU_ElemCommand:
//         return "TU_ElemCommand";
//     case TU_ElemFloat:
//         return "TU_ElemFloat";
//     case TU_ElemString:
//         return "TU_ElemString";
//     case TU_ElemRegister:
//         return "TU_ElemRegister";
//     case TU_ElemCategory:
//         return "TU_ElemCategory";
//     case TU_ElemEnumeration:
//         return "TU_ElemEnumeration";
//     case TU_ElemEnumEntry:
//         return "TU_ElemEnumEntry";
//     case TU_ElemPort:
//         return "TU_ElemPort";
//     default:
//         return "Unknown";
//     }
// }

CDLL_EXPORT Camera *camera_open(int camera_index,
                                unsigned num_copy_threads_per_buffer) {
    Camera *camera = new Camera(num_copy_threads_per_buffer);
    camera->open_camera(camera_index);
    return camera;
}

CDLL_EXPORT void camera_close(Camera *camera) { delete camera; }

CDLL_EXPORT void get_config(Camera *camera, CameraConfig &config,
                            CameraConfig &minvals, CameraConfig &maxvals) {
    camera->get_config(config, minvals, maxvals);
}

CDLL_EXPORT void get_info(Camera *camera, CameraInfo &info) {
    camera->get_info(info);
}

CDLL_EXPORT void set_config(Camera *camera, const CameraConfig &config) {
    camera->set_config(config);
}

CDLL_EXPORT void start(Camera *camera) { camera->start(); }

CDLL_EXPORT void stop(Camera *camera) { camera->stop(); }

CDLL_EXPORT std::size_t get_frame_size(Camera *camera) {
    return camera->get_frame_size();
}

CDLL_EXPORT void add_buffer(Camera *camera, unsigned char *const buffer,
                            const std::size_t frame_size,
                            const std::size_t number_of_frames) {
    camera->add_buffer(buffer, frame_size, number_of_frames);
}

CDLL_EXPORT void get_next_buffer(Camera *camera, std::uint8_t **data) {
    *data = camera->get_next_buffer();
}

CDLL_EXPORT std::uint64_t get_lost_frames(Camera *camera) {
    return camera->get_lost_frames();
}

CDLL_EXPORT void get_temperature(Camera *camera, float &deviceTemp,
                                 float &sensorTemp) {
    camera->get_temperature(deviceTemp, sensorTemp);
}

CDLL_EXPORT void set_temperature_control(Camera *camera, bool enableFan,
                                         bool enableTEC) {
    camera->set_temperature_control(enableFan, enableTEC);
}
