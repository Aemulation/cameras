#pragma once

#include <TUCamApi.h>

#include <assert.h>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <multi_threaded_copier.h>

// TODO: No way
// #pragma pack(push, 4)

#define DC_NO_TRIGGER 0
#define DC_HW_TRIGGER 1
#define DC_SW_TRIGGER 2

enum struct BitsPerPixel : unsigned {
    Bits8 = 8,
    Bits10 = 10,
    Bits12 = 12,
};

inline unsigned to_bytes_per_pixel(BitsPerPixel bits_per_pixel) {
    switch (bits_per_pixel) {
    case BitsPerPixel::Bits8:
        return 1;
    case BitsPerPixel::Bits10:
        return 2;
    case BitsPerPixel::Bits12:
        return 2;
    default:
        throw std::invalid_argument(std::format(
            "Invalid variant id: {}", static_cast<unsigned>(bits_per_pixel)));
    }
}

struct CameraConfig {
    unsigned width;
    unsigned height;
    unsigned offsetx;
    unsigned offsety;
    float exposure_ms;
    float framerate;
    int num_buffer_frames;
    int fast_binning; // bool
    int input_trigger;
    /* How many frames does one trigger event produce. */
    int trigger_frames;
    BitsPerPixel bits_per_pixel = BitsPerPixel::Bits12;
    int test_pattern = 0;
    int full_mode = 1;
    /* Gain0,1,2,3 (mode index, not the actual gain). */
    int analog_gain = 0;

    std::size_t image_size() {
        return static_cast<std::size_t>(width) * height *
               to_bytes_per_pixel(bits_per_pixel);
    }
};

struct CameraInfo {
    unsigned maxWidth;
    unsigned maxHeight;
    float maxFPS;
    unsigned bytesPerPixel;
};

class Camera {
  public:
    Camera(unsigned num_copy_threads_per_buffer);

    ~Camera();

    void open_camera(const unsigned camera_index);

    void get_config(CameraConfig &config, CameraConfig &minvals,
                    CameraConfig &maxvals);

    void set_config(const CameraConfig &config);

    std::size_t get_frame_sizes(const Camera &camera);

    void get_info(CameraInfo &info);

    void start();

    void stop();

    std::uint64_t get_frame_size();

    std::uint64_t get_lost_frames();
    std::uint64_t get_captured_frames();

    uint8_t *get_next_buffer();

    void get_temperature(float &deviceTemp, float &sensorTemp);

    void set_temperature_control(bool enableFan, bool enableTEC);

    void wait_for_frames();

    void add_buffer(unsigned char *const buffer, const std::size_t frame_size,
                    const std::size_t number_of_frames);

  private:
    std::string get_enum_or_string_element(const char *name);

    TUCAM_ELEMENT get_element(const char *element_name);
    int get_int_element(const char *name, int *min = nullptr,
                        int *max = nullptr);
    int get_unsigned_element(const char *name, unsigned *min = nullptr,
                             unsigned *max = nullptr);
    float get_float_element(const char *name, float *min = nullptr,
                            float *max = nullptr);

    void set_enum_element(const char *name, const char *value);

    void set_int_element(const char *name, int value);
    void set_float_element(const char *name, float value);
    void set_bool_element(const char *name, bool value);

    void frame_callback();
    static void frame_callback_caller(Camera *camera);

    std::vector<std::string> get_element_names();

  private:
    struct FrameBuffer {
        unsigned char *data;
        std::size_t frame_size;
        std::size_t current_frame_id;
        std::size_t max_frames;

        std::unique_ptr<MultiThreadedCopier> copier;

        FrameBuffer(unsigned char *data, std::size_t frame_size,
                    std::size_t number_of_frames, unsigned num_copy_threads)
            : data(data), frame_size(frame_size), current_frame_id(0),
              max_frames(number_of_frames),
              copier(std::make_unique<decltype(copier)::element_type>(
                  num_copy_threads)) {}
    };

    TUCAM_INIT init;
    TUCAM_OPEN camera;
    int capture_mode;

    std::uint64_t num_captured_frames = 0;
    volatile bool capturing = false;
    CameraConfig config;
    std::mutex camera_mutex;

    std::thread capturing_thread;

    unsigned num_lost_frames = 0;
    unsigned bytes_per_pixel = 0;

    TUCAM_FRAME tucam_frame;
    std::condition_variable frame_condition_variable;

    unsigned num_copy_threads_per_buffer;
    std::deque<struct FrameBuffer> empty_frame_buffers;
    std::mutex empty_frame_buffers_mutex;
    std::deque<struct FrameBuffer> full_frame_buffers;
    std::mutex full_frame_buffers_mutex;
};

#ifdef _WIN32

#define DLL_EXPORT __declspec(dllexport)
#define CDLL_EXPORT extern "C" DLL_EXPORT
CDLL_EXPORT Camera *camera_open(int camera_index,
                                unsigned num_copy_threads_per_buffer);

CDLL_EXPORT void camera_close(Camera *camera);

CDLL_EXPORT void get_config(Camera *camera, CameraConfig &config,
                            CameraConfig &minvals, CameraConfig &maxvals);

CDLL_EXPORT void get_info(Camera *camera, CameraInfo &info);

CDLL_EXPORT void set_config(Camera *camera, const CameraConfig &config);

CDLL_EXPORT void start(Camera *camera);

CDLL_EXPORT void stop(Camera *camera);

CDLL_EXPORT std::size_t get_frame_size(Camera *camera);

CDLL_EXPORT void add_buffer(Camera *camera, unsigned char *const buffer,
                            const std::size_t frame_size,
                            const std::size_t number_of_frames);

CDLL_EXPORT void get_next_buffer(Camera *camera, std::uint8_t **data);

CDLL_EXPORT std::uint64_t get_lost_frames(Camera *camera);

CDLL_EXPORT void get_temperature(Camera *camera, float &deviceTemp,
                                 float &sensorTemp);

CDLL_EXPORT void set_temperature_control(Camera *camera, bool enableFan,
                                         bool enableTEC);
#endif
