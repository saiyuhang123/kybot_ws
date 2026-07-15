#include "hk_camera/hk_decoder.h"
#include "hk_camera/hk_camera.h"

namespace hk_camera
{

// ============================================================
// RTSP URL 构造
// ============================================================
std::string HKDecoder::buildRtspUrl(const CameraLoginInfo& info, int channel)
{
    char buf[256];
    snprintf(buf, sizeof(buf),
             "rtsp://%s:%s@%s:554/h264/ch%d/main/av_stream",
             info.username.c_str(),
             info.password.c_str(),
             info.ip.c_str(),
             channel);
    return std::string(buf);
}

// ============================================================
// GStreamer 硬解码 pipeline (Jetson NVDEC)
// ============================================================
std::string HKDecoder::buildGstPipeline(const std::string& url)
{
    // uridecodebin 自动处理 RTSP → 解码, 优先选硬解 (nvv4l2decoder)
    // nvvidconv/nvvideoconvert 做 GPU 色彩转换
    std::string pipeline =
        "uridecodebin uri=" + url +
        " ! nvvidconv"
        " ! video/x-raw,format=BGRx"
        " ! videoconvert"
        " ! video/x-raw,format=BGR"
        " ! appsink drop=1 max-buffers=2";

    return pipeline;
}

// ============================================================
HKDecoder::HKDecoder(QObject* parent)
    : QObject(parent)
{
}

HKDecoder::~HKDecoder()
{
    stop();
}

bool HKDecoder::start(const CameraLoginInfo& info, int channel)
{
    if (running_) return true;

    login_info_ = info;
    channel_    = channel;
    running_    = true;

    // 优先尝试硬解码
    thread_ = std::thread(&HKDecoder::captureLoopHW, this);
    return true;
}

void HKDecoder::stop()
{
    running_ = false;
    if (thread_.joinable())
    {
        thread_.join();
    }
}

bool HKDecoder::isRunning() const
{
    return running_;
}

// ============================================================
// 硬解码路径 (cv::CAP_GSTREAMER → nvv4l2decoder)
// ============================================================
void HKDecoder::captureLoopHW()
{
    std::string url  = buildRtspUrl(login_info_, channel_);
    std::string pipe = buildGstPipeline(url);

    printf("[Decoder-HW] Trying GStreamer hardware decode...\n");
    printf("[Decoder-HW] Pipeline: %s\n", pipe.c_str());
    fflush(stdout);

    cv::VideoCapture cap;
    cap.open(pipe, cv::CAP_GSTREAMER);

    if (!cap.isOpened())
    {
        fprintf(stderr, "[Decoder-HW] GStreamer open FAILED, "
                "falling back to software decode\n");
        fflush(stderr);
        captureLoop();
        return;
    }

    printf("[Decoder-HW] Hardware decode OK (nvv4l2decoder)\n");
    fflush(stdout);

    int frame_count = 0;
    cv::Mat frame;

    while (running_)
    {
        if (!cap.read(frame) || frame.empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        frame_count++;

        cv::Mat rgb;
        cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);

        QImage img(rgb.data, rgb.cols, rgb.rows,
                   static_cast<int>(rgb.step),
                   QImage::Format_RGB888);
        QImage copy = img.copy();

        if (frame_count == 1 || frame_count % 30 == 0)
        {
            printf("[Decoder-HW] frame=%d, size=%dx%d\n",
                   frame_count, copy.width(), copy.height());
            fflush(stdout);
        }

        emit frameDecoded(copy);
    }

    cap.release();
    printf("[Decoder-HW] loop ended, frames=%d\n", frame_count);
    fflush(stdout);
}

// ============================================================
// 软解码路径 (cv::CAP_FFMPEG, 仅在硬解失败时使用)
// ============================================================
void HKDecoder::captureLoop()
{
    std::string url = buildRtspUrl(login_info_, channel_);
    printf("[Decoder-SW] Falling back to FFmpeg software decode: %s\n",
           url.c_str());
    fflush(stdout);

    cv::VideoCapture cap;
    cap.open(url, cv::CAP_FFMPEG);

    if (!cap.isOpened())
    {
        fprintf(stderr, "[Decoder-SW] FFmpeg open FAILED\n");
        fflush(stderr);
        running_ = false;
        return;
    }

    printf("[Decoder-SW] Software decode OK\n");
    fflush(stdout);

    int frame_count = 0;
    cv::Mat frame;

    while (running_)
    {
        if (!cap.read(frame) || frame.empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        frame_count++;

        cv::Mat rgb;
        cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);

        QImage img(rgb.data, rgb.cols, rgb.rows,
                   static_cast<int>(rgb.step),
                   QImage::Format_RGB888);
        QImage copy = img.copy();

        if (frame_count == 1 || frame_count % 30 == 0)
        {
            printf("[Decoder-SW] frame=%d, size=%dx%d\n",
                   frame_count, copy.width(), copy.height());
            fflush(stdout);
        }

        emit frameDecoded(copy);
    }

    cap.release();
    printf("[Decoder-SW] loop ended, frames=%d\n", frame_count);
    fflush(stdout);
}

} // namespace hk_camera
