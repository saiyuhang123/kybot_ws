#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cmath>
#include <cstring>
#include <vector>

// ── ring 重映射表 ──────────────────────────────────────────────
static int RING_ID_MAP_RUBY[] = {
        3, 66, 33, 96, 11, 74, 41, 104, 19, 82, 49, 112, 27, 90, 57, 120,
        35, 98, 1, 64, 43, 106, 9, 72, 51, 114, 17, 80, 59, 122, 25, 88,
        67, 34, 97, 0, 75, 42, 105, 8, 83, 50, 113, 16, 91, 58, 121, 24,
        99, 2, 65, 32, 107, 10, 73, 40, 115, 18, 81, 48, 123, 26, 89, 56,
        7, 70, 37, 100, 15, 78, 45, 108, 23, 86, 53, 116, 31, 94, 61, 124,
        39, 102, 5, 68, 47, 110, 13, 76, 55, 118, 21, 84, 63, 126, 29, 92,
        71, 38, 101, 4, 79, 46, 109, 12, 87, 54, 117, 20, 95, 62, 125, 28,
        103, 6, 69, 36, 111, 14, 77, 44, 119, 22, 85, 52, 127, 30, 93, 60
};
static int RING_ID_MAP_16[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8
};

// ── 临时点结构 ─────────────────────────────────────────────────
struct PtXYZIRT  { float x,y,z; float intensity; uint16_t ring; float time; };
struct PtXYZIR   { float x,y,z; float intensity; uint16_t ring; };
struct PtXYZI    { float x,y,z; float intensity; };

// ── 原始数据读取 ────────────────────────────────────────────────
inline float    read_f32(const uint8_t *p) { float v;    std::memcpy(&v, p, 4); return v; }
inline uint8_t  read_u8 (const uint8_t *p) { return *p; }
inline uint16_t read_u16(const uint8_t *p) { uint16_t v; std::memcpy(&v, p, 2); return v; }
inline double   read_f64(const uint8_t *p) { double v;   std::memcpy(&v, p, 8); return v; }

class RsToVelodyne : public rclcpp::Node {
public:
    RsToVelodyne() : Node("rs_to_velodyne") {
        this->declare_parameter<std::string>("input_type", "XYZIRT");
        this->declare_parameter<std::string>("output_type", "XYZIRT");

        input_type_  = get_parameter("input_type").as_string();
        output_type_ = get_parameter("output_type").as_string();

        if (input_type_ != "XYZI" && input_type_ != "XYZIRT") {
            RCLCPP_ERROR(get_logger(), "Unsupported input_type: '%s'", input_type_.c_str());
            rclcpp::shutdown();
            return;
        }
        if (output_type_ != "XYZI" && output_type_ != "XYZIR" && output_type_ != "XYZIRT") {
            RCLCPP_ERROR(get_logger(), "Unsupported output_type: '%s'", output_type_.c_str());
            rclcpp::shutdown();
            return;
        }

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points", rclcpp::SensorDataQoS());

        auto qos = rclcpp::SensorDataQoS();
        if (input_type_ == "XYZI") {
            sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                "/rslidar_points", qos,
                std::bind(&RsToVelodyne::handleXYZI, this, std::placeholders::_1));
        } else {
            sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                "/rslidar_points", qos,
                std::bind(&RsToVelodyne::handleXYZIRT, this, std::placeholders::_1));
        }

        RCLCPP_INFO(get_logger(), "Listening /rslidar_points  input=%s  output=%s",
                    input_type_.c_str(), output_type_.c_str());
    }

private:
    std::string input_type_, output_type_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    bool in_logged_ = false, out_logged_ = false;
    uint64_t frame_count_ = 0;

    static int find_offset(const sensor_msgs::msg::PointCloud2 &m, const std::string &n) {
        for (const auto &f : m.fields) if (f.name == n) return f.offset;
        return -1;
    }
    static bool has_nan(float x, float y, float z) {
        return std::isnan(x) || std::isnan(y) || std::isnan(z);
    }

    void log_input_once(const sensor_msgs::msg::PointCloud2 &m) {
        if (in_logged_) return; in_logged_ = true;
        std::string s;
        for (const auto &f : m.fields) s += f.name + "@" + std::to_string(f.offset) + " ";
        RCLCPP_INFO(get_logger(), "Input : %s pt_step=%u", s.c_str(), m.point_step);
    }
    void log_output_once(const sensor_msgs::msg::PointCloud2 &m) {
        if (out_logged_) return; out_logged_ = true;
        std::string s;
        for (const auto &f : m.fields) s += f.name + "@" + std::to_string(f.offset) + " ";
        RCLCPP_INFO(get_logger(), "Output: %s pt_step=%u", s.c_str(), m.point_step);
    }

    // ══════════════════════════════════════════════════════════════
    //  XYZI → XYZIR
    // ══════════════════════════════════════════════════════════════
    void handleXYZI(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        frame_count_++;
        if (frame_count_ % 100 == 1) {
            RCLCPP_INFO(get_logger(), "[DIAG] frame=%lu  pts=%u  subs=%lu",
                        frame_count_, msg->width * msg->height, pub_->get_subscription_count());
        }
        log_input_once(*msg);
        int ox = find_offset(*msg, "x"), oy = find_offset(*msg, "y");
        int oz = find_offset(*msg, "z"), oi = find_offset(*msg, "intensity");
        if (ox < 0 || oy < 0 || oz < 0 || oi < 0) {
            RCLCPP_ERROR(get_logger(), "Missing x/y/z/intensity"); return;
        }

        std::vector<PtXYZIR> pts;
        const uint8_t *buf = msg->data.data();
        uint32_t n = msg->width * msg->height, step = msg->point_step;
        for (uint32_t i = 0; i < n; ++i) {
            const uint8_t *p = buf + i * step;
            float x = read_f32(p + ox), y = read_f32(p + oy), z = read_f32(p + oz);
            if (has_nan(x, y, z)) continue;
            PtXYZIR pt{x, y, z, float(read_u8(p + oi)), 0};
            if (msg->height == 16)      pt.ring = RING_ID_MAP_16[i / msg->width];
            else if (msg->height == 128) pt.ring = RING_ID_MAP_RUBY[i % msg->height];
            pts.push_back(pt);
        }
        publish_XYZIR(pts, msg->header);
    }

    // ══════════════════════════════════════════════════════════════
    //  XYZIRT → XYZIRT / XYZIR / XYZI
    // ══════════════════════════════════════════════════════════════
    void handleXYZIRT(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        frame_count_++;
        if (frame_count_ % 100 == 1) {
            RCLCPP_INFO(get_logger(), "[DIAG] frame=%lu  pts=%u  subs=%lu",
                        frame_count_, msg->width * msg->height, pub_->get_subscription_count());
        }
        log_input_once(*msg);
        int ox = find_offset(*msg, "x"), oy = find_offset(*msg, "y"), oz = find_offset(*msg, "z");
        int oi = find_offset(*msg, "intensity");
        int oring = find_offset(*msg, "ring");
        int ots   = find_offset(*msg, "timestamp");
        if (oi < 0) oi = find_offset(*msg, "reflectivity");
        if (oring < 0) oring = find_offset(*msg, "laser_id");
        if (ots < 0) ots = find_offset(*msg, "time");

        if (ox < 0 || oy < 0 || oz < 0 || oi < 0) {
            RCLCPP_ERROR(get_logger(), "Missing x/y/z/intensity"); return;
        }

        const uint8_t *buf = msg->data.data();
        uint32_t n = msg->width * msg->height, step = msg->point_step;

        double t0 = 0; bool t0_set = false;
        if (ots >= 0) {
            for (uint32_t i = 0; i < n; ++i) {
                const uint8_t *p = buf + i * step;
                if (!has_nan(read_f32(p + ox), read_f32(p + oy), read_f32(p + oz)))
                    { t0 = read_f64(p + ots); t0_set = true; break; }
            }
        }

        if (output_type_ == "XYZIRT") {
            std::vector<PtXYZIRT> pts;
            for (uint32_t i = 0; i < n; ++i) {
                const uint8_t *p = buf + i * step;
                float x = read_f32(p + ox), y = read_f32(p + oy), z = read_f32(p + oz);
                if (has_nan(x, y, z)) continue;
                pts.push_back({
                    x, y, z,
                    float(read_u8(p + oi)),
                    (oring >= 0) ? read_u16(p + oring) : uint16_t(0),
                    (ots >= 0 && t0_set) ? float(read_f64(p + ots) - t0) : 0.0f
                });
            }
            publish_XYZIRT(pts, msg->header);
        } else if (output_type_ == "XYZIR") {
            std::vector<PtXYZIR> pts;
            for (uint32_t i = 0; i < n; ++i) {
                const uint8_t *p = buf + i * step;
                float x = read_f32(p + ox), y = read_f32(p + oy), z = read_f32(p + oz);
                if (has_nan(x, y, z)) continue;
                pts.push_back({
                    x, y, z,
                    float(read_u8(p + oi)),
                    (oring >= 0) ? read_u16(p + oring) : uint16_t(0)
                });
            }
            publish_XYZIR(pts, msg->header);
        } else {  // XYZI
            std::vector<PtXYZI> pts;
            for (uint32_t i = 0; i < n; ++i) {
                const uint8_t *p = buf + i * step;
                float x = read_f32(p + ox), y = read_f32(p + oy), z = read_f32(p + oz);
                if (has_nan(x, y, z)) continue;
                pts.push_back({x, y, z, float(read_u8(p + oi))});
            }
            publish_XYZI(pts, msg->header);
        }
    }

    // ══════════════════════════════════════════════════════════════
    //  输出（严格匹配 Velodyne ROS2 驱动格式）
    // ══════════════════════════════════════════════════════════════

    // 通用构建函数
    sensor_msgs::msg::PointCloud2 make_cloud_header(const std_msgs::msg::Header &hdr,
                                                     uint32_t npts, uint32_t pt_step) {
        sensor_msgs::msg::PointCloud2 out;
        out.header = hdr;
        out.header.frame_id = "velodyne";
        out.height = 1;
        out.width = npts;
        out.is_dense = true;
        out.point_step = pt_step;
        out.row_step = pt_step * npts;
        out.data.resize(out.row_step, 0);
        return out;
    }

    void publish_XYZIRT(const std::vector<PtXYZIRT> &pts, const std_msgs::msg::Header &hdr) {
        // 完全匹配 Velodyne ROS2 驱动输出的结构体布局:
        // PCL_ADD_POINT4D(16) + PCL_ADD_INTENSITY(4) + ring(2) + 2pad + time(4) + 4pad = 32
        // fields 使用结构体实际成员偏移: x:0 y:4 z:8 intensity:16 ring:20 time:24
        auto out = make_cloud_header(hdr, pts.size(), 32);
        out.header.stamp = this->now();   // 用系统时间，跟 IMU 时间戳对齐

        out.fields.resize(6);
        out.fields[0].set__name("x")         .set__offset(0)  .set__datatype(7).set__count(1);  // FLOAT32=7
        out.fields[1].set__name("y")         .set__offset(4)  .set__datatype(7).set__count(1);
        out.fields[2].set__name("z")         .set__offset(8)  .set__datatype(7).set__count(1);
        out.fields[3].set__name("intensity") .set__offset(16) .set__datatype(7).set__count(1);
        out.fields[4].set__name("ring")      .set__offset(20) .set__datatype(4).set__count(1);  // UINT16=4
        out.fields[5].set__name("time")      .set__offset(24) .set__datatype(7).set__count(1);

        uint8_t *buf = out.data.data();
        for (size_t i = 0; i < pts.size(); ++i) {
            uint8_t *p = buf + i * 32;
            std::memcpy(p + 0,  &pts[i].x,         4);
            std::memcpy(p + 4,  &pts[i].y,         4);
            std::memcpy(p + 8,  &pts[i].z,         4);
            // offset 12-15: padding (PCL_ADD_POINT4D data[3])
            std::memcpy(p + 16, &pts[i].intensity, 4);
            std::memcpy(p + 20, &pts[i].ring,      2);
            // offset 22-23: alignment padding
            std::memcpy(p + 24, &pts[i].time,      4);
            // offset 28-31: struct tail padding
        }
        log_output_once(out);
        pub_->publish(out);
    }

    void publish_XYZIR(const std::vector<PtXYZIR> &pts, const std_msgs::msg::Header &hdr) {
        // VelodynePointXYZIR: PCL_ADD_POINT4D(16) + PCL_ADD_INTENSITY(4) + ring(2) + 2pad = 24
        auto out = make_cloud_header(hdr, pts.size(), 24);
        out.header.stamp = this->now();

        out.fields.resize(5);
        out.fields[0].set__name("x")         .set__offset(0)  .set__datatype(7).set__count(1);
        out.fields[1].set__name("y")         .set__offset(4)  .set__datatype(7).set__count(1);
        out.fields[2].set__name("z")         .set__offset(8)  .set__datatype(7).set__count(1);
        out.fields[3].set__name("intensity") .set__offset(16) .set__datatype(7).set__count(1);
        out.fields[4].set__name("ring")      .set__offset(20) .set__datatype(4).set__count(1);

        uint8_t *buf = out.data.data();
        for (size_t i = 0; i < pts.size(); ++i) {
            uint8_t *p = buf + i * 24;
            std::memcpy(p + 0,  &pts[i].x,         4);
            std::memcpy(p + 4,  &pts[i].y,         4);
            std::memcpy(p + 8,  &pts[i].z,         4);
            std::memcpy(p + 16, &pts[i].intensity, 4);
            std::memcpy(p + 20, &pts[i].ring,      2);
        }
        pub_->publish(out);
    }

    void publish_XYZI(const std::vector<PtXYZI> &pts, const std_msgs::msg::Header &hdr) {
        // pcl::PointXYZI: PCL_ADD_POINT4D(16) + PCL_ADD_INTENSITY(4) + 12pad = 32
        auto out = make_cloud_header(hdr, pts.size(), 32);
        out.header.stamp = this->now();
        out.header.stamp = this->now();   // 用系统时间，跟 IMU 时间戳对齐

        out.fields.resize(4);
        out.fields[0].set__name("x")         .set__offset(0)  .set__datatype(7).set__count(1);
        out.fields[1].set__name("y")         .set__offset(4)  .set__datatype(7).set__count(1);
        out.fields[2].set__name("z")         .set__offset(8)  .set__datatype(7).set__count(1);
        out.fields[3].set__name("intensity") .set__offset(16) .set__datatype(7).set__count(1);

        uint8_t *buf = out.data.data();
        for (size_t i = 0; i < pts.size(); ++i) {
            uint8_t *p = buf + i * 32;
            std::memcpy(p + 0,  &pts[i].x,         4);
            std::memcpy(p + 4,  &pts[i].y,         4);
            std::memcpy(p + 8,  &pts[i].z,         4);
            std::memcpy(p + 16, &pts[i].intensity, 4);
        }
        pub_->publish(out);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RsToVelodyne>());
    rclcpp::shutdown();
    return 0;
}
