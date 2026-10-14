#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <string>

namespace {
double EnvironmentDouble(const char *name, double fallback) {
  const char *value = std::getenv(name);
  if (!value) return fallback;
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return fallback;
  }
}

constexpr double kPi = 3.14159265358979323846;
constexpr double kWaterDensityKgM3 = 997.0;

double LognormalMoment(double median_radius_um, double geometric_sigma, int order) {
  const double median_radius_m = median_radius_um * 1e-6;
  const double logarithmic_sigma = std::log(geometric_sigma);
  return std::pow(median_radius_m, order) *
         std::exp(0.5 * order * order * logarithmic_sigma * logarithmic_sigma);
}

// Number-weighted bimodal distribution based on the same 68% small / 32%
// mature-droplet mixture used by the visual super-particles.
double DistributionMoment(int order) {
  return 0.68 * LognormalMoment(2.2, 1.45, order) +
         0.32 * LognormalMoment(6.0, 1.35, order);
}

double DropletConcentrationM3(double liquid_water_content_g_m3) {
  const double water_mass_kg_m3 = liquid_water_content_g_m3 * 1e-3;
  const double mean_droplet_mass =
      kWaterDensityKgM3 * (4.0 / 3.0) * kPi * DistributionMoment(3);
  return water_mass_kg_m3 / mean_droplet_mass;
}

double MicrophysicalOpticalExtinction(double liquid_water_content_g_m3) {
  // For fog droplets whose size parameter is large at a 905 nm lidar
  // wavelength, the Mie extinction efficiency approaches 2.
  constexpr double mie_extinction_efficiency = 2.0;
  return DropletConcentrationM3(liquid_water_content_g_m3) *
         mie_extinction_efficiency * kPi * DistributionMoment(2);
}
}  // namespace

class FogSensorFilter final : public rclcpp::Node {
 public:
  FogSensorFilter()
      : Node("fog_sensor_filter"),
        requested_visibility_(std::clamp(EnvironmentDouble("EGO_FOG_VISIBILITY_M", 20.0), 8.0, 1000.0)),
        liquid_water_content_(std::clamp(EnvironmentDouble("EGO_FOG_LWC_G_M3", 0.7), 0.01, 2.0)),
        droplet_concentration_m3_(DropletConcentrationM3(liquid_water_content_)),
        microphysical_extinction_(MicrophysicalOpticalExtinction(liquid_water_content_)),
        extinction_(std::max(microphysical_extinction_, 3.912 / requested_visibility_)),
        radar_extinction_(std::clamp(EnvironmentDouble(
            "EGO_FOG_RADAR_DB_KM_PER_G_M3", 4.2), 0.0, 20.0) * liquid_water_content_ *
            std::log(10.0) / 10000.0),
        backscatter_probability_(std::clamp(0.22 + 0.45 * liquid_water_content_, 0.2, 0.62)),
        random_(42), uniform_(0.0, 1.0), normal_(0.0, 1.0) {
    const auto qos = rclcpp::SensorDataQoS();
    scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
        "/ego/dashboard_lidar/scan", qos);
    fog_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
        "/ego/dashboard_lidar/fog_scan", qos);
    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        "/ego/dashboard_camera/fog_image", qos);
    radar_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/ego/front_radar/fog_points", qos);
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/ego/dashboard_lidar/clear_scan", qos,
        std::bind(&FogSensorFilter::FilterScan, this, std::placeholders::_1));
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        "/ego/dashboard_camera/image_raw", qos,
        std::bind(&FogSensorFilter::FilterImage, this, std::placeholders::_1));
    radar_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ego/front_radar/points", qos,
        std::bind(&FogSensorFilter::FilterRadar, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(),
                "Dense fog: requested visibility %.1f m, LWC %.2f g/m^3, droplets %.0f cm^-3, optical extinction %.5f 1/m (physical visibility %.1f m), radar extinction %.7f 1/m",
                requested_visibility_, liquid_water_content_, droplet_concentration_m3_ * 1e-6,
                extinction_, 3.912 / extinction_, radar_extinction_);
  }

 private:
  void FilterScan(const sensor_msgs::msg::LaserScan::SharedPtr input) {
    auto output = *input;
    for (std::size_t index = 0; index < output.ranges.size(); ++index) {
      float &range = output.ranges[index];
      const bool valid_return = std::isfinite(range) && range >= output.range_min && range <= output.range_max;
      const double optical_range = valid_return ? range : output.range_max;
      const double transmission = std::exp(-2.0 * extinction_ * optical_range);
      if (uniform_(random_) > transmission) {
        // A fog droplet can produce a short backscatter return. Otherwise the
        // attenuated beam is reported as a dropout.
        if (uniform_(random_) < backscatter_probability_) {
          const double backscatter = -std::log(std::max(1e-9, 1.0 - uniform_(random_))) /
                                     (2.0 * extinction_);
          range = static_cast<float>(std::clamp(backscatter,
              static_cast<double>(output.range_min), optical_range));
        } else {
          range = std::numeric_limits<float>::infinity();
        }
      } else if (valid_return) {
        const double sigma = 0.01 + 0.018 * range * (1.0 - transmission);
        range = static_cast<float>(std::clamp(range + sigma * normal_(random_),
            static_cast<double>(output.range_min), static_cast<double>(output.range_max)));
      }
      if (index < output.intensities.size())
        output.intensities[index] = static_cast<float>(output.intensities[index] * transmission);
    }
    scan_publisher_->publish(output);
    fog_scan_publisher_->publish(output);
  }

  void FilterImage(const sensor_msgs::msg::Image::SharedPtr input) {
    auto output = *input;
    if (output.height == 0 || output.width == 0 || output.step == 0 || output.data.empty()) return;
    const bool mono = output.encoding == "mono8" || output.encoding == "8UC1";
    const std::size_t channels = mono ? 1u : (output.step / output.width >= 4 ? 4u : 3u);
    const double time_phase = 0.035 * static_cast<double>(image_frame_++);
    for (std::size_t y = 0; y < output.height; ++y) {
      const double normalized_y = static_cast<double>(y) / std::max<std::size_t>(1, output.height - 1);
      const double assumed_depth = 4.0 + 52.0 * std::pow(1.0 - normalized_y, 1.35);
      for (std::size_t x = 0; x < output.width; ++x) {
        const double normalized_x = static_cast<double>(x) / std::max<std::size_t>(1, output.width - 1);
        const double turbulent_density = std::clamp(1.0 + .14 * std::sin(7.0 * normalized_x + 4.3 * normalized_y + time_phase) +
            .07 * std::sin(17.0 * normalized_x - 8.0 * normalized_y - 1.7 * time_phase), .72, 1.28);
        const double haze = std::clamp(1.0 - std::exp(-extinction_ * turbulent_density * assumed_depth), 0.0, 0.97);
        const std::size_t pixel = y * output.step + x * channels;
        for (std::size_t channel = 0; channel < std::min<std::size_t>(channels, 3); ++channel) {
          const double fog_color = mono ? 190.0 : (channel == 0 ? 188.0 : (channel == 1 ? 198.0 : 202.0));
          const double photon_noise = (1.0 + 2.5 * haze) * normal_(random_);
          output.data[pixel + channel] = static_cast<std::uint8_t>(
              std::clamp((1.0 - haze) * output.data[pixel + channel] + haze * fog_color + photon_noise, 0.0, 255.0));
        }
      }
    }
    image_publisher_->publish(output);
  }

  void FilterRadar(const sensor_msgs::msg::PointCloud2::SharedPtr input) {
    int x_offset = -1, y_offset = -1, z_offset = -1, intensity_offset = -1;
    for (const auto &field : input->fields) {
      if (field.name == "x") x_offset = static_cast<int>(field.offset);
      else if (field.name == "y") y_offset = static_cast<int>(field.offset);
      else if (field.name == "z") z_offset = static_cast<int>(field.offset);
      else if (field.name == "intensity") intensity_offset = static_cast<int>(field.offset);
    }
    if (x_offset < 0 || y_offset < 0 || z_offset < 0 || input->point_step == 0 ||
        static_cast<std::size_t>(std::max({x_offset, y_offset, z_offset}) + sizeof(float)) >
            input->point_step) return;

    auto output = *input;
    output.height = 1;
    output.width = 0;
    output.row_step = 0;
    output.is_dense = false;
    output.data.clear();
    output.data.reserve(input->data.size());
    const std::size_t point_count = input->data.size() / input->point_step;
    for (std::size_t point = 0; point < point_count; ++point) {
      const std::uint8_t *source = input->data.data() + point * input->point_step;
      float x = 0.f, y = 0.f, z = 0.f;
      std::memcpy(&x, source + x_offset, sizeof(float));
      std::memcpy(&y, source + y_offset, sizeof(float));
      std::memcpy(&z, source + z_offset, sizeof(float));
      const double range = std::sqrt(x * x + y * y + z * z);
      if (!std::isfinite(range) || range < 0.5) continue;
      const double transmission = std::exp(-2.0 * radar_extinction_ * range);
      if (uniform_(random_) > transmission) continue;

      const std::size_t destination = output.data.size();
      output.data.insert(output.data.end(), source, source + input->point_step);
      const double radial_sigma = 0.025 + 0.0015 * range * (1.0 - transmission);
      const double noisy_range = std::max(0.5, range + radial_sigma * normal_(random_));
      const float scale = static_cast<float>(noisy_range / range);
      x *= scale; y *= scale; z *= scale;
      std::memcpy(output.data.data() + destination + x_offset, &x, sizeof(float));
      std::memcpy(output.data.data() + destination + y_offset, &y, sizeof(float));
      std::memcpy(output.data.data() + destination + z_offset, &z, sizeof(float));
      if (intensity_offset >= 0 && static_cast<std::size_t>(intensity_offset + sizeof(float)) <= input->point_step) {
        float intensity = 0.f;
        std::memcpy(&intensity, output.data.data() + destination + intensity_offset, sizeof(float));
        intensity = static_cast<float>(intensity * transmission);
        std::memcpy(output.data.data() + destination + intensity_offset, &intensity, sizeof(float));
      }
      ++output.width;
    }
    output.row_step = output.width * output.point_step;
    radar_publisher_->publish(output);
  }

  double requested_visibility_;
  double liquid_water_content_;
  double droplet_concentration_m3_;
  double microphysical_extinction_;
  double extinction_;
  double radar_extinction_;
  double backscatter_probability_;
  std::uint64_t image_frame_{0};
  std::mt19937 random_;
  std::uniform_real_distribution<double> uniform_;
  std::normal_distribution<double> normal_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fog_scan_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radar_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FogSensorFilter>());
  rclcpp::shutdown();
  return 0;
}
