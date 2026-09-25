#pragma once

#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gz/math/Vector3.hh>

namespace mid360_simulation_plugin_ros2
{
struct ScanSample
{
  gz::math::Vector3d direction;
  std::uint8_t line;
};

inline std::vector<ScanSample> LoadScanPattern(const std::string &path)
{
  std::ifstream input(path);
  if (!input.is_open())
    throw std::runtime_error("cannot open scan pattern: " + path);

  std::vector<ScanSample> samples;
  std::string row;
  std::getline(input, row);
  std::size_t index = 0;
  constexpr double degToRad = M_PI / 180.0;
  while (std::getline(input, row))
  {
    if (row.empty())
      continue;
    std::stringstream stream(row);
    std::string timeText, azimuthText, zenithText;
    if (!std::getline(stream, timeText, ',') ||
        !std::getline(stream, azimuthText, ',') ||
        !std::getline(stream, zenithText, ','))
      continue;

    const double azimuth = std::stod(azimuthText) * degToRad;
    const double zenith = std::stod(zenithText) * degToRad - M_PI_2;
    const double c = std::cos(zenith);
    samples.push_back({
      gz::math::Vector3d(c * std::cos(azimuth),
                         c * std::sin(azimuth),
                         -std::sin(zenith)),
      static_cast<std::uint8_t>(index % 4)});
    ++index;
  }
  if (samples.empty())
    throw std::runtime_error("scan pattern contains no valid samples: " + path);
  return samples;
}
}  // namespace mid360_simulation_plugin_ros2
