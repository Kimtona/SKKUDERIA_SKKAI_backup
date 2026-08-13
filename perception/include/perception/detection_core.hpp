#ifndef PERCEPTION__DETECTION_CORE_HPP_
#define PERCEPTION__DETECTION_CORE_HPP_

#include <cstddef>
#include <functional>
#include <vector>

namespace perception
{

struct ScanPoint
{
  double x;
  double y;
  double range;
  bool valid;
};

struct MapPoint
{
  double x;
  double y;
};

struct DetectedObstacle
{
  int id;
  double center_x;
  double center_y;
  double size;
  double theta;
};

struct DetectionConfig
{
  double lambda_rad{10.0 * 3.14159265358979323846 / 180.0};
  double sigma_m{0.03};
  double new_cluster_threshold_m{0.4};
  double min_obstacle_size_m{0.2};
  double max_obstacle_size_m{1.0};
  double min_two_points_distance_m{0.01};
  std::size_t min_points{10U};
  // The centre of a fitted obstacle is found by stepping inward from the corner
  // nearest the sensor. Legacy behaviour steps by half of `size`, the LARGER of
  // the two observed extents, along BOTH axes -- i.e. it assumes the obstacle is
  // square. For a car seen side-on (about 0.58 m by 0.20 m) that pushes the
  // centre roughly 0.19 m too far from the sensor across the narrow axis.
  //
  // That matters because the centre, not the near edge, is what
  // state_machine._check_ofree compares against lateral_width_ot_m: a centre
  // displaced away from the ego makes an overtake look clear when it is not.
  // With this false, the step uses each axis's own extent, putting the centre at
  // the centre of what was actually observed.
  bool square_obstacle_fit{true};
};

using Cluster = std::vector<ScanPoint>;
using PointPredicate = std::function<bool (double, double)>;

[[nodiscard]] std::vector<Cluster> clusterScan(
  const std::vector<ScanPoint> & points,
  double angle_increment,
  const DetectionConfig & config,
  const PointPredicate & is_on_track);

[[nodiscard]] std::vector<DetectedObstacle> fitLShapes(
  const std::vector<Cluster> & clusters,
  const MapPoint & sensor_position,
  const DetectionConfig & config);

[[nodiscard]] std::vector<DetectedObstacle> filterAndNumberObstacles(
  const std::vector<DetectedObstacle> & obstacles,
  const DetectionConfig & config);

[[nodiscard]] double wrapPositive(double value, double period);

}  // namespace perception

#endif  // PERCEPTION__DETECTION_CORE_HPP_
