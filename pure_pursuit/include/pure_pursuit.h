#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <vector>

namespace f110
{

/*std::vector<WayPoint> transform(const std::vector<WayPoint>& reference_way_points, const WayPoint& current_way_point,
        const std::shared_ptr<tf2_ros::Buffer>& tfBuffer, const std::shared_ptr<tf2_ros::TransformListener>& tf2_listener)
{
    geometry_msgs::msg::TransformStamped map_to_base_link;
    map_to_base_link = tfBuffer->lookupTransform("map", "base_link", tf2::TimePointZero);

    std::vector<WayPoint> transformed_way_points;
    for(const auto& reference_way_point: reference_way_points)
    {
        geometry_msgs::msg::PoseStamped map_way_point;
        map_way_point.pose.position.x = reference_way_point.x;
        map_way_point.pose.position.y = reference_way_point.y;
        map_way_point.pose.position.z = 0;
        map_way_point.pose.orientation.x = 0;
        map_way_point.pose.orientation.y = 0;
        map_way_point.pose.orientation.z = 0;
        map_way_point.pose.orientation.w = 1;

        tf2::doTransform(map_way_point, map_way_point, map_to_base_link);

        transformed_way_points.emplace_back(map_way_point);
    }
    return transformed_way_points;
}
*/

size_t get_best_track_point_index(const std::vector<f110::WayPoint> way_point_data, const WayPoint& current_way_point, double lookahead_distance, size_t& last_best_index)
{
    double closest_distance = std::numeric_limits<double>::max();
    const size_t way_point_size = way_point_data.size();

    auto update_best_index_within_interval = [&](const size_t start_index, const size_t end_index){
        for(size_t i=start_index; i <end_index; ++i)
        {
	    double x_diff = way_point_data[i].x - current_way_point.x;
	    double y_diff = way_point_data[i].y - current_way_point.y;
            double distance = sqrt(x_diff*x_diff + y_diff*y_diff);
            double lookahead_diff = std::abs(distance - lookahead_distance);
            if(lookahead_diff < closest_distance)
            {
                closest_distance = lookahead_diff;
                last_best_index = i;
            }
        }
    };

    if(last_best_index > way_point_size - way_point_size/10)
    {
        update_best_index_within_interval(last_best_index, way_point_size);
        update_best_index_within_interval(0, 100);
    }
    else
    {
        update_best_index_within_interval(last_best_index, std::min(last_best_index + 100, way_point_size));
    }
    RCLCPP_DEBUG(rclcpp::get_logger("pure_pursuit.h"), "closest_way_point_index %i", static_cast<int>(last_best_index));
    return last_best_index;
}

} // namespace f110
