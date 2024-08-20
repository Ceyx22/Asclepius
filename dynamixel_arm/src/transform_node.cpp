#include "chrono"

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std;


class Transform : public rclcpp::Node {
    private:

        void update(){
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now();
            t.header.frame_id = "world";
            t.child_frame_id = "base_link";

            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            
            tf_broadcaster_->sendTransform(t);
        }
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;

    public:
        Transform() : Node("transform_node"),  count_(0){
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
            timer_ = this->create_wall_timer(10ms, std::bind(&Transform::update, this));
            this->update();
           
        }
        // ~transform_node();
};


int main(int argc, char * argv[]) {
    std::cout << "Transform Node!";
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Transform>());
    rclcpp::shutdown();
    return 0;
}
