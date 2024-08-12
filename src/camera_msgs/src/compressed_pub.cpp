#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "image_transport/image_transport.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

class ImageCompressionNode : public rclcpp::Node
{
public:
    ImageCompressionNode()
    : Node("image_compression_node")
    {
        // Create image transport
        image_transport::ImageTransport it(this);

        // Create subscribers for raw images and depth images
        image_sub_ = it.subscribe("oak_pro/left", 1, std::bind(&ImageCompressionNode::imageCallback, this, std::placeholders::_1));
        depth_sub_ = it.subscribe("oak_pro/depth", 1, std::bind(&ImageCompressionNode::depthCallback, this, std::placeholders::_1));

        // Create publishers for compressed images
        image_compressed_pub_ = it.advertise("oak_pro/left_compressed", 1);
        depth_compressed_pub_ = it.advertise("oak_pro/depth_compressed", 1);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        // Convert ROS image message to OpenCV Mat
        cv::Mat cv_image(cv::Size(msg->width, msg->height), CV_mono8, const_cast<uint8_t*>(msg->data.data()));

        // Compress image
        std::vector<uint8_t> buffer;
        cv::imencode(".jpg", cv_image, buffer); // Use JPEG compression

        // Create CompressedImage message
        sensor_msgs::msg::CompressedImage::SharedPtr compressed_msg(new sensor_msgs::msg::CompressedImage());
        compressed_msg->header = msg->header;
        compressed_msg->format = "jpeg";
        compressed_msg->data = buffer;

        // Publish compressed image
        image_compressed_pub_->publish(*compressed_msg);
        RCLCPP_INFO(this->get_logger(), "Published compressed image");
    }

    void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        // Convert ROS image message to OpenCV Mat
        cv::Mat cv_depth(cv::Size(msg->width, msg->height), CV_16UC1, const_cast<uint8_t*>(msg->data.data()));

        // Compress depth image using PNG
        std::vector<uint8_t> buffer;
        cv::imencode(".png", cv_depth, buffer); // Use PNG compression for lossless compression

        // Create CompressedImage message
        sensor_msgs::msg::CompressedImage::SharedPtr compressed_msg(new sensor_msgs::msg::CompressedImage());
        compressed_msg->header = msg->header;
        compressed_msg->format = "png";
        compressed_msg->data = buffer;

        // Publish compressed depth image
        depth_compressed_pub_->publish(*compressed_msg);
        RCLCPP_INFO(this->get_logger(), "Published compressed depth image");
    }

    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;
    image_transport::Publisher image_compressed_pub_;
    image_transport::Publisher depth_compressed_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageCompressionNode>());
    rclcpp::shutdown();
    return 0;
}
