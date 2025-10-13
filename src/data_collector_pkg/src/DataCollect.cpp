#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <filesystem>

namespace fs = std::filesystem;

class DataCollector : public rclcpp::Node
{
public:
    DataCollector() : Node("data_collector"), frame_count_(0)
    {
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&DataCollector::rgbCallback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10,
            std::bind(&DataCollector::depthCallback, this, std::placeholders::_1));

        fs::create_directories("dataset/rgb");
        fs::create_directories("dataset/depth");

        RCLCPP_INFO(this->get_logger(), "Press 'c' to capture frame (RGB + Depth)");
        RCLCPP_INFO(this->get_logger(), "Press 'q' to quit the viewer");
    }

    ~DataCollector()
    {
        cv::destroyAllWindows();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    sensor_msgs::msg::Image::SharedPtr latest_rgb_;
    sensor_msgs::msg::Image::SharedPtr latest_depth_;
    size_t frame_count_;

    void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        latest_rgb_ = msg;
        showPreview();
        handleKeypress();
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        latest_depth_ = msg;
    }

    void showPreview()
    {
        if (!latest_rgb_)
            return;

        try
        {
            cv_bridge::CvImagePtr cv_rgb = cv_bridge::toCvCopy(latest_rgb_, "bgr8");
            cv::imshow("RGB Preview", cv_rgb->image);
            cv::waitKey(1);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
        }
    }

    void handleKeypress()
    {
        char key = getKeyNonBlocking();
        if (key == 'c' && latest_rgb_ && latest_depth_)
        {
            saveData();
        }
        else if (key == 'q')
        {
            RCLCPP_INFO(this->get_logger(), "Exiting viewer...");
            rclcpp::shutdown();
        }
    }

    char getKeyNonBlocking()
    {
        int bytesWaiting;
        ioctl(STDIN_FILENO, FIONREAD, &bytesWaiting);
        if (bytesWaiting > 0)
        {
            char c = getchar();
            return c;
        }
        return 0;
    }

    void saveData()
    {
        cv_bridge::CvImagePtr cv_rgb, cv_depth;
        try
        {
            cv_rgb = cv_bridge::toCvCopy(latest_rgb_, "bgr8");
            cv_depth = cv_bridge::toCvCopy(latest_depth_, "32FC1");
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
            return;
        }

        char filename[100];
        sprintf(filename, "frame_%04zu", frame_count_);

        cv::imwrite("dataset/rgb/" + std::string(filename) + ".png", cv_rgb->image);
        cv::imwrite("dataset/depth/" + std::string(filename) + ".png", cv_depth->image);

        RCLCPP_INFO(this->get_logger(), "Captured frame %zu", frame_count_);
        frame_count_++;
    }
};

int main(int argc, char **argv)
{
    termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataCollector>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return 0;
}
