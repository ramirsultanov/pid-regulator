#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <car_msgs/MotorsControl.h>
#include <stdint.h>

void image_callback(const sensor_msgs::Image::ConstPtr& msg);
void motors(int16_t left, int16_t right);
int16_t truncate(int16_t pwm);

const uint16_t MAX_PWM = 255;

ros::Publisher pub;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_follower");
    ros::NodeHandle nh;

    auto sub = nh.subscribe("/car_gazebo/camera1/image_raw", 5, image_callback);
    pub = nh.advertise<car_msgs::MotorsControl>("/motors_commands", 10);

    ros::spin();
    return 0;
}

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{        
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;
    
    cv::Mat hsvImage;
    cv:cvtColor(image, hsvImage, CV_BGR2HSV);

    // Threshold the HSV image, keep only the yellow pixels
    cv::Mat mask;
    cv::Scalar lower_red(0, 100, 100);
    cv::Scalar upper_red(10, 255, 255);
    cv::inRange(hsvImage, lower_red, upper_red, mask);

    int width = mask.cols;
    int height = mask.rows;

    int search_top = 7 * height / 8;
    int search_bottom = height;

    // Zero out pixels outside the desired region
    for (int y = 0; y < height - 2; y++) {
        if (y < search_top || y > search_bottom) {
            for (int x = 0; x < width; x++) {
                mask.at<cv::Vec3b>(y, x)[0] = 0;
                mask.at<cv::Vec3b>(y, x)[1] = 0;
                mask.at<cv::Vec3b>(y, x)[2] = 0;
            }
        }
    }

    // Use the moments() function to calculate the centroid of the blob of the binary image
    cv::Moments M = cv::moments(mask);

    if (M.m00 > 0) {
        int cx = int(M.m10 / M.m00);
        int cy = int(M.m01 / M.m00);
        cv::circle(image, cv::Point(cx, cy), 20, CV_RGB(255, 0, 0), -1);

        // Move the robot in proportion to the error signal
        int err = cx - width / 2;
        //geometry_msgs::Twist cmd;
        //cmd.linear.x = 0.1;
        //cmd.angular.z = -(float)err / 500;
        const float angle = -err / 10;
        const float wheelDist = 0.2;
        const float speed = 12;
        const float right = (angle * wheelDist) / 2 + speed;
float left = speed * 2 - right;
        motors(left, right);
    }

    // TODO: control the motors
    // motors(50,50);

    cv::imshow("img", image);
    cv::waitKey(1);
}

void motors(int16_t left, int16_t right)
{
    car_msgs::MotorsControl msg;
    msg.left = truncate(left);
    msg.right = truncate(right);
    pub.publish(msg);
}

int16_t truncate(int16_t pwm)
{
    if(pwm < -MAX_PWM)
        return -MAX_PWM;
    if(pwm > MAX_PWM)
        return MAX_PWM;
    return pwm;
}
