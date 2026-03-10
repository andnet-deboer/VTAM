#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include <visp3/core/vpImageConvert.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayOpenCV.h>

typedef enum { STATE_DETECTION, STATE_TRACKING } state_t;

class UmiCubeTrackerNode : public rclcpp::Node {
public:
    UmiCubeTrackerNode() : Node("umi_cube_tracker_node"), state_(STATE_DETECTION) {
        // 1. Setup Detector
        detector_ = std::make_shared<vpDetectorAprilTag>(vpDetectorAprilTag::TAG_36h11);
        detector_->setAprilTagQuadDecimate(2.0);

        // 2. Setup Tracker (Edge Only)
        tracker_ = std::make_shared<vpMbGenericTracker>(std::vector<int>{vpMbGenericTracker::EDGE_TRACKER});
        
        vpMe me;
        me.setMaskSize(5);
        me.setRange(20); 
        me.setThreshold(25);
        me.setSampleStep(4);
        tracker_->setMovingEdge(me);

        // 3. INTERNAL GEOMETRY (Meter Scale)
        // This defines a 64mm square on the face of an 85mm cube
        createInternalModel(0.085, 0.064);

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("umi_cube_pose", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/camera/color/image_raw", 10, std::bind(&UmiCubeTrackerNode::image_callback, this, std::placeholders::_1));
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera/camera/color/camera_info", 10, std::bind(&UmiCubeTrackerNode::info_callback, this, std::placeholders::_1));
    }

private:
    void createInternalModel(double cube_sz, double tag_sz) {
        double cc = cube_sz / 2.0; 
        double tc = tag_sz / 2.0;
        
        std::string temp_model_path = "/tmp/internal_umi.cao";
        
        std::ofstream f(temp_model_path);
        if (!f.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open %s for writing!", temp_model_path.c_str());
            return;
        }

        f << "V1\n# Points\n4\n" 
          << tc << " " << tc << " " << cc << "\n"
          << tc << " " << -tc << " " << cc << "\n"
          << -tc << " " << -tc << " " << cc << "\n"
          << -tc << " " << tc << " " << cc << "\n"
          << "# Lines\n4\n0 1\n1 2\n2 3\n3 0\n# Faces\n1\n4 0 3 2 1\n# Cylinders\n0\n# Circles\n0\n";
        f.close();

        RCLCPP_INFO(this->get_logger(), "Internal model written to %s", temp_model_path.c_str());
        
        try {
            tracker_->loadModel(temp_model_path);
        } catch (const vpException &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load internal model: %s", e.what());
        }
    }

    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (cam_init_) return;
        cam_.initPersProjWithDistortion(msg->k[0], msg->k[4], msg->k[2], msg->k[5], msg->d[0], msg->d[1]);
        tracker_->setCameraParameters(cam_);
        cam_init_ = true;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!cam_init_) return;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        vpImage<vpRGBa> I_color;
        vpImage<unsigned char> I_gray;
        vpImageConvert::convert(cv_ptr->image, I_color);
        vpImageConvert::convert(I_color, I_gray);

        if (!disp_init_) {
            display_ = std::make_unique<vpDisplayOpenCV>(I_color, 0, 0, "UMI Tracker Debug");
            disp_init_ = true;
        }
        vpDisplay::display(I_color);

        if (state_ == STATE_DETECTION) {
            std::vector<vpHomogeneousMatrix> cMo_vec;
            if (detector_->detect(I_gray, 0.064, cam_, cMo_vec)) {
                for (size_t i = 0; i < detector_->getTagsId().size(); i++) {
                    if (detector_->getTagsId()[i] == 5) {
                        // Tag 5 handoff: Z points at camera, so push 42.5mm IN (Positive Z)
                        // Then flip 180 on X to align cube 'Up' with camera 'Down'
                        vpTranslationVector t_off(0, 0, 0.085/2.0);
                        vpRotationMatrix r_flip(vpRzyxVector(0, 0, M_PI)); 
                        cMo_ = cMo_vec[i] * vpHomogeneousMatrix(t_off, r_flip);
                        
                        tracker_->initFromPose(I_gray, cMo_);
                        state_ = STATE_TRACKING;
                        break;
                    }
                }
            }
        } else {
            try {
                tracker_->track(I_gray);
                tracker_->getPose(cMo_);
                tracker_->display(I_color, cMo_, cam_, vpColor::red, 2);
                publish_pose(msg->header.stamp);
            } catch (...) { state_ = STATE_DETECTION; }
        }
        vpDisplay::flush(I_color);
    }

    void publish_pose(const rclcpp::Time& stamp) {
        vpTranslationVector t; cMo_.extract(t);
        vpQuaternionVector q; cMo_.extract(q);
        geometry_msgs::msg::PoseStamped p;
        p.header.stamp = stamp; p.header.frame_id = "camera_color_optical_frame";
        p.pose.position.x = t[0]; p.pose.position.y = t[1]; p.pose.position.z = t[2];
        p.pose.orientation.x = q.x(); p.pose.orientation.y = q.y();
        p.pose.orientation.z = q.z(); p.pose.orientation.w = q.w();
        pose_pub_->publish(p);
    }

    state_t state_;
    vpCameraParameters cam_;
    bool cam_init_ = false, disp_init_ = false;
    vpHomogeneousMatrix cMo_;
    std::shared_ptr<vpDetectorAprilTag> detector_;
    std::shared_ptr<vpMbGenericTracker> tracker_;
    std::unique_ptr<vpDisplayOpenCV> display_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UmiCubeTrackerNode>());
    rclcpp::shutdown();
    return 0;
}