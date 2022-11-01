#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <arduino_bridge/Frame.h>
#include <arduino_bridge/can_utils_rev.hpp>


namespace test
{
    class Test final : public nodelet::Nodelet
    {
        ros::NodeHandle nh_;
        ros::Subscriber arduino_tx_sub_;
        ros::Publisher arduino_rx_pub_;
        ros::Timer timer_;
        int counter = 0;
        void onInit() override
        {
            nh_ = getMTNodeHandle();
            arduino_tx_sub_ = nh_.subscribe("arduino_tx", 100, &Test::arduinoTxCallback, this);
            arduino_rx_pub_ = nh_.advertise<arduino_bridge::Frame>("arduino_rx", 100);
            timer_ = nh_.createTimer(ros::Duration(1.0), &Test::timerCallback, this);
            NODELET_INFO("Test initialized");
        }

        void arduinoTxCallback(const arduino_bridge::Frame::ConstPtr& msg)
        {
            if(msg->topic_name == "")
            {
                ROS_INFO("test");
            }
        }
        void timerCallback(const ros::TimerEvent& event)
        {
            NODELET_INFO("timerCallback");
            if(counter ==190){
                counter = 0;
            }
            arduino_bridge::Frame msg;
            msg.topic_name = "servo";
            auto data_ = can_utils::pack<int>(counter, can_utils::BIG);
            msg.data = std::vector<uint8_t>(data_.begin(),data_.end());
            arduino_rx_pub_.publish(msg);
            counter++;
        }
    };
}// namespace test
PLUGINLIB_EXPORT_CLASS(test::Test, nodelet::Nodelet)


