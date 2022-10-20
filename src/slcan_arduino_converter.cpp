#include <ros/ros.h>
#include<nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/String.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>


namespace slcan_arduino_converter
{
    class SlcanArduinoConverter : public nodelet::Nodelet
    {
        private:
            ros::NodeHandle nh_;
            ros::Subscriber arduino_tx_sub_;
            ros::Publisher arduino_rx_pub_;
        public:
            void onInit() override{
                nh_ = getNodeHandle();

                arduino_tx_sub_ = nh_.subscribe("arduino_tx", 1, &SlcanArduinoConverter::arduinoTxCallback, this);
                arduino_rx_pub_ = nh_.advertise<std_msgs::String>("arduino_rx", 1);

            }   

            void arduinoTxCallback(const std_msgs::String::ConstPtr& msg){
                
            }

        
    };

} // namespace arduino_bridge
PLUGINLIB_EXPORT_CLASS(slcan_arduino_converter::SlcanArduinoConverter, nodelet::Nodelet);