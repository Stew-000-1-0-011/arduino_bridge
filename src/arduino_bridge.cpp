#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/String.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <vector>
#include <boost/filesystem.hpp>
#include <future>

namespace arduino_bridge
{

    //This class is used usb-serial communication with arduinos.
    //First, it tries to find the arduino by all available ports.
    //If it finds the arduino, it tries to connect to it.
    //If it connects to the arduino, it starts to read and write data.
    class ArduinoBridge : public nodelet::Nodelet
    {
        private:
            ros::NodeHandle nh_;
            ros::Subscriber arduino_tx_sub_;
            ros::Publisher arduino_rx_pub_;

            //serial port group
            boost::asio::io_service io_service_;
            
            //you don't use this variable directly.
            std::vector<boost::asio::serial_port*> serial_ports_;
            //mutex for serial port group
            boost::mutex serial_ports_mutex_;


            //read and write threads for each serial port.
            boost::thread_group read_write_threads_;

            //serial port settings
            int baud_rate_ = 115200;




        public:
            void onInit() override{
                nh_ = getNodeHandle();
                arduino_tx_sub_ = nh_.subscribe("arduino_tx", 1, &ArduinoBridge::arduinoTxCallback, this);
                arduino_rx_pub_ = nh_.advertise<std_msgs::String/*TODO*/>("arduino_rx", 1);
                baud_rate_ = nh_.param("baud_rate", 115200);

                //find all available serial ports
                scanPortsAndConnect();

                //shakehand with the arduino
                for(int i = 0, n = serial_ports_.size(); i < n; i++){
                    if(serial_ports_[i] != nullptr){
                        bool can_connect = handShake(*serial_ports_[i]);
                        //if it can't connect to the arduino, close the port and delete the pointer.
                        if(!can_connect){
                            serial_ports_[i]->close();
                            serial_ports_.erase(serial_ports_.begin() + i);
                        }
                    }
                }

            }   

        private:
            //callback function for arduino_tx topic
            void arduinoTxCallback(const std_msgs::String::ConstPtr& msg){
                
            }

            //add serial port group with mutex
            inline void addSerialPort(boost::asio::serial_port* serial_port){
                serial_ports_mutex_.lock();
                serial_ports_.push_back(serial_port);
                serial_ports_mutex_.unlock();
            }




            //handshake with the arduino
            bool handShake(boost::asio::serial_port& serial_port){
                //if connected, send handshake message
                //if handshake message is received, return true
                //else return false
                if(serial_port.is_open()){
                    boost::asio::write(serial_port, boost::asio::buffer("HelloCRS", 8));

                    //wait response and check it
                    boost::asio::streambuf response;
                    boost::asio::read_until(serial_port, response, "\n");
                    if(response.size() > 0){
                        std::istream response_stream(&response);
                        std::string response_string;
                        std::getline(response_stream, response_string);
                        if(response_string == "HelloCRS"){
                            return true;
                        }
                    }
                }

            }

            void scanPortsAndConnect(){
                //scan all serianl ports
                std::vector<std::string> ports;
                boost::filesystem::directory_iterator itr = boost::filesystem::directory_iterator(boost::filesystem::absolute("/dev"));
                for(; itr != boost::filesystem::directory_iterator(); ++itr)
                {
                    if(boost::filesystem::is_symlink(itr->status())){
            
                    }     
                    else if(boost::filesystem::is_regular_file(itr->status())){
                        if(itr->path().filename().string().substr(0,3) == "tty"||itr->path().extension() == ""){
                            ports.push_back(itr->path().string());
                        }
                    }
                }

                //try to connect to all serial ports
                for (size_t i = 0; i < ports.size(); i++)
                {
                    boost::asio::serial_port serial_port(io_service_);
                    serial_port.open(ports[i]);

                    if(serial_port.is_open()){
                        break;
                    }
                    //set options
                    serial_port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
                    serial_port.set_option(boost::asio::serial_port_base::character_size(8));
                    serial_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
                    serial_port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
                    serial_port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

                    //add serial port to the group
                    addSerialPort(&serial_port);
                }
            };
    };

} // namespace arduino_bridge
PLUGINLIB_EXPORT_CLASS(arduino_bridge::ArduinoBridge, nodelet::Nodelet);