#include <vector>

#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/lockfree/queue.hpp>

#include <arduino_bridge/Frame.h>


namespace arduino_bridge
{
    class ArduinoSerial{
        public:
            std::shared_ptr<boost::asio::io_context> io_context_;
            std::shared_ptr<boost::asio::serial_port> port_;
            std::vector<uint8_t> topic_id_;
            bool is_active_ =false;
            ArduinoSerial():port_(new boost::asio::serial_port(io_context_)){}; 
    };

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

            //main thread for publishing arduion_tx data. It works once.
            ros::Timer main_thread_timer_;

            boost::thread_group read_write_threads_;
            
            //serial port group
            std::vector<arduino_bridge::ArduinoSerial> arduino_serials_;
            boost::mutex arduino_serials_mutex_;

            //working queue from read_write_threads_ to main thread.
            boost::lockfree::queue<arduino_bridge::Frame> working_queue_;
            boost::mutex working_queue_mutex_for_push_;

            //serial port settings
            int baud_rate_ = 115200;

        public:
            void onInit() override{
                nh_ = getNodeHandle();
                arduino_tx_sub_ = nh_.subscribe<arduino_bridge::Frame>("arduino_tx", 100, &ArduinoBridge::arduinoTxCallback, this);
                arduino_rx_pub_ = nh_.advertise<arduino_bridge::Frame>("arduino_rx", 100);
                main_thread_timer_ = nh_.createTimer(ros::Duration(0.1), &ArduinoBridge::mainThread, this,true,true);
                baud_rate_ = nh_.param("baud_rate", 115200);

                //you should not heavy task in onInit() function.
                //heavy task is done in mainThread().

            }

        private:
            //This function is called by main_thread_timer_.
            void mainThread(const ros::TimerEvent& event){
                //scan all available serial ports and connect to them.
                scanPortsAndConnect();

                //shakehand with the arduino
                size_t i = 0;
                for(const auto& arduino_serial : arduino_serials_){
                    //handshake on each thread and if it fails, remove the thread.

                    read_write_threads_.create_thread(boost::bind(&ArduinoBridge::handShake, this, arduino_serial));
                    ++i;
                }
                
                read_write_threads_.join_all();

                //if it can't connect to the arduino, close the port and delete the pointer.
                for(auto& arduino_serial : arduino_serials_){
                    if(!arduino_serial.is_active_){
                        arduino_serial.port_->close();
                        arduino_serials_.erase(arduino_serials_.begin() + i);
                    }
                }

                //start read and write threads for each serial port.
                for(const auto& arduino_serial : arduino_serials_){
                    read_write_threads_.create_thread(boost::bind(&ArduinoBridge::comunicateSerialPort, this, arduino_serial));
                }

                //main loop for publishing.
                while(ros::ok()){
                    //publish data from working_queue_.
                    while(!working_queue_.empty()){
                        arduino_bridge::Frame frame;
                        working_queue_.pop(frame);
                        arduino_rx_pub_.publish(frame);
                    }
                        
                    //sleep for 10ms.
                    //you should change this value if you want to change the publish rate.
                    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
                }
            }

            void comunicateSerialPort(arduino_bridge::ArduinoSerial arduino_serial){

            }

            //callback function for arduino_tx topic
            void arduinoTxCallback(const std_msgs::String::ConstPtr& msg){
                
            }

            //add serial port group with mutex
            inline void addSerialPort(arduino_bridge::ArduinoSerial serial_port){
                arduino_serials_mutex_.lock();
                arduino_serials_.push_back(serial_port);
                arduino_serials_mutex_.lock();
            }

            //handshake with the arduino
            bool handShake(arduino_bridge::ArduinoSerial *arduino_serial){
                //if connected, send handshake message
                //if handshake message is received, return true
                //else return false
                if(arduino_serial->port_->is_open()){
                    boost::asio::write(arduino_serial->port_, boost::asio::buffer("HelloCRS", 8));

                    //wait response and check it
                    boost::asio::streambuf response;
                    boost::asio::read_until(arduino_serial->port_, response, "\n");
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
                        if(itr->path().filename().string().substr(0,3) == "tty" || itr->path().extension() == ""){
                            ports.push_back(itr->path().string());
                        }
                    }
                }

                //try to connect to all serial ports
                for (size_t i = 0; i < ports.size(); i++)
                {
                    ArduinoSerial arduino_serial;
                    try{
                        arduino_serial.port_->open(ports[i]);
                    }catch(boost::system::system_error& e){
                        continue;
                    }

                    // set options
                    arduino_serial.port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
                    arduino_serial.port_->set_option(boost::asio::serial_port_base::character_size(8));
                    arduino_serial.port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
                    arduino_serial.port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
                    arduino_serial.port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

                    //add serial port to the group

                    addSerialPort(std::move(arduino_serial));
                }
            }
    };

} // namespace arduino_bridge
PLUGINLIB_EXPORT_CLASS(arduino_bridge::ArduinoBridge, nodelet::Nodelet);