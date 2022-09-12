#ifndef GYSFDMAXB_GPS_NODE_H
#define GYSFDMAXB_GPS_NODE_H



// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "gysfdmaxb_gps/gps.h"

namespace gysfdmaxb_gps {

//class gysfdmaxbProduct {
// public:
//  gysfdmaxbProduct(){}
//  void subscribe();
//  void callback();
//};


class GysfdmaxbNode {
 public:
    GysfdmaxbNode();

    void initializeIo();
    void initialize();
    void getRosParams();

  private:

    void publish_void_char(char *data){
      std::cout << data ;
    }

    static void publish_void_str(std::string& data){
      std::cout << "called publish_void_str() \n" << data << std::endl;
    }

    void publish_nmea(const char *data) {
      //static ros::Publisher publisher = nh->advertise<nmea_msgs::Sentence>(topic,
      //                                                          kROSQueueSize);
      //nmea_msgs::Sentence m;
      //m.header.stamp = ros::Time::now();
      //m.header.frame_id = frame_id;
      //m.sentence = sentence;
      //publisher.publish(m);
      std::cout << "called publish_nmea() " << data << std::endl;
    }

    void publish_nmea_str(std::string& data);

    static void subscribe_gps(const char *data) {
      std::cout << "called ubscribe_gps() " << data << std::endl;

    }

    //! Device port
    std::string device_;
    //! dynamic model type
    std::string dynamic_model_;
    //! Fix mode type
    std::string fix_mode_;
    //! Set from dynamic model string
    uint8_t dmodel_;
    //! Set from fix mode string
    uint8_t fmode_;
    //! UART1 baudrate
    uint32_t baudrate_;
    //! UART in protocol (see CfgPRT message for constants)
    uint16_t uart_in_;
    //! UART out protocol (see CfgPRT message for constants)
    uint16_t uart_out_;
    //! USB TX Ready Pin configuration (see CfgPRT message for constants)
    uint16_t usb_tx_;
    //! Whether to configure the USB port
    /*! Set to true if usb_in & usb_out parameters are set */
    bool set_usb_;
    //! USB in protocol (see CfgPRT message for constants)
    uint16_t usb_in_;
    //! USB out protocol (see CfgPRT message for constants)
    uint16_t usb_out_ ;
    //! The measurement rate in Hz
    double rate_;
    //! If true, set configure the User-Defined Datum
    bool set_dat_;

    //gysfdmaxbProduct product;


    //! The ROS frame ID of this device
    std::string frame_id_;
    std::string fix_topic_;

    ros::Publisher fix_publisher_;

    sensor_msgs::NavSatFix fix_;

};


// start node.cpp common variable

//! Node Handle for GPS node
boost::shared_ptr<ros::NodeHandle> nh;


//! Handles communication with the U-Blox Device
gysfdmaxb_gps::Gps gps;


//! Flag for enabling configuration on startup
bool config_on_startup_flag_;


} // nemespace gysfdmaxb_gps


#endif

