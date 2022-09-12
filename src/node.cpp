/*-----------------------------------
- gysfdmaxb_gps
-   node.cpp
-
- みちびきのデータ形式は、下記参照
- https://www.petitmonte.com/robot/howto_gysfdmaxb.html
-
------------------------------------*/

#include "gysfdmaxb_gps/node.h"

using namespace gysfdmaxb_gps;


#ifdef USE_BUNKAI
// NMEAの緯度経度を「度分秒」(DMS)の文字列に変換する
std::string NMEA2DMS(float val) {
  int d = val / 100;
  int m = ((val / 100.0) - d) * 100.0;
  float s = ((((val / 100.0) - d) * 100.0) - m) * 60;
  return std::to_string(d) + "度" + std::to_string(m) + "分" + std::to_string(s, 1) + "秒";
}
 
// (未使用)NMEAの緯度経度を「度分」(DM)の文字列に変換する
std::string NMEA2DM(float val) {
  int d = val / 100;
  float m = ((val / 100.0) - d) * 100.0;
  return std::to_string(d) + "度" + std::to_string(m, 4) + "分";
}
 
// NMEAの緯度経度を「度」(DD)の文字列に変換する
std::string NMEA2DD(float val) {
  int d = val / 100;
  int m = (((val / 100.0) - d) * 100.0) / 60;
  float s = (((((val / 100.0) - d) * 100.0) - m) * 60) / (60 * 60);
  return std::to_string(d + m + s, 6);
}
 
// UTC時刻から日本の標準時刻に変換する(GMT+9:00)
std::string UTC2GMT900(std::string str) {
  int hh = (str.substring(0,2).toInt()) + 9;
  if(hh > 24) hh = hh - 24;
 
  return std::to_string(hh,DEC) + ":" + str.substring(2,4) + ":" + str.substring(4,6);  
}
#endif

//------------------------
// gysfdmaxb ROS Node
//------------------------   
GysfdmaxbNode::GysfdmaxbNode(){
  initialize();
}

void GysfdmaxbNode::initialize() {
  // Params must be set before initializing IO
  getRosParams();

  // set up Ros Publish part
  fix_.status.status  = sensor_msgs::NavSatStatus::STATUS_FIX;
  fix_.status.service = 0;

  fix_.header.frame_id = frame_id_;
  fix_publisher_ = nh->advertise<sensor_msgs::NavSatFix>(fix_topic_, 10);


  initializeIo();

  // subscribe GPS raw data
  //gps.subscribe_gps(GysfdmaxbNode::subscribe_gps);


  //gps.fncStr_=publish_void_str;
  //gps.fncStr_=boost::bind(&GysfdmaxbNode::publish_void_str,_1);
  //gps.subscribe_str(boost::bind(&GysfdmaxbNode::publish_void_str,_1));


  //gps.subscribe_char(boost::bind(&GysfdmaxbNode::publish_void_char,this,_1));


  // subscribe GPS nmea data and Publish
  gps.subscribe_str(boost::bind(&GysfdmaxbNode::publish_nmea_str,this,_1));

}

void GysfdmaxbNode::initializeIo(){
  gps.setConfigOnStartup(config_on_startup_flag_);

  gps.initializeSerial(device_, baudrate_, uart_in_, uart_out_);

}

void GysfdmaxbNode::getRosParams() {
  nh->param("device", device_, std::string("/dev/ttyUSB0"));
  nh->param("frame_id", frame_id_, std::string("gps_link"));

  nh->param("topicName", fix_topic_, std::string("fix"));


  set_usb_ = false;

  // Measurement rate params
  nh->param("rate", rate_, 4.0);  // in Hz


  //if (enable_ppp_)
  //  ROS_WARN("Warning: PPP is enabled - this is an expert setting.");


  // measurement period [ms]
  //meas_rate = 1000 / rate_;

  // activate/deactivate any config
  //nh->param("config_on_startup", config_on_startup_flag_, true);

  // raw data stream logging 
  //rawDataStreamPa_.getRosParams();
}

//------------------------------------
// subscribe GPS nmea data and Publish
//------------------------------------
void GysfdmaxbNode::publish_nmea_str(std::string& data) {
  //static ros::Publisher publisher = nh->advertise<nmea_msgs::Sentence>(topic,
  //                                                          kROSQueueSize);
  //nmea_msgs::Sentence m;
  //m.header.stamp = ros::Time::now();
  //m.header.frame_id = frame_id;
  //m.sentence = sentence;
  //publisher.publish(m);
  
  //std::cout << ">>> called publish_nmea_str() \n" << data << std::endl;
  //std::cout <<  data << std::endl;

  if(data != ""){
    int i, index = 0, len = data.length();
    std::string str = "";
  
    // StringListの生成(簡易)
    std::string list[30];
    for (i = 0; i < 30; i++) {
      list[i] = "";
    }
 
    // 「,」を区切り文字として文字列を配列にする
    for (i = 0; i < len; i++) {
      if (data[i] == ',') {
        list[index++] = str;
        str = "";
        continue;
      }
      str += data[i];
    }
    
    // $GPGGAセンテンスのみ読み込む
    if (list[0] == "$GPGGA") {
      
      // ステータス
      if(list[6] != "0"){      

        #ifdef USE_BUNKAI

        // 現在時刻
        //Serial.print(UTC2GMT900(list[1]));
        std::cout << UTC2GMT900(list[1]);
        
        // 緯度:latitude
        std::cout <<" 緯度:";

        std::cout << NMEA2DMS(std::stof(list[2]));
        std::cout << "(";
        std::cout << NMEA2DD(std::stof(list[2]));
        std::cout <<")";
 
        // 経度:longitude
        std::cout <<" 経度:";
        std::cout << NMEA2DMS(std::stof(list[4]));
        std::cout << "(";
        std::cout << NMEA2DD(std::stof(list[4]));
        std::cout <<")";
 
        // 海抜:altitude
        std::cout <<" 海抜:";
        std::cout << list[9]; 
        //list[10].toLowerCase();
        std::cout <<list[10] << std::endl;
        #else
        //std::cout <<  data << std::endl;

        #endif


        fix_.header.stamp = ros::Time::now();
        fix_.latitude  = std::stof(list[2])/100.0;  // 緯度
        fix_.longitude = std::stof(list[4])/100.0;  // 経度
        fix_.altitude  = std::stof(list[9]);  // 高度、海抜


        //float64[9] position_covariance
        //# If the covariance of the fix is known, fill it in completely. If the
        //# GPS receiver provides the variance of each measurement, put them
        //# along the diagonal. If only Dilution of Precision is available,
        //# estimate an approximate covariance from that.

        //uint8 COVARIANCE_TYPE_UNKNOWN = 0
        //uint8 COVARIANCE_TYPE_APPROXIMATED = 1
        //uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
        //uint8 COVARIANCE_TYPE_KNOWN = 3
        //uint8 position_covariance_type

        fix_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        //fix_.position_covariance[0] = position_error_model_.drift.X()*position_error_model_.drift.X() + position_error_model_.gaussian_noise.X()*position_error_model_.gaussian_noise.X();
        //fix_.position_covariance[4] = position_error_model_.drift.Y()*position_error_model_.drift.Y() + position_error_model_.gaussian_noise.Y()*position_error_model_.gaussian_noise.Y();
        //fix_.position_covariance[8] = position_error_model_.drift.Z()*position_error_model_.drift.Z() + position_error_model_.gaussian_noise.Z()*position_error_model_.gaussian_noise.Z();

        fix_publisher_.publish(fix_);

      }
      else{
        //Serial.print("測位できませんでした。");
      }
      
      //Serial.println("");
    }
  }


}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gysfdmaxb_gps");
  nh.reset(new ros::NodeHandle("~"));
  //ros::Subscriber subRtcm = nh->subscribe("/rtcm", 10, rtcmCallback);
  //nh->param("debug", ublox_gps::debug, 1);
  //if(ublox_gps::debug) {
  //  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  //                                     ros::console::levels::Debug))
  //   ros::console::notifyLoggerLevelsChanged();
  //
  //}
  GysfdmaxbNode node;

  ros::spinOnce();

  ros::Rate rate(1);   //  1[Hz]

  //int i=0;

  //while(1){
  //  gps.read_line();
  //  ros::spinOnce();
  //  i++;
  //
  //  if(i > 50)
  //    break;
  //  rate.sleep();
  //}

  ros::spin();

  return 0;
}

