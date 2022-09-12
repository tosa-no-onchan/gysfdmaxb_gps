#include "gysfdmaxb_gps/gps.h"
#include <boost/version.hpp>


namespace gysfdmaxb_gps {

//using namespace gysfdmaxb_gps;

//! Sleep time [ms] after setting the baudrate
constexpr static int kSetBaudrateSleepMs = 500;

const boost::posix_time::time_duration Gps::default_timeout_ =
    boost::posix_time::milliseconds(
        static_cast<int>(Gps::kDefaultAckTimeout * 1000));


//Gps::Gps() : configured_(false), config_on_startup_flag_(true) {
Gps::Gps(){
 //subscribeAcks();
}

Gps::~Gps() {
    ok = false;
    #ifdef USE_READ_THREAD
      read_thread.join();
    #endif
    serial_->close();
   //close(); 
}

#ifdef USE_WORKER
void Gps::setWorker(const boost::shared_ptr<Worker>& worker) {
  if (worker_) return;
  worker_ = worker;
  worker_->setCallback(boost::bind(&CallbackHandlers::readCallback,
                                   &callbacks_, _1, _2));
  configured_ = static_cast<bool>(worker);
}
#endif

// https://tips.hecomi.com/entry/20120728/1343504831
// https://blog.myon.info/entry/2015/04/19/boost-asio-serial/
void Gps::initializeSerial(std::string port, unsigned int baudrate,
                           uint16_t uart_in, uint16_t uart_out) {
  port_ = port;

  boost::shared_ptr<boost::asio::io_service> io_service(
      new boost::asio::io_service);

  boost::shared_ptr<boost::asio::serial_port> serial(
      new boost::asio::serial_port(*io_service));

  // open serial port
  try {
    serial->open(port);
  } 
  catch (std::runtime_error& e) {
    throw std::runtime_error("Gysfdmaxb: Could not open serial port :"
                             + port + " " + e.what());
  }

  ROS_INFO("Gysfdmaxb: Opened serial port %s", port.c_str());
    
  if(BOOST_VERSION < 106600)
  {
    // NOTE(Kartik): Set serial port to "raw" mode. This is done in Boost but
    // until v1.66.0 there was a bug which didn't enable the relevant code,
    // fixed by commit: https://github.com/boostorg/asio/commit/619cea4356
    int fd = serial->native_handle();
    termios tio;
    tcgetattr(fd, &tio);
    cfmakeraw(&tio);
    tcsetattr(fd, TCSANOW, &tio);
  }

  //#define SET_CONFIG
  #ifdef SET_CONFIG
  // Set the I/O worker
  //if (worker_) return;
  //setWorker(boost::shared_ptr<Worker>(
  //    new AsyncWorker<boost::asio::serial_port>(serial, io_service)));

  configured_ = false;

  // Set the baudrate
  boost::asio::serial_port_base::baud_rate current_baudrate;
  serial->get_option(current_baudrate);
  // Incrementally increase the baudrate to the desired value
  for (int i = 0; i < sizeof(kBaudrates)/sizeof(kBaudrates[0]); i++) {
    if (current_baudrate.value() == baudrate)
      break;
    // Don't step down, unless the desired baudrate is lower
    if(current_baudrate.value() > kBaudrates[i] && baudrate > kBaudrates[i])
      continue;
    serial->set_option(
        boost::asio::serial_port_base::baud_rate(kBaudrates[i]));
    boost::this_thread::sleep(
        boost::posix_time::milliseconds(kSetBaudrateSleepMs));
    serial->get_option(current_baudrate);
    //ROS_DEBUG("Gysfdmaxb: Set ASIO baudrate to %u", current_baudrate.value());
    ROS_INFO("Gysfdmaxb: Set ASIO baudrate to %u", current_baudrate.value());
  }
  //if (config_on_startup_flag_) {
  //  configured_ = configUart1(baudrate, uart_in, uart_out);
  //  if(!configured_ || current_baudrate.value() != baudrate) {
  //    throw std::runtime_error("Could not configure serial baud rate");
  //  }
  //} 
  //else {
  //  configured_ = true;
  //}
  #endif

  io_service_ =  io_service;
  serial_=serial;

  #ifdef USE_READ_THREAD
    /* and turn on the streamer */
    ok = true;
    read_thread = boost::thread(boost::bind(&Gps::read_gps, this));
  #else
    // exec async_read_some();
    io_service_->post(boost::bind(&Gps::async_read_some, this));
    background_thread_.reset(new boost::thread(
        boost::bind(&boost::asio::io_service::run, io_service_)));

  #endif

}

void Gps::resetSerial(std::string port) {
  boost::shared_ptr<boost::asio::io_service> io_service(
      new boost::asio::io_service);
  boost::shared_ptr<boost::asio::serial_port> serial(
      new boost::asio::serial_port(*io_service));

  // open serial port
  try {
    serial->open(port);
  } 
  catch (std::runtime_error& e) {
    throw std::runtime_error("Gysfdmaxb: Could not open serial port :"
                             + port + " " + e.what());
  }

  ROS_INFO("Gysfdmaxb: Reset serial port %s", port.c_str());

  #ifdef USE_WORKER_X
  // Set the I/O worker
  if (worker_) return;
  setWorker(boost::shared_ptr<Worker>(
      new AsyncWorker<boost::asio::serial_port>(serial, io_service)));
  configured_ = false;

  // Poll UART PRT Config
  std::vector<uint8_t> payload;
  payload.push_back(CfgPRT::PORT_ID_UART1);
  if (!poll(CfgPRT::CLASS_ID, CfgPRT::MESSAGE_ID, payload)) {
    ROS_ERROR("Resetting Serial Port: Could not poll UART1 CfgPRT");
    return;
  }
  CfgPRT prt;
  if(!read(prt, default_timeout_)) {
    ROS_ERROR("Resetting Serial Port: Could not read polled UART1 CfgPRT %s",
                "message");
    return;
  }

  // Set the baudrate
  serial->set_option(boost::asio::serial_port_base::baud_rate(prt.baudRate));
  configured_ = true;
  #endif
}

void Gps::read_gps(){
  // 受信データ
	boost::array<unsigned char, 512> receive_api_frame;

  while(ok){
    //nmea_str.clear();
    serial_->read_some( boost::asio::buffer(receive_api_frame) );
    // 受信結果書き出し
    //for (size_t i = 0; i < receive_api_frame.size() - 1; ++i) {
    for (size_t i = 0; i < receive_api_frame.size(); ++i) {
      //std::cout << std::hex << (unsigned int)receive_api_frame[i] << " ";
      std::cout <<  receive_api_frame[i];
      nmea_data[i] =receive_api_frame[i];
      nmea_data[i+1] = 0x00;
      //nmea_str+=receive_api_frame[i];
    }
    //std::cout << std::endl;
    //fsub_(nmea_data);
    //fncChar_(nmea_data);
    //fncStr_(nmea_str);
    //break;
  }
}

void Gps::async_read_some()
{
	if (serial_.get() == NULL || !serial_->is_open()) return;
	serial_->async_read_some( 
		boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
		boost::bind(
			&Gps::on_receive,this,
      //_1,_2));
			boost::asio::placeholders::error, 
			boost::asio::placeholders::bytes_transferred));
}

void Gps::on_receive(const boost::system::error_code& ec, size_t bytes_transferred)
{
	boost::mutex::scoped_lock look(mutex_);

	if (serial_.get() == NULL || !serial_->is_open()) return;
	if (ec) {
		async_read_some();
		return;
	}

	for (unsigned int i = 0; i < bytes_transferred; ++i) {
		char c = read_buf_raw_[i];
		if (c == end_of_line_char_) {
			this->on_receive_all(nmea_str);
			nmea_str.clear();
		}
		else {
			nmea_str += c;
		}
	}
  //async_read_some();
  io_service_->post(boost::bind(&Gps::async_read_some, this));
}

void Gps::on_receive_all(std::string &data)
{
	//std::cout << data << std::endl;
  if(call_back_f==true){
    fncStr_(data);
  }
}
}