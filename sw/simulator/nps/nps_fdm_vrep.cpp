#include "nps_fdm.h"

#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/asio.hpp>


using std::endl;
using std::ostream;
using boost::filesystem::ofstream;
using boost::filesystem::current_path;
using boost::asio::ip::tcp;

struct NpsFdm fdm;
struct LtpDef_d ltpRef;
boost::asio::io_service io_service;

ofstream vrepLog(current_path()/"vrep.log");

void nps_fdm_init(double dt) {

  fdm.on_ground=1;
  fdm.ecef_pos.x=0.0;
  fdm.ecef_pos.y=0.0;
  fdm.ecef_pos.z=0.0;
  fdm.ecef_ecef_vel.x=0.0f;
  fdm.ecef_ecef_vel.y=0.0f;
  fdm.ecef_ecef_vel.z=0.0f;
  fdm.ltp_ecef_vel.x=0.0f;
  fdm.ltp_ecef_vel.y=0.0f;
  fdm.ltp_ecef_vel.z=0.0f;
  fdm.ecef_ecef_accel.x=0.0f;
  fdm.ecef_ecef_accel.y=0.0f;
  fdm.ecef_ecef_accel.z=0.0f;
  ltpRef.ecef.x=0.0;
  ltpRef.ecef.y=0.0;
  ltpRef.ecef.z=0.0;
  ltpRef.lla.lat=0.0;
  ltpRef.lla.lon=0.0;
  ltpRef.lla.alt=0.0;
  ltp_def_from_ecef_d(&ltpRef, &ltpRef.ecef);
  /*
  ltpRef.ltp_of_ecef[0]=1.0;
  ltpRef.ltp_of_ecef[1]=0.0;
  ltpRef.ltp_of_ecef[2]=0.0;
  ltpRef.ltp_of_ecef[3]=0.0;
  ltpRef.ltp_of_ecef[4]=1.0;
  ltpRef.ltp_of_ecef[5]=0.0;
  ltpRef.ltp_of_ecef[6]=0.0;
  ltpRef.ltp_of_ecef[7]=0.0;
  ltpRef.ltp_of_ecef[8]=1.0;
  */
  fdm.time=0;
  fdm.init_dt=dt;
  vrepLog << "[" << fdm.time << "] vrep fdm init: dt=" << dt << endl;
}

void nps_fdm_run_step(bool_t launch, double *commands, int commands_nb) {
  fdm.time+=fdm.init_dt;
  fdm.ltp_ecef_vel;
  vrepLog << "[" << fdm.time << "] vrep fdm step: launch=" << (launch?"yes":"no") << " commands=[";
  for(int i=0;i<commands_nb;i++)
    vrepLog << commands[i] << ((i==commands_nb-1)?"":", ");
  vrepLog << "]" << endl;



  //Server communication
  try
  {

    tcp::socket s(io_service);
    tcp::resolver resolver(io_service);
    boost::asio::connect(s, resolver.resolve({"localhost", "50013"}));

    char request[1024] = "grabbing data";
    size_t request_length = std::strlen(request);
    std::cout << "Query is: " << request << std::endl;
    boost::asio::write(s, boost::asio::buffer(request, request_length));

    char reply[1024];
    size_t reply_length = boost::asio::read(s, boost::asio::buffer(reply, request_length));
    std::cout << "Reply is: ";
    std::cout.write(reply, reply_length);
    std::cout << "\n";
    std::cout << "\n";

    char request2[1024] = "sending data";
    request_length = std::strlen(request);
    std::cout << "Query is: " << request2 << std::endl;
    boost::asio::write(s, boost::asio::buffer(request2, request_length));
    reply_length = boost::asio::read(s, boost::asio::buffer(reply, request_length));
    std::cout << "Reply is: ";
    std::cout.write(reply, reply_length);
    std::cout << "\n";

  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }


}

void nps_fdm_set_wind(double speed, double dir) {
  vrepLog << "[" << fdm.time << "] vrep fdm set wind: speed=" << speed << " dir=" << dir << endl;
}

void nps_fdm_set_wind_ned(double wind_north, double wind_east, double wind_down) {
  vrepLog << "[" << fdm.time << "] vrep fdm init ned wind_north=" << wind_north << " wind_east=" <<  wind_east << " wind_down=" <<  wind_down << endl;
}

void nps_fdm_set_turbulence(double wind_speed, int turbulence_severity) {
  vrepLog << "[" << fdm.time << "] vrep fdm set turbulance: wind_speed=" << wind_speed << " severity=" << turbulence_severity << endl;
}
