#include "nps_fdm.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

using std::endl;
using std::ostream;
using boost::filesystem::ofstream;
using boost::filesystem::current_path;

struct NpsFdm fdm;
struct LtpDef_d ltpRef;

ofstream vrepLog(current_path()/"vrep.log");

void nps_fdm_init(double dt) {
  fdm.on_ground=1;
  fdm.ecef_pos.x=0.0;
  fdm.ecef_pos.y=0.0;
  fdm.ecef_pos.z=0.0;
  fdm.ecef_ecef_vel.x=0.0f;
  fdm.ecef_ecef_vel.y=0.0f;
  fdm.ecef_ecef_vel.z=0.0f;
  fdm.ecef_ecef_accel.x=0.0f;
  fdm.ecef_ecef_accel.y=0.0f;
  fdm.ecef_ecef_accel.z=0.0f;
  ltpRef.ecef.x=0.0;
  ltpRef.ecef.y=0.0;
  ltpRef.ecef.z=0.0;
  ltpRef.lla.lat=0.0;
  ltpRef.lla.lon=0.0;
  ltpRef.lla.alt=0.0;
  ltpRef.ltp_of_ecef[0]=1.0;
  ltpRef.ltp_of_ecef[1]=0.0;
  ltpRef.ltp_of_ecef[2]=0.0;
  ltpRef.ltp_of_ecef[3]=0.0;
  ltpRef.ltp_of_ecef[4]=1.0;
  ltpRef.ltp_of_ecef[5]=0.0;
  ltpRef.ltp_of_ecef[6]=0.0;
  ltpRef.ltp_of_ecef[7]=0.0;
  ltpRef.ltp_of_ecef[8]=1.0;
  fdm.time=0;
  fdm.init_dt=dt;
  vrepLog << "[" << fdm.time << "] vrep fdm init: dt=" << dt << endl;
}

void nps_fdm_run_step(bool_t launch, double *commands, int commands_nb) {
  fdm.time+=fdm.init_dt;
  ltp_ecef_vel
  vrepLog << "[" << fdm.time << "] vrep fdm step: launch=" << (launch?"yes":"no") << " commands=[";
  for(int i=0;i<commands_nb;i++)
    vrepLog << commands[i] << ((i==commands_nb-1)?"":", ");
  vrepLog << "]" << endl;
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
