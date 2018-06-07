/***************************************************************************
 tag: Bert Willaert  Fri Sept 21 09:31:20 CET 2012  soem_robotiq_3Finger.cpp

 soem_robotiq_3Finger.cpp -  description
 -------------------
 begin                : Fri September 21 2012
 copyright            : (C) 2012 Bert Willaert
 email                : first.last@mech.kuleuven.be

 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include "soem_robotiq_3Finger.h"
#include <soem_master/soem_driver_factory.h>

using namespace RTT;

namespace soem_robotiq_drivers
{

SoemRobotiq::SoemRobotiq(ec_slavet* mem_loc) :
    soem_master::SoemDriver(mem_loc), status_port("Status"),
    stPos(4, 0.0), stVel(4, 0.0), stForce(4, 0.0),
	 ts_(1),time_(0), ampl_(0), freq_(0), mode(OPERATION_DRIVEN)
{
    m_service->doc(std::string("Services for Robotiq Hand ") + std::string(
            m_datap->name) );
    m_service->addOperation("Activate", &SoemRobotiq::Activate, this, RTT::OwnThread).doc(
            "Activate or de-activate - starts the calibration procedure");
    m_service->addOperation("Start", &SoemRobotiq::Start, this, RTT::OwnThread).doc(
            "Start or stop all possible motions");
    m_service->addOperation("SetMode", &SoemRobotiq::SetMode, this, RTT::OwnThread).doc(
            "Select the mode to go two").arg("rMOD1","rMOD2");
    m_service->addOperation("SetOptions", &SoemRobotiq::SetOptions, this, RTT::OwnThread).doc(
            "Select the options for individual control").arg("rICF","rICS");
    m_service->addOperation("SetGripperPos", &SoemRobotiq::SetGripperPos, this, RTT::OwnThread).doc(
            "Command position for all three Fingers");
    m_service->addOperation("SetMotorPos", &SoemRobotiq::SetMotorPos, this, RTT::OwnThread).doc(
            "Command position for individual Fingers");
    //m_service->addOperation("SetScissorPos", &SoemRobotiq::SetScissorPos, this, RTT::OwnThread).doc(
    //        "Command position for Scissors");
    m_service->addOperation("SetFingerVel", &SoemRobotiq::SetVelocity, this, RTT::OwnThread).doc(
            "Set velocity for all fingers");
    m_service->addOperation("SetFingerForce", &SoemRobotiq::SetForce, this, RTT::OwnThread).doc(
            "Set maximum force for all fingers");
    m_service->addOperation("SetSinus", &SoemRobotiq::SetSinus, this, RTT::OwnThread).doc(
            "Set parameters for sinusoidal motion");

    //ADD setmode operations
    m_service->addOperation("SetModeBasic", &SoemRobotiq::SetModeBasic, this, RTT::OwnThread).doc(
              "Set basic gripper mode.");
    m_service->addOperation("SetModePinch", &SoemRobotiq::SetModePinch, this, RTT::OwnThread).doc(
              "Set pinch gripper mode.");
    m_service->addOperation("SetModeWide", &SoemRobotiq::SetModeWide, this, RTT::OwnThread).doc(
              "Set wide gripper mode.");
    m_service->addOperation("SetModeScissor", &SoemRobotiq::SetModeScissor, this, RTT::OwnThread).doc(
              "Set scissor gripper mode.");
    m_service->addOperation("SetModeIndipendent", &SoemRobotiq::SetModeIndipendent, this, RTT::OwnThread).doc(
              "Independent control of the finger.");




    m_service->addPort(status_port).doc("Data port to communicate full bitsets");

}

bool SoemRobotiq::configure()
{	 
	// Get period of the owner of the service, i.e. the soem master
	RTT::TaskContext *upla = m_service->getOwner();
	std::cout << m_name << ": Name of owner: " << upla->getName() << std::endl;
	ts_ = upla->getPeriod();
   std::cout << m_name << ": Period of owner: " << ts_ << std::endl;
	// Set initial configuration parameters
	SetMode(0,0);
	SetOptions(0,0);
	SetVelocity(0.8);
	SetForce(0.5);
	return true;
}



void SoemRobotiq::update()
{


	 // ******************************
	 // *** Read data from gripper ***
	 // ******************************
    input_bits = ((in_Robotiqt*) (m_datap->inputs))->GripperStatus;
	 // gACT	
    status_msg.ACT = (int32) ( (  (input_bits.to_ulong()) & BYTE0_ACT ) >> LSB_NonZeroBit(BYTE0_ACT));
	 // Mode
	 status_msg.MOD = (int32) ( (  (input_bits.to_ulong()) & BYTE0_MOD ) >> LSB_NonZeroBit(BYTE0_MOD));
	 // gGTO
    status_msg.GTO = (int32) ( (  (input_bits.to_ulong()) & BYTE0_GTO ) >> LSB_NonZeroBit(BYTE0_GTO));
	 // IMC
    status_msg.IMC = (int32) ( (  (input_bits.to_ulong()) & BYTE0_IMC ) >> LSB_NonZeroBit(BYTE0_IMC));
	 // STA
    status_msg.STA = (int32) ( (  (input_bits.to_ulong()) & BYTE0_STA ) >> LSB_NonZeroBit(BYTE0_STA));

	 // ******************************
	 // *** Read data from gripper ***
	 // ******************************
    fault_bits = ((in_Robotiqt*) (m_datap->inputs))->FaultStatus;
	 for (int i = 0; i<4; i++)
			status_msg.Fault[i] = fault_bits[i];

	 // *****************************************
	 // *** Read data from individual fingers ***
	 // *****************************************
	 // Note that the scissor dof is also considered a finger //
    input_bits = ((in_Robotiqt*) (m_datap->inputs))->ObjectDetection;
	 for (int i = 0; i<4; i++){
		 // DTi
		 int BYTE1_DTi = ((0x01 | 0x02) << (i*2));
       status_msg.Fingers[i].DTi = (int32) ( (  (input_bits.to_ulong()) & BYTE1_DTi ) >> LSB_NonZeroBit(BYTE1_DTi));
		 // Requested Position
		 status_msg.Fingers[i].ReqPosition = ((in_Robotiqt*) (m_datap->inputs))->Fingers[i].FingerPosRequestEcho;
		 // Actual Position
		 status_msg.Fingers[i].ActPosition = ((in_Robotiqt*) (m_datap->inputs))->Fingers[i].FingerPosition;
		 // Actual Current
		 status_msg.Fingers[i].Current = ((in_Robotiqt*) (m_datap->inputs))->Fingers[i].FingerCurrent;
	 }

	 status_port.write(status_msg);

	 // *******************************
	 // *** Define sinusoidal motion ***
	 // *******************************

	 
	 time_ = time_ + ts_;
	 switch (mode)
	 {
	 case SINUS:
		 stPos[0] = (128 + ((int) (256*ampl_))*sin(2*3.14*freq_*time_))/256;
		 break;
	 //case PORT_BASIC, PORT_ADVANCED:
		 //TODO add port reading here!
	 case OPERATION_DRIVEN:
			// stPos is filled in by calling operations
			 break;
	 default:
		 break;
	 }



	 // *****************************
	 // *** Write data to gripper ***
	 // *****************************

	 ((out_Robotiqt*) (m_datap->outputs))->ActionRequest = action_bits.to_ulong();
	 ((out_Robotiqt*) (m_datap->outputs))->GripperOptions = option_bits.to_ulong();

	 for (int i = 0; i<4; i++){
    	((out_Robotiqt*) (m_datap->outputs))->Fingers[i].PositionFinger = ((int) (256*stPos[i]));
    	((out_Robotiqt*) (m_datap->outputs))->Fingers[i].SpeedFinger = ((int) (256*stVel[i]));
    	((out_Robotiqt*) (m_datap->outputs))->Fingers[i].ForceFinger = ((int) (256*stForce[i]));
	 }

}

bool SoemRobotiq::Activate(bool act)
{
   action_bits.set(0,act);			// rACT 
	return act;

}

bool SoemRobotiq::Start(bool gto)
{
   action_bits.set(3,gto);			// rGTO
	return gto;

}

bool SoemRobotiq::SetSinus(double ampl, double freq)
{
	ampl_ = ampl;
	freq_ = freq;
   time_ = 0;
	mode = SINUS;
	return true;

}


bool SoemRobotiq::SetMode(bool valueL, bool valueR)
{
   action_bits.set(0,true);		// rACT
	action_bits.set(1,valueL);		// rMOD
	action_bits.set(2,valueR);		// rMOD
   action_bits.set(3,true);		// rGTO
	return true;
}
bool SoemRobotiq::SetModeBasic()
{
	return SetOptions(false,false) && SetMode(false,false);
}
bool SoemRobotiq::SetModePinch()
{
	return SetOptions(false,false) && SetMode(true,false);
}
bool SoemRobotiq::SetModeWide()
{
	return SetOptions(false,false) && SetMode(false,true);
}
bool SoemRobotiq::SetModeScissor()
{
	return SetOptions(false,false) && SetMode(true,true);
}
bool SoemRobotiq::SetModeIndipendent()
{
	return SetOptions(true,true);
}


bool SoemRobotiq::SetOptions(bool valueL, bool valueR)
{
   option_bits.set(0,false);		// rGLV
	option_bits.set(1,false);		// rAAC
	option_bits.set(2,valueL);		// rICF
   option_bits.set(3,valueR);		// rICS
	return true;

}


bool SoemRobotiq::SetGripperPos(double st)
{
	if ( st < 0 || st > 1){
 		log(Error) << "Wrong value for finger position" << endlog();
		return false;
	}
   else{
		stPos[0] = st; 
		stPos[1] = st;
		stPos[2] = st;
	}
	return true;
}

bool SoemRobotiq::SetMotorPos(double mt1, double mt2, double mt3, double mt4)
{
	if ( mt1 < 0 || mt1 > 1 || mt2 < 0 || mt2 > 1 || mt3 < 0 || mt3 > 1|| mt4 < 0 || mt4 > 1){
 		log(Error) << "Wrong values for finger position" << endlog();
		return false;
	}
   else{
		stPos[0] = mt1;
		stPos[1] = mt2;
		stPos[2] = mt3;
		stPos[3] = mt4;
	}
	return true;
}

/*bool SoemRobotiq::SetScissorPos(double st)
{
	if ( st < 0 || st >= 1){
 		log(Error) << "Wrong value for scissors position" << endlog();
		return false;
	}
   else{
		stPos[3] = st; 
	}
	return true;
}*/

bool SoemRobotiq::SetVelocity(double st)
{
	if ( st < 0 || st > 1){
 		log(Error) << "Wrong value for finger velocity" << endlog();
		return false;
	}
   else{
		stVel[0] = st; 
		stVel[1] = st;
		stVel[2] = st;
		stVel[3] = st;
	}
	return true;
}

bool SoemRobotiq::SetForce(double st)
{
	if ( st < 0 || st > 1){
 		log(Error) << "Wrong value for maximum finger force" << endlog();
		return false;
	}
   else{
		stForce[0] = st; 
		stForce[1] = st;
		stForce[2] = st;
		stForce[3] = st;
	}
	return true;
}

// This functinon returns the location of the least significant non-zero bit
int SoemRobotiq::LSB_NonZeroBit(int value)
{
	int LSB_NonZeroBit = 0;
	while (!((value >> LSB_NonZeroBit) & 1) ) 
		LSB_NonZeroBit++;
	return LSB_NonZeroBit;
}



namespace
{
soem_master::SoemDriver* createSoemRobotiq(ec_slavet* mem_loc)
{
    return new SoemRobotiq(mem_loc);
}
const bool registered0 =
        soem_master::SoemDriverFactory::Instance().registerDriver("netX",
                createSoemRobotiq);

}
}
