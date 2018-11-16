/***************************************************************************
  tag: Bert Willaert  Fri Sept 21 09:31:20 CET 2012   soem_robotiq_3Finger.h

                        soem_robotiq_3Finger.h -  description
                           -------------------
    begin                : September 21 2012
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


#ifndef SOEM_Robotiq_3Finger_H
#define SOEM_Robotiq_3Finger_H

#include <soem_master/soem_driver.h>
#include <soem_robotiq_drivers/RobotiqMsg.h>
#include <rtt/Port.hpp>
#include <bitset>

#define MESSAGE_SIZE 8

#define BYTE0_ACT (0x01 )					// 0b00000001
#define BYTE0_MOD (0x02 | 0x04 )			// 0b00000110
#define BYTE0_GTO (0x08 )					// 0b00001000
#define BYTE0_IMC (0x10 | 0x20 )			// 0b00110000
#define BYTE0_STA (0x40 | 0x80 )			// 0b11000000

namespace soem_robotiq_drivers{

  class SoemRobotiq : public soem_master::SoemDriver
  {
    /////////////////////////////////////////////////////////// 
	 // Note that the scissor dof is also considered a finger //
    typedef struct PACKED
    {
		uint8 FingerPosRequestEcho;
	   uint8 FingerPosition;
		uint8 FingerCurrent;
    } in_Robotiqt_Finger; 


    typedef struct PACKED
    {
      uint8 GripperStatus;
		uint8 ObjectDetection;
		uint8 FaultStatus;
		in_Robotiqt_Finger Fingers[4];
		uint8 Reserved; 
    } in_Robotiqt; 

    /////////////////////////////////////////////////////////// 
	 // Note that the scissor dof is also considered a finger //
    typedef struct PACKED
    {
		uint8 PositionFinger;
	   uint8 SpeedFinger;
		uint8 ForceFinger;
    } out_Robotiqt_Finger;

    typedef struct PACKED
    {
      uint8 ActionRequest;
		uint8 GripperOptions;
		uint8 GripperOptions2;
	   out_Robotiqt_Finger Fingers[4];
		uint8 Reserved; 
    } out_Robotiqt;
    
  public:
    SoemRobotiq(ec_slavet* mem_loc);
    ~SoemRobotiq(){};
   
   bool configure(); 
 	void update();
    	 
	 bool Activate(bool act);
	 bool Start(bool gto);


	 bool SetGripperPos(double st);
	 bool SetMotorPos(double mt1, double mt2, double mt3, double mt4);
	 //bool SetScissorPos(double st);
	 bool SetVelocity(double st);
	 bool SetForce(double st);
	 bool SetSinus(double ampl, double freq);

	 int LSB_NonZeroBit(int value);

	 //function to set working mode
	 bool SetOptions(bool valueL, bool valueR);
	 bool SetMode(bool valueL, bool valueR);
     //new functions to help set correctly the system
	 bool SetModeBasic();
	 bool SetModePinch();
	 bool SetModeWide();
	 bool SetModeScissor();
	 bool SetModeIndipendent();

  private:
	 int message_size;

    RobotiqMsg status_msg;
    std::bitset<MESSAGE_SIZE> status_bits;
	RTT::OutputPort<RobotiqMsg> status_port;

    std::bitset<MESSAGE_SIZE> input_bits;
    std::bitset<MESSAGE_SIZE> fault_bits;
    std::bitset<MESSAGE_SIZE> action_bits;
	std::bitset<MESSAGE_SIZE> option_bits;

	std::vector<double> stPos;
	std::vector<double> stVel;
    std::vector<double> stForce;

	 double ts_;
	 double time_;
	 double ampl_;
	 double freq_;
	 enum {OPERATION_DRIVEN,PORT_BASIC, SINUS,PORT_ADVANCED} mode;
	 //bool act_sinus_;

};
 
}
#endif
