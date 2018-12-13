/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once


#include "ctre/Phoenix.h"

#include <IterativeRobot.h>
#include "WPILib.h"

class Robot : public frc::IterativeRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void DisabledInit() override;

  void initPID();

 private:
  	WPI_TalonSRX *l_master;
	WPI_TalonSRX *r_master;

	WPI_VictorSPX *l_slave1;
	WPI_VictorSPX *l_slave2;
	WPI_VictorSPX *l_slave3;

	WPI_VictorSPX *r_slave1;
	WPI_VictorSPX *r_slave2;
	WPI_VictorSPX *r_slave3;

	WPI_VictorSPX *l_intake;
	WPI_VictorSPX *r_intake;

	WPI_TalonSRX *lift_master;

    Joystick *joystick_l;
	Joystick *joystick_r;
    // auxiliary joystick for pid testing 
	Joystick *joystick_aux;

    Compressor *compressor;

    Solenoid *shifter;
    // TODO: implement this on the right side and fix the stuff
    DifferentialDrive *drive;

    // this will be used to calculate the error
    void velocityClosedLoop();

    	struct pid_values {

        const float RATIO = 168.04;
		// speed in ticks/100ms
        float max_speed = 1065;
		double rpm = 461;
		int ticksPerRevolution = 128;

        float l_scale = 1.0;
        float r_scale = 1.0;

		// ticks / 100 ms / second
		float accel = this->max_speed * 1.0;
		// maximum ticks / 100 ms
		float vel = this->max_speed * 1.0;
		// % * constant / error
        // (1.00 * 128) / 250 = 0.512
		const float Kp = 1.202;
		const float Ki = 0;
        const float Kz = 0;
		const float Kd = this->Kp * 10;
		// 
		const float Kf = (0.000585 * this->max_speed); 
		const float timeout = 0.005;
	} pid;

    // struct pid_values_low {
    //     // speed in ticks/100ms
    //     float max_speed = 1065;
	// 	double rpm = 461;
	// 	int ticksPerRevolution = 128;

    //     float l_scale = 1.0;
    //     float r_scale = 1.0;

	// 	// ticks / 100 ms / second
	// 	float accel = this->max_speed * 1;
	// 	// maximum ticks / 100 ms
	// 	float vel = this->max_speed * 1;
	// 	// % * constant / error
    //     // (1.00 * 128) / 250 = 0.512
	// 	const float Kp = (1.33 * 0.512);
	// 	const float Ki = 0.001;
    //     const float Kiz = 50;
	// 	const float Kd = 40 * this->Kp;
	// 	// 
	// 	const float Kf = (0.00001 * this->max_speed); 
	// 	const float timeout = 0.005;
    // };
    //The 1023 are the native units the SRX would get if set to 100% output
};
