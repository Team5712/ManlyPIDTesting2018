/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <iostream>

using namespace std;


void Robot::RobotInit() {

    compressor = new Compressor(0);

    shifter = new Solenoid(2);

    l_master = new WPI_TalonSRX(1);
    r_master = new WPI_TalonSRX(5);

    l_slave1 = new WPI_VictorSPX(2);
    l_slave2 = new WPI_VictorSPX(3);
    l_slave3 = new WPI_VictorSPX(4);

    r_slave1 = new WPI_VictorSPX(6);
    r_slave2 = new WPI_VictorSPX(7);
    r_slave3 = new WPI_VictorSPX(8);

    l_intake = new WPI_VictorSPX(9);
    r_intake = new WPI_VictorSPX(10); // needs to be 10

    lift_master = new WPI_TalonSRX(11);

    l_slave1->Follow(*l_master);
	l_slave2->Follow(*l_master);
	l_slave3->Follow(*l_master);

	r_slave1->Follow(*r_master);
	r_slave2->Follow(*r_master);
	r_slave3->Follow(*r_master);

    drive = new DifferentialDrive(*l_master, *r_master);

    joystick_l = new Joystick(0);
	joystick_r = new Joystick(1);
	joystick_aux = new Joystick(2);

    // that thang noisy :(
    compressor->Start();
    shifter->Set(true); 
    drive->SetSafetyEnabled(false);

    l_master->SetSelectedSensorPosition(0, 0, 10);
    r_master->SetSelectedSensorPosition(0, 0, 10);

    l_master->SetSensorPhase(true);

    initPID();   
}


void Robot::RobotPeriodic() {


    // cout << "left: " << l_master->GetSelectedSensorVelocity(0) << endl;
    // cout << "right: " << r_master->GetSelectedSensorVelocity(0) << endl;
}

void Robot::AutonomousInit() {
    cout << "starting auto" << endl;
}

void Robot::AutonomousPeriodic() {
    cout << "left value: " << l_master->GetSelectedSensorPosition(0) << " err " << l_master->GetClosedLoopError(0) << endl;
    // cout << "right value: " << r_master->GetSelectedSensorPosition(0) << " err " << r_master->GetClosedLoopError(0) << endl;
    l_master->Set(ControlMode::MotionMagic, pid.RATIO * (20 * 12));
    r_master->Set(ControlMode::MotionMagic, pid.RATIO * (20 * 12));
}


void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {

    cout << "starting  teleop" << endl;

    drive->TankDrive(-joystick_l->GetRawAxis(1), joystick_r->GetRawAxis(1), false);

    // cout << "l " << l_master->GetSelectedSensorVelocity(0) << endl;
    // cout << "r " << r_master->GetSelectedSensorVelocity(0) << endl;


    // if(joystick_aux->GetRawAxis(1)) {
    //     // set position to input * ticks per wheel for 10 revolutions
    //     double targetPos = joystick_aux->GetRawAxis(1) * pid.ticksPerRevolution * 20.0;
    //     cout << "target " << targetPos << " value: " << l_master->GetSelectedSensorPosition(0) << " err " << l_master->GetClosedLoopError(0) << endl;
    //     l_master->Set(ControlMode::MotionMagic, targetPos);
    // }

    // if (joystick_aux->GetRawButton(1) || joystick_aux->GetRawButton(1)) {
	// 	shifter->Set(false);    
	// } else {
    //     shifter->Set(true);
    // }

    // drive->TankDrive(joystick_l->GetRawAxis(1), joystick_r->GetRawAxis(1), false);
    // cout << "l: " << l_master->GetSelectedSensorPosition(0) << endl;
    // cout << "r: " <<  r_master->GetSelectedSensorPosition(0) << endl;

}

void Robot::DisabledInit() {
    cout << "I'm disabled :D" << endl;
}

void Robot::TestPeriodic() {
    
}

/**
 * Initialize the pid settings
 * 
 */  
void Robot::initPID() {

    l_master->SetSensorPhase(true);

	l_master->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,
			pid.timeout);
	r_master->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,
			pid.timeout);

	// r_master->ConfigSensorTerm(SensorTerm::SensorTerm_Diff0,
	// 		FeedbackDevice::RemoteSensor0, pid.timeout);
	// r_master->ConfigSensorTerm(SensorTerm::SensorTerm_Diff1,
	// 		FeedbackDevice::QuadEncoder, pid.timeout);

	// r_master->ConfigSensorTerm(SensorTerm::SensorTerm_Sum0,
	// 		FeedbackDevice::RemoteSensor0, pid.timeout);
	// r_master->ConfigSensorTerm(SensorTerm::SensorTerm_Sum1,
	// 		FeedbackDevice::QuadEncoder, pid.timeout);

	// // 0 is for primary, not aux
	// r_master->ConfigSelectedFeedbackSensor(FeedbackDevice::SensorSum, 0, pid.timeout);

	// r_master->SetStatusFramePeriod(StatusFrame::Status_12_Feedback1_, 20, pid.timeout);
	// r_master->SetStatusFramePeriod(StatusFrame::Status_13_Base_PIDF0_, 20, pid.timeout);
	// r_master->SetStatusFramePeriod(StatusFrame::Status_14_Turn_PIDF1_, 20, pid.timeout);
	// r_master->SetStatusFramePeriod(StatusFrame::Status_10_MotionMagic_, 20, pid.timeout);
	// l_master->SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 5, pid.timeout);

	l_master->SelectProfileSlot(0, 0);
	l_master->Config_kP(0, pid.Kp, pid.timeout);
	l_master->Config_kI(0, pid.Ki, pid.timeout);
	l_master->Config_kD(0, pid.Kd, pid.timeout);
	l_master->Config_kF(0, pid.Kf, pid.timeout);
    l_master->Config_IntegralZone(0, pid.Kz, pid.timeout);

	r_master->SelectProfileSlot(0, 0);
	r_master->Config_kP(0, pid.Kp, pid.timeout);
	r_master->Config_kI(0, pid.Ki, pid.timeout);
	r_master->Config_kD(0, pid.Kd, pid.timeout);
	r_master->Config_kF(0, pid.Kf, pid.timeout);
    r_master->Config_IntegralZone(0, pid.Kz, pid.timeout);

    
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Note: these values are are measured to be the fastest speed our robot can attain units/100ms
    // These may be scaled down so that they are more consistant with one another
    //..............................................................................................
	l_master->ConfigMotionAcceleration(pid.accel, pid.timeout);
	l_master->ConfigMotionCruiseVelocity(pid.vel, pid.timeout);

	r_master->ConfigMotionAcceleration(pid.accel, pid.timeout);
	r_master->ConfigMotionCruiseVelocity(pid.vel, pid.timeout);
}

#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif
