#include "WPILib.h"

class Robot: public SampleRobot
{
	CANTalon *right = new CANTalon(0);
	CANTalon *left = new CANTalon(1);
	//CANTalon *motor1 = new CANTalon(2);
	//CANTalon *motor2 = new CANTalon(3);
	Joystick *stick = new Joystick(0);
	Joystick *buttons = new Joystick(1);
	Joystick *tank = new Joystick(2);
	RobotDrive *myRobot = new RobotDrive(left, right);
	SendableChooser *chooser = new SendableChooser();
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "Stronghold";
	Gyro gyro;
	static const double Kp = 0.03;
	void Autonomous()
	{
		gyro.Reset (0);
		while (IsAutonomous(0))
		{
			GetWatchdog().Feed();
			double angle = gyro.GetAngle(0);
			myRobot.Drive(-1.0, -angle * Kp);
			Wait(0.004);
		}
		//std::string autoSelected = *((std::string*)chooser->GetSelected());
		std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		myRobot->SetSafetyEnabled(false);
		myRobot->SetLeftRightMotorOutputs(0.5, 0.0);
		//Add condition to stop when the ballast comes into view. Need to wait until vision can be done.
		Wait(10.0);
		myRobot->SetLeftRightMotorOutputs(0.0, 0.0);
	}



	void RobotInit()
	{
		myRobot->SetExpiration(0.1);
	}

	void OperatorControl()
	{
		myRobot->SetSafetyEnabled(false);
		//motor1->EnableControl();
		while (IsOperatorControl() and IsEnabled())
		{
			SmartDashboard::PutNumber("Motor Y: ", stick->GetY());
			SmartDashboard::PutNumber("Motor X: ", stick->GetX());
			bool t_drive = false;
			bool a_drive = true;
			if(buttons->GetRawButton(7))
			{
				SmartDashboard::PutString("Drive Mode: ", "Tank Drive");
				t_drive = true;
				a_drive = false;
			}else
			{
				SmartDashboard::PutString("Drive Mode: ", "Arcade Drive");
				t_drive = false;
				a_drive = true;
			}
			if(stick->GetRawButton(5))
			{
				if(stick->GetRawAxis(2) < 0.1)
				{
					myRobot->SetMaxOutput(0.5);
				}else if(stick->GetRawAxis(2) >= -0.1)
				{
					myRobot->SetMaxOutput(1);
				}
				if(t_drive)
				{
					myRobot->TankDrive(-stick->GetRawAxis(1), -stick->GetRawAxis(5), true);
				}else if(a_drive)
				{
					myRobot->ArcadeDrive(-stick->GetRawAxis(1), -stick->GetRawAxis(0), true);
				}
				/*if(buttons->GetRawButton(3))
				{
					motor1->SetPID(1, 3, 4);
					motor1->Set(f);
				}else if(buttons->GetRawButton(4))
				{
					motor1->SetPID(1, 3, 4);
					motor1->Set(-f);
				}*/
			}else
			{
				myRobot->SetLeftRightMotorOutputs(0.0, 0.0);
			}
			Wait(0.005);
		}
	}

	void Test()
	{
	}

	void Disabled()
	{
		myRobot->SetLeftRightMotorOutputs(0.0, 0.0);
	}

};

START_ROBOT_CLASS(Robot)
