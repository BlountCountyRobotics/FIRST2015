#include "WPILib.h"

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
class Robot: public SampleRobot
{
	CANTalon *right = new CANTalon(0);
	CANTalon *left = new CANTalon(1);
	Joystick *stick = new Joystick(0);
	Joystick *buttons = new Joystick(1);
	Joystick *tank = new Joystick(2);
	RobotDrive *myRobot = new RobotDrive(left, right);
	SendableChooser *chooser = new SendableChooser();
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "Stronghold";

	void Autonomous()
	{
		//std::string autoSelected = *((std::string*)chooser->GetSelected());
		std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if(autoSelected == autoNameCustom)
		{
			//printf("Running custom Autonomous");
			myRobot->SetSafetyEnabled(false);
			myRobot->Drive(-0.5, 1.0); 	// spin at half speed
			Wait(2.0); 					// for 2 seconds
			myRobot->Drive(0.0, 0.0); 	// stop robot
		} else
		{
			//printf("Running default Autonomous");
			myRobot->SetSafetyEnabled(false);
			myRobot->Drive(-0.5, 0.0); 	// drive forwards half speed
			Wait(3.0); 				//    for 2 seconds
			myRobot->Drive(0.0, 0.0); 	// stop robot
		}

	}

	void RobotInit()
	{
		myRobot->SetExpiration(0.1);
	}

	void OperatorControl()
	{
		myRobot->SetSafetyEnabled(false);
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
