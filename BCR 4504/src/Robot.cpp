#include "WPILib.h"

class Robot: public SampleRobot
{
	Compressor *compressor = new Compressor(3);
	CANTalon *right = new CANTalon(0);
	CANTalon *left = new CANTalon(1);
	//CANTalon *motor1 = new CANTalon(2);
	//CANTalon *motor2 = new CANTalon(3);
	Joystick *stick = new Joystick(0);
	Joystick *buttons = new Joystick(1);
	RobotDrive *myRobot = new RobotDrive(left, right);
	SendableChooser *chooser = new SendableChooser();
	const std::string autoNameDefault = "Default";
	AnalogInput *tiltsensor = new AnalogInput(0);
	AnalogGyro *gyro = new AnalogGyro(1);
	
	void Autonomous()
	{
		static const double Kp = 0.03;
		gyro->Reset();
		//std::string autoSelected = *((std::string*)chooser->GetSelected());
		std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		myRobot->SetSafetyEnabled(false);
		bool flag, flag2;
		std::ofstream file("gyro.txt", std::ofstream::out);
		while(!flag)
		{
			double angle = gyro->GetAngle();
			myRobot->Drive(0.25, -angle * Kp);
			static int revolutions = 0;
			if(file.is_open())
			{
				file << revolutions << '\t';
				file << -angle * Kp << '\n';
			}else if(!flag2)
			{
				SmartDashboard::PutString("Error: ", "Cannot load file");
				flag2 = true;
			}
			Wait(0.004);
			revolutions++;
			if(revolutions == 2300)
			{
				flag = true;
			}
		}
		file.close();
		//Add condition to stop when the ballast comes into view. Need to wait until vision can be done.
		myRobot->SetLeftRightMotorOutputs(0.0, 0.0);
	}

	void RobotInit()
	{
		myRobot->SetExpiration(0.1);
		gyro->SetSensitivity(0.007);
		gyro->Calibrate();
	}

	void OperatorControl()
	{
		myRobot->SetSafetyEnabled(false);
		gyro->Reset();
		//motor1->EnableControl();
		while (IsOperatorControl() and IsEnabled())
		{
			SmartDashboard::PutNumber("Motor Y: ", stick->GetY());
			SmartDashboard::PutNumber("Motor X: ", stick->GetX());
			bool t_drive = false;
			bool a_drive = true;
			static bool toggle = true;
			SmartDashboard::PutNumber("Test: ",toggle);
			if(stick->GetRawButton(1))
			{
				toggle = !toggle;
			}
			compressor->SetClosedLoopControl(toggle);
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
			SmartDashboard::PutNumber("Gyro Rate: ", gyro->GetRate());
			SmartDashboard::PutNumber("Gyro Angle: ", fmod(gyro->GetAngle(),360.0));
			//SmartDashboard::PutNumber("Gyro Angle: ", gyro->GetAngle());
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
		static const double Kp = 0.03;
		gyro->Reset();
		std::ofstream file("gyro.txt", std::ofstream::out);
		while(true)
		{
			double angle = gyro->GetAngle();
			static int revolutions = 0;
			if(file.is_open())
			{
				file << revolutions << '\t';
				file << -angle * Kp << '\n';
				SmartDashboard::PutString("Error: ", " ");
			}else
			{
				SmartDashboard::PutString("Error: ", "Cannot load file");
			}
			Wait(0.01);
			revolutions++;
		}
		file.close();
	}

	void Disabled()
	{
		myRobot->SetLeftRightMotorOutputs(0.0, 0.0);
	}

};

START_ROBOT_CLASS(Robot)
