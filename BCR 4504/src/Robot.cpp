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
	std::vector<double> gyros;
	BuiltInAccelerometer *accel = new BuiltInAccelerometer(Accelerometer::Range::kRange_8G);
	const double kUpdatePeriod = 0.005;
	const double Kp = 0.03;
	const double Ki = 0.003;
	
	double integral(double angle, double x)
	{
		double result = 0;
		int n = 1;
		result += gyros[0];
		if(x > 0.1)
		{
			for(double i = 1; i <= x * 10; i += 1)
			{
				result += 2*gyros[i];
				n++;
			}
		}
		result += angle;
		result *= x/(2*n);
		return result;
	}

	void Autonomous() override
	{
		gyro->Reset();
		//std::string autoSelected = *((std::string*)chooser->GetSelected());
		std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		myRobot->SetSafetyEnabled(false);
		double angle = gyro->GetAngle();
		myRobot->Drive(0.15, -Kp * angle );
		std::cout << "test" << std::endl;
		Wait(0.5);
		//Add condition to stop when the ballast comes into view. Need to wait until vision can be done.
		myRobot->SetLeftRightMotorOutputs(0.0, 0.0);
	}

	void RobotInit()
	{
		myRobot->SetExpiration(1);
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
			double tiltvalue = (round(abs(tiltsensor->GetValue())/10)*10)*(9.0/197.0);
			SmartDashboard::PutNumber("Tilt Sensor: ", tiltvalue);
			SmartDashboard::PutNumber("Motor Y: ", stick->GetY());
			SmartDashboard::PutNumber("Motor X: ", stick->GetX());
			bool t_drive = false;
			bool a_drive = true;
			static bool flag = true;
			static bool toggle = true;
			if(stick->GetRawButton(1) && !flag)
			{
				toggle = !toggle;
				flag = true;
			}else if(!stick->GetRawButton(1) && flag)
			{
				flag = false;
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
			//SmartDashboard::PutNumber("Gyro Rate: ", gyro->GetRate());
			SmartDashboard::PutNumber("Gyro Angle: ", fmod(gyro->GetAngle(),360.0));
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
		double previousX = 0;
		double previousY = 0;
		double previousZ = 1;
		double xAcceleration = accel->GetX();
		double yAcceleration = accel->GetY();
		double zAcceleration = accel->GetZ();

		SmartDashboard::PutNumber("X-Axis G:", xAcceleration);
		SmartDashboard::PutNumber("Y-Axis G:", yAcceleration);
		SmartDashboard::PutNumber("Z-Axis G:", zAcceleration);

		SmartDashboard::PutNumber("Recursive X-Axis Average:", ((xAcceleration*0.1) + (0.9*previousX)));

		SmartDashboard::PutNumber("Recursive Y-Axis Average:", ((yAcceleration*0.1) + (0.9*previousX)));

		SmartDashboard::PutNumber("Recursive Z-Axis Average:", ((zAcceleration*0.1) + (0.9*previousX)));

		previousX = (xAcceleration*0.1) + (0.9*previousX);
		previousY = (yAcceleration*0.1) + (0.9*previousY);
		previousZ = (zAcceleration*0.1) + (0.9*previousZ);
		Wait(kUpdatePeriod);
	}

	void Test() override
	{
		for(int x = 0; x < 10000; x++)
		{
			if(!IsTest())
				break;
			double angle = gyro->GetAngle();
			myRobot->Drive(0.15, -Kp*angle );
			gyros.push_back(angle);
			std::cout << x << ' ' << Ki*integral(angle, x) << ',';
			printf("%d %e,", x, Ki*integral(angle, x));
			Wait(0.01);
		}
		myRobot->SetLeftRightMotorOutputs(0.0, 0.0);
	}

	void Disabled()
	{
		myRobot->SetLeftRightMotorOutputs(0.0, 0.0);
	}

};

START_ROBOT_CLASS(Robot)
