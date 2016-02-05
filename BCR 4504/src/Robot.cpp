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
	double gyros[535];
	
	double integral(double angle, int start)
	{
		double result = 0;
		result += gyros[0];
		for(int i = 1; i <= start - 2; i++)
		{
			result += 2*gyros[i];
		}
		if(start > 0)
		{
			result += gyros[start - 1];
		}
		result *= start/(2*200);
		return result;
	}

	void Autonomous() override
	{
		//SmartDashboard::PutNumber("Thingy ganagy ",(double)(sizeof(gyros)/sizeof(int)));
		static const double Kp = 0.03;
		static const double Ki = 0.003;
		gyro->Reset();
		//std::string autoSelected = *((std::string*)chooser->GetSelected());
		std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		myRobot->SetSafetyEnabled(false);
		//static int revolutions = 0;
		for(int x = 0; x < 10000; x++)
		{
			double angle = gyro->GetAngle();
			myRobot->Drive(0.15, -angle * Kp -integral(angle, x/10)*Ki );
			Wait(0.001);
			//gyros[x] =  (-angle * Kp);
			//std"%d,%d,%d\t",(double)((-angle * Kp)),(double)(-angle),(double)(Kp));
			gyros[x] = angle;
			if(x % 10 == 0)
			{
				//std::cout << x/15 << ' ' << -angle*Kp << ',';
				std::cout << x/10 << ' ' << -integral(angle, x/10)*Ki << ',';

			}
			//printf("%p\n", gyros);
			//Wait(0.01);
		}
		//Add condition to stop when the ballast comes into view. Need to wait until vision can be done.
		myRobot->SetLeftRightMotorOutputs(0.0, 0.0);
	}

	void RobotInit()
	{
		myRobot->SetExpiration(0.1);
		std::cout << "test" << std::endl;
		gyro->SetSensitivity(0.007);
		gyro->Calibrate();
		//gyros = (int*)malloc(sizeof(int)*1001);
		//printf("a:%p\n",gyros);
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
			static bool flag = true;
			static bool toggle = true;
			SmartDashboard::PutNumber("Test: ",toggle);
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
			//SmartDashboard::PutNumber("Gyro Angle: ", fmod(gyro->GetAngle(),360.0));
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

	void Test() override
	{
		/*static const double Kp = 0.03;
		gyro->Reset();*/
		/*while(true)
		{
			double angle = gyro->GetAngle();
			static int revolutions = 0;
			if(file == NULL)
			{
				SmartDashboard::PutString("Error: ", "Cannot load file");
			}else
			{
				fprintf(file, "%d %c", revolutions, '\t');
				fprintf(file, "%e %c", (-angle * Kp), '\n');
				SmartDashboard::PutString("Error: ", " ");
			}
			Wait(0.01);
			revolutions++;
		}
		fclose(file);*/
	}

	void Disabled()
	{
		myRobot->SetLeftRightMotorOutputs(0.0, 0.0);
		//fclose(file);
	}

};

START_ROBOT_CLASS(Robot)
