#include "WPILib.h"
#include <algorithm>

class Robot: public SampleRobot
{
	//typedef void (*funcp)(bool on);
	CANTalon *right = new CANTalon(1);
	CANTalon *left = new CANTalon(0);
	DoubleSolenoid *arm = new DoubleSolenoid(6,7);
	Solenoid *gripper = new Solenoid(1);
	Solenoid *gripper2 = new Solenoid(4);
	Solenoid *shooter = new Solenoid(5);
	//Solenoid *gearshift = new Solenoid(7);
	Compressor *compressor = new Compressor(0);
	Joystick *stick = new Joystick(0);
	Joystick *buttons = new Joystick(1);
	RobotDrive *myRobot = new RobotDrive(left, right/*, fr, bl*/);
	SendableChooser *chooser = new SendableChooser();
	const std::string autoNameDefault = "Default";
	AnalogInput *tiltsensor = new AnalogInput(0);
	AnalogGyro *gyro = new AnalogGyro(1);
	std::vector<double> gyros;
	BuiltInAccelerometer *accel = new BuiltInAccelerometer(Accelerometer::Range::kRange_8G);
	const double kUpdatePeriod = 0.005;
	const double Kp = 0.003;
	const double Ki = 0.003;
	CameraServer camserver;
	AxisCamera *camera;
	Image *camimage;
	int error = 0;
public:
	/*void toggle(funcp, bool joy, bool curr, bool &flag)
	{
		if(joy && !flag)
		{
			funcp(!curr);
			flag = true;
		}else if(!joy && flag)
		{
			flag = false;
		}
	}*/
	void Autonomous() override
	{
		gyro->Reset();
		
		//int partnum = 0;
		//SmartDashboard::PutNumber("Particle Amount ",partnum);
		
		std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		
		myRobot->SetSafetyEnabled(false);
		
		while(IsAutonomous() && IsEnabled() /*&& flag*/)
		{
			double angle = gyro->GetAngle();
			myRobot->Drive(0.15, -Kp * angle );
			Wait(0.5);
		}
		myRobot->SetLeftRightMotorOutputs(0.0, 0.0);
	}

	void RobotInit()
	{
		myRobot->SetExpiration(1);
		gyro->SetSensitivity(0.007);
		gyro->Calibrate();
		camera = new AxisCamera("axis-camera.local");
		camimage = imaqCreateImage(IMAQ_IMAGE_RGB,0);
	}

	void OperatorControl()
	{
		myRobot->SetSafetyEnabled(false);
		gyro->Reset();
		//motor1->EnableControl();
		while (IsOperatorControl() and IsEnabled())
		{
			camera->GetImage(camimage);
			camserver.SetImage(camimage);
			double tiltvalue = (round(abs(tiltsensor->GetValue())/10)*10)*(9.0/197.0);
			SmartDashboard::PutNumber("Tilt Sensor: ", tiltvalue);
			SmartDashboard::PutNumber("Motor Y: ", stick->GetY());
			SmartDashboard::PutNumber("Motor X: ", stick->GetX());
			bool t_drive = false;
			bool a_drive = true;
			static bool flag = true;
			//static bool flag2 = true;
			//static bool flag3 = true;
			//static bool flag4 = true;
			SmartDashboard::PutBoolean("Compressor ",compressor->Enabled());
			//static bool toggle = true;
			//void toggle(funcp, bool joy, bool curr, bool &flag)
			//toggle(compressor->SetCompressor,stick->GetRawButton(1),compressor->Enabled(),flag);
			if(stick->GetRawButton(1) && !flag)
			{
				//compressor->SetCompressor(!compressor->Enabled());
				compressor->Enabled() ? compressor->Stop() : compressor->Start();
				flag = true;
			}else if(!stick->GetRawButton(1) && flag)
			{
				flag = false;
			}
			//Porculus arm lifting toggle?

			/*if(buttons->GetRawButton(2) && !flag2)
			{
				armup->Set(true);
				armdown->Set(false);
				flag2 = true;
			}else if(!buttons->GetRawButton(2) && flag2)
			{
				armup->Set(false);
				armdown->Set(true);
				flag2 = false;
			}*/
			//Gear Shift toggle
			//toggle(gearshift->Set,buttons->GetRawButton(3),gearshift->Get(),flag3);
			/*if(buttons->GetRawButton(3) && !flag3)
			{
				gearshift->Set(true);
				flag3 = true;
			}else if(!buttons->GetRawButton(3) && flag3)
			{
				gearshift->Set(false);
				flag3 = false;
			}*/
			//Gripper on?
			//toggle(gripper->Set,stick->GetRawButton(2),gripper->Get(),flag4);
			if(stick->GetRawButton(2))
			{
				gripper->Set(true);
				Wait(0.5); // test times
				gripper2->Set(true);
			}
			//Degrip?
			if(stick->GetRawButton(10))
			{
				gripper2->Set(false);
				Wait(0.1);
				gripper->Set(false);
			}
			//Shoot?
			if(stick->GetRawButton(4))
			{
				shooter->Set(true);
				Wait(0.05);
				shooter->Set(false);
			}
			if(stick->GetPOV(0) == 90)
			{
				arm->Set(DoubleSolenoid::kForward);
			}else if(stick->GetPOV(0) == 270)
			{
				arm->Set(DoubleSolenoid::kReverse);
			}else
			{
				arm->Set(DoubleSolenoid::kOff);
			}
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
			//SmartDashboard::PutNumber("Distance to Image:, ", computeDistance(binimage, ));
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
			double previousX = 0;
			double previousY = 0;
			double previousZ = 0;
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
	}

	void Test() override
	{
		/*8for(int x = 0; x < 10000; x++)
		{
			if(!IsTest())
				break;
			double angle = gyro->GetAngle();
			myRobot->Drive(0.15, -Kp*angle );
			gyros.push_back(angle);
			std::cout << x << ' ' << Ki*integral(angle, x) << ',';
			printf("%d %e,", x, Ki*integral(angle, x));
			Wait(0.01);
		}*/
		myRobot->SetLeftRightMotorOutputs(0.0, 0.0);
	}

	void Disabled()
	{
		myRobot->SetLeftRightMotorOutputs(0.0, 0.0);
	}
/*
	bool istarget(ParticleReport target)
	{
		bool flag = false;
		double percent = getscore(1.0/3.0,target.Area/(target.BoundingRectBottom*target.BoundingRectLeft));
		double percent2 = getscore(.625, target.BoundingRectLeft/target.BoundingRectBottom);
		if(percent > 85)
		{
			flag = true;
		}
		return flag;
	}

	double getscore(double goal, double arearatio)
	{
		double score = (arearatio > goal) ? (1/(goal-1))*arearatio-(1/(goal-1)) : (1/(goal))*arearatio;
		return 100*score;
	}

	double computeDistance (Image *image, ParticleReport report)
	{
		double normalizedWidth, targetWidth;
		int xRes, yRes;

		imaqGetImageSize(image, &xRes, &yRes);
		normalizedWidth = 2*(report.BoundingRectRight - report.BoundingRectLeft)/xRes;
		SmartDashboard::PutNumber("Width", normalizedWidth);
		targetWidth = 7;

		return  targetWidth/(normalizedWidth*12*tan(VIEW_ANGLE*M_PI/(180*2)));
	}

	static bool CompareParticleSizes(ParticleReport particle1, ParticleReport particle2)
	{
		//we want descending sort order
		return particle1.PercentAreaToImageArea > particle2.PercentAreaToImageArea;
	}
*/
};

START_ROBOT_CLASS(Robot)
