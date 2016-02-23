#include "WPILib.h"
#include <algorithm>

class Robot: public SampleRobot
{
	//typedef void (*funcp)(bool on);
	CANTalon *right = new CANTalon(0);
	CANTalon *left = new CANTalon(1);
	DoubleSolenoid *arm = new DoubleSolenoid(3,7,6);
	Solenoid *gripper = new Solenoid(3,1);
	Solenoid *gripper2 = new Solenoid(3,4);
	Solenoid *shooter = new Solenoid(3,5);
	//Solenoid *gearshift = new Solenoid(7);
	Compressor *compressor = new Compressor(3);
	Joystick *stick = new Joystick(0);
	Joystick *buttons = new Joystick(1);
	RobotDrive *myRobot = new RobotDrive(left, right/*, fr, bl*/);
	SendableChooser *chooser = new SendableChooser();
	const std::string autoNameDefault = "Default";
	AnalogInput *tiltsensor = new AnalogInput(0);
	AnalogGyro *gyro = new AnalogGyro(1);
	std::vector<double> gyros;
	BuiltInAccelerometer *accel = new BuiltInAccelerometer(Accelerometer::Range::kRange_8G);
	//const double kUpdatePeriod = 0.005;
	const double Kp = 0.003;
	const double Ki = 0.003;
	//CameraServer camserver;
	//AxisCamera *camera;
	//Image *camimage;
	enum drive_angles {DO_NOTHING = -1, PORT_ANGLE = 20, DRIVE_ANGLE = 25, SHOOT_ANGLE = 50};
	int error = 0;
public:
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
		//camera = new AxisCamera("axis-camera.local");
		//camimage = imaqCreateImage(IMAQ_IMAGE_RGB,0);
	}

	void OperatorControl()
	{
		myRobot->SetSafetyEnabled(false);
		gyro->Reset();
		while (IsOperatorControl() and IsEnabled())
		{
			//camera->GetImage(camimage);
			//camserver.SetImage(camimage);
			double angle = (round(abs(tiltsensor->GetValue())/10)*10)*(9.0/197.0);
			double epsilon = 0.1;
			static double goalangle = DO_NOTHING;
			if(stick->GetRawButton(1))
				goalangle = DRIVE_ANGLE;
			if(stick->GetRawButton(2))
				goalangle = DO_NOTHING;
			if(stick->GetRawButton(3))
				goalangle = PORT_ANGLE;
			if(stick->GetRawButton(4))
				goalangle = SHOOT_ANGLE;

			SmartDashboard::PutNumber("Tilt Sensor: ", angle);
			SmartDashboard::PutNumber("Motor Y: ", stick->GetY());
			SmartDashboard::PutNumber("Motor X: ", stick->GetX());
			bool t_drive = true;
			bool a_drive = false;
			static bool flag = true;
			static bool flag2 = true;
			static bool flag3 = true;
			static bool flag4 = true;
			static bool flag5 = true;
			SmartDashboard::PutBoolean("Compressor ",compressor->Enabled());
			if(buttons->GetRawButton(5) && !flag)
			{
				compressor->Enabled() ? compressor->Stop() : compressor->Start();
				flag = true;
			}else if(!buttons->GetRawButton(5) && flag)
			{
				flag = false;
			}
			/*2 settings for tilt sensor. Find angles
			 * Driving 1
			 * Portcullis (auto) 3
			 * Low Goal 4
			 */
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
			//Gripper and Clamp
			if(buttons->GetRawButton(10) && !flag2)
			{
				//compressor->SetCompressor(!compressor->Enabled());
				gripper->Set(!gripper->Get());
				Wait(0.5);
				gripper2->Set(!gripper2->Get());
				flag2 = true;
			}else if(!buttons->GetRawButton(10) && flag2)
			{
				flag2 = false;
			}
			//Clamp
			if(buttons->GetRawButton(1) && !flag3)
			{
				gripper2->Set(!gripper2->Get());
				flag3 = true;
			}else if(!buttons->GetRawButton(1) && flag3)
			{
				flag3 = false;
			}
			//Gripper
			if(buttons->GetRawButton(2) && !flag4)
			{
				gripper->Set(!gripper->Get());
				flag4 = true;
			}else if(!buttons->GetRawButton(2) && flag4)
			{
				flag4 = false;
			}
			//Shoot
			if(buttons->GetRawButton(4) && !flag5)
			{
				if(gripper2->Get())
					gripper2->Set(false);
				shooter->Set(true);
				Wait(.25);
				shooter->Set(false);
				flag5 = true;
			}else if(!buttons->GetRawButton(4) && flag5)
			{
				flag5 = false;
			}
			//Arm Up
			if(buttons->GetRawButton(8))
			{
				arm->Set(DoubleSolenoid::kForward);
			//Arm Down
			}else if(buttons->GetRawButton(11))
			{
				arm->Set(DoubleSolenoid::kReverse);
			}else
			{
				arm->Set(DoubleSolenoid::kOff);
			}

			//Tank Drive switch
			if(buttons->GetRawButton(7))
			{
				SmartDashboard::PutString("Drive Mode: ", "Arcade Drive");
				t_drive = false;
				a_drive = true;
			}else
			{
				SmartDashboard::PutString("Drive Mode: ", "Tank Drive");
				t_drive = true;
				a_drive = false;
			}
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
				//PID with shooter
			}else
			{
				myRobot->SetLeftRightMotorOutputs(0.0, 0.0);
			}
			/*
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
			*/
			Wait(0.005);
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
