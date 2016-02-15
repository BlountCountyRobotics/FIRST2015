#include "WPILib.h"
#include <algorithm>

class Robot: public SampleRobot
{
	struct ParticleReport
	{
		double PercentAreaToImageArea;
		double Area;
		double BoundingRectLeft;
		double BoundingRectTop;
		double BoundingRectRight;
		double BoundingRectBottom;
	};
	struct Scores
	{
		double Area;
		double Aspect;
	};
	Image *camimage;
	Image *binimage;
	int error;
	AxisCamera *camera;
	bool flag = true;
	CameraServer x;

	Range hue = {101,64};
	Range sat = {88,255};
	Range val = {134,255};

	double AREA_MINIMUM = .25;
	double AREA_MAXIMUM = .4;
	double minscore = 75;
	double VIEW_ANGLE = 64;

	Scores scores;
	ParticleFilterCriteria2 crit[1];
	ParticleFilterOptions2 options = {0,0,1,1};
	const int PCM_ID = 0;
	const int CAN_ID = 5;
	
	//Compressor *compressor = new Compressor(PCM_ID);
	//Solenoid *solenoid = new Solenoid(CAN_ID, PCM_ID);
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
	const double Kp = 0.003;
	const double Ki = 0.003;
	
	/*double integral(double angle, double x)
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
	}*/

	void Autonomous() override
	{
		gyro->Reset();
		
		int partnum = 0;
		SmartDashboard::PutNumber("Particle Amount ",partnum);
		
		std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		
		myRobot->SetSafetyEnabled(false);
		
		while(IsAutonomous() && IsEnabled() && flag)
		{
			/*double angle = gyro->GetAngle();
			myRobot->Drive(0.15, -Kp * angle );
			Wait(0.5);*/
			camera->GetImage(camimage);
			myRobot->Drive(0.0,0.0);
			error = imaqColorThreshold(binimage,camimage,255, IMAQ_HSV, &hue, &sat, &val);
			error = imaqCountParticles(binimage, 1, &partnum);
			x.SetImage(binimage);
			crit[0] = {IMAQ_MT_AREA_BY_IMAGE_AREA, (float)AREA_MINIMUM, 100.0, false, false};
			flag = false;
			if(partnum > 0)
			{
				std::vector<ParticleReport> partvec;
				for(int partindex = 0; partindex < partnum; partindex++)
				{
					ParticleReport par;
					imaqMeasureParticle(binimage, partindex, 0, IMAQ_MT_AREA, &(par.Area));
					imaqMeasureParticle(binimage, partindex, 0, IMAQ_MT_BOUNDING_RECT_TOP, &(par.BoundingRectTop));
					imaqMeasureParticle(binimage, partindex, 0, IMAQ_MT_BOUNDING_RECT_LEFT, &(par.BoundingRectLeft));
					imaqMeasureParticle(binimage, partindex, 0, IMAQ_MT_BOUNDING_RECT_BOTTOM, &(par.BoundingRectBottom));
					imaqMeasureParticle(binimage, partindex, 0, IMAQ_MT_BOUNDING_RECT_RIGHT, &(par.BoundingRectRight));
					partvec.push_back(par);
				}
				sort(partvec.begin(), partvec.end(), CompareParticleSizes);
				bool isTarget = istarget(partvec[0]);
				SmartDashboard::PutBoolean("IsTarget: ", isTarget);
				double distance = computeDistance(binimage, partvec[0]);
				SmartDashboard::PutNumber("Distance: ", distance);
			}
		}
		camimage = imaqCreateImage(IMAQ_IMAGE_RGB,0);
		binimage = imaqCreateImage(IMAQ_IMAGE_U8,0);
		myRobot->SetLeftRightMotorOutputs(0.0, 0.0);
	}

	void RobotInit()
	{
		myRobot->SetExpiration(1);
		gyro->SetSensitivity(0.007);
		gyro->Calibrate();
		camera = new AxisCamera("axis-camera.local");
		camimage = imaqCreateImage(IMAQ_IMAGE_RGB,0);
		binimage = imaqCreateImage(IMAQ_IMAGE_U8,0);
	}

	void OperatorControl()
	{
		myRobot->SetSafetyEnabled(false);
		gyro->Reset();
		//motor1->EnableControl();
		while (IsOperatorControl() and IsEnabled())
		{
			camera->GetImage(camimage);

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
			//compressor->SetClosedLoopControl(toggle);
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

};

START_ROBOT_CLASS(Robot)
