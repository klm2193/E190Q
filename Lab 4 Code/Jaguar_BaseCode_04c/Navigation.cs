using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;
using System.Collections.Generic;

namespace DrRobot.JaguarControl
{   
    //THIS LINE SHOULD NOT EXIST IF REVERTED SUCCESSFULLY
    //HELLO THIS IS A TEST CASE FOR REVERT
    public class Navigation
    {
        #region Navigation Variables
        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double initialX, initialY, initialT;
        public double x, y, t;
        public double x_est, y_est, t_est;
        public double desiredX, desiredY, desiredT;

        public double currentEncoderPulseL, currentEncoderPulseR;
        public double lastEncoderPulseL, lastEncoderPulseR;
        public double wheelDistanceR, wheelDistanceL;
        public double tiltAngle, zoom;
        public double currentAccel_x, currentAccel_y, currentAccel_z;
        public double lastAccel_x, lastAccel_y, lastAccel_z;
        public double currentGyro_x, currentGyro_y, currentGyro_z;
        public double last_v_x, last_v_y;
        public double filteredAcc_x, filteredAcc_y;

        public int robotType, controllerType;
        enum ROBOT_TYPE { SIMULATED, REAL };
        enum CONTROLLERTYPE { MANUALCONTROL, POINTTRACKER, EXPERIMENT };
        public bool motionPlanRequired, displayParticles, displayNodes, displaySimRobot;
        private JaguarCtrl jaguarControl;
        private AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;
        private AxDDrRobotSentinel_Simulator simulatedJaguar;
        private Thread controlThread;
        private short motorSignalL, motorSignalR;
        private short desiredRotRateR, desiredRotRateL;
        public bool runThread = true;
        public bool loggingOn;
        StreamWriter logFile;
        public int deltaT = 10;
        private static int encoderMax = 32767;
        public int pulsesPerRotation = 190;
        public double wheelRadius = 0.089;
        public double robotRadius = 0.242;//0.232
        private double angleTravelled, distanceTravelled;
        private double diffEncoderPulseL, diffEncoderPulseR;
        private double maxVelocity = 0.3;
        private double Kpho = 0.5;//0.5;
        private double Kalpha = -2.0;//-2.0;
        private double Kbeta = 0.5;//0.5;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;

        public short K_P = 15;//15;
        public short K_I = 0;//0;
        public short K_D = 3;//3;
        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;
        public double e_L_last = 0;
        public double e_R_last = 0;

        public double rotRateL, rotRateR;
        public double K_p, K_i, K_d, maxErr;

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        public double pho = 0;
        public double deltaAngle = 0;
        Boolean InitialReached = false;
        double desiredAngle = 0;

        private String streamPath_;
        Random rnd = new Random((int)DateTime.Now.Ticks); // seed random number with system time

        // PF Variables
        public Map map;
        public Particle[] particles;
        public Particle[] propagatedParticles;
        public int numParticles = 1000;
        public double K_wheelRandomness = 0.15;//0.25
        public Random random = new Random();
        public bool newLaserData = false;
        public double laserMaxRange = 4.0;
        public double laserMinRange = 0.2;
        public double[] laserAngles;
        private int laserCounter;
        private int laserStepSize = 3;

        double laserX;
        double laserY;
        double laserT;

        public class Particle
        {
            public double x, y, t, w;

            public Particle()
            {
            }
        }

        #endregion


        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            map = new Map();
            particles = new Particle[numParticles];
            propagatedParticles = new Particle[numParticles];
            // Create particles
            for (int i = 0; i < numParticles; i++)
            {
                particles[i] = new Particle();
                propagatedParticles[i] = new Particle();
            }

            this.Initialize();


            // Start Control Thread
            controlThread = new Thread(new ThreadStart(runControlLoop));
            controlThread.Start();
        }

        // All class variables are initialized here
        // This is called every time the reset button is pressed
        public void Initialize()
        {
            // Initialize state estimates
            x = 0;//initialX;
            y = 0;//initialY;
            t = 1.57;//initialT;

            // Initialize state estimates
            x_est = 0;//initialX;
            y_est = 0;//initialY;
            t_est = 0;//initialT;

            // Set desired state
            desiredX = 0;// initialX;
            desiredY = 0;// initialY;
            desiredT = 0;// initialT;

            initialX = 0; // Defining the initial values
            initialY = 0;
            initialT = 0;

            // Reset Localization Variables
            wheelDistanceR = 0;
            wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            loggingOn = false;

            // Set random start for particles
            InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;

            laserAngles = new double[LaserData.Length];
            for (int i = 0; i < LaserData.Length; i++)                
                laserAngles[i] = DrRobot.JaguarControl.JaguarCtrl.startAng + DrRobot.JaguarControl.JaguarCtrl.stepAng * i;

        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
        {
            simulatedJaguar.Reset();
            GetFirstEncoderMeasurements();
            CalibrateIMU();
            Initialize();
            //InitializeParticles(); //places particles in proper location
        }
        #endregion


        #region Main Loop

        /************************ MAIN CONTROL LOOP ***********************/
        // This is the main control function called from the control loop
        // in the RoboticsLabDlg application. This is called at every time
        // step.
        // Students should choose what type of localization and control 
        // method to use. 
        public void runControlLoop()
        {
            // Wait
            Thread.Sleep(500);

            // Don't run until we have gotten our first encoder measurements to difference with
            GetFirstEncoderMeasurements();

            // Run infinite Control Loop
            while (runThread)
            {
                // ****************** Additional Student Code: Start ************

                // Students can select what type of localization and control
                // functions to call here. For lab 1, we just call the function
                // WallPositioning to have the robot maintain a constant distance
                // to the wall (see lab manual).
                
                // Update Sensor Readings
                UpdateSensorMeasurements();

                // Determine the change of robot position, orientation (lab 2)	
                MotionPrediction();

                // Update the global state of the robot - x,y,t (lab 2)
                LocalizeRealWithOdometry();

                // Update the global state of the robot - x,y,t (lab 2)
                //LocalizeRealWithIMU();
                

                // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                if ((diffEncoderPulseL != 0) && (diffEncoderPulseR != 0))
                {
                    LocalizeEstWithParticleFilter();
                }


                // If using the point tracker, call the function
                if (jaguarControl.controlMode == jaguarControl.AUTONOMOUS)
                {

                    // Check if we need to create a new trajectory
                    if (motionPlanRequired)
                    {
                        // Construct a new trajectory (lab 5)
                        PRMMotionPlanner();
                        motionPlanRequired = false;
                    }

                    // Drive the robot to a desired Point (lab 3)
                    FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                    //TrackTrajectory();

                    // Actuate motors based actuateMotorL and actuateMotorR
                    if (jaguarControl.Simulating())
                    {
                        CalcSimulatedMotorSignals();
                        ActuateMotorsWithVelControl();
                    }
                    else 
                    {
                        // Determine the desired PWM signals for desired wheel speeds
                        CalcMotorSignals();
                        ActuateMotorsWithPWMControl();
                    }

                }
                else
                {
                    e_sum_L = 0;
                    e_sum_R = 0;
                }
                
                // ****************** Additional Student Code: End   ************

                // Log data
                LogData();

                // Sleep to approximate 20 Hz update rate
                Thread.Sleep(deltaT); //not sure if this works anymore..... -wf
            }
        }


        public void CalibrateIMU()
        {

            accCalib_x = 0;
            accCalib_y = 0;
            int numMeasurements = 100;
            for (int i = 0; i < numMeasurements; i++)
            {
                accCalib_x += currentAccel_x;
                accCalib_y += currentAccel_y;

                Thread.Sleep(deltaT);
            }
            accCalib_x = accCalib_x / numMeasurements;
            accCalib_y = accCalib_y /numMeasurements;


        }


        // Before starting the control loop, the code checks to see if 
        // the robot needs to get the first encoder measurements
        public void GetFirstEncoderMeasurements()
        {
            if (!jaguarControl.Simulating())
            {
                // Get last encoder measurements
                bool gotFirstEncoder = false;
                int counter = 0;
                while (!gotFirstEncoder && counter < 10)
                {
                    try
                    {
                        currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                        currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();
                        lastEncoderPulseL = currentEncoderPulseL;
                        lastEncoderPulseR = currentEncoderPulseR;
                        gotFirstEncoder = true;

                        currentAccel_x = jaguarControl.getAccel_x();
                        currentAccel_y = jaguarControl.getAccel_y();
                        currentAccel_z = jaguarControl.getAccel_z();
                        lastAccel_x = currentAccel_x;
                        lastAccel_y = currentAccel_y;
                        lastAccel_z = currentAccel_z;
                        last_v_x = 0;
                        last_v_y = 0;

                    }
                    catch (Exception e) { }
                    counter++;
                    Thread.Sleep(100);
                }
            }
            else
            {
                currentEncoderPulseL = 0;
                currentEncoderPulseR = 0;
                lastEncoderPulseL = 0;
                lastEncoderPulseR = 0;
                lastAccel_x = 0;
                lastAccel_y = 0;
                lastAccel_z = 0;
                last_v_x = 0;
                last_v_y = 0;

            }
        }

        // At every iteration of the control loop, this function will make 
        // sure all the sensor measurements are up to date before
        // makeing control decisions.
        public void UpdateSensorMeasurements()
        {
            // For simulations, update the simulated measurements
            if (jaguarControl.Simulating())
            {
                jaguarControl.simulatedJaguar.UpdateSensors(deltaT);

                // Get most recenct encoder measurements
                currentEncoderPulseL = simulatedJaguar.GetEncoderPulse4();
                currentEncoderPulseR = simulatedJaguar.GetEncoderPulse5();

                // Get most recent laser scanner measurements
                laserCounter = laserCounter + deltaT;
                if (laserCounter >= 2000)
                {
                    for (int i = 0; i < LaserData.Length; i=i+laserStepSize)
                    {
                        LaserData[i] = (long)(1000 * map.GetClosestWallDistance(x, y, t -1.57 + laserAngles[i]));
                    }
                    laserCounter = 0;
                    newLaserData = true;

                    laserX = x;
                    laserY = y;
                    laserT = t;
                }
            }
            else
            {
                // Get most recenct encoder measurements
                try
                {

                    // Update IMU Measurements
                    currentAccel_x = jaguarControl.getAccel_x();
                    currentAccel_y = jaguarControl.getAccel_y();
                    currentAccel_z = jaguarControl.getAccel_z();
                   
                    // Update Encoder Measurements
                    currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                    currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();

                }
                catch (Exception e)
                {
                }
            }
        }

        // At every iteration of the control loop, this function calculates
        // the PWM signal for corresponding desired wheel speeds
        public void CalcSimulatedMotorSignals()
        {

            motorSignalL = (short)(desiredRotRateL);
            motorSignalR = (short)(desiredRotRateR);

        }
        public void CalcMotorSignals()
        {
            short zeroOutput = 16383;       //sets lower limit to motor signal
            short maxPosOutput = 32767;     //sets upper limit to motor signal

            double K_p = 65; // 25;       //PID Constants
            double K_i = 4.9;// 0.1;
            double K_d = 250; //0.5

            double K_p_R = 95; // 30;// 25;       //Only Rotation PID Constants
            double K_i_R = 0; // 4.5;// 0.1;
            double K_d_R = 375;// 1;

            double maxErr = 8000 / deltaT;  //sets the maximum value for integration of velocity error


            e_L = desiredRotRateL - diffEncoderPulseL / deltaT; //calculates the proportional error of velocity
            e_R = desiredRotRateR - diffEncoderPulseR / deltaT;

            e_sum_L = .9 * e_sum_L + e_L * deltaT;  //calculates the integral term ... not sure what the 0.9 term is
            e_sum_R = .9 * e_sum_R + e_R * deltaT;

            e_sum_L = Math.Max(-maxErr, Math.Min(e_sum_L, maxErr)); //limits the integral term of the error
            e_sum_R = Math.Max(-maxErr, Math.Min(e_sum_R, maxErr));

            u_L = ((K_p * e_L) + (K_i * e_sum_L) + (K_d * (e_L - e_L_last) / deltaT));
            e_L_last = e_L;

            u_R = ((K_p * e_R) + (K_i * e_sum_R) + (K_d * (e_R - e_R_last) / deltaT));
            e_R_last = e_R;
            // The following settings are used to help develop the controller in simulation.
            // They will be replaced when the actual jaguar is used.
            //motorSignalL = (short)(zeroOutput + desiredRotRateL * 100);// (zeroOutput + u_L);
            //motorSignalR = (short)(zeroOutput - desiredRotRateR * 100);//(zeroOutput - u_R);

            //PID Control for Rotation
            if ((pho < 0.1) && (Math.Abs(deltaAngle) < 0.18)) // if the robot reaches a desired distance range and angle, it should stop the motors
            {
                motorSignalL = 0;
                motorSignalR = 0;
            }
            else if ((pho < 0.1) && (Math.Abs(deltaAngle) > 0.18))
            {
                u_L = ((K_p_R * e_L) + (K_i_R * e_sum_L) + (K_d_R * (e_L - e_L_last) / deltaT));
                u_R = ((K_p_R * e_R) + (K_i_R * e_sum_R) + (K_d_R * (e_R - e_R_last) / deltaT));
            }

            motorSignalL = (short)(zeroOutput + u_L);
            motorSignalR = (short)(zeroOutput - u_R);   //u_R is negative because the encoders count backwards

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL)); //sets limit to motorSignal
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));

            //motorSignalR = -18000;
            //motorSignalL = 18000;

            //Console.Write("left motor: " + motorSignalL + "\n");
            //Console.Write("right motor: " + motorSignalR + "\n");

        }

        // At every iteration of the control loop, this function sends
        // the width of a pulse for PWM control to the robot motors
        public void ActuateMotorsWithPWMControl()
        { 
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            else
            {
                jaguarControl.realJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotorsWithVelControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
        }
        #endregion


        #region Logging Functions

        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        public void TurnLoggingOn()
        {
            //int fileCnt= 0;
            String date = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "-" + DateTime.Now.Minute.ToString();
            ToString();
            //logFile = File.CreateText("../../../Data/JaguarData_" + date + ".txt");

            streamPath_ = "../../../Data/JaguarData_" + date + ".csv";
            logFile = File.CreateText(streamPath_);
            string header = "time,x,y,t";
            logFile.WriteLine(header);
            logFile.Close();

            //logFile = File.CreateText("../../../Data/JaguarData_" + date + ".csv"); // write to an Excel file instead
            startTime = DateTime.Now;
            loggingOn = true;
        }

        // This function is called from a dialogue window "Record" button
        // It closes the log file and sets the logging On flag to false
        public void TurnLoggingOff()
        {
            if (logFile != null)
                logFile.Close();
            loggingOn = false;
        }

        // This function is called at every iteration of the control loop
        // IF the loggingOn flag is set to true, the function checks how long the 
        // logging has been running and records this time
        private void LogData()
        {
            if (loggingOn)
            {
                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;
                //String newData = time.ToString() + " " + x.ToString() + " " + y.ToString() + " " + t.ToString();

                //String newData = time.ToString() + " " + (LaserData[113]).ToString() + " " + (1000 - LaserData[113]).ToString();

                String newData = time.ToString() + "," + x.ToString() + "," + y.ToString() + "," + t.ToString(); // separate by commas

                logFile = File.AppendText(streamPath_);
                logFile.WriteLine(newData);
                logFile.Close();
            }
        }
        #endregion


        # region Control Functions

        // This will bound the input angle to be between -PI and +PI
        private double BoundAngle(double angle)
        {
            if (angle > Math.PI)
                angle = -(2 * Math.PI) + angle;
            else if (angle < -Math.PI)
                angle = (2 * Math.PI) + angle;

            return angle;
        }

        // This function is called at every iteration of the control loop
        // It will drive the robot forward or backward to position the robot 
        // 1 meter from the wall.
        private void WallPositioning()
        {

            // Here is the distance measurement for the central laser beam 
            double centralLaserRange = LaserData[113];

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            // Send Control signals, put negative on left wheel control
            motorSignalR = (short)(-maxVelocity * (1000 - centralLaserRange));
            motorSignalL = (short)(-maxVelocity * (1000 - centralLaserRange));

 

            // ****************** Additional Student Code: End   ************                
        }


        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
        private void FlyToSetPoint()
        {

            // ****************** Additional Student Code: Start ************

            double deltaX = desiredX - x_est;
            double deltaY = desiredY - y_est;
            double deltaT = desiredT - t_est;
            deltaAngle = deltaT;

            //double pho = Math.Sqrt(Math.Pow(deltaX, 2) + Math.Pow(deltaY, 2));
            pho = Math.Sqrt(Math.Pow(deltaX, 2) + Math.Pow(deltaY, 2));
            double alpha = -t + Math.Atan2(deltaY, deltaX);
            double beta = -t - alpha + desiredT;

            /*
            if (alpha > Math.PI)
                alpha = -(2 * Math.PI) + alpha;
            else if (alpha < -Math.PI)
                alpha = (2 * Math.PI) + alpha;
            */

            alpha = BoundAngle(alpha);
            beta = BoundAngle(beta);

            double desiredV;
            double desiredW;

            if (pho >= 0.1)
            {

                if (Math.Abs(alpha) < Math.PI / 2)
                {
                    desiredV = Kpho * pho;
                    desiredW = (Kalpha * alpha) + (Kbeta * beta);

                    desiredRotRateL = (short)(((robotRadius / wheelRadius * desiredW) + (1 / wheelRadius * desiredV)) * pulsesPerRotation / (2 * Math.PI));
                    desiredRotRateR = (short)(((1 / wheelRadius * desiredV) - (robotRadius / wheelRadius * desiredW)) * pulsesPerRotation / (2 * Math.PI));
                }

                // If the target is behind the robot, the desiredV and desiredW should have their signs flipped
                //if (Math.Abs(alpha) > Math.PI/2)
                else
                {
                    if (alpha > 0)
                    {
                        alpha = alpha - Math.PI;
                    }
                    else
                    {
                        alpha = alpha + Math.PI;
                    }

                    beta = -t - alpha + desiredT;
                    desiredV = Kpho * pho;
                    desiredW = (Kalpha * alpha) + (Kbeta * beta);

                    desiredRotRateR = (short)-(((robotRadius / wheelRadius * desiredW) + (1 / wheelRadius * desiredV)) * pulsesPerRotation / (2 * Math.PI));
                    desiredRotRateL = (short)-(((1 / wheelRadius * desiredV) - (robotRadius / wheelRadius * desiredW)) * pulsesPerRotation / (2 * Math.PI));
                }
            }


            if ((pho < 0.1) && (Math.Abs(deltaT) < 0.18)) // if the robot reaches a desired distance range and angle, it should stop the motors
            {
                desiredRotRateL = 0;
                desiredRotRateR = 0;
            }
            else if ((pho < 0.1) && (Math.Abs(deltaT) > 0.18))
            {
                desiredRotRateR = (short)(maxVelocity * 5000 * deltaT);
                desiredRotRateL = (short)-(maxVelocity * 5000 * deltaT);

                if (Math.Abs(deltaT) >= Math.PI)
                {
                    desiredRotRateR = (short)-(maxVelocity * 5000 * deltaT);
                    desiredRotRateL = (short)(maxVelocity * 5000 * deltaT);
                }
            }


            // Setting the Maximum Velocity
            double VelL = desiredRotRateL * 2 * Math.PI * wheelRadius / pulsesPerRotation;
            double VelR = desiredRotRateR * 2 * Math.PI * wheelRadius / pulsesPerRotation;

            double maxPulsesPerSec = maxVelocity * pulsesPerRotation / (2 * Math.PI * wheelRadius);

            if ((Math.Abs(VelL) > maxVelocity) && (Math.Abs(VelL) >= Math.Abs(VelR)))
            {
                double VelRatio = Math.Abs(desiredRotRateR / desiredRotRateL);
                double signL = (desiredRotRateL < 0) ? -1 : 1;
                double signR = (desiredRotRateR < 0) ? -1 : 1;

                desiredRotRateL = (short)(maxPulsesPerSec * signL);
                desiredRotRateR = (short)(maxPulsesPerSec * signR * VelRatio);
            }

            else if ((Math.Abs(VelR) > maxVelocity) && (Math.Abs(VelR) >= Math.Abs(VelL)))
            {
                double VelRatio = Math.Abs(desiredRotRateL / desiredRotRateR);
                double signR = (desiredRotRateR < 0) ? -1 : 1;
                double signL = (desiredRotRateL < 0) ? -1 : 1;

                desiredRotRateR = (short)(maxPulsesPerSec * signR);
                desiredRotRateL = (short)(maxPulsesPerSec * signL * VelRatio);
            }

            //Console.Write("Left Speed: " + (desiredRotRateL * 2 * Math.PI * 0.089/190) + "\n");
            //Console.Write("Right Speed: " + (desiredRotRateR* 2 * Math.PI * 0.089/190) + "\n");
            // ****************** Additional Student Code: End   ************

        }



        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {
            /*
            // Following a Straight Line
            desiredX = x_est + 0.1;
            desiredY = desiredX * 2 + 1;
            desiredT = Math.Atan2(2, 1);
            */

            /*
            // Following a Circular Path
            if ((x_est < 2) && (!InitialReached))
            {
                desiredX = 2.2;
                desiredY = 0;
                desiredT = -1.57;
            }

            else
            {
                InitialReached = true;
                //double angleWRTCircle = Math.Atan2(y_est, x_est);
                //double desiredAngle = angleWRTCircle - 0.31;
                desiredAngle = desiredAngle - 0.001;
                desiredT = desiredAngle - 1.57;
                
                if (desiredT > Math.PI)
                    desiredT = -(2 * Math.PI) + desiredT;
                else if (desiredT < -Math.PI)
                    desiredT = (2 * Math.PI) + desiredT;
                
                desiredX = 2 * Math.Cos(desiredAngle);  //THIS MIGHT BE WRONG< CHECK IT. OR DESIRED ANGLE
                desiredY = 2 * Math.Sin(desiredAngle);
            }
            */

            /*
            // Following a Sinusoidal Path
            desiredX = x_est + 0.1;
            desiredY = Math.Sin(desiredX);
            */

            FlyToSetPoint();
        }

        // THis function is called to construct a collision-free trajectory for the robot to follow
        private void PRMMotionPlanner()
        {

        }


        #endregion


        #region Localization Functions
        /************************ LOCALIZATION ***********************/

        // This function will grab the most recent encoder measurements
        // from either the simulator or the robot (whichever is activated)
        // and use those measurements to predict the RELATIVE forward 
        // motion and rotation of the robot. These are referred to as
        // distanceTravelled and angleTravelled respectively.
        public void MotionPrediction()
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated distanceTravelled and angleTravelled.
            // You can set and use variables like diffEncoder1, currentEncoderPulse1,
            // wheelDistanceL, wheelRadius, encoderResolution etc. These are defined
            // in the Robot.h file.

            //double diffEncoderPulseL;
            //double diffEncoderPulseR;

            // Accounted for the encoder on the right side that moves in the opposite direction
            // relative to the left side by giving it a negative sign
            double initialL = lastEncoderPulseL;
            double finalL = currentEncoderPulseL;
            double initialR = -lastEncoderPulseR;
            double finalR = -currentEncoderPulseR;

            // Accounted for the wrap around by making sure that the distance traveled by the encoder
            // remained under the threshold value of a single wheel rotation
            if (finalL - initialL < -pulsesPerRotation)
                diffEncoderPulseL = encoderMax - initialL + finalL;
            else if (finalL - initialL > pulsesPerRotation)
                diffEncoderPulseL = finalL - initialL - encoderMax;
            else
                diffEncoderPulseL = finalL - initialL;

            if (finalR - initialR < -pulsesPerRotation)
                diffEncoderPulseR = encoderMax - initialR + finalR;
            else if (finalR - initialR > pulsesPerRotation)
                diffEncoderPulseR = finalR - initialR - encoderMax;
            else
                diffEncoderPulseR = finalR - initialR;

            // update the last encoder measurements
            lastEncoderPulseL = currentEncoderPulseL;
            lastEncoderPulseR = currentEncoderPulseR;

            // calculate wheel distances based on encoder pulses
            wheelDistanceL = ((double)diffEncoderPulseL / (double)pulsesPerRotation) * 2 * Math.PI * wheelRadius;
            wheelDistanceR = ((double)diffEncoderPulseR / (double)pulsesPerRotation) * 2 * Math.PI * wheelRadius;

            // calculate distance travelled
            distanceTravelled = (double)((wheelDistanceL + wheelDistanceR) / 2.0);
            angleTravelled = (double)((wheelDistanceR - wheelDistanceL) / (2.0 * robotRadius));


            // ****************** Additional Student Code: End   ************
        }

        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithOdometry()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi

            double deltaX = distanceTravelled * Math.Cos(t + (double)angleTravelled / (double)2);
            double deltaY = distanceTravelled * Math.Sin(t + (double)angleTravelled / (double)2);
            double totalAngle = t + angleTravelled;

            // Update the actual
            x += deltaX;
            y += deltaY;

            if (totalAngle > Math.PI)
                t = -(2 * Math.PI) + totalAngle;
            else if (totalAngle < -Math.PI)
                t = (2 * Math.PI) + totalAngle;
            else
                t = totalAngle;

            // ****************** Additional Student Code: End   ************
        }

        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithIMU()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi


            // ****************** Additional Student Code: End   ************
        }


        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab
            

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x_est, y_est, t_est using a PF

            //x_est = 0; y_est = 0; t_est = 0; (this was in lab 4)

            List<int> weightedParticles = new List<int>();

            for (int i = 0; i < numParticles; i++)
            {
                double wheelLError = 0.01;
                double wheelRError = 0.01;

                double wheelDistanceLRand = wheelDistanceL + RandomGaussian() * wheelLError;
                double wheelDistanceRRand = wheelDistanceR + RandomGaussian() * wheelRError;

                double distanceTravelledGaussian = (double)((wheelDistanceLRand + wheelDistanceRRand) / 2.0);
                double angleTravelledGaussian = (double)((wheelDistanceRRand - wheelDistanceLRand) / (2.0 * robotRadius));


                double deltaX = distanceTravelledGaussian * Math.Cos(particles[i].t + (double)angleTravelledGaussian / (double)2);
                double deltaY = distanceTravelledGaussian * Math.Sin(particles[i].t + (double)angleTravelledGaussian / (double)2);

                /*
                propagatedParticles[i].x = particles[i].x + deltaX;
                propagatedParticles[i].y = particles[i].y + deltaY;

                double totalAngle = particles[i].t + angleTravelledGaussian;
                 * */

                propagatedParticles[i].x = x;
                propagatedParticles[i].y = y;
                double totalAngle = t;

                if (totalAngle > Math.PI)
                    propagatedParticles[i].t = -(2 * Math.PI) + totalAngle;
                else if (totalAngle < -Math.PI)
                    propagatedParticles[i].t = (2 * Math.PI) + totalAngle;
                else
                    propagatedParticles[i].t = totalAngle;

                CalculateWeight(i);

                int numCopies = 0;

                // Approximate Method to generate the Weighted Particle List
                if (propagatedParticles[i].w < 0.25)
                {
                    numCopies = 1;
                }
                else if (propagatedParticles[i].w < 0.5)
                {
                    numCopies = 2;
                }
                else if (propagatedParticles[i].w < 0.75)
                {
                    numCopies = 3;
                }
                else if (propagatedParticles[i].w <= 1.0)
                {
                    numCopies = 4;
                }

                for (int j = 0; j < numCopies; j++)
                {
                    weightedParticles.Add(i);
                }
            }

             double totalX = 0;
             double totalY = 0;
             double totalTReal = 0;
             double totalTImag = 0;
             double TReal;
             double TImag;

            // Resampling the Particle List
            for (int i = 0; i < numParticles; i++)
            {
                int sampledParticle = (int)(random.NextDouble() * weightedParticles.Count);
                //particles[i] = propagatedParticles[i];
                particles[i] = propagatedParticles[weightedParticles[sampledParticle]];

                totalX = particles[i].x + totalX;
                totalY = particles[i].y + totalY;

                TReal = Math.Cos(particles[i].t);
                TImag = Math.Sin(particles[i].t);

                totalTReal += TReal;
                totalTImag += TImag; 

            }


            // This is from Lab 1
            x_est = totalX / (double)numParticles;
            y_est = totalY / (double)numParticles;
            t_est = Math.Atan2(totalTImag, totalTReal);


            // ****************** Additional Student Code: End   ************

        }

        // Particle filters work by setting the weight associated with each
        // particle, according to the difference between the real robot 
        // range measurements and the predicted measurements associated 
        // with the particle.
        // This function should calculate the weight associated with particle p.

        void CalculateWeight(int p)
        {
            double weight = 0;

            double laserSD = 1;
            double xParticle = propagatedParticles[p].x;
            double yParticle = propagatedParticles[p].y;
            double tParticle = propagatedParticles[p].t;

            //double[] particleAngleArray = { BoundAngle(t+1.05), BoundAngle(t+0.52), t, BoundAngle(t-0.52), BoundAngle(t-1.05) };
            int[] nominalAngleArray = {114};// 54, 84, 114, 144, 174 };

            if (newLaserData)
            {
                for (int i = 0; i < nominalAngleArray.Length; i++)
                {
                    int angle = nominalAngleArray[i]; // angle from laser scanner
                    //double particleLaserDist = map.GetClosestWallDistance(xParticle, yParticle, particleAngleArray[i]);

                    double particleLaserDist = map.GetClosestWallDistance(xParticle, yParticle, tParticle - 1.57 + laserAngles[angle]);
                    double robotLaserDist = map.GetClosestWallDistance(x, y, BoundAngle(t - 1.57 + laserAngles[angle]));
                    double nominalLaserDist = LaserData[angle] / (double)1000; //converts laser data to meters

                    double angleWeight = Math.Exp(-0.5 * (Math.Pow((particleLaserDist - nominalLaserDist) / laserSD, 2.0)));
                    weight += angleWeight;

                    if (Math.Abs(particleLaserDist - nominalLaserDist) > 0.1)
                    {
                        double xDiff = x - laserX;
                        double yDiff = y - laserY;
                        double tDiff = t - laserT;

                        double partlaserdist = map.GetClosestWallDistance(x, y, BoundAngle(t - 1.57 + laserAngles[angle]));
                        double laserdist = map.GetClosestWallDistance(laserX, laserY, BoundAngle(laserT - 1.57 + laserAngles[angle]));

                        double addition = xDiff + yDiff + tDiff + partlaserdist + laserdist;
                        addition += addition;
                    }
                }

                weight = weight / nominalAngleArray.Length;
                propagatedParticles[p].w = weight;

                newLaserData = false;

                
            } 

            weight = weight / nominalAngleArray.Length;
            propagatedParticles[p].w = weight;

	        // ****************** Additional Student Code: Start ************

	        // Put code here to calculated weight. Feel free to use the
	        // function map.GetClosestWallDistance from Map.cs.

        }



        // This function is used to initialize the particle states 
        // for particle filtering. It should pick a random location in the 
        // environment for each particle by calling SetRandomPos

        void InitializeParticles() {

	        // Set particles in random locations and orientations within environment
	        for (int i=0; i< numParticles; i++)
            {
		        // Either set the particles at known start position [0 0 0],  
		        // or set particles at random locations.

                if (jaguarControl.startMode == jaguarControl.UNKNOWN)
    		        SetRandomPos(i);
                else if (jaguarControl.startMode == jaguarControl.KNOWN)
		            SetStartPos(i);
	        }
            
        }



        // For particle p, this function will select a valid position. It should
        // select the position randomly, with equal likelihood of being anywhere 
        // in the environement. Should work for rectangular environments to make 
        // things easier.

        void SetRandomPos(int p){

	        // ****************** Additional Student Code: Start ************

	        // Put code here to calculate the position, orientation of 
            // particles[p]. Feel free to use the random.NextDouble() function. 
	        // It might be helpful to use boundaries defined in the
	        // Map.cs file (e.g. map.minX)

            double mapXRange = map.maxX - map.minX;
            double mapYRange = map.maxY - map.minY;

            // set position and orientation of the given particle
            
            particles[p].x = random.NextDouble() * mapXRange + map.minX;
            particles[p].y = random.NextDouble() * mapYRange + map.minY;
            double pT = random.NextDouble() * 2 * Math.PI;
            particles[p].t = BoundAngle(pT);

         

            // ****************** Additional Student Code: End   ************
        }




        // For particle p, this function will select a start predefined position. 
        void SetStartPos(int p){
	        particles[p].x = 0; //initialX;
	        particles[p].y = 0; //initialY;
            particles[p].t = 1.57; //initialT;
        }



        // Random number generator with gaussian distribution
        // Often random guassian numbers are used in particle filters. This
        // function might help.

        double RandomGaussian()
        {
	        double U1, U2, V1=0, V2;
	        double S = 2.0;
	        while(S >= 1.0) 
	        {
		        U1 = random.NextDouble();
                U2 = random.NextDouble();
		        V1 = 2.0*U1-1.0;
		        V2 = 2.0*U2-1.0;
		        S = Math.Pow(V1,2) + Math.Pow(V2,2);
	        }
	        double gauss = V1*Math.Sqrt((-2.0*Math.Log(S))/S);
	        return gauss;
        }



        // Get the sign of a number
        double Sgn(double a)
        {
	        if (a>0)
                return 1.0;
	        else if (a<0)
                return -1.0;
	        else
                return 0.0;
        }

        #endregion

    }
}
