// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Imported Libraries  */
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Preferences;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private MecanumDrive m_robotDrive;
  private Joystick m_stick;
  private XboxController c_stick;
  private SlewRateLimiter filterx;
  private SlewRateLimiter filtery;
  private SlewRateLimiter filterz;
  private int autoState;
  /** These are the CAN channels for the pickup, elevator and shooter motors. */
  private int kmotor1Channel = 4;
  private int kmotor2Channel = 5;
  private int kmotor3Channel = 6;
  private WPI_TalonSRX motor1;
  private WPI_VictorSPX motor2;
  private WPI_TalonSRX motor3;
  private DoubleSolenoid climbDoublePCM;

  private Timer auto_timer;
  private DigitalInput Frontsensor;
  private DigitalInput Backsensor; 
  private int elvstate;
  private Solenoid pickPCM;

  private boolean enter;
  private double backwardsTime;

  /** Code to create a Motor Speed tab on the shuffleboard and add adjustment 
   * sliders to adjust the motor speed for the shooter and pickup motors
   * This allows us to make motor speed adjustments without altering the code.
   * Refer to the following information.
   * https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/getting-started/shuffleboard-tour.html
   */
  private ShuffleboardTab tab = Shuffleboard.getTab("Motor Speed");
  private NetworkTableEntry shooterSpeed = tab.add("Shooter Speed", -.8) 
  .withWidget(BuiltInWidgets.kNumberSlider)
  .getEntry();

  private NetworkTableEntry pickUpSpeedf = tab.add("PickUp Speed Forward", .8) 
  .withWidget(BuiltInWidgets.kNumberSlider)
  .getEntry();

  private NetworkTableEntry pickUpSpeedr = tab.add("PickUp Speed Reverse", -.8) 
  .withWidget(BuiltInWidgets.kNumberSlider)
  .getEntry();

  private NetworkTableEntry twistratio = tab.add("Twist", .8) 
  .withWidget(BuiltInWidgets.kNumberSlider)
  .getEntry();

  /** This is a method that is used to modify the joystick value. 
   * We are cubeing the values.  What this does is to make the joystick less senstive 
   * at slower speeds making the robot easier to control but allowing the robot to go at full speed.
   * x is the joystick value
   * y is the power
   * The result is the multiplation of the joystick value by itself three times.
  Example: For a joystick value of .5 or half speed (.5 × .5 × .5 = .125) .125 or 1/8 speed is sent to the drive software.
  For a joystick value of 1 (full speed)  (1 x 1 x 1 = 1) 1 (full speed) is sent to the drive software.
   */
  private double signedPow (double x, double y) {
    double a = Math.pow(x, y);
      return a;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() { 

    auto_timer = new Timer();

    /** The slew rate limiter is used to control how fast the robot can accerate.
     * In short it limits the rate of change of the output.
     * We use this to keep the robot from tipping as we change speeds.
     * Note: The higher the value the slower the acceleration rate.
     */
    filterx = new SlewRateLimiter(Preferences.getDouble("XRateLimit", 1.5));
    filtery = new SlewRateLimiter(Preferences.getDouble("yRateLimit", .8));
    filterz = new SlewRateLimiter(Preferences.getDouble("ZRateLimit", 1.5));
    
  /** This sets up the Digital inputs on the RoboRio to read the ball sensors on the robot. */
    Frontsensor  = new DigitalInput(0);                        
    Backsensor = new DigitalInput(1);

    /** These are the CAN channels for the Falon 500 motors used for the drive system. */
    int kFrontLeftChannel = 3;
    int kFrontRightChannel = 7;
    int kRearRightChannel = 1;
    int kRearLeftChannel = 2;
    /** This sets up the Joy stick channels */
    int kJoystickControlChannel = 1;
    int kJoystickMovementChannel = 0;
    /** This sets the outputs on the pneumatic control module that are used to control
     * the climber solenoid
     */
    int kclimbFowardChannel = 1;
    int kclimbBackwardChannel=2;
    climbDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, kclimbFowardChannel, kclimbBackwardChannel);

    /** This sets the output on the pneumatic control module used to control
     * the pickup in and out solenoid.
     */
    pickPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 3);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    WPI_TalonFX frontLeft = new WPI_TalonFX (kFrontLeftChannel);
    WPI_TalonFX rearLeft = new WPI_TalonFX (kRearLeftChannel);
    WPI_TalonFX frontRight = new WPI_TalonFX (kFrontRightChannel);
    WPI_TalonFX rearRight = new  WPI_TalonFX (kRearRightChannel);
    motor1 = new  WPI_TalonSRX (kmotor1Channel);
    motor2 = new  WPI_VictorSPX (kmotor2Channel);
    motor3 = new  WPI_TalonSRX (kmotor3Channel);

    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
   
    rearRight.setInverted(true);
    frontRight.setInverted(true);

    /** This sets up the drive system. */
    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    m_stick = new Joystick(kJoystickMovementChannel);
    c_stick = new XboxController(kJoystickControlChannel);
  }
  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
   
    /** This sends information to the Smart Dashboatd to indicate system states*/
      SmartDashboard.putNumber("Elevator State  ", elvstate);
      SmartDashboard.putData("Pickup Arm Extended  ", pickPCM);
      SmartDashboard.putData("Climber State  ", climbDoublePCM);
    }
  

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    autoState = 1;
    enter = true;
    backwardsTime = 0;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    switch (autoState) {
      case 1:
      if (enter == true) 
      {
        auto_timer.reset();
        auto_timer.start();
        enter = false;
      }
        // shoot 
        double fSpeed = shooterSpeed.getDouble  (-.7);
         motor1.set(Preferences.getDouble("Motor1ForwardSpeed", fSpeed));
         motor2.set(Preferences.getDouble("Motor2ForwardSpeed", -.4));
         motor3.set(0);
         m_robotDrive.driveCartesian(0, 0,0);

        // If 3 seconds
        if (auto_timer.get() >1)
        { 
          autoState = 2;
          enter = true;
        }
        break;
      case 2:
      if (enter == true) 
      {
        auto_timer.reset();
        auto_timer.start();
        enter = false;
      }
       // setup pickup 
       pickPCM.set(true);
       double pSpeedr = pickUpSpeedr.getDouble  (-.8); 
      motor3.set (Preferences.getDouble("Motor3BackwardSpeed", pSpeedr));
      motor1.set(0);
      motor2.set(0);
      m_robotDrive.driveCartesian(0, 0,0);
       // if robot has stopped moving then goto state 3
       if (auto_timer.get() >.1)
        { 
          autoState = 3;
          enter = true;
        }
        break;
      case 3:
       if (enter == true) 
      {
        auto_timer.reset();
        auto_timer.start();
        enter = false;
      }
      //move backwards 
      m_robotDrive.driveCartesian(.2, 0,0);
      pSpeedr = pickUpSpeedr.getDouble  (-.8); 
      motor3.set (Preferences.getDouble("Motor3BackwardSpeed", pSpeedr));
      motor1.set(0);
      motor2.set(0);
      if (auto_timer.get() >5)
        { 
          autoState = 7;
          enter = true;
        }
      if (Frontsensor.get()==false)
      {
         backwardsTime=auto_timer.get();     
         autoState = 4;
         enter = true;
      }
        
         break;
      case 4:
      if (enter == true) 
      {
        auto_timer.reset();
        auto_timer.start();
        enter = false;
      }
      // pickup ball 
      motor2.set(Preferences.getDouble("Motor2ForwardSpeed", -.5));
      // If it has been 1 second then goto state 5
      motor1.set(0);
      pSpeedr = pickUpSpeedr.getDouble  (-.8); 
      motor3.set (Preferences.getDouble("Motor3BackwardSpeed", pSpeedr));
      m_robotDrive.driveCartesian(0, 0,0);
      if (Backsensor.get()==false)
        { 
          autoState = 5;
          enter = true;
        }
        break;
      case 5:
      if (enter == true) 
      {
        auto_timer.reset();
        auto_timer.start();
        enter = false;
      }
      // move forward
      motor1.set(0);
      motor2.set(0);
      motor3.set(0);
      m_robotDrive.driveCartesian(-.2, 0,0);
      if (auto_timer.get()> backwardsTime -0.5)
        { 
          autoState = 6;
          enter = true;
        }
        break;
      case 6:
     if (enter == true) 
      {
        auto_timer.reset();
        auto_timer.start();
        enter = false;
      }
      //shoot
      fSpeed = shooterSpeed.getDouble  (-.6);
      motor1.set(Preferences.getDouble("Motor1ForwardSpeed", fSpeed));
      motor2.set(Preferences.getDouble("Motor2ForwardSpeed", -.3));
      motor3.set(0);
      m_robotDrive.driveCartesian(0, 0,0);
      if (auto_timer.get() > 3)
        { 
          autoState = 7;
          enter = true;
        }
        break;
      case 7:
      if (enter == true) 
      {
        auto_timer.reset();
        auto_timer.start();
        enter = false;
      }
        // stop 
       motor1.set(0);
       motor2.set(0);
       motor3.set(0);
       m_robotDrive.driveCartesian(0, 0,0);
      
        break;
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
 elvstate = 1;

 
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    /** This code takes the joystick values to control the speed and direction of the drive motors
     * signedPow(joystick value, Power Value) This takes the value of the joystick and cubes it.
     * 
     * Note: The power value must be 3 to make this work correctly.
     * 
     * filter.calculate (calculated joystick value)  This limits the rate of changed
     * or how fast the robot can accelerate or decellerate.
     */
    m_robotDrive.driveCartesian
     (filtery.calculate (0 - signedPow (m_stick.getY(), 3)) ,
      filterx.calculate( signedPow(m_stick.getX(), 3)), 
      filterz.calculate( signedPow((m_stick.getZ()*twistratio.getDouble(.8)), 3)));

     /** This controls the moving the pickup arm in and out
      * The solenoid we are using is spring loaded.
      When the power to the solenoid is off (false) the arm
      will retract.  This also happens when the robot is diabled or the 
      power is turned off.
      */
    if (c_stick.getYButton())
     {pickPCM.set(true);
    
     }
     else if (c_stick.getXButton())
     {pickPCM.set(false);
    
     }

     /** This controls the speed and direction of the pickup motor
      * The arm must be extended (pickPCM.get() == true) for the motor to run.
      We get the speed value from the Shuffleboard (pSpeedf and pSpeedr)
      .8 is the default speed.
      */
     if (c_stick.getRightBumper() && pickPCM.get() == true)
     {
     double pSpeedf = pickUpSpeedf.getDouble  (.8); 
      motor3.set (Preferences.getDouble("Motor3ForwardSpeed", pSpeedf));
     }
     else if (c_stick.getLeftBumper() && pickPCM.get() == true)
     {
      double pSpeedr = pickUpSpeedr.getDouble  (-.8); 
      motor3.set (Preferences.getDouble("Motor3BackwardSpeed", pSpeedr));
     }
     else 
     {
       motor3.set(0);
     } 
     
     /** This code controls the climber cylinder
      * As a safety measure two buttons need to be pressed.
      This solenoid does not have a spring return and will stay
      in the state it is in when the robot is disabled or powered off.
      */
    if (c_stick.getRightStickButton() && c_stick.getBButton())
    {
      climbDoublePCM.set(kForward);
    } 
    else if (c_stick.getLeftStickButton() && c_stick.getBButton())
    {
      climbDoublePCM.set(kReverse);
    }
    else
    {
      climbDoublePCM.set(kOff);
    }

    /** This controls the automatic operation of the elevator to load a picked up ball
     * 
     */
    switch(elvstate)
    {
      case 1:
      motor1.set(0); 
      motor2.set(0);
      if(Frontsensor.get()==false)
      {
        elvstate=2;
      }
      break;
      case 2:
      motor1.set(0); 
      motor2.set(Preferences.getDouble("Motor2ForwardSpeed", -.5));
      if (Backsensor.get()==false)
      {
        elvstate=3;
      }
      break;
      case 3:
      motor1.set(0); 
      motor2.set(0);
      if(c_stick.getAButton())
      {
        elvstate=4;
      }
      break;
      case 4:
      double fSpeed = shooterSpeed.getDouble  (-.8);
       motor1.set(Preferences.getDouble("Motor1ForwardSpeed", fSpeed));
       motor2.set(Preferences.getDouble("Motor2ForwardSpeed", -.5));
      if(c_stick.getAButton()==false)
      {
        elvstate=1;
      }
      break;
    }


    /* if (Frontsensor.get()==true && Backsensor.get() == true && elvstate != 2) 
    {
      elvstate=1;
    }
    else if (Frontsensor.get()==false && Backsensor.get() == true) {
      elvstate=2;
    }
    else if (Frontsensor.get()==true && Backsensor.get() == false) {
      elvstate=3;
    } 
    else if (Frontsensor.get()==false && Backsensor.get() == false) {
      elvstate=4;
    } */
        


/** This controls the shooter motor
 * The motor speed is set by the value set in the Shuffleboard (fSpeed)
 */
    if (c_stick.getAButton())
     {
       double fSpeed = shooterSpeed.getDouble  (-.8);
       motor1.set(Preferences.getDouble("Motor1ForwardSpeed", fSpeed));
     }

     /** This controls the elevator motor
      * It should run when the A button is pressed or the elevator state =2.
      Note: The or state (|| elevator = 2) is currently removed because we
      have a problem with our ball sensors.
      */
     if (c_stick.getAButton())
     {
       motor2.set(Preferences.getDouble("Motor2ForwardSpeed", -.5));
     }
    }

  /** This function is called 
   * once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
