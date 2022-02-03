// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
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
  private int kmotor1Channel = 4;
  private int kmotor2Channel = 5;
  private int kmotor3Channel = 6;
  private WPI_TalonFX motor1;
  private WPI_TalonFX motor2;
  private WPI_TalonFX motor3;
  private DoubleSolenoid climbDoublePCM;
  private AHRS ahrs;
  private Timer auto_timer;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() { 

    auto_timer = new Timer();
    ahrs = new AHRS(SPI.Port.kMXP); 
    
    // Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5 units per second
    filterx = new SlewRateLimiter(Preferences.getDouble("XRateLimit", 0.5));
    filtery = new SlewRateLimiter(Preferences.getDouble("yRateLimit", 0.5));
    filterz = new SlewRateLimiter(Preferences.getDouble("ZRateLimit", 0.5));

    int kFrontLeftChannel = 3;
    int kFrontRightChannel = 0;
    int kRearRightChannel = 1;
    int kRearLeftChannel = 2;
    int kJoystickControlChannel = 2;
    int kJoystickMovementChannel = 1;
    int kclimbFowardChannel = 1;
    int kclimbBackwardChannel=2;
    climbDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, kclimbFowardChannel, kclimbBackwardChannel);
    
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    WPI_TalonFX frontLeft = new WPI_TalonFX (kFrontLeftChannel);
    WPI_TalonFX rearLeft = new WPI_TalonFX (kRearLeftChannel);
    WPI_TalonFX frontRight = new WPI_TalonFX (kFrontRightChannel);
    WPI_TalonFX rearRight = new  WPI_TalonFX (kRearRightChannel);
    motor1 = new  WPI_TalonFX (kmotor1Channel);
    motor2 = new  WPI_TalonFX (kmotor2Channel);
    motor3 = new  WPI_TalonFX (kmotor3Channel);

    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    frontRight.setInverted(true);
    rearRight.setInverted(true);

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
  public void robotPeriodic() {}

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
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    boolean enter = true;
    
    switch (autoState) {
      case 1:
      if (enter == true) 
      {
        auto_timer.reset();
        enter = false;
      }
        // Move Forward 
        m_robotDrive.driveCartesian(0,1,Preferences.getDouble("MovingForwardSpeed", 1),0);
        // If hit wall then goto state 2
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
        enter = false;
      }
       // Stop (at the hub)
       m_robotDrive.driveCartesian(0,0,0);
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
        enter = false;
      }
      // Raise Crane to the height of the hub
      motor1.set(Preferences.getDouble("Motor1ForwardSpeed", 1.0));
      // If crane is the height of the hub goto state 4
      if (auto_timer.get() >1.5)
        { 
          autoState = 4;
          enter = true;
        }
        break;
      case 4:
      if (enter == true) 
      {
        auto_timer.reset();
        enter = false;
      }
      // Push Ball
       motor2.set(Preferences.getDouble("Motor2ForwardSpeed", 1.0));
      // If it has been 1 second then goto state 5
      if (auto_timer.get() > Preferences.getDouble("PushBallTime", 1.0))
        { 
          autoState = 5;
          enter = true;
        }
        break;
      case 5:
      if (enter == true) 
      {
        auto_timer.reset();
        enter = false;
      }
      // Lower Crane to the bottom
      motor1.set(Preferences.getDouble("Motor1BackwardSpeed", -1.0));
      // If the crane at the bottom goto state 6
      if (auto_timer.get() > Preferences.getDouble("CraneDownTime", 1.0))
        { 
          autoState = 6;
          enter = true;
        }
        break;
      case 6:
     if (enter == true) 
      {
        auto_timer.reset();
        enter = false;
      }
      //Turn 180 degrees
       m_robotDrive.driveCartesian(0,0, Preferences.getDouble("Turning180speed", -1));
      //If robot has turned 180 degrees goto stat 7
      if (auto_timer.get() > Preferences.getDouble("Turning180Time", 1.0))
        { 
          autoState = 7;
          enter = true;
        }
        break;
      case 7:
      if (enter == true) 
      {
        auto_timer.reset();
        enter = false;
      }
        // Move foward
       m_robotDrive.driveCartesian(0, Preferences.getDouble("MovingForwardSpeed", 1),0);
        // if we have the ball goto 8
        if (auto_timer.get() >1)
        { 
          autoState = 8;
          enter = true;
        }
        break;
        case 8:
        if (enter == true) 
      {
        auto_timer.reset();
        enter = false;
      }
      // Stop at the ball
       m_robotDrive.driveCartesian(0,0,0);
      // if has been 1 second goto 9
      if (auto_timer.get() > Preferences.getDouble("StopAtTheBallTime", 1.0))
        { 
          autoState = 9;
          enter = true;
        }
        break;
        case 9:
        if (enter == true) 
      {
        auto_timer.reset();
        enter = false;
      }
      // Pick up ball
       motor3.set(Preferences.getDouble("Motor3ForwardSpeed", 1.0));
      // if 4 seconds goto 10
      if (auto_timer.get() > Preferences.getDouble("PickUpBallTime", 1.0))
        { 
          autoState = 10;
          enter = true;
        }
        break;
        case 10:
        if (enter == true) 
      {
        auto_timer.reset();
        enter = false;
      }
        // Turn 180 degrees
         m_robotDrive.driveCartesian(0,0,Preferences.getDouble("Turning180speed", -1));
        //if robot has turned 180 degrees
        if (auto_timer.get() > Preferences.getDouble("Turning180Time", 1.0))
        { 
          autoState = 11;
          enter = true;
        }
        break;
      case 11:
      if (enter == true) 
      {
        auto_timer.reset();
        enter = false;
      }
        // Move Forward
        m_robotDrive.driveCartesian(0, Preferences.getDouble("MovingForwardSpeed", 1),0);
        // if hit the hub
        if (auto_timer.get() >1)
        { 
          autoState = 12;
        }
        break;
        case 12:
        if (enter == true) 
      {
        auto_timer.reset();
        enter = false;
      }
        //Stop at the hub
      m_robotDrive.driveCartesian(0,0,0);
        // if robot has stopped goto 13
        if (auto_timer.get() >.1)
        { 
          autoState = 13;
          enter = true;
        }
        break;
      case 13:
      if (enter == true) 
      {
        auto_timer.reset();
        enter = false;
      }
        // Raise Crane to the height of the hub
      motor1.set(Preferences.getDouble("Motor1ForwardSpeed", 1.0)); 
        // if the crane height of the hub goto 14
        if (auto_timer.get() > Preferences.getDouble("CraneUpTime", 1.0))
        { 
          autoState = 14;
          enter = true;
        }
        break;
      case 14:
      if (enter == true) 
      {
        auto_timer.reset();
        enter = false;
      }
        // Push ball
        motor1.set(Preferences.getDouble("Motor1BackwardSpeed", -1.0)); 
        // if it has been 1 second goto 15
        if (auto_timer.get() > Preferences.getDouble("PushBallTime", 1.0))
        { 
          autoState = 15;
          enter = true;
        }
        break;
      case 15:
      if (enter == true) 
      {
        auto_timer.reset();
        enter = false;
      }
        // Lower Crane
        motor1.set(Preferences.getDouble("Motor1BackwardSpeed", -1.0)); 
        // if crane is at the bottom goto 16
        if (auto_timer.get() > Preferences.getDouble("CraneDownTime", 1.0))
        { 
          autoState = 16;
          enter = true;
        }
        break;
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
     m_robotDrive.driveCartesian(filterx.calculate(m_stick.getX()) ,filtery.calculate(-m_stick.getY()),filterz.calculate(m_stick.getZ()), ahrs.getAngle());

     if (c_stick.getAButtonPressed())
     {
       motor1.set(Preferences.getDouble("Motor1ForwardSpeed", 1.0));
     }
     else if (c_stick.getBButtonPressed())
     {
       motor1.set(Preferences.getDouble("Motor1BackwardSpeed", -1.0));
       
     }
     else 
     {
       motor1.set(0);
     }
      
    if (c_stick.getYButtonPressed())
     {
       motor2.set(Preferences.getDouble("Motor2ForwardSpeed", 1.0));
     }
     else if (c_stick.getXButtonPressed())
     {
       motor2.set(Preferences.getDouble("Motor2BackwardSpeed", -1.0));
     }
     else 
     {
       motor2.set(0);
     }

     if (c_stick.getRightBumperPressed())
     {
       motor3.set (Preferences.getDouble("Motor3ForwardSpeed", 1.0));
     }
     else if (c_stick.getLeftBumperPressed())
     {
       motor3.set (Preferences.getDouble("Motor3BackwardSpeed", -1.0));
     }
     else 
     {
       motor3.set(0);
     } 
     
    if (c_stick.getRightStickButtonPressed())
    {
      climbDoublePCM.set(kForward);
    } 
    else if (c_stick.getLeftStickButtonPressed())
    {
      climbDoublePCM.set(kReverse);
    }
    else
    {
      climbDoublePCM.set(kOff);
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
