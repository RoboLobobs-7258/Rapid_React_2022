// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
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
  private SlewRateLimiter filterx;
  private SlewRateLimiter filtery;
  private SlewRateLimiter filterz;
  private int autoState;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() { 
    // Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5 units per second
    filterx = new SlewRateLimiter(0.5);
    filtery = new SlewRateLimiter(0.5);
    filterz = new SlewRateLimiter(0.5);
    int kFrontLeftChannel = 1;
    int kFrontRightChannel = 2;
    int kRearRightChannel = 3;
    int kRearLeftChannel = 4;
    int kJoystickChannel = 1;
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
   WPI_TalonFX frontLeft = new WPI_TalonFX (kFrontLeftChannel);
   WPI_TalonFX rearLeft = new WPI_TalonFX (kRearLeftChannel);
   WPI_TalonFX frontRight = new WPI_TalonFX (kFrontRightChannel);
   WPI_TalonFX rearRight = new  WPI_TalonFX (kRearRightChannel);

    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    frontRight.setInverted(true);
    rearRight.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    m_stick = new Joystick(kJoystickChannel);
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

    switch (autoState) {
      case 1:
        // Move Forward 
        // If hit wall then goto state 2
        break;
      case 2:
       // Stop (at the hub)
       // if robot has stopped moving then goto state 3
        break;
      case 3:
      // Raise Crane to the height of the hub
      // If crane is the height of the hub goto state 4
        break;
      case 4:
      // Push Ball
      // If it has been 1 second then goto state 5
        break;
      case 5:
      // Lower Crane to the bottom
      // If the crane at the bottom goto state 6
        break;
      case 6:
      //Turn 180 degrees
      //If robot has turned 180 degrees goto stat 7
        break;
      case 7:
        // Move foward
        // if we have the ball goto 8
        break;
        case 8:
      // Stop at the ball
      // if has been 1 second goto 9
        break;
        case 9:
      // Pick up ball
      // if 4 seconds goto 10
        break;
        case 10:
        // Turn 180 degrees
        //if robot has turned 180 degrees
        break;
      case 11:
        // Move Forward
        // if hit the hub
        break;
      case 12:
        //Stop at the hub
        // if robot has stopped goto 13
        break;
      case 13:
        // Raise Crane to the height of the hub
        // if the crane height of the hub goto 14
        break;
      case 14:
        // Push ball
        // if it has been 1 second goto 15
        break;
      case 15:
        // Lower Crane
        // if crane is at the bottom goto 16
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
     m_robotDrive.driveCartesian(filterx.calculate(m_stick.getX()) ,filtery.calculate(-m_stick.getY()),filterz.calculate(m_stick.getZ()));
    
  }

  /** This function is called once when the robot is disabled. */
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
