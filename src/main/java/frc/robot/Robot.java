// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
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
  // private static final String kDefaultAuto = "Default";
  // private static final String kCustomAuto = "My Auto";
  // private String m_autoSelected;
  // private final SendableChooser<String> m_chooser = new SendableChooser<>();

  final int kEncoderCPR = 2048;
  final Double distancePerPalse = 0.13;

  Joystick js1 = new Joystick(0);

   // PWM Channels
   public final int kLeftFrontChannel = 13;
   public final int kLeftRearChannel = 14;
   public final int kRightFrontChannel = 11;
   public final int kRightRearChannel = 12;
   public final int joystickPort = 0;
 
   // Driver Station Ports
   public final int leftStick_X = 0;
   public final int leftStick_Y = 1;
   public final int rightStick_X = 4;
   public final int rightStick_Y = 5;
   public final int trigger_L = 2;
   public final int trigger_R = 3;
   public final int Btn_A = 1;
   public final int Btn_B = 2;
   public final int Btn_X = 3;
   public final int Btn_Y = 4;
   public final int Btn_LB = 5;
   public final int Btn_RB = 6;
   public final int Btn_LS = 9;  
   public final int Btn_RS = 10;

   // Motors
   WPI_TalonFX LF = new WPI_TalonFX(1);
   WPI_TalonFX LR = new WPI_TalonFX(2);
   WPI_TalonFX RF = new WPI_TalonFX(3);
   WPI_TalonFX RR = new WPI_TalonFX(4);
   
   MecanumDrive mDrive = new MecanumDrive(LF, LR, RF, RR);

   MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, gyroAngle);

   AHRS gyro = new AHRS(SPI.Port.kMXP);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    LF.setInverted(true);
    LR.setInverted(true);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
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

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double ySpeed = js1.getRawAxis(leftStick_Y) * 0.6;
    double xSpeed = js1.getRawAxis(leftStick_X) * 0.6;
    double zRotation = js1.getRawAxis(rightStick_X) * 0.4;

    mDrive.driveCartesian(ySpeed, xSpeed, zRotation);

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

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
