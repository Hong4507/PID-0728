// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

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

   AHRS m_gyro = new AHRS(SPI.Port.kMXP);

   MecanumDriveKinematics kinematics = null; // Not understand, remember to fill it.

   MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, m_gyro.getRotation2d());

   // Encoders
   //  distancePerPulse = gearRatio * wheelPerimeter / EncoderCPR
   //  distance = motor.getSelectedSensorPosition() * distancePerPulse
   //  velocity = motor.getSelectedSensorVelocity() * distancePerPulse
   final double kEncoderCPR = 2048;
   final double kDistancePerPulse = 0.13;
   double leftDistance = LF.getSelectedSensorPosition();
   double rightDistance = RF.getSelectedSensorPosition();
   double currentPosition = (leftDistance + rightDistance) / 2;

   // Motor speeds
   double speedFL = LF.getSelectedSensorVelocity() * kDistancePerPulse;
   double speedFR = RF.getSelectedSensorVelocity() * kDistancePerPulse;
   double speedRL = LR.getSelectedSensorVelocity() * kDistancePerPulse;
   double speedRR = RR.getSelectedSensorVelocity() * kDistancePerPulse;

   MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(speedFL, speedFR, speedRL, speedRR);

   // Timers
   double elapsedTime;
   double startTime;
   double currentTime;

   // PID
   double kP = 0.5; 
   double targetPosition;
   double error;
   double output;

   double currentX = odometry.getPoseMeters().getX();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // invert motors and encoders
    LF.setInverted(true);
    LR.setInverted(true);
    LF.setSensorPhase(true);
    LR.setSensorPhase(true);

    // reset encoders
    resetEncoders();

    // set motors' deadband
    mDrive.setDeadband(0.05);
  }

  @Override
  public void robotPeriodic() {
    // make odometry updates periodic
    odometry.update(m_gyro.getRotation2d(), wheelSpeeds);
  }

  @Override
  public void autonomousInit() {
    // reset encoders again(for sure)
    resetEncoders();

    // set startTime
    startTime = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // get elapsed time
    currentTime = Timer.getFPGATimestamp();
    elapsedTime = currentTime - startTime;

    // set move distance
    targetPosition = 10;
    error = targetPosition - currentX;

    output = kP * error;
    
    mDrive.driveCartesian(output, 0, 0);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // mecanum drive
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

  // reset encoders to zero
  public void resetEncoders() {
    // it will report an error if timeout(10s)
    LF.setSelectedSensorPosition(0, 0, 10);
    LR.setSelectedSensorPosition(0, 0, 10);
    RF.setSelectedSensorPosition(0, 0, 10);
    RR.setSelectedSensorPosition(0, 0, 10);
  }
}
