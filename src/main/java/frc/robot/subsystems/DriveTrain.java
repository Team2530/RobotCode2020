/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.SPI.Port;
// import edu.wpi.first.wpilibj.controller.PIDController;

import frc.robot.Constants;
import frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase {
  // private static final Port i2c_port_id = null;
  /**
   * Creates a new DriveTrain.
   */
  private static WPI_TalonSRX motor_Front_Left = new WPI_TalonSRX(Constants.motor_Front_Left_Port);

  private static WPI_VictorSPX motor_Back_Left = new WPI_VictorSPX(Constants.motor_Back_Left_Port);

  private static WPI_VictorSPX motor_Back_Right = new WPI_VictorSPX(Constants.motor_Back_Right_Port);
  private static WPI_TalonSRX motor_Front_Right = new WPI_TalonSRX(Constants.motor_Front_Right_Port);

  private static double encoder_Left;
  private static double encoder_Right;
  private static double encoder_Left_Rate;
  private static double encoder_Right_Rate;

  private static SpeedControllerGroup drive_left = new SpeedControllerGroup(motor_Front_Left, motor_Back_Left);
  private static SpeedControllerGroup drive_right = new SpeedControllerGroup(motor_Front_Right, motor_Back_Right);

  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  private static DifferentialDrive robotDrive = new DifferentialDrive(drive_left, drive_right);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.WHEEL_DISTANCE);

  private final DifferentialDriveOdometry m_odometry;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.motor_Front_Left_Port,
      Constants.motor_Front_Right_Port);

  double P, I, D = 1;
  double integral, derivative, previous_error, setpoint, error = 0;

  // PIDController
  // private static Encoder encoder_Left = new
  // Encoder(Constants.encoder_Left_Ports[0],Constants.encoder_Left_Ports[1]);
  // private static Encoder encoder_Right = new
  // Encoder(Constants.encoder_Right_Ports[0],Constants.encoder_Right_Ports[1]);

  // private static FeedbackDevice encoder_Left = new
  // FeedbackDevice(FeedbackDevice.QuadEncoder);
  // encoder_Left = (FeedbackDevice.QuadEncoder);
  // encoder_Right = (FeedbackDevice.QuadEncoder);

  // motor_Back_Left.setFeedbackDevice(encoder_Left);
  // encoder_Left.configEncoderCodesPerRev(1024); //? idk magic number

  public static AHRS ahrs = new AHRS();

  public DriveTrain() {
    resetEncoders();
    ahrs.reset();
    m_leftPIDController.setTolerance(Constants.tol);
    m_rightPIDController.setTolerance(Constants.tol);
    m_odometry = new DifferentialDriveOdometry(getHeading());
    // encoder_Left.setDistancePerPulse(Constants.ENCODER_TICKS_PER_REVOLUTION);
    // encoder_Right.setDistancePerPulse(Constants.ENCODER_TICKS_PER_REVOLUTION);
    drive_right.setInverted(false);
    drive_left.setInverted(true);
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = m_leftPIDController.calculate(encoder_Left_Rate, speeds.leftMetersPerSecond);
    final double rightOutput = m_rightPIDController.calculate(encoder_Right_Rate, speeds.rightMetersPerSecond);
    drive_left.setVoltage(leftOutput + leftFeedforward);
    drive_right.setVoltage(rightOutput + rightFeedforward);
  }

  public void resetEncoders() {
    motor_Front_Left.getSensorCollection().setQuadraturePosition(0, 10);
    motor_Front_Right.getSensorCollection().setQuadraturePosition(0, 10);

    // motor_Back_Right.setEncPosition(0);
    // encoder_Right.reset();
  }

  public void setSingleMotorPower(final DriveMotors id, final double speed) {
    switch (id) {// TODO THESE ARE ARBITRARY
    case FL:
      motor_Front_Left.set(ControlMode.PercentOutput, speed);
      return;
    case BR:
      motor_Back_Right.set(ControlMode.PercentOutput, speed);
      return;
    case BL:
      motor_Back_Left.set(ControlMode.PercentOutput, speed);
      return;
    case FR:
      motor_Front_Right.set(ControlMode.PercentOutput, speed);
      return;
    default:
      return;
    }
  }

  public void stop() {
    robotDrive.stopMotor();
  }

  public double getEncoder() {
    SmartDashboard.putNumber("Sensor Vel:", motor_Back_Left.getSelectedSensorVelocity(1));
    return motor_Back_Left.getSelectedSensorPosition(1);
  }

  public void periodic() {
    encoder_Left = motor_Front_Left.getSelectedSensorPosition(1) / (Constants.DISTANCE_PER_PULSE);
    encoder_Right = motor_Front_Right.getSelectedSensorPosition(1) / (Constants.DISTANCE_PER_PULSE);
    encoder_Left_Rate = motor_Front_Left.getSelectedSensorVelocity(1) / (Constants.DISTANCE_PER_PULSE);
    encoder_Right_Rate = motor_Front_Right.getSelectedSensorVelocity(1) / (Constants.DISTANCE_PER_PULSE);

    SmartDashboard.putNumber("Encoder left:", encoder_Left);
    SmartDashboard.putNumber("Encoder right:", encoder_Right);
    SmartDashboard.putNumber("Angle", ahrs.getAngle());
    // This method will be called once per scheduler run
  }

  public void setPower(double output) {
    // robotDrive.
    robotDrive.tankDrive(output, output);
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-ahrs.getAngle());
  }

  public double getAngle() {
    return -ahrs.getAngle();
  }

  public void arcadeDrive(double zRotation, double xSpeed) {
    robotDrive.arcadeDrive(-xSpeed, zRotation);
    robotDrive.setSafetyEnabled(false);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    robotDrive.tankDrive(leftSpeed, rightSpeed);
    robotDrive.setSafetyEnabled(false);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
   */
  @SuppressWarnings("ParameterName")
  public void timedDrive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  public void updateOdometry() {
    m_odometry.update(getHeading(), encoder_Left, encoder_Right);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param power           Starting motor power in m/s.
   * @param targetAngle     Target Angle in degrees.
   * @param targetDistance  Target Distance in m
   * @param currentAngle    Current Angle in degrees.
   * @param currentDistance Current Distance in m
   */
  public void alignToTarget(double power, double targetAngle, double targetDistance, double currentAngle,
      double currentDistance) {
    if (currentDistance > targetDistance - Constants.distanceTolerance
        || currentDistance < targetDistance + Constants.distanceTolerance
        || currentAngle > targetAngle - Constants.angleTolerance
        || currentAngle < targetAngle + Constants.angleTolerance) {
      this.stop();
    } else {
      timedDrive(power*(currentDistance-targetDistance), position[0]);
    }

  }
}
