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
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
// import edu.wpi.first.wpilibj.SPI.Port;
// import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase {
  // private static final Port i2c_port_id = null;

  // -------------------- Motors -------------------- \\
  // Left Motors
  private static WPI_VictorSPX motor_Front_Left = new WPI_VictorSPX(Constants.motor_Front_Left_Port);
  private static WPI_VictorSPX motor_Back_Left = new WPI_VictorSPX(Constants.motor_Back_Left_Port);

  // Right Motors
  private static WPI_VictorSPX motor_Back_Right = new WPI_VictorSPX(Constants.motor_Back_Right_Port);
  private static WPI_VictorSPX motor_Front_Right = new WPI_VictorSPX(Constants.motor_Front_Right_Port);

  // -------------------- Encoder Stuff -------------------- \\
  // Values
  // private static double encoder_Left_Value;
  // private static double encoder_Right_Value;

  // Rates
  // private static double encoder_Left_Rate;
  // private static double encoder_Right_Rate;

  // Encoders
  private static Encoder encoder_Left = new Encoder(Constants.encoder_Left_Ports[0], Constants.encoder_Left_Ports[1]);
  private static Encoder encoder_Right = new Encoder(Constants.encoder_Right_Ports[0],
      Constants.encoder_Right_Ports[1]);

  // private static boolean isEnabled;

  private static SpeedControllerGroup drive_left;
  private static SpeedControllerGroup drive_right = new SpeedControllerGroup(motor_Front_Right, motor_Back_Right);

  private static PIDController pid_left = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  private static PIDController pid_right = new PIDController(Constants.kP, Constants.kI, Constants.kD);

  private static DifferentialDrive robotDrive;

  private final DifferentialDriveKinematics m_kinematics;

  private final DifferentialDriveOdometry m_odometry;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV);

  // Create a voltage constraint to ensure we don't accelerate too fast
  private DifferentialDriveVoltageConstraint autoVoltageConstraint;
  double integral, derivative, previous_error, setpoint, error = 0;

  TrajectoryConfig config;

  // PIDController
  // private static FeedbackDevice encoder_Left = new
  // FeedbackDevice(FeedbackDevice.QuadEncoder);
  // encoder_Left = (FeedbackDevice.QuadEncoder);
  // encoder_Right = (FeedbackDevice.QuadEncoder);

  // motor_Back_Left.setFeedbackDevice(encoder_Left);
  // encoder_Left.configEncoderCodesPerRev(1024); //? idk magic number

  public static AHRS ahrs = new AHRS();

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {

    resetEncoders();
    ahrs.reset();
    pid_left.setTolerance(Constants.tol);
    pid_right.setTolerance(Constants.tol);
    m_odometry = new DifferentialDriveOdometry(getHeading());

    encoder_Left.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);
    encoder_Right.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);

    encoder_Right.setReverseDirection(false);
    encoder_Left.setReverseDirection(true);

    drive_left = new SpeedControllerGroup(motor_Front_Left, motor_Back_Left);
    drive_left.setInverted(true);

    drive_right = new SpeedControllerGroup(motor_Front_Right, motor_Back_Right);
    drive_right.setInverted(false);

    // drive_right.setInverted(false);
    // drive_left.setInverted(true);

    robotDrive = new DifferentialDrive(drive_left, drive_right);
    m_kinematics = new DifferentialDriveKinematics(Constants.WHEEL_DISTANCE);
    autoVoltageConstraint = new DifferentialDriveVoltageConstraint(m_feedforward, m_kinematics, 11);

    // robotDrive.setSafetyEnabled(false);
    // THis set Trajectory configuration
    config = new TrajectoryConfig(Constants.MAX_DRIVE_SPEED, Constants.maxAccelerationMetersPerSecondSq);

  }

  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public PIDController getLeftController() {
    return pid_left;
  }

  public PIDController getRightController() {
    return pid_right;
  }

  public SimpleMotorFeedforward getFeedForward() {
    return m_feedforward;
  }

  public TrajectoryConfig getTrajConfig() {
    return config;
  }

  /**
   * Sets the speed of the left and right side motors somehow
   * 
   * @param speeds use constructor of DifferentialDriveWheelSpeeds to set left m/s
   *               and right m/s
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = pid_left.calculate(encoder_Left.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput = pid_right.calculate(encoder_Right.getRate(), speeds.rightMetersPerSecond);
    drive_left.setVoltage(leftOutput + leftFeedforward);
    drive_right.setVoltage(rightOutput + rightFeedforward);
    // DifferentialDriveWheelSpeeds speeds1 = new DifferentialDriveWheelSpeeds();
  }

  public void resetEncoders() {
    // motor_Front_Left.getSensorCollection().setQuadraturePosition(0, 10);
    // motor_Front_Right.getSensorCollection().setQuadraturePosition(0, 10);

    encoder_Right.reset();
    encoder_Left.reset();
  }

  /**
   * Sets the speed of one motor
   * 
   * @param id    A DriveMotors enum that represents the motor you want to set the
   *              speed of
   * @param speed The speed that you want the motor to go at (-1 to 1)
   */
  public void setSingleMotorPower(final DriveMotors id, final double speed) {
    switch (id) {
      case FL:
        motor_Front_Left.set(speed);
        return;
      case BR:
        motor_Back_Right.set(ControlMode.PercentOutput, speed);
        return;
      case BL:
        motor_Back_Left.set(ControlMode.PercentOutput, speed);
        return;
      case FR:
        motor_Front_Right.set(speed);
        return;
      default:
        return;
    }
  }

  public void stop() {
    robotDrive.stopMotor();
  }

  public double getEncoder() {
    // SmartDashboard.putNumber("Sensor Vel:",
    // motor_Back_Left.getSelectedSensorVelocity(1));
    return (encoder_Left.getDistance() + encoder_Right.getDistance()) / 2;
  }

  public void periodic() {

    // encoder_Left_Value = encoder_Left.getDistance();
    // encoder_Right_Value = encoder_Right.getDistance();
    // encoder_Left_Rate = encoder_Left.getRate();
    // encoder_Right_Rate = encoder_Right.getRate();

    updateOdometry();
    // SmartDashboard.putNumber("Encoder left:", encoder_Left.getDistance());
    // SmartDashboard.putNumber("Encoder right:", encoder_Right.getDistance());
    // SmartDashboard.putNumber("Angle", ahrs.getAngle());
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

  /**
   * Drives the robot with 1 Joystick
   * 
   * @param zRotation
   * @param xSpeed
   */
  public void arcadeDrive(double zRotation, double xSpeed) {
    robotDrive.arcadeDrive(-xSpeed, zRotation);
    robotDrive.setSafetyEnabled(false);
  }

  /**
   * Drives the robot with 2 Joysticks
   * 
   * @param leftSpeed
   * @param rightSpeed
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    robotDrive.tankDrive(leftSpeed, rightSpeed);
    // robotDrive.setSafetyEnabled(false);
  }

  /**
   * Drives the robot with 1 Joystick and makes the robot more controllable at
   * high speeds
   * 
   * @param xSpeed
   * @param zRotation
   * @param quickTurn If true, allows turn-in-place maneuvers
   */
  public void curvatureDrive(double xSpeed, double zRotation, boolean quickTurn) { // I want to test this
    robotDrive.curvatureDrive(-xSpeed, zRotation, quickTurn);
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
    m_odometry.update(getHeading(), encoder_Left.getDistance(), encoder_Right.getDistance());
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
  public boolean alignToTarget(double power, double targetAngle, double targetDistance, double currentAngle,
      double currentDistance) {
    // if (currentDistance > targetDistance - Constants.distanceTolerance
    // || currentDistance < targetDistance + Constants.distanceTolerance
    // || currentAngle > targetAngle - Constants.angleTolerance
    // || currentAngle < targetAngle + Constants.angleTolerance) {
    // this.stop();
    // return true;
    // } else {
    // // timedDrive(power * (currentDistance - targetDistance), power *
    // (currentAngle - targetAngle));
    // arcadeDrive(power * (currentAngle - targetAngle), power * (currentDistance -
    // targetDistance)); //fix not working?
    // return false;
    // }
  
   

    if (Math.abs(currentAngle) < targetAngle + Constants.angleTolerance
        && Math.abs(currentDistance) < targetDistance + Constants.distanceTolerance) { // angle AND distance is correct
          return true;
    } else if(Math.abs(currentAngle) < targetAngle + Constants.angleTolerance) { //angle is correct but distance is not
      //move to make distance in correct range
      //!!! TEMPORARY CHANGE: disabled distance adjustment
      if (Math.abs(currentDistance) < targetDistance + Constants.distanceTolerance) {
        return true;
      } else {
        double distanceSpeed = power * (currentDistance - targetDistance);
        arcadeDrive(0, distanceSpeed);
        return false;
      }

    } else if (Math.abs(currentDistance) < targetDistance + Constants.distanceTolerance) { // distance is correct but
                                                                                           // angle is not
      // move to make angle correct
      double angleSpeed = power * (currentAngle - targetAngle);
      arcadeDrive(0, angleSpeed);
      return false;

    } else { // neither are correct
      // move both
      double angleSpeed = power * (currentAngle - targetAngle);
      double distanceSpeed = power * (currentDistance - targetDistance);
      arcadeDrive(0, angleSpeed/*distanceSpeed*/);
      return false;

    }
     //uhm build
  }

  public Trajectory getAlignToTargetTrajectory(Pose2d pos){
    List<Pose2d> waypoints = new ArrayList<Pose2d>();
    waypoints.add(pos);

    var trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);
    var transformTrajectory = trajectory.relativeTo(m_odometry.getPoseMeters());
    return transformTrajectory;
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(encoder_Left.getRate(), encoder_Right.getRate());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, getHeading());
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    // SmartDashboard.putNumber("Left Volts", leftVolts);
    // SmartDashboard.putNumber("Right Volts", rightVolts);
    drive_left.setVoltage(leftVolts);
    drive_right.setVoltage(rightVolts);
    robotDrive.feed();
  }

  public PIDController getPid_left() {
    return pid_left;
  }

  public PIDController getPid_right() {
    return pid_right;
  }

}
