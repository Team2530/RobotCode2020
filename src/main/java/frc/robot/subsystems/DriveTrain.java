/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
// import edu.wpi.first.wpilibj.SPI.Port;
// import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends PIDSubsystem {
  // private static final Port i2c_port_id = null;
  /**
   * Creates a new DriveTrain.
   */
  private static TalonSRX motor_Front_Left = new TalonSRX(Constants.motor_Front_Left_Port);
  
  private static VictorSPX motor_Back_Left = new VictorSPX(Constants.motor_Back_Left_Port);


  private static VictorSPX motor_Back_Right = new VictorSPX(Constants.motor_Back_Right_Port);
  private static TalonSRX motor_Front_Right = new TalonSRX(Constants.motor_Front_Right_Port);

  public static double encoder_Left;
  public static double encoder_Right;

  //public static SpeedControllerGroup drive_left = new SpeedControllerGroup(motor_Front_Left, motor_Back_Left);
  //new SpeedControllerGroup()

  double P, I, D = 1;
  double integral, derivative, previous_error, setpoint, error = 0;
  DifferentialDrive robotDrive;
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

  public static AHRS ahrs = new AHRS();// ! NEED A PORT ID
  public static PIDController pid = new PIDController(Constants.kP, Constants.kI, Constants.kD);

  public DriveTrain() {
    super(pid);
    getController().setTolerance(Constants.tol);
    //encoder_Left.setDistancePerPulse(Constants.ENCODER_TICKS_PER_REVOLUTION);
    //encoder_Right.setDistancePerPulse(Constants.ENCODER_TICKS_PER_REVOLUTION);
    setSetpoint(Constants.setPoint);
    motor_Back_Right.setInverted(false);
    motor_Front_Right.setInverted(false);
    motor_Back_Left.setInverted(true);
    motor_Front_Left.setInverted(true);
    resetEncoders();
    ahrs.reset();
  }

  public void setSetpoint(int setpoint) {
    this.setpoint = setpoint;
  }

  public void resetEncoders() {
    motor_Front_Left.setSelectedSensorPosition(0,0,10);
    motor_Front_Right.setSelectedSensorPosition(0,0,10);
    // motor_Back_Right.setEncPosition(0);
    // encoder_Right.reset();
  }

  public void setMotorPower(final DriveMotors id, final double speed) {
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

  public void Stop() {
    for (final DriveMotors motor : DriveMotors.values()) {
      setMotorPower(motor, 0);
    }
  }
  // TODO:complete drive for Distance
  public void driveDistance(double distance, double power)// Distance is inches and set power to negative to go                                                       // backwards
  {
    double encoderdistance = Constants.ENCODER_TICKS_PER_REVOLUTION * Math.PI * Math.pow(Constants.WHEEL_RADIUS, 2)
        * distance;

    resetEncoders();
    // starts all motors at starting speed
    for (final DriveMotors motor : DriveMotors.values()) {
      setMotorPower(motor, power);
    }

    /*
     * while(getGreatestEncoder()<encoderdistance){ if() }
     */
  }
  public double getEncoder() {
    // motor_Back_Left.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition,
    // 0, 0);
    // return motor_Back_Left.getSelectedSensorPosition(0);

    SmartDashboard.putNumber("Sensor Vel:", motor_Back_Left.getSelectedSensorVelocity(1));
    return motor_Back_Left.getSelectedSensorPosition(1);
  }
  @Override
  public void periodic() {
    encoder_Left = motor_Front_Left.getSelectedSensorPosition(1);
    encoder_Right = motor_Front_Right.getSelectedSensorPosition(1);
    SmartDashboard.putNumber("Encoder left:", encoder_Left);
    SmartDashboard.putNumber("Encoder right:", encoder_Right);
    SmartDashboard.putNumber("Angle", ahrs.getAngle());
    // This method will be called once per scheduler run
  }

  public void rotateRight(double speed) {
    setMotorPower(DriveMotors.FR, -speed);
    setMotorPower(DriveMotors.BR, -speed);

    setMotorPower(DriveMotors.FL, speed);
    setMotorPower(DriveMotors.BL, speed);
  }

  public void rotateLeft(double speed) {
    setMotorPower(DriveMotors.FR, speed);
    setMotorPower(DriveMotors.BR, speed);

    setMotorPower(DriveMotors.FL, speed);
    setMotorPower(DriveMotors.BL, speed);
  }
  public void setPower(double output,double setpoint){
    //robotDrive.
    useOutput(output, setpoint);
  }
  @Override
  protected void useOutput(double output, double setpoint) {
    setMotorPower(DriveMotors.FR, output);
    setMotorPower(DriveMotors.BR, output);

    setMotorPower(DriveMotors.FL, output);
    setMotorPower(DriveMotors.BL, output);
  }

  @Override
  protected double getMeasurement() {
    // TODO Auto-generated method stub
    return 0;
  }
  public double getHeading(){
    return ahrs.getAngle();
  }
}
