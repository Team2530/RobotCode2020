/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorLimitSwitches;
import frc.robot.Constants.ElevatorMotors;

public class Elevator extends SubsystemBase {

  private static TalonSRX motor_Left = new TalonSRX(Constants.motor_Left_Leadscrew_Port);
  private static TalonSRX motor_Right = new TalonSRX(Constants.motor_Right_Leadscrew_Port);

  // private static Encoder encoder_Left_Leadscrew = new
  // Encoder(Constants.encoder_Left_Leadscrew_Ports[0],Constants.encoder_Left_Leadscrew_Ports[1]);
  // private static Encoder encoder_Right_Leadscrew = new
  // Encoder(Constants.encoder_Right_Leadscrew_Ports[0],Constants.encoder_Right_Leadscrew_Ports[1]);

  private static DigitalInput limit_Switch_Left_Leadscrew = new DigitalInput(
      Constants.limit_Switch_Left_Leadscrew_Port);
  private static DigitalInput limit_Switch_Right_Leadscrew = new DigitalInput(
      Constants.limit_Switch_Right_Leadscrew_Port);

  private static DigitalInput limit_Switch_Left_Pulley = new DigitalInput(Constants.limit_Switch_Left_Pulley_Port);
  private static DigitalInput limit_Switch_Right_Pulley = new DigitalInput(Constants.limit_Switch_Right_Pulley_Port);

  /**
   * Creates a new Elevator.
   */
  public Elevator() {
    /* Configure the drivetrain's left side Feedback Sensor as a Magnet Encoder */
    motor_Left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, // Local Feedback Source
        Constants.PID_PRIMARY, // PID Slot for Source [0, 1]
        Constants.kTimeoutMs);
    motor_Right.configRemoteFeedbackFilter(motor_Left.getDeviceID(), // Device ID of Source
        RemoteSensorSource.TalonSRX_SelectedSensor, // Remote Feedback Source
        Constants.REMOTE_0, // Source number [0, 1]
        Constants.kTimeoutMs); // Configuration Timeout
    /*
     * Setup difference signal to be used for turn when performing Drive Straight
     * with encoders
     */

    // Feedback Device of Remote Talon
    motor_Right.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);
    // Mag Encoder of current Talon
    motor_Right.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kTimeoutMs);

    /*
     * Difference term calculated by right Talon configured to be selected sensor of
     * turn PID
     */
    motor_Right.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, Constants.PID_TURN, Constants.kTimeoutMs);

    motor_Right.configSelectedFeedbackCoefficient(
        ((double) Constants.DROP_IN_DISTANCE_PER_REVOLUTION) / Constants.FLYWHEEL_DISTANCE_PER_REVOLUTION, // Coefficient
        Constants.PID_TURN, // PID Slot of Source
        Constants.kTimeoutMs); // Configuration Timeout

    // Inverting Motors and Encoders
    motor_Left.setInverted(false);
    motor_Left.setSensorPhase(true);
    motor_Right.setInverted(true);
    motor_Right.setSensorPhase(true);

    /* Set status frame periods */
    motor_Right.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
    motor_Right.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
    // Used remotely by right Talon, speed up
    motor_Left.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

    /* Configure neutral deadband */
    motor_Right.configNeutralDeadband(Constants.NeutralDeadband, Constants.kTimeoutMs);
    motor_Left.configNeutralDeadband(Constants.NeutralDeadband, Constants.kTimeoutMs);

    motor_Left.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
    motor_Left.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
    motor_Right.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
    motor_Right.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

    /* FPID Gains for turn servo */
    motor_Right.config_kP(Constants.kSlot_Turning, Constants.kElevator_Gains_Velocit.kP, Constants.kTimeoutMs);
    motor_Right.config_kI(Constants.kSlot_Turning, Constants.kElevator_Gains_Velocit.kI, Constants.kTimeoutMs);
    motor_Right.config_kD(Constants.kSlot_Turning, Constants.kElevator_Gains_Velocit.kD, Constants.kTimeoutMs);
    motor_Right.config_kF(Constants.kSlot_Turning, Constants.kElevator_Gains_Velocit.kF, Constants.kTimeoutMs);
    motor_Right.config_IntegralZone(Constants.kSlot_Turning, Constants.kShooter_Gains_Turning.kIzone,
        Constants.kTimeoutMs);
    motor_Right.configClosedLoopPeakOutput(Constants.kSlot_Turning, Constants.kShooter_Gains_Turning.kPeakOutput,
        Constants.kTimeoutMs);
    motor_Right.configAllowableClosedloopError(Constants.kSlot_Turning, 0, Constants.kTimeoutMs);

    /*
     * 1ms per loop. PID loop can be slowed down if need be. For example, - if
     * sensor updates are too slow - sensor deltas are very small per update, so
     * derivative error never gets large enough to be useful. - sensor movement is
     * very slow causing the derivative error to be near zero.
     */
    int closedLoopTimeMs = 1;
    motor_Right.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
    motor_Right.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

    /*
     * configAuxPIDPolarity(boolean invert, int timeoutMs) false means talon's local
     * output is PID0 + PID1, and other side Talon is PID0 - PID1 true means talon's
     * local output is PID0 - PID1, and other side Talon is PID0 + PID1
     */
    motor_Right.configAuxPIDPolarity(false, Constants.kTimeoutMs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetEncoders() {
    /* Update Quadrature position */
    motor_Left.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
    motor_Right.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
  }

  private int getEncoder() {
    /* Update Quadrature position */
    return ((motor_Left.getSelectedSensorPosition() + motor_Left.getSelectedSensorPosition()) / 2);
  }

  public void setMotorPower(final ElevatorMotors id, final double speed) {
    switch (id) {

    case LL:
      if (limit_Switch_Left_Leadscrew.get() && speed > 0) { // if limit switch is pressed and it wants to go up, dont
        // motor_Left_Leadscrew.set(ControlMode.PercentOutput, 0);
        return;
      } else {
        // motor_Left_Leadscrew.set(ControlMode.PercentOutput, speed);
        return;
      }

    case RL:
      if (limit_Switch_Right_Leadscrew.get() && speed > 0) { // if limit switch is pressed and it wants to go up, dont
        // motor_Right_Leadscrew.set(ControlMode.PercentOutput, 0);
        return;
      } else {
        // motor_Right_Leadscrew.set(ControlMode.PercentOutput, speed);
        return;
      }

    default:
      return;
    }
  }

  // TODO get Angle function return radians
  public double getAngle() {
    /**
     * pusdo code angle = arctan(getHeight()/bottomLeg)
     */

    return 40; // temp
  }

  // TODO get Height function return meters
  public double getHeight() {

    return Constants.leadscrewDistancePerRotation * getHeight();
  }

  // TODO get limelight height return degrees
  public double getLimeLightHeight() {

    return Constants.sensor_Limelight_Height; // temp test value
  }

  public void Stop() {
    for (final ElevatorMotors motor : ElevatorMotors.values()) {
      setMotorPower(motor, 0);
    }
  }

  public boolean getLimitSwitchValue(final ElevatorLimitSwitches id) { // * true = pressed, false = not pressed
    switch (id) { // !make sure limit switches are wired correctly
    case LL:
      return limit_Switch_Left_Leadscrew.get();
    case RL:
      return limit_Switch_Right_Leadscrew.get();
    case LP:
      return limit_Switch_Left_Pulley.get();
    case RP:
      return limit_Switch_Right_Pulley.get();
    default:
      return false; // uhm false? idk
    }
  }

}
