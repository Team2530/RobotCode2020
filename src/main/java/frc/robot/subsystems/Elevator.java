/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorLimitSwitches;
import frc.robot.Constants.ElevatorMotors;

public class Elevator extends SubsystemBase {

  private static TalonSRX motor_Left = new TalonSRX(Constants.motor_Elevator_Left_Port);
  private static TalonSRX motor_Right = new TalonSRX(Constants.motor_Elevator_Right_Port);

  // private static Encoder encoder_Left_Leadscrew = new
  // Encoder(Constants.encoder_Left_Leadscrew_Ports[0],Constants.encoder_Left_Leadscrew_Ports[1]);
  // private static Encoder encoder_Right_Leadscrew = new
  // Encoder(Constants.encoder_Right_Leadscrew_Ports[0],Constants.encoder_Right_Leadscrew_Ports[1]);

  private static DigitalInput limit_Switch_Left_Bottom = new DigitalInput(Constants.limit_Switch_Left_Bottom_Port);
  private static DigitalInput limit_Switch_Right_Bottom = new DigitalInput(Constants.limit_Switch_Right_Bottom_Port);

  private static DigitalInput limit_Switch_Left_Top = new DigitalInput(Constants.limit_Switch_Left_Top_Port);
  private static DigitalInput limit_Switch_Right_Top = new DigitalInput(Constants.limit_Switch_Right_Top_Port);

  private static DigitalInput limit_Switch_Left_Middle = new DigitalInput(Constants.limit_Switch_Left_Middle_Port);
  private static DigitalInput limit_Switch_Right_Middle = new DigitalInput(Constants.limit_Switch_Right_Middle_Port);

  private static boolean firstCall = true;
  private boolean endGame = false; // dont go above 45 inches if this is false

  /**
   * Creates a new Elevator.
   */
  public Elevator() {

    /* Disable all motors */
    motor_Left.set(ControlMode.PercentOutput, 0);
    motor_Right.set(ControlMode.PercentOutput, 0);

    /* Factory Default all hardware to prevent unexpected behavior */
    motor_Left.configFactoryDefault();
    motor_Right.configFactoryDefault();

    /* Set neutral modes */
    motor_Left.setNeutralMode(NeutralMode.Brake);
    motor_Right.setNeutralMode(NeutralMode.Brake);

    /* Configure the drivetrain's left side Feedback Sensor as a Magnet Encoder */
    motor_Left.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, // Local Feedback Source
        Constants.PID_PRIMARY, // PID Slot for Source [0, 1]
        Constants.kTimeoutMs);
    motor_Right.configRemoteFeedbackFilter(motor_Left.getDeviceID(), // Device ID of Source
        RemoteSensorSource.TalonSRX_SelectedSensor, // Remote Feedback Source
        Constants.REMOTE_1, // Source number [0, 1]
        Constants.kTimeoutMs); // Configuration Timeout
    /*
     * Setup difference signal to be used for turn when performing Drive Straight
     * with encoders
     */

    // Feedback Device of Remote Talon
    motor_Right.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor1, Constants.kTimeoutMs);
    // Mag Encoder of current Talon
    motor_Right.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTimeoutMs);

    /*
     * Difference term calculated by right Talon configured to be selected sensor of
     * turn PID
     */
    motor_Right.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, Constants.PID_PRIMARY, Constants.kTimeoutMs);

    motor_Right.configSelectedFeedbackCoefficient(
        0.5*((double) Constants.DROP_IN_DISTANCE_PER_REVOLUTION), // Coefficient
        Constants.PID_PRIMARY, // PID Slot of Source
        Constants.kTimeoutMs); // Configuration Timeout

    // Inverting Motors and Encoders
    motor_Left.setInverted(false);
    motor_Left.setSensorPhase(true);
    motor_Right.setInverted(false);
    motor_Right.setSensorPhase(true);

    /* Set status frame periods */
    motor_Right.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
    motor_Right.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
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
    SmartDashboard.putNumber("Left Elevator Encoder", motor_Left.getSelectedSensorPosition(1));
    //SmartDashboard.putNumber("Right Elevator Encoder", motor_Right.getSelectedSensorPosition(1));
    //SmartDashboard.putNumber("Right Elevator Encoder2", motor_Right.getSelectedSensorPosition(0));
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

  // @Deprecated
  public void setMotorPower(final ElevatorMotors id, final double speed) {
    switch (id) {

    case Left:
      if (limit_Switch_Left_Top.get()) { // if we are at the very top

        // only go down
        if (speed <= 0) {
          motor_Left.set(ControlMode.PercentOutput, speed);
        } else {
          motor_Left.set(ControlMode.PercentOutput, 0);
        }

      } else if (limit_Switch_Left_Bottom.get()) { // if we are at the bottom

        // only go up
        if (speed >= 0) {
          motor_Left.set(ControlMode.PercentOutput, speed);
        } else {
          motor_Left.set(ControlMode.PercentOutput, 0);
        }

      } else if (limit_Switch_Left_Middle.get() && !endGame) { // if we are at the middle/top and not endgame

        // only go down
        if (speed <= 0) {
          motor_Left.set(ControlMode.PercentOutput, speed);
        } else {
          motor_Left.set(ControlMode.PercentOutput, 0);
        }

      } else { // doesnt matter direction

        motor_Left.set(ControlMode.PercentOutput, speed);

      }

      return;

    case Right:
      if (limit_Switch_Right_Top.get()) { // if we are at the very top

        // only go down
        if (speed <= 0) {
          motor_Right.set(ControlMode.PercentOutput, speed);
        } else {
          motor_Right.set(ControlMode.PercentOutput, 0);
        }

      } else if (limit_Switch_Right_Bottom.get()) { // if we are at the bottom

        // only go up
        if (speed >= 0) {
          motor_Right.set(ControlMode.PercentOutput, speed);
        } else {
          motor_Right.set(ControlMode.PercentOutput, 0);
        }

      } else if (limit_Switch_Right_Middle.get() && !endGame) { // if we are at the middle/top and not endgame

        // only go down
        if (speed <= 0) {
          motor_Right.set(ControlMode.PercentOutput, speed);
        } else {
          motor_Right.set(ControlMode.PercentOutput, 0);
        }

      } else { // doesnt matter direction

        motor_Right.set(ControlMode.PercentOutput, speed);

      }
      return;

    default:
      return;
    }
  }

  public void setLeftPowerUp() {
    // setMotorPower(ElevatorMotors.Left, 0.5);
    motor_Left.set(ControlMode.PercentOutput, 0.5);
  }

  public void setLeftPowerDown() {
    // setMotorPower(ElevatorMotors.Left, -0.5);
    motor_Left.set(ControlMode.PercentOutput, -0.5);
  }

  public void setRightPowerUp() {
    // setMotorPower(ElevatorMotors.Right, 0.5);
    motor_Right.set(ControlMode.PercentOutput, 0.5);
  }

  public void setRightPowerDown() {
    // setMotorPower(ElevatorMotors.Right, -0.5);
    motor_Right.set(ControlMode.PercentOutput, -0.5);
  }

  public void setPowerUp() {
    setHeight(0.5, 50);
  }

  public void setPowerDown() {
    setHeight(-0.5, 50);
  }

  public double getAngle() {
    /**
     * pusdo code angle = arctan(getHeight()/bottomLeg)
     */

    // double degrees = (radians * 180)/Math.PI;

    return Math.atan2(getParallelHeight(), Constants.bottomLeg);
  }

  public double getFloorHeight() {
    /**
     * pusdo code cant really do this until i know more specs of elevator from
     * hardware return +- from level i think would be easiest for getAngle()
     * 
     * * any gearing?
     * 
     * encoder pos/magic number (1024) = number of turns?
     * 
     * number of turns/turns per inch = inches traveled
     * 
     * if you just set enocder to 0 where you want inches to be 0 then never have to
     * reset will just tell you inches from that point need to figure out how to
     * reset at that 0 point
     * 
     * @PETER THIS MATH SHOULD HAVE BEEN COMPLETED IN THE INITALIZATION
     * 
     */

    double numberOfTurns = getEncoder();

    return numberOfTurns / Constants.leadscrewDistancePerRotation;
  }

  public double getParallelHeight() {
    return getFloorHeight() + Constants.pivotHeight;
  }

  public void setHeight(double power, double height) {
    /*
     * Example 2 - Lift Mechanism Consider a lifting mechanism composed of two
     * closed-loops (one for each side) and no mechanical linkage between them. In
     * other words, the left and right side each have a unique motor controller and
     * sensor. The goal in this circumstance is to closed-loop the elevation while
     * keeping the left and right side reasonably synchronized.
     * 
     * This can be accomplished by using the sum of each side as the elevator
     * height, and the difference as the level deviation between the left and right,
     * which must be kept near zero.
     * 
     * Aux PID[1] can then be used to apply a corrective difference component
     * (adding to one side and subtracting from the other) to maintain a synchronous
     * left and right position, while employing Position/Velocity/Motion-Magic to
     * the primary axis of control (the elevator height).
     */
    
    if(firstCall){
      motor_Left.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
      firstCall = false;
    } // ! I think this should work I not sure how the f term works though
    double target_sensorUnits = power * Constants.ENCODER_TICKS_PER_REVOLUTION*Constants.leadscrewDistancePerRotation*height;
    motor_Left.set(ControlMode.Position, target_sensorUnits, DemandType.AuxPID,0);
    motor_Right.follow(motor_Left, FollowerType.AuxOutput1);
  }

  public void Stop() {
    for (final ElevatorMotors motor : ElevatorMotors.values()) {
      setMotorPower(motor, 0);
    }
  }

  public boolean getLimitSwitchValue(final ElevatorLimitSwitches id) { // * true = pressed, false = not pressed
    switch (id) { // !make sure limit switches are wired correctly

    case LeftBottom:
      return limit_Switch_Left_Bottom.get();

    case RightBottom:
      return limit_Switch_Right_Bottom.get();

    case LeftTop:
      return limit_Switch_Left_Top.get();

    case RightTop:
      return limit_Switch_Right_Top.get();

    case LeftMiddle:
      return limit_Switch_Left_Middle.get();

    case RightMiddle:
      return limit_Switch_Right_Middle.get();

    default:
      DriverStation.reportWarning("Elevator.getLimitSwtichValue default case", true);
      return false; // something went wrong somehow

    }
  }

  public void activateEndGame() {
    endGame = true;
  }

  public boolean getEndgame() {
    return endGame;
  }

}
