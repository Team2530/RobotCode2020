/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  private static WPI_TalonSRX motor_Left = new WPI_TalonSRX(Constants.motor_Left_FlyWheel_Port);
  private static WPI_TalonSRX motor_Right = new WPI_TalonSRX(Constants.motor_Right_Flywheel_Port);

  DigitalInput laser = new DigitalInput(Constants.laser_switch);

  private static double currentSpeed = 0;
  boolean hasCurrentBall = false;
  int ballCount = 0;
  /** Tracking variables */
  boolean _firstCall = false;
  boolean _state = false;

  // private static Encoder encoder_Left = new
  // Encoder(Constants.encoder_Left_Flywheel_Ports[0],Constants.encoder_Left_Flywheel_Ports[1]);
  // private static Encoder encoder_Right = new
  // Encoder(Constants.encoder_Right_Flywheel_Port[0],Constants.encoder_Right_Flywheel_Port[1]);

  public Shooter() {
    // this.xbox = xbox;
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
    motor_Right.config_kP(Constants.kSlot_Turning, Constants.kShooter_Gains_Velocit.kP, Constants.kTimeoutMs);
    motor_Right.config_kI(Constants.kSlot_Turning, Constants.kShooter_Gains_Velocit.kI, Constants.kTimeoutMs);
    motor_Right.config_kD(Constants.kSlot_Turning, Constants.kShooter_Gains_Velocit.kD, Constants.kTimeoutMs);
    motor_Right.config_kF(Constants.kSlot_Turning, Constants.kShooter_Gains_Velocit.kF, Constants.kTimeoutMs);
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
    SmartDashboard.putNumber("Left voltage", motor_Left.getBusVoltage());
    SmartDashboard.putNumber("Right voltage", motor_Right.getBusVoltage());
    // if (xbox.getRawAxis(3) > 0) {
    // startFW(currentSpeed);
    // } else if (xbox.getRawAxis(3) < 0) {
    // startFW(-0.3);
    // } else {
    // stopFW();
    // }
    shotCounter();
    // This method will be called once per scheduler run
  }

  public void resetEncoders() {
    /* Update Quadrature position */
    motor_Left.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
    motor_Right.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
  }

  public void startFW(double speed) {
    double curl = 0;
    // currentSpeed = 0.75; //for testing
    motor_Right.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
    if (!_state) {
      if (_firstCall)
        // System.out.println("This is Arcade Drive.\n");

        motor_Left.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, curl);
      motor_Right.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, -curl);
    } else {
      if (_firstCall) {
        // System.out.println("This is Drive Straight using the auxiliary feature with"
        // + "the difference between two encoders to maintain current heading.\n");

        /* Determine which slot affects which PID */
        motor_Right.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
      }

      /*
       * Configured for percentOutput with Auxiliary PID on Quadrature Encoders'
       * Difference
       */
      motor_Right.set(ControlMode.Velocity, speed, DemandType.AuxPID, curl);
      motor_Left.follow(motor_Right, FollowerType.AuxOutput1);
    }
    _firstCall = false;
  }

  public void stopFW() {
    // currentSpeed = 0;
    motor_Left.set(ControlMode.PercentOutput, 0);
    motor_Right.set(ControlMode.PercentOutput, 0);
  }

  /**
   * @return Average Velocity of flyweels in rad/s
   */
  public double getAvgSpeed() {
    return ((motor_Left.getSelectedSensorVelocity() + motor_Right.getSelectedSensorVelocity()) / 2);
    // / Constants.DROP_IN_DISTANCE_PER_REVOLUTION;
    // return 1.0;
  }

  public void startSpinning(double targetballspeed) {
    double wheelspeed = targetballspeed * Math.sqrt(
        Constants.I / (0.5 * Constants.ball_Weight + Constants.I / (Math.pow(Constants.SHOOTER_WHEEL_RADIUS, 2))));
    if (wheelspeed < Constants.MAX_SHOOTING_VEOLCITY && wheelspeed <= getAvgSpeed()) {
      currentSpeed += 0.05;
    }
    startFW(currentSpeed);
    SmartDashboard.putNumber("Shooting Velocity", wheelspeed);
  }

  public void shotCounter() {
    if (laser.get() && getAvgSpeed() < 0 && !hasCurrentBall) {
      ballCount++;
      hasCurrentBall = !hasCurrentBall;
    }

    else if (laser.get() && getAvgSpeed() > 0 && !hasCurrentBall) {
      ballCount--;
      hasCurrentBall = !hasCurrentBall;
    } else if (!laser.get() && hasCurrentBall) {
      hasCurrentBall = !hasCurrentBall;
    }
  }

  public void fireBall() {

  }

  public void increaseSpeed() {
    currentSpeed = currentSpeed + 0.1;

    SmartDashboard.putNumber("Current Shooter Speed", currentSpeed);
  }

  public void decreaseSpeed() {

    currentSpeed = currentSpeed - 0.1;
    SmartDashboard.putNumber("Current Shooter Speed", currentSpeed);
  }

  public void setSpeed(double speed) {
    currentSpeed = speed;
    SmartDashboard.putNumber("Current Shooter Speed", currentSpeed);
  }

  public boolean getHasBallNear() {
    return hasCurrentBall;
  }

}
