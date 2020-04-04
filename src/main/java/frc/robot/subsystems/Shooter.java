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
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  private static WPI_TalonSRX motor_Left = new WPI_TalonSRX(Constants.motor_Left_FlyWheel_Port);
  private static WPI_TalonSRX motor_Right = new WPI_TalonSRX(Constants.motor_Right_Flywheel_Port);

  // DigitalInput laser = new DigitalInput(Constants.laser_switch);

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

    motor_Right.set(ControlMode.PercentOutput, 0);
		motor_Left.set(ControlMode.PercentOutput, 0);

		/* Factory Default all hardware to prevent unexpected behaviour */
		motor_Right.configFactoryDefault();
		motor_Left.configFactoryDefault();
		
		/* Set Neutral Mode */
		motor_Left.setNeutralMode(NeutralMode.Coast);
    motor_Right.setNeutralMode(NeutralMode.Coast);
    
    /* Configure the drivetrain's left side Feedback Sensor as a Magnet Encoder */
    /* Configure the left Talon's selected sensor to a Quad Encoder */
    motor_Left.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, // Local Feedback Source
        Constants.PID_PRIMARY, // PID Slot for Source [0, 1]
        Constants.kTimeoutMs); // Configuration Timeout

    /*
     * Configure the Remote Talon's selected sensor as a remote sensor for the right
     * Talon
     */
    motor_Right.configRemoteFeedbackFilter(motor_Left.getDeviceID(), // Device ID of Source
        RemoteSensorSource.TalonSRX_SelectedSensor, // Remote Feedback Source
        Constants.REMOTE_1, // Source number [0, 1]
        Constants.kTimeoutMs); // Configuration Timeout

    /* Setup Sum signal to be used for Distance */
    motor_Right.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor1, Constants.kTimeoutMs); // Feedback
                                                                                                        // Device of
                                                                                                        // Remote Talon
    motor_Right.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs); // Quadrature
                                                                                                      // Encoder of
                                                                                                      // current Talon

    /* Setup Difference signal to be used for Turn */
    motor_Right.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor1, Constants.kTimeoutMs);
    motor_Right.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);

    /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
    motor_Right.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, Constants.PID_PRIMARY, Constants.kTimeoutMs);

    /* Scale Feedback by 0.5 to half the sum of Distance */
    motor_Right.configSelectedFeedbackCoefficient(0.5, // Coefficient
        Constants.PID_PRIMARY, // PID Slot of Source
        Constants.kTimeoutMs); // Configuration Timeout

    /*
     * Configure Difference [Difference between both QuadEncoders] to be used for
     * Auxiliary PID Index
     */
    motor_Right.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, Constants.PID_TURN,
        Constants.kTimeoutMs);

    /* Don't scale the Feedback Sensor (use 1 for 1:1 ratio) */
    motor_Right.configSelectedFeedbackCoefficient(1, Constants.PID_TURN, Constants.kTimeoutMs);

    // Inverting Motors and Encoders
    motor_Left.setInverted(true);
    motor_Left.setSensorPhase(true);
    motor_Right.setInverted(false);
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
    /* FPID Gains for velocity servo */

    motor_Right.config_kP(Constants.kSlot_Velocit, Constants.kShooter_Gains_Velocit.kP, Constants.kTimeoutMs);
    motor_Right.config_kI(Constants.kSlot_Velocit, Constants.kShooter_Gains_Velocit.kI, Constants.kTimeoutMs);
    motor_Right.config_kD(Constants.kSlot_Velocit, Constants.kShooter_Gains_Velocit.kD, Constants.kTimeoutMs);
    motor_Right.config_kF(Constants.kSlot_Velocit, Constants.kShooter_Gains_Velocit.kF, Constants.kTimeoutMs);
    motor_Right.config_IntegralZone(Constants.kSlot_Velocit, Constants.kShooter_Gains_Velocit.kIzone,
        Constants.kTimeoutMs);
    motor_Right.configClosedLoopPeakOutput(Constants.kSlot_Velocit, Constants.kShooter_Gains_Velocit.kPeakOutput,
        Constants.kTimeoutMs);
    motor_Right.configAllowableClosedloopError(Constants.kSlot_Velocit, 0, Constants.kTimeoutMs);

    /* FPID Gains for turn servo */
    motor_Right.config_kP(Constants.kSlot_Turning, Constants.kShooter_Gains_Turning.kP, Constants.kTimeoutMs);
    motor_Right.config_kI(Constants.kSlot_Turning, Constants.kShooter_Gains_Turning.kI, Constants.kTimeoutMs);
    motor_Right.config_kD(Constants.kSlot_Turning, Constants.kShooter_Gains_Turning.kD, Constants.kTimeoutMs);
    motor_Right.config_kF(Constants.kSlot_Turning, Constants.kShooter_Gains_Turning.kF, Constants.kTimeoutMs);
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
    _firstCall = true;
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Left voltage", motor_Left.getBusVoltage());
    // SmartDashboard.putNumber("Right voltage", motor_Right.getBusVoltage());
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
		System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");
  }

  public void startFW(double power){
    motor_Right.set(ControlMode.PercentOutput, power);
    motor_Left.follow(motor_Right,FollowerType.PercentOutput);
  }
  public void startFW() {
    double speed = SmartDashboard.getNumber("speed", 0);
    double curl = 0;
        if (_firstCall) {
          System.out.println("This is Velocity Closed Loop with the Auxiliary PID using quadrature encoders.");
          
          /* Determine which slot affects which PID */
          motor_Right.selectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);
          motor_Right.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
          curl = motor_Right.getSelectedSensorPosition(Constants.PID_TURN);
          _firstCall = !_firstCall;
        }
      /*
       * Configured for percentOutput with Auxiliary PID on Quadrature Encoders'
       * Difference
       */
      // double target_unitsPer100ms = getTarget_unitsPer100ms(getTargetWheelSpeed(speed));
      double target_unitsPer100ms = speed;
      SmartDashboard.putNumber("Target Speed", target_unitsPer100ms);
			double target_turn = curl;
			
			/* Configured for Velocity Closed Loop on Quad Encoders' Sum and Auxiliary PID on Quad Encoders' Difference */
			motor_Right.set(ControlMode.Velocity, target_unitsPer100ms, DemandType.AuxPID, target_turn );
		  motor_Left.follow(motor_Right, FollowerType.AuxOutput1);
    
  }

  public void stopFW() {
    // currentSpeed = 0;
    motor_Left.set(ControlMode.PercentOutput, 0);
    motor_Right.set(ControlMode.PercentOutput, 0);
    _firstCall = true;
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
    double wheelspeed = getTargetWheelSpeed(targetballspeed);
    if (wheelspeed < Constants.MAX_SHOOTING_VEOLCITY && wheelspeed <= getAvgSpeed()) {
      currentSpeed += 0.05;
    }
    //startFW(currentSpeed);
    SmartDashboard.putNumber("Shooting Velocity", wheelspeed);
  }

  public void shotCounter() {
    // if (laser.get() && getAvgSpeed() < 0 && !hasCurrentBall) {
    // ballCount++;
    // hasCurrentBall = !hasCurrentBall;
    // }

    // else if (laser.get() && getAvgSpeed() > 0 && !hasCurrentBall) {
    // ballCount--;
    // hasCurrentBall = !hasCurrentBall;
    // } else if (!laser.get() && hasCurrentBall) {
    // hasCurrentBall = !hasCurrentBall;
    // }
  }

  public void fireBall() {

  }

  public double getMaxTemperature() {
    return Math.max(motor_Left.getTemperature(), motor_Right.getTemperature());
  }

  public void increaseSpeed() {
    currentSpeed = currentSpeed + 0.1;

    SmartDashboard.putNumber("Current Shooter Speed", currentSpeed);
  }
  public double getTargetWheelSpeed(double targetballspeed){
    return targetballspeed * Math.sqrt(
        Constants.I / (0.5 * Constants.ball_Weight + Constants.I / (Math.pow(Constants.SHOOTER_WHEEL_RADIUS, 2))));
  }
  //v = r × RPM × 0.10472
  public double getTarget_unitsPer100ms(double targetwheelspeed){
    return targetwheelspeed/(Constants.SHOOTER_WHEEL_RADIUS*0.10472);
  }

  public void resetSpeed() {

    currentSpeed = 0;
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
