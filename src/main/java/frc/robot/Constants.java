/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;
import frc.robot.libraries.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // ! NEED TO BE ACTUALLY SET
    // --------------------Motor Ports--------------------\\
    // DriveTrain Motors
    // ports set up for test drivetrain currently
    public static final int motor_Front_Left_Port = 2;
    public static final int motor_Back_Left_Port = 1;
    public static final int motor_Back_Right_Port = 4; // ? 2 motors driving one wheel on each side, i think
    public static final int motor_Front_Right_Port = 3;

    public enum DriveMotors {
        FL, FR, BL, BR;
    }

    // Elevator Motors
    public static final int motor_Left_Leadscrew_Port = 4;
    public static final int motor_Right_Leadscrew_Port = 3;

    public enum ElevatorMotors {
        LL, RL, LP, RP;
    }

    // Shooter
    public static final int motor_Right_Flywheel_Port = 3;
    public static final int motor_Left_FlyWheel_Port = 4;

    public static final int motor_Conveyor_Port = 2;

    // --------------------Sensor Ports--------------------\\
    // DriveTrain Encoders
    public static final int[] encoder_Left_Ports = { 0, 1 };
    public static final int[] encoder_Right_Ports = { 2, 3 };

    // !!!!!!!!!!!!!!!!!!!!!!Encoders plug into SRX, figure out how to read

    // Elevator Encoders
    // public static final int[] encoder_Left_Leadscrew_Ports = {5,6};
    // public static final int[] encoder_Right_Leadscrew_Ports = {7,8};

    // Elevator Limit Switches
    public static final int limit_Switch_Left_Leadscrew_Port = 5;
    public static final int limit_Switch_Right_Leadscrew_Port = 6;

    public static final int limit_Switch_Left_Pulley_Port = 7;
    public static final int limit_Switch_Right_Pulley_Port = 8;

    public enum ElevatorLimitSwitches {
        LL, RL, LP, RP; // ? or we could just use elevatorMotors enum.
    }

    // Shooter
    public static final int[] encoder_Left_Flywheel_Ports = { 9, 10 };
    public static final int[] encoder_Right_Flywheel_Port = { 11, 12 };

    // ----------Sensor Constants-----------\\
    public static final int ENCODER_TICKS_PER_REVOLUTION = 2048;
    public static final int gyroDrift = 5;
    public static final double sensor_Limelight_Height = 25;// ? mounting height in inches

    //----------Driving Constants----------\\
    public static final double GEAR_RATIO = 1; //?This ratio is the ratio between the encoder and the driven wheels
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6); //!Diameter 
    public static final double DISTANCE_PER_PULSE = Math.PI * Constants.WHEEL_DIAMETER/ENCODER_TICKS_PER_REVOLUTION;
    public static final double kRamseteB = 0.1;//! Not calculated
    public static final double kRamseteZeta = 0.5;//! Not calculated

    //public static final double ALIGN = 0.025;
    public static final double WHEEL_DISTANCE = Units.inchesToMeters(22);//was in inches
    public static final double MAX_DRIVE_SPEED = 10;//Need this in m/s
    public static final double MAX_ANGULAR_SPEED = 0.5;//Need this in rad/s
    
    public static final double kP = .000122;
    public static final double kI = 0;
    public static final double kD = 0.892;
    public static final double kS = 0.967;
    public static final double kV = 0.784;

    public static final double tol = 5;
    public static final int setPoint = 1;

    public static final double maxVelocityMetersPerSecond = 1; // !This needs to be set
    public static final double maxAccelerationMetersPerSecondSq = 1; // !This needs to be set

    // ----------Field Constants----------\\
    public static final double target_Height = 105 * 2.54; // temp test value
    public static final double ball_Weight = 0.3125;

    // ----------Control (Joystick) Constants----------\\
    public static final double deadzone = 0.1;

    // ----------Control (Shooting) Constants----------\\
    public static final float I = 1;// ?moment of inertia
    public static final double SHOOTER_WHEEL_RADIUS = Units.inchesToMeters(6 / 2);
    public static final double eff = 0.8;// ?effective efficiency percentage
    public static final double MAX_SHOOTING_DISTANCE = 2.54;// m
    public static final double MAX_SHOOTING_VEOLCITY = 20;// meters per sec.
    public static final double IDEAL_SHOOTING_DISTANCE = 1.90;// m
    public static final double distanceTolerance = .10; // m
    public static final double angleTolerance = 10; // degrees
    public static final int DROP_IN_DISTANCE_PER_REVOLUTION = 1024;
    public static final double FLYWHEEL_DISTANCE_PER_REVOLUTION = SHOOTER_WHEEL_RADIUS * 2 * Math.PI;
    public static final double NeutralDeadband = 0.001;
    /**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
     * Not all set of Gains are used in this project and may be removed as desired.
     * 
	 * 	                                    			  kP   kI   kD   kF               Iz    PeakOut */
    public final static Gains kGains_Distanc = new Gains( 0.1, 0.0,  0.0, 0.0,            100,  0.50 );//! NEED TO BE SET
	public final static Gains kGains_Turning = new Gains( 2.0, 0.0,  4.0, 0.0,            200,  1.00 );//!
	public final static Gains kGains_Velocit = new Gains( 0.1, 0.0, 20.0, 1023.0/6800.0,  300,  0.50 );//!
	public final static Gains kGains_MotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/6800.0,  400,  1.00 );//!

    public final static int SLOT_0 = 0;
    public final static int SLOT_1 = 1;
    public final static int SLOT_2 = 2;
    public final static int SLOT_3 = 3;
    /* ---- Named slots, used to clarify code ---- */
	public final static int kSlot_Distanc = SLOT_0;
	public final static int kSlot_Turning = SLOT_1;
	public final static int kSlot_Velocit = SLOT_2;
	public final static int kSlot_MotProf = SLOT_3;
    /*
     * We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1
     * is auxiliary
     */
    public final static int PID_PRIMARY = 0;
    public final static int PID_TURN = 1;

    /*
     * We allow either a 0 or 1 when selecting an ordinal for remote devices [You
     * can have up to 2 devices assigned remotely to a talon/victor]
     */
    public final static int REMOTE_0 = 0;
    public final static int REMOTE_1 = 1;

    public final static int kTimeoutMs = 30;// The time before TImeout for encoder is called

    // ----------Control (Elevator) Constants----------\\
    // measurements based around shooter being level
    // !these are all wrong, hardware didnt know anything
    public static final double bottomLeg = 29.68168 * 2.54; // cm
    public static final double maxAngle = 45; // degrees (45, 45, 90 triangle)
    public static final double maxHeight = 18.54717 * 2.54; // cm
    public static final double minAngle = -32; // degrees
    public static final double minHeight = -29.68168 * 2.54; // cm
    public static final double gravity = -9.8;// meters per second

}
