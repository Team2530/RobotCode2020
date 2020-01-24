/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //! NEED TO BE ACTUALLY SET
    //--------------------Motor Ports--------------------\\
    //DriveTrain Motors
    public static final int motor_Front_Left_Port = 1; 
    public static final int motor_Back_Left_Port = 3;
    public static final int motor_Back_Right_Port = 4; //? 2 motors driving one wheel on each side, i think
    public static final int motor_Front_Right_Port = 2;
    public enum DriveMotors
    { 
        FL, FR, BL, BR; 
    } 

    //Elevator Motors
    public static final int motor_Left_Leadscrew_Port = 5;
    public static final int motor_Right_Leadscrew_Port = 6;
    public static final int motor_Left_Pulley_Port = 7;
    public static final int motor_Right_Pulley_Port = 8;
    public enum ElevatorMotors
    {
        LL, RL, LP, RP;
    }

    //Shooter
    public static final int motor_Right_Flywheel_Port = 9;
    public static final int motor_Left_FlyWheel_Port = 10;

    public static final int motor_Ball_Leadscrew_Port = 11;


    //--------------------Sensor Ports--------------------\\
    //DriveTrain Encoders
    public static final int[] encoder_Left_Ports = {1,2}; 
    public static final int[] encoder_Right_Ports = {3,4};

    //!!!!!!!!!!!!!!!!!!!!!!Encoders plug into SRX, figure out how to read

    //Elevator Encoders
    public static final int[] encoder_Left_Leadscrew_Ports = {5,6}; 
    public static final int[] encoder_Right_Leadscrew_Ports = {7,8};

    //Elevator Limit Switches
    public static final int limit_Switch_Left_Leadscrew_Port = 1;
    public static final int limit_Switch_Right_Leadscrew_Port = 2;

    public static final int limit_Switch_Left_Pulley_Port = 3;
    public static final int limit_Switch_Right_Pulley_Port = 4;
    public enum ElevatorLimitSwitches 
    {
        LL, RL, LP, RP; //? or we could just use elevatorMotors enum.
    }

    //Shooter
    public static final int[] encoder_Left_Flywheel_Ports = {9,10}; 
    public static final int[] encoder_Right_Flywheel_Port = {11,12};

    //----------Sensor Constants-----------\\
    public static final int ENCODER_TICKS_PER_REVOLUTION = 60;
    public static final int gyroDrift = 5;
    public static final double sensor_Limelight_Height = 10;//? mounting height in inches

    //----------Driving Constants----------\\
    public static final double GEAR_RATIO = 1; //?This ratio is the ratio between the encoder and the driven wheels
    public static final double WHEEL_RADIUS = 6; //!Not diameter radius
    
    public static final int kP = 1;
    public static final int kI = 0;
    public static final int kD = 0;

    //----------Field Constants----------\\
    public static final int target_Height = 98;
    public static final double ball_Weight = 0.3125;

    
    //----------Control (Joystick) Constants----------\\
    public static final double deadzone = 0.1;

    //----------Control (Shooting) Constants----------\\
    public static final float I = 1;//?moment of inertia
    public static final int SHOOTER_WHEEL_RADIUS = 6;
    public static final double eff = 0.8;//?effective efficiency percentage
    public static final int MAX_DISTANCE = 100;

    //----------Control (Elevator) Constants----------\\
    //measurements based around shooter being level  
    //!these are all wrong, hardware didnt know anything
    public static final double bottomLeg = 29.68168; //inches
    public static final double maxAngle = 45; //degrees (45, 45, 90 triangle)
    public static final double maxHeight = 18.54717; //inches
    public static final double minAngle = -32; //degrees
    public static final double minHeight = -29.68168; //inches

    //----------TestBench (Motors) Constants----------\\
    public static final int test_motor_frontLeft_1_port = 1;
    public static final int test_motor_frontRight_1_port = 2;
    public static final int test_motor_backLeft_1_port = 3;
    //public static final int test_motor_backRight_1_port = 4;
     public enum TestMotors
    { 
        FL, FR, BL, BR; 
    } 
}
