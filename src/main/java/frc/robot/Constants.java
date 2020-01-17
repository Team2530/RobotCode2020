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
    //----------Motor Ports----------\\
    //DriveTrain
    public static final int motor_Front_Left_Port = 1; 
    public static final int motor_Back_Left_Port = 3;
    public static final int motor_Back_Right_Port = 4;
    public static final int motor_Front_Right_Port = 2;
    public enum DriveMotors
    { 
        FL, FR, BL, BR; 
    } 

    //Elevator
    public static final int motor_Left_Leadscrew_Port = 5;
    public static final int motor_Right_Leadscrew_Port = 6;
    public static final int motor_Left_Pulley_Port = 7;
    public static final int motor_Right_Pulley_Port = 8;

    //----------Sensor Ports----------\\
    //DriveTrain
    public static final int[] encoder_Left_Ports = {1,2}; 
    public static final int[] encoder_Right_Port = {3,4};

    //!!!!!!!!!!!!!!!!!!!!!!Encoders plug into SRX, figure out how to read

    //Elevator
    public static final int[] encoder_Left_Leadscrew_Ports = {5,6}; 
    public static final int[] encoder_Right_Leadscrew_Port = {7,8};

    //TODO 4 limit switches, 2 on leadscrew, 2 on pulley for elevator

    //----------Driving Constants----------\\
    public static final double GEAR_RATIO = 1; //?This ratio is the ratio between the encoder and the driven wheels
    public static final double WHEEL_RADIUS = 6; //!Not diameter radius
    public static final int ENCODER_TICKS_PER_REVOLUTION = 60;

    public static final int kP = 1;
    public static final int kI = 0;
    public static final int kD = 0;

    public static final int gyroDrift = 5;

    //----------Controll (Joystick) Constants----------\\
    public static final double deadzone = 0.1;
}
