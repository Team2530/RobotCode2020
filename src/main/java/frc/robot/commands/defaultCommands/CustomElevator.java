/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.defaultCommands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class CustomElevator extends CommandBase {
  /**
   * Creates a new CustomElevator.
   */
  Joystick _gamepad;
  Elevator elevator;

  public CustomElevator(Joystick stick,Elevator elevator) {
    _gamepad = stick;
    this.elevator =elevator;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = _gamepad.getY();
		double turn = _gamepad.getTwist();
		forward = elevator.Deadband(forward);
		turn = elevator.Deadband(turn);
	
		/* Button processing for state toggle and sensor zeroing */
		if(_gamepad.getRawButtonPressed(2)){
			elevator.two();
		}else if (_gamepad.getRawButtonPressed(1)) {
			elevator.resetEncoders();			// Zero Sensors
		}
		
		if(!elevator._state){
			if (elevator._firstCall)
				System.out.println("This is a Arcade Drive.\n");
			
			elevator.motor_Left.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
			elevator.motor_Right.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
		}else{
			if (elevator._firstCall) {
				System.out.println("This is Drive Straight Distance with the Auxiliary PID using the difference between two encoders.");
				System.out.println("Servo [-6, 6] rotations while also maintaining a straight heading.\n");
				
				/* Determine which slot affects which PID */
				elevator.motor_Right.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
				elevator.motor_Right.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
			}
			
			/* Calculate targets from gamepad inputs */
			double target_sensorUnits = forward * Constants.DROP_IN_DISTANCE_PER_REVOLUTION* Constants.leadscrewDistancePerRotation  + elevator._lockedDistance;
      double target_turn = 0;
      SmartDashboard.putNumber("target", target_sensorUnits);
			SmartDashboard.putNumber("targetangle", elevator._targetAngle);
			/* Configured for Position Closed loop on Quad Encoders' Sum and Auxiliary PID on Quad Encoders' Difference */
			elevator.motor_Right.set(ControlMode.Position, target_sensorUnits, DemandType.AuxPID, target_turn);
			elevator.motor_Left.follow(elevator.motor_Right, FollowerType.AuxOutput1);
		}
		elevator._firstCall = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
