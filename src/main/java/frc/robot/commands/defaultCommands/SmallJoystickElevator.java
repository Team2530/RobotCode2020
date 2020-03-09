/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.defaultCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorLimitSwitches;
import frc.robot.Constants.ElevatorMotors;
import frc.robot.subsystems.Elevator;

public class SmallJoystickElevator extends CommandBase {
  Elevator elevatorSub;

  Joystick stick1;

  double y1;

  /**
   * Creates a new SmallJoystickElevator.
   */
  public SmallJoystickElevator(Elevator elevatorSub, Joystick stick1) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSub);
    this.stick1 = stick1;
    this.elevatorSub = elevatorSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    y1 = stick1.getRawAxis(4);
    if (Math.abs(y1) <= Constants.deadzone) {
      y1 = 0;
    }

    // y1 = 1 * (0.5 * Math.pow(y1, 3) + 0.5 * y1); prob dont need this but can add
    // it

    elevatorSub.setMotorPower(ElevatorMotors.Left, y1);
    elevatorSub.setMotorPower(ElevatorMotors.Right, y1);

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
