/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class ConveyorControl extends CommandBase {

  Conveyor conveyor;
  XboxController xbox;

  /**
   * Creates a new ConveyorControl.
   */
  public ConveyorControl(Conveyor conveyor, XboxController xbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.conveyor = conveyor;
    this.xbox = xbox;
    
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dpadAngle = xbox.getPOV();
    SmartDashboard.putNumber("Xbox POV", dpadAngle);
    if(dpadAngle == -1) {
      //do nothing
      conveyor.stopIntake();
    } else if(dpadAngle > 90 && dpadAngle < 270) {
      //spit out
      conveyor.out();
    } else if(dpadAngle > 270 || dpadAngle < 90) {
      //succ in
      conveyor.in();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
