/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Vibrate extends CommandBase {
  private XboxController xbox;
  
  public Vibrate(XboxController xbox) {
   // addRequirements(elevatorSub); //might have to do xbox = xbox
    this.xbox = xbox;
    
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    //requires(Robot.m_oi);
  }

  
  boolean Rumbling = false;

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    //final XboxController xbox = Robot.m_oi.getXbox();
    
    if (Rumbling) {
      xbox.setRumble(RumbleType.kLeftRumble, 0);
      xbox.setRumble(RumbleType.kRightRumble, 0);
      Rumbling = false;
    } else {
      xbox.setRumble(RumbleType.kLeftRumble, .5);
      xbox.setRumble(RumbleType.kRightRumble, .5);
      Rumbling = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
}
