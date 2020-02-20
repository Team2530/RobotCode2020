/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class StartIntake extends CommandBase {

  Shooter shooter;
  XboxController xbox;
  /**
   * Creates a new StartIntake.
   */
  public StartIntake(Shooter m_shooter, XboxController xbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = m_shooter;
    this.xbox = xbox;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Xbox trigger", xbox.getRawAxis(3));
    // xbox.getRawAxis(3)
    shooter.startFW(-0.3); //xbox.getY(Hand.kRight)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopFW();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
