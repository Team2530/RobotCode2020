/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pixy;
import frc.robot.commands.LocateBall;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class WoFSpinner extends CommandBase {
  /**
   * Creates a new WoFSpinner.
   */
  Shooter shooter;

  public void WoFSpinner(Shooter shooter, Pixy pixy, DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    final DriveTrain m_driveTrain;
    final Pixy m_pixy;
    m_pixy = pixy;
    m_driveTrain = driveTrain;
    addRequirements(shooter);
    addRequirements(pixy);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.

  public WoFSpinner() {
    double displaySpeedRads = shooter.getAvgSpeed();
    double displaySpeedRPM = displaySpeedRads * 9.549296585513702;
    SmartDashboard.putString("The Current Shooter Speed is: " + displaySpeedRads +"", "shooter-speed");
  }
    public static void spin() {
    while (1 == 1) {
      if (1 == 1/*If pixy object WoF_Red or WoF_Yellow or WoF_lightBlue or WoF_limeGreen is detected, run the below code*/) {
        //run motors at 0.1 speed
        m_driveTrain.setSingleMotorPower(DriveMotors.FR, 0);
        m_driveTrain.setSingleMotorPower(DriveMotors.FL, 0);
      }

      else {

      }
    {
  } 
}

  // Called once the command ends or is interrupted.

  // Returns true when the command should end.
}} 
