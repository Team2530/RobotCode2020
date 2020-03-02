/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.libraries.EulersMethod;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Shooter;

public class AimShooter extends CommandBase {
  /**
   * Creates a new EmptyMag.
   */
  private Shooter m_shooter;
  private Elevator m_elevator;
  private LimeLight m_limelight;
  private double[] sposition;

  public AimShooter(Shooter shooter, Elevator elevator, LimeLight limelight) {
    m_shooter = shooter;
    m_elevator = elevator;
    m_limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(elevator);
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    sposition = m_limelight.getCylindricalPosition(m_elevator.getAngle(), Constants.sensor_Limelight_Height);
    double shootingV = EulersMethod.bisection(0, 30,
        new double[] {
            0.5 * Constants.gravity * Math.pow(sposition[0], 2) * Math.pow((1 / Math.cos(m_elevator.getAngle())), 2), 0,
            -m_elevator.getFloorHeight()[0], 1 });
    m_shooter.startFW(shootingV);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return true when magazine is empty|| if target is no longer in sight
    return false;
  }
}
