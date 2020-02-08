/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.DriveTrain;

public class PIDTune extends CommandBase {
  /**
   * Set all gains to zero. Increase the P gain until the response to a
   * disturbance is steady oscillation. Increase the D gain until the the
   * oscillations go away (i.e. it's critically damped). Repeat steps 2 and 3
   * until increasing the D gain does not stop the oscillations. Set P and D to
   * the last stable values. Increase the I gain until it brings you to the
   * setpoint with the number of oscillations desired (normally zero but a quicker
   * response can be had if you don't mind a couple oscillations of overshoot)
   */
  NetworkTableInstance inst;
  NetworkTable table;
  NetworkTableEntry P, I, D,setpoint;
  private DriveTrain m_driveTrain;

  public PIDTune() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("PID TUNE"); // note the custom table name
    P = table.getEntry("P");
    I = table.getEntry("I");
    D = table.getEntry("D");
    setpoint = table.getEntry("setpoint");
    P.setNumber(m_driveTrain.getPID()[0]);
    I.setNumber(m_driveTrain.getPID()[1]);
    D.setNumber(m_driveTrain.getPID()[2]);
    setpoint.setNumber(m_driveTrain.getSetPoint());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Number[] value = setpoint.getNumberArray(new Number[]{0,0});
    m_driveTrain.setPID(P.getNumber(-1).doubleValue(), I.getNumber(-1).doubleValue(), D.getNumber(-1).doubleValue());
    m_driveTrain.setSetPoint(new double[]{value[0].doubleValue(),value[1].doubleValue()});
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
