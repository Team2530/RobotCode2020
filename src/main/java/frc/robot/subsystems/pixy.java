/*All pixy code*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class pixy extends SubsystemBase {
  private DriveTrain driveTrain;
  private Shooter shooter;
  private Elevator elevator;
  public ballSensor() {

  }

  @Override
  public void periodic() {
    if (//button pressed
    ){
        this.driveTrain = driveTrain;
        this.shooter = shooter;
        this.elevator = elevator;
        //take over drivetrain, elevator, and shooter
        //stuff when we know what info the pixy gives us
    }
  }
}
