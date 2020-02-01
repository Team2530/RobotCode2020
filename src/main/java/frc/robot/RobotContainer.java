/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DualLargeJoystickDrive;
import frc.robot.commands.EncoderTest;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LineUp;
import frc.robot.commands.LocateBall;
import frc.robot.commands.TestPixy;
import frc.robot.commands.ToggleLimeLightLED;
import frc.robot.commands.TurnToAngle;
// import frc.robot.commands.SmallJoystickElevator;
// import frc.robot.commands.XboxJoystickElevator;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Pixy;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Elevator elevatorSub = new Elevator();
  private final LimeLight limeLightSub = new LimeLight();
  private final Pixy m_pixy = new Pixy();
  private final Shooter m_shooter = new Shooter();
  

  final Joystick stick1 = new Joystick(1); // Creates a joystick on port 1
  final Joystick stick2 = new Joystick(2); // Creates a joystick on port 1
  private final JoystickButton Button1 = new JoystickButton(stick1, 1); // Creates a new JoystickButton object for
  private final JoystickButton Button3 = new JoystickButton(stick1, 3);
  private final JoystickButton Button4 = new JoystickButton(stick1, 4);
<<<<<<< HEAD


                                                                        // button 1 on stick
  private final JoystickButton Button3 = new JoystickButton(stick1, 3);
  private final JoystickButton Button4 = new JoystickButton(stick1, 4);
  final XboxController xbox = new XboxController(0);

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final DualLargeJoystickDrive m_telopCommand = new DualLargeJoystickDrive(m_driveTrain, stick1, stick2);
  // private final XboxJoystickElevator elevatorCommand = new
  // XboxJoystickElevator(elevatorSub, xbox);
  // private final SmallJoystickElevator elevatorCommand = new
  // SmallJoystickElevator(elevatorSub, stick1);
  // private final EncoderTest m_telopCommand = new EncoderTest(m_driveTrain);
  private final LineUp lineUp = new LineUp(m_driveTrain, limeLightSub, elevatorSub);
  private final TestPixy pixy = new TestPixy(m_pixy);
  private final ToggleLimeLightLED toggleLED = new ToggleLimeLightLED(limeLightSub);
  private final LocateBall locateBall = new LocateBall(m_pixy, m_driveTrain, m_shooter);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Button1.whileHeld(lineUp);
    Button1.whenReleased(m_telopCommand);
    Button3.whenPressed(toggleLED);
<<<<<<< HEAD
    Button4.whenPressed(locateBall);
=======
    Button4.whenPressed(new TurnToAngle(45, m_driveTrain));
>>>>>>> ballSensor
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public Command getTelopCommand() {
    return m_telopCommand;
  }

}
