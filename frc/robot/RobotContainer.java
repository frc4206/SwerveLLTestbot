// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.Autos;
import frc.robot.commands.ChangePipelineCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Swerve.TRAP_;
import frc.robot.commands.Swerve.ZeroGyroCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Joystick driver = new Joystick(0);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final Limelight m_Limelight = new Limelight();
  private final SwerveSubsystem swerve = new SwerveSubsystem(m_Limelight);

  public final static edu.wpi.first.wpilibj.XboxController.Axis tAxis = XboxController.Axis.kLeftY;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(0);
  
  Joystick operator = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    boolean fieldRelative = true;
    boolean openLoop = true;
    //swerve.setDefaultCommand(new TeleopSwerve(swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    m_driverController.x().onTrue(new ZeroGyroCommand(swerve));
    m_driverController.a().onTrue(new ChangePipelineCommand(m_Limelight, 0));
    m_driverController.b().onTrue(new ChangePipelineCommand(m_Limelight, 1));
    m_driverController.y().onTrue(new ChangePipelineCommand(m_Limelight, 2));
    m_driverController.rightBumper().whileTrue(new TRAP_(m_Limelight, swerve, driver, translationAxis, strafeAxis, rotationAxis, false, false, rotationAxis));

    new JoystickButton(operator, 1).onTrue(new ChangePipelineCommand(m_Limelight, 0));
    new JoystickButton(operator, 2).onTrue(new ChangePipelineCommand(m_Limelight, 1));
    new JoystickButton(operator, 3).onTrue(new ChangePipelineCommand(m_Limelight, 2));
    // Schedule `exampleMehodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
