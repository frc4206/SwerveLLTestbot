// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class TRAP_ extends CommandBase {

  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;
  
  private SwerveSubsystem s_Swerve;
  private Joystick controller;
  private int translationAxis;
  private int strafeAxis;
  private int rotationAxis;

  private double max_Velocity = 10;
  private double max_Acceleration = 2;


  Limelight m_Limelight;

  private double Xset = 0;
  public double Xpidset = 0;

  Constraints constraints = new Constraints(max_Velocity, max_Acceleration);
  State Xstate;

  TrapezoidProfile X_trap;
  public PIDController pidx = new PIDController(0, 0, 0);


  public double startTime = 0;
  public double timeElapsed = 0;

  /** Creates a new TRAP_. */
  public TRAP_(Limelight limelight, SwerveSubsystem s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop, double xset) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    
    this.controller = controller;
    this.translationAxis = translationAxis;
    this.strafeAxis = strafeAxis;
    this.rotationAxis = rotationAxis;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
    m_Limelight = limelight;
    Xset = xset;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    timeElapsed = Timer.getFPGATimestamp() - startTime;

    


    Xstate = new State(Xset, 0);
    X_trap = new TrapezoidProfile(constraints, Xstate);



    Xpidset = X_trap.calculate(timeElapsed).position;
    double yAxis = pidx.calculate(s_Swerve.swerveOdometry.getPoseMeters().getX(), Xpidset);
    SmartDashboard.putNumber("Xset", yAxis);
    double xAxis = 0;
    double rAxis = 0;
  


    translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
    rotation = rAxis * Constants.Swerve.maxAngularVelocity;
    s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    translation = new Translation2d(0, 0).times(Constants.Swerve.maxSpeed);
    rotation = 0;
    s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
