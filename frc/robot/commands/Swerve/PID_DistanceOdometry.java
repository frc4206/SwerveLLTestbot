  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class PID_DistanceOdometry extends CommandBase {
  Limelight Limelight;
  SwerveSubsystem swerveSubsystem;
  Joystick controller;
  boolean isfinished; 

  private double rotation;

  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;
  
  double StartCommandTime;
  double CurrentTime;
  
  public double lastErrorX = 0;
  public double lastErrorY = 0;
  public double lastErrorYaw = 0;

  public double xSet;
  public double ySet;
  public double yawSet;
  public double timeout;

  double outputYaw;

  public double m_kP = 0.95;
  public double m_kI = 0.67;
  public double m_kD = 0.0;
  public double m_YkP = 0.55;
  public double m_YkD = 0.6;
  public double m_YkI = 0.0;
  public double m_YawkP = 0;
  public double m_YawkI = 0;
  public double m_YawkD = 0;
  public PIDController pidx = new PIDController(m_kP, 0, m_kD);
  public PIDController pidxi = new PIDController(m_kP, m_kI, m_kD);

  public PIDController pidy = new PIDController(m_YkP, 0, m_YkD);
  public PIDController pidyi = new PIDController(m_YkP, m_YkI, m_YkD);

  public PIDController pidyaw = new PIDController(m_YawkP, 0, m_YawkD);
  public PIDController pidyawi = new PIDController(m_YawkP, m_YawkI, m_YawkD);

  /** Creates a new PID_Distance. */
  public PID_DistanceOdometry(Limelight m_Limelight, SwerveSubsystem m_swerveSubsystem, int m_translationAxis, int m_strafeAxis, int m_rotationAxis, boolean m_fieldRelative, boolean m_openLoop, double m_xSetpoint, double m_ySetpoint, double m_yawSetpoint, double m_timeout) {
    Limelight = m_Limelight;
    swerveSubsystem = m_swerveSubsystem;
    addRequirements(Limelight);
    addRequirements(swerveSubsystem);
    

    xSet = m_xSetpoint;
    ySet = m_ySetpoint;
    yawSet = m_yawSetpoint;
    timeout = m_timeout;


    fieldRelative = m_fieldRelative;
    openLoop = m_openLoop;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isfinished = false;
    System.out.println("innit run in odometry command");
    StartCommandTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("has run");
    CurrentTime = Timer.getFPGATimestamp() - StartCommandTime;
    /*m_kP = Limelight.XkP;
    m_kI = Limelight.XkI;
    m_kD = Limelight.XkD;
    m_YkP = Limelight.YkP;
    m_YkD = Limelight.YkD;
    m_YkI = Limelight.YkI;
    m_YawkP = Limelight.YawkP;
    m_YawkI = Limelight.YawkI;
    m_YawkD = Limelight.YawkD; */
    
    double m_MaxError = .025;
    //double[] robotToCam = {.1143, .00762};

    //0.57
    double errorX = Limelight.limelightManger.X - xSet;


    double outputX;
    
    //= errorX*m_kP + errorDifferenceX*m_kD

    if (Math.abs(errorX) > 1) {
      outputX = pidx.calculate(swerveSubsystem.swerveOdometry.getPoseMeters().getX(), xSet);
    } else {
      outputX = pidxi.calculate(swerveSubsystem.swerveOdometry.getPoseMeters().getX(), xSet);
    }

    double errorY = Limelight.limelightManger.Y - ySet;


    double outputY;

    

    if (Math.abs(errorY) > .15) {
      outputY = pidy.calculate(swerveSubsystem.swerveOdometry.getPoseMeters().getY(), ySet);
    } else {
      outputY = pidyi.calculate(swerveSubsystem.swerveOdometry.getPoseMeters().getY(), ySet);
    }


    double errorYaw = Limelight.Yaw - yawSet;
    if (Math.abs(errorYaw) > .15) {
      outputYaw = pidyaw.calculate(swerveSubsystem.getYaw().getDegrees(), yawSet);
    } else {
      outputYaw = pidyawi.calculate(swerveSubsystem.getYaw().getDegrees(), yawSet);
    }
    
    double yAxis = -outputY;
    double xAxis = outputX;

    double rAxis = 0;
    
    //(Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis
    //xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
    //rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;
    
    //translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
    translation = new Translation2d(xAxis, yAxis).times(Constants.Swerve.maxSpeed);
    rotation = rAxis * Constants.Swerve.maxAngularVelocity;
    swerveSubsystem.drive(translation, rotation, fieldRelative, openLoop);


    if (Math.abs(errorY) < m_MaxError && Math.abs(errorX) < m_MaxError) {
      System.out.println("terminated");
      isfinished = true;
      isFinished();
    }

    if (CurrentTime > timeout) {
      System.out.println("terminated on timeout");
      System.out.println(errorX);
      System.out.println(errorY);
      isfinished = true;
      isFinished();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isfinished;
  }
}
