package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private SwerveSubsystem s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    /**
     * Driver control
     */
    public TeleopSwerve(SwerveSubsystem s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        /*sets the axis to the controller sticks*/
        double yAxis = -controller.getRawAxis(translationAxis)*0.75;
        double xAxis = -controller.getRawAxis(strafeAxis)*0.75;
        double rAxis = -controller.getRawAxis(rotationAxis)*0.75;
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

        translation = new Translation2d(yAxis*(s_Swerve.currPercent), xAxis*(s_Swerve.currPercent)).times(Constants.Swerve.maxSpeed).times(s_Swerve.currPercent);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity * s_Swerve.currPercent;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}