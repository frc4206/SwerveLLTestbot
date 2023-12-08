package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightCameraClass {

    //Initalize limelight
    public String limelightName;
    public NetworkTable limelightTable;
    public int camID;


    //April tag variables
    public double[] aprilTagResult ;
    public double[] Fieldresult;


    // Ai variables

    // how many degrees back is your limelight rotated from perfectly vertical?
   double limelightMountAngleDegrees = -25.0;

   // distance from the center of the Limelight lens to the floor
   double limelightLensHeightInches = 24.5;
 
   // distance from the target to the floor
   double goalHeightInches = 0;

   double distanceToTarget = 150;


   public LimelightCameraClass(int mcamID, String mlimelightName, double angle, double height, double targetHeight) {
    camID = mcamID;
    limelightName = mlimelightName;
    limelightMountAngleDegrees = angle;
    limelightLensHeightInches = height;
    goalHeightInches = targetHeight;

    limelightTable =  NetworkTableInstance.getDefault().getTable(limelightName);
   }


   public double HasTarget() {
    return limelightTable.getEntry("tv").getDouble(0);
   }
   public void ChangePipelines(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }

    public double GetPipeline() {
        return limelightTable.getEntry("pipeline").getDouble(0);
    }

  public double GetDistanceToGamePiece() {
    NetworkTableEntry ty = limelightTable.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

    return distanceFromLimelightToGoalInches;
  }

  public void UpdateResults() {
    //Updates the current pipleines calcuations
    if (limelightTable.getEntry("pipeline").getDouble(0) == 2) {
        aprilTagResult = limelightTable.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        Fieldresult = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
    } else if (limelightTable.getEntry("pipeline").getDouble(0) == 1) {
        SmartDashboard.putNumber(limelightName + " ai distance: ", GetDistanceToGamePiece());
    }

  }
}
