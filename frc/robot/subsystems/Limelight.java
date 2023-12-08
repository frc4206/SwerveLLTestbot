// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightCameraClass;
import frc.robot.LimelightCameraManagerClass;

public class Limelight extends SubsystemBase {
  public static final double Yaw = 0;
/** Creates a new Limelight. */

  LimelightCameraClass limelightfront = new LimelightCameraClass(1, "limelight-front", 25, 24.5, 0);
  LimelightCameraClass limelightright = new LimelightCameraClass(1, "limelight-right", 25, 24.5, 0);
  LimelightCameraClass limelightleft = new LimelightCameraClass(1, "limelight-left", 25, 24.5, 0);

  LimelightCameraClass[] limelightList = {limelightfront, limelightleft, limelightright};

  public LimelightCameraManagerClass limelightManger = new LimelightCameraManagerClass(limelightList);

  public static boolean isenabled = false;
  boolean init = false;

  public Limelight() {}

  public void ChangePipelines(int pipeline) {
    limelightManger.changeAllPipelines(pipeline);
  }

  public double[] getFieldCordsTEST() {
    return limelightManger.pose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    limelightManger.Update();

      if (!isenabled) {
        ChangePipelines(0);
        init= true;
      }
  } 
}
