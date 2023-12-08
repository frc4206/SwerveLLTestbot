package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightCameraManagerClass {
    public LimelightCameraClass[] cameraList;

    public double X;
    public double Y;
    public double Z;

    public double[] pose;

    public LimelightCameraManagerClass (LimelightCameraClass[] camlist) {
        cameraList = camlist;
    }

    

    public void changeAllPipelines(int pip) {
        for (int i = 0; i < cameraList.length; i++) {
            cameraList[i].ChangePipelines(pip);
        }
    }

    public int GetClosestCameraAi() {
        int camID = 4;
        for (int i = 0; i < cameraList.length; i++) {
            try {
                if (cameraList[i].GetDistanceToGamePiece() < cameraList[i + 1].GetDistanceToGamePiece()) {
                    camID = cameraList[i].camID;
                }
            } catch (Exception e) {
                // TODO: handle exception
                //chill out
            }
            
        }
        return camID;
    }

    public void Update() {
        for (int i = 0; i < cameraList.length; i++) {
            cameraList[i].UpdateResults();
            if (cameraList[i].GetPipeline() == 2) {

                if (cameraList[i].aprilTagResult[2] == 0) {
                    cameraList[i].aprilTagResult[2] = 150;
                }
                
                SmartDashboard.putNumber(i + " X", cameraList[i].aprilTagResult[2]);
                for (int ii = 0; ii < cameraList.length; ii++) {
                    try {
                        if (cameraList[ii].aprilTagResult[2] < cameraList[ii + 1].aprilTagResult[2] && cameraList[ii + 1].GetPipeline() == 2) {
                            if (ii >= 1) {
                                if (cameraList[ii].aprilTagResult[2] < cameraList[ii - 1].aprilTagResult[2]&& cameraList[ii - 1].GetPipeline() == 2) {
                                    X = cameraList[ii].Fieldresult[5];
                                    Y = cameraList[ii].Fieldresult[3];
                                    Z = cameraList[ii].Fieldresult[2];
                                }
                            } else {
                                X = cameraList[ii].Fieldresult[5];
                                Y = cameraList[ii].Fieldresult[3];
                                Z = cameraList[ii].Fieldresult[2];
                            }
                        }
                    } catch (Exception e) {
                        // TODO: handle exception
                        //chill out
                    }
                    double[] pos = {X, Y, Z};
                    pose = pos;
                }
                
            } if (cameraList[i].GetPipeline() == 1) {
                for (int ii = 0; ii < cameraList.length; ii++) {
                    if (cameraList[ii].GetDistanceToGamePiece() > 50) {
                        cameraList[ii].distanceToTarget = 250;
                    }
                  }
            }

        }
    }
}
