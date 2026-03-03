// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libs;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class NetworkTables {
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  
  private static NetworkTable dash = inst.getTable(Constants.NetworktablePaths.Dashboard);
    public static NetworkTableEntry dashCoralLoc = dash.getEntry("coral");
    public static NetworkTableEntry commands = dash.getEntry("commands_sa");  
    public static NetworkTableEntry commandStatuses = dash.getEntry("commandStatuses_ia");  

  private static NetworkTable dsInfo = dash.getSubTable(Constants.NetworktablePaths.DS);
    public static NetworkTableEntry state_s = dsInfo.getEntry("state_s");
    public static NetworkTableEntry voltage_d = dsInfo.getEntry("voltage_d");


  private static NetworkTable devBoard = dash.getSubTable(Constants.NetworktablePaths.Test);
    public static NetworkTableEntry numOLoggedCmds_i = devBoard.getEntry("numOLoggedCmds_i");
    
    // Former Camera average position on dev board.
    public static NetworkTableEntry pathplannerGoalPose = devBoard.getEntry("cameraPose_ad");
    public static NetworkTableEntry oneCameraPose = devBoard.getEntry("oneCameraPose_ad");
    public static NetworkTableEntry twoCameraPose = devBoard.getEntry("twoCameraPose_ad");

    public static NetworkTableEntry measuredSwerveStates_da = devBoard.getEntry("measuredStates_da");
    public static NetworkTableEntry desiredSwerveStates_da = devBoard.getEntry("desiredStates_da");
    public static NetworkTableEntry botRotDeg_d = devBoard.getEntry("botRotDeg_d");
    public static NetworkTableEntry maxVelo = devBoard.getEntry("maxVelo_d");

    // Test buttons
    public static NetworkTableEntry swerveButton_b = devBoard.getEntry("Swerve_b");
    public static NetworkTableEntry turretButton_b = devBoard.getEntry("Turret_b");
    public static NetworkTableEntry climberButton_b = devBoard.getEntry("Climber_b");

  // private static NetworkTable sensors = inst.getTable(Constants.NetworktablePaths.Sensors);
  //   public static NetworkTableEntry globalCameraTimestamp = sensors.getEntry("timestamp");

  //   private static NetworkTable aprilTags = sensors.getSubTable("apriltags");

  //   private static NetworkTable camera0 = aprilTags.getSubTable("camera0");
  //     public static NetworkTableEntry camera0_Timestamp = camera0.getEntry("timestamp");

  //     // X, Y, Z
  //     public static NetworkTableEntry camera0_Position = camera0.getEntry("camera_position");
  //     public static NetworkTableEntry camera0_Direction = camera0.getEntry("camera_direction");
      
  //     public static NetworkTableEntry camera0_Distances = camera0.getEntry("distances");
  //     public static NetworkTableEntry camera0_Bearings = camera0.getEntry("bearings");

  //     // Double 
  //     public static NetworkTableEntry camera0_Angle = camera0.getEntry("camera_angle");
      
  //     public static NetworkTableEntry camera0_IDs = camera0.getEntry("ids");

  //     public static NetworkTableEntry camera0_requestedID = camera0.getEntry("target_id");
  //     public static NetworkTableEntry camera0_requestedTimestamp = camera0.getEntry("target_timestamp");
  //     public static NetworkTableEntry camera0_requestedBearing = camera0.getEntry("target_bearing");
  //     public static NetworkTableEntry camera0_requestedDistance = camera0.getEntry("target_distance");

  //   private static NetworkTable camera2 = aprilTags.getSubTable("camera2");
  //     public static NetworkTableEntry camera2_Timestamp = camera2.getEntry("timestamp");

  //     // X, Y, Z
  //     public static NetworkTableEntry camera2_Position = camera2.getEntry("camera_position");
  //     public static NetworkTableEntry camera2_Direction = camera2.getEntry("camera_direction");
      
  //     public static NetworkTableEntry camera2_Distances = camera2.getEntry("distances");
  //     public static NetworkTableEntry camera2_Bearings = camera2.getEntry("bearings");

  //     // Double 
  //     public static NetworkTableEntry camera2_Angle = camera2.getEntry("camera_angle");
      
  //     public static NetworkTableEntry camera2_IDs = camera2.getEntry("ids");

  //     public static NetworkTableEntry camera2_requestedID = camera2.getEntry("target_id");
  //     public static NetworkTableEntry camera2_requestedTimestamp = camera2.getEntry("target_timestamp");
  //     public static NetworkTableEntry camera2_requestedBearing = camera2.getEntry("target_bearing");
  //     public static NetworkTableEntry camera2_requestedDistance = camera2.getEntry("target_distance");

  private static NetworkTable Debug = dash.getSubTable(Constants.NetworktablePaths.Debug); 
    public static NetworkTableEntry runningCommand = Debug.getEntry("Command");
    
  private static NetworkTable misc = dash.getSubTable(Constants.NetworktablePaths.Misc);
    public static NetworkTableEntry driveModeManual_b = misc.getEntry("driveModeManual_b");
    public static NetworkTableEntry fieldOriented_b = misc.getEntry("fieldOriented_b");
    public static NetworkTableEntry autoAimForCorral_b = misc.getEntry("autoAimForCorral_b");
    public static NetworkTableEntry shouldShoot_b = misc.getEntry("shouldShoot_b");
    public static NetworkTableEntry lookTowardsTarget_b = misc.getEntry("lookTowardsTarget_b");
}
  
