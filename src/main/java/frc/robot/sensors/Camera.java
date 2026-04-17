// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.FieldAprilTags;
import frc.robot.libs.NetworkTables;
import frc.robot.subsystems.odometry.Odometry;

public class Camera extends SubsystemBase {
  private static Camera instance = null;

  private final double delayTime = 5;

  private boolean connected = false;

  private double lastIteration = 0;

  private Pose2d estimatedPose = new Pose2d();

  private PhotonCamera one = new PhotonCamera("one");
  private PhotonCamera two = new PhotonCamera("two");

  private Transform3d oneToBot = new Transform3d(Constants.CameraConstants.offsetToCenterHoriz,
      Constants.CameraConstants.rightOffsetToCenter,
      Constants.CameraConstants.offsetToCenterVert,
      new Rotation3d(0, Constants.CameraConstants.pitch, Math.toRadians(45 + 180)));
  private Transform3d twoToBot = new Transform3d(Constants.CameraConstants.offsetToCenterHoriz,
      Constants.CameraConstants.leftOffsetToCenter,
      Constants.CameraConstants.offsetToCenterVert,
      new Rotation3d(0, Constants.CameraConstants.pitch, Math.toRadians(-45 + 180)));

  private AprilTagFieldLayout layout = FieldAprilTags.getInstance().field;
  private PhotonPoseEstimator oneEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      oneToBot);
  private PhotonPoseEstimator twoEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      twoToBot);

  VisionSystemSim visionSim = new VisionSystemSim("main");
  SimCameraProperties cameraProp = new SimCameraProperties();
  SimCameraProperties cameraPropTwo = new SimCameraProperties();
  PhotonCameraSim oneCameraSim = new PhotonCameraSim(one, cameraProp);
  PhotonCameraSim twoCameraSim = new PhotonCameraSim(two, cameraPropTwo);

  boolean newMeasurement = false;
  boolean calculatRotation = false;

  // private boolean tooFar = false;
  /**
   * Represents a distance measurement obtained from a camera sensor.
   */
  public class AprilTagMeasurement {
    public final double distance;
    public final double yaw;
    public final int id;

    /**
     * Constructs a new DistMeasurement object with the specified distance and
     * ambiguity values.
     * 
     * @param Xdistance the measured distance in the X axis
     * @param Ydistance the measured distance in the Y axis
     * @param yaw       the measured yaw
     * @param id        the fiducial id of the detected apriltag
     */
    public AprilTagMeasurement(double distance, double yaw, int id) {
      this.distance = distance;
      this.yaw = yaw;
      this.id = id;
    }
  }

  /**
   * Represents a camera used for vision processing.
   * 
   * @return instance of Camera class
   */
  public static Camera getInstance() {
    if (instance == null) {
      instance = new Camera();
    }
    return instance;
  }

  /**
   * Represents a camera used for vision processing.
   * This class handles the connection and configuration of the camera.
   *
   * @param PhotonvisionConnectionAttempts The number of connection attempts to
   *                                       make to PhotonVision.
   * @param delayBetweenAttempts           The delay in seconds between each
   *                                       connection attempt.
   * @param minAmbiguity                   The minimum ambiguity value for
   *                                       AprilTags.
   */
  private Camera() {
    visionSim.addAprilTags(layout);
    
    cameraProp.setCalibration(1024, 768, Rotation2d.fromDegrees(129));
    cameraProp.setCalibError(0.25, 0.08);
    cameraProp.setFPS(20);
    cameraProp.setAvgLatencyMs(43);
    cameraProp.setLatencyStdDevMs(5);

    cameraProp.setCalibration(1280, 960, Rotation2d.fromDegrees(129));
    cameraProp.setCalibError(0.25, 0.08);
    cameraProp.setFPS(20);
    cameraProp.setAvgLatencyMs(43);
    cameraProp.setLatencyStdDevMs(5);
    
    visionSim.addCamera(oneCameraSim, oneToBot);
    visionSim.addCamera(twoCameraSim, twoToBot);
    
    oneEstimator.setPrimaryStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
    twoEstimator.setPrimaryStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
  }

  @Override
  public void periodic() {
    if (RobotBase.isSimulation()) {
      visionSim.update(Odometry.getInstance().getRealSimPose());
    }
    if (((Timer.getFPGATimestamp() - lastIteration)) > delayTime) {
      isConnected();

      lastIteration = Timer.getFPGATimestamp();
    }

    oneEstimator.addHeadingData(Timer.getFPGATimestamp(), Odometry.getInstance().getRotation());
    twoEstimator.addHeadingData(Timer.getFPGATimestamp(), Odometry.getInstance().getRotation());
    if (connected || RobotBase.isSimulation()) {
      Pose3d estimatedPoseOne = Pose3d.kZero, estimatedPoseTwo = Pose3d.kZero;
      Pose2d curPose = Odometry.getInstance().getPose();

      List<PhotonPipelineResult> firstCameraResults = one.getAllUnreadResults();
      List<PhotonPipelineResult> secondCameraResults = two.getAllUnreadResults();

      oneEstimator.setReferencePose(curPose);
      twoEstimator.setReferencePose(curPose);

      if (firstCameraResults.size() != 0) {
        for (PhotonPipelineResult result : firstCameraResults) {
          Optional<MultiTargetPNPResult> multiResult = result.getMultiTagResult();
          if (multiResult.isPresent()
              && multiResult.get().estimatedPose.bestReprojErr < Constants.CameraConstants.maxReprojectionError) {
            Optional<EstimatedRobotPose> pose = oneEstimator.update(new PhotonPipelineResult(result.metadata,
                result.getTargets(),
                multiResult));
            if (pose.isPresent()) {
              estimatedPoseOne = pose.get().estimatedPose;
            }

          } else {
            // Update pose one with single tag estimation if it is available
            if (!result.getTargets().isEmpty()
                && result.getTargets().get(0).poseAmbiguity < Constants.CameraConstants.maxAmb) {
              Optional<EstimatedRobotPose> pose = oneEstimator.update(result);
              if (pose.isPresent()) {
                estimatedPoseOne = pose.get().estimatedPose;
              }
            }
          }
        }
      }
      if (secondCameraResults.size() != 0) {
        for (PhotonPipelineResult result : secondCameraResults) {
          Optional<MultiTargetPNPResult> multiResult = result.getMultiTagResult();
          if (multiResult.isPresent()
              && multiResult.get().estimatedPose.bestReprojErr < Constants.CameraConstants.maxReprojectionError) {
            Optional<EstimatedRobotPose> pose = twoEstimator.update(new PhotonPipelineResult(result.metadata,
                result.getTargets(),
                multiResult));
            if (pose.isPresent()) {
              estimatedPoseTwo = pose.get().estimatedPose;
            }
          } else {
            // Update pose two with single tag estimation if it is available
            if (!result.getTargets().isEmpty()
                && result.getTargets().get(0).poseAmbiguity < Constants.CameraConstants.maxAmb) {
              Optional<EstimatedRobotPose> pose = twoEstimator.update(result);
              if (pose.isPresent()) {
                estimatedPoseTwo = pose.get().estimatedPose;
              }
            }
          }
        }
      }

      setDebugPoses(estimatedPoseOne != Pose3d.kZero, estimatedPoseTwo != Pose3d.kZero, estimatedPoseOne,
          estimatedPoseTwo);
      if (estimatedPoseOne != Pose3d.kZero && estimatedPoseTwo != Pose3d.kZero) {
        estimatedPose = new Pose2d(
            (estimatedPoseOne.getX() + estimatedPoseTwo.getX()) / 2,
            (estimatedPoseOne.getY() + estimatedPoseTwo.getY()) / 2,
            new Rotation2d(
                (estimatedPoseOne.getRotation().getAngle() + estimatedPoseTwo.getRotation().getAngle()) / 2));
      } else if (estimatedPoseOne != Pose3d.kZero) {
        estimatedPose = new Pose2d(estimatedPoseOne.getX(), estimatedPoseOne.getY(),
            estimatedPoseOne.getRotation().toRotation2d());
      } else if (estimatedPoseTwo != Pose3d.kZero) {
        estimatedPose = new Pose2d(estimatedPoseTwo.getX(), estimatedPoseTwo.getY(),
            estimatedPoseTwo.getRotation().toRotation2d());
      } else {
        estimatedPose = Pose2d.kZero;
      }
      if (estimatedPose != Pose2d.kZero) {
        Odometry.getInstance().addVisionMeasurement(estimatedPose, Timer.getFPGATimestamp());
        newMeasurement = true;
      }

    }
  }

  public boolean isConnected() {
    connected = (one.isConnected() && two.isConnected());
    return connected;
  }

  public Integer getClosestApriltag() {
    if (connected) {
      PhotonTrackedTarget oneTarget = one.getLatestResult().getBestTarget();
      PhotonTrackedTarget twoTarget = two.getLatestResult().getBestTarget();

      if (oneTarget == null && twoTarget == null) {
        return null;
      } else if (oneTarget == null) {
        return twoTarget.getFiducialId();
      } else if (twoTarget == null) {
        return oneTarget.getFiducialId();
      } else {
        double oneDistance = oneTarget.getBestCameraToTarget().getTranslation().getNorm();
        double twoDistance = twoTarget.getBestCameraToTarget().getTranslation().getNorm();
        return oneDistance < twoDistance ? oneTarget.getFiducialId() : twoTarget.getFiducialId();
      }
    } else {
      return null;
    }
  }

  private void setDebugPoses(boolean one, boolean two, Pose3d onePose, Pose3d twoPose) {
    if (one) {
      NetworkTables.oneCameraPose.setDoubleArray(new double[] {
          onePose.getX(),
          onePose.getY(),
          Math.toDegrees(onePose.getRotation().getZ()) });
      Logger.recordOutput("Odometry/cameraOnePrediction", onePose.toPose2d());
    }
    if (two) {
      NetworkTables.twoCameraPose.setDoubleArray(new double[] {
          twoPose.getX(),
          twoPose.getY(),
          Math.toDegrees(twoPose.getRotation().getZ()) });
      Logger.recordOutput("Odometry/cameraTwoPrediction", twoPose.toPose2d());
    }
  }

  // ignoreRepeats should be true on any getPoseFromCamera call that is not the
  // first call of the method within a rio loop.
  public Pose2d getPoseFromCamera() {
    if (connected || RobotBase.isSimulation()) {
      newMeasurement = false;
      return estimatedPose;
    }

    return null;
  }

  public boolean hasNewMeasurement() {
    return newMeasurement;
  }

  public int[] getDetectedTags() {
    if (connected) {
      List<PhotonTrackedTarget> detectedTagsOne = one.getLatestResult().targets;
      List<PhotonTrackedTarget> detectedTagsTwo = two.getLatestResult().targets;

      if (detectedTagsOne.size() == 0)
        return null;

      int[] detectedTagsInt = new int[detectedTagsOne.size() + detectedTagsOne.size()];

      for (int i = 0; i < detectedTagsOne.size(); i++) {
        detectedTagsInt[i] = (int) detectedTagsOne.get(i).getFiducialId();
      }

      for (int i = 0; i < detectedTagsTwo.size(); i++) {
        detectedTagsInt[i + detectedTagsOne.size()] = (int) detectedTagsTwo.get(i).getFiducialId();
      }

      return detectedTagsInt;
    } else {
      return null;
    }
  }
}

///
///       Red                      <----- Zero Angle                       Blue
///                            <--- 0 degrees    180 degrees ---->     X  <-----
///   ---  +-------------------------------------------------------------------+ (0, 0)
///    ^   |                7  6              |             17 28           29 |  
///    |   |                                  |                             30 |  |
///    |   |                                  |                                |  |
///    |   |                                  |                                |  V
///    |   |                                  |                                |
///    |   |                8  5              |             18 27              |  Y
/// 8.07 m | 16          9       4            |          19       26        31 |
///    |   | 15         10       3            |          20       25        32 |
///    |   |               11  2              |             21 24              |
///    |   |                                  |                                |
///    |   |                                  |                                |
///    |   | 14                               |                                |
///    V   | 13            12  1              |             22 23              |
///   ---  +-------------------------------------------------------------------+
///        |<----------------------------- 16.56 m --------------------------->|
///    
///
/// One:
/// Unprocessed: http://10.31.40.67:1183/stream.mjpg
/// Processed: http://10.31.40.67:1184/stream.mjpg
///
/// Two:
/// Unprocessed: http://10.31.40.67:1185/stream.mjpg
/// Processed http://10.31.40.67:1186/stream.mjpg
///
/// Intake: http://10.31.40.67:1182/stream.mjpg
///
///
