// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.libs.Vector2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM; // Mode.REPLAY to replay

  public static class SIM {
    public static final double odometryDrift = 0.025;

    public static final Pose3d intakeMechOffset = new Pose3d(Units.inchesToMeters(11.120000), 0,
        Units.inchesToMeters(7.254931), new Rotation3d(0, 45, 0));
    public static final Pose3d turretMechOffset = new Pose3d(Units.inchesToMeters(-5.379), 0,
        Units.inchesToMeters(16.112162), new Rotation3d(0, 0, 45));
    public static final Pose3d hoodMechOffset = new Pose3d(Units.inchesToMeters(-6.067574), 0,
        Units.inchesToMeters(19.112162), new Rotation3d(0, 45, 45));
  }

  public static class MotorIDs {
    /* Swerve Drive Motors: */
    // FL
    public static final int FLNeo = 7; // NEO 1
    public static final int FLVortex = 8; // VORTEX 1

    // FR
    public static final int FRNeo = 1; // NEO 2
    public static final int FRVortex = 2; // VORTEX 2

    // BL
    public static final int BLNeo = 5; // NEO 3
    public static final int BLVortex = 6; // VORTEX 3

    // BR
    public static final int BRNeo = 3; // NEO 4
    public static final int BRVortex = 4; // VORTEX 4

    public static final int turretRotation = 9; // NEO 5
    public static final int flywheelMotor = 10; // VORTEX 5
    public static final int hoodMotor = 11; // MINI 1

    public static final int intakeMotor = 12; // MINI 2
    public static final int intakeArmMotor = 13; // NEO 6
    public static final int intakeArmMotorFollower = 14; // NEO 7

    public static final int feederMotor = 17; // NEO 8
    public static final int rollerMotor = 16; // NEO 9

    public static final int climberMotor = 18; // NEO 10
  }

  public static class SensorIDs {
    // Swerve Modules
    public static final int FL = 2;

    public static final int FR = 0;

    public static final int BL = 3;

    public static final int BR = 1;

    // Intake Absolute Encoder
    public static final int intakeEncoderL = 0;
    public static final int intakeEncoderR = 4;

    // Turret Sensors
    public static final int turretEncoderA = 2;
    public static final int turretEncoderB = 3;

    public static final int hoodEncoder = 1;

    public static final int climberEncoderA = 6;
    public static final int climberEncoderB = 7;
  }

  public static class MotorSpeeds {
    public static class Intake {
      public static final double intakeSpeed = 0.75;
      public static final double outtakeSpeed = -0.75;

      public static final double agitateSpeed = 0.1;
    }

    public static class Feeder {
      public static final double feederSpeed = 0.75;
      public static final double rollerSpeed = 0.75;
    }
  }

  public static class PID {
    public static class Turret {
      public static final double hoodP = Robot.isReal() ? 0.0040 : 0.001; // PH TUNED
      public static final double hoodI = Robot.isReal() ? 0.00005 : 0; // PH TUNED
      public static final double hoodD = Robot.isReal() ? 0.00002 : 0.000; // F TUNED

      public static final double turretP = Robot.isReal() ? 0.0050 : 0.0015;// 0.0075;
      public static final double turretI = Robot.isReal() ? 0.0010 : 0;// 0.000;
      public static final double turretD = Robot.isReal() ? 0.00040 : 0;// .00015;

      public static final double turretMaxVel = 360 / 2;
      public static final double turretMaxAcc = 1440 / 2;

      public static final double flywheelP = Robot.isReal() ? 0.0004 : 0;
      public static final double flywheelI = Robot.isReal() ? 0.000 : 0.000;
      public static final double flywheelD = Robot.isReal() ? 0.00004 : 0.000;
    }

    public static class Intake {
      public static final double intakeP = Robot.isReal() ? 1.5 : 0.0035; // TUNED
      public static final double intakeI = Robot.isReal() ? 0.008 : 0.000; // TUNED
      public static final double intakeD = 0.00; // TUNED
    }
  }

  public static class FeedFoward {
    public static class Turret {
      public static final double flywheelS = 0.0; // PHISCIALLY TUNED
      public static final double flywheelV = Robot.isReal() ? 0.0018 : 0.001755; // FISICALY TUNEED
      public static final double flywheelA = 0.0; // TUNED

      public static final double turretS = 0.0;
      public static final double turretV = 0.08;
      public static final double turretA = 0;
    }
  }

  public static class CurrentLimits {
    public static class Turret {
      public static final int flywheelLimit = 80;
      public static final int hoodLimit = 15;
      public static final int turretLimit = 30;
    }

    public static class Intake {
    }

    public static class Feeder {
    }
  }

  public static class Bot {
    public static final double gearRatio = 6.75 / 1.14;
    public static final double steerGearRatio = 150 / 7;
    public static final double botMass = 46.856;
    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double botLength = Units.inchesToMeters(26) + Units.inchesToMeters(2.5);

    // In meters per second, determined from the free speed of the bot via
    // SwerveDriveSpecialties
    public static final double maxChassisSpeed = 5.36448;
    public static final double maxChassisSpeedPathplanner = 4;
    public static final double maxModuleSpeed = maxChassisSpeed / (Math.PI * wheelDiameter);
    public static final double maxTurnSpeed = Double.MAX_VALUE; // These are basically infinite for our purposes
    public static final double maxAcceleration = 19;
    public static final double botRadius = Math.hypot(botLength, botLength);
    // Max Speed divided by the circumference a circle determined by the distance of
    // the module from the center, divided by 2 pi to convert to radians
    public static final double maxChassisTurnSpeed = maxChassisSpeed / botRadius;
    public static final double encoderRotationToMeters = Math.PI * wheelDiameter / gearRatio;

    public static final double turnP = 0.01;
    public static final double turnD = 0.0001;

    /////// AI CODE ///////
    // Simulation-only momentum constants for realistic coast-down behavior
    public static final double simMaxDeceleration = 100; // m/s² for translation coast-down
    public static final double simMaxRotationalDeceleration = Math.toRadians(1200); // rad/s² for rotation
    public static final double simDragCoefficient = 0.05; // Friction/drag coefficient (0.01-0.1)

    // Simulation-only PID tuning (much softer to reduce oscillation in fast sim
    // updates)
    public static final double simDriveP = 0.0005; // Very low P - rely mostly on feedforward
    public static final double simDriveI = 0.0; // No integral term
    public static final double simDriveD = 0.00001; // Minimal D for damping only
    public static final double simTurnP = 0.003; // Reduced turn P for smoother rotation
                                                 /////// END AI CODE ///////

    // Swerve Module Base Angles
    public static final double FLZeroOffset = 343.5;

    public static final double FRZeroOffset = 291.15;

    public static final double BLZeroOffset = 254.45;

    public static final double BRZeroOffset = 247.475;

    public static final double[] lockedAngles = {
        135, // 45 + 90
        225, // 315 + 90
        225, // 315 + 90
        135 // 45 + 90
    };

    // Default swerve state
    // new SwerveModuleState initializes states with 0s for angle and velocity
    public static final SwerveModuleState[] defaultSwerveStates = {
        new SwerveModuleState(0, new Rotation2d(0)),
        new SwerveModuleState(0, new Rotation2d(0)),
        new SwerveModuleState(0, new Rotation2d(0)),
        new SwerveModuleState(0, new Rotation2d(0))
    };

    public static final double intakeGearRatio = 50;
    public static final double turretGearRatio = 95 / 18.0;
    public static final double hoodGearRatio = 1;
    public static final double hoodZeroOffset = 52.856759 + 65.594380;
    public static final double flywheelGearRatio = 10;
  }

  public static class Limits {
    public static class Turret {
      public static final double minPitch = 0; // degrees above horizontal
      public static final double maxPitch = 42.5;
      public static final double maxAngularVelocity = 1000000;// 30; // degrees per second

      public static final double maxFuelVelocity = 11.25;

      public static final double minYaw = -90;
      public static final double maxYaw = 90;
    }

    public static class Intake {
      public static final double deployedPosition = 0.262776;// rotations
      public static final double feedPosition = 0.1;// rotations
      public static final double stowedPosition = 0.0;// 0.93; // rotations
      public static final double leftOffset = 0.82 + 0.1945 + 0.106;// 0.93; // rotations
    }
  }

  public static class Controller {
    public static final int DriverControllerPort = 0;
    public static final int SecondaryDriverControllerPort = 1;

    public static final double triggerThreshold = 0.3;
  }

  public static class Constraints {
  }

  public static class CameraConstants {
    // public static final double maxTimeBeteweenFrames = 0.1;
    public static final double leftOffsetToCenter = Units.inchesToMeters(8.5);
    public static final double rightOffsetToCenter = -Units.inchesToMeters(8.5);
    public static final double offsetToCenterVert = Units.inchesToMeters(8);
    public static final double offsetToCenterHoriz = Units.inchesToMeters(-9);
    public static final double pitch = Math.toRadians(-10);
    public static final double maxReprojectionError = 0.2;
    public static final double maxAmb = 0.3;
    public static final double stdDev = 0.3;
  }

  public static class Odometry {
    public static final double TagCorrectionSpeed = 25;

    public static final double maxCorrectionDistance = 50;
    public static final int startingCameraPasses = 20;
  }

  public static class PathplannerConstants {
    public static RobotConfig config;

    // Field Dimensions - Based on PathPlanner Rebuilt Field
    public static final double FieldLength = 16.540;
    public static final double FieldWidth = 8.070;

    public static final double FuelDiameterInches = 5.91;
    public static final double FuelRadiusInches = FuelDiameterInches / 2;
    public static final double TopOfHubHeightInches = 72;

    public static final Vector2 botTurretOffset = new Vector2(-0.1366, 0); // when the robot is facing positive x at the
                                                                           // origin, position of turret.

    public static final double blueTrenchX = 4.625;
    public static final double redTrenchX = 11.915;

    // Translation PID Values
    public static final double TransP = 6;
    public static final double TransI = .3;
    public static final double TransD = 0;

    // Rotation PID Values
    public static final double RotP = 5;
    public static final double RotI = 0.25;
    public static final double RotD = 0;

    public static final PathConstraints pathplannerConstraints = new PathConstraints(
        Constants.Bot.maxChassisSpeedPathplanner,
        1, // 4
        Units.degreesToRadians(540),
        Units.degreesToRadians(720));

    public static final Pose2d shootPoseL = new Pose2d(1, 7, new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d shootPoseR = new Pose2d(1, 1, new Rotation2d(Units.degreesToRadians(0)));

    public static final Pose2d climbPoseL = new Pose2d(1.5, 4.175, new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d climbPoseR = new Pose2d(1.5, 3.319, new Rotation2d(Units.degreesToRadians(0)));

    public static final double redAllianceShootPreventionX = 11.9;
    public static final double blueAllianceShootPreventionX = 4.65;
    public static final double neutralZoneDivision = 4.65;
    public static final double middleY = FieldWidth / 2;
  }

  public static class NetworktablePaths {
    public static final String Dashboard = "Dashboard";

    public static final String Sensors = "sensors3140";

    // Subtables of Dashboard
    public static final String DS = "DS";
    public static final String Voltage = "Voltage";
    public static final String Pose = "Pose";
    public static final String Test = "Dev";
    public static final String Debug = "Debug";
    public static final String Misc = "Misc";
  }

  public static class LED {
    public static final int Port = 0;
    public static final int LEDCount = 76;
    public static final int RainbowSpan = 5;
  }

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
