// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.Climb_Auto;
import frc.robot.commands.auto.Depot;
import frc.robot.commands.auto.L2R_Neutral;
import frc.robot.commands.auto.Pickup_Outpost_Shoot;
import frc.robot.commands.auto.R2L_Neutral;
import frc.robot.commands.auto.SimpleShoot;
import frc.robot.commands.auto.SuperSimpleShoot;
import frc.robot.commands.swerveDrive.Drive;
import frc.robot.commands.swerveDrive.SetSwerveStates;
import frc.robot.commands.turret.FireAway;
import frc.robot.libs.FieldAprilTags;
import frc.robot.libs.FlipPose;
import frc.robot.libs.FuelSim;
import frc.robot.libs.NetworkTables;
import frc.robot.libs.Vector2;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.TestRunner;
import frc.robot.subsystems.Turret.TurretMain;
import frc.robot.subsystems.odometry.Odometry;
import frc.robot.subsystems.odometry.PoseOdometry;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static RobotContainer container = null;

  // The robot's subsystems and commands are defined here...
  public static SwerveDrive swerveDrive = SwerveDrive.getInstance();
  public static FieldAprilTags fieldAprilTags = FieldAprilTags.getInstance();
  public static Camera camera = Camera.getInstance();
  public static Odometry odometry = Odometry.getInstance();
  public static Controller controller = Controller.getInstance();
  public static TurretMain turret = TurretMain.getInstance();
  public static Feeder feeder = Feeder.getInstance();
  public static Intake intake = Intake.getInstance();
  public static Climbers climber = Climbers.getInstance();
  public static FuelSim fuelSim = new FuelSim("physicFuel");

  public static TestRunner testRunner = TestRunner.getInstance();

  private SendableChooser<String> Path = new SendableChooser<>();

  private SendableChooser<Supplier<Command>> Climb = new SendableChooser<>();

  // Get the singleton instance or create it if it doesn't exist
  public static RobotContainer getInstance() {
    if (container == null) {
      container = new RobotContainer();
    }
    return container;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    Path.setDefaultOption("Normal - No PathPlanner", null);
    Path.addOption("Simple Mobility", "SM");
    Path.addOption("Simple Shoot", "SS");
    Path.addOption("Super Simple Shoot", "SSS");
    Path.addOption("L2R Neutral", "L2R");
    Path.addOption("R2L Neutral", "R2L");
    Path.addOption("Outpost", "O");
    Path.addOption("Depot", "D");
    Path.addOption("L-Center - Premade Auto", "AL");

    SmartDashboard.putData("Path", Path);

    Climb.setDefaultOption("Normal - No Climber", null);
    Climb.addOption("L1", () -> new Climb_Auto(Climb_Auto.POSITIONS.LEFT, 1));
    Climb.addOption("L2", () -> new Climb_Auto(Climb_Auto.POSITIONS.LEFT, 2));
    Climb.addOption("L3", () -> new Climb_Auto(Climb_Auto.POSITIONS.LEFT, 3));
    Climb.addOption("R1", () -> new Climb_Auto(Climb_Auto.POSITIONS.RIGHT, 1));
    Climb.addOption("R2", () -> new Climb_Auto(Climb_Auto.POSITIONS.RIGHT, 2));
    Climb.addOption("R3", () -> new Climb_Auto(Climb_Auto.POSITIONS.RIGHT, 3));

    SmartDashboard.putData("Climb", Climb);

    NetworkTables.shouldShoot_b.setBoolean(false);
    if (Robot.isSimulation()) {
      fuelSim.spawnStartingFuel(); // spawns fuel in the depots and neutral zone

      // Register a robot for collision with fuel
      fuelSim.registerRobot(
          Constants.Bot.botLength, // from left to right in meters
          Constants.Bot.botLength, // from front to back in meters
          4, // from floor to top of bumpers in meters
          () -> { return PoseOdometry.getInstance().getRealSimPose(); } , // Supplier<Pose2d> of robot pose
          () -> {
            Vector2 velocity = odometry.getBotVelocity(true);
            return new ChassisSpeeds(
                velocity.X * 0, 
                velocity.Y * 0, 
                odometry.getAngularVelocity() * 0);
          }); // Supplier<ChassisSpeeds> of field-centric chassis speeds

      // Register an intake to remove fuel from the field as a rectangular bounding box
      fuelSim.registerIntake(Constants.Bot.botLength / 2, Constants.Bot.botLength / 2 + Units.inchesToMeters(6) , 
                          -Constants.Bot.botLength / 2 + Units.inchesToMeters(3), Constants.Bot.botLength / 2 - Units.inchesToMeters(3), () ->{ return Intake.getInstance().isDeployed() || Intake.getInstance().isActive();});


      fuelSim.start(); // enables the simulation to run (updateSim must still be called periodically)
                       //     fuelSim.stop(); // stops the simulation running (updateSim will do nothing until start is called again)

      fuelSim.enableAirResistance(); // an additional drag force will be applied to fuel in physics update step
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    String selectedPath = Path.getSelected();

    if ("SM".equals(selectedPath))
      return new Drive(1000, false, Constants.Bot.maxChassisSpeed / 2, 0, 0);

    if ("AL".equals(selectedPath))
      return new PathPlannerAuto("L Center Pickup");

    // If we are going to climb, do NOT append the extra pathfind-to-shoot pose on
    /// L2R/R2L.
    // If we are NOT climbing, we DO append it (original behavior).
    Supplier<Command> climbSupplier = Climb.getSelected();
    boolean willClimb = (climbSupplier != null);

    SequentialCommandGroup autoCommand = new SequentialCommandGroup();
    if (NetworkTables.shouldShoot_b.getBoolean(false)) {

      switch (selectedPath != null ? selectedPath : "") {

        case "SS":
          // Simple Shoot
          autoCommand = new SimpleShoot();
          break;
        case "SSS":
          autoCommand = new SuperSimpleShoot();
          break;

        case "L2R":
          // Left-to-Right neutral path with shooting

          autoCommand = willClimb
              ? new L2R_Neutral()
              : new L2R_Neutral().andThen(AutoBuilder
                  .pathfindToPose(FlipPose.flipIfRed(Constants.PathplannerConstants.shootPoseR),
                      Constants.PathplannerConstants.pathplannerConstraints))
                  .andThen(new SetSwerveStates(swerveDrive));

          break;

        case "R2L":
          // Right-to-Left neutral path with shooting

          autoCommand = willClimb
              ? new R2L_Neutral()
              : new R2L_Neutral().andThen(AutoBuilder
                  .pathfindToPose(FlipPose.flipIfRed(Constants.PathplannerConstants.shootPoseL),
                      Constants.PathplannerConstants.pathplannerConstraints))
                  .andThen(new SetSwerveStates(swerveDrive));

          break;

        case "O":
          // Outpost pickup then shoot
          autoCommand = new Pickup_Outpost_Shoot();
          break;

        case "D":
          // Depot autonomous with shooting
          autoCommand = new Depot();
          break;

        default:
          // No autonomous selected / fallback
          return autoCommand;
      }
    } else {

      switch (selectedPath != null ? selectedPath : "") {

        case "SS":
          // Simple Shoot
          autoCommand = new SimpleShoot();
          break;

        case "SSS":
          autoCommand = new SuperSimpleShoot();
          break;

        case "L2R":
          // Left-to-Right neutral path with shooting
          autoCommand = new L2R_Neutral();
          break;

        case "R2L":
          // Right-to-Left neutral path with shooting
          autoCommand = new R2L_Neutral();
          break;

        case "O":
          // Outpost pickup then shoot
          autoCommand = new Pickup_Outpost_Shoot();
          break;

        case "D":
          // Depot autonomous with shooting
          autoCommand = new Depot();
          break;

        default:
          // No autonomous selected / fallback
          return autoCommand;
      }
    }

    // Re-use the same climbSupplier computed above (factory), and instantiate a
    /// fresh command now.
    if (willClimb && autoCommand != null) {
      autoCommand = autoCommand.andThen(climbSupplier.get());
    }

    if (NetworkTables.shouldShoot_b.getBoolean(false))
      return autoCommand.alongWith(new FireAway(turret));

    return autoCommand;
  }
}
