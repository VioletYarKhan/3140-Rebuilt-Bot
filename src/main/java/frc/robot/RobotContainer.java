// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.Depot;
import frc.robot.commands.auto.L2R_Neutral;
import frc.robot.commands.auto.Pickup_Outpost_Shoot;
import frc.robot.commands.auto.R2L_Neutral;
import frc.robot.commands.auto.SimpleShoot;
import frc.robot.commands.auto.SuperSimpleShoot;
import frc.robot.commands.climber.Extend;
import frc.robot.commands.climber.Retract;
import frc.robot.commands.swerveDrive.Drive;
import frc.robot.commands.swerveDrive.SetSwerveStates;
import frc.robot.commands.turret.FireAway;
import frc.robot.libs.FieldAprilTags;
import frc.robot.libs.FlipPose;
import frc.robot.libs.FuelSim;
import frc.robot.libs.NetworkTables;
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

  public static TestRunner testRunner = TestRunner.getInstance();

  private SendableChooser<String> Path = new SendableChooser<>();


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

    Path.addOption("L-Center-Depot - Premade Auto", "AOK");
    Path.addOption("L-Center - Premade Auto", "AL");
    Path.addOption("L-Double-Center - Premade Auto", "AL2");
    Path.addOption("R-Center - Premade Auto", "AR");
    Path.addOption("R-Double-Center - Premade Auto", "AR2");
    Path.addOption("LR-Center - Premade Auto", "ALR");
    Path.addOption("RL-Center - Premade Auto", "ARL");
    Path.addOption("Outpost - Premade Auto", "AO");
    Path.addOption("Depot - Premade Auto", "AD");
    Path.addOption("Climb - Premade Auto", "AC");
    Path.addOption("Simple Mobility", "SM");
    Path.addOption("Simple Shoot", "SS");
    Path.addOption("Super Simple Shoot", "SSS");
    Path.addOption("L2R Neutral", "L2R");
    Path.addOption("R2L Neutral", "R2L");
    Path.addOption("Outpost", "O");
    Path.addOption("Depot", "D");
    
    NamedCommands.registerCommand("FireAway", new DeferredCommand(()->new FireAway(turret, true), Set.of(turret)));
    NamedCommands.registerCommand("Climb", new Retract());
    NamedCommands.registerCommand("ExtendClimber", new Extend());
    NamedCommands.registerCommand("DeployIntake", new InstantCommand(()->intake.deploy()));
    NamedCommands.registerCommand("StowIntake", new InstantCommand(()->intake.stow()));
    NamedCommands.registerCommand("StartIntake", new InstantCommand(()->intake.intake(Constants.MotorSpeeds.Intake.intakeSpeed)));
    NamedCommands.registerCommand("EndIntake", new InstantCommand(()->intake.intake(0)));

    SmartDashboard.putData("Path", Path);



    NetworkTables.shouldShoot_b.setBoolean(false);
    if (Robot.isSimulation()) {
      FuelSim fuelSim = FuelSim.getInstance();
      fuelSim.spawnStartingFuel(); // spawns fuel in the depots and neutral zone

      // Register a robot for collision and intake simulation
      fuelSim.registerRobot(
          Constants.Bot.botLength, // from left to right in meters
          Constants.Bot.botLength, // from front to back in meters
          Units.inchesToMeters(8.0), // from floor to top of bumpers in meters
          () -> PoseOdometry.getInstance().getRealSimPose(), // Supplier<Pose2d> of robot pose
          () -> SwerveDrive.getInstance()
              .getFieldRelativeSpeeds()); // Supplier<ChassisSpeeds> of field-centric chassis speeds

      // Register an intake to remove fuel from the field as a rectangular bounding box
      fuelSim.registerIntake(
          Constants.Bot.botLength / 2.0,
          Constants.Bot.botLength / 2.0 + Units.inchesToMeters(6.0),
          -Constants.Bot.botLength / 2.0 + Units.inchesToMeters(3.0),
          Constants.Bot.botLength / 2.0 - Units.inchesToMeters(3.0),
          () -> ((Intake.getInstance().isDeployed() || Intake.getInstance().isActive()) && !Intake.getInstance().isFull()),
          (Intake.getInstance()::addBall)
          );

      fuelSim.start(); // enables the simulation to run (updateSim must still be called periodically)

      fuelSim.enableAirResistance();
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    String selectedPath = Path.getSelected();
    if (selectedPath == null)
      selectedPath = "";
    if ("SM".equals(selectedPath))
      return new Drive(1000, false, Constants.Bot.maxChassisSpeed / 2, 0, 0);

    switch (selectedPath) {
      case "AL":
        return new PathPlannerAuto("L Center Pickup");
      case "AOK":
        return new PathPlannerAuto("L Center and Depot");
      case "AL2":
        return new PathPlannerAuto("L Double");
      case "AR":
        return new PathPlannerAuto("R Center Pickup");
      case "AR2":
        return new PathPlannerAuto("R Double");
      case "ALR":
        return new PathPlannerAuto("L R Auto");
      case "ARL":
        return new PathPlannerAuto("R L Auto");
      case "AO":
        return new PathPlannerAuto("Outpost Auto");
      case "AD":
        return new PathPlannerAuto("Depot Auto");
      case "AC":
        return new PathPlannerAuto("Climb and Shoot");
    }

    // If we are going to climb, do NOT append the extra pathfind-to-shoot pose on
    /// L2R/R2L.
    // If we are NOT climbing, we DO append it (original behavior).

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

          autoCommand = new L2R_Neutral().andThen(AutoBuilder
                  .pathfindToPose(FlipPose.flipIfRed(Constants.PathplannerConstants.shootPoseR),
                      Constants.PathplannerConstants.pathplannerConstraints))
                  .andThen(new SetSwerveStates(swerveDrive));

          break;

        case "R2L":
          // Right-to-Left neutral path with shooting

          autoCommand = new R2L_Neutral().andThen(AutoBuilder
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


    if (NetworkTables.shouldShoot_b.getBoolean(false))
      return autoCommand.alongWith(new FireAway(turret));

    return autoCommand;
  }
}
