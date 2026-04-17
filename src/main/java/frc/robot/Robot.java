// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.swerveDrive.SetSwerveStates;
import frc.robot.commands.swerveDrive.SwerveDriveManualControl;
import frc.robot.libs.FuelSim;
import frc.robot.libs.NetworkTables;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.TestRunner;
import frc.robot.subsystems.Turret.TurretMain;
import frc.robot.subsystems.odometry.NavXSim;
import frc.robot.subsystems.odometry.PoseOdometry;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final TestRunner m_testRunner;

  public static boolean enabled = false;

  @AutoLogOutput(key = "Mecanisms")
  public static Pose3d mecanismPoses[] = new Pose3d[] {
      Constants.SIM.intakeMechOffset,
      Constants.SIM.turretMechOffset,
      Constants.SIM.hoodMechOffset,
  };

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    Logger.recordMetadata("ProjectName", "2026-Rebuilt-Bot-3140"); // Set a metadata value

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_testRunner = TestRunner.getInstance();
    m_robotContainer = RobotContainer.getInstance();

    // Pose2d pose = Odometry.getInstance().getPose();

    // Make sure the path planner config has been setup (which happens when swerve
    // drive is initialized)
    SwerveDrive.getInstance();

    // Get Pathplanner ready for autos
    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    NetworkTables.voltage_d.setDouble(RobotController.getBatteryVoltage());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

    enabled = false;
    m_testRunner.stopAll();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    enabled = true;
    m_testRunner.stopAll();

    RobotContainer.swerveDrive.setDefaultCommand(new SetSwerveStates(RobotContainer.swerveDrive));

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    NetworkTables.state_s.setString("AUTO");
    PoseOdometry.getInstance().cameraPasses = 100;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  public static boolean locked = false;

  @Override
  public void teleopInit() {
    enabled = true;
    m_testRunner.stopAll();

    RobotContainer.swerveDrive.setDefaultCommand(new SwerveDriveManualControl(RobotContainer.swerveDrive,
        Constants.Bot.maxChassisSpeed, Constants.Bot.maxChassisTurnSpeed, true, locked));
    TurretMain.flywheelRPMOverride = false;
    TurretMain.hoodAngleOverride = false;


    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    NetworkTables.state_s.setString("TELEOP");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    m_testRunner.stopAll();
    enabled = true;

    RobotContainer.swerveDrive.setDefaultCommand(new SetSwerveStates(RobotContainer.swerveDrive));

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    NetworkTables.state_s.setString("DEV");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_testRunner.updateStates();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    final double dt = 0.02;
    final double vbus = 12.0;

    // motor free speeds (RPM)
    final double vortexFreeRPM = 6784.0; // drive (Spark Flex + NEO Vortex)
    final double neoFreeRPM = 5676.0; // steer (Spark MAX + NEO)

    for (SwerveModule module : SwerveDrive.getInstance().modules) {

      // --- DRIVE ---
      double driveDuty = module.simDriveMotor.getAppliedOutput();
      double driveRPM = driveDuty * vortexFreeRPM;
      module.simDriveMotor.iterate(driveRPM, vbus, dt);

      // --- STEER ---
      double turnDuty = module.simTurnMotor.getAppliedOutput();
      double turnRPM = turnDuty * neoFreeRPM;
      module.simTurnMotor.iterate(turnRPM, vbus, dt);

      // update custom absolute encoder (convert motor rotations → wheel degrees)
      double wheelRotations = module.simTurnMotor.getPosition() / Constants.Bot.steerGearRatio;
      double angleDeg = wheelRotations * 360.0;

      // normalize to [0, 360)
      angleDeg = ((angleDeg % 360.0) + 360.0) % 360.0;

      module.turnEncoder.setDistance(angleDeg);
    }

    // --- SIMULATED INTAKE ---
    double intakeDuty = Intake.getInstance().intakeArmMotorSim.getAppliedOutput();
    double intakeRPM = intakeDuty * neoFreeRPM;
    Intake.getInstance().intakeArmMotorSim.iterate(intakeRPM, vbus, dt);

    double intakeRotations = Intake.getInstance().intakeArmMotorSim.getPosition() / Constants.Bot.intakeGearRatio;
    double intakeAngleDeg = intakeRotations * 360.0;
    Intake.getInstance().intakeEncoderSim.set(intakeAngleDeg);

    // --- SIMULATED TURRET ---
    double turretDuty = TurretMain.getInstance().turretRotationMotorSim.getAppliedOutput();
    double turretRPM = turretDuty * neoFreeRPM;
    TurretMain.getInstance().turretRotationMotorSim.iterate(-turretRPM, vbus, dt);

    double hoodDuty = TurretMain.getInstance().hoodMotorSim.getAppliedOutput();
    double hoodRPM = hoodDuty * neoFreeRPM;
    TurretMain.getInstance().hoodMotorSim.iterate(hoodRPM, vbus, dt);

    double flywheelDuty = TurretMain.getInstance().flywheelMotorSim.getAppliedOutput();
    double flyWheelRPM = flywheelDuty * vortexFreeRPM;
    TurretMain.getInstance().flywheelMotorSim.iterate(flyWheelRPM, vbus, dt);

    double turretRotations = TurretMain.getInstance().turretRotationMotorSim.getPosition()
        / Constants.Bot.turretGearRatio;
    double turretAngleDeg = turretRotations * 360.0;
    turretAngleDeg = ((turretAngleDeg % 360.0) + 360.0) % 360.0;
    TurretMain.getInstance().turretEncoderSim.setDistance(turretAngleDeg);

    double hoodRotations = TurretMain.getInstance().hoodMotorSim.getPosition() / Constants.Bot.hoodGearRatio;
    double hoodAngleDeg = hoodRotations * 360.0;
    TurretMain.getInstance().hoodEncoderSim.set(hoodAngleDeg);

    TurretMain.getInstance().simFuel(dt);

    // --- SIMULATED GYRO ---
    // compute chassis speeds from wheel positions
    SwerveModuleState[] positions = SwerveDrive.getInstance().getModuleStates();
    ChassisSpeeds speeds = SwerveDrive.getInstance().kinematics.toChassisSpeeds(positions);
    NavXSim.getInstance().update(speeds.omegaRadiansPerSecond, dt);

    FuelSim.getInstance().updateSim();
  }

}
