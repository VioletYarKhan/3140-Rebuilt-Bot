// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret.TurretMain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FireAway extends Command {
  private final SwerveDrive swerve = SwerveDrive.getInstance();
  private final TurretMain turret;

  // --- Simulation-only rate limiting (non-blocking) ---
  private static final double SIM_SHOT_DELAY_SEC = 0.25; // tune as desired
  private double lastSimShotTimeSec = -1.0;

  /**
   * Creates a new Unload.
   * 
   * @param turret The turret subsystem
   * 
   *               <p>
   *               Simply shoots <b><i>UNTIL CANCELLED</i></b>.
   */
  public FireAway(TurretMain turret) {
    this.turret = turret;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.turret);
  }

  boolean bypassDistance = false;

  public FireAway(TurretMain turret, boolean bypassDistance) {
    this(turret);
    this.bypassDistance = bypassDistance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.setFlywheelActive(true);

    // reset sim timing gate
    lastSimShotTimeSec = -1.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Shooting, " + bypassDistance);
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      // BLUE ALLIANCE --> is closer to x = 0
      if (swerve.getPose().getX() < Constants.PathplannerConstants.blueAllianceShootPreventionX || bypassDistance) {

        if (Robot.isSimulation())
          tryShootSimFuel();

        if (Robot.isReal())
          Feeder.getInstance().setFeederActive(true);

      } else {
        Feeder.getInstance().setFeederActive(false);
      }
    } else {
      // RED ALLIANCE
      if (swerve.getPose().getX() > Constants.PathplannerConstants.redAllianceShootPreventionX || bypassDistance) {

        if (Robot.isSimulation())
          tryShootSimFuel();

        if (Robot.isReal())
          Feeder.getInstance().setFeederActive(true);

      } else {
        Feeder.getInstance().setFeederActive(false);
      }
    }
  }

  /**
   * Simulation-only: shoots a simulated fuel at most once every
   * SIM_SHOT_DELAY_SEC.
   * Non-blocking (does not use Timer.delay()).
   */
  private void tryShootSimFuel() {
    double now = Timer.getFPGATimestamp();

    // First shot happens immediately after entering the valid region
    if (lastSimShotTimeSec < 0.0 || (now - lastSimShotTimeSec) >= SIM_SHOT_DELAY_SEC) {
      turret.shootSimFuel();
      lastSimShotTimeSec = now;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setFlywheelActive(false);

    if (Robot.isReal())
      Feeder.getInstance().setFeederActive(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
