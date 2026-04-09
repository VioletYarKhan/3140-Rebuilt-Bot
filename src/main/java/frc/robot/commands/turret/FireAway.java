// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.libs.LoggedCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret.TurretMain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FireAway extends LoggedCommand {
  private final SwerveDrive swerve = SwerveDrive.getInstance();
  private final TurretMain turret;


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

    Feeder.getInstance().setFeederInverted(false);
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setFlywheelActive(true);
    Intake.getInstance().intake(Constants.MotorSpeeds.Intake.intakeSpeed);
    if (TurretMain.getInstance().flywheelAtSpeed()) {
      if (Robot.isSimulation()) turret.shootSimFuel();

      if (Robot.isReal())
        Feeder.getInstance().setFeederActive(true);

    } else {
      Feeder.getInstance().setFeederActive(false);
    }

    if(Math.abs(Math.abs(Intake.getInstance().getAngle()) - Constants.Limits.Intake.deployedPosition) < (35.0 / 360.0)) {
      Intake.getInstance().stow();
    }
    if(Math.abs(Math.abs(Intake.getInstance().getAngle()) - Constants.Limits.Intake.feedPosition) < (1.0 / 360.0))  {
      Intake.getInstance().deploy();
    }
    SwerveDrive.getInstance().setSwerveModuleStates(SwerveDrive.getInstance().getModuleStates(), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    Intake.getInstance().intake(0);
    System.out.println("Defiring");
    turret.setFlywheelActive(false);
    Intake.getInstance().deploy();
    super.end(interrupted);

    if (Robot.isReal())
      Feeder.getInstance().setFeederActive(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
