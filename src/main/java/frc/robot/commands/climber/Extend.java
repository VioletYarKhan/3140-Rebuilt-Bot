package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbers;


public class Extend extends Command {

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    Climbers.getInstance().setMotor(Climbers.climbSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    Climbers.getInstance().setMotor(0);
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Climbers.getInstance().getEncoderDistance() > Climbers.extenededPosition;
  }
}
