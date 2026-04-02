// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.swerveDrive.SetSwerveStates;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Pickup_Outpost extends SequentialCommandGroup {
  private Command pathCommand;

  /** Creates a new Outpost. */
  public Pickup_Outpost() {
    try {
      pathCommand = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Outpost Approach"),
          Constants.PathplannerConstants.pathplannerConstraints);
    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
    }

    this.addCommands(pathCommand.andThen(new SetSwerveStates(SwerveDrive.getInstance()).alongWith(new InstantCommand(() -> Intake.getInstance().deploy()))),
        new WaitCommand(4) /* Then GO CLIMB */);
  }
}
