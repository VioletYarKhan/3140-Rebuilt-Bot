// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.climber.Climb;
import frc.robot.commands.swerveDrive.SetSwerveStates;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb_Auto extends SequentialCommandGroup {
  public static enum POSITIONS {
    LEFT,
    RIGHT
  }

  /** Creates a new Climb_Auto. */
  public Climb_Auto(POSITIONS position, int level) {
    level = Math.max(0, Math.min(3, level));

    addCommands(AutoBuilder.pathfindToPose(position == POSITIONS.LEFT ? Constants.PathplannerConstants.climbPoseL
        : Constants.PathplannerConstants.climbPoseR, Constants.PathplannerConstants.pathplannerConstraints)
        .andThen(new SetSwerveStates(SwerveDrive.getInstance())),
        new Climb(Climbers.getInstance(), level));
  }
}
