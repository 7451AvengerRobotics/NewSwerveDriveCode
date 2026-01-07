package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class AutoRoutines {
  private final Drive drive;

  public AutoRoutines(Drive drive) {
    System.out.println("AutoRoutines: Constructing AutoRoutines");
    this.drive = drive;
  }

  public Command firstAuto() {
    System.out.println("AutoRoutines: Starting First Auto");
    return Commands.sequence(
        drive.driveToReefFace(new Transform2d(new Translation2d(-0.64, 0), new Rotation2d(Math.PI))),
        drive.followPPPathCommand("Example Path"));
  }

  public Command runVelAuto() {
    return Commands.run(() -> drive.runVelocity(new ChassisSpeeds(1, 0, 0)));
  }
}
