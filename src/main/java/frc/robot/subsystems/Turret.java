package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  Drive drive;

  private final List<Fuel> activeFuel = new ArrayList<>();

  public Turret(Drive drive) {
    this.drive = drive;
  }

  @Override
  public void periodic() {
    Translation3d leftShooterOffset = new Translation3d(-0.17, 0.15, 0.39);
    Translation3d rightShooterOffset = new Translation3d(-0.17, -0.15, 0.39);
    Logger.recordOutput("ZeroedComponentPoses", new Pose3d[] {new Pose3d()});
    Logger.recordOutput(
        "FinalPoses",
        new Pose3d[] {
          new Pose3d(
              leftShooterOffset,
              new Rotation3d(
                  0,
                  0,
                  trackHub(
                      drive.getPose(),
                      new Transform2d(
                          leftShooterOffset.getX(), leftShooterOffset.getY(), new Rotation2d())))),
          new Pose3d(
              rightShooterOffset,
              new Rotation3d(
                  0,
                  0,
                  trackHub(
                      drive.getPose(),
                      new Transform2d(
                          rightShooterOffset.getX(), rightShooterOffset.getY(), new Rotation2d()))))
        });

    activeFuel.removeIf(fuel -> {
        fuel.update(0.02);
        return fuel.isDead();
    });

    Logger.recordOutput(
        "GamePieces/Fuel", activeFuel.stream().map(Fuel::getPose).toArray(Pose3d[]::new));
  }

  public double trackHub(Pose2d robotPose, Transform2d turretOffset) {
    Translation2d hub = new Translation2d(11.915, 4.035);
    Pose2d turretPose = robotPose.plus(turretOffset);

    double deltax = hub.getX() - turretPose.getX();
    double deltay = hub.getY() - turretPose.getY();

    // Initial theta
    double initTheta = Math.atan(deltay / deltax);

    double theta =
        (initTheta - robotPose.getRotation().getRadians())
            + Math.PI / 2
            + Math.signum(deltax) * Math.PI / 2;

    SmartDashboard.putNumber("Turret Theta", theta * 180 / Math.PI);

    return theta;
  }

  public void shootBall() {
    Transform2d turretOffset = new Transform2d(-0.17, -0.15, new Rotation2d());
    activeFuel.add(new Fuel(calcPitch(), calcVelocity(), drive.getPose().plus(turretOffset)));
  }

  public Command shootBallCommand() {
    return runOnce(
      () -> {
        shootBall();
      });
  }

  public double calcVelocity() {
    return 6;
  }

  public double calcPitch() {
    return Math.PI / 4;
  }
}
