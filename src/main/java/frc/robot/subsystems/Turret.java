package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  Drive drive;
  Transform3d turretOffset;
  String name;
  Transform2d turretOffset2d;
  Pose2d turretPos;

  private final List<Fuel> activeFuel = new ArrayList<>();
  Translation2d hub = new Translation2d(11.915, 4.035);

  public Turret(Drive drive, Transform3d turretOffset, String name) {
    this.drive = drive;
    this.turretOffset = turretOffset;
    this.name = name;

    turretOffset2d = new Transform2d(turretOffset.getX(), turretOffset.getY(), new Rotation2d());
    turretPos = drive.getPose().plus(turretOffset2d);
  }

  @Override
  public void periodic() {
    turretPos = drive.getPose().plus(turretOffset2d);

    Logger.recordOutput("ZeroedComponentPoses_" + name, new Pose3d[] {new Pose3d()});
    Logger.recordOutput(
        "FinalPoses_" + name,
        new Pose3d[] {new Pose3d(turretOffset.getTranslation(), new Rotation3d(0, 0, calcYaw()))});

    activeFuel.removeIf(
        fuel -> {
          fuel.update(0.02);
          return fuel.isDead();
        });

    Logger.recordOutput(
        "GamePieces/Fuel_" + name, activeFuel.stream().map(Fuel::getPose).toArray(Pose3d[]::new));
  }

  public double calcYaw() {
    double deltax = hub.getX() - turretPos.getX();
    double deltay = hub.getY() - turretPos.getY();
    double initTheta;
    // Initial theta
    if (deltax == 0) {
      initTheta = 0;
    } else {
      initTheta = Math.atan(deltay / deltax);
    }

    double theta =
        (initTheta - drive.getPose().getRotation().getRadians())
            + Math.PI / 2
            + Math.signum(deltax) * Math.PI / 2;

    

    SmartDashboard.putNumber("Pitch Theta", calcPitch() * 180 / Math.PI);

    return theta;
  }

  public void shootBall() {
    double v0 = calcUnadjustedVelocity();
    double pitch0 = calcUnadjustedPitch();

    activeFuel.add(new Fuel(v0, pitch0, turretPos));
  }

  public Command shootBallCommand() {
    return runOnce(
        () -> {
          shootBall();
        });
  }

  public double calcUnadjustedVelocity() {
    double yf = 1.329;
    double xf =
        Math.sqrt(
            Math.pow((hub.getX() - turretPos.getX()), 2)
                + Math.pow((hub.getY() - turretPos.getY()), 2));
    double g = 9.8;
    double vel = Math.sqrt(g * yf + g * Math.sqrt(Math.pow(xf, 2) + Math.pow(yf, 2))) + 0.5;

    return vel;
  }

  public double calcUnadjustedPitch() {
    double yf = 1.329;
    double xf =
        Math.sqrt(
            Math.pow((hub.getX() - turretPos.getX()), 2)
                + Math.pow((hub.getY() - turretPos.getY()), 2));
    double v = calcVelocity();
    double g = 9.8;
    double a = -g;

    double A = a * Math.pow(xf, 2) / (2 * Math.pow(v, 2));
    double B = xf;
    double C = A - yf;

    double theta = Math.atan((-B - Math.sqrt(Math.pow(B, 2) - 4 * A * C)) / (2 * A));

    return theta;
  }
}
