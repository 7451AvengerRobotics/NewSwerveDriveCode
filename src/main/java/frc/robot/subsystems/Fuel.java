package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

public class Fuel {
  double pitch;
  double velocity;
  Pose2d initialPos;

  double interval = 0.02;
  double time = 0;
  double accel = 9.8;
  double height = 0.5;
  Translation2d hub = new Translation2d(11.915, 4.035);

  public Fuel(double pitch, double velocity, Pose2d initialPos) {
    this.pitch = pitch;
    this.velocity = velocity;
    this.initialPos = initialPos;
  }

  public Pose3d getPose() {
    double xChange = hub.getX() - initialPos.getX();
    double yChange = hub.getY() - initialPos.getY();
    double yaw = Math.atan2(yChange, xChange);

    double t = time;

    double z = (-0.5 * accel * t * t) + (velocity * Math.sin(pitch) * t) + height;

    double x = initialPos.getX() + velocity * Math.cos(pitch) * Math.cos(yaw) * t;

    double y = initialPos.getY() + velocity * Math.cos(pitch) * Math.sin(yaw) * t;

    return new Pose3d(new Translation3d(x, y, Math.max(z, 0)), new Rotation3d());
  }

  public void update(double dt) {
    time += dt;
  }

  //Removes the ball after __ seconds
  public boolean isDead() {
    return time >= 4;
  }
}
