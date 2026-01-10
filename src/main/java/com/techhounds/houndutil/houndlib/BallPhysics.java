package com.techhounds.houndutil.houndlib;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class BallPhysics {

    private BallPhysics() {
    }

    private static Translation3d gravityForce(BallConstants c) {
        return new Translation3d(0, 0, -c.mass * c.gravity);
    }

    private static Translation3d dragForce(
            Translation3d v, BallConstants c) {

        double speed = v.getNorm();
        if (speed < 1e-6)
            return new Translation3d();

        double scale = -0.5 * c.rho * c.cd * c.area * speed;
        return v.times(scale);
    }

    private static Translation3d magnusForce(
            Translation3d v, Translation3d omega, BallConstants c) {

        double speed = v.getNorm();
        double wMag = omega.getNorm();
        if (speed < 1e-6 || wMag < 1e-6)
            return new Translation3d();

        double spinRatio = wMag * c.radius / speed;
        double cl = Math.min(c.clGain * spinRatio, c.clMax);

        Translation3d vHat = v.div(speed);
        Translation3d wHat = omega.div(wMag);

        Translation3d direction = new Translation3d(Vector.cross(wHat.toVector(), vHat.toVector()));
        double magnitude = 0.5 * c.rho * cl * c.area * speed * speed;

        return direction.times(magnitude);
    }

    private static Rotation3d integrateRotation(
            Rotation3d current,
            Translation3d omega,
            double dt) {

        Rotation3d delta = new Rotation3d(
                omega.getX() * dt,
                omega.getY() * dt,
                omega.getZ() * dt);

        return current.plus(delta);
    }

    public static void step(
            BallState s, BallConstants c, double dt) {

        Translation3d force = gravityForce(c)
                .plus(dragForce(s.velocity, c))
                .plus(magnusForce(s.velocity, s.omega, c));

        Translation3d accel = force.div(c.mass);

        s.velocity = s.velocity.plus(accel.times(dt));
        s.position = s.position.plus(s.velocity.times(dt));

        double decay = Math.exp(-dt / c.spinDecayTau);
        s.omega = s.omega.times(decay);

        s.orientation = integrateRotation(s.orientation, s.omega, dt);
    }
}