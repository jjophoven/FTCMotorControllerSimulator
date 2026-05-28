package org.firstinspires.ftc.teamcode.test;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

/**
 * Models what I have experimented FTC motors physics to be like.
 * <p>
 * <pre>
 * a = max(-slipDeceleration,
 *         currentVoltage / appliedVoltage * (maxAcceleration + naturalDeceleration)
 *          - signum(velocity) * naturalDeceleration
 *          - (if signum(velocity) != signum(appliedVoltage) ? velocity * kBackEMF : 0)
 * </pre>
 * Note: slippage is unavoidable in FTC for high speeds.
 * <p>
 * 1. Natural deceleration (+0 or positive voltage) is very slow (~-30in/s/s)
 * <p>
 * 2. Braking with negative voltage actives regenerative braking which is a strong unavoidable force that opposes motion (~-200 in/s at 60 in/s)
 * <p>
 * Goal: given curren velocity and distance remaining,
 * what motorPower should be applied to reach a desired stopping point as fast as possible?
 * <p>
 * Optional: allow velocity and acceleration limits and more than full stops
 * <p>
 */
public class PhysicsSim {
    // Constants (inches)
    private final double dt = 0.015; // 15ms
    private final double maxAccel = 60;
    public final double zeroPowerAcceleration = 30;
    private final double requiredPowerToMove = 0.05;
    private final double slipDecel = 200;
    private final double backEMF = 2.5; // regenerative -inches/s^2 per velocity when opposing motion
    private final double maxSpeed = 80;

    // State
    private double position = 0;
    private double speed = 0;
    private double virtualTime = 0;
    private double motorPower = 0;

    public void setMotorPower(double power) {
        this.motorPower = Math.max(-1, Math.min(1, power));
    }

    public void update() throws InterruptedException {
        double accel = motorPower * (maxAccel + zeroPowerAcceleration);

        if (Math.signum(speed) != Math.signum(motorPower) && Math.abs(speed) > 0.1) {
            accel -= speed * backEMF;
        }

        if (Math.abs(speed) < 1) {
            double staticFriction = maxAccel * requiredPowerToMove;

            if (Math.abs(accel) < staticFriction) {
                accel = 0;
                speed = 0;
            } else {
                accel -= Math.signum(accel) * staticFriction;
            }
        } else {
            accel -= Math.signum(speed) * zeroPowerAcceleration;
        }

        accel = Math.max(-slipDecel, accel);

        speed += accel * dt;
        position += speed * dt;
        speed = Math.max(-maxSpeed, Math.min(maxSpeed, speed));

        virtualTime += dt;
        Thread.sleep((long) (dt*1000));

        logState(accel);
    }


    double predictStoppingDistance(double velocity, double motorPower) {
        double v = velocity;
        double dist = 0.0;

        for (int i = 0; i < 5000; i++) {

            if (Math.abs(v) < 0.001) break;

            double accel = motorPower * (maxAccel + zeroPowerAcceleration);

            // back EMF (same logic as update)
            if (Math.signum(v) != Math.signum(motorPower) && Math.abs(v) > 0.1 || motorPower == 0) {
                accel -= v * backEMF;
            }

             if (Math.abs(v) < 1) {
                double staticFriction = maxAccel * requiredPowerToMove;

                if (Math.abs(accel) < staticFriction) {
                    accel = 0;
                } else {
                    accel -= Math.signum(accel) * staticFriction;
                }

            } else {
                accel -= Math.signum(v) * zeroPowerAcceleration;
            }

            accel = Math.max(-slipDecel, accel);

            double nextV = v + accel * dt;

            if (Math.signum(nextV) != Math.signum(v)) {
                dist += 0.5 * v * dt; // better than full-step approximation
                break;
            }

            dist += 0.5 * (v + nextV) * dt; // trapezoidal integration
            v = nextV;
        }

        return dist;
    }

    public double bangBang(double currentVelocity, double distanceRemaining) {
        double dir = Math.signum(distanceRemaining);
        double stoppingDist = predictStoppingDistance(currentVelocity, -dir);

        if (Math.signum(stoppingDist) == Math.signum(distanceRemaining)
                && Math.abs(stoppingDist) + 0.3 >= Math.abs(distanceRemaining)) {
            return -dir; // brake
        } else {
            return dir;  // accelerate
        }
    }

    public double minimumPowerToMaxDecel(double currentVelocity, double distanceRemaining) {
        // required decel to stop exactly at target
        double desiredDecel =
                (currentVelocity * currentVelocity)
                        / (2.0 * Math.abs(distanceRemaining));

        desiredDecel =
                Math.min(desiredDecel, slipDecel);

        // passive braking already happening
        double passiveDecel =
                zeroPowerAcceleration
                        + Math.abs(currentVelocity) * backEMF;

        double motorDecel =
                Math.max(0, desiredDecel - passiveDecel);

        double power =
                motorDecel
                        / (maxAccel + zeroPowerAcceleration);

        // opposite velocity
        power *= -Math.signum(currentVelocity);

        return Math.max(-1, Math.min(1, power));
    }

    public double minimumBrakePowerBangBang(double currentVelocity,
                                            double distanceRemaining) {

        double dir = Math.signum(distanceRemaining);

        double stoppingDist = predictStoppingDistance(currentVelocity, -dir);

        if (Math.signum(stoppingDist) != Math.signum(distanceRemaining)
                || Math.abs(stoppingDist) + currentVelocity * dt < Math.abs(distanceRemaining)) {

            // accelerate toward target
            return dir;

        } else {
            return minimumPowerToMaxDecel(currentVelocity, distanceRemaining);
        }
    }


    public double stoppingDistance(double v, double brakingPower) {
        double sign = Math.signum(v);
        v = Math.abs(v);
        if (v < 1e-9) return 0.0;

        double k   = backEMF;
        double zPA = zeroPowerAcceleration;
        double sd  = slipDecel;
        double C   = zPA + (-brakingPower * (maxAccel + zPA));

        double totalDist = 0.0;

        // ── Phase 1: slip ────────────────────────────────────────────────────
        double vExit = (Math.abs(k) > 1e-9) ? Math.max((sd - C) / k, 0.0) : Double.MAX_VALUE;

        if (v > vExit) {
            totalDist += (v * v - vExit * vExit) / (2.0 * sd);
            v = vExit;
        }

        if (v < 1e-9) return sign * totalDist;

        // ── Phase 2: drag ODE  dv/dt = -(C + k*v) ───────────────────────────
        if (Math.abs(k) < 1e-6) {
            totalDist += (v * v) / (2.0 * C);
        } else {
            totalDist += v / k - (C / (k * k)) * Math.log1p(k * v / C);
        }

        Logger.recordOutput("stoppingDistance", sign * totalDist);

        return sign * totalDist;
    }

    public double finalVelocityAfterBraking(double initialVelocity,
                                            double distance,
                                            double brakingPower) {

        double sign = Math.signum(initialVelocity);
        double v0 = Math.abs(initialVelocity);
        double absD = Math.abs(distance);

        if (v0 < 1e-9 || absD <= 0.0)
            return initialVelocity;

        // If we stop before reaching the distance, return 0
        double stopDist = Math.abs(stoppingDistance(initialVelocity, brakingPower));
        if (stopDist <= absD)
            return 0.0;

        double vLo = 0.0;
        double vHi = v0;

        // Binary search for velocity whose stopping distance equals remaining distance
        // stoppingDistance(vf) = stoppingDistance(v0) - distance
        double targetRemaining = stopDist - absD;

        for (int i = 0; i < 64; i++) {
            double vMid = (vLo + vHi) * 0.5;

            double dMid = Math.abs(stoppingDistance(vMid, brakingPower));

            if (dMid > targetRemaining)
                vHi = vMid;
            else
                vLo = vMid;

            if ((vHi - vLo) / (vHi + 1e-12) < 1e-6)
                break;
        }

        return ((vLo + vHi) * 0.5) * sign;
    }

    public double stoppingDistanceQuadratic(double v, double brakingPower) {
        int N = 40;
        double maxV = maxSpeed;

        double sumV2 = 0, sumV3 = 0, sumV4 = 0, sumV2d = 0, sumVd = 0;
        double[] vs = new double[N], ds = new double[N];
        for (int i = 0; i < N; i++) {
            double vi = maxV * (i + 1) / N;
            double di = stoppingDistance(vi, brakingPower);
            vs[i] = vi;
            ds[i] = di;
            sumV2  += vi * vi;
            sumV3  += vi * vi * vi;
            sumV4  += vi * vi * vi * vi;
            sumV2d += vi * vi * di;
            sumVd  += vi * di;
        }

        double det = sumV4 * sumV2 - sumV3 * sumV3;
        double a   = (sumV2d * sumV2 - sumVd  * sumV3) / det;
        double b   = (sumV4  * sumVd  - sumV3 * sumV2d) / det;

        // R² and mean absolute error
        double dMean = 0;
        for (double di : ds) dMean += di;
        dMean /= N;

        double ssTot = 0, ssRes = 0, sumAbsErr = 0;
        for (int i = 0; i < N; i++) {
            double predicted = a * vs[i] * vs[i] + b * vs[i];
            double residual  = ds[i] - predicted;
            ssTot    += (ds[i] - dMean) * (ds[i] - dMean);
            ssRes    += residual * residual;
            sumAbsErr += Math.abs(residual);
        }
        double r2  = 1.0 - ssRes / ssTot;
        double mae = sumAbsErr / N;

        Logger.recordOutput("StoppingDist/brakingPower", brakingPower);
        Logger.recordOutput("StoppingDist/a",            a);
        Logger.recordOutput("StoppingDist/b",            b);
        Logger.recordOutput("StoppingDist/R2",           r2);
        Logger.recordOutput("StoppingDist/MAE",          mae);

        double sign = Math.signum(v);
        double vAbs = Math.abs(v);
        return sign * (a * vAbs * vAbs + b * vAbs);
    }

    public double maxVelocityForDistance(double d, double brakingPower) {
        if (d == 0) return 0.0;

        double sign = Math.signum(d);
        double absD = Math.abs(d);

        double C = zeroPowerAcceleration
                + (-Math.abs(brakingPower) * (maxAccel + zeroPowerAcceleration));

        double vLo = 0.0;
        double vHi = Math.sqrt(2.0 * C * absD) * 1.5 + 1.0;

        for (int i = 0; i < 64; i++) {
            double vMid = (vLo + vHi) / 2.0;

            double signedVel = vMid * sign;

            if (Math.abs(stoppingDistance(signedVel, brakingPower)) < absD)
                vLo = vMid;
            else
                vHi = vMid;

            if ((vHi - vLo) / (vHi + 1e-12) < 1e-5) break;
        }

        return ((vLo + vHi) / 2.0) * sign;
    }

    public double maxVelocityForDistanceQuadratic(double d, double brakingPower) {
        double C   = zeroPowerAcceleration + (-brakingPower * (maxAccel + zeroPowerAcceleration));
        double vEst = Math.sqrt(2.0 * C * d);
        double Ceff = C + backEMF * vEst / 3.0;
        return Math.sqrt(2.0 * Ceff * d);
    }

    private void logState(double accel) {
        Logger.recordOutput("sim/power", motorPower);
        Logger.recordOutput("sim/speed", speed);
        Logger.recordOutput("sim/position", position);
        Logger.recordOutput("sim/accel", accel);

        double inchesToMeters = 0.0254;
        Logger.recordOutput("sim/pose", new Pose2d(position * inchesToMeters, 0, new Rotation2d(0)));
    }

    public double getPosition() { return position; }
    public double getSpeed() { return speed; }
    public double getVirtualTime() { return virtualTime; }
    public double getDt() { return dt; }
    
    public void reset() {
        position = 0;
        speed = 0;
        virtualTime = 0;
        motorPower = 0;
    }
}
