package org.firstinspires.ftc.teamcode.test;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGServer;

import static org.junit.jupiter.api.Assertions.assertTrue;

class SimulationTest {
    private static final RLOGServer server = new RLOGServer();
    private PhysicsSim sim;
    private FakeMotor motor;

    @BeforeEach
    void setUp() throws InterruptedException {
        sim = new PhysicsSim();
        motor = new FakeMotor(sim);
        
        Logger.addDataReceiver(server);
        Logger.start();
        Logger.setTimeSource(sim::getVirtualTime);

        Thread.sleep(1000);
    }

    @AfterEach
    void tearDown() {
        Logger.end();
    }

    @Test
    void bangBang() throws InterruptedException {
        runSimulation((currentPos, currentSpeed, target) -> sim.bangBang(currentSpeed, target - currentPos));
    }


    @Test
    void minimumPowerBangBang() throws InterruptedException {
        runSimulation((currentPos, currentSpeed, target) -> sim.minimumBrakePowerBangBang(currentSpeed, target - currentPos));
    }

    @Test
    void proportional() throws InterruptedException {
        runSimulation((currentPos, currentSpeed, target) -> {
            double error = target - currentPos;
            return error * 0.5;
        });
    }

    @Test
    void PD() throws InterruptedException {
        runSimulation((currentPos, currentSpeed, target) -> {
            double error = target - currentPos;
            return error * 0.5 - currentSpeed * 0.08;
        });
    }

    @Test
    void predictiveBrake20Percent() throws InterruptedException {
        runSimulation((currentPos, currentSpeed, target) -> {
            double error = target - currentPos - sim.predictStoppingDistance(currentSpeed,-0.2) - 0.2;

            return error * 0.5;
        });
    }

    @Test
    void predictiveBrakeMax() throws InterruptedException {
        runSimulation((currentPos, currentSpeed, target) -> {
            double error = target - currentPos - sim.predictStoppingDistance(currentSpeed, -1) - (1-sim.minimumPowerToMaxDecel(currentSpeed, target - currentPos));

            return error * 0.5;
        });
    }

    @Test
    // the robot can brake faster than using 0 power
    void predictiveBrakeZero() throws InterruptedException {
        runSimulation((currentPos, currentSpeed, target) -> {
            double error = target - currentPos - sim.stoppingDistance(currentSpeed,0);

            return error * 0.5;
        });
    }

    @Test
    void predictiveQuadraticBrakeZero() throws InterruptedException {
        runSimulation((currentPos, currentSpeed, target) -> {
            double error = target - currentPos - sim.stoppingDistanceQuadratic(currentSpeed,0);

            return error * 0.5;
        });
    }

    @Test
    void predictiveVelocitySpace() throws InterruptedException {
        runSimulation((currentPos, currentSpeed, target) -> {
            double error = target - currentPos;
            double targetVel = sim.maxVelocityForDistance(error, -0.2);

            double finalMomentumVelocity = Math.signum(error) *
                    Math.sqrt(currentSpeed * currentSpeed
                            + 2.0 * sim.zeroPowerAcceleration * Math.abs(error));

            finalMomentumVelocity = Math.max(0, -finalMomentumVelocity);
            double finalBrakedVelocity = sim.finalVelocityAfterBraking(currentSpeed, error, -0.2);

            Logger.recordOutput("sim/error", error);
            Logger.recordOutput("sim/targetVel", targetVel);
            Logger.recordOutput("sim/finalMomentumVelocity", finalMomentumVelocity);
            Logger.recordOutput("sim/finalBrakedVelocity", finalBrakedVelocity);
            Logger.recordOutput("sim/(targetVel - finalMomentumVelocity)", (targetVel - finalMomentumVelocity));
            Logger.recordOutput("sim/targetVelPerError", targetVel/error);

            return (targetVel*2 - currentSpeed - finalBrakedVelocity) * (0.0125);
        });
    }


    @Test
    void predictiveVelocitySpace2() throws InterruptedException {
        runSimulation((currentPos, currentSpeed, target) -> {
            double error = target - currentPos;
            double targetVel = sim.maxVelocityForDistance(error, -0.2);

            Logger.recordOutput("sim/error", error);
            Logger.recordOutput("sim/targetVel", targetVel);
            Logger.recordOutput("sim/targetVelPerError", targetVel/error);

            return (targetVel*2 - currentSpeed) * (0.0125);
        });
    }


    @Test
    @DisplayName("Heuristic Braking Control Test")
    void testHeuristicControl() throws InterruptedException {
        runSimulation((currentPos, currentSpeed, target) -> {

            Logger.recordOutput("sim/target", target);
            double error = target - currentPos;
            double sign = Math.signum(error);
            double stoppingDistance = 0.2 * currentSpeed;

            if (Math.abs(error) < 0.5 && Math.abs(currentSpeed) < 1.0) {
                return 0.0;
            } else if (Math.signum(currentSpeed) == sign && stoppingDistance >= Math.abs(error)) {
                return -sign * 0.2; // Gentle brake
            } else {
                return sign; // Full power
            }
        });
    }

    /**
     * Functional interface for a simple controller
     */
    interface Controller {
        double calculatePower(double currentPos, double currentSpeed, double target);
    }

    private void runSimulation(Controller controller) throws InterruptedException {
        double target = 64;
        while (true) {
            Logger.periodicBeforeUser();
            
            double power = controller.calculatePower(sim.getPosition(), sim.getSpeed(), target);
            motor.setPower(power);
            
            sim.update();
            
            Logger.recordOutput("test/target", target);
            Logger.periodicAfterUser(0, 0);

            // Exit condition: low speed and near target
            if (Math.abs(target - sim.getPosition()) < 0.5 && Math.abs(sim.getSpeed()) < 0.1) {
                break;
            }
        }
    }

    private void assertAtTarget(double target) {
        assertTrue(Math.abs(target - sim.getPosition()) < 1.0, 
            String.format("Failed to reach target. Target: %.2f, Actual: %.2f", target, sim.getPosition()));
    }
}
