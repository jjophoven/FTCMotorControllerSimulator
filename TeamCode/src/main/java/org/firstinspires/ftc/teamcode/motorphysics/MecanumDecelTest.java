package org.firstinspires.ftc.teamcode.motorphysics;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 * Drive buttons (toggle on — robot drives until a stop button is pressed):
 *   Dpad Up          - drive FORWARD
 *   Dpad Left        - drive LEFT
 *   Dpad Right       - drive RIGHT
 *
 * Stop mode buttons (press once — latches until next drive button):
 *   A                - COAST       (0.0,     FLOAT)
 *   Dpad Down        - BRAKE       (0.0,     BRAKE)
 *   Left bumper      - NEG TINY    (-0.0001, FLOAT)
 *   Right bumper     - NEG LOW     (-0.2,    FLOAT)
 *
 * Utility:
 *   X                - zero PinPoint odometry + heading
 */
@Config
@TeleOp(name = "Mecanum Decel Test")
public class MecanumDecelTest extends LinearOpMode {

    public static double DRIVE_SPEED = 1.0;

    private DcMotorEx fl, fr, bl, br;
    private GoBildaPinpointDriver pinpoint;

    private double lastVelocityMps = 0.0;

    private double driveFx = 0, driveFy = 0;
    private boolean driving = false;

    private String activeMode = "IDLE";

    private final ElapsedTime loopTimer = new ElapsedTime();

    private void initHardware() {
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        br = hardwareMap.get(DcMotorEx.class, "backRight");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setAllZPB(DcMotor.ZeroPowerBehavior.FLOAT);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
        sleep(300);
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initHardware();

        telemetry.addLine("Ready. Press START.");
        telemetry.update();
        waitForStart();

        loopTimer.reset();

        while (opModeIsActive()) {
            double loopDtMs = loopTimer.milliseconds();
            loopTimer.reset();
            double dtSec = loopDtMs / 1000.0;

            pinpoint.update();
            Pose2D pose       = pinpoint.getPosition();
            double headingRad = pinpoint.getHeading(AngleUnit.RADIANS);
            double velMps     = getPinpointVelocityMps();
            double voltage    = getVoltage();

            double accelMps2 = (dtSec > 0) ? (velMps - lastVelocityMps) / dtSec : 0.0;
            lastVelocityMps  = velMps;

            if (gamepad1.xWasPressed()) {
                pinpoint.resetPosAndIMU();
            }

            if (gamepad1.dpadUpWasPressed()) {
                setDriveDirection(1.0, 0.0); // forward
            }

            if (gamepad1.aWasPressed()) {
                applyStop(0.0, DcMotor.ZeroPowerBehavior.FLOAT);
                activeMode = "COAST  (A)";
                driving = false;
            } else if (gamepad1.dpadDownWasPressed()) {
                applyStop(0.0, DcMotor.ZeroPowerBehavior.BRAKE);
                activeMode = "BRAKE  (Dpad↓)";
                driving = false;
            } else if (gamepad1.leftBumperWasPressed()) {
                applyStop(-0.0001, DcMotor.ZeroPowerBehavior.FLOAT);
                activeMode = "NEG TINY  (LB)";
                driving = false;
            } else if (gamepad1.rightBumperWasPressed()) {
                applyStop(-0.2, DcMotor.ZeroPowerBehavior.FLOAT);
                activeMode = "NEG LOW   (RB)";
                driving = false;
            }

            if (driving) {
                applyDrivePower();
            }

            updateTelemetry(pose, headingRad, velMps, accelMps2, voltage, loopDtMs);
        }
    }

    private void setDriveDirection(double fy, double fx) {
        driveFy  = fy;
        driveFx  = fx;
        driving  = true;
        activeMode = "DRIVING";
        setAllZPB(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void applyDrivePower() {
        double flP =  driveFy + driveFx;
        double frP =  driveFy - driveFx;
        double blP =  driveFy - driveFx;
        double brP =  driveFy + driveFx;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(flP), Math.abs(frP)),
                Math.max(Math.abs(blP), Math.abs(brP))));
        fl.setPower(flP / max * DRIVE_SPEED);
        fr.setPower(frP / max * DRIVE_SPEED);
        bl.setPower(blP / max * DRIVE_SPEED);
        br.setPower(brP / max * DRIVE_SPEED);
    }

    private void applyStop(double power, DcMotor.ZeroPowerBehavior zpb) {
        setAllZPB(zpb);
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    private void setAllZPB(DcMotor.ZeroPowerBehavior zpb) {
        fl.setZeroPowerBehavior(zpb);
        fr.setZeroPowerBehavior(zpb);
        bl.setZeroPowerBehavior(zpb);
        br.setZeroPowerBehavior(zpb);
    }

    private double getPinpointVelocityMps() {
        double vx = pinpoint.getVelX(DistanceUnit.INCH);
        double vy = pinpoint.getVelY(DistanceUnit.INCH);
        return Math.sqrt(vx * vx + vy * vy);
    }

    private double getVoltage() {
        double v = Double.MIN_VALUE;
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            if (vs.getVoltage() > v) v = vs.getVoltage();
        }
        return v;
    }

    private void updateTelemetry(Pose2D pose, double headingRad,
                                 double velMps, double accelMps2,
                                 double voltage, double loopDtMs) {
        telemetry.addData("Mode",          activeMode);
        telemetry.addData("X inches",        pose.getX(DistanceUnit.INCH));
        telemetry.addData("Y inches",        pose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading deg",     Math.toDegrees(headingRad));
        telemetry.addData("Velocity in s",  velMps);
        telemetry.addData("Accel inches s2",    accelMps2);
        telemetry.addData("Voltage volts",     voltage);
        telemetry.addData("Loop milliseconds",       loopDtMs);
        telemetry.update();
    }
}
