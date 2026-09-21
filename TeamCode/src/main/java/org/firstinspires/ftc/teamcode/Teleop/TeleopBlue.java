package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveToPoint;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

import java.util.HashMap;

@TeleOp(name = "TeleOp Blue", group = "Test")
public class TeleopBlue extends LinearOpMode {
    private Robot robot;
    // Hardware
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor turretMotor;
    DcMotor intakeMotor;
    Servo  lift;
    DcMotorEx flywheel;
    private VoltageSensor battery;
    private ElapsedTime timer;

    // Motor constants
    // 6000 RPM × 28 / 60 ≈ 2800 ticks/sec
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_RPM_FAR = 3800.0;
    private static final double TARGET_RPM_NEAR = 2900.0;

    // Voltage compensation constants
    private static final double REFERENCE_VOLTAGE = 12.0;

    // Base PIDF (tuned at REFERENCE_VOLTAGE)
    private static final double BASE_P = 20.0;
    private static final double BASE_I = 0.0;
    private static final double BASE_D = 2.0;
    private static final double BASE_F = 11.7;

    // Runtime
    private void initOpMode() {
        frontLeftMotor = hardwareMap.dcMotor.get("fl"); //1 port
        backLeftMotor = hardwareMap.dcMotor.get("rl");  //0
        frontRightMotor = hardwareMap.dcMotor.get("fr");    //3
        backRightMotor = hardwareMap.dcMotor.get("rr"); //2
        turretMotor = hardwareMap.dcMotor.get("turretMotor"); // ext 1
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor"); // ext 3
        lift = hardwareMap.get(Servo.class, "lift"); // ext 0 servo
        flywheel = hardwareMap.get(DcMotorEx.class, "shootMotor");  // ext 0
        battery = hardwareMap.voltageSensor.iterator().next();

        // to create robot
        timer = new ElapsedTime();
        DriveToPoint nav = new DriveToPoint(this);
        HashMap<String, Boolean> flags = new HashMap<>();
        flags.put("web", true);
        flags.put("vision", false);
        this.robot = new Robot(hardwareMap, telemetry, timer, nav, AllianceColor.BLUE, gamepad1, gamepad2, flags);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void runOpMode() throws InterruptedException{
        initOpMode();
        waitForStart();

        double lastVoltage = 0;
        lastVoltage = battery.getVoltage();

        boolean shooterActive = false;
        boolean prevA = false;
        double distToTarget = 150.0;

        while (opModeIsActive()) {
            // ===== Drive Only - Start =====
            // Reverse the right side motors. This may be wrong for your setup.
            // If your robot moves backwards when commanded to go forwards,
            // reverse the left side instead.
            // See the note about this earlier on this page.
            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            double sensitivity = 0.6; // less sensitive, 0.5=half speed
            double y = -gamepad2.left_stick_y * sensitivity; // Remember, Y stick value is reversed
            double x = gamepad2.left_stick_x * 1.1 * sensitivity; // Counteract imperfect strafing
            double rx = gamepad2.right_stick_x * sensitivity;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            // ===== Drive Only - Ends =====

            double voltage = battery.getVoltage();

            // update PIDF when voltage meaningfully changes
            // PID stands for Proportional, Integral, Derivative. These three terms control how the motor responds
            // to error (difference between target and actual value).
            // PIDF adds a Feedforward ("F") term. The F term predicts the needed output based on the target value,
            // helping the motor reach the target faster and more accurately, especially for velocity control.
            if (Math.abs(voltage - lastVoltage) > 0.1) {
                double scaledF = BASE_F * (REFERENCE_VOLTAGE / voltage);
                flywheel.setPIDFCoefficients(
                        DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(BASE_P, BASE_I, BASE_D, scaledF)
                );
                lastVoltage = voltage;
            }

            // --- Distance-based flywheel velocity --
            robot.limelight.loop(); // initialize robot.limelight.blueGoal
            if (robot.limelight.blueGoal != null && robot.limelight.detectBlue) {
                distToTarget = robot.limelight.getDistanceFromTags(robot.limelight.blueGoal);
            } else {
                distToTarget = 150.0;
                telemetry.addLine("Warning: Limelight tag not detected");
            }

            if (gamepad1.a) {        // Intake on/off
                robot.control.startIntake();
            } else {
                robot.control.stopIntake();
            }

            // Toggle shooterActive on gamepad1.x press (not held).
            // The flywheel runs and updates velocity based on distance until toggled off with another press.
            // Pressing gamepad1.b will also turn off the shooter.
            if (gamepad1.x && !prevA) {
                shooterActive = !shooterActive;
            }
            prevA = gamepad1.x;

            double targetRPM = 0.0;
            double targetVelocity = 0.0;
            if (gamepad1.b) {
                flywheel.setVelocity(0);
                shooterActive = false;
                targetRPM = 0.0;
                targetVelocity = 0.0;
            } else if (shooterActive) {
                targetRPM = getTargetRPM(distToTarget);
                targetVelocity = targetRPM * TICKS_PER_REV / 60.0;
                flywheel.setVelocity(targetVelocity);
            }
            // Set flywheel for near shot when dpad_right is pressed
            if (gamepad1.dpad_right && shooterActive) {
                targetRPM = 2800;
                targetVelocity = targetRPM * TICKS_PER_REV / 60.0;
                flywheel.setVelocity(targetVelocity);
                telemetry.addLine("Target velocity for Near - ");
                telemetry.addLine(String.valueOf(targetVelocity));
            }
            // Set flywheel for far shot when dpad_left is pressed
            if (gamepad1.dpad_left && shooterActive) {
                targetRPM = 3700;
                targetVelocity = targetRPM * TICKS_PER_REV / 60.0;
                flywheel.setVelocity(targetVelocity);
                telemetry.addLine("Target velocity for Far - ");
                telemetry.addLine(String.valueOf(targetVelocity));
            }

            double error = Math.abs(flywheel.getVelocity() - targetVelocity);
            boolean shooterReady = error < 50 && targetVelocity > 0;    // Todo: use this
            if (gamepad1.y && shooterActive) {
                telemetry.log().add("Starting the lift");
                lift.setPosition(0.7);
                Thread.sleep(500);
                lift.setPosition(0);
            }

            if (Robot.gamepad1.bumperRight.isPressed()){     // Push servo starts and stops
                telemetry.log().add("Starting the push");
                robot.control.push.setPosition(0);    // to push 0
                Thread.sleep(1000);
                robot.control.push.setPosition(0.5);    // back to origin 0.5
            }

            if (Robot.gamepad1.bumperLeft.isPressed()){     // Piush servo back to origin if stuck
                robot.control.push.setPosition(0.5);    // back to origin 0.5
            }

            if (gamepad1.right_trigger > 0.05){         // To shoot 3 balls
                robot.shootAll2();
                telemetry.addLine("All Balls Shot");
            }

            if (gamepad1.left_trigger > 0.05){         // To shoot the remaining middle ball
                robot.shootRemainingMiddleBall();
            }

            if(gamepad1.dpad_down && (robot.limelight.detectBlue)){
                double degreeError = 0.0;
                degreeError = robot.limelight.blueGoal.getTargetXDegrees();
                double aimSpeed = robot.autoAimSpeed(degreeError , 12);
                robot.control.turretMotor.setMotorEnable();
                robot.control.turretMotor.setPower(aimSpeed);
            }
            else{
                robot.control.turretMotor.setPower(0);
            }

            // Telemetry
            telemetry.addData("Target RPM (auto)", getTargetRPM(distToTarget));
            telemetry.addData("Actual RPM", flywheel.getVelocity() * 60.0 / TICKS_PER_REV);
            telemetry.addData("Battery Voltage", voltage);
            telemetry.addData("Scaled F", BASE_F * (REFERENCE_VOLTAGE / voltage));
            telemetry.addData("Distance to Target", distToTarget);
            telemetry.update();
        }
    }
    private double getTargetRPM(double distanceMm) {
        if (distanceMm < 160) return 2800;
        if (distanceMm < 180) return 2950;
        if (distanceMm < 200) return 3050;
        if (distanceMm < 220) return 3150;
        if (distanceMm < 250) return 3250;
        if (distanceMm < 350) return 3920;
        return 3920;
    }
}
