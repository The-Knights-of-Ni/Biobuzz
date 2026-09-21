package org.firstinspires.ftc.teamcode.Testop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOpMecanum extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl"); //1 port
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("rl");  //0
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");    //3
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rr"); //2
        DcMotor turretMotor = hardwareMap.dcMotor.get("turretMotor"); // ext 1
        DcMotor shootMotor = hardwareMap.dcMotor.get("shootMotor"); // ext 0
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor"); // ext 3
        Servo  lift = hardwareMap.get(Servo.class, "lift"); // ext 0 servo


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

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

            if (gamepad1.a) {
                intakeMotor.setPower(-0.9);
                shootMotor.setPower(-0.75);
                turretMotor.setPower(0);
            }
            if (gamepad1.b) {
                intakeMotor.setPower(0);
                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                shootMotor.setPower(0);
            }
            if (gamepad1.y) {
                lift.setPosition(0.6);
                Thread.sleep(2000);
                lift.setPosition(0);
            }
            if (gamepad1.x) {
                shootMotor.setPower(0);
//                Thread.sleep(2000);
//                intakeMotor2.setPower(0);
            }
        }
    }
}