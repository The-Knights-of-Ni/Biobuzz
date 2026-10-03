package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveToPoint;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

import java.util.HashMap;
import java.util.List;


@TeleOp(name = "ShooterTest")
public class ShooterTest extends LinearOpMode{
    double deltaT;
    double timeCurrent;
    double timePre;
    double shootMotorConstant = 0.0;
    ElapsedTime timer;
    private Robot robot;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    double ShootMotorPower = 0.55;

    private VoltageSensor battery;
    private ElapsedTime timer2;

    // Motor constants
    // 6000 RPM × 28 / 60 ≈ 2800 ticks/sec
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_RPM_FAR = 3800.0;
    private static final double TARGET_RPM_NEAR = 2900.0;

    // Voltage compensation constants
    private static final double REFERENCE_VOLTAGE = 12.0;

    private static final double BASE_P = 20.0;
    private static final double BASE_I = 0.0;
    private static final double BASE_D = 2.0;
    private static final double BASE_F = 11.7;

    private void initOpMode() {
        // Initialize DC motor objects
        timer = new ElapsedTime();
        HashMap<String, Boolean> flags = new HashMap<>();
        flags.put("web", true);
        flags.put("vision", false);
        DriveToPoint nav = new DriveToPoint(this);
        this.robot = new Robot(hardwareMap, telemetry, timer, nav, AllianceColor.BLUE, gamepad1, gamepad2, flags);
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        telemetry.addData("Waiting for start", "...");
    }

    public void runShootMotor(double rpm){
        double tps = -(rpm * 28)/60; // 28 points per rotation
        robot.control.shootMotor.setVelocity(tps);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();

        // Activates bulk reading, a faster way of reading data
        // IMPORTANT: Caches have to be cleared every loop
        List<LynxModule> allHubs = robot.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        ElapsedTime timer = new ElapsedTime();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.log().add("Initialized, ready to start");
        waitForStart();

        battery = hardwareMap.voltageSensor.iterator().next();

        telemetry.clearAll();
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        //==== Motor Initialization ====
        frontLeftMotor = hardwareMap.dcMotor.get("fl"); //1 port
        backLeftMotor = hardwareMap.dcMotor.get("rl");  //0
        frontRightMotor = hardwareMap.dcMotor.get("fr");    //3
        backRightMotor = hardwareMap.dcMotor.get("rr"); //2

        DcMotorEx shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor"); // ext 0
        DcMotor intakeMotorR = hardwareMap.dcMotor.get("intakeMotorR"); // ext 3
        DcMotor intakeMotorG = hardwareMap.dcMotor.get("intakeMotorG");

        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(BASE_P, BASE_I, BASE_D, BASE_F));
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double shooterRPM = 0;
        double intakePower = 0;
        boolean intakeOn = false;
        boolean shooterOn = false;

        while (opModeIsActive()){
            // Clears cache to refresh data
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            // update data from gamepads
            Robot.updateGamepads();

            if (gamepad1.a){
                intakeOn = !intakeOn;
            }
            if (intakeOn){
                intakeMotorR.setPower(-intakePower);
                intakeMotorG.setPower(intakePower);
                telemetry.addLine("Intake is ON");
            } else {
                intakeMotorR.setPower(0);
                intakeMotorG.setPower(0);
                telemetry.addLine("Intake is OFF");
            }

            if (gamepad1.dpad_right){
                intakePower = intakePower + 0.01;
            }

            if (gamepad1.dpad_left){
                intakePower = intakePower - 0.01;
            }

            if (gamepad1.dpad_up){
                shooterRPM = shooterRPM + 10;
            }

            if (gamepad1.dpad_down){
                shooterRPM = shooterRPM - 10;
            }

            if (gamepad1.b){
                shooterOn = !shooterOn;
            }

            if (shooterOn){
                runShootMotor(shooterRPM);
                telemetry.addLine("Shooter ON");
            } else {
                runShootMotor(0);
                telemetry.addLine("Shooter OFF");
            }

            telemetry.addData("Shooter Power: ", intakePower);
            telemetry.addData("Shooter RPM: ", shooterRPM);

            telemetry.update();
        }


    }
}
