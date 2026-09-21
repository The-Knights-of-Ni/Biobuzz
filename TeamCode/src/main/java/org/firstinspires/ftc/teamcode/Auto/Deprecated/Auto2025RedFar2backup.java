package org.firstinspires.ftc.teamcode.Auto.Deprecated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveToPoint;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

import java.util.HashMap;
import java.util.Locale;

//@Autonomous(name = "Auto2025RedFar2backup")
public class Auto2025RedFar2backup extends LinearOpMode {
    ElapsedTime timer;
    private Robot robot;
    double timeCurrent;
    double timePre;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
//    private DcMotorEx flywheel;
    private VoltageSensor battery;
//    // Base PIDF (tuned at REFERENCE_VOLTAGE)
//    private static final double BASE_P = 20.0;
//    private static final double BASE_I = 0.0;
//    private static final double BASE_D = 2.0;
//    private static final double BASE_F = 11.7;

    private void initOpMode() {
        // Initialize DC motor objects
        timer = new ElapsedTime();
        HashMap<String, Boolean> flags = new HashMap<>();
//        flywheel = hardwareMap.get(DcMotorEx.class, "shootMotor");
        flags.put("web", true);
        flags.put("vision", false);
        DriveToPoint nav = new DriveToPoint(this);
        this.robot = new Robot(hardwareMap, telemetry, timer, nav, AllianceColor.RED, gamepad1, gamepad2, flags);
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        battery = hardwareMap.voltageSensor.iterator().next();
        telemetry.addData("Waiting for start", "...");

//        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        flywheel.setDirection(DcMotor.Direction.REVERSE);
    }

    void DriveToTarget(Pose2D TARGET, double power, double holdTime, double powerMultiplier, double patience, double timeOut){
        double startTime = getRuntime();
        while(opModeIsActive()){
            robot.odo.update();
            if (robot.nav.driveTo(robot.odo.getPosition(), TARGET, power, holdTime)){
                telemetry.addLine("at position #1!");
                powerMultiplier = 0.85;
                break;
            }
            else{
                telemetry.addLine("going to position #1");
                if(getRuntime()-startTime>patience && powerMultiplier != 1){
                    // If the robot has not reached the target after an adjustable amount of time, make it go faster
                    powerMultiplier = 1;
                }
                if(getRuntime()-startTime>timeOut && timeOut>0){
                    // Stop if robot is taking too long if timeout is positive
                    telemetry.addData("Exceeded time limit (seconds) of ", timeOut);
                    break;
                }
            }
            frontLeftMotor.setPower(powerMultiplier*robot.nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            frontRightMotor.setPower(powerMultiplier*robot.nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            backLeftMotor.setPower(powerMultiplier*robot.nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            backRightMotor.setPower(powerMultiplier*robot.nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            telemetry.addData("LF motor power:",robot.nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            telemetry.addData("RF motor power:",robot.nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            telemetry.addData("LB motor power:",robot.nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            telemetry.addData("RB motor power:",robot.nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            Pose2D pos = robot.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

        }
    }

    Pose2D makeTarget(double xpos, double ypos, double hpos){
        return new Pose2D(DistanceUnit.MM,xpos,ypos, AngleUnit.DEGREES,hpos);
    }


    public void tripleShoot(Servo lift, DcMotor shootMotor, DcMotor intakeMotor, double shootPower, long waitTime) throws InterruptedException {
        robot.control.turretMotor.setMotorEnable();
        robot.waitAim(50);
        robot.control.turretMotor.setPower(0);
        shootMotor.setPower(-shootPower);
        Thread.sleep(waitTime);
        robot.waitAim(50);
        robot.shootAll2();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();

        frontLeftMotor = hardwareMap.dcMotor.get("fl"); //1 port
        backLeftMotor = hardwareMap.dcMotor.get("rl");  //0
        frontRightMotor = hardwareMap.dcMotor.get("fr");    //3
        backRightMotor = hardwareMap.dcMotor.get("rr"); //2
        DcMotor turretMotor = hardwareMap.dcMotor.get("turretMotor"); // ext 1
        DcMotor shootMotor = hardwareMap.dcMotor.get("shootMotor"); // ext 0
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor"); // ext 3
        Servo lift = hardwareMap.get(Servo.class, "lift"); // ext 0 servo
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

//        // PIDF for flywheel
//        double lastVoltage = 0;
//        lastVoltage = battery.getVoltage();
//        double voltage = battery.getVoltage();
//        double REFERENCE_VOLTAGE = 13.0;
//
//        // update PIDF when voltage meaningfully changes
//        // PID stands for Proportional, Integral, Derivative. These three terms control how the motor responds
//        // to error (difference between target and actual value).
//        // PIDF adds a Feedforward ("F") term. The F term predicts the needed output based on the target value,
//        // helping the motor reach the target faster and more accurately, especially for velocity control.
//        if (Math.abs(voltage - lastVoltage) > 0.1) {
//            double scaledF = BASE_F * (REFERENCE_VOLTAGE / voltage);
//            flywheel.setPIDFCoefficients(
//                    DcMotor.RunMode.RUN_USING_ENCODER,
//                    new PIDFCoefficients(BASE_P, BASE_I, BASE_D, scaledF)
//            );
//            lastVoltage = voltage;
//        }
        robot.odo.update();

        double shootPower;
        double powerDiff = robot.getBatteryVoltage() - 13.0;

        if (battery.getVoltage() > 13) {
            shootPower = 0.79;
        } else {
            shootPower = 0.81;
        }

        tripleShoot(lift, shootMotor, intakeMotor,shootPower - powerDiff * 0.05,3600);
        Thread.sleep(1000);
//        DriveToTarget(makeTarget(0,-300,0), 0.5, 0.2, 0.7, 1, 1);


//        Thread.sleep(10000);
//
        DriveToTarget(makeTarget(680,0,0), 0.5, 0.2, 0.7, 1, 2);
        DriveToTarget(makeTarget(680,0,-90), 0.5, 0.2, 0.7, 1, 2);

        turretMotor.setPower(-0.35);

        // first intake
        intakeMotor.setPower(-0.9);
        DriveToTarget(makeTarget(680,-250,-90), 0.5, 0.2, 0.7, 1, 1);
        intakeMotor.setPower(0);
        intakeMotor.setPower(-0.9);
        DriveToTarget(makeTarget(680,-470,-90), 0.4, 0.2, 0.7, 1, 1);
        intakeMotor.setPower(0);
//        intakeMotor.setPower(-0.6);
//        DriveToTarget(makeTarget(680,690,90), 0.4, 0.2, 0.7, 1, 1);
//        intakeMotor.setPower(0);

        DriveToTarget(makeTarget(680,-690,45), 0.6, 0.2, 0.7, 1, 1);

        DriveToTarget(makeTarget(1800,350,45), 0.4, 0.2, 0.7, 1, 2);
        DriveToTarget(makeTarget(1800,350,-45), 0.4, 0.2, 0.7, 1, 2);

//        shootMotor.setPower(0.85); // add auto aim later
        turretMotor.setPower(0);
        //  use 0.61 with high voltage
        if (battery.getVoltage() > 13) {
            shootPower = 0.58;
        } else {
            shootPower = 0.61;
        }
//        robot.waitAim(100);
        tripleShoot(lift, shootMotor, intakeMotor,shootPower - powerDiff * 0.05,2500);

        DriveToTarget(makeTarget(1800,350,-90), 0.4, 0.2, 0.7, 1, 2);
        DriveToTarget(makeTarget(1310,0,-90), 0.5, 0.2, 0.7, 1, 2);

        //second intake
        intakeMotor.setPower(-0.9);
        DriveToTarget(makeTarget(1310,-250,-90), 0.5, 0.2, 0.7, 1, 1);
        intakeMotor.setPower(0);
        intakeMotor.setPower(-0.9);
        DriveToTarget(makeTarget(1310,-470,-90), 0.4, 0.2, 0.7, 1, 1);
        intakeMotor.setPower(0);

        DriveToTarget(makeTarget(1800,350,-90), 0.4, 0.2, 0.7, 1, 1);
        DriveToTarget(makeTarget(1800,350,-45), 0.4, 0.2, 0.7, 1, 1);

        DriveToTarget(makeTarget(600,0,-90), 0.8, 0.2, 0.7, 1, 5);

    }
}
