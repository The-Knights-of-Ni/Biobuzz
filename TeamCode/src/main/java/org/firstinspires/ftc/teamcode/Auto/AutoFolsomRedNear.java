package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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

@Autonomous(name = "AutoFolsomRedNear")
public class AutoFolsomRedNear extends LinearOpMode {
    ElapsedTime timer;
    private Robot robot;
    double timeCurrent;
    double timePre;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotorEx flywheel;
    DcMotor turretMotor;
    DcMotor intakeMotor;
    Servo lift;
    VoltageSensor battery;
    private static final double TICKS_PER_REV = 28.0;
    private static final double REFERENCE_VOLTAGE = 13.0;
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

        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            battery = hardwareMap.voltageSensor.iterator().next();
        } else {
            throw new IllegalStateException("No voltage sensor found in hardware map!");
        }
        flywheel = hardwareMap.get(DcMotorEx.class, "shootMotor");  // ext 0
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Waiting for start", "...");

        frontLeftMotor = hardwareMap.dcMotor.get("fl"); //1 port
        backLeftMotor = hardwareMap.dcMotor.get("rl");  //0
        frontRightMotor = hardwareMap.dcMotor.get("fr");    //3
        backRightMotor = hardwareMap.dcMotor.get("rr"); //2
        turretMotor = hardwareMap.dcMotor.get("turretMotor"); // ext 1
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor"); // ext 3
        lift = hardwareMap.get(Servo.class, "lift"); // ext 0 servo
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    void DriveToTarget(Pose2D TARGET, double power, double holdTime, double powerMultiplier, double patience, double timeOut){
        double startTime = getRuntime();
        while(opModeIsActive()){
            robot.odo.update();
            if (robot.nav.driveTo(robot.odo.getPosition(), TARGET, power, holdTime)){
                telemetry.addLine("at position #1!");
                // Sleep to give the reset position time, as it takes 0.25s
                //                        odo.resetPosAndIMU();
                //                        sleep(300);
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


    public void tripleShoot(Servo lift, DcMotorEx flywheel, DcMotor intakeMotor, double targetVelocity, long waitTime) throws InterruptedException {
        flywheel.setVelocity(targetVelocity);
        Thread.sleep(waitTime);
        robot.shootAll2();
    }

    /**
     * Sets the PIDF coefficients for the flywheel motor, compensating for battery voltage.
     * @param motor The DcMotorEx flywheel motor
     * @param baseP Base P value
     * @param baseI Base I value
     * @param baseD Base D value
     * @param baseF Base F value
     * @param referenceVoltage Reference voltage for scaling F
     * @param batteryVoltage Current battery voltage
     */
    private void setFlywheelPIDF(DcMotorEx motor, double baseP, double baseI, double baseD, double baseF, double referenceVoltage, double batteryVoltage) {
        double scaledF = baseF * (referenceVoltage / batteryVoltage);
        motor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(baseP, baseI, baseD, scaledF)
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();

        waitForStart();

        robot.odo.update();

        while(opModeIsActive()){
            // Shooting with velocity control
            // First shot
            // Set flywheel for far shot with fixed RPM, PIDF still compensates for voltage
            double targetRPM = 2800;
            double targetVelocity = targetRPM * TICKS_PER_REV / 60.0;
            double voltage = battery.getVoltage();

            setFlywheelPIDF(flywheel, BASE_P, BASE_I, BASE_D, BASE_F, REFERENCE_VOLTAGE, voltage);
            flywheel.setVelocity(targetVelocity);

            DriveToTarget(makeTarget(100,0,0), 0.5, 0.2, 0.7, 1, 1);
            DriveToTarget(makeTarget(100,0,45), 0.5, 0.2, 0.7, 1, 1);
            DriveToTarget(makeTarget(960,0,45), 0.5, 0.2, 0.7, 1, 1.5);
            DriveToTarget(makeTarget(960,450,45), 0.5, 0.2, 0.7, 1, 1.5);
            DriveToTarget(makeTarget(960,450,135), 0.5, 0.2, 0.7, 1, 1.5);
            DriveToTarget(makeTarget(960,450,225), 0.5, 0.2, 0.7, 1, 1.5);

            long firstShotWaitTime = 0;
            robot.waitAim(500);
            tripleShoot(lift, flywheel, intakeMotor, targetVelocity, firstShotWaitTime);

            DriveToTarget(makeTarget(960,700,-180), 0.5, 0.2, 0.7, 1, 1.5);
            DriveToTarget(makeTarget(750,700,-180), 0.5, 0.2, 0.7, 1, 1.5);

            intakeMotor.setPower(-0.9);
            DriveToTarget(makeTarget(440,700,-180), 0.5, 0.2, 0.7, 1, 1);
            intakeMotor.setPower(0);

            intakeMotor.setPower(-0.9);
            DriveToTarget(makeTarget(220,700,-180), 0.4, 0.2, 0.7, 1, 1);
            intakeMotor.setPower(0);

            DriveToTarget(makeTarget(960,450,-180), 0.5, 0.2, 0.7, 1, 1.5);
            DriveToTarget(makeTarget(960,450,-135), 0.5, 0.2, 0.7, 1, 1.5);

            long secondShotWaitTime = 0;
            robot.waitAim(500);
            tripleShoot(lift, flywheel, intakeMotor, targetVelocity, secondShotWaitTime);

            DriveToTarget(makeTarget(960,0,-135), 0.5, 0.2, 0.7, 1, 1.5);
            DriveToTarget(makeTarget(960,0,-180), 0.5, 0.2, 0.7, 1, 1.5);
            Thread.sleep(30000);
        }
//        while(opModeIsActive()){
//            Thread.sleep(100);
//        }

    }
}
