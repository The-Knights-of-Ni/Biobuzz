package org.firstinspires.ftc.teamcode.Teleop;


import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveToPoint;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.firstinspires.ftc.teamcode.Util.DatalogMethod;

import java.util.HashMap;
import java.util.List;
import java.util.Locale;


@TeleOp(name = "hanTeleopFCLOG")
public class hanTeleopFCLOG extends LinearOpMode {
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

    // Logging
    DatalogMethod.Datalog datalog;

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

    public double sigmoid(double x, double k){
        return 1/(1+Math.exp(-k*x));
    }

    public double autoAimSpeed(double dist, double k){
        double magnitude = 0.5, tolerance = 2.0;
        return (-magnitude*sigmoid(dist-tolerance,k) +
                magnitude-magnitude*sigmoid(dist+tolerance,k));
    }

    Pose2D makeTarget(double xpos, double ypos, double hpos){
        return new Pose2D(DistanceUnit.MM,xpos,ypos, AngleUnit.DEGREES,hpos);
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

    public void adjustAngle(double dist) {
        robot.odo.update();
        Pose2D pos = robot.odo.getPosition();
        Pose2D TARGET = makeTarget(pos.getX(DistanceUnit.MM),pos.getY(DistanceUnit.MM),pos.getHeading(AngleUnit.DEGREES)-dist);
        telemetry.addData("Current Heading:", pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Degree difference:",dist);
        String data = String.format(Locale.US, "%f, %f", pos.getHeading(AngleUnit.DEGREES), dist);
        Log.println(Log.DEBUG, "ROBOT", data);
        DriveToTarget(TARGET, 1, 0.5, 1, 1, 5); // original powerMutliplier = 0.7 // original power = 0.8
    }

    private double getTargetRPM(double distance){
        if (distance < 130){
            telemetry.addLine("Too Close!");
            return 2600;
        }
        double targetRPM = 5.7298 * distance + 1844.68582; // with the data from racgael's branch, r^2 greater than 0.95.
        return targetRPM;
    }

    public void runShootMotor(double rpm){
        double tps = -(rpm * 28)/60; // 28 points per rotation
        robot.control.shootMotor.setVelocity(tps);
    }

    void shootAll() throws InterruptedException{
        robot.control.lift.setPosition(0.65); // trigger first ball launch - move up
        Thread.sleep(500);        // wait to get there
        robot.control.lift.setPosition(0);    // move down
        Thread.sleep(500);        // wait to get there

        robot.control.startIntake();          // run intake to move 2 balls up
        Thread.sleep(1000);       // run enough to have enough power to move balls up
        robot.control.stopIntake();           // stop

        robot.control.lift.setPosition(0.65); // trigger second ball launch - move up
        //Thread.sleep(500);        // wait to get there
        robot.waitAim(250);
        robot.control.lift.setPosition(0);    // move down
//        Thread.sleep(500);        // removed, probably redundant.

        Thread.sleep(800);        // wait for flywheel to get back to speed after first ball is shot
        robot.control.push.setPosition(0.0);  // push third ball up - 0.0
//        Thread.sleep(500);        // wait to get there - redundant also, since they both move rather slowly

        robot.control.lift.setPosition(0.65); // trigger third ball launch - move up
        Thread.sleep(500);        // wait to get there
        robot.control.lift.setPosition(0);    // move down

        robot.control.push.setPosition(0.55);  // put back push 0.6
    }


    /**
     * Override of runOpMode()
     *
     * <p>Please do not swallow the InterruptedException, as it is used in cases where the op mode
     * needs to be terminated early.
     *
     * @see LinearOpMode
     */
    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();

        datalog = new DatalogMethod.Datalog("Teleop1");

        // Activates bulk reading, a faster way of reading data
        // IMPORTANT: Caches have to be cleared every loop
        List<LynxModule> allHubs = robot.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        datalog.opModeStatus.set("INIT");
        datalog.battery.set(battery.getVoltage());
        datalog.writeLine();

        ElapsedTime timer = new ElapsedTime();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.log().add("Initialized, ready to start");
        waitForStart();

        datalog.opModeStatus.set("RUNNING");

        battery = hardwareMap.voltageSensor.iterator().next();

        telemetry.clearAll();
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        //==== Motor Initialization ====
        frontLeftMotor = hardwareMap.dcMotor.get("fl"); //1 port
        backLeftMotor = hardwareMap.dcMotor.get("rl");  //0
        frontRightMotor = hardwareMap.dcMotor.get("fr");    //3
        backRightMotor = hardwareMap.dcMotor.get("rr"); //2

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor turretMotor = hardwareMap.dcMotor.get("turretMotor"); // ext 1
        DcMotorEx shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor"); // ext 0
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor"); // ext 3

        Servo push = hardwareMap.servo.get("pushServo");

        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(BASE_P, BASE_I, BASE_D, BASE_F));
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
// Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
// Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // ==== Variable Initialization ====
        /*
         * 0 = color not selected
         * 1 = redf
         * 2 = blue
         * */
        int color = 0;
        double lastVoltage = 0;
        boolean twoGamepads = true;
        boolean reversedDrive = true;
        boolean shootMotorActive = false;
        double intendedShootVelocity = 0.0;
        boolean shootMotorGood = false;
        long lastReversed = 0;
        double distToTarget = 0.0;

        double shooterTriggerMS = 0.0;
        boolean wantToShoot = false;
        boolean liftUp = false;
        boolean pushUp = false;
        boolean intakeOn = false;
        int checkpoint = 0;

        // ==== TELEOP ====
        while (opModeIsActive()) {
            // Clears cache to refresh data
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            // update data from gamepads
            Robot.updateGamepads();

            if (color == 0) {
                if (Robot.gamepad1.bButton.isPressed()) {
                    // red alliance
                    color = 1;
                    robot.allianceColor = AllianceColor.RED;
                } else if (Robot.gamepad1.xButton.isPressed()) {
                    // blue alliance
                    color = 2;
                    robot.allianceColor = AllianceColor.BLUE;
                } else {
                    telemetry.addLine("Alliance color not yet selected.");
                    telemetry.update();
                }
                continue;
            }

            if (color == 2) {
                telemetry.addLine("Color is blue");
            } else if (color == 1) {
                telemetry.addLine("Color is red");
            }

            double actualVelocity = -(shootMotor.getVelocity() * 60) / 28 + 450;
            telemetry.addData("Intended Shootmotor Velocity is ", intendedShootVelocity);
            telemetry.addData("Shootmotor Velocity is (reading +450):", actualVelocity);
            shootMotorGood = actualVelocity >= intendedShootVelocity - 50 && actualVelocity <= intendedShootVelocity + 50;
            if (shootMotorGood) {
                telemetry.addLine("Shoot Motor is GOOD");
            } else {
                telemetry.addLine("Shoot Motor is BAD");
            }

            robot.limelight.loop();


            if (robot.limelight.detectBlue && color == 2) {
                distToTarget = robot.limelight.getDistanceFromTags(robot.limelight.blueGoal);
            } else if (robot.limelight.detectRed && color == 1) {
                distToTarget = robot.limelight.getDistanceFromTags(robot.limelight.redGoal);
            }

            telemetry.addLine("\n");

            telemetry.addData("Distance is ", distToTarget);

            double voltage = battery.getVoltage();

            // update PIDF when voltage meaningfully changes
            if (Math.abs(voltage - lastVoltage) > 0.2) {
                double scaledF = BASE_F * (REFERENCE_VOLTAGE / voltage);
                shootMotor.setPIDFCoefficients(
                        DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(BASE_P, BASE_I, BASE_D, scaledF)
                );
                lastVoltage = voltage;
            }


//            // Improved continuous voltage compensation for shooter
//            double targetVoltage = 12.8; // ideal voltage
//            double kVoltage = 0.02; // change as needed
//            double voltage = robot.getBatteryVoltage();

            // Continuously update shoot motor power if active
            if (shootMotorActive && !gamepad2.a && !gamepad2.b && (robot.limelight.detectBlue || robot.limelight.detectRed)) {
                intendedShootVelocity = getTargetRPM(distToTarget);
                runShootMotor(intendedShootVelocity);
                telemetry.addData("Auto Speed Active, at ", intendedShootVelocity);
            }

            if (twoGamepads) {
                // Get current time and compute delta
                timeCurrent = timer.nanoseconds();
                deltaT = timeCurrent - timePre;
                timePre = timeCurrent;

                // ===== Gamepad 1 - Intake, Shoot =====
                if (gamepad1.a) {        // Intake on/off
                    robot.control.startIntake();
                } else {
                    if (!wantToShoot) {
                        robot.control.stopIntake();

                    }

                    if (gamepad1.y) {       // Lift starts and stops
                        telemetry.log().add("Starting the lift");
                        robot.control.lift.setPosition(0.7);
                        Thread.sleep(500);
                        robot.control.lift.setPosition(0);
                    }

                    if (gamepad1.dpad_down && (robot.limelight.detectBlue || robot.limelight.detectRed)) {
                        double degreeError = 0.0;

                        if (color == 2 && robot.limelight.detectBlue) {
                            degreeError = robot.limelight.blueGoal.getTargetXDegrees();
                        } else if (color == 1 && robot.limelight.detectRed) {
                            degreeError = robot.limelight.redGoal.getTargetXDegrees();
                        }

                        if (Math.abs(degreeError) > 15) {
                            adjustAngle(degreeError);
                        } else {
                            double aimSpeed = autoAimSpeed(degreeError, 12);
                            robot.control.turretMotor.setMotorEnable();
                            robot.control.turretMotor.setPower(aimSpeed);
                        }
                    } else {
                        robot.control.turretMotor.setPower(0);
                    }

                    if (Robot.gamepad1.bumperRight.isPressed()) {     // Push servo starts and stops
                        telemetry.log().add("Starting the push");
                        robot.control.push.setPosition(0);    // to push 0
                        Thread.sleep(1000);
                        robot.control.push.setPosition(0.5);    // back to origin 0.5
                    }

                    if (Robot.gamepad1.bumperLeft.isPressed()) {     // Push servo back to origin if stuck
                        robot.control.push.setPosition(0.5);    // back to origin 0.5
                    }

                    if (gamepad1.right_trigger > 0.05 && System.currentTimeMillis() > shooterTriggerMS + 500) {         // To shoot 3 balls
//                    robot.shootAll2();
                        shooterTriggerMS = System.currentTimeMillis();
                        wantToShoot = true;
                        checkpoint = 0;
                    }

//                if (gamepad1.dpad_left){
//                    robot.control.lift.setPosition(0.3); // to stop the balls from intake.
//                } else {robot.control.lift.setPosition(0);}

                    if (gamepad1.left_trigger > 0.05) {
                        robot.shootRemainingMiddleBall();
                    }

                    // ===== Gamepad 2 - Drive Only =====

                    double x = 0;
                    double y = 0;
                    double rx = 0;
                    double sensitivity = 1; // prev 0.7

                    if (reversedDrive) {
                        telemetry.addLine("Drive is reversed");
                        y = -gamepad2.left_stick_y * sensitivity;
                        x = gamepad2.left_stick_x * 1.1 * sensitivity;   // Counteract imperfect strafing
                        rx = gamepad2.right_stick_x * sensitivity;
                    } else {
                        telemetry.addLine("Drive is not reversed");
                        y = gamepad2.left_stick_y * sensitivity;
                        x = -gamepad2.left_stick_x * 1.1 * sensitivity;   // Counteract imperfect strafing
                        rx = gamepad2.right_stick_x * sensitivity;
                    }

                    // == START FC DRIVE ==

                    // Denominator is the largest motor power (absolute value) or 1
                    // This ensures all the powers maintain the same ratio,
                    // but only if at least one is out of the range [-1, 1]
                    if (gamepad2.start) {
                        imu.resetYaw();
                    }

                    double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                    // Rotate the movement direction counter to the bot's rotation
                    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    rotX = rotX * 1.1;  // Counteract imperfect strafing

                    // Denominator is the largest motor power (absolute value) or 1
                    // This ensures all the powers maintain the same ratio,
                    // but only if at least one is out of the range [-1, 1]
                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                    double frontLeftPower = (rotY + rotX + rx) / denominator;
                    double backLeftPower = (rotY - rotX + rx) / denominator;
                    double frontRightPower = (rotY - rotX - rx) / denominator;
                    double backRightPower = (rotY + rotX - rx) / denominator;

                    frontLeftMotor.setPower(frontLeftPower);
                    backLeftMotor.setPower(backLeftPower);
                    frontRightMotor.setPower(frontRightPower);
                    backRightMotor.setPower(backRightPower);

                    // == END FC DRIVE

                    if (gamepad2.dpad_down && System.currentTimeMillis() - lastReversed > 500) {
                        lastReversed = System.currentTimeMillis();
                        reversedDrive = !reversedDrive;
                    }

                    if (reversedDrive) { // different rotations if reversed (as intuitive). may have to switch signs
                        if (gamepad2.left_trigger > 0.05) {
                            adjustAngle(-45);
                        }
                        if (gamepad2.right_trigger > 0.05) {
                            adjustAngle(45);
                        }
                        if (gamepad2.left_bumper) {
                            adjustAngle(-62);
                        }
                        if (gamepad2.right_bumper) {
                            adjustAngle(62);
                        }
                    } else {
                        if (gamepad2.left_trigger > 0.05) {
                            adjustAngle(135);
                        } // for near shooting
                        if (gamepad2.right_trigger > 0.05) {
                            adjustAngle(-135);
                        }
                        if (gamepad2.left_bumper) {
                            adjustAngle(118);
                        } // for far shooting.
                        if (gamepad2.right_bumper) {
                            adjustAngle(-118);
                        }
                    }

                    if (gamepad2.x) {        // Shootmotor starts with default power
                        robot.control.shootMotor.setPower(-0.6); // why was this positive before :(
                        shootMotorActive = true;
                    }
                    if (gamepad2.y) {
                        robot.control.shootMotor.setPower(0.0);
                        shootMotorActive = false;
                    }

                    if (gamepad2.a) { // move instead of .setPower on gamepad1 if works.
                        shootMotorActive = true;
                        runShootMotor(3700);
                        intendedShootVelocity = 3700;
                    }

                    if (gamepad2.b) {
                        shootMotorActive = true;
                        runShootMotor(2900);
                        intendedShootVelocity = 2900;
                    }

                    telemetry.addData("time since launch: ", System.currentTimeMillis() - shooterTriggerMS);
                    telemetry.addData("want to shoot?", wantToShoot);
                    telemetry.addData("lift up?", liftUp);
                    telemetry.addData("push up?", pushUp);
                    telemetry.addData("intake on?", intakeOn);
                    telemetry.addData("checkpoint", checkpoint);

                    // === shooting control ===
                    if (wantToShoot && System.currentTimeMillis() < shooterTriggerMS + 500 && checkpoint == 0) {
                        robot.control.lift.setPosition(0.65);
                        liftUp = true; // first ball launch
                        telemetry.addLine("first ball");
                        checkpoint = 1;
                    } // the intention of the below wait is for lift to reach the position. so the first ball is actually
                    // launched 500 ms after setPosition(0.65).
                    if (checkpoint == 1 && wantToShoot && System.currentTimeMillis() >= shooterTriggerMS + 500 && liftUp && System.currentTimeMillis() < shooterTriggerMS + 800) {
                        robot.control.lift.setPosition(0);
                        liftUp = false;
                        checkpoint = 2;
                    }

                    if (checkpoint == 2 && wantToShoot && System.currentTimeMillis() >= shooterTriggerMS + 800 && System.currentTimeMillis() < shooterTriggerMS + 1300) {
                        robot.control.startIntake();
                        intakeOn = true;
                        telemetry.addLine("intake on");
                        checkpoint = 3;
                    }

                    if (checkpoint == 3 && wantToShoot && System.currentTimeMillis() >= shooterTriggerMS + 1300 && System.currentTimeMillis() < shooterTriggerMS + 1850) {
                        robot.control.stopIntake();
                        intakeOn = false;
                        robot.control.lift.setPosition(0.65); // second ball launch
                        liftUp = true;
                        telemetry.addLine("intake off");
                        checkpoint = 4;
                    }

                    if (checkpoint == 4 && wantToShoot && System.currentTimeMillis() >= shooterTriggerMS + 1850 && System.currentTimeMillis() < shooterTriggerMS + 2300) {
                        robot.control.lift.setPosition(0);
                        liftUp = false;
                        checkpoint = 5;
                    }

                    if (checkpoint == 5 && wantToShoot && System.currentTimeMillis() >= shooterTriggerMS + 2300 && System.currentTimeMillis() < shooterTriggerMS + 2400) {
                        robot.control.push.setPosition(0);
                        pushUp = true;
                        checkpoint = 6;
                    }

                    if (checkpoint == 6 && wantToShoot && System.currentTimeMillis() >= shooterTriggerMS + 2400 && System.currentTimeMillis() < shooterTriggerMS + 2850) {
                        robot.control.lift.setPosition(0.65);
                        liftUp = true;
                        checkpoint = 7;
                    }

                    if (checkpoint == 7 && wantToShoot && System.currentTimeMillis() >= shooterTriggerMS + 3000) { // change from 5500 to 6000
                        robot.control.lift.setPosition(0);
                        robot.control.push.setPosition(0.55);
                        liftUp = false;
                        pushUp = false;
                        wantToShoot = false;
                        checkpoint = 0;
                    }

                    if (checkpoint == 7 && wantToShoot && System.currentTimeMillis() >= shooterTriggerMS + 6000) {
                        wantToShoot = false;
                        telemetry.addLine("Shootall took too long");
                        checkpoint = 0;
                    }

                    for (int i = 0; opModeIsActive(); i++)
                    {
                        // Note that the order in which we set datalog fields
                        // does *not* matter! The order is configured inside
                        // the Datalog class constructor.

                        datalog.loopCounter.set(i);
                        datalog.battery.set(battery.getVoltage());

                        Orientation orientation = imu.getRobotOrientation(AxesReference.EXTRINSIC,
                                AxesOrder.ZYX, AngleUnit.DEGREES);

                        datalog.yaw.set(orientation.firstAngle);
                        datalog.pitch.set(orientation.secondAngle);
                        datalog.roll.set(orientation.thirdAngle);

                        // The logged timestamp is taken when writeLine() is called.
                        datalog.writeLine();

                        // Datalog fields are stored as text only; do not format here.
                        telemetry.addData("Yaw", datalog.yaw);
                        telemetry.addData("Pitch", datalog.pitch);
                        telemetry.addData("Roll", datalog.roll);
                        telemetry.addLine();
                        telemetry.addData("OpMode Status", datalog.opModeStatus);
                        telemetry.addData("Loop Counter", datalog.loopCounter);
                        telemetry.addData("Battery", datalog.battery);

                        telemetry.update();

                        sleep(20);
                    }
                }


                telemetry.update();
            }
        }
    }
}
