package org.firstinspires.ftc.teamcode.Teleop.Deprecated;


import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveToPoint;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

import java.util.HashMap;
import java.util.List;
import java.util.Locale;


//@TeleOp(name = "brownieTeleOp")
public class brownieTeleop extends LinearOpMode {
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
        DriveToTarget(TARGET, 0.8, 0.5, 0.7, 1, 5);
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

        telemetry.clearAll();
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        final double sensitivityHighPower = 1.0; // multiply inputs with this on high power mode
        final double sensitivityLowPower = 0.5; // multiply inputs with this on non-high power mode


        boolean twoGamepads = false;
        boolean intakeOn = false;
        boolean flapOpen = false;
        boolean reversedDrive = false;

        telemetry.addLine("dwbug 0");

        frontLeftMotor = hardwareMap.dcMotor.get("fl"); //1 port
        backLeftMotor = hardwareMap.dcMotor.get("rl");  //0
        frontRightMotor = hardwareMap.dcMotor.get("fr");    //3
        backRightMotor = hardwareMap.dcMotor.get("rr"); //2

        DcMotor turretMotor = hardwareMap.dcMotor.get("turretMotor"); // ext 1
        DcMotor shootMotor = hardwareMap.dcMotor.get("shootMotor"); // ext 0
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor"); // ext 3

        Servo push = hardwareMap.servo.get("pushServo");

        Servo lift = hardwareMap.servo.get("lift");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("dwbug 1");

        /*
         * 0 = color not selected
         * 1 = redf
         * 2 = blue
         * */
        int color = 0;

        boolean shootMotorActive = false;

        while (opModeIsActive()) {
            // Clears cache to refresh data
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            // update data from gamepads
            Robot.updateGamepads();

            telemetry.addLine("While loop exists :(");

            if(color == 0){
                if(Robot.gamepad1.bButton.isPressed()){
                    // red alliance
                    color = 1;
                    robot.allianceColor = AllianceColor.RED;
                }
                else if(Robot.gamepad1.xButton.isPressed()){
                    // blue alliance
                    color = 2;
                    robot.allianceColor = AllianceColor.BLUE;
                }
                else{
                    telemetry.addLine("Alliance color not yet selected.");
                    telemetry.update();
                }
                continue;
            }

            if(color == 2){
                telemetry.addLine("Color is blue");
            }
            else if(color == 1){
                telemetry.addLine("Color is red");
            }
            telemetry.addData("Shootmotor power is ", ShootMotorPower);

            robot.limelight.loop();

            double distToTarget = 0.0;

            if(color == 2 && robot.limelight.detectBlue){
                distToTarget = robot.limelight.getDistanceFromTags( robot.limelight.blueGoal );
            }
            else if(color == 1 && robot.limelight.detectRed){
                distToTarget = robot.limelight.getDistanceFromTags( robot.limelight.redGoal );
            }

            telemetry.addLine("\n");

            telemetry.addData("Hard-coded Shootmotor power is ", robot.control.shootMotorVelocity(distToTarget)+shootMotorConstant);
            telemetry.addData("ShootMotorConstant is ", shootMotorConstant);
            telemetry.addData("Distance is ", distToTarget);

            if(robot.getBatteryVoltage() > 12.5){
                shootMotorConstant = -0.015;
            }
            else if(robot.getBatteryVoltage() > 12) {
                shootMotorConstant = -0.01;
            }
            else if(robot.getBatteryVoltage() > 11.5) {
                shootMotorConstant = 0.02;
            }
            else if(robot.getBatteryVoltage() > 11){
                shootMotorConstant = 0.025;
            }
            else if(robot.getBatteryVoltage() > 10.5){
                shootMotorConstant = 0.03;
            }

//            // Improved continuous voltage compensation for shooter
//            double targetVoltage = 12.8; // ideal voltage
//            double kVoltage = 0.02; // change as needed
//            double voltage = robot.getBatteryVoltage();
//            if (voltage < targetVoltage) {
//                shootMotorConstant = kVoltage * (targetVoltage - voltage);
//                telemetry.addData("ShootmotorConstant: ", shootMotorConstant);
//            }

            //            if(distToTarget != 0 && shootMotorActive){
            // Continuously update shoot motor power if active
            if (shootMotorActive) { // wouldn't this be commented out?
                double sm_power = robot.control.shootMotorVelocity(distToTarget) + shootMotorConstant;
                robot.control.shootMotor.setPower(-sm_power);
                telemetry.addData("Shootmotor power (auto-updating)", -sm_power);
            }



            // un-deprecate later
            if (twoGamepads) {

                // Starting the shooting motor (Trigger Left)
//                if (Robot.gamepad2.triggerLeft > 0.05) {
//                    robot.control.startShoot(ShootMotorPower);
//                    // put limelight tests for teleop here for now?
//                    telemetry.log().add("Starting the shoot motor");
//                    robot.limelight.loop();
//                }
//
//                // Stopping the shooting motor (Bumper Left)
//                if (Robot.gamepad2.bumperLeft.isPressed()) {
//                    robot.control.stopShoot();
//                    telemetry.log().add("Stopping the shoot motor");
//                }
//
//                // Starting and stopping the intake motor
//                if (Robot.gamepad2.yButton.isPressed()) {
//                    if (intakeOn){
//                        robot.control.stopIntake();
//                        telemetry.log().add("Stopping the intake");
//                    }
//                    else {
//                        robot.control.startIntake();
//                        telemetry.log().add("Starting the intake");
//                    }
//                    intakeOn = !intakeOn;
//                }

                if(gamepad1.dpad_down && (robot.limelight.detectBlue || robot.limelight.detectRed)){
                    double degreeError = 0.0;

                    if(color == 2 && robot.limelight.detectBlue){
                        degreeError = robot.limelight.blueGoal.getTargetXDegrees();
                    }
                    else if(color == 1 && robot.limelight.detectRed){
                        degreeError = robot.limelight.redGoal.getTargetXDegrees();
                    }

                    if(Math.abs(degreeError) > 15){
                        adjustAngle(degreeError);
                    }
                    else {
                        double aimSpeed = autoAimSpeed(degreeError , 12);
                        robot.control.turretMotor.setMotorEnable();
                        robot.control.turretMotor.setPower(aimSpeed);
                    }
                }
                else{
                    robot.control.turretMotor.setPower(0);
                }

                // Get current time and compute delta
                timeCurrent = timer.nanoseconds();
                deltaT = timeCurrent - timePre;
                timePre = timeCurrent;

                double sensitivity = 0.6; // less sensitive, 0.5=half speed
                double y = gamepad2.left_stick_y * sensitivity;    // Remember, Y stick value is reversed // i reversed it back to drive backwards because of field orientation
                double x = -gamepad2.left_stick_x * 1.1 * sensitivity;   // Counteract imperfect strafing // now this is reversed
                double rx = -gamepad2.right_stick_x * sensitivity;

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

                //
                if (gamepad1.a) {
                    robot.control.startIntake();
                } else {
                    robot.control.stopIntake();
                }

                if (gamepad2.a){ // for some reason, there was a stop motor but not a start motor. copied from main.
                    robot.control.startShoot(ShootMotorPower);
                    // put limelight tests for teleop here for now?
                    telemetry.log().add("Starting the shoot motor");
                    robot.limelight.loop();
                }

                if (gamepad2.b) {
                    robot.control.shootMotor.setPower(0);
                    shootMotorActive = false;
                }

                if (gamepad1.b) { // assume this is the new one
                    robot.control.push.setPosition(-0.8);
                    telemetry.addLine("Trying and probably failing to move push servo");
                    Thread.sleep(500);
                    robot.control.push.setPosition(0);
                }

                if (gamepad1.y) { //assume this is the old one
                    telemetry.addLine("Y pressed successfully");
                    robot.control.lift.setPosition(0.6); //
                    Thread.sleep(250);
                    robot.control.lift.setPosition(0);
                }

                if (Robot.gamepad1.bumperLeft.isPressed()){
                    double sm_power = 0.85 + shootMotorConstant;
                    robot.control.shootMotor.setPower(-sm_power);
                    telemetry.addData("Shoot motor power before lift is ", -sm_power);
                }
                if (Robot.gamepad1.bumperRight.isPressed()){
                    double sm_power = 0.6 + shootMotorConstant;
                    robot.control.shootMotor.setPower(-sm_power);
                    telemetry.addData("Shoot motor power before lift is ", -sm_power);
                }

                if(gamepad1.dpad_down && (robot.limelight.detectBlue || robot.limelight.detectRed)){
                    double degreeError = 0.0;

                    if(color == 2 && robot.limelight.detectBlue){
                        degreeError = robot.limelight.blueGoal.getTargetXDegrees();
                    }
                    else if(color == 1 && robot.limelight.detectRed){
                        degreeError = robot.limelight.redGoal.getTargetXDegrees();
                    }

                    if(Math.abs(degreeError) > 15){
                        adjustAngle(degreeError);
                    }
                    else {
                        double aimSpeed = autoAimSpeed(degreeError , 12);
                        robot.control.turretMotor.setMotorEnable();
                        robot.control.turretMotor.setPower(aimSpeed);
                    }
                }

                if (gamepad1.x){
                    robot.control.shootAll();
                }
                // does following condition mean you have to keep holding x? idk

//                if (Robot.gamepad1.aButton.isPressed()) {
//                    ShootMotorPower += 0.10;
//                    telemetry.addData("Motor power reduced by 0.1 and is now ",ShootMotorPower);
//                }
//                if (Robot.gamepad1.bButton.isPressed()) {
//                    ShootMotorPower -= 0.10;
//                    telemetry.addData("Motor power increased by 0.1 and is now ",ShootMotorPower);
//                }
//                if (Robot.gamepad1.xButton.isPressed()) {
//                    ShootMotorPower = -0.85;
//                    telemetry.addData("Motor power is reset and is now ",ShootMotorPower);
//                }
//                if (Robot.gamepad1.yButton.isPressed()) {
//                    robot.control.shootMotor.setPower(ShootMotorPower);
//                    telemetry.log().add("Shooting motor with motor power", ShootMotorPower);
//                }
                // We need to add incrementing button later
            } else { // singular gamepad
                // Get current time and compute delta
                timeCurrent = timer.nanoseconds();
                deltaT = timeCurrent - timePre;
                timePre = timeCurrent;

                double y = 0;
                double x = 0;
                double rx = 0; // initialize to zero to escape the if statement?
                double sensitivity = 0.85; // less sensitive, 0.5=half speed

                if (gamepad1.dpad_left){
                    reversedDrive = true;
                } else if (gamepad1.dpad_right){
                    reversedDrive = false;
                }

                if (reversedDrive){
                    telemetry.addLine("Drive is reversed");
                    y = -gamepad1.left_stick_y * sensitivity;
                    x = gamepad1.left_stick_x * 1.1 * sensitivity;   // Counteract imperfect strafing
                    rx = gamepad1.right_stick_x * sensitivity;
                } else{
                    telemetry.addLine("Drive is not reversed");
                    y = gamepad1.left_stick_y * sensitivity;
                    x = -gamepad1.left_stick_x * 1.1 * sensitivity;   // Counteract imperfect strafing
                    rx = -gamepad1.right_stick_x * sensitivity;
                }

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;



                frontLeftMotor.setPower(frontLeftPower); // what if setting motor power is equivalent?
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);

                if (gamepad1.y){ // starts shoot motor, no off since we dont realy need to turn off
                    robot.control.startShoot(ShootMotorPower);
                    // put limelight tests for teleop here for now?
                    telemetry.log().add("Starting the shoot motor");
                    robot.limelight.loop();
                }

                if (gamepad1.left_trigger > 0.05){
                    telemetry.log().add("Starting the lift");
                    robot.control.lift.setPosition(0.7);
                    Thread.sleep(500);
                    robot.control.lift.setPosition(0);
                }

                if (gamepad1.right_trigger > 0.05){
                    telemetry.log().add("Starting the push");
                    robot.control.push.setPosition(0);    // to push 0
                    Thread.sleep(1000);
                    robot.control.push.setPosition(0.5);    // back to origin 0.5
                }

                if (gamepad1.x){ // resets positions
//                    robot.control.push.setPosition(0);
//                    robot.control.lift.setPosition(0);
                }

                if (gamepad1.a){
                    robot.control.startIntake();
                } else{robot.control.stopIntake();}

                if(gamepad1.dpad_down && (robot.limelight.detectBlue || robot.limelight.detectRed)){
                    double degreeError = 0.0;

                    if(color == 2 && robot.limelight.detectBlue){
                        degreeError = robot.limelight.blueGoal.getTargetXDegrees();
                    }
                    else if(color == 1 && robot.limelight.detectRed){
                        degreeError = robot.limelight.redGoal.getTargetXDegrees();
                    }

                    if(Math.abs(degreeError) > 15){
                        adjustAngle(degreeError);
                    }
                    else {
                        double aimSpeed = autoAimSpeed(degreeError , 12);
                        robot.control.turretMotor.setMotorEnable();
                        robot.control.turretMotor.setPower(aimSpeed);
                    }
                }
                else{
                    robot.control.turretMotor.setPower(0);
                }

                if (gamepad1.b){ // shoot all balls, remember to hold aim.
                    robot.control.shootAll();
                    telemetry.addLine("All Balls Shot");
                }

                if (Robot.gamepad1.bumperLeft.isPressed()){
                    double sm_power = 0.85 + shootMotorConstant;
                    robot.control.shootMotor.setPower(-sm_power);
                    telemetry.addData("Shoot motor power before lift is ", -sm_power);
                }
                if (Robot.gamepad1.bumperRight.isPressed()){
                    double sm_power = 0.6 + shootMotorConstant;
                    robot.control.shootMotor.setPower(-sm_power);
                    telemetry.addData("Shoot motor power before lift is ", -sm_power);
                } // added these buttons for easier shooting for 1 gamepad.
            }

            telemetry.update();
        }
    }
}

// launch all not from robot.control
//robot.control.lift.setPosition(0.6); // trigger first ball launch
//
//                    robot.control.startIntake();
//                    Thread.sleep(500); // wait for a set time before stopping, magic number.
//                    robot.control.stopIntake();
//                    Thread.sleep(250);
//
//                    robot.control.push.setPosition(-0.8); // pushes last two balls into shooter
//
//                    robot.control.lift.setPosition(0);
//                    robot.control.push.setPosition(0); // resets positions