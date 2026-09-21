package org.firstinspires.ftc.teamcode.Testop.OdometryTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilTagLimelightTest;

import java.util.Locale;

@Autonomous(name="Pinpoint Navigation Example", group="Pinpoint")
//@Disabled



public class SensorPinpointDriveToPoint extends LinearOpMode {

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    DcMotor turret;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    AprilTagLimelightTest A;

    double kk;

    Pose2D makeTarget(double xpos, double ypos, double hpos){
        return new Pose2D(DistanceUnit.MM,xpos,ypos, AngleUnit.DEGREES,hpos);
    }


    void DriveToTarget(Pose2D TARGET, double power, double holdTime, double powerMultiplier, double patience, double timeOut){
        double startTime = getRuntime();
        while(opModeIsActive()){
            odo.update();
            //        telemetry.addData("Reached target", nav.driveTo(odo.getPosition(), TARGET_1, 0.7, 0));
            if (nav.driveTo(odo.getPosition(), TARGET, power, holdTime)){
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
            leftFrontDrive.setPower(powerMultiplier*nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFrontDrive.setPower(powerMultiplier*nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBackDrive.setPower(powerMultiplier*nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBackDrive.setPower(powerMultiplier*nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            telemetry.addData("LF motor power:",nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            telemetry.addData("RF motor power:",nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            telemetry.addData("LB motor power:",nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            telemetry.addData("RB motor power:",nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));


            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.update();
        }
    }


    void InitializeMotors(){
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "rl");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rr");
        turret  = hardwareMap.get(DcMotor.class, "turretMotor"); // ??

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    void InitializeOdometry(){
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
//        odo.setOffsets(-142.0, 120.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setOffsets(-67.0, -168.0); // change later ?

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.recalibrateIMU();

        odo.resetPosAndIMU();
    }


    // use in auto for aiming the turret (shocking ik) shoot separately
    void aimTurret(double timeOut){
        double startTime = getRuntime();
        while(getRuntime()-startTime < timeOut){
            A.loop();

            double Tx = A.getTargetX();
            double Ty = A.getTargetY();
            double distance = A.getDis();
            double heading = A.getBotHeadingDeg();

            double tolerance = 1000.0; // set to be smaller later for now keep it large for testing purposes
            double target = 0.0; // change later

            if(Math.abs(Tx-target)<tolerance/distance){
                turret.setPower(0.0);
                return;
            }

            // uncomment when testing actual aiming (may have to tweak motor power)
//            if(Tx < target){
//                turret.setPower(0.25);
//            }
//            else{
//                turret.setPower(-0.25);
//            }
        }
    }

    @Override
    public void runOpMode() {
        InitializeMotors();

        InitializeOdometry();

        A = new AprilTagLimelightTest(hardwareMap,telemetry);

        A.init();
        A.start();

        kk = A.getBotXmm();

        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());

        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        double power = 0.7, holdTime = 0.5;

        // Stuff:
        // each square of the game floor is 60cm (600mm)

        // Path 1
        DriveToTarget(makeTarget(0,0,45), 0.8, holdTime, 0.7,1,5);
//        DriveToTarget(makeTarget(2100,300,-45), 0.8, holdTime, 0.7,1,5);
//        DriveToTarget(makeTarget(300,-900,-146), 0.8, holdTime, 0.7,1,5);
//        DriveToTarget(makeTarget(900,300,45), 0.8, holdTime, 0.7,1,5);


        // Path 2
        // If this doesn't work split horizontal movement and rotation into two function calls
//        DriveToTarget(makeTarget(900,0,180), 0.8, holdTime, 0.7,1,5);
//        DriveToTarget(makeTarget(900,0,-90), 0.8, holdTime, 0.7,1,5);
//        //        // Robot starts at an angle which i have approximated as 45 degrees for convenience
//        DriveToTarget(makeTarget(900,600,-45), 0.8, holdTime, 0.7,1,5);
//        odo.resetPosAndIMU();
//        sleep(300);
//        DriveToTarget(makeTarget(500,0,0), 0.8, holdTime, 0.7,1,5);
//        DriveToTarget(makeTarget(500,600,90), 0.8, holdTime, 0.7,1,5);
//

        // using two motions instead of one for safety (robot may push the balls to the side instead)
//        DriveToTarget(makeTarget(0,-600,0), 0.8, holdTime, 0.7,1,5);
//        DriveToTarget(makeTarget(-300,-600,180), 0.8, holdTime, 0.7,1,5);
//
//        DriveToTarget(makeTarget(-300,0,180), 0.8, holdTime, 0.7,1,5);
//        DriveToTarget(makeTarget(0,0,-45), 0.8, holdTime, 0.7,1,5);

        // Sleep to give the reset position time, as it takes 0.25s
//        odo.resetPosAndIMU();
//        sleep(300);
//        DriveToTarget(makeTarget(0,0,90), 0.75, holdTime, 0.7,1,-1);
//        DriveToTarget(makeTarget(200,200,0), power, holdTime, 0.7,1,-1);
//        DriveToTarget(makeTarget(-200,0,0), power, holdTime, 0.7,1,-1);
//        DriveToTarget(makeTarget(200,-200,0), power, holdTime, 0.7,1,-1);
//        DriveToTarget(makeTarget(0,0,90), power, holdTime, 0.7,1,-1);
//        DriveToTarget(makeTarget(0,0,-90), power, holdTime, 0.7,1,-1);
//        DriveToTarget(makeTarget(0,0,179), power, holdTime, 0.7,1,-1);
//        DriveToTarget(makeTarget(0,0,0), power, holdTime, 0.7,1,-1);


    }}