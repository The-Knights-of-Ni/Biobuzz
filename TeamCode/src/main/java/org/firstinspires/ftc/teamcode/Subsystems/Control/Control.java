package org.firstinspires.ftc.teamcode.Subsystems.Control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.FeedForward;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
//import org.firstinspires.ftc.teamcode.Util.ServoEx;


/**
 * Control subsystem for controlling arms and claws
 */
public class Control extends Subsystem {
//    public final Servo shootFlap;
    public final DcMotorEx shootMotor;
    public final DcMotorEx intakeMotor;
    public final DcMotorEx turretMotor;
    public final Servo lift;

    public final Servo push;

    private final FeedForward feedFowardVelocity = new FeedForward(10, 10);
    private final PID PIDVelocity = new PID(0.002, 0, 0.0002);

    public Control(Telemetry telemetry, DcMotorEx intakeMotor, DcMotorEx shootMotor, DcMotorEx  turretMotor, Servo lift, Servo push) {
        super(telemetry, "control");
        this.shootMotor = shootMotor;
        this.intakeMotor = intakeMotor;
        this.turretMotor = turretMotor;
        this.lift = lift;
        this.push = push;
    }

    /**
     * Gets all defaults, directions,etc. ready for the autonomous period
     */
    public void initDevicesAuto() {
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setDirection(DcMotor.Direction.FORWARD);
        shootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /**
     * Gets all defaults, directions,etc. ready for the teleop period
     */
    public void initDevicesTeleop() {
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void startIntake() {
//        intakeMotor.setMotorEnable();
        intakeMotor.setPower(-1);
    }

    public void stopIntake() {
        intakeMotor.setMotorDisable();
        intakeMotor.setPower(0);

    }

    public void setIntakePower(double power) {
        intakeMotor.setMotorEnable();
        intakeMotor.setPower(-power);
    }

    public void startShoot(double power) {
        shootMotor.setMotorEnable();
        shootMotor.setPower(-power);
    }


    public void stopShoot() {
        shootMotor.setMotorDisable();
        shootMotor.setPower(0);
    }

    public void holdShootVelocity(double targetVelocity) {
        double currentVelocity = shootMotor.getVelocity();
        double feedForward = feedFowardVelocity.calculate(targetVelocity, targetVelocity-currentVelocity+0.1);
        double PIDCorrect = PIDVelocity.calculate(targetVelocity, currentVelocity);

        double variable = feedForward + PIDCorrect;

        shootMotor.setVelocity(variable);
    }

    // turret will drift during the sleeps, move this to robot so auto aiming can be used.
    public void shootAll() throws InterruptedException{
        lift.setPosition(0.65); // trigger first ball launch
        Thread.sleep(500);
        // wait for a set time before stopping, magic number.
        lift.setPosition(0);
        Thread.sleep(500);

        startIntake();
        Thread.sleep(1500);
        stopIntake();

        Thread.sleep(800);
        push.setPosition(0.3);
        Thread.sleep(500);
        push.setPosition(0.0);
        Thread.sleep(500);

        lift.setPosition(0.65); // trigger third ball launch
        Thread.sleep(500); // wait for a set time before stopping, magic number.
        lift.setPosition(0);

        push.setPosition(0.6);  //back to origin
        Thread.sleep(500);
    }

    public double shootMotorVelocity(double distance){
        if(distance < 120){
            return 0.58;
        }
        else if(distance<130){
            return 0.60;
        }
        else if(distance<140){
            return 0.61;
        }
        else if(distance<150){
            return 0.65;
        }
        else if(distance<160){
            return 0.66;
        }
        else if(distance<170){
            return 0.67;
        }
        else if(distance<180){
            return 0.68;
        }
        else if(distance<190){
            return 0.69;
        }
        else if(distance<220){
            return 0.79;
        }
        else{
            return 0.85;
            // 0.85 for long distance shooting area, 0.6 for short
        }
    }

}