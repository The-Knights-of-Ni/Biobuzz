package org.firstinspires.ftc.teamcode.Subsystems.Drive.Localizer;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.Controller.HolonomicControllerOutput;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MotorGeneric;

public class MecanumLocalizer extends HolonomicLocalizer {
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx rearLeft;
    public DcMotorEx rearRight;

    public MecanumLocalizer(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx rearLeft, DcMotorEx rearRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
    }

    public MotorGeneric<Double> localize(HolonomicControllerOutput output) {
        double xPower = output.x;
        double yPower = output.y;
        double thetaPower = output.heading;
        double yRotated = xPower * Math.cos(output.currentPose.heading) - yPower * Math.sin(output.currentPose.heading); // Inverted bc api
        double xRotated = xPower * Math.sin(output.currentPose.heading) + yPower * Math.cos(output.currentPose.heading);
        return cropMotorPowers(
                new MotorGeneric<>(
                        xRotated + yRotated + thetaPower,
                        xRotated - yRotated + thetaPower,
                        xRotated - yRotated - thetaPower,
                        xRotated + yRotated - thetaPower)
        );
    }

    @Override
    public void setPowers(HolonomicControllerOutput output) {
        MotorGeneric<Double> powers = localize(output);
        frontLeft.setPower(powers.frontLeft);
        frontRight.setPower(powers.frontRight);
        rearLeft.setPower(powers.rearLeft);
        rearRight.setPower(powers.rearRight);
    }
}
