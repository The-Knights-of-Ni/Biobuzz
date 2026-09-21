package org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseEstimation;

import static org.firstinspires.ftc.teamcode.Util.MathUtil.normalizeAngle;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Util.Pose;
import org.firstinspires.ftc.teamcode.Util.Vector;

public class Odometry implements PoseEstimationMethod {
    private final DcMotorEx odL;
    private final DcMotorEx odB;
    private final DcMotorEx odR;
    Pose currentPosition = new Pose(0, 0, 0);
    int previousLeftOdometryTicks = 0;
    int previousBackOdometryTicks = 0;
    int previousRightOdometryTicks = 0;

    public final static double ODOMETRY_TRACKWIDTH = 406.4;
    public final static double ODOMETRY_FOWARD_DISPLACEMENT = -50.8; // How far back the back odometry wheel is

    public final static double ODOMETRY_COUNTS_PER_MM = 3; // TODO: Calibrate


    public Odometry(DcMotorEx odL, DcMotorEx odB, DcMotorEx odR) {
        this.odL = odL;
        this.odB = odB;
        this.odR = odR;
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        int odlTicks = odL.getCurrentPosition();
        int odbTicks = odB.getCurrentPosition();
        int odrTicks = odR.getCurrentPosition();

        int deltaOdlTicks = odlTicks - previousLeftOdometryTicks;
        int deltaOdbTicks = odbTicks - previousBackOdometryTicks;
        int deltaOdrTicks = odrTicks - previousRightOdometryTicks;

        double deltaOdlMM = deltaOdlTicks / ODOMETRY_COUNTS_PER_MM;
        double deltaOdbMM = deltaOdbTicks / ODOMETRY_COUNTS_PER_MM;
        double deltaOdrMM = deltaOdrTicks / ODOMETRY_COUNTS_PER_MM;

        double deltaTheta = (deltaOdlMM - deltaOdrMM) / (ODOMETRY_TRACKWIDTH);
        double deltaXC = (deltaOdlMM + deltaOdrMM) / 2;
        double deltaPerpendicular = deltaOdbMM - ODOMETRY_FOWARD_DISPLACEMENT * deltaTheta;

        double deltaX = deltaXC * Math.sin(currentPosition.heading) + deltaPerpendicular * Math.cos(currentPosition.heading);
        double deltaY = deltaXC * Math.cos(currentPosition.heading) - deltaPerpendicular * Math.sin(currentPosition.heading);

        currentPosition.heading += deltaTheta;
        currentPosition.heading = normalizeAngle(currentPosition.heading);
        currentPosition.x += deltaX;
        currentPosition.y += deltaY;
        currentPosition.velocity = new Vector(0, 0);

        previousLeftOdometryTicks = odlTicks;
        previousBackOdometryTicks = odbTicks;
        previousRightOdometryTicks = odrTicks;
    }

    @Override
    public void stop() {
    }

    @Override
    public Pose getPose() {
        return currentPosition;
    }
}
