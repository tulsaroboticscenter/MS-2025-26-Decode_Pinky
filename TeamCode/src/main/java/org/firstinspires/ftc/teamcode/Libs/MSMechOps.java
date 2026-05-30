package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.Hardware.MSParams;

import java.util.Timer;

public class MSMechOps {


    public HWProfile2 robot;
    public LinearOpMode opMode;

    public MSParams params = new MSParams();

    /*
     * Constructor
     */
    public MSMechOps(HWProfile2 myRobot, LinearOpMode myOpMode, MSParams autoParams) {
        robot = myRobot;
        opMode = myOpMode;
        params = autoParams;

    }   // close RRMechOps constructor

    /**
     * Method shooterControl()
     *
     * @param targetRPM
     */
    public void shooterControl(double targetRPM) {
        robot.motorShooter.setVelocity((targetRPM));
        robot.motorShooterTop.setVelocity((targetRPM));
    }   // end of method shooterControl

    /**
     * method rpmToTicksPerSecond
     *
     * @param targetRPM
     */
    private double rpmToTicksPerSecond(double targetRPM) {
        return (targetRPM * 28 / 60);
    }   // end of method rpmToTicksPerSecond

    public void feedShooter(double feederVel) {
            robot.motorFeeder.setPower(feederVel);
    }

    public void trigger(double trigVel, double trigPulse) {


    }

    public void intake(double intakePower) {
        robot.motorIntake.setPower(intakePower);
    }

    public void hoodAngle(double hoodAngle){
        robot.servoHOOD1.setPosition(hoodAngle);
        robot.servoHOOD2.setPosition(1-hoodAngle);
    }

}



