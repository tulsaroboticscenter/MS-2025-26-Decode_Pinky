package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.Hardware.MSParams;

public class CTSMechOps {


    public HWProfile2 robot;
    public LinearOpMode opMode;
    public MSParams params = new MSParams();

    /*
     * Constructor
     */
    public CTSMechOps(HWProfile2 myRobot, LinearOpMode myOpMode, MSParams autoParams) {
        robot = myRobot;
        opMode = myOpMode;
        params = autoParams;

    }   // close RRMechOps constructor

    /**
     * Method shooterControl()
     * @param targetRPM
     */
    public void shooterControl(double targetRPM){
        robot.motorShooter.setVelocity(rpmToTicksPerSecond(targetRPM));
        robot.motorShooterTop.setVelocity(rpmToTicksPerSecond(targetRPM));
    }   // end of method shooterControl

    /**
     * method rpmToTicksPerSecond
     * @param targetRPM
     */
    private double rpmToTicksPerSecond(double targetRPM){
        return (targetRPM * 28 / 60);
    }   // end of method rpmToTicksPerSecond

    public void feedShooter(double intakePower, double feederPower){
        robot.motorIntake.setPower(intakePower);
        robot.motorFeeder.setPower(feederPower);

    }
    public void intake(double intakePower, double feederPower){
        robot.motorIntake.setPower(intakePower);
        robot.motorFeeder.setPower(feederPower);

    }

}







