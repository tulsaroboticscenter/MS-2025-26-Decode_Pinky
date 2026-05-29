package org.firstinspires.ftc.teamcode.Libs;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.Hardware.MSParams;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.MSParams;

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

    public void feedShooter(double feederRPM){
        robot.motorFeeder.setVelocity(rpmToTicksPerSecond(feederRPM));

    }
    public void intake(double intakePower){
        robot.motorIntake.setPower(intakePower);
    }




}



