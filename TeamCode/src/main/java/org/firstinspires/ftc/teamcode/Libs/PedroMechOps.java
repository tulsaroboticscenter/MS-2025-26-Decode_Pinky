package org.firstinspires.ftc.teamcode.Libs;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class PedroMechOps extends OpMode {
    private HWProfile2 robot = new HWProfile2();

    public DcMotorEx motorShooter = null;
    public DcMotorEx motorShooterTop = null;
    public DcMotorEx motorIntake;
    public DcMotorEx motorFeeder;
    //ArtSensor
    public Servo servoFLIPPER;
    public Servo servoLIFT;
    public Servo servoHOOD1;
    public Servo servoHOOD2;
    public Servo servoRPMLight;

    @Override
    public void init() {
        //motorFeeder = robot.hwMap.get(DcMotorEx.class, "motorFeeder");
        motorFeeder.setDirection(DcMotor.Direction.FORWARD);
        motorFeeder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFeeder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        motorFeeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFeeder.setPower(0);

    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        //follower.startTeleopDrive();
    }
    @Override
    public void loop() {

    }
}
