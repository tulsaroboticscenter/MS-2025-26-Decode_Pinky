package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@TeleOp
public class FlywheelTuner extends OpMode {
    public DcMotorEx motorShooter;
    public DcMotorEx motorShooterTop;
    public double highVel = 1900;
    public double lowVel = 1300;
    double curTargetVel = highVel;
    double[] stepsizes = {10,1,0.1,.001,.0001};
    int stepIndex = 1;
    double F=0;
    double P=0;







    public void init(){
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);

        motorShooter = hardwareMap.get(DcMotorEx.class, "motorShooter");
        motorShooter.setDirection(DcMotor.Direction.REVERSE);
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pidfCoefficients));
        //motorShooter.setPower(0);

        motorShooterTop = hardwareMap.get(DcMotorEx.class, "motorShooterTop");
        motorShooterTop.setDirection(DcMotor.Direction.FORWARD);
        motorShooterTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooterTop.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pidfCoefficients));
        //motorShooterTop.setPower(0);

    telemetry.addLine("Init complete");



    }

    public void loop(){

        //get commands
        // set velocity
        // update telemetry

        if (gamepad1.yWasPressed()){
            if(curTargetVel==highVel){
                curTargetVel=lowVel;
            }else{curTargetVel=highVel;}
        }
        if(gamepad1.bWasPressed()){
            stepIndex =(stepIndex+1)%stepsizes.length;
        }
        if(gamepad1.dpadLeftWasPressed()){
            F -= stepsizes[stepIndex];
        }
        if(gamepad1.dpadRightWasPressed()){
            F += stepsizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            P -= stepsizes[stepIndex];
        }
        if(gamepad1.dpadUpWasPressed()){
            P += stepsizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        motorShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pidfCoefficients));
        motorShooterTop.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pidfCoefficients));

        motorShooter.setVelocity(curTargetVel);
        motorShooterTop.setVelocity(curTargetVel);

        double curVel = motorShooter.getVelocity();
        double error = curTargetVel -   curVel;

        telemetry.addData("Target Vel", curTargetVel);
        telemetry.addData("Cur Vel","%.2f",curVel);
        telemetry.addData("Error","%.2f",error);
        telemetry.addLine("------------------------");
        telemetry.addData("Tuning P","%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F","%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size","%.4f (B button)",stepsizes[stepIndex] );

        //increase F until RPM is in "normal Range" 14.99 is close for Brogran
        //Tune P to increase rate of change.  increase P unitl it begins to overshoot. 265 was his value

    }

}
