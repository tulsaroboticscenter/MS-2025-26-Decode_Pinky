/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.Hardware.MSParams;
import org.firstinspires.ftc.teamcode.Libs.MSMechOps;

import java.util.Locale;
import com.qualcomm.robotcore.hardware.HardwareMap;
/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Shooter Testing", group="Testing")
public class ShooterTest extends LinearOpMode {


    public ElapsedTime buttonPressTimer = new ElapsedTime();
    private double shooterRPM = 0;
    public HardwareMap hwmap;
    public DcMotorEx motorShooter = null;

    public void runOpMode() {
        motorShooter = hwmap.get(DcMotorEx.class, "motorShooter");
        motorShooter.setDirection(DcMotor.Direction.FORWARD);
        motorShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorShooter.setPower(0);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                shooterRPM = 1000;
            }
            if (gamepad1.a) {
                shooterRPM = 0;
            }

            if(gamepad1.dpad_down){
                if((buttonPressTimer.time() > 0.25)) {
                    shooterRPM = shooterRPM - 10;
                    buttonPressTimer.reset();
                }

            }
            if(gamepad1.dpad_up){
                if((buttonPressTimer.time() > 0.25)) {
                    shooterRPM = shooterRPM + 10;
                    buttonPressTimer.reset();
                }

            }

            shooterControl(shooterRPM);

            telemetry.addData("Shooter = ", motorShooter.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Shooter RPM = ", motorShooter.getVelocity());
            telemetry.addData("Angular Rate = ", shooterRPM);
            telemetry.addData("Status", "Running");

            telemetry.addData("Eli Pink Shirt", "yes");
            telemetry.update();
        }
    }

    /**
     * Method shooterControl()
     * @param targetRPM
     */
    public void shooterControl(double targetRPM){
        motorShooter.setVelocity(rpmToTicksPerSecond(targetRPM));
    }   // end of method shooterControl

    /**
     * method rpmToTicksPerSecond
     * @param targetRPM
     */
    private double rpmToTicksPerSecond(double targetRPM){
        return (targetRPM * 28 / 60);
    }   // end of method rpmToTicksPerSecond

}
