package org.firstinspires.ftc.teamcode.Libs;

//import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import static com.pedropathing.math.MathFunctions.clamp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.Hardware.MSParams;

public class DriveMecanumFTCLib {

    private final HWProfile2 robot;
    public LinearOpMode opMode;
    public final static MSParams params = new MSParams();


    /*
     * Constructor method
     */
    public DriveMecanumFTCLib(HWProfile2 myRobot, LinearOpMode myOpMode){
        robot = myRobot;
        opMode = myOpMode;

    }   // close DriveMecanum constructor Method





    /* #########################################################################################
       #########################################################################################
       ################################  DRIVE METHODS #########################################
       #########################################################################################
       #########################################################################################
     */

    /******************************************************************************************
     * Method:      ftclibDrive
     * Function:    Robot drives the direction of the heading, at the power provided,
     *              for the distance provided
     * Note:        This function is intended to work at 0, 90, 180, and -90 headings
     * Parameters:
     * @param heading   - Direction robot should drive
     * @param distance  - Distance in Inches to drive
     */
    public void driveDistance(double heading, double distance) {
        double initZ = getZAngle();
        double rflrPower = 0;
        double lfrrPower = 0;
        double currentZ, zCorrection, distanceTraveled = 0;
        boolean active = true;
        double derivative, lastError=0, error;
        double integral, drivePower;
        ElapsedTime rotateTime = new ElapsedTime();
        double maxDrivePower = 0.55;
        double Kp = 0.0025;
        double Ki = 0.001;
        double Kd = 0.0005;
        double minSpeed = robot.MIN_STRAFE_POWER;
        double theta = Math.toRadians(90 + heading);
        double RF = 0, RR = 0, LF = 0, LR = 0;
        double rightPower = 1.4;
        double leftPower = 1;

        robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        opMode.sleep(100);  // allow time for encoder resets
        // make sure distance is positive. Use heading to change direction.
        distance = Math.abs(distance);

            while (opMode.opModeIsActive() && active) {

                error = distance - distanceTraveled;
                derivative = lastError - error;
                integral = rotateTime.time() * error;
                drivePower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
                lastError = error;

                if (drivePower >  -minSpeed && drivePower < 0 ){
                    drivePower = minSpeed;
                } else if (drivePower < minSpeed && drivePower > 0){
                    drivePower = minSpeed;
                }
                rflrPower = drivePower * (Math.sin(theta) - Math.cos(theta));
                lfrrPower = drivePower * (Math.sin(theta) + Math.cos(theta));

                RF = rflrPower;
                LR = rflrPower;
                LF = lfrrPower;
                RR = lfrrPower;

                if (initZ > 170 || initZ < -170) {
                    currentZ = gyro360(0);      // always use 0 as the reference angle
                } else {
                    currentZ = getZAngle();
                }
                zCorrection = Math.abs(initZ - currentZ) * 0.015;

                if (initZ < currentZ) {
                    if (heading == 0 || heading == 180) {
                        /*
                        RF = rflrPower + (zCorrection * rightPower);
                        RR = lfrrPower + (zCorrection * rightPower);
                        LF = lfrrPower - (zCorrection * leftPower);
                        LR = rflrPower - (zCorrection * leftPower);

                         */
                        RF = rflrPower + (zCorrection );
                        RR = lfrrPower + (zCorrection );
                        LF = lfrrPower - (zCorrection );
                        LR = rflrPower - (zCorrection );
                    } else {
                        RF = rflrPower + (zCorrection * rightPower);
                        RR = lfrrPower - (zCorrection * leftPower);
                        LF = lfrrPower - (zCorrection * leftPower);
                        LR = rflrPower + (zCorrection * rightPower);
                    }
                }
                if (initZ > currentZ) {
                    if (heading == 0 || heading == 180) {
                        /*
                        RF = rflrPower - (zCorrection * rightPower);
                        RR = lfrrPower - (zCorrection * rightPower);
                        LF = lfrrPower + (zCorrection * leftPower);
                        LR = rflrPower + (zCorrection * leftPower);

                         */
                        RF = rflrPower - (zCorrection );
                        RR = lfrrPower - (zCorrection );
                        LF = lfrrPower + (zCorrection );
                        LR = rflrPower + (zCorrection );
                    } else {
                        RF = rflrPower - (zCorrection * rightPower);
                        RR = lfrrPower + (zCorrection * leftPower);
                        LF = lfrrPower + (zCorrection * leftPower);
                        LR = rflrPower - (zCorrection * rightPower);
                    }

                }
                /*
                 * Limit that value of the drive motors so that the power does not exceed 100%
                 */
                RF = Range.clip(RF, -Math.abs(maxDrivePower), Math.abs(maxDrivePower));
                LF = Range.clip(LF, -Math.abs(maxDrivePower), Math.abs(maxDrivePower));
                RR = Range.clip(RR, -Math.abs(maxDrivePower), Math.abs(maxDrivePower));
                LR = Range.clip(LR, -Math.abs(maxDrivePower), Math.abs(maxDrivePower));

                /*
                 * Apply power to the drive wheels
                 */
                setDrivePower(RF, LF, LR, RR);

                distanceTraveled = calcDistance(heading);
                if (distanceTraveled >= distance) {
                    active = false;
                    setDrivePower(0,0,0,0);
                }
                opMode.telemetry.addData("Distance Traveled = ", distanceTraveled);
                opMode.telemetry.addData("Distance = ", distance);
                opMode.telemetry.update();
                opMode.idle();

                YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
                AngularVelocity angularVelocity = robot.imu.getRobotAngularVelocity(AngleUnit.DEGREES);


            }   // end of while (opMode.opModeIsActive() && active) loop
            motorsHalt();

    }   // close driveDistance method

    private void setMotorVelocityZero(){
        robot.motorLF.setVelocity(0);
        robot.motorRF.setVelocity(0);
        robot.motorLR.setVelocity(0);
        robot.motorRR.setVelocity(0);
    }

    /**
     * Method: PIDRotate
     * Parameters:
     * @param targetAngle -> desire ending angle/position of the robot
     * @param targetError -> how close should the robot get to the desired angle
     */
    public void pidRotate(double targetAngle, double targetError){
        double integral = 0;
        ElapsedTime rotateTime = new ElapsedTime();
        double error;
        double Kp = 0.375;
        double Ki = 0.0; //0.001;
        double Kd = 0.01; //0.02;
        double minRotateSpeed = robot.MIN_PIDROTATE_POWER;
        double maxRotateSpeed = 0.5;
        double rotationSpeed;
        double derivative, lastError=0;
        double rightRotate;
        double leftRotate;

        // check to see how far the robot is rotating to decide which gyro sensor value to use

        targetAngle = Math.toRadians(targetAngle);  // convert targetAngle to radians
        targetError = Math.toRadians(targetError);
//        error = 100 * (getZAngleRadians() - targetAngle);
        error = (getZAngleRadians() - targetAngle);

        // reset the time to track rotation speed
        rotateTime.reset();

        while ((Math.abs(error) >= Math.abs(targetError)) && opMode.opModeIsActive()) {
            derivative = (error - lastError) / rotateTime.time();
            integral = integral + (rotateTime.time() * error);
            rotationSpeed = ((Kp * error) + (Ki * integral) + (Kd * derivative));
            lastError = error;

            // Clip motor speed
            rotationSpeed = clamp(rotationSpeed, -maxRotateSpeed, maxRotateSpeed);

            if ((rotationSpeed > -0.15) && (rotationSpeed < 0)) {
                rotationSpeed = -minRotateSpeed;
            } else if ((rotationSpeed < 0.15) && (rotationSpeed > 0)) {
                rotationSpeed = minRotateSpeed;
            }

            leftRotate = -rotationSpeed;
            rightRotate = rotationSpeed;


            robot.motorRF.setPower(rightRotate);
            robot.motorLF.setPower(leftRotate);
            robot.motorLR.setPower(leftRotate);
            robot.motorRR.setPower(rightRotate);

            // check to see how far the robot is rotating to decide which gyro sensor value to use
//            error = 100 * (getZAngleRadians() - targetAngle);
            error = (getZAngleRadians() - targetAngle);

        }   // end of while Math.abs(error)
        motorsHalt();

    }   //end of the PIDRotate Method

    /******************************************************************************************
     * Sets power to all four drive motors
     * Method:
     * @param RF    - power for right front motor
     * @param LF    - power for left front motor
     * @param LR    - power for left rear motor
     * @param RR    - power for right rear motor
     ******************************************************************************************/
    public void setDrivePower(double RF, double LF, double LR, double RR){
        robot.motorRF.setPower(RF);
        robot.motorLF.setPower(LF);
        robot.motorLR.setPower(LR);
        robot.motorRR.setPower(RR);
    }   // end of setDrivePower method

    /******************************************************************************************
     * Method:      motorsHalt
     * Function:    Shut off all drive motors
     ******************************************************************************************/
    public void motorsHalt(){
        robot.motorRF.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);
    }   // end of motorsHalt method


    /* #########################################################################################
       #########################################################################################
       ################################  MECHANISM METHODS #####################################
       #########################################################################################
       #########################################################################################
     */

    public void StrafeDrive(double drive, double turn, double strafe) {

        double leftPower    = Range.clip(drive - turn, -robot.MAX_DRIVING_POWER, robot.MAX_DRIVING_POWER);
        double rightPower   = Range.clip(drive + turn, -robot.MAX_DRIVING_POWER, robot.MAX_DRIVING_POWER);
        double strafePower  =  Range.clip( strafe, -robot.MAX_DRIVING_POWER, robot.MAX_DRIVING_POWER);

        robot.motorLF.setPower(leftPower - strafePower);
        robot.motorLR.setPower(leftPower + strafePower);
        robot.motorRF.setPower(rightPower + strafePower);
        robot.motorRR.setPower(rightPower - strafePower);
    }

    public void fieldCentricDrive(double drive, double turn, double strafe) {

        double leftPower    = Range.clip(drive - turn, -robot.MAX_DRIVING_POWER, robot.MAX_DRIVING_POWER);
        double rightPower   = Range.clip(drive + turn, -robot.MAX_DRIVING_POWER, robot.MAX_DRIVING_POWER);
        double strafePower  =  Range.clip( strafe, -robot.MAX_DRIVING_POWER, robot.MAX_DRIVING_POWER);

        robot.motorLF.setPower(leftPower - strafePower);
        robot.motorLR.setPower(leftPower + strafePower);
        robot.motorRF.setPower(rightPower + strafePower);
        robot.motorRR.setPower(rightPower - strafePower);
    }

    /*******************************************************************************************
     * Method:      calcDistance
     * Function:    Calculates the distance that the robot has traveled based on a starting set of
     *              encoder values and the current encoder values
     * Parameters:
     * @param heading   - indicates the direction the robot is angled/heading
     * @return  distanceTraveled
     *******************************************************************************************/
    public double calcDistance(double heading){

        double strafeFactor = 1;        // defaults to 1; changed if the robot is strafing

        if(heading == 90 || heading == -90){
            strafeFactor = robot.STRAFE_FACTOR;
        }

        int totEncoder = Math.abs(robot.motorRF.getCurrentPosition()) + Math.abs(robot.motorLF.getCurrentPosition())
                + Math.abs(robot.motorRR.getCurrentPosition()) + Math.abs(robot.motorLR.getCurrentPosition());

        double avgEncoder = totEncoder/ 4;

        double distanceTraveled = avgEncoder / robot.DRIVE_TICKS_PER_INCH;

        return Math.abs(distanceTraveled * strafeFactor);
    }

    /******************************************************************************************
     * Method:  getZAngle()
     ******************************************************************************************/
    public double getZAngle(){
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = robot.imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        return (orientation.getYaw(AngleUnit.DEGREES));
    }   // close getZAngle method

    /******************************************************************************************
     * Method:  getZAngleRadians()
     ******************************************************************************************/
    public double getZAngleRadians(){
        return 1;
//        return (Math.toRadians(-robot.imu.getAbsoluteHeading()));
    }   // close getZAngle method

    /* #########################################################################################
       #########################################################################################
       ################################  CLASS CALCULATIONS ####################################
       #########################################################################################
       #########################################################################################
     */


    /*******************************************************************************************
     * Method gyro360
     *  - Causes the Gyro to behave in 360 mode instead of 180 degree mode
     *******************************************************************************************/
    public double gyro360(double targetAngle){
        double currentZ = getZAngle();
        double rotationalAngle;

        if (targetAngle > 0){
            if ((currentZ >= 0) && (currentZ <= 180)) {
                rotationalAngle = currentZ;
            } else {
                rotationalAngle = 180 + (180 + currentZ);
            }// end if(currentZ <=0) - else
        } else {
            if ((currentZ <= 0) && (currentZ >= -180)) {
                rotationalAngle = currentZ;
            } else {
                rotationalAngle = -180 - (180 - currentZ);
            }   // end if(currentZ <=0) - else
        }   // end if(targetAngle >0)-else

        return rotationalAngle;
    }   // end method gyro360

    public double angleToServoVal(int degree) {
        double servoVal = degree/360;
        return servoVal;
    }

    /*******************************************************************************************
     * Method gyro360
     *  - Causes the Gyro to behave in 360 mode instead of 180 degree mode
     *******************************************************************************************/
    public double gyro360Radians(double targetAngle){
        double rotationalAngle = 0;

        if(targetAngle > Math.PI){
            rotationalAngle = targetAngle - (2 * Math.PI);
        }
        if(targetAngle < -Math.PI){
            rotationalAngle = targetAngle + (2 * Math.PI);
        }
        return rotationalAngle;
    }   // end method gyro360

}   // close the class
