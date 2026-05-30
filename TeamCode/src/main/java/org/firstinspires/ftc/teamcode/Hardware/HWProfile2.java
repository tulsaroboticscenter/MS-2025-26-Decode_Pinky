package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HWProfile2 {


    /*
     * Constants
     */

    public final double PIVOT_SPEED = 0.5;
    public final double COUNTS_PER_ROTATION = 28;
    public final double GB_COUNTS_PER_ROTATION = 28;    // goBilda encoder value
    public final double MIN_PIDROTATE_POWER = 0.10;


    /*
     *  Constants & variables for wheel parameters
     */

    public final double DRIVE_TICKS_PER_INCH = 32;

    public final double STRAFE_FACTOR = 0.9;

    public final double RIGHT_DRIVE_CORRECTION_MULTIPLIER = 1.4;
    public final double LEFT_DRIVE_CORRECTION_MULTIPLIER = 1.2;

    public final double MAX_DRIVING_POWER = 1;

    public double MIN_STRAFE_POWER = 0.35;

    public final double PID_Kp = 0.08;
    public final double PID_Ki = 0.01;
    public final double PID_Kd = 0.000001;
    public final double PID_MIN_SPEED = 0.05;
    public final double PID_ROTATE_ERROR = 1;

    public final double DRIVE_Kp = 0.05;
    public final double DRIVE_Ki = 0.01;
    public final double DRIVE_Kd = 0.31;

    /*
     * Hardware devices
     */

//    public RevIMU imu = null;
    public IMU imu;

    public DcMotorEx motorLF;
    public DcMotorEx motorLR;
    public DcMotorEx motorRF;
    public DcMotorEx motorRR;
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
    //public DistanceSensor ArtSensor;
    //public DistanceSensor AftSensor;
    public DigitalChannel LredLED;
    public DigitalChannel LgreenLED;
    public DigitalChannel RredLED;
    public DigitalChannel RgreenLED;

    public GoBildaPinpointDriver pinpoint; // Declare OpMode member for the Odometry Computer


    HardwareMap hwMap;

    /*
     * Declare Odometry hardware
     */

    /* Constructor */
    public HWProfile2() {
    }

    //private DistanceSensor ArtSensor;






    public void init(HardwareMap ahwMap, boolean teleop) {

        hwMap = ahwMap;
        if(teleop) {

            motorLF = ahwMap.get(DcMotorEx.class, "motorLF");
            motorLF.setDirection(DcMotor.Direction.REVERSE);
            motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorLF.setPower(0);
            motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            motorLR = ahwMap.get(DcMotorEx.class, "motorLR");
            motorLR.setDirection(DcMotor.Direction.REVERSE);
            motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorLR.setPower(0);
            motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorRF = ahwMap.get(DcMotorEx.class, "motorRF");
            motorRF.setDirection(DcMotor.Direction.FORWARD);
            motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorRF.setPower(0);
            motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorRR = ahwMap.get(DcMotorEx.class, "motorRR");
            motorRR.setDirection(DcMotor.Direction.FORWARD);
            motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorRR.setPower(0);
            motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Retrieve and initialize the IMU.
            // This sample expects the IMU to be in a REV Hub and named "imu".

            /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
             *
             * Two input parameters are required to fully specify the Orientation.
             * The first parameter specifies the direction the printed logo on the Hub is pointing.
             * The second parameter specifies the direction the USB connector on the Hub is pointing.
             * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
             *
             * If you are using a REV 9-Axis IMU, you can use the Rev9AxisImuOrientationOnRobot class instead of the
             * RevHubOrientationOnRobot class, which has an I2cPortFacingDirection instead of a UsbFacingDirection.
             */

            /* The next two lines define Hub orientation.
             * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
             *
             * To Do:  EDIT these two lines to match YOUR mounting configuration.
             */
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            // Now initialize the IMU with this mounting orientation
            // Note: if you choose two conflicting directions, this initialization will cause a code exception.
//            imu.initialize(new IMU.Parameters(orientationOnRobot));

        }

        motorShooter = ahwMap.get(DcMotorEx.class, "motorShooter");
        motorShooter.setDirection(DcMotor.Direction.REVERSE);
        motorShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(210, 0.0, 0.0, 14.3));//big shooter (210, 0.0, 0.0, 14.3)
        motorShooter.setPower(0);

        motorShooterTop = ahwMap.get(DcMotorEx.class, "motorShooterTop");
        motorShooterTop.setDirection(DcMotor.Direction.FORWARD);
        motorShooterTop.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorShooterTop.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorShooterTop.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(210, 0.0, 0.0, 14.3));//(800, 0.0, 0.0, 1490))
        motorShooterTop.setPower(0);
        //motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorIntake = ahwMap.get(DcMotorEx.class, "motorIntake");
        motorIntake.setDirection(DcMotor.Direction.FORWARD);
        motorIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorIntake.setPower(0);
        motorIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFeeder = ahwMap.get(DcMotorEx.class, "motorFeeder");
        motorFeeder.setDirection(DcMotor.Direction.FORWARD);
        motorFeeder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFeeder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        motorFeeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFeeder.setPower(0);

        pinpoint = ahwMap.get(GoBildaPinpointDriver.class,"pinpoint");
//      pinpoint.resetPosAndIMU();
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-2,0, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /**
         * Initialize Servos
         **/
        servoFLIPPER = ahwMap.servo.get("servoFLIPPER");
        servoLIFT = ahwMap.servo.get("servoLIFT");
        servoRPMLight = ahwMap.servo.get("servoRPMLight");
        servoHOOD1 =ahwMap.servo.get("servoHOOD1");
        servoHOOD2 =ahwMap.servo.get("servoHOOD2");

        //ArtSensor = ahwMap.get(DistanceSensor.class,"ArtSensor");
        //AftSensor = ahwMap.get(DistanceSensor.class,"AftSensor");

        LredLED = ahwMap.get(DigitalChannel.class, "Lred");
        LgreenLED = ahwMap.get(DigitalChannel.class, "Lgreen");

        RredLED = ahwMap.get(DigitalChannel.class, "Rred");
        RgreenLED = ahwMap.get(DigitalChannel.class, "Rgreen");

        // Zeroing Servos
        //servoIntake.setPower(0.5);
        //servoWrist.setPosition(0);
        //servoExtend.setPosition(0);
        //servoBucket.setPosition(0.5);
        //servoExtendRight.setPosition(0);


        /* Webcam device will go here */
//        webcam = hwMap.get(WebcamName.class, "Webcam 1");
    }


    /**
     * The RunMode of the motor.
     */
    public enum RunMode {
        VelocityControl, PositionControl, RawPower
    }
}
