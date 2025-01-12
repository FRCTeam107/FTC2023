/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 *B: 33.8,33.8 23.3,-23.3 21.6,21.6 23.3,-23.3, 25,25 23.3,-23.3 12.6,12.6
 *
 *
 * F: 42.6,-42.6
 *
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: RobotAutoDriveByTime;
 *
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forward, and causes the encoders to count UP.
 *
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backward for 24 inches
 * - Stop and close the claw.
 *
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This method assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Dry Towelette 1", group="Robot")
@Disabled
public class Casey_Auto_Pos_1 extends LinearOpMode {
    /* Declare OpMode members. */
//    private DcMotor leftFrontDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightBackDrive = null;

//    private DcMotor flipperMotor = null;

//    private Servo flipperServo = null;
    RobotHardware robot = new RobotHardware();

    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 537.7 ; // GoBILDA 5203-2402-0019
    static final double DRIVE_GEAR_REDUCTION = 1.0 ; // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0 ; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED = 0.7;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;



    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Initialize the drive system variables.
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels. Gear Reduction or 90 Deg drives may require direction flips
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d :%7d :%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition(),
                robot.leftBackDrive.getCurrentPosition(),
                robot.rightBackDrive.getCurrentPosition());
        telemetry.update();


        initGyro();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        sleep(1000);
        encoderDrive(DRIVE_SPEED, 24, 24, 3); //
        robot.flipperMotor.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds()< 0.45){}
        robot.flipperMotor.setPower(0);
        robot.flipperServo.setPosition(0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds()< 1.5){}
        robot.flipperServo.setPosition(0.5);
        robot.flipperMotor.setPower(1);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds()< 0.3){}
        robot.flipperMotor.setPower(0);
        //TODO:gyro
//        turnWithGyro(86,.5);
        encoderDrive(DRIVE_SPEED, 21, -21, 3); //
        // encoderDrive(DRIVE_SPEED, 19, 19, 3); //
        runtime.reset();
        robot.towerMotor.setPower(0.5);
        while (opModeIsActive() && runtime.seconds()< 2){}
        encoderDrive(DRIVE_SPEED, 44, 44, 3); //
        robot.towerMotor.setPower(0);
        robot.flipperMotor.setPower(0);
        robot.flipperServo.setPosition(0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds()< 5){}
        runtime.reset();
//        robot.flipperMotor.setPower(0);
        robot.flipperServo.setPosition(0.5);

        while (opModeIsActive()) {
            telemetry.addData("Currently at", " at LF%7d :RF%7d :LB%7d :RB%7d ",
                    robot.leftFrontDrive.getCurrentPosition(), robot.rightFrontDrive.getCurrentPosition(),
                    robot.leftBackDrive.getCurrentPosition(), robot.rightBackDrive.getCurrentPosition());
            telemetry.update();

        }
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000); // pause to display final telemetry message.
    }
    /*
     * Method to perform a relative move, based on encoder counts.
     * Encoders are not reset as the move is based on the current position.
     * Move will stop if any of three conditions occur:
     * 1) Move gets to the desired position
     * 2) Move runs out of time
     * 3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            speed = Math.abs(speed);
            robot.leftFrontDrive.setPower(speed);
            robot.leftBackDrive.setPower(speed);
            robot.rightFrontDrive.setPower(speed);
            robot.rightBackDrive.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop. This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() ||
                            robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d :%7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Currently at", " at %7d :%7d :%7d :%7d ",
                        robot.leftFrontDrive.getCurrentPosition(), robot.rightFrontDrive.getCurrentPosition(),
                        robot.leftBackDrive.getCurrentPosition(), robot.rightBackDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontDrive.setPower(0);
            robot.leftBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250); // optional pause after each move.
        }
    }

    public void turnWithGyro(double degrees, double speedDirection){

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();

        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        double first;
        double second;


        if (speedDirection > 0){//set target positions
            //turn right
            if (degrees > 10){
                first = (degrees - 10) + deconvert(yaw);
                second = degrees + deconvert(yaw);
            }else{
                first = deconvert(yaw);
                second = degrees + deconvert(yaw);
            }

        }else{
            //turn left
            if (degrees > 10){
                first = deconvert(-(degrees - 10) + deconvert(yaw));
                second = deconvert(-degrees + deconvert(yaw));
            }else{
                first = deconvert(yaw);
                second = deconvert(-degrees + deconvert(yaw));
            }
        }
    }

    public double deconvert(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convert(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }

    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
        public void initGyro(){
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            //
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }

    }