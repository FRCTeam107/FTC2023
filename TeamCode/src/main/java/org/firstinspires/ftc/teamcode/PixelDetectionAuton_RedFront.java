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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

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


@Autonomous(name="Red Front", group="Robot")

public class PixelDetectionAuton_RedFront extends LinearOpMode {
    /* Declare OpMode members. */
//    private DcMotor leftFrontDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightBackDrive = null;

//    private DcMotor flipperMotor = null;

    //    private Servo flipperServo = null;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
//    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/model_2023.tflite";

    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Blue Object",
            "Red Object"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    private String pixelLocation = null;
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

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



        initTfod();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetryTfod();
        telemetryTfod();
        telemetryTfod();
        telemetryTfod();
        telemetryTfod();
        telemetryTfod();

        if(pixelLocation == "Left"){
            telemetry.addData("Location","Left");
            telemetry.update();
            encoderDrive(DRIVE_SPEED, 22,22,22, 22, 3); //
            encoderDrive(DRIVE_SPEED, -11.65,-11.65,11.65,11.65,3);
            robot.flipperMotor.setPower(-1);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds()< 0.45){}
            robot.flipperMotor.setPower(0);
            robot.flipperServo.setPosition(0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds()< 1.5){}
            robot.flipperServo.setPosition(0.5);
            encoderDrive(DRIVE_SPEED,11.65,11.65,-11.65, -11.65, 3 );
            robot.flipperMotor.setPower(1);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds()< 0.3){}
            robot.flipperMotor.setPower(0);
            encoderDrive(DRIVE_SPEED, 33, 33,33,33, 5); //
            encoderDrive(DRIVE_SPEED, 21,21,-21, -21, 3); //
            encoderDrive(DRIVE_SPEED, 89,89,89, 89, 3); //




        }else if(pixelLocation == "Middle"){
            telemetry.addData("Location","Middle");
            telemetry.update();
            sleep(1000);
            encoderDrive(DRIVE_SPEED, 23,23,23, 23, 3); //
            robot.flipperMotor.setPower(-1);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds()< 0.45){}
//            robot.flipperMotor.setPower(0);
//            robot.flipperServo.setPosition(0);
//            runtime.reset();
//            while (opModeIsActive() && runtime.seconds()< 1.5){}
//            robot.flipperServo.setPosition(0.5);
//            robot.flipperMotor.setPower(1);
            encoderDrive(DRIVE_SPEED, -4,-4,4,4,3);
            robot.flipperMotor.setPower(0);
            robot.flipperServo.setPosition(0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds()< 1.5){}
            encoderDrive(DRIVE_SPEED, 4,4,-4,-4,3);
            robot.flipperServo.setPosition(0.5);
//            robot.towerMotor.setPower(0.5);
//            runtime.reset();
//            while (opModeIsActive() && runtime.seconds()< .5){}
//            robot.towerMotor.setPower(0);
            //backup slightly
            encoderDrive(DRIVE_SPEED,-6,-6,-6,-6,5);
            //strafe right
            encoderDrive(DRIVE_SPEED,-24,24,24,-24,5);
            //drive forward
//            encoderDrive(DRIVE_SPEED, 33,33,33,33,5);
            //turn robot

            encoderDrive(DRIVE_SPEED, 35,35,35, 35, 5); //
            encoderDrive(DRIVE_SPEED, 18, 18,-18,-18, 3); //
//            robot.towerMotor.setPower(-0.5);
//            runtime.reset();
//            while (opModeIsActive() && runtime.seconds()< 0.45|| !robot.touchSensor.isPressed()){}
            robot.flipperMotor.setPower(1);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds()< 0.3){}
            robot.flipperMotor.setPower(0);
            encoderDrive(DRIVE_SPEED, 33,33,33, 33, 5); //
//            encoderDrive(DRIVE_SPEED, 21, 21,-21,-21, 3); //
            encoderDrive(DRIVE_SPEED, 89, 89,89,89, 3); //

        }else {
            //right or no signal

            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            sleep(1000);
            encoderDrive(DRIVE_SPEED, 22,22,22, 22, 3); //
            encoderDrive(DRIVE_SPEED, 11.65,11.65,-11.65,-11.65,3);
            robot.flipperMotor.setPower(-1);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds()< 0.45){}
            robot.flipperMotor.setPower(0);
            robot.flipperServo.setPosition(0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds()< 1.5){}
            robot.flipperServo.setPosition(0.5);
            encoderDrive(DRIVE_SPEED,-11.65,-11.65,11.65, 11.65, 3 );
            robot.flipperMotor.setPower(1);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds()< 0.3){}
            robot.flipperMotor.setPower(0);
            encoderDrive(DRIVE_SPEED, 33,33,33, 33, 5); //
            encoderDrive(DRIVE_SPEED, 21,21,-21, -21, 3); //
            encoderDrive(DRIVE_SPEED, 89,89,89, 89, 3); //

//            while (opModeIsActive()) {
//                telemetry.addData("Currently at", " at LF%7d :RF%7d :LB%7d :RB%7d ",
//                        robot.leftFrontDrive.getCurrentPosition(), robot.rightFrontDrive.getCurrentPosition(),
//                        robot.leftBackDrive.getCurrentPosition(), robot.rightBackDrive.getCurrentPosition());
//                telemetry.update();

//            }
        }
//        telemetry.addData("Path", "Complete");
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
                             double leftFrontInches, double leftBackInches, double rightFrontInches, double rightBackInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
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


    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
//            .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.60f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {
        sleep(1000);
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            if(x < 300 && (recognition.getLabel() == "Red Object" ||recognition.getLabel() == "Blue Object" ) )
            {
                pixelLocation = "Left";
                telemetry.addData("Position","Left");
                telemetry.update();
            }else  if(x > 300 && (recognition.getLabel() == "Red Object" ||recognition.getLabel() == "Blue Object" ) ){
                pixelLocation = "Middle";
                telemetry.addData("Position","Middle");
                telemetry.update();
            }else {
                pixelLocation = "Right";
                telemetry.addData("Position","Right");
                telemetry.update();
            }
        }   // end for() loop


    }   // end method telemetryTfod()

}