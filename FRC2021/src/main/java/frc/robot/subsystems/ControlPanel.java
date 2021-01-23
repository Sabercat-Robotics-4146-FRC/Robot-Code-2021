package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import java.text.BreakIterator;

public class ControlPanel extends Subsystem {
    private static ControlPanel mInstance;

    public static ControlPanel getInstance() {
        if (mInstance == null) {
            mInstance = new ControlPanel();
        }

        return mInstance;
    }

    private final I2C.Port mI2CPort;
    private final CANSparkMax mControlPanelMotor;
    private final int mCounter;
    private final boolean mBlueColorFlag;
    private final String mGameData;
    private final String mColorString;
    private final ColorSensorV3 mColorSensor;
    private final ColorMatch mColorMatcher;
    private final Color mDetectedColor;
    private final ColorMatchResult mMatch;

    private ControlPanel() {
        mI2CPort = I2C.Port.kOnboard;
        mControlPanelMotor = new CANSparkMax(Constants.kControlPanelMotorId, MotorType.kBrushless);
        mCounter = 0;
        mBlueColorFlag = false;
        mGameData = DriverStation.getInstance().getGameSpecificMessage();
        mColorSensor = new ColorSensorV3(mI2CPort);
        mColorMatcher = new ColorMatch();
        mDetectedColor = mColorSensor.getColor();
        mMatch = mColorMatcher.matchClosestColor(mDetectedColor);
        mColorMatcher.addColorMatch(Constants.kBlueTarget);
        mColorMatcher.addColorMatch(Constants.kGreenTarget);
        mColorMatcher.addColorMatch(Constants.kRedTarget);
        mColorMatcher.addColorMatch(Constants.kYellowTarget);

        if (mMatch.color == Constants.kBlueTarget) {
            mColorString = "Blue";
        } else if (mMatch.color == Constants.kRedTarget) {
            mColorString = "Red";
        } else if (mMatch.color == Constants.kGreenTarget) {
            mColorString = "Green";
        } else if (mMatch.color == Constants.kYellowTarget) {
            mColorString = "Yellow";
        } else {
            mColorString = "Unknown";
        }
    }

    public synchronized void setLightSensor() {
        SmartDashboard.putNumber("Red", mDetectedColor.red);
        SmartDashboard.putNumber("Green", mDetectedColor.green);
        SmartDashboard.putNumber("Blue", mDetectedColor.blue);
        SmartDashboard.putNumber("Confidence", mMatch.confidence);
        SmartDashboard.putNumber("Detected Color", mColorString);
        SmartDashboard.putNumber("Counter", mCounter);

        if (mColorString.equals("Blue") && !mBlueColorFlag) {
            mBlueColorFlag = true;
            mCounter += 1;
        }
        if ((mColorString.equals("Blue") != true) && mBlueColorFlag) {
            mBlueColorFlag = false;
        }
        if (mCounter >= 8 && mCounter <= 10) {
            mControlPanelMotor.set(0);
        }
        
        if(mGameData.length() > 0) {
            switch (mGameData.charAt(0)) {
                case 'B' : ColorMatch.makeColor(0.143, 0.427, 0.429);
                break;
                case 'G' : ColorMatch.makeColor(0.197, 0.561, 0.240);
                break;
                case 'R' : ColorMatch.makeColor(0.561, 0.232, 0.114);
                break;
                case 'Y' : ColorMatch.makeColor(0.361, 0.524, 0.113);
                break;
                default : //This is corrupt data
                break;
            }
        } else {
            SmartDashboard.putString("No Color Yet", mColorString);
        }

        if (mGameData.equals('B')) {
            mControlPanelMotor.set(0.1);
            if (mColorString.equals("Blue")) {
                mControlPanelMotor.set(0);
            } else {
                mControlPanelMotor.set(0);
            }
        } else {
            mControlPanelMotor.set(0);
        }
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {}
}