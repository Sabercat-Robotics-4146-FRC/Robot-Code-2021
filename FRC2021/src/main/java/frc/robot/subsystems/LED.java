package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED extends Subsystem {
    public static LED mInstance;

    public static LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }

        return mInstance;
    }

    private final AddressableLED mLedStrip;
    private final AddressableLEDBuffer mLedBuffer;

    private LED() {
        mLedStrip = new AddressableLED(Constants.kLedStrip);
        mLedBuffer = new AddressableLEDBuffer(7);
    }

    public synchronized void setLedColor(boolean red, boolean green, boolean blue, boolean yellow, boolean off) {
        mLedStrip.setLength(mLedBuffer.getLength());
        mLedStrip.setData(mLedBuffer);
        mLedStrip.start();
        for (int i = 0; i < mLedBuffer.getLength(); i++){
            if (red) {
                mLedBuffer.setRGB(i, 255, 0, 0); 
                mLedStrip.stop();
            } else {
                mLedStrip.stop();
            }
            if (green) {
                mLedBuffer.setRGB(i, 0, 255, 0);
                mLedStrip.stop();
            } else {
                mLedStrip.stop();
            }
            if (blue) {
                mLedBuffer.setRGB(i, 0, 0, 255);
                mLedStrip.stop();
            } else {
                mLedStrip.stop();
            }
            if (yellow) {
                mLedBuffer.setRGB(i, 255, 200, 0);
                mLedStrip.stop();
            } else {
                mLedStrip.stop();
            }
            if (off) {
                mLedBuffer.setRGB(i, 0, 0, 0);
                mLedStrip.stop();
            } else {
                mLedStrip.stop();
            }
        }
        mLedStrip.setData(mLedBuffer);
    };

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {}
}