package com.example.abdullah.audiotwomics;

import android.widget.EditText;
import android.widget.TextView;

/**
 * Created by Abdullah on 9/4/2017.
 */

public class Settings {
    public static int Fs = 16000;
    public static float stepTime = 20.0f;
    public static float frameTime = 40.0f;
    public static int stepSize = Math.round(Fs*stepTime*0.001f);
    public static int frameSize = Math.round(Fs*frameTime*0.001f);
    public static float stepFactor = 0.5f;
    public static final int queueSize = 20;

    public static float ThreadTime = 10;
    public static float DurationTime = 2;
    public static boolean isEnchanced = false;

    public static boolean playback = true;
    public static int noisetype = 1;
    public static int debugLevel = 0;

    public static TextView ake_txt;
    public static EditText ake_timer_txt;


    public static float counter = 0;
    public static float timer = 0;
    public static short[] outputData;


    private static MainActivity main;
    public static final AudioFrame STOP = new AudioFrame(new short[] {1,2,4,8},true);
    public static void setCallbackInterface(MainActivity uiInterface) {
        main = uiInterface;
    }
    public static MainActivity getCallbackInterface() {
        return main;
    }

    public static boolean setupStepSize(float new_stepsize){
        if (new_stepsize>0){
            stepFactor = new_stepsize;
        }
        return false;
    }

    public static boolean setupThreadTime(float new_ThreadTime){
        if (new_ThreadTime>0){
            ThreadTime = new_ThreadTime;
        }
        return false;
    }

    public static boolean setupDurationTime(float new_ThreadTime){
        if (new_ThreadTime>0){
            DurationTime = new_ThreadTime;
        }
        return false;
    }

    public static boolean setupFrameTime(float new_frametime){
        if (new_frametime>0){
            frameTime = new_frametime;
            stepTime = frameTime/2;
            frameSize = Math.round(Fs*frameTime*0.001f);
            stepSize = Math.round(Fs*stepTime*0.001f);
            return true;
        }
        return false;
    }

    public static boolean setupNoiseType(int new_noisetype){
        if (new_noisetype!=noisetype){
            noisetype = new_noisetype;
            return true;
        }
        return false;
    }

    public static boolean setupPlayback(int new_playback){
        if(debugLevel!=new_playback){
            debugLevel = new_playback;
            return true;
        }
        return false;
    }
    public static boolean setupEnchanced(boolean new_isEnchanced){

        isEnchanced = new_isEnchanced;

        return isEnchanced;
    }
}
