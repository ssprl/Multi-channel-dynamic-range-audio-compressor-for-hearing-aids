//
// Created by Abdullah Kucuk on 9/4/2017.
//


#include <cassert>
#include <cstring>
#include <jni.h>

#include <sys/types.h>
#include <SLES/OpenSLES.h>

#include "audio_common.h"
//#include "audio_recorder.h"
//#include "audio_player.h"
#include "audio_main.h"

//#include "twoDOA.h"
float max(float a, float b)
{
    if (a > b)
        return a;
    else
        return b;
}

int *find(float *a, float *b, int len)
{
    int *outi = (int*)malloc((len+1)*sizeof(int));
    //outi[0] = 0;
    int counter = 0;
    for (int i = 0; i < len; i++)
    {
        if (a[i] > b[i])
        {
            counter ++;
            outi[counter] = i;
        }
    }
    outi[0] = counter;

    return outi;
}


struct EchoAudioEngine {
    SLmilliHertz fastPathSampleRate_;
    uint32_t     fastPathFramesPerBuf_;
    uint16_t     sampleChannels_;
    uint16_t     bitsPerSample_;

    SLObjectItf  slEngineObj_;
    SLEngineItf  slEngineItf_;

   /* AudioRecorder  *recorder_;
    AudioPlayer    *player_;
    AudioQueue     *freeBufQueue_;    //Owner of the queue
    AudioQueue     *recBufQueue_;     //Owner of the queue

    sample_buf  *bufs_;*/
    uint32_t     bufCount_;
    uint32_t     frameCount_;
};
static EchoAudioEngine engine;


bool EngineService(void* ctx, uint32_t msg, void* data );

extern "C" {
//JNIEXPORT void JNICALL
//Java_com_google_sample_echo_MainActivity_createSLEngine(JNIEnv *env, jclass, jint, jint);
JNIEXPORT jlong JNICALL
Java_com_example_abdullah_audiotwomics_SignalProcessing_paramInitilization(JNIEnv *env,
                                                                           jobject thiz,
                                                                           int frequency,
                                                                           int stepSize,
                                                                           int frameSize,
                                                                           float stepFactor,
                                                                           int noisetype,
                                                                           float ThreadTime,
                                                                           float DurationTime,
                                                                           bool isEnchanced);
JNIEXPORT void JNICALL
Java_com_example_abdullah_audiotwomics_SignalProcessing_realtimeProcessing(JNIEnv *env,
                                                                           jobject thiz,
                                                                           jlong memoryPointer,
                                                                           jshortArray input);
JNIEXPORT void JNICALL
Java_com_example_abdullah_audiotwomics_SignalProcessing_paramElimination(JNIEnv *env, jobject thiz,
                                                                         jlong memoryPointer);
JNIEXPORT jshortArray JNICALL
Java_com_example_abdullah_audiotwomics_SignalProcessing_soundOutput(JNIEnv *env, jobject thiz,
                                                                    jlong memoryPointer,
                                                                    jint outputSelection);

JNIEXPORT jfloatArray JNICALL
Java_com_example_abdullah_audiotwomics_SignalProcessing_FFTtiming(JNIEnv *env, jobject thiz,
                                                                    jlong memoryPointer,
                                                                    jint outputSelection);

JNIEXPORT jshortArray JNICALL
Java_com_example_abdullah_audiotwomics_SignalProcessing_dataOutput(JNIEnv *env, jobject thiz,
                                                                   jlong memoryPointer,
                                                                   jint outputSelection);
JNIEXPORT jfloat JNICALL
Java_com_example_abdullah_audiotwomics_SignalProcessing_getTime(JNIEnv *env, jobject thiz,
                                                                jlong memoryPointer);
JNIEXPORT jfloat JNICALL
Java_com_example_abdullah_audiotwomics_SignalProcessing_getComputeTime(JNIEnv *env, jobject thiz,
                                                                       jlong memoryPointer);
JNIEXPORT jfloat JNICALL
Java_com_example_abdullah_audiotwomics_SignalProcessing_getFilteringTime(JNIEnv *env, jobject thiz,
                                                                         jlong memoryPointer);





/*
JNIEXPORT jboolean JNICALL
Java_com_google_sample_echo_MainActivity_createSLBufferQueueAudioPlayer(JNIEnv *env, jclass);
JNIEXPORT void JNICALL
Java_com_google_sample_echo_MainActivity_deleteSLBufferQueueAudioPlayer(JNIEnv *env, jclass type);

JNIEXPORT void JNICALL
Java_com_google_sample_echo_SignalProcessing_realtimeProcessing(JNIEnv *env, jobject thiz,  jlong memoryPointer, jshortArray input);
JNIEXPORT void JNICALL
Java_com_google_sample_echo_MainActivity_deleteAudioRecorder(JNIEnv *env, jclass type);
JNIEXPORT void JNICALL

}*/
JNIEXPORT void JNICALL
Java_com_example_abdullah_audiotwomics_SignalProcessing_realtimeProcessing(JNIEnv *env,
                                                                           jobject thiz,
                                                                           jlong memoryPointer,
                                                                           jshortArray input);

JNIEXPORT void JNICALL
Java_com_example_abdullah_audiotwomics_MainActivity_startPlay(JNIEnv *env, jclass type);
JNIEXPORT void JNICALL
Java_com_example_abdullah_audiotwomics_MainActivity_stopPlay(JNIEnv *env, jclass type);


}

JNIEXPORT jlong JNICALL
Java_com_example_abdullah_audiotwomics_SignalProcessing_paramInitilization(
        JNIEnv *env, jobject thiz,int frequency, int stepSize,int frameSize, float stepFactor, int noisetype, float ThreadTime, float DurationTime, bool isEnchanced){
    Variables *mainParam = (Variables*)malloc(sizeof(Variables));

    int a =10;
    int i;
    mainParam->counter = 0;
    mainParam->u = stepFactor;
    mainParam->stepSize = 320;
    mainParam->samplingRate = frequency;
    mainParam->frameSize = 640;  ////Threadshold Time
    mainParam->filLen = 97;
    mainParam->topMicBuffer = (float*)calloc(stepSize+mainParam->filLen-1,sizeof(float));
    mainParam->botMicBuffer = (float*)calloc(stepSize+mainParam->filLen-1,sizeof(float));
    mainParam->xL_frame = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->xR_frame = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputL1 = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputR1 = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputL2 = (float*)calloc(mainParam->stepSize,sizeof(float));
    mainParam->outputR2 = (float*)calloc(mainParam->stepSize,sizeof(float));

    mainParam->topMicBuffer_prev = (float*)calloc(mainParam->filLen-1,sizeof(float));
    mainParam->botMicBuffer_prev = (float*)calloc(mainParam->filLen-1,sizeof(float));
    mainParam->FB_c = (float**)malloc(num_chan*sizeof(float));
    mainParam->FBbufferL = (float**)malloc(num_chan*sizeof(float));
    mainParam->FBbufferR = (float**)malloc(num_chan*sizeof(float));
    mainParam->FBbufferL_current = (float**)malloc(num_chan*sizeof(float));
    mainParam->FBbufferR_current = (float**)malloc(num_chan*sizeof(float));
    mainParam->FBbufferL_o = (float**)malloc(num_chan*sizeof(float));
    mainParam->FBbufferR_o = (float**)malloc(num_chan*sizeof(float));
    mainParam->FBbufferL_prev = (float**)malloc(num_chan*sizeof(float));
    mainParam->FBbufferR_prev = (float**)malloc(num_chan*sizeof(float));
    mainParam->outputL2FB = (float**)malloc(num_chan*sizeof(float));
    mainParam->outputR2FB = (float**)malloc(num_chan*sizeof(float));

    mainParam->FB_c[0] = (float*)malloc(len_FB[0]*sizeof(float));
    mainParam->FB_c[1] = (float*)malloc(len_FB[1]*sizeof(float));
    mainParam->FB_c[2] = (float*)malloc(len_FB[2]*sizeof(float));
    mainParam->FB_c[3] = (float*)malloc(len_FB[3]*sizeof(float));
    mainParam->FB_c[4] = (float*)malloc(len_FB[4]*sizeof(float));
    mainParam->FB_c[5] = (float*)malloc(len_FB[5]*sizeof(float));
    mainParam->FB_c[6] = (float*)malloc(len_FB[6]*sizeof(float));
    mainParam->FB_c[7] = (float*)malloc(len_FB[7]*sizeof(float));
    mainParam->FB_c[8] = (float*)malloc(len_FB[8]*sizeof(float));

    mainParam->FBbufferL[0] = (float*)calloc(mainParam->stepSize*2+len_FB[0]-1,sizeof(float));
    mainParam->FBbufferL[1] = (float*)calloc(mainParam->stepSize*2+len_FB[1]-1,sizeof(float));
    mainParam->FBbufferL[2] = (float*)calloc(mainParam->stepSize*2+len_FB[2]-1,sizeof(float));
    mainParam->FBbufferL[3] = (float*)calloc(mainParam->stepSize*2+len_FB[3]-1,sizeof(float));
    mainParam->FBbufferL[4] = (float*)calloc(mainParam->stepSize*2+len_FB[4]-1,sizeof(float));
    mainParam->FBbufferL[5] = (float*)calloc(mainParam->stepSize*2+len_FB[5]-1,sizeof(float));
    mainParam->FBbufferL[6] = (float*)calloc(mainParam->stepSize*2+len_FB[6]-1,sizeof(float));
    mainParam->FBbufferL[7] = (float*)calloc(mainParam->stepSize*2+len_FB[7]-1,sizeof(float));
    mainParam->FBbufferL[8] = (float*)calloc(mainParam->stepSize*2+len_FB[8]-1,sizeof(float));

    mainParam->FBbufferR[0] = (float*)calloc(mainParam->stepSize*2+len_FB[0]-1,sizeof(float));
    mainParam->FBbufferR[1] = (float*)calloc(mainParam->stepSize*2+len_FB[1]-1,sizeof(float));
    mainParam->FBbufferR[2] = (float*)calloc(mainParam->stepSize*2+len_FB[2]-1,sizeof(float));
    mainParam->FBbufferR[3] = (float*)calloc(mainParam->stepSize*2+len_FB[3]-1,sizeof(float));
    mainParam->FBbufferR[4] = (float*)calloc(mainParam->stepSize*2+len_FB[4]-1,sizeof(float));
    mainParam->FBbufferR[5] = (float*)calloc(mainParam->stepSize*2+len_FB[5]-1,sizeof(float));
    mainParam->FBbufferR[6] = (float*)calloc(mainParam->stepSize*2+len_FB[6]-1,sizeof(float));
    mainParam->FBbufferR[7] = (float*)calloc(mainParam->stepSize*2+len_FB[7]-1,sizeof(float));
    mainParam->FBbufferR[8] = (float*)calloc(mainParam->stepSize*2+len_FB[8]-1,sizeof(float));

    mainParam->FBbufferL_current[0] = (float*)malloc((mainParam->stepSize+len_FB[0]-1)*sizeof(float));
    mainParam->FBbufferL_current[1] = (float*)malloc((mainParam->stepSize+len_FB[1]-1)*sizeof(float));
    mainParam->FBbufferL_current[2] = (float*)malloc((mainParam->stepSize+len_FB[2]-1)*sizeof(float));
    mainParam->FBbufferL_current[3] = (float*)malloc((mainParam->stepSize+len_FB[3]-1)*sizeof(float));
    mainParam->FBbufferL_current[4] = (float*)malloc((mainParam->stepSize+len_FB[4]-1)*sizeof(float));
    mainParam->FBbufferL_current[5] = (float*)malloc((mainParam->stepSize+len_FB[5]-1)*sizeof(float));
    mainParam->FBbufferL_current[6] = (float*)malloc((mainParam->stepSize+len_FB[6]-1)*sizeof(float));
    mainParam->FBbufferL_current[7] = (float*)malloc((mainParam->stepSize+len_FB[7]-1)*sizeof(float));
    mainParam->FBbufferL_current[8] = (float*)malloc((mainParam->stepSize+len_FB[8]-1)*sizeof(float));

    mainParam->FBbufferR_current[0] = (float*)malloc((mainParam->stepSize+len_FB[0]-1)*sizeof(float));
    mainParam->FBbufferR_current[1] = (float*)malloc((mainParam->stepSize+len_FB[1]-1)*sizeof(float));
    mainParam->FBbufferR_current[2] = (float*)malloc((mainParam->stepSize+len_FB[2]-1)*sizeof(float));
    mainParam->FBbufferR_current[3] = (float*)malloc((mainParam->stepSize+len_FB[3]-1)*sizeof(float));
    mainParam->FBbufferR_current[4] = (float*)malloc((mainParam->stepSize+len_FB[4]-1)*sizeof(float));
    mainParam->FBbufferR_current[5] = (float*)malloc((mainParam->stepSize+len_FB[5]-1)*sizeof(float));
    mainParam->FBbufferR_current[6] = (float*)malloc((mainParam->stepSize+len_FB[6]-1)*sizeof(float));
    mainParam->FBbufferR_current[7] = (float*)malloc((mainParam->stepSize+len_FB[7]-1)*sizeof(float));
    mainParam->FBbufferR_current[8] = (float*)malloc((mainParam->stepSize+len_FB[8]-1)*sizeof(float));

    mainParam->FBbufferL_o[0] = (float*)malloc((mainParam->stepSize)*sizeof(float));
    mainParam->FBbufferL_o[1] = (float*)malloc((mainParam->stepSize)*sizeof(float));
    mainParam->FBbufferL_o[2] = (float*)malloc((mainParam->stepSize)*sizeof(float));
    mainParam->FBbufferL_o[3] = (float*)malloc((mainParam->stepSize)*sizeof(float));
    mainParam->FBbufferL_o[4] = (float*)malloc((mainParam->stepSize)*sizeof(float));
    mainParam->FBbufferL_o[5] = (float*)malloc((mainParam->stepSize)*sizeof(float));
    mainParam->FBbufferL_o[6] = (float*)malloc((mainParam->stepSize)*sizeof(float));
    mainParam->FBbufferL_o[7] = (float*)malloc((mainParam->stepSize)*sizeof(float));
    mainParam->FBbufferL_o[8] = (float*)malloc((mainParam->stepSize)*sizeof(float));

    mainParam->FBbufferR_o[0] = (float*)malloc((mainParam->stepSize)*sizeof(float));
    mainParam->FBbufferR_o[1] = (float*)malloc((mainParam->stepSize)*sizeof(float));
    mainParam->FBbufferR_o[2] = (float*)malloc((mainParam->stepSize)*sizeof(float));
    mainParam->FBbufferR_o[3] = (float*)malloc((mainParam->stepSize)*sizeof(float));
    mainParam->FBbufferR_o[4] = (float*)malloc((mainParam->stepSize)*sizeof(float));
    mainParam->FBbufferR_o[5] = (float*)malloc((mainParam->stepSize)*sizeof(float));
    mainParam->FBbufferR_o[6] = (float*)malloc((mainParam->stepSize)*sizeof(float));
    mainParam->FBbufferR_o[7] = (float*)malloc((mainParam->stepSize)*sizeof(float));
    mainParam->FBbufferR_o[8] = (float*)malloc((mainParam->stepSize)*sizeof(float));

    mainParam->FBbufferL_prev[0] = (float*)calloc((mainParam->stepSize+len_FB[0]-1),sizeof(float));
    mainParam->FBbufferL_prev[1] = (float*)calloc((mainParam->stepSize+len_FB[1]-1),sizeof(float));
    mainParam->FBbufferL_prev[2] = (float*)calloc((mainParam->stepSize+len_FB[2]-1),sizeof(float));
    mainParam->FBbufferL_prev[3] = (float*)calloc((mainParam->stepSize+len_FB[3]-1),sizeof(float));
    mainParam->FBbufferL_prev[4] = (float*)calloc((mainParam->stepSize+len_FB[4]-1),sizeof(float));
    mainParam->FBbufferL_prev[5] = (float*)calloc((mainParam->stepSize+len_FB[5]-1),sizeof(float));
    mainParam->FBbufferL_prev[6] = (float*)calloc((mainParam->stepSize+len_FB[6]-1),sizeof(float));
    mainParam->FBbufferL_prev[7] = (float*)calloc((mainParam->stepSize+len_FB[7]-1),sizeof(float));
    mainParam->FBbufferL_prev[8] = (float*)calloc((mainParam->stepSize+len_FB[8]-1),sizeof(float));

    mainParam->FBbufferR_prev[0] = (float*)calloc((mainParam->stepSize+len_FB[0]-1),sizeof(float));
    mainParam->FBbufferR_prev[1] = (float*)calloc((mainParam->stepSize+len_FB[1]-1),sizeof(float));
    mainParam->FBbufferR_prev[2] = (float*)calloc((mainParam->stepSize+len_FB[2]-1),sizeof(float));
    mainParam->FBbufferR_prev[3] = (float*)calloc((mainParam->stepSize+len_FB[3]-1),sizeof(float));
    mainParam->FBbufferR_prev[4] = (float*)calloc((mainParam->stepSize+len_FB[4]-1),sizeof(float));
    mainParam->FBbufferR_prev[5] = (float*)calloc((mainParam->stepSize+len_FB[5]-1),sizeof(float));
    mainParam->FBbufferR_prev[6] = (float*)calloc((mainParam->stepSize+len_FB[6]-1),sizeof(float));
    mainParam->FBbufferR_prev[7] = (float*)calloc((mainParam->stepSize+len_FB[7]-1),sizeof(float));
    mainParam->FBbufferR_prev[8] = (float*)calloc((mainParam->stepSize+len_FB[8]-1),sizeof(float));

    mainParam->outputL2FB[0] = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputL2FB[1] = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputL2FB[2] = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputL2FB[3] = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputL2FB[4] = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputL2FB[5] = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputL2FB[6] = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputL2FB[7] = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputL2FB[8] = (float*)malloc(mainParam->stepSize*sizeof(float));

    mainParam->outputR2FB[0] = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputR2FB[1] = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputR2FB[2] = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputR2FB[3] = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputR2FB[4] = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputR2FB[5] = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputR2FB[6] = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputR2FB[7] = (float*)malloc(mainParam->stepSize*sizeof(float));
    mainParam->outputR2FB[8] = (float*)malloc(mainParam->stepSize*sizeof(float));

    for (int j = 0; j < len_FB[0]; j++)
        mainParam->FB_c[0][j] = FB0[j];

    for (int j = 0; j < len_FB[1]; j++)
        mainParam->FB_c[1][j] = FB1[j];

    for (int j = 0; j < len_FB[2]; j++)
        mainParam->FB_c[2][j] = FB2[j];

    for (int j = 0; j < len_FB[3]; j++)
        mainParam->FB_c[3][j] = FB3[j];

    for (int j = 0; j < len_FB[4]; j++)
        mainParam->FB_c[4][j] = FB4[j];

    for (int j = 0; j < len_FB[5]; j++)
        mainParam->FB_c[5][j] = FB5[j];

    for (int j = 0; j < len_FB[6]; j++)
        mainParam->FB_c[6][j] = FB6[j];

    for (int j = 0; j < len_FB[7]; j++)
        mainParam->FB_c[7][j] = FB7[j];

    for (int j = 0; j < len_FB[8]; j++)
        mainParam->FB_c[8][j] = FB8[j];

    float temp1 = mainParam->FB_c[6][3];

    mainParam->originalBuffer = (int*)calloc(2*stepSize,sizeof(int));
    mainParam->mixedBuffer = (short*)calloc(2*stepSize,sizeof(short));
    mainParam->w = (float*)malloc(mainParam->filLen*sizeof(float));
    for(i=0;i<mainParam->filLen;i++){
        mainParam->w[i]=1;
    }
    mainParam->y_curr = (float*)calloc(frameSize,sizeof(float));
    mainParam->y_prev = (float*)calloc(frameSize,sizeof(float));
    mainParam->y = (float*)calloc(stepSize,sizeof(float));
    mainParam->e = (float*)calloc(stepSize,sizeof(float));
    mainParam->outputBuffer = (float*)calloc(stepSize,sizeof(float));
    mainParam->uplimit = 5*frequency/stepSize;
    //mainParam->logmmsePtr = initiallogMMSE(frameSize, stepSize, frequency,noisetype);
    //mainParam->timer = newTimer();
    /*mainParam->Threadtime = ThreadTime;
    mainParam->DurationTime = DurationTime;
    mainParam->angle = 0;*/
    mainParam->angle_count = 0;
    mainParam->trans1 = newTransform(DOA_NFFT);
    mainParam->trans2 = newTransform(DOA_NFFT);
    mainParam->isEnchanced=isEnchanced;
    //mainParam->logMMSE=newparameters(DOA_NFFT, DOA_n, 50); //PERC=50
    //mainParam->logMMSE2=newparameters(DOA_NFFT, DOA_n, 50); //PERC=50
    mainParam->ake_counter=0;
    mainParam->ake_avg_timer=0;


    //////////compressor//////
    /*
    float t_att = 200;
    float t_rel = 2000;
    float cr = 2;
    float t_attlim = 6;
    float t_rellim = 80;
    float chan_dBthr = -34.4165;
    float chan0dBgn_lvl = -19.4165;
    float deltaFSdB = 13;
    mainParam->t_ix = mainParam->stepSize; //size of input frame


    float ANSI_ATTdB = 3; // X dB, ATT time defined as settlement within this (in dB)
    float ANSI_RELdB = 4;  // Y dB, REL time defined as settlement within this (in dB)
    float ANSI_STEPdB = 35; // for this step change in input(55 - 90 dB SPL)
    float min_dstpdB = ANSI_STEPdB / 8; // set minimum attack & release time constants to ensure that auto - calculation does not end up with no adjustment at low CRs.Here / 8 effective when CR <= 1.14
    mainParam->expon = (1 - cr) / cr; // exponent for envelope to gain conversion
    mainParam->cthresh = pow(10, (0.05*chan_dBthr));
    float zerodBpt = max(chan_dBthr, chan0dBgn_lvl);  // gain is 0dB for greater of compress.threshold OR reference level.

    mainParam->g0dB = pow(pow(10, (-0.05*chan0dBgn_lvl)), mainParam->expon);  // implicit inversion by '-' for normalisation
    float dstp_att = max(min_dstpdB, ANSI_STEPdB - ANSI_ATTdB*cr / (cr - 1));
    float dstp_rel = max(min_dstpdB, ANSI_STEPdB - ANSI_RELdB*cr / (cr - 1));

    // if dstp == 0 then k_xyz = 1 inline below, so put in maximum value(< 1), related to standard
    mainParam->k_att = pow(10, (.05*(-dstp_att / (t_att*mainParam->samplingRate / 1000)))); // t_att in msec, hence / 1000
    mainParam->k_rel = pow(10, (.05*(-dstp_rel / (t_rel*mainParam->samplingRate / 1000)))); // ditto

    // time constants for limiter: no chance to change attack, 2.5msec, but release can be varied externally.
    float CRLIM = 100; // true limiter...............
    float dstp_limatt = max(min_dstpdB, ANSI_STEPdB - ANSI_ATTdB*CRLIM / (CRLIM - 1)); // full gain change, and....to be within X dB of final
    float dstp_limrel = max(min_dstpdB, ANSI_STEPdB - ANSI_RELdB*CRLIM / (CRLIM - 1)); // full gain change, and....to be within Y dB of final
    mainParam->k_attlim = pow(10, ((-.05*(dstp_limatt + ANSI_ATTdB)) / (t_attlim*mainParam->samplingRate / 1000))); // ANSI X & YdB appears here
    mainParam->k_rellim = pow(10, ((-.05*(dstp_limrel + ANSI_RELdB)) / (t_rellim*mainParam->samplingRate / 1000))); // t_att & t_rel in msec, hence / 1000
    mainParam->deltaFSlin = pow(10, (-.05*deltaFSdB)); // FAST - SLOW threshold difference : INVERT & convert to linear value
    mainParam->ExponLim = (1 - CRLIM) / CRLIM; // exponent for envelope to gain conversion
    float env_ip,k,env_temp,max_temp;
    int ix, pc_act;

    mainParam->gain_envlp = (float*)malloc((1+mainParam->t_ix)*sizeof(float));
    mainParam->limit = (int*)malloc((1 + mainParam->t_ix) * sizeof(int));
    float *output = (float*)malloc((1 + mainParam->t_ix) * sizeof(float));

    mainParam->env_c = (float*)calloc(1+ mainParam->t_ix,sizeof(float));
    mainParam->env_c[0] = mainParam->cthresh;

    mainParam->envlim = (float*)calloc(1 + mainParam->t_ix, sizeof(float));
    mainParam->envlim[0] = mainParam->cthresh;

    mainParam->gain_lim = (float*)malloc(mainParam->limit[0]*sizeof(float));
    //////////////////////////
*/




    __android_log_print(ANDROID_LOG_ERROR, "Parameter Initialized","Successfully");
    return (jlong)mainParam;
}


JNIEXPORT void JNICALL
Java_com_example_abdullah_audiotwomics_SignalProcessing_realtimeProcessing(JNIEnv *env, jobject thiz,  jlong memoryPointer, jshortArray input){
    Variables *mainParam = (Variables*) memoryPointer;
    //startTimer(mainParam->timer);
    short *_in = env->GetShortArrayElements(input, NULL);
    int i,j,stepSize,frameSize,filLen;
    float tempVar,tempEng, mThreadTime, mDurationTime;
    stepSize = mainParam->stepSize;
    frameSize = mainParam->frameSize;
    filLen = mainParam->filLen;
    float temp1,temp2;
    float env_ip,env_temp,max_temp,k_c;
    //mThreadTime = mainParam->Threadtime;
    //mDurationTime = mainParam->DurationTime;


    __android_log_print(ANDROID_LOG_ERROR, "processing begin","Successfully");

    for (int k = 0; k < stepSize; k++)
    {

        mainParam->xL_frame[k] = _in[k*2];
        mainParam->xR_frame[k] = _in[k*2+1];

    }
    env->ReleaseShortArrayElements(input, _in, 0);
     /*
    ///It is needed for Transform function
    for (int k = DOA_n; k < DOA_NFFT; k++)
    {

        x_frame[k] = 0;
        h_frame[k] = 0;

    }
    */
    clock_t t;
    t = clock();

    //mainParam->botMicBuffer = conv(mainParam->xL_frame,gains,stepSize,filLen);
    //mainParam->topMicBuffer = conv(mainParam->xR_frame,gains,stepSize,filLen);


    //__android_log_print(ANDROID_LOG_ERROR, "Gain function Done","Successfully");

   // mainParam->twoDOA->doIT(mainParam->twoDOA, x_frame, h_frame, win, mainParam->angle_count, mainParam->twoDOA->prevmag_fft_Framex, mainParam->twoDOA->prevmag_fft_Frameh, mainParam->twoDOA->prevcorrtheta_est, mainParam->twoDOA->SFxmax, mainParam->twoDOA->SFxavg, mainParam->twoDOA->flagSFx, DOA_fs, DOA_n, DOA_NFFT,mThreadTime,mDurationTime,mainParam->logMMSE,mainParam->isEnchanced);
    //mainParam->trans1->doTransform(mainParam->trans1,x_frame);
    //mainParam->trans2->doTransform(mainParam->trans2,h_frame);




/*

    for(i=0;i<stepSize;i++){
        if (i<filLen-1)
        {
            mainParam->outputL1[i] =  mainParam->botMicBuffer[i]+mainParam->botMicBuffer_prev[i];
            mainParam->outputR1[i] =  mainParam->topMicBuffer[i]+mainParam->topMicBuffer_prev[i];
            mainParam->botMicBuffer_prev[i] = mainParam->botMicBuffer[i+stepSize];
            mainParam->topMicBuffer_prev[i] = mainParam->topMicBuffer[i+stepSize];
        }

        else
        {
            mainParam->outputL1[i] =  mainParam->botMicBuffer[i];
            mainParam->outputR1[i] =  mainParam->topMicBuffer[i];
        }
    }
*/

    //__android_log_print(ANDROID_LOG_ERROR, "Overlap for Gain function Done","Successfully");
/////filter banks
    //mainParam->outputL2 = conv(mainParam->outputL1,filterBank1,stepSize,len_bank1);
    //mainParam->outputR2 = conv(mainParam->outputR1,filterBank1,stepSize,len_bank1);
    /*
    for (i=0;i<num_chan;i++)
    {
        mainParam->FBbufferL_current[i] = conv(mainParam->outputL1,mainParam->FB_c[i],stepSize,len_FB[i]);
        mainParam->FBbufferR_current[i] = conv(mainParam->outputR1,mainParam->FB_c[i],stepSize,len_FB[i]);

        for (j=0;j<2*stepSize+len_FB[i]-1;j++)
        {
            if((i==6))
            {
                int temp = i;
            }
            if(j<stepSize)
            {
                mainParam->FBbufferL[i][j] = mainParam->FBbufferL_prev[i][j];
                mainParam->FBbufferR[i][j] = mainParam->FBbufferR_prev[i][j];

            }
            else if(j<stepSize+len_FB[i]-1)
            {
                mainParam->FBbufferL[i][j] = mainParam->FBbufferL_prev[i][j] + mainParam->FBbufferL_current[i][j-stepSize];
                mainParam->FBbufferR[i][j] = mainParam->FBbufferR_prev[i][j] + mainParam->FBbufferR_current[i][j-stepSize];
            }
            else
            {
                mainParam->FBbufferL[i][j] = mainParam->FBbufferL_current[i][j-stepSize];
                mainParam->FBbufferR[i][j] = mainParam->FBbufferR_current[i][j-stepSize];
            }

            /////output
            int delay = delay_FB[i];
            if((j>= stepSize - delay)&&(j<2*stepSize-delay))
            {
                int tempj = j;
                int tempi = i;
                if (i==6&&j==280)
                {
                    tempi = i;
                }
                mainParam->FBbufferL_o[i][j-stepSize+delay] = mainParam->FBbufferL[i][j];
                mainParam->FBbufferR_o[i][j-stepSize+delay] = mainParam->FBbufferR[i][j];
            }

            ////store prev
            if(j>=stepSize)
            {
                mainParam->FBbufferL_prev[i][j-stepSize] = mainParam->FBbufferL[i][j];
                mainParam->FBbufferR_prev[i][j-stepSize] = mainParam->FBbufferR[i][j];
            }
        }

        ////re-construction
        for (int k=0;k<stepSize;k++)
        {
            if(i==0)
            {
                mainParam->outputL2[k] = mainParam->FBbufferL_o[i][k];
                mainParam->outputR2[k] = mainParam->FBbufferR_o[i][k];
            }
            else
            {
                mainParam->outputL2[k] += mainParam->FBbufferL_o[i][k];
                mainParam->outputR2[k] += mainParam->FBbufferR_o[i][k];
            }
        }

    }
     */
/*
    //////////////compressor//////////////

    for (int ix = 0; ix < mainParam->t_ix; ix++)
    {

        if(ix==0)
        {
            k_c = mainParam->k_att;
        }


        env_ip = fabs(mainParam->outputL1[ix]); // full wave rectify

        if (env_ip > mainParam->env_c[ix]) // attacking
            k_c = mainParam->k_att;
        else // releasing
            k_c = mainParam->k_rel;

        mainParam->env_c[ix + 1] = max(mainParam->cthresh, (1 - k_c)*env_ip + k_c*mainParam->env_c[ix]);
        max_temp = (1 - k_c)*env_ip + k_c*mainParam->env_c[ix];

        //env_temp = mainParam->env_c[ix + 1];

        env_ip = env_ip * mainParam->deltaFSlin; // want LIMITER mean to track relative to SLOW levels

        if (env_ip > mainParam->envlim[ix]) // attacking
            k_c = mainParam->k_attlim;
        else      // releasing
            k_c = mainParam->k_rellim;
        mainParam->envlim[ix + 1] = max(mainParam->cthresh, (1 - k_c)*env_ip + k_c*mainParam->envlim[ix]); // NB env_ip already reduced for this part of loop
        //env_temp = mainParam->envlim[ix + 1];

        mainParam->gain_envlp[ix] = (pow(mainParam->env_c[ix], mainParam->expon))*mainParam->g0dB;

        if(ix==160)
        {
            k_c = mainParam->k_att;
        }


        //fprintf(f, "%f ", envlim[ix]);
    }

    mainParam->limit = find(mainParam->envlim , mainParam->env_c,mainParam->t_ix);  // only calculate exponent & division when limiter in action
    for (int i = 0; i < mainParam->limit[0]; i++)
    {
        mainParam->gain_lim[i] = mainParam->envlim[mainParam->limit[i + 1]] / pow(mainParam->env_c[mainParam->limit[i + 1]], mainParam->ExponLim);// normalise gain change relative to slower envelope measure
        mainParam->gain_envlp[mainParam->limit[i + 1]] = mainParam->gain_envlp[mainParam->limit[i + 1]] * mainParam->gain_lim[i]; //pull in limiter gain
    }

    for (int i = 0; i < mainParam->t_ix; i++)
    {
        mainParam->outputL2[i] = mainParam->outputL1[i] * mainParam->gain_envlp[i + 1];
    }

*/
    //////////////////////////////////////




    ////////////
    /////output
    for(i=0;i<stepSize;i++){
        /*if (i<len_bank1-1)
        {
            mainParam->mixedBuffer[i*2] = mainParam->outputL2[i]+mainParam->outputL2_prev[i];
            mainParam->mixedBuffer[i*2+1] = mainParam->outputR2[i]+mainParam->outputR2_prev[i];
            mainParam->outputL2_prev[i] = mainParam->outputL2[i+stepSize];
            mainParam->outputR2_prev[i] = mainParam->outputR2[i+stepSize];
        }*/

        //else
        //{
            //mainParam->mixedBuffer[i*2] = mainParam->outputL1[i];
            //mainParam->mixedBuffer[i*2+1] = mainParam->outputR1[i];
        mainParam->mixedBuffer[i*2] = mainParam->xL_frame[i];
        mainParam->mixedBuffer[i*2+1] = mainParam->xR_frame[i];

        //}
    }

    //__android_log_print(ANDROID_LOG_ERROR, "output Done","Successfully");
    t = clock() - t;

    float ake_timing=(((float)t)/ CLOCKS_PER_SEC);

    mainParam->ake_counter= mainParam->ake_counter+1;
    mainParam->ake_avg_timer=ake_timing;

    /*float ake_timing=(((float)t)/ CLOCKS_PER_SEC)+(mainParam->ake_avg_timer*(mainParam->ake_counter));

    mainParam->ake_counter= mainParam->ake_counter+1;
    mainParam->ake_avg_timer=ake_timing/(mainParam->ake_counter);*/

    //__android_log_print(ANDROID_LOG_INFO,"Time", "Time1= %1.9g ms",(((float)mainParam->ake_avg_timer)*1000));
    //__android_log_print(ANDROID_LOG_INFO,"Time", "Time1= %1.9g",(((double)t)/ CLOCKS_PER_SEC));
    mainParam->angle_count++;

    /*for(i=0;i<2*stepSize;i++){
        mainParam->mixedBuffer[i] = _in[i];
    }*/
    //__android_log_print(ANDROID_LOG_ERROR, "Parameter Computed 1st","Successfully");
    //__android_log_print(ANDROID_LOG_ERROR, "timing Done","Successfully");

}


JNIEXPORT void JNICALL
Java_com_example_abdullah_audiotwomics_SignalProcessing_paramElimination(JNIEnv* env, jobject thiz, jlong memoryPointer){
    Variables *mainParam = (Variables*) memoryPointer;
    if(mainParam != NULL){
        free(mainParam->topMicBuffer);mainParam->topMicBuffer = NULL;
        free(mainParam->botMicBuffer);mainParam->botMicBuffer = NULL;
        free(mainParam->xL_frame);mainParam->xL_frame = NULL;
        free(mainParam->xR_frame);mainParam->xR_frame = NULL;
        free(mainParam->outputL1);mainParam->outputL1 = NULL;
        free(mainParam->outputR1);mainParam->outputR1 = NULL;
        free(mainParam->outputL2);mainParam->outputL2 = NULL;
        free(mainParam->outputR2);mainParam->outputR2 = NULL;

        free(mainParam->env_c);mainParam->env_c = NULL;
        free(mainParam->envlim);mainParam->envlim = NULL;
        free(mainParam->gain_envlp);mainParam->gain_envlp = NULL;
        free(mainParam->gain_lim);mainParam->gain_lim = NULL;
        free(mainParam->limit);mainParam->limit = NULL;


        free(mainParam->topMicBuffer_prev);mainParam->topMicBuffer_prev = NULL;
        free(mainParam->botMicBuffer_prev);mainParam->botMicBuffer_prev = NULL;
        free(mainParam->FB_c);mainParam->FB_c = NULL;
        free(mainParam->FBbufferL);mainParam->FBbufferL = NULL;
        free(mainParam->FBbufferL_prev);mainParam->FBbufferL_prev = NULL;
        free(mainParam->FBbufferL_current);mainParam->FBbufferL_current = NULL;
        free(mainParam->FBbufferR);mainParam->FBbufferR = NULL;
        free(mainParam->FBbufferR_current);mainParam->FBbufferR_current = NULL;
        free(mainParam->FBbufferR_prev);mainParam->FBbufferR_prev = NULL;
        free(mainParam->outputL2FB);mainParam->outputL2FB = NULL;
        free(mainParam->outputR2FB);mainParam->outputR2FB = NULL;
        free(mainParam->FBbufferL_o);mainParam->FBbufferL_o = NULL;
        free(mainParam->FBbufferR_o);mainParam->FBbufferR_o = NULL;


        free(mainParam->outputBuffer);mainParam->outputBuffer = NULL;
        free(mainParam->originalBuffer);mainParam->originalBuffer = NULL;
        free(mainParam->mixedBuffer);mainParam->mixedBuffer = NULL;
        free(mainParam->y_prev);mainParam->y_prev = NULL;
        free(mainParam->y_curr);mainParam->y_curr = NULL;
        free(mainParam->y);mainParam->y = NULL;
        free(mainParam->e);mainParam->e = NULL;
        free(mainParam->w);mainParam->w = NULL;
        //destroylogMMSE(&(mainParam->logmmsePtr));
       // destroyTimer(&(mainParam->timer));
        free(mainParam);mainParam = NULL;
    }
}

JNIEXPORT jshortArray JNICALL
Java_com_example_abdullah_audiotwomics_SignalProcessing_soundOutput(JNIEnv* env, jobject thiz, jlong memoryPointer, jint outputSelection){

    __android_log_print(ANDROID_LOG_ERROR, "Sound Output Begin","Successfully");

    Variables* mainParam = (Variables*) memoryPointer;
    jshortArray output = env->NewShortArray(2*mainParam->stepSize);
    short *_output = env->GetShortArrayElements( output, NULL);
    short mixedbuffer1;
    short mixedbuffer2;
    short mixedbuffer3;
    short mixedbuffer4;
    short mixedbuffer0;
    int i;
    switch (outputSelection){
        case 0:		// Original

            /*for (i=0;i<2*mainParam->stepSize;i++){
                _output[i] = (short)checkRange(mainParam->mixedBuffer[i]);
            }*/
            //mixedbuffer1 = mainParam->mixedBuffer[0];
            //mixedbuffer2 = mainParam->mixedBuffer[1];
            //mixedbuffer3 = mainParam->mixedBuffer[2];
            //mixedbuffer4 = mainParam->mixedBuffer[3];
            //mixedbuffer0 = mainParam->mixedBuffer[mainParam->stepSize-1];

            //mainParam->angle = mainParam->angle_count;
            //mainParam->angle_count++;

            for (i=0;i<2*mainParam->stepSize;i++){
                _output[i] = mainParam->mixedBuffer[i];
            }
            break;
        case 1:		// Enhanced
            for (i=0;i<2*mainParam->stepSize;i++){
                _output[i] = (short)checkRange(mainParam->outputBuffer[i]);
            }
            /*
            for (i=0,j=0;i<2*mainParam->stepSize;i+=2,j++){
                _output[i] = (short)checkRange(mainParam->e[j]);
                _output[i+1] = (short)checkRange(mainParam->e[j]);
            }*/
            break;
        case 2:
            break;
    }
    env->ReleaseShortArrayElements(output, _output, 0);

    __android_log_print(ANDROID_LOG_ERROR, "Sound Output Done","Successfully");
    return output;
}


JNIEXPORT jfloatArray JNICALL
Java_com_example_abdullah_audiotwomics_SignalProcessing_FFTtiming(JNIEnv* env, jobject thiz, jlong memoryPointer, jint outputSelection){
    Variables* mainParam = (Variables*) memoryPointer;
    jfloatArray output = env->NewFloatArray(2);
    //float tmp = 0;
    float *_output = env->GetFloatArrayElements( output, NULL);



    //int i;
    switch (outputSelection){
        case 0:		// Original



           // tmp = max1(2,3);

           // mainParam->angle = tmp;
           // mainParam->angle_count++;
            _output[0] = mainParam->ake_counter;
            _output[1] = mainParam->ake_avg_timer;

            break;
        case 1:		// Enhanced
            _output[0] = -1;
            /*
            for (i=0,j=0;i<2*mainParam->stepSize;i+=2,j++){
                _output[i] = (short)checkRange(mainParam->e[j]);
                _output[i+1] = (short)checkRange(mainParam->e[j]);
            }*/
            break;
        case 2:
            break;
    }
    env->ReleaseFloatArrayElements(output, _output, 0);
    return output;
}

JNIEXPORT jshortArray JNICALL
Java_com_example_abdullah_audiotwomics_SignalProcessing_dataOutput(JNIEnv* env, jobject thiz, jlong memoryPointer, jint outputSelection){
    Variables* mainParam = (Variables*) memoryPointer;
    jshortArray output = env->NewShortArray(2*mainParam->stepSize);
    short *_output = env->GetShortArrayElements(output, NULL);
    int i;
    switch (outputSelection){
        case 0:
            for(i= 0;i<2*mainParam->stepSize;i++){
                _output[i] = mainParam->originalBuffer[i];
            }
            break;
        case 1:
            for(i= 0;i<2*mainParam->stepSize;i++){
                _output[i] = (short)checkRange(mainParam->outputBuffer[i]);
            }
            break;
        case 2:
            break;
    }
    env->ReleaseShortArrayElements(output, _output, 0);
    return output;
}





/*JNIEXPORT jfloat JNICALL
Java_com_example_abdullah_audiotwomics_SignalProcessing_getTime(JNIEnv* env, jobject thiz, jlong memoryPointer)
{
    Variables* mainParam = (Variables*) memoryPointer;
    return getTimerMS(mainParam->timer);
}

JNIEXPORT jfloat JNICALL
Java_com_example_abdullah_audiotwomics_SignalProcessing_getComputeTime(JNIEnv* env, jobject thiz, jlong memoryPointer)
{
    Variables* mainParam = (Variables*) memoryPointer;
    return getCT(mainParam->timer);
}
/*JNIEXPORT jfloat JNICALL*//*
Java_com_example_abdullah_audiotwomics_SignalProcessing_getFilteringTime(JNIEnv* env, jobject thiz, jlong memoryPointer)
{
    Variables* mainParam = (Variables*) memoryPointer;
    return getFT(mainParam->timer);
}*/













int checkRange(float input)
{
    int output;
    if(input>1){
        output = 32767;
    }else if(input<-1){
        output = -32768;
    }else
        output = 32768*input;
    return output;
}
//////////////
///////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*JNIEXPORT jboolean JNICALL
Java_com_google_sample_echo_MainActivity_createSLBufferQueueAudioPlayer(JNIEnv *env, jclass type) {
    SampleFormat sampleFormat;
    memset(&sampleFormat, 0, sizeof(sampleFormat));
    sampleFormat.pcmFormat_ = (uint16_t)engine.bitsPerSample_;
    sampleFormat.framesPerBuf_ = engine.fastPathFramesPerBuf_;

    // SampleFormat.representation_ = SL_ANDROID_PCM_REPRESENTATION_SIGNED_INT;
    sampleFormat.channels_ = (uint16_t)engine.sampleChannels_;
    sampleFormat.sampleRate_ = engine.fastPathSampleRate_;

    engine.player_ = new AudioPlayer(&sampleFormat, engine.slEngineItf_);
    assert(engine.player_);
    if(engine.player_ == nullptr)
        return JNI_FALSE;

    engine.player_->SetBufQueue(engine.recBufQueue_, engine.freeBufQueue_);
    engine.player_->RegisterCallback(EngineService, (void*)&engine);

    return JNI_TRUE;
}

JNIEXPORT void JNICALL
Java_com_google_sample_echo_MainActivity_deleteSLBufferQueueAudioPlayer(JNIEnv *env, jclass type) {
    if(engine.player_) {
        delete engine.player_;
        engine.player_= nullptr;
    }
}



JNIEXPORT void JNICALL
Java_com_google_sample_echo_MainActivity_deleteAudioRecorder(JNIEnv *env, jclass type) {
    if(engine.recorder_)
        delete engine.recorder_;

    engine.recorder_ = nullptr;
}

JNIEXPORT void JNICALL
Java_com_google_sample_echo_MainActivity_startPlay(JNIEnv *env, jclass type) {

    engine.frameCount_  = 0;
    /*
     * start player: make it into waitForData state
     */
 /*   if(SL_BOOLEAN_FALSE == engine.player_->Start()){
        LOGE("====%s failed", __FUNCTION__);
        return;
    }
    engine.recorder_->Start();
}

JNIEXPORT void JNICALL
Java_com_google_sample_echo_MainActivity_stopPlay(JNIEnv *env, jclass type) {
    engine.recorder_->Stop();
    engine.player_ ->Stop();

    delete engine.recorder_;
    delete engine.player_;
    engine.recorder_ = NULL;
    engine.player_ = NULL;
}



uint32_t dbgEngineGetBufCount(void) {
    uint32_t count = engine.player_->dbgGetDevBufCount();
    count += engine.recorder_->dbgGetDevBufCount();
    count += engine.freeBufQueue_->size();
    count += engine.recBufQueue_->size();

    LOGE("Buf Disrtibutions: PlayerDev=%d, RecDev=%d, FreeQ=%d, "
                 "RecQ=%d",
         engine.player_->dbgGetDevBufCount(),
         engine.recorder_->dbgGetDevBufCount(),
         engine.freeBufQueue_->size(),
         engine.recBufQueue_->size());
    if(count != engine.bufCount_) {
        LOGE("====Lost Bufs among the queue(supposed = %d, found = %d)",
             BUF_COUNT, count);
    }
    return count;
}

/*
 * simple message passing for player/recorder to communicate with engine
 *//*
bool EngineService(void* ctx, uint32_t msg, void* data ) {
    assert(ctx == &engine);
    switch (msg) {
        case ENGINE_SERVICE_MSG_KICKSTART_PLAYER:
            engine.player_->PlayAudioBuffers(PLAY_KICKSTART_BUFFER_COUNT);
            // we only allow it to call once, so tell caller do not call
            // anymore
            return false;
        case ENGINE_SERVICE_MSG_RETRIEVE_DUMP_BUFS:
            *(static_cast<uint32_t*>(data)) = dbgEngineGetBufCount();
            break;
        default:
            assert(false);
            return false;
    }

    return true;
}
*/



