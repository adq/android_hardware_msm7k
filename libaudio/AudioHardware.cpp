/*
** Copyright 2008, The Android Open-Source Project
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/

#include <math.h>

//#define LOG_NDEBUG 0
#define LOG_TAG "AudioHardwareMSM72XX"
#include <utils/Log.h>
#include <utils/String8.h>

#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dlfcn.h>
#include <fcntl.h>

// hardware specific functions

#include "AudioHardware.h"
#include <media/AudioRecord.h>

#define LOG_SND_RPC 0  // Set to 1 to log sound RPC's

#define PCM_OUT_DEVICE "/dev/msm_pcm_out"
#define PCM_IN_DEVICE "/dev/msm_pcm_in"
#define PCM_CTL_DEVICE "/dev/msm_pcm_ctl"

namespace android {

static int get_audpp_filter(void);
static int get_audpre_filter(void);
static int acoustic_set_vrmode(int vr_mode);
static int load_audpara(int acoustic_fd);

static uint16_t audpp_adrc_flag;
static uint16_t audpp_eq_flag;
static uint16_t audpp_rx_iir_flag;

static uint16_t 	audpre_tx_agc_flags[9];
static tx_agc_config 	audpre_tx_agc_cfg[9];
static uint16_t 	audpre_ns_flags[9];
static ns_config 	audpre_ns_cfg[9];
static uint16_t 	audpre_tx_iir_flags[18];
static tx_iir_filter 	audpre_tx_iir_cfg[18];

static int cur_vr_mode = 0;

static bt_headset *audpara_headsets;
static bt_headset *audpara_headsets_tail;
uint8_t audpara_A_data[24][128];
uint8_t audpara_U_data[24][128];
uint16_t audpara_F_data[35][160];
uint16_t audpara_D_data[24][10];
uint16_t audpara_G_data[100][160];
static int audpara_ec_off_id;
static uint32_t audpara_default_carkit_id;

static int initialised = 0;

static int audpre_index, tx_iir_index;
const uint32_t AudioHardware::inputSamplingRates[] = {
        8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000
};
// ----------------------------------------------------------------------------

AudioHardware::AudioHardware() :
    mInit(false), mMicMute(true), mBluetoothNrec(true), mBluetoothId(0),
    mOutput(0), mOutputMP3(0), mCurSndDevice(-1),
    SND_DEVICE_CURRENT(-1),
    SND_DEVICE_HANDSET(-1),
    SND_DEVICE_SPEAKER(-1),
    SND_DEVICE_HEADSET(-1),
    SND_DEVICE_BT(-1),
    SND_DEVICE_CARKIT(-1),
    SND_DEVICE_TTY_FULL(-1),
    SND_DEVICE_TTY_VCO(-1),
    SND_DEVICE_TTY_HCO(-1),
    SND_DEVICE_NO_MIC_HEADSET(-1),
    SND_DEVICE_FM_HEADSET(-1),
    SND_DEVICE_HEADSET_AND_SPEAKER(-1),
    SND_DEVICE_FM_SPEAKER(-1),
    SND_DEVICE_BT_EC_OFF(-1)
{
    struct bt_headset* ept = NULL;

    int acoustic_fd = open("/dev/htc-acoustic", O_RDWR);
    if (acoustic_fd < 0) {
        LOGE("Could not open /dev/htc-acoustic");
	return;
    }

    if (load_audpara(acoustic_fd)) {
        LOGE("Could not load audpara");
	close(acoustic_fd);
	return;
    }
    if (get_audpp_filter()) {
        LOGE("Could not load audpp");
	close(acoustic_fd);
	return;
    }
    if (get_audpre_filter()) {
        LOGE("Could not load audpre");
	close(acoustic_fd);
	return;
    }
    
    close(acoustic_fd);
    initialised = 1;
    mInit = true;
    
    ept = audpara_headsets;
    while(ept) {
	    
#define CHECK_FOR(desc) \
        if (!strcmp(ept->name, #desc)) { \
            SND_DEVICE_##desc = ept->id; \
            LOGD("BT MATCH " #desc); \
        } else
        CHECK_FOR(CURRENT)
        CHECK_FOR(HANDSET)
        CHECK_FOR(SPEAKER)
        CHECK_FOR(BT)
        CHECK_FOR(BT_EC_OFF)
        CHECK_FOR(HEADSET)
        CHECK_FOR(CARKIT)
        CHECK_FOR(TTY_FULL)
        CHECK_FOR(TTY_VCO)
        CHECK_FOR(TTY_HCO)
        CHECK_FOR(NO_MIC_HEADSET)
        CHECK_FOR(FM_HEADSET)
        CHECK_FOR(FM_SPEAKER)
        CHECK_FOR(HEADSET_AND_SPEAKER) {}
#undef CHECK_FOR
	ept = ept->next;
    }

    LOGI("AudioHardware initialised");
}

AudioHardware::~AudioHardware()
{
    struct bt_headset *cur;
    
    for (size_t index = 0; index < mInputs.size(); index++) {
        closeInputStream((AudioStreamIn*)mInputs[index]);
    }
    mInputs.clear();
    closeOutputStream((AudioStreamOut*)mOutput);
    closeOutputStream((AudioStreamOut*)mOutputMP3);
    
    cur = audpara_headsets;
    while(cur) {
        struct bt_headset *next = cur->next;
	free(cur);
	cur = next;
    }

    initialised = 0;
    mInit = false;
}

status_t AudioHardware::initCheck()
{
    return mInit ? NO_ERROR : NO_INIT;
}

AudioStreamOut* AudioHardware::openOutputStream(
        uint32_t devices, int *format, uint32_t *channels, uint32_t *sampleRate, status_t *status)
{
    int lFormat = *format;
    
    if ((lFormat >= 0) && (lFormat <= AudioSystem::PCM_16_BIT)) {
        Mutex::Autolock lock(mLock);

        // only one output stream allowed
        if (mOutput) {
            if (status) {
                *status = INVALID_OPERATION;
            }
            return 0;
        }

        // create new output stream
        AudioStreamOutMSM72xx* out = new AudioStreamOutMSM72xx();
        status_t lStatus = out->set(this, devices, format, channels, sampleRate);
        if (status) {
            *status = lStatus;
        }
        if (lStatus == NO_ERROR) {
            mOutput = out;
        } else {
            delete out;
        }
        return mOutput;
    } else if (lFormat == AudioSystem::MP3) {
        Mutex::Autolock lock(mLock);

        // only one output stream allowed
        if (mOutputMP3) {
            if (status) {
                *status = INVALID_OPERATION;
            }
            return 0;
        }

        // create new output stream
        AudioStreamOutMP3MSM72xx* out = new AudioStreamOutMP3MSM72xx();
        status_t lStatus = out->set(this, devices, format, channels, sampleRate);
        if (status) {
            *status = lStatus;
        }
        if (lStatus == NO_ERROR) {
            mOutputMP3 = out;
        } else {
            delete out;
        }
        return mOutputMP3;
	
    } else {
        LOGE("Unknown stream format requested");    
    }

    return 0;
}

void AudioHardware::closeOutputStream(AudioStreamOut* out) {
   
    if (out == NULL)
	return;
    
    int lFormat = out->format();
    if ((lFormat >= 0) && (lFormat <= AudioSystem::PCM_16_BIT) && (out == mOutput)) {
	Mutex::Autolock lock(mLock);
	delete mOutput;
	mOutput = 0;
    } else if ((lFormat == AudioSystem::MP3) && (out == mOutputMP3)) {
	Mutex::Autolock lock(mLock);
	mOutputMP3->standby();
	delete mOutputMP3;
	mOutputMP3 = 0;
    } else {
        LOGW("Attempt to close invalid output stream");
    }
}

AudioStreamIn* AudioHardware::openInputStream(
        uint32_t devices, int *format, uint32_t *channels, uint32_t *sampleRate, status_t *status,
        AudioSystem::audio_in_acoustics acoustic_flags)
{
    // check for valid input source
    if (!AudioSystem::isInputDevice((AudioSystem::audio_devices)devices)) {
        return 0;
    }

    mLock.lock();

    AudioStreamInMSM72xx* in = new AudioStreamInMSM72xx();
    status_t lStatus = in->set(this, devices, format, channels, sampleRate, acoustic_flags);
    if (status) {
        *status = lStatus;
    }
    if (lStatus != NO_ERROR) {
        mLock.unlock();
        delete in;
        return 0;
    }

    mInputs.add(in);
    mLock.unlock();

    return in;
}

void AudioHardware::closeInputStream(AudioStreamIn* in) {
    Mutex::Autolock lock(mLock);

    ssize_t index = mInputs.indexOf((AudioStreamInMSM72xx *)in);
    if (index < 0) {
        LOGW("Attempt to close invalid input stream");
    } else {
        mLock.unlock();
        delete mInputs[index];
        mLock.lock();
        mInputs.removeAt(index);
    }
}

status_t AudioHardware::setMode(int mode)
{
    if ((mode == 2) && cur_vr_mode) {
	acoustic_set_vrmode(0);
	cur_vr_mode = 0;
    }
	
    status_t status = AudioHardwareBase::setMode(mode);
    if (status == NO_ERROR) {
        // make sure that doAudioRouteOrMute() is called by doRouting()
        // even if the new device selected is the same as current one.
        clearCurDevice();
    }
    return status;
}

bool AudioHardware::checkOutputStandby()
{
    if (mOutput)
        if (!mOutput->checkStandby())
            return false;

    return true;
}

status_t AudioHardware::setMicMute(bool state)
{
    Mutex::Autolock lock(mLock);
    return setMicMute_nosync(state);
}

// always call with mutex held
status_t AudioHardware::setMicMute_nosync(bool state)
{
    if (mMicMute != state) {
        mMicMute = state;
        return doAudioRouteOrMute(SND_DEVICE_CURRENT);
    }
    return NO_ERROR;
}

status_t AudioHardware::getMicMute(bool* state)
{
    *state = mMicMute;
    return NO_ERROR;
}

status_t AudioHardware::setParameters(const String8& keyValuePairs)
{
    AudioParameter param = AudioParameter(keyValuePairs);
    String8 value;
    String8 key;
    const char BT_NREC_KEY[] = "bt_headset_nrec";
    const char BT_NAME_KEY[] = "bt_headset_name";
    const char BT_NREC_VALUE_ON[] = "on";


    LOGV("setParameters() %s", keyValuePairs.string());

    if (keyValuePairs.length() == 0) return BAD_VALUE;

    key = String8(BT_NREC_KEY);
    if (param.get(key, value) == NO_ERROR) {
        if (value == BT_NREC_VALUE_ON) {
            mBluetoothNrec = true;
        } else {
            mBluetoothNrec = false;
            LOGI("Turning noise reduction and echo cancellation off for BT "
                 "headset");
        }
    }
    key = String8(BT_NAME_KEY);
    if (param.get(key, value) == NO_ERROR) {
        mBluetoothId = 0;
	struct bt_headset *ept = audpara_headsets;
	while(ept) {
            if (!strcasecmp(value.string(), ept->name)) {
                mBluetoothId = ept->id;
                LOGI("Using custom acoustic parameters for %s", value.string());
                break;
            }
            
            ept = ept->next;
        }
        if (mBluetoothId == 0) {
            LOGI("Using default acoustic parameters "
                 "(%s not in acoustic database)", value.string());
            doRouting();
        }
    }
    return NO_ERROR;
}

String8 AudioHardware::getParameters(const String8& keys)
{
    AudioParameter param = AudioParameter(keys);
    return param.toString();
}

static int acoustic_set_vrmode(int vr_mode)
{
    int fd = open("/dev/htc-acoustic", O_RDWR);
    if (fd < 0) {
        LOGD("Cannot open htc-acoustic");
        return -EPERM;
    }
    
    if (ioctl(fd, SET_VR_MODE, &vr_mode) < 0) {
        LOGE("set vrmode error");
        close(fd);
        return -EPERM;
    }
    cur_vr_mode = vr_mode;

    close(fd);
    return 0;
}

int check_and_set_audpp_parameters(char *buf, int size)
{
    char *p, *ps;
    static const char *const seps = ",";
    int table_num;
    int i, j;
    uint16_t adrc_filter[8];
    eq_filter_type eq[12];
    rx_iir_filter iir_cfg;
    eqalizer eqalizer;
    int fd;
    void *audioeq;
    void *(*eq_cal)(int32_t, int32_t, int32_t, uint16_t, int32_t, int32_t *, int32_t *, uint16_t *);
    uint16_t numerator[6];
    uint16_t denominator[4];
    uint16_t shift[2];

    fd = open(PCM_CTL_DEVICE, O_RDWR);
    if (fd < 0) {
        LOGE("Cannot open PCM Ctl device");
        return -EPERM;
    }

    if (buf[0] == 'A' && buf[1] == '1') {
        /* IIR filter */
	if (!(p = strtok(buf, ",")))
            goto token_err;
	
	/* Table header */	
        table_num = strtol(p + 1, &ps, 10);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
    	/* Table description */
    	if (!(p = strtok(NULL, seps)))
            goto token_err;

	for (i = 0; i < 48; i++) {
	    j = (i >= 40)? i : ((i % 2)? (i - 1) : (i + 1));
            iir_cfg.iir_params[j] = (uint16_t)strtol(p, &ps, 16);
	    if (!(p = strtok(NULL, seps)))
                goto token_err;
        }
        audpp_rx_iir_flag = (uint16_t)strtol(p, &ps, 16);
	if (!(p = strtok(NULL, seps)))
            goto token_err;
	iir_cfg.num_bands = (uint16_t)strtol(p, &ps, 16);
	
        if (ioctl(fd, AUDIO_SET_RX_IIR, &iir_cfg) < 0) {
            LOGE("set rx iir filter error.");
            return -EIO;
        }
    } else if (buf[0] == 'B' && buf[1] == '1') {
        /* This is the ADRC record we are looking for.  Tokenize it */
        if (!(p = strtok(buf, ",")))
            goto token_err;

        /* Table header */
        table_num = strtol(p + 1, &ps, 10);
        if (!(p = strtok(NULL, seps)))
            goto token_err;

        /* Table description */
        if (!(p = strtok(NULL, seps)))
            goto token_err;
        audpp_adrc_flag = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_filter[0] = (uint16_t)strtol(p, &ps, 16);
	
        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_filter[1] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_filter[2] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_filter[3] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_filter[4] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_filter[5] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_filter[6] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_filter[7] = (uint16_t)strtol(p, &ps, 16);

        LOGI("ADRC Filter ADRC FLAG = %02x.", audpp_adrc_flag);
        LOGI("ADRC Filter COMP THRESHOLD = %02x.", adrc_filter[0]);
        LOGI("ADRC Filter COMP SLOPE = %02x.", adrc_filter[1]);
        LOGI("ADRC Filter COMP RMS TIME = %02x.", adrc_filter[2]);
        LOGI("ADRC Filter COMP ATTACK[0] = %02x.", adrc_filter[3]);
        LOGI("ADRC Filter COMP ATTACK[1] = %02x.", adrc_filter[4]);
        LOGI("ADRC Filter COMP RELEASE[0] = %02x.", adrc_filter[5]);
        LOGI("ADRC Filter COMP RELEASE[1] = %02x.", adrc_filter[6]);
        LOGI("ADRC Filter COMP DELAY = %02x.", adrc_filter[7]);

        if (ioctl(fd, AUDIO_SET_ADRC, &adrc_filter) < 0) {
            LOGE("set adrc filter error.");
            return -EIO;
        }
    } else if (buf[0] == 'C' && buf[1] == '1') {
        /* This is the EQ record we are looking for.  Tokenize it */
        if (!(p = strtok(buf, ",")))
            goto token_err;

        /* Table header */
        table_num = strtol(p + 1, &ps, 10);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
        /* Table description */
        if (!(p = strtok(NULL, seps)))
            goto token_err;

        audpp_eq_flag = (uint16_t)strtol(p, &ps, 16);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
        LOGI("EQ flag = %02x.", audpp_eq_flag);

        audioeq = ::dlopen("/system/lib/libaudioeq.so", RTLD_NOW);
        if (audioeq == NULL) {
            LOGE("audioeq library open failure");
            return -1;
        }
        eq_cal = (void *(*) (int32_t, int32_t, int32_t, uint16_t, int32_t, int32_t *, int32_t *, uint16_t *))::dlsym(audioeq, "audioeq_calccoefs");
        memset(&eqalizer, 0, sizeof(eqalizer));
        /* Temp add the bands here */
        eqalizer.bands = 8;
        for (i = 0; i < eqalizer.bands; i++) {

            eq[i].gain = (uint16_t)strtol(p, &ps, 16);

            if (!(p = strtok(NULL, seps)))
                goto token_err;
            eq[i].freq = (uint16_t)strtol(p, &ps, 16);

            if (!(p = strtok(NULL, seps)))
                goto token_err;
            eq[i].type = (uint16_t)strtol(p, &ps, 16);

            if (!(p = strtok(NULL, seps)))
                goto token_err;
            eq[i].qf = (uint16_t)strtol(p, &ps, 16);
		
	    if (i != eqalizer.bands - 1)
		if (!(p = strtok(NULL, seps)))
			goto token_err;
	    
	    //LOGI("gain[%d] = %d", i, eq[i].gain);
            //LOGI("freq[%d] = %d", i, eq[i].freq);
            //LOGI("type[%d] = %d", i, eq[i].type);
            //LOGI("  qf[%d] = %d", i, eq[i].qf);
            eq_cal(eq[i].gain, eq[i].freq, 48000, eq[i].type, eq[i].qf, (int32_t*)numerator, (int32_t *)denominator, shift);
            for (j = 0; j < 6; j++) {
                eqalizer.params[ ( i * 6) + j] = numerator[j];
            }
            for (j = 0; j < 4; j++) {
                eqalizer.params[(eqalizer.bands * 6) + (i * 4) + j] = denominator[j];
            }
            eqalizer.params[(eqalizer.bands * 10) + i] = shift[0];
        }
        ::dlclose(audioeq);

        if (ioctl(fd, AUDIO_SET_EQ, &eqalizer) < 0) {
            LOGE("set Equalizer error.");
            return -EIO;
        }
    }
    close(fd);
    return 0;

token_err:
    LOGE("malformatted audpp control buffer");
    return -EINVAL;
}

static int get_audpp_filter(void)
{
    struct stat st;
    char *read_buf;
    char *next_str, *current_str;
    int csvfd;

    LOGI("get_audpp_filter");
    static const char *const path = 
        "/system/etc/AudioFilter.csv";
    csvfd = open(path, O_RDONLY);
    if (csvfd < 0) {
        /* failed to open normal acoustic file ... */
        LOGE("failed to open AUDIO_NORMAL_FILTER %s: %s (%d).",
             path, strerror(errno), errno);
        return -1;
    } else LOGI("open %s success.", path);

    if (fstat(csvfd, &st) < 0) {
        LOGE("failed to stat %s: %s (%d).",
             path, strerror(errno), errno);
        close(csvfd);
        return -1;
    }

    read_buf = (char *) mmap(0, st.st_size,
                    PROT_READ | PROT_WRITE,
                    MAP_PRIVATE,
                    csvfd, 0);
    
    if (read_buf == MAP_FAILED) {
        LOGE("failed to mmap parameters file: %s (%d)",
             strerror(errno), errno);
        close(csvfd);
        return -1;
    }

    current_str = read_buf;
    next_str = read_buf;
    while (1) {
        int len;
	if (*next_str == 0xff)
	    break;
        next_str = strchr(current_str, '\n');
        if (!next_str)
           break;
        len = next_str - current_str;
        *next_str++ = '\0';
        if (check_and_set_audpp_parameters(current_str, len)) {
            LOGI("failed to set audpp parameters, exiting.");
            munmap(read_buf, st.st_size);
	    close(csvfd);
	    return -1;
        }
        current_str = next_str;
    }

    munmap(read_buf, st.st_size);
    close(csvfd);    
    return 0;
}

static int msm72xx_enable_audpp(uint16_t enable_mask)
{
    int fd;
    
    if (!initialised)
	return 0;
    
    fd = open(PCM_CTL_DEVICE, O_RDWR);
    if (fd < 0) {
        LOGE("Cannot open PCM Ctl device");
        return -EPERM;
    }

    if (audpp_adrc_flag == 0 && (enable_mask & ADRC_ENABLE))
       	enable_mask &= ~ADRC_ENABLE;
    if (audpp_eq_flag == 0 && (enable_mask & EQ_ENABLE))
	enable_mask &= ~EQ_ENABLE;
    if (audpp_rx_iir_flag == 0 && (enable_mask & RX_IIR_ENABLE))
        enable_mask &= ~RX_IIR_ENABLE;   

    LOGE("msm72xx_enable_audpp: 0x%04x", enable_mask);
    if (ioctl(fd, AUDIO_ENABLE_AUDPP, &enable_mask) < 0) {
        LOGE("enable audpp error");
        close(fd);
        return -EPERM;
    }
    
    close(fd);
    return 0;
}

int check_and_set_audpre_parameters(char *buf, int size)
{
    char *p, *ps;
    static const char *const seps = ",";
    int table_num;
    int i, j;

    if (buf[0] == 'A') {
        /* IIR filter */
	if (!(p = strtok(buf, ",")))
            goto token_err;
	
	/* Table header */	
        table_num = strtol(p + 1, &ps, 10);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
    	/* Table description */
    	if (!(p = strtok(NULL, seps)))
            goto token_err;

	for (i = 0; i < 48; i++) {
	    j = (i >= 40)? i : ((i % 2)? (i - 1) : (i + 1));
            audpre_tx_iir_cfg[table_num].iir_params[j] = (uint16_t)strtol(p, &ps, 16);
	    if (!(p = strtok(NULL, seps)))
                goto token_err;
        }
        audpre_tx_iir_flags[table_num] = (uint16_t)strtol(p, &ps, 16);
	if (!(p = strtok(NULL, seps)))
            goto token_err;
	audpre_tx_iir_cfg[table_num].num_bands = (uint16_t)strtol(p, &ps, 16);

    } else if (buf[0] == 'B') {
        /* This is the AGC record we are looking for.  Tokenize it */
        if (!(p = strtok(buf, ",")))
            goto token_err;

        /* Table header */
        table_num = strtol(p + 1, &ps, 10);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
        /* Table description */
        if (!(p = strtok(NULL, seps)))
            goto token_err;
	
        audpre_tx_agc_flags[table_num] = (uint16_t)strtol(p, &ps, 16);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
	
	for (i = 0; i < 20; i++) {
            audpre_tx_agc_cfg[table_num].agc_params[i] = (uint16_t)strtol(p, &ps, 16);
	    if (i != 19)
		if (!(p = strtok(NULL, seps)))
			goto token_err;
        }
    } else if (buf[0] == 'C') {
        /* This is the NS record we are looking for.  Tokenize it */
        if (!(p = strtok(buf, ",")))
            goto token_err;

        /* Table header */
        table_num = strtol(p + 1, &ps, 10);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
        /* Table description */
        if (!(p = strtok(NULL, seps)))
            goto token_err;

	audpre_ns_flags[table_num] = (uint16_t)strtol(p, &ps, 16);
        if (!(p = strtok(NULL, seps)))
            goto token_err;

        for (i = 0; i < 6; i++) {
            audpre_ns_cfg[table_num].ns_params[i] = (uint16_t)strtol(p, &ps, 16);
	    if (i != 5)
		if (!(p = strtok(NULL, seps)))
			goto token_err;
        }
    }
    return 0;

token_err:
    LOGE("malformatted audpre control buffer");
    return -EINVAL;
}

static int get_audpre_filter(void)
{
    struct stat st;
    char *read_buf;
    char *next_str, *current_str;
    int csvfd;

    LOGI("get_audpre_filter");
    static const char *const path = 
        "/system/etc/AudioPreProcess.csv";
    csvfd = open(path, O_RDONLY);
    if (csvfd < 0) {
        /* failed to open normal acoustic file ... */
        LOGE("failed to open AUDIO_PRE_PROCESS %s: %s (%d).",
             path, strerror(errno), errno);
        return -1;
    } else LOGI("open %s success.", path);

    if (fstat(csvfd, &st) < 0) {
        LOGE("failed to stat %s: %s (%d).",
             path, strerror(errno), errno);
        close(csvfd);
        return -1;
    }

    read_buf = (char *) mmap(0, st.st_size,
                    PROT_READ | PROT_WRITE,
                    MAP_PRIVATE,
                    csvfd, 0);
    
    if (read_buf == MAP_FAILED) {
        LOGE("failed to mmap parameters file: %s (%d)",
             strerror(errno), errno);
        close(csvfd);
        return -1;
    }

    current_str = read_buf;
    next_str = read_buf;
    while (1) {
        int len;
	if (*next_str == 0xff)
	    break;
        next_str = strchr(current_str, '\n');
        if (!next_str)
           break;
        len = next_str - current_str;
        *next_str++ = '\0';
        if (check_and_set_audpre_parameters(current_str, len)) {
            LOGI("failed to set audpre parameters, exiting.");
            munmap(read_buf, st.st_size);
	    close(csvfd);
	    return -1;
        }
        current_str = next_str;
    }

    munmap(read_buf, st.st_size);
    close(csvfd);    
    return 0;
}

static int msm72xx_set_audpre_params(int audpre_index, int tx_iir_index)
{
    int fd;
            
    if (!initialised)
	return 0;

    fd = open("/dev/msm_audpre", O_RDWR);
    if (fd < 0) {
        LOGE("Cannot open msm_audpre device");
        return -EPERM;
    }

    if (ioctl(fd, AUDIO_SET_AGC, &audpre_tx_agc_cfg[audpre_index]) < 0) {
        LOGE("AUDIO_SET_AGC failed");
	goto err;
    }
    if (ioctl(fd, AUDIO_SET_NS, &audpre_ns_cfg[audpre_index]) < 0) {
        LOGE("AUDIO_SET_NS failed");
	goto err;
    }
    if (ioctl(fd, AUDIO_SET_TX_IIR, &audpre_tx_iir_cfg[tx_iir_index]) < 0) {
        LOGE("AUDIO_SET_TX_IIR failed");
	goto err;
    }

    close(fd);
    return 0;

err:
    close(fd);
    return -EPERM;
}

static int msm72xx_enable_audpre(int enable_mask, int audpre_index, int tx_iir_index)
{
    int fd;

    if (!initialised)
	return 0;

    fd = open("/dev/msm_audpre", O_RDWR);
    if (fd < 0) {
        LOGE("Cannot open msm_audpre device");
        return -EPERM;
    }

    if (audpre_tx_agc_flags[audpre_index] == 0 && (enable_mask & AUDPRE_AGC_ENABLE))
       	enable_mask &= ~AUDPRE_AGC_ENABLE;
    if (audpre_ns_flags[audpre_index] == 0 && (enable_mask & AUDPRE_NS_ENABLE))
	enable_mask &= ~AUDPRE_NS_ENABLE;
    if (audpre_tx_iir_flags[tx_iir_index] == 0 && (enable_mask & AUDPRE_TX_IIR_ENABLE))
        enable_mask &= ~AUDPRE_TX_IIR_ENABLE;   

    LOGE("msm72xx_enable_audpre: 0x%04x", enable_mask);
    if (ioctl(fd, AUDIO_ENABLE_AUDPRE, &enable_mask) < 0) {
        LOGE("enable audpre error");
        close(fd);
        return -EPERM;
    }
    
    close(fd);
    return 0;
}

static struct bt_headset *add_bt_headset(int id, const char *name)
{
	struct bt_headset *newset = (struct bt_headset *) malloc(sizeof(struct bt_headset));
	if (newset == NULL)
		return NULL;
	memset(newset, 0, sizeof(struct bt_headset));

	if (audpara_headsets == NULL)
		audpara_headsets = newset;
	if (audpara_headsets_tail != NULL)
		audpara_headsets_tail->next = newset;
	audpara_headsets_tail = newset;
	
	newset->id = id;
	strncpy(newset->name, name, 31);

	return newset;
}

int check_and_set_audpara_parameters(char *buf, int size)
{
    char *p, *ps;
    static const char *const seps = ",";
    int table_num;
    int i, j;

    if (buf[0] == 'A') {
	if (!(p = strtok(buf, ",")))
            goto token_err;
	
	/* Table header */	
        table_num = strtol(p + 1, &ps, 10);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
    	/* Table description */
    	p = strtok(NULL, seps);

	i = 0;
	while(p) {
            audpara_A_data[table_num][i++] = (uint8_t)strtol(p, &ps, 16);
	    p = strtok(NULL, seps);
        }

    } else if (buf[0] == 'U') {
        if (!(p = strtok(buf, ",")))
            goto token_err;

        /* Table header */
        table_num = strtol(p + 1, &ps, 10);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
    	/* Table description */
    	p = strtok(NULL, seps);
	
	i = 0;
	while(p) {
            audpara_U_data[table_num][i++] = (uint8_t)strtol(p, &ps, 16);
	    p = strtok(NULL, seps);
        }
        
    } else if (buf[0] == 'F') {
        if (!(p = strtok(buf, ",")))
            goto token_err;

        /* Table header */
        table_num = strtol(p + 1, &ps, 10);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
    	/* Table description */
    	p = strtok(NULL, seps);
	
	i = 0;
	while(p) {
            audpara_F_data[table_num][i++] = (uint16_t)strtol(p, &ps, 16);
	    p = strtok(NULL, seps);
        }
        
    } else if (buf[0] == 'D') {
        if (!(p = strtok(buf, ",")))
            goto token_err;

        /* Table header */
        table_num = strtol(p + 1, &ps, 10);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
    	/* Table description */
    	p = strtok(NULL, seps);
	
	i = 0;
	while(p) {
            audpara_D_data[table_num][i++] = (uint16_t)strtol(p, &ps, 16);
	    p = strtok(NULL, seps);
        }
     
    } else if (buf[0] == 'G') {
        /* This is the AGC record we are looking for.  Tokenize it */
        if (!(p = strtok(buf, ",")))
            goto token_err;

        /* Table header */
        table_num = strtol(p + 1, &ps, 10);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
	
    	/* Table description */
	if (table_num != 0) {
		struct bt_headset *newheadset = add_bt_headset(table_num + 11, p);		
		if (!strcmp(newheadset->name, "BT_EC_OFF"))
			audpara_ec_off_id = newheadset->id;
		else if (!strcmp(newheadset->name, "Default_Carkit"))
			audpara_default_carkit_id = newheadset->id;
	}
	
    	p = strtok(NULL, seps);
	
	i = 0;
	while(p) {
            audpara_G_data[table_num][i++] = (uint16_t)strtol(p, &ps, 16);
	    p = strtok(NULL, seps);
        }
    }
    return 0;

token_err:
    LOGE("malformatted audpara control buffer");
    return -EINVAL;
}

static int load_audpara(int acoustic_fd)
{
    struct stat st;
    char *read_buf;
    char *acoustic_buf;
    char *next_str, *current_str;
    int csvfd;
    uint32_t magic = 0x12345678;

    LOGI("get_audpara_filter");
    static const char *const path = 
        "/system/etc/AudioPara4.csv";
    csvfd = open(path, O_RDONLY);
    if (csvfd < 0) {
        /* failed to open normal acoustic file ... */
        LOGE("failed to open AUDIO_PARA %s: %s (%d).",
             path, strerror(errno), errno);
        return -1;
    } else LOGI("open %s success.", path);

    if (fstat(csvfd, &st) < 0) {
        LOGE("failed to stat %s: %s (%d).",
             path, strerror(errno), errno);
        close(csvfd);
        return -1;
    }

    read_buf = (char *) mmap(0, st.st_size,
                    PROT_READ | PROT_WRITE,
                    MAP_PRIVATE,
                    csvfd, 0);
    
    if (read_buf == MAP_FAILED) {
        LOGE("failed to mmap parameters file: %s (%d)",
             strerror(errno), errno);
        close(csvfd);
        return -1;
    }

    audpara_headsets = NULL;
    audpara_headsets_tail = NULL;
    audpara_ec_off_id = 3;
    audpara_default_carkit_id = 0;
    memset(audpara_A_data, 0, sizeof(audpara_A_data));
    memset(audpara_U_data, 0, sizeof(audpara_U_data));
    memset(audpara_F_data, 0, sizeof(audpara_F_data));
    memset(audpara_D_data, 0, sizeof(audpara_D_data));
    memset(audpara_G_data, 0, sizeof(audpara_G_data));
    
    add_bt_headset(0, "HANDSET");
    add_bt_headset(1, "SPEAKER");
    add_bt_headset(2, "HEADSET");
    add_bt_headset(3, "BT");
    struct bt_headset *carkit_bt_headset = add_bt_headset(4, "CARKIT");
    add_bt_headset(5, "TTY_FULL");
    add_bt_headset(6, "TTY_VCO");
    add_bt_headset(7, "TTY_HCO");
    add_bt_headset(8, "NO_MIC_HEADSET");
    add_bt_headset(9, "FM_HEADSET");
    add_bt_headset(10, "HEADSET_AND_SPEAKER");
    add_bt_headset(11, "FM_SPEAKER");    

    current_str = read_buf;
    next_str = read_buf;
    while (1) {
        int len;
	if (*next_str == 0xff)
	    break;
        next_str = strchr(current_str, '\n');
        if (!next_str)
           break;
        len = next_str - current_str;
        *next_str++ = '\0';
        if (check_and_set_audpara_parameters(current_str, len)) {
            LOGI("failed to set audpara parameters, exiting.");
            munmap(read_buf, st.st_size);
	    close(csvfd);
	    return -1;
        }
        current_str = next_str;
    }

    munmap(read_buf, st.st_size);
    close(csvfd);

    if (audpara_ec_off_id == 3)
	add_bt_headset(3, "BT_EC_OFF");
    if (audpara_default_carkit_id != 0)
	carkit_bt_headset->id = audpara_default_carkit_id;
    else
	carkit_bt_headset->id = 3;
    add_bt_headset(256, "CURRENT");    

    acoustic_buf = (char *) mmap(0, 0xc8e4,
				PROT_READ | PROT_WRITE,
				MAP_SHARED,
				acoustic_fd, 0);
    
    if (acoustic_buf == MAP_FAILED) {
        LOGE("failed to mmap acoustic device: %s (%d)",
             strerror(errno), errno);
        return -1;
    }
    
    memcpy(acoustic_buf, audpara_A_data, 0xc00);
    memcpy(acoustic_buf + 0xc00, audpara_U_data, 0xc00);
    memcpy(acoustic_buf + 0x1800, audpara_F_data, 0x2bc0);
    memcpy(acoustic_buf + 0x4a00, audpara_G_data, 0x7d00);
    memcpy(acoustic_buf + 0xc700, audpara_D_data, 0x1e0);
    memcpy(acoustic_buf + 0xc8e0, &magic, 4);
    munmap(acoustic_buf, 0xc8e4);

    ioctl(acoustic_fd, ACOUSTIC_ARM11_DONE);

    return 0;
}


static unsigned calculate_audpre_table_index(unsigned index)
{
    switch (index) {
        case 48000:    return SAMP_RATE_INDX_48000;
        case 44100:    return SAMP_RATE_INDX_44100;
        case 32000:    return SAMP_RATE_INDX_32000;
        case 24000:    return SAMP_RATE_INDX_24000;
        case 22050:    return SAMP_RATE_INDX_22050;
        case 16000:    return SAMP_RATE_INDX_16000;
        case 12000:    return SAMP_RATE_INDX_12000;
        case 11025:    return SAMP_RATE_INDX_11025;
        case 8000:    return SAMP_RATE_INDX_8000;
        default:     return -1;
    }
}
size_t AudioHardware::getInputBufferSize(uint32_t sampleRate, int format, int channelCount)
{
    if (format != AudioSystem::PCM_16_BIT) {
        LOGW("getInputBufferSize bad format: %d", format);
        return 0;
    }
    if (channelCount < 1 || channelCount > 2) {
        LOGW("getInputBufferSize bad channel count: %d", channelCount);
        return 0;
    }

    return 2048*channelCount;
}

static status_t set_volume_rpc(uint32_t device,
                               uint32_t method,
                               uint32_t volume)
{
    int fd;
#if LOG_SND_RPC
    LOGD("rpc_snd_set_volume(%d, %d, %d)\n", device, method, volume);
#endif

    if (device == -1UL) return NO_ERROR;

    fd = open("/dev/msm_snd", O_RDWR);
    if (fd < 0) {
        LOGE("Can not open snd device");
        return -EPERM;
    }
    /* rpc_snd_set_volume(
     *     device,            # Any hardware device enum, including
     *                        # SND_DEVICE_CURRENT
     *     method,            # must be SND_METHOD_VOICE to do anything useful
     *     volume,            # integer volume level, in range [0,5].
     *                        # note that 0 is audible (not quite muted)
     *  )
     * rpc_snd_set_volume only works for in-call sound volume.
     */
     struct msm_snd_volume_config args;
     args.device = device;
     args.method = method;
     args.volume = volume;

     if (ioctl(fd, SND_SET_VOLUME, &args) < 0) {
         LOGE("snd_set_volume error.");
         close(fd);
         return -EIO;
     }
     close(fd);
     return NO_ERROR;
}

status_t AudioHardware::setVoiceVolume(float v)
{
    if (v < 0.0) {
        LOGW("setVoiceVolume(%f) under 0.0, assuming 0.0\n", v);
        v = 0.0;
    } else if (v > 1.0) {
        LOGW("setVoiceVolume(%f) over 1.0, assuming 1.0\n", v);
        v = 1.0;
    }

    int vol = lrint(v * 5.0);
    LOGD("setVoiceVolume(%f)\n", v);
    LOGI("Setting in-call volume to %d (available range is 0 to 5)\n", vol);

    Mutex::Autolock lock(mLock);
    set_volume_rpc(SND_DEVICE_CURRENT, SND_METHOD_VOICE, vol);
    return NO_ERROR;
}

status_t AudioHardware::setMasterVolume(float v)
{
    Mutex::Autolock lock(mLock);
    int vol = ceil(v * 5.0);
    LOGI("Set master volume to %d.\n", vol);
    /*
    set_volume_rpc(SND_DEVICE_HANDSET, SND_METHOD_VOICE, vol);
    set_volume_rpc(SND_DEVICE_SPEAKER, SND_METHOD_VOICE, vol);
    set_volume_rpc(SND_DEVICE_BT,      SND_METHOD_VOICE, vol);
    set_volume_rpc(SND_DEVICE_HEADSET, SND_METHOD_VOICE, vol);
    */
    // We return an error code here to let the audioflinger do in-software
    // volume on top of the maximum volume that we set through the SND API.
    // return error - software mixer will handle it
    return -1;
}

static status_t do_route_audio_rpc(uint32_t device,
                                   bool ear_mute, bool mic_mute)
{
    if (device == -1UL)
        return NO_ERROR;

    int fd;
#if LOG_SND_RPC
    LOGD("rpc_snd_set_device(%d, %d, %d)\n", device, ear_mute, mic_mute);
#endif

    fd = open("/dev/msm_snd", O_RDWR);
    if (fd < 0) {
        LOGE("Can not open snd device");
        return -EPERM;
    }
    // RPC call to switch audio path
    /* rpc_snd_set_device(
     *     device,            # Hardware device enum to use
     *     ear_mute,          # Set mute for outgoing voice audio
     *                        # this should only be unmuted when in-call
     *     mic_mute,          # Set mute for incoming voice audio
     *                        # this should only be unmuted when in-call or
     *                        # recording.
     *  )
     */
    struct msm_snd_device_config args;
    args.device = device;
    args.ear_mute = ear_mute ? SND_MUTE_MUTED : SND_MUTE_UNMUTED;
    args.mic_mute = mic_mute ? SND_MUTE_MUTED : SND_MUTE_UNMUTED;

    if (ioctl(fd, SND_SET_DEVICE, &args) < 0) {
        LOGE("snd_set_device error.");
        close(fd);
        return -EIO;
    }

    close(fd);
    return NO_ERROR;
}

// always call with mutex held
status_t AudioHardware::doAudioRouteOrMute(uint32_t device)
{
    if (device == (uint32_t)SND_DEVICE_BT || device == (uint32_t)SND_DEVICE_CARKIT) {
        if (mBluetoothId) {
            device = mBluetoothId;
        } else if (!mBluetoothNrec) {
            device = SND_DEVICE_BT_EC_OFF;
        }
    }
    LOGV("doAudioRouteOrMute() device %x, mMode %d, mMicMute %d", device, mMode, mMicMute);
    return do_route_audio_rpc(device,
                              mMode != AudioSystem::MODE_IN_CALL, mMicMute);
}

status_t AudioHardware::doRouting()
{
    Mutex::Autolock lock(mLock);
    uint32_t outputDevices = mOutput->devices();
    status_t ret = NO_ERROR;
    int audProcess = (ADRC_DISABLE | EQ_DISABLE | RX_IIR_DISABLE);
    AudioStreamInMSM72xx *input = getActiveInput_l();
    uint32_t inputDevice = (input == NULL) ? 0 : input->devices();
    int sndDevice = -1;

    if (inputDevice != 0) {
        LOGI("do input routing device %x\n", inputDevice);
        if (inputDevice & AudioSystem::DEVICE_IN_BLUETOOTH_SCO_HEADSET) {
            LOGI("Routing audio to Bluetooth PCM\n");
            sndDevice = SND_DEVICE_BT;
        } else if (inputDevice & AudioSystem::DEVICE_IN_WIRED_HEADSET) {
            if ((outputDevices & AudioSystem::DEVICE_OUT_WIRED_HEADSET) &&
                (outputDevices & AudioSystem::DEVICE_OUT_SPEAKER)) {
                LOGI("Routing audio to Wired Headset and Speaker\n");
                sndDevice = SND_DEVICE_HEADSET_AND_SPEAKER;
                audProcess = (ADRC_ENABLE | EQ_ENABLE | RX_IIR_ENABLE);
            } else {
                LOGI("Routing audio to Wired Headset\n");
                sndDevice = SND_DEVICE_HEADSET;
            }
        } else {
            if (outputDevices & AudioSystem::DEVICE_OUT_SPEAKER) {
                LOGI("Routing audio to Speakerphone\n");
                sndDevice = SND_DEVICE_SPEAKER;
                audProcess = (ADRC_ENABLE | EQ_ENABLE | RX_IIR_ENABLE);
            } else {
                LOGI("Routing audio to Handset\n");
                sndDevice = SND_DEVICE_HANDSET;
            }
        }
    }
    // if inputDevice == 0, restore output routing

    if (sndDevice == -1) {
        if (outputDevices & (outputDevices - 1)) {
            if ((outputDevices & AudioSystem::DEVICE_OUT_SPEAKER) == 0) {
                LOGW("Hardware does not support requested route combination (%#X),"
                     " picking closest possible route...", outputDevices);
            }
        }

        if (outputDevices &
            (AudioSystem::DEVICE_OUT_BLUETOOTH_SCO | AudioSystem::DEVICE_OUT_BLUETOOTH_SCO_HEADSET)) {
            LOGI("Routing audio to Bluetooth PCM\n");
            sndDevice = SND_DEVICE_BT;
        } else if (outputDevices & AudioSystem::DEVICE_OUT_BLUETOOTH_SCO_CARKIT) {
            LOGI("Routing audio to Bluetooth PCM\n");
            sndDevice = SND_DEVICE_CARKIT;
        } else if ((outputDevices & AudioSystem::DEVICE_OUT_WIRED_HEADSET) &&
                   (outputDevices & AudioSystem::DEVICE_OUT_SPEAKER)) {
            LOGI("Routing audio to Wired Headset and Speaker\n");
            sndDevice = SND_DEVICE_HEADSET_AND_SPEAKER;
            audProcess = (ADRC_ENABLE | EQ_ENABLE | RX_IIR_ENABLE);
        } else if (outputDevices & AudioSystem::DEVICE_OUT_WIRED_HEADPHONE) {
            if (outputDevices & AudioSystem::DEVICE_OUT_SPEAKER) {
                LOGI("Routing audio to No microphone Wired Headset and Speaker (%d,%x)\n", mMode, outputDevices);
                sndDevice = SND_DEVICE_HEADSET_AND_SPEAKER;
                audProcess = (ADRC_ENABLE | EQ_ENABLE | RX_IIR_ENABLE);
            } else {
                LOGI("Routing audio to No microphone Wired Headset (%d,%x)\n", mMode, outputDevices);
                sndDevice = SND_DEVICE_NO_MIC_HEADSET;
            }
        } else if (outputDevices & AudioSystem::DEVICE_OUT_WIRED_HEADSET) {
            LOGI("Routing audio to Wired Headset\n");
            sndDevice = SND_DEVICE_HEADSET;
        } else if (outputDevices & AudioSystem::DEVICE_OUT_SPEAKER) {
            LOGI("Routing audio to Speakerphone\n");
            sndDevice = SND_DEVICE_SPEAKER;
            audProcess = (ADRC_ENABLE | EQ_ENABLE | RX_IIR_ENABLE);
        } else {
            LOGI("Routing audio to Handset\n");
            sndDevice = SND_DEVICE_HANDSET;
        }
    }

    if (sndDevice != -1 && sndDevice != mCurSndDevice) {
        ret = doAudioRouteOrMute(sndDevice);
	msm72xx_enable_audpp(audProcess);
        mCurSndDevice = sndDevice;
    }

    return ret;
}

status_t AudioHardware::checkMicMute()
{
    Mutex::Autolock lock(mLock);
    if (mMode != AudioSystem::MODE_IN_CALL) {
        setMicMute_nosync(true);
    }

    return NO_ERROR;
}

status_t AudioHardware::dumpInternals(int fd, const Vector<String16>& args)
{
    const size_t SIZE = 256;
    char buffer[SIZE];
    String8 result;
    result.append("AudioHardware::dumpInternals\n");
    snprintf(buffer, SIZE, "\tmInit: %s\n", mInit? "true": "false");
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmMicMute: %s\n", mMicMute? "true": "false");
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmBluetoothNrec: %s\n", mBluetoothNrec? "true": "false");
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmBluetoothId: %d\n", mBluetoothId);
    result.append(buffer);
    ::write(fd, result.string(), result.size());
    return NO_ERROR;
}

status_t AudioHardware::dump(int fd, const Vector<String16>& args)
{
    dumpInternals(fd, args);
    for (size_t index = 0; index < mInputs.size(); index++) {
        mInputs[index]->dump(fd, args);
    }

    if (mOutput) {
        mOutput->dump(fd, args);
    }
    return NO_ERROR;
}

uint32_t AudioHardware::getInputSampleRate(uint32_t sampleRate)
{
    uint32_t i;
    uint32_t prevDelta;
    uint32_t delta;

    for (i = 0, prevDelta = 0xFFFFFFFF; i < sizeof(inputSamplingRates)/sizeof(uint32_t); i++, prevDelta = delta) {
        delta = abs(sampleRate - inputSamplingRates[i]);
        if (delta > prevDelta) break;
    }
    // i is always > 0 here
    return inputSamplingRates[i-1];
}

// getActiveInput_l() must be called with mLock held
AudioHardware::AudioStreamInMSM72xx *AudioHardware::getActiveInput_l()
{
    for (size_t i = 0; i < mInputs.size(); i++) {
        // return first input found not being in standby mode
        // as only one input can be in this state
        if (mInputs[i]->state() > AudioStreamInMSM72xx::AUDIO_INPUT_CLOSED) {
            return mInputs[i];
        }
    }

    return NULL;
}
// ----------------------------------------------------------------------------

AudioHardware::AudioStreamOutMSM72xx::AudioStreamOutMSM72xx() :
    mHardware(0), mFd(-1), mStartCount(0), mRetryCount(0), mStandby(true), mDevices(0)
{
}

status_t AudioHardware::AudioStreamOutMSM72xx::set(
        AudioHardware* hw, uint32_t devices, int *pFormat, uint32_t *pChannels, uint32_t *pRate)
{
    int lFormat = pFormat ? *pFormat : 0;
    uint32_t lChannels = pChannels ? *pChannels : 0;
    uint32_t lRate = pRate ? *pRate : 0;

    mHardware = hw;

    // fix up defaults
    if (lFormat == 0) lFormat = format();
    if (lChannels == 0) lChannels = channels();
    if (lRate == 0) lRate = sampleRate();

    // check values
    if ((lFormat != format()) ||
        (lChannels != channels()) ||
        (lRate != sampleRate())) {
        if (pFormat) *pFormat = format();
        if (pChannels) *pChannels = channels();
        if (pRate) *pRate = sampleRate();
        return BAD_VALUE;
    }

    if (pFormat) *pFormat = lFormat;
    if (pChannels) *pChannels = lChannels;
    if (pRate) *pRate = lRate;

    mDevices = devices;

    return NO_ERROR;
}

AudioHardware::AudioStreamOutMSM72xx::~AudioStreamOutMSM72xx()
{
    if (mFd >= 0) close(mFd);
}

ssize_t AudioHardware::AudioStreamOutMSM72xx::write(const void* buffer, size_t bytes)
{
    // LOGD("AudioStreamOutMSM72xx::write(%p, %u)", buffer, bytes);
    status_t status = NO_INIT;
    size_t count = bytes;
    const uint8_t* p = static_cast<const uint8_t*>(buffer);

    if (mStandby) {

        // open driver
        LOGV("open driver");
        status = ::open("/dev/msm_pcm_out", O_RDWR);
        if (status < 0) {
            LOGE("Cannot open /dev/msm_pcm_out errno: %d", errno);
            goto Error;
        }
        mFd = status;

        // configuration
        LOGV("get config");
        struct msm_audio_config config;
        status = ioctl(mFd, AUDIO_GET_CONFIG, &config);
        if (status < 0) {
            LOGE("Cannot read config");
            goto Error;
        }

        LOGV("set config");
        config.channel_count = AudioSystem::popCount(channels());
        config.sample_rate = sampleRate();
        config.buffer_size = bufferSize();
        config.buffer_count = AUDIO_HW_NUM_OUT_BUF;
        config.codec_type = CODEC_TYPE_PCM;
        status = ioctl(mFd, AUDIO_SET_CONFIG, &config);
        if (status < 0) {
            LOGE("Cannot set config");
            goto Error;
        }

        LOGV("buffer_size: %u", config.buffer_size);
        LOGV("buffer_count: %u", config.buffer_count);
        LOGV("channel_count: %u", config.channel_count);
        LOGV("sample_rate: %u", config.sample_rate);

        // fill 2 buffers before AUDIO_START
        mStartCount = AUDIO_HW_NUM_OUT_BUF;
        mStandby = false;
    }

    while (count) {
        ssize_t written = ::write(mFd, p, count);
        if (written >= 0) {
            count -= written;
            p += written;
        } else {
            if (errno != EAGAIN) return written;
            mRetryCount++;
            LOGW("EAGAIN - retry");
        }
    }

    // start audio after we fill 2 buffers
    if (mStartCount) {
        if (--mStartCount == 0) {
            ioctl(mFd, AUDIO_START, 0);
        }
    }
    return bytes;

Error:
    if (mFd >= 0) {
        ::close(mFd);
        mFd = -1;
    }
    // Simulate audio output timing in case of error
    usleep(bytes * 1000000 / frameSize() / sampleRate());

    return status;
}

status_t AudioHardware::AudioStreamOutMSM72xx::standby()
{
    status_t status = NO_ERROR;
    if (!mStandby && mFd >= 0) {
        ::close(mFd);
        mFd = -1;
    }
    mStandby = true;
    return status;
}

status_t AudioHardware::AudioStreamOutMSM72xx::dump(int fd, const Vector<String16>& args)
{
    const size_t SIZE = 256;
    char buffer[SIZE];
    String8 result;
    result.append("AudioStreamOutMSM72xx::dump\n");
    snprintf(buffer, SIZE, "\tsample rate: %d\n", sampleRate());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tbuffer size: %d\n", bufferSize());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tchannels: %d\n", channels());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tformat: %d\n", format());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmHardware: %p\n", mHardware);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmFd: %d\n", mFd);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmStartCount: %d\n", mStartCount);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmRetryCount: %d\n", mRetryCount);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmStandby: %s\n", mStandby? "true": "false");
    result.append(buffer);
    ::write(fd, result.string(), result.size());
    return NO_ERROR;
}

bool AudioHardware::AudioStreamOutMSM72xx::checkStandby()
{
    return mStandby;
}


status_t AudioHardware::AudioStreamOutMSM72xx::setParameters(const String8& keyValuePairs)
{
    AudioParameter param = AudioParameter(keyValuePairs);
    String8 key = String8(AudioParameter::keyRouting);
    status_t status = NO_ERROR;
    int device;
    LOGV("AudioStreamOutMSM72xx::setParameters() %s", keyValuePairs.string());

    if (param.getInt(key, device) == NO_ERROR) {
        mDevices = device;
        LOGV("set output routing %x", mDevices);
        status = mHardware->doRouting();
        param.remove(key);
    }

    if (param.size()) {
        status = BAD_VALUE;
    }
    return status;
}

String8 AudioHardware::AudioStreamOutMSM72xx::getParameters(const String8& keys)
{
    AudioParameter param = AudioParameter(keys);
    String8 value;
    String8 key = String8(AudioParameter::keyRouting);

    if (param.get(key, value) == NO_ERROR) {
        LOGV("get routing %x", mDevices);
        param.addInt(key, (int)mDevices);
    }

    LOGV("AudioStreamOutMSM72xx::getParameters() %s", param.toString().string());
    return param.toString();
}

status_t AudioHardware::AudioStreamOutMSM72xx::getRenderPosition(uint32_t *dspFrames)
{
    //TODO: enable when supported by driver
    return INVALID_OPERATION;
}

// ----------------------------------------------------------------------------

AudioHardware::AudioStreamOutMP3MSM72xx::AudioStreamOutMP3MSM72xx() :
    mHardware(0), mFd(-1), mStartCount(0), mRetryCount(0), mChannelCount(2), mSampleRate(44100), mStandby(true), mVolume(8192)
{
}

status_t AudioHardware::AudioStreamOutMP3MSM72xx::set(
        AudioHardware* hw, uint32_t devices, int *pFormat, uint32_t *pChannels, uint32_t *pRate)
{
    uint32_t lChannels = pChannels ? *pChannels : 0;
    uint32_t lSampleRate = pRate ? *pRate : 0;
    if (lChannels == 0) lChannels = channels();
    if (lSampleRate == 0) lSampleRate = sampleRate();

    mChannelCount = lChannels;
    mSampleRate = lSampleRate;
    mHardware = hw;

    return NO_ERROR;
}

AudioHardware::AudioStreamOutMP3MSM72xx::~AudioStreamOutMP3MSM72xx()
{
    if (mFd >= 0) close(mFd);
}

status_t AudioHardware::AudioStreamOutMP3MSM72xx::setVolume(float left, float right)
{
	if (left < 0.0)
		left = 0.0;
	if (right > 1.0)
		left = 1.0;
	
	mVolume = (int) (8192 * left);
	if (mFd >= 0)
		return ioctl(mFd, AUDIO_SET_VOLUME, mVolume);

	return NO_INIT;
}

status_t AudioHardware::AudioStreamOutMP3MSM72xx::pause() 
{
    int r = NO_INIT;

    if (mFd >= 0) {
	r = ioctl(mFd, AUDIO_ADSP_PAUSE);
	if (r == 0)
	    r = ioctl(mFd, AUDIO_FLUSH);
    }

    return r;
}

status_t AudioHardware::AudioStreamOutMP3MSM72xx::resume()
{
    if (mFd >= 0)
        return ioctl(mFd, AUDIO_ADSP_RESUME);    
    
    return NO_INIT;
}

status_t AudioHardware::AudioStreamOutMP3MSM72xx::get_ADSP_State(uint32_t *byte_count, uint32_t *sample_count)
{
    struct msm_audio_stats stats;
    int r = NO_INIT;

    if (mFd >= 0) {
	r = ioctl(mFd, AUDIO_GET_STATS, &stats);
	if (r == 0) {
	    *byte_count = stats.out_bytes;
	    *sample_count = stats.sample_count;
	}
    }

    return r;
}

status_t AudioHardware::AudioStreamOutMP3MSM72xx::Wait_Complete()
{
    if (mFd >= 0)
        return ioctl(mFd, AUDIO_WAIT_ADSP_DONE);    
    
    return NO_INIT;
}

ssize_t AudioHardware::AudioStreamOutMP3MSM72xx::write(const void* buffer, size_t bytes)
{
    status_t status = NO_INIT;
    size_t count = bytes;
    const uint8_t* p = static_cast<const uint8_t*>(buffer);

    if (mStandby) {

        // open driver
        LOGV("open msm_mp3 driver");
        status = ::open("/dev/msm_mp3", O_RDWR);
        if (status < 0) {
            LOGE("Cannot open /dev/msm_mp3 errno: %d", errno);
            goto Error;
        }
        mFd = status;

        // configuration
        struct msm_audio_config config;
        status = ioctl(mFd, AUDIO_GET_CONFIG, &config);
        if (status < 0) {
            LOGE("Cannot read config");
            goto Error;
        }

        config.channel_count = mChannelCount;
        config.sample_rate = mSampleRate;
        config.buffer_size = bufferSize();
        config.buffer_count = AUDIO_HW_NUM_OUT_BUF;
        config.codec_type = CODEC_TYPE_MP3;
        status = ioctl(mFd, AUDIO_SET_CONFIG, &config);
        if (status < 0) {
            LOGE("Cannot set config");
            goto Error;
        }

        status = ioctl(mFd, AUDIO_SET_VOLUME, mVolume);
        if (status < 0)
            goto Error;

        // fill 1 buffer before AUDIO_START
        mStartCount = 1;
        mStandby = false;
    }

    while (count) {
        ssize_t written = ::write(mFd, p, count);
        if (written >= 0) {
            count -= written;
            p += written;
        } else {
            if (errno != EAGAIN) return written;
            mRetryCount++;
            LOGW("EAGAIN - retry");
        }
    }

    // start audio after we fill 2 buffers
    if (mStartCount) {
        if (--mStartCount == 0) {
            ioctl(mFd, AUDIO_START, 0);
        }
    }
    return bytes;

Error:
    if (mFd >= 0) {
        ::close(mFd);
        mFd = -1;
    }
    // Simulate audio output timing in case of error
    usleep(bytes * 1000000 / frameSize() / sampleRate());

    return status;
}

status_t AudioHardware::AudioStreamOutMP3MSM72xx::standby()
{
    status_t status = NO_ERROR;
    if (!mStandby && mFd >= 0) {
        ::close(mFd);
        mFd = -1;
    }
    mStandby = true;
    return status;
}

status_t AudioHardware::AudioStreamOutMP3MSM72xx::dump(int fd, const Vector<String16>& args)
{
    const size_t SIZE = 256;
    char buffer[SIZE];
    String8 result;
    result.append("AudioStreamOutMSM72xx::dump\n");
    snprintf(buffer, SIZE, "\tsample rate: %d\n", sampleRate());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tbuffer size: %d\n", bufferSize());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tchannels: %d\n", channels());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tformat: %d\n", format());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmHardware: %p\n", mHardware);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmFd: %d\n", mFd);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmStartCount: %d\n", mStartCount);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmRetryCount: %d\n", mRetryCount);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmStandby: %s\n", mStandby? "true": "false");
    result.append(buffer);
    ::write(fd, result.string(), result.size());
    return NO_ERROR;
}

bool AudioHardware::AudioStreamOutMP3MSM72xx::checkStandby()
{
    return mStandby;
}


status_t AudioHardware::AudioStreamOutMP3MSM72xx::setParameters(const String8& keyValuePairs)
{
    return NO_ERROR;
}

String8 AudioHardware::AudioStreamOutMP3MSM72xx::getParameters(const String8& keys)
{
    AudioParameter param = AudioParameter(keys);
    return param.toString();
}

status_t AudioHardware::AudioStreamOutMP3MSM72xx::getRenderPosition(uint32_t *dspFrames)
{
    //TODO: enable when supported by driver
    return INVALID_OPERATION;
}


// ----------------------------------------------------------------------------

AudioHardware::AudioStreamInMSM72xx::AudioStreamInMSM72xx() :
    mHardware(0), mFd(-1), mState(AUDIO_INPUT_CLOSED), mRetryCount(0),
    mFormat(AUDIO_HW_IN_FORMAT), mChannels(AUDIO_HW_IN_CHANNELS),
    mSampleRate(AUDIO_HW_IN_SAMPLERATE), mBufferSize(AUDIO_HW_IN_BUFFERSIZE),
    mAcoustics((AudioSystem::audio_in_acoustics)0), mDevices(0)
{
}

status_t AudioHardware::AudioStreamInMSM72xx::set(
        AudioHardware* hw, uint32_t devices, int *pFormat, uint32_t *pChannels, uint32_t *pRate,
        AudioSystem::audio_in_acoustics acoustic_flags)
{
    if (pFormat == 0 || *pFormat != AUDIO_HW_IN_FORMAT) {
        *pFormat = AUDIO_HW_IN_FORMAT;
        return BAD_VALUE;
    }
    if (pRate == 0) {
        return BAD_VALUE;
    }
    uint32_t rate = hw->getInputSampleRate(*pRate);
    if (rate != *pRate) {
        *pRate = rate;
        return BAD_VALUE;
    }

    if (pChannels == 0 || (*pChannels != AudioSystem::CHANNEL_IN_MONO &&
        *pChannels != AudioSystem::CHANNEL_IN_STEREO)) {
        *pChannels = AUDIO_HW_IN_CHANNELS;
        return BAD_VALUE;
    }

    mHardware = hw;

    LOGV("AudioStreamInMSM72xx::set(%d, %d, %u)", *pFormat, *pChannels, *pRate);
    if (mFd >= 0) {
        LOGE("Audio record already open");
        return -EPERM;
    }

    // open audio input device
    status_t status = ::open("/dev/msm_pcm_in", O_RDWR);
    if (status < 0) {
        LOGE("Cannot open /dev/msm_pcm_in errno: %d", errno);
        goto Error;
    }
    mFd = status;

    // configuration
    LOGV("get config");
    struct msm_audio_config config;
    status = ioctl(mFd, AUDIO_GET_CONFIG, &config);
    if (status < 0) {
        LOGE("Cannot read config");
        goto Error;
    }

    LOGV("set config");
    config.channel_count = AudioSystem::popCount(*pChannels);
    config.sample_rate = *pRate;
    config.buffer_size = bufferSize();
    config.buffer_count = 2;
    config.codec_type = CODEC_TYPE_PCM;
    status = ioctl(mFd, AUDIO_SET_CONFIG, &config);
    if (status < 0) {
        LOGE("Cannot set config");
        if (ioctl(mFd, AUDIO_GET_CONFIG, &config) == 0) {
            if (config.channel_count == 1) {
                *pChannels = AudioSystem::CHANNEL_IN_MONO;
            } else {
                *pChannels = AudioSystem::CHANNEL_IN_STEREO;
            }
            *pRate = config.sample_rate;
        }
        goto Error;
    }

    LOGV("confirm config");
    status = ioctl(mFd, AUDIO_GET_CONFIG, &config);
    if (status < 0) {
        LOGE("Cannot read config");
        goto Error;
    }
    LOGV("buffer_size: %u", config.buffer_size);
    LOGV("buffer_count: %u", config.buffer_count);
    LOGV("channel_count: %u", config.channel_count);
    LOGV("sample_rate: %u", config.sample_rate);

    mDevices = devices;
    mFormat = AUDIO_HW_IN_FORMAT;
    mChannels = *pChannels;
    mSampleRate = config.sample_rate;
    mBufferSize = config.buffer_size;

    //mHardware->setMicMute_nosync(false);
    mState = AUDIO_INPUT_OPENED;

    audpre_index = calculate_audpre_table_index(mSampleRate);
    tx_iir_index = (audpre_index * 2) + (hw->checkOutputStandby() ? 0 : 1);
    LOGD("audpre_index = %d, tx_iir_index = %d\n", audpre_index, tx_iir_index);

    /**
     * If audio-preprocessing failed, we should not block record.
     */
    status = msm72xx_set_audpre_params(audpre_index, tx_iir_index);
    if (status < 0)
        LOGE("Cannot set audpre parameters");

    mAcoustics = acoustic_flags;
    status = msm72xx_enable_audpre((int)acoustic_flags, audpre_index, tx_iir_index);
    if (status < 0)
        LOGE("Cannot enable audpre");

    return NO_ERROR;

Error:
    if (mFd >= 0) {
        ::close(mFd);
        mFd = -1;
    }
    return status;
}

AudioHardware::AudioStreamInMSM72xx::~AudioStreamInMSM72xx()
{
    LOGV("AudioStreamInMSM72xx destructor");
    standby();
}

ssize_t AudioHardware::AudioStreamInMSM72xx::read( void* buffer, ssize_t bytes)
{
    LOGV("AudioStreamInMSM72xx::read(%p, %ld)", buffer, bytes);
    if (!mHardware) return -1;

    size_t count = bytes;
    uint8_t* p = static_cast<uint8_t*>(buffer);

    if (mState < AUDIO_INPUT_OPENED) {
        Mutex::Autolock lock(mHardware->mLock);
        if (set(mHardware, mDevices, &mFormat, &mChannels, &mSampleRate, mAcoustics) != NO_ERROR) {
            return -1;
        }
    }

    if (mState < AUDIO_INPUT_STARTED) {
        mState = AUDIO_INPUT_STARTED;
        // force routing to input device
        mHardware->clearCurDevice();
        mHardware->doRouting();
        if (ioctl(mFd, AUDIO_START, 0)) {
            LOGE("Error starting record");
            standby();
            return -1;
        }
    }

    while (count) {
        ssize_t bytesRead = ::read(mFd, buffer, count);
        if (bytesRead >= 0) {
            count -= bytesRead;
            p += bytesRead;
        } else {
            if (errno != EAGAIN) return bytesRead;
            mRetryCount++;
            LOGW("EAGAIN - retrying");
        }
    }
    return bytes;
}

status_t AudioHardware::AudioStreamInMSM72xx::standby()
{
    if (mState > AUDIO_INPUT_CLOSED) {
        if (mFd >= 0) {
            ::close(mFd);
            mFd = -1;
        }
        mState = AUDIO_INPUT_CLOSED;
    }
    if (!mHardware) return -1;
    // restore output routing if necessary
    mHardware->clearCurDevice();
    mHardware->doRouting();
    return NO_ERROR;
}

status_t AudioHardware::AudioStreamInMSM72xx::dump(int fd, const Vector<String16>& args)
{
    const size_t SIZE = 256;
    char buffer[SIZE];
    String8 result;
    result.append("AudioStreamInMSM72xx::dump\n");
    snprintf(buffer, SIZE, "\tsample rate: %d\n", sampleRate());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tbuffer size: %d\n", bufferSize());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tchannels: %d\n", channels());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tformat: %d\n", format());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmHardware: %p\n", mHardware);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmFd count: %d\n", mFd);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmState: %d\n", mState);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmRetryCount: %d\n", mRetryCount);
    result.append(buffer);
    ::write(fd, result.string(), result.size());
    return NO_ERROR;
}

status_t AudioHardware::AudioStreamInMSM72xx::setParameters(const String8& keyValuePairs)
{
    AudioParameter param = AudioParameter(keyValuePairs);
    String8 vr_mode_key = String8("vr_mode");
    String8 key = String8(AudioParameter::keyRouting);
    status_t status = NO_ERROR;
    int device;
    int vr_mode;
    LOGV("AudioStreamInMSM72xx::setParameters() %s", keyValuePairs.string());

    if (param.getInt(vr_mode_key, vr_mode) == NO_ERROR) {
	acoustic_set_vrmode(vr_mode);
        param.remove(key);
    }
    
    if (param.getInt(key, device) == NO_ERROR) {
        LOGV("set input routing %x", device);
        if (device & (device - 1)) {
            status = BAD_VALUE;
        } else {
            mDevices = device;
            status = mHardware->doRouting();
        }
        param.remove(key);
    }

    if (param.size()) {
        status = BAD_VALUE;
    }
    return status;
}

String8 AudioHardware::AudioStreamInMSM72xx::getParameters(const String8& keys)
{
    AudioParameter param = AudioParameter(keys);
    String8 value;
    String8 key = String8(AudioParameter::keyRouting);

    if (param.get(key, value) == NO_ERROR) {
        LOGV("get routing %x", mDevices);
        param.addInt(key, (int)mDevices);
    }

    LOGV("AudioStreamInMSM72xx::getParameters() %s", param.toString().string());
    return param.toString();
}

// ----------------------------------------------------------------------------

extern "C" AudioHardwareInterface* createAudioHardware(void) {
    return new AudioHardware();
}

}; // namespace android
