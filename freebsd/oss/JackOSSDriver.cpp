/*
Copyright (C) 2003-2007 Jussi Laako <jussi@sonarnerd.net>
Copyright (C) 2008 Grame & RTL 2008

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

*/

#include "driver_interface.h"
#include "JackThreadedDriver.h"
#include "JackDriverLoader.h"
#include "JackOSSDriver.h"
#include "JackEngineControl.h"
#include "JackGraphManager.h"
#include "JackError.h"
#include "JackTime.h"
#include "JackShmMem.h"
#include "memops.h"

#include <sys/ioctl.h>
#include <sys/soundcard.h>
#include <fcntl.h>
#include <iostream>
#include <assert.h>
#include <stdio.h>

using namespace std;

namespace
{

int GetSampleFormat(int bits)
{
    switch(bits) {
        // Native-endian signed 32 bit samples.
        case 32:
            return AFMT_S32_NE;
        // Native-endian signed 24 bit (packed) samples.
        case 24:
            return AFMT_S24_NE;
        // Native-endian signed 16 bit samples, used by default.
        case 16:
        default:
            return AFMT_S16_NE;
    }
}

}

void sosso::Log::log(sosso::SourceLocation location, const char* message) {
    jack_log(message);
}

void sosso::Log::info(sosso::SourceLocation location, const char* message) {
    jack_info(message);
}

void sosso::Log::warn(sosso::SourceLocation location, const char* message) {
    jack_error(message);
}

namespace Jack
{

#ifdef JACK_MONITOR

#define CYCLE_POINTS 500000

struct OSSCycle {
    jack_time_t fBeforeRead;
    jack_time_t fAfterRead;
    jack_time_t fAfterReadConvert;
    jack_time_t fBeforeWrite;
    jack_time_t fAfterWrite;
    jack_time_t fBeforeWriteConvert;
};

struct OSSCycleTable {
    jack_time_t fBeforeFirstWrite;
    jack_time_t fAfterFirstWrite;
    OSSCycle fTable[CYCLE_POINTS];
};

OSSCycleTable gCycleTable;
int gCycleCount = 0;

#endif

static inline void CopyAndConvertIn(jack_sample_t *dst, void *src, size_t nframes, int channel, int chcount, int bits)
{
    switch (bits) {

        case 16: {
            signed short *s16src = (signed short*)src;
            s16src += channel;
            sample_move_dS_s16(dst, (char*)s16src, nframes, chcount<<1);
            break;
        }
        case 24: {
            char *s24src = (char*)src;
            s24src += channel * 3;
            sample_move_dS_s24(dst, s24src, nframes, chcount*3);
            break;
        }
        case 32: {
            signed int *s32src = (signed int*)src;
            s32src += channel;
            sample_move_dS_s32u24(dst, (char*)s32src, nframes, chcount<<2);
            break;
        }
    }
}

static inline void CopyAndConvertOut(void *dst, jack_sample_t *src, size_t nframes, int channel, int chcount, int bits)
{
    switch (bits) {

        case 16: {
            signed short *s16dst = (signed short*)dst;
            s16dst += channel;
            sample_move_d16_sS((char*)s16dst, src, nframes, chcount<<1, NULL); // No dithering for now...
            break;
        }
        case 24: {
            char *s24dst = (char*)dst;
            s24dst += channel * 3;
            sample_move_d24_sS(s24dst, src, nframes, chcount*3, NULL);
            break;
        }
        case 32: {
            signed int *s32dst = (signed int*)dst;
            s32dst += channel;
            sample_move_d32u24_sS((char*)s32dst, src, nframes, chcount<<2, NULL);
            break;
        }
    }
}

void JackOSSDriver::DisplayDeviceInfo()
{
    audio_buf_info info;
    memset(&info, 0, sizeof(audio_buf_info));
    int cap = 0;

    // Duplex cards : http://manuals.opensound.com/developer/full_duplex.html
    jack_info("Audio Interface Description :");

    if (fPlayback) {
        int fd = fWriteChannel.file_descriptor();

        jack_info("Sampling Frequency : %d, Sample Size : %d", fWriteChannel.sample_rate(), fWriteChannel.bytes_per_sample() * 8);

        oss_sysinfo si;
        if (ioctl(fd, OSS_SYSINFO, &si) == -1) {
            jack_error("JackOSSDriver::DisplayDeviceInfo OSS_SYSINFO failed : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        } else {
            jack_info("OSS product %s", si.product);
            jack_info("OSS version %s", si.version);
            jack_info("OSS version num %d", si.versionnum);
            jack_info("OSS numaudios %d", si.numaudios);
            jack_info("OSS numaudioengines %d", si.numaudioengines);
            jack_info("OSS numcards %d", si.numcards);
        }

        jack_info("Output capabilities - %d channels : ", fPlaybackChannels);

        if (ioctl(fd, SNDCTL_DSP_GETOSPACE, &info) == -1)  {
            jack_error("JackOSSDriver::DisplayDeviceInfo SNDCTL_DSP_GETOSPACE failed : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        } else {
            jack_info("output space info: fragments = %d, fragstotal = %d, fragsize = %d, bytes = %d",
                info.fragments, info.fragstotal, info.fragsize, info.bytes);
        }

        if (ioctl(fd, SNDCTL_DSP_GETCAPS, &cap) == -1)  {
            jack_error("JackOSSDriver::DisplayDeviceInfo SNDCTL_DSP_GETCAPS failed : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        } else {
            if (cap & DSP_CAP_DUPLEX)   jack_info(" DSP_CAP_DUPLEX");
            if (cap & DSP_CAP_REALTIME) jack_info(" DSP_CAP_REALTIME");
            if (cap & DSP_CAP_BATCH)    jack_info(" DSP_CAP_BATCH");
            if (cap & DSP_CAP_COPROC)   jack_info(" DSP_CAP_COPROC");
            if (cap & DSP_CAP_TRIGGER)  jack_info(" DSP_CAP_TRIGGER");
            if (cap & DSP_CAP_MMAP)     jack_info(" DSP_CAP_MMAP");
            if (cap & DSP_CAP_MULTI)    jack_info(" DSP_CAP_MULTI");
            if (cap & DSP_CAP_BIND)     jack_info(" DSP_CAP_BIND");
        }
    }

    if (fCapture) {
        int fd = fReadChannel.file_descriptor();

        jack_info("Sampling Frequency : %d, Sample Size : %d", fReadChannel.sample_rate(), fReadChannel.bytes_per_sample() * 8);

        oss_sysinfo si;
        if (ioctl(fd, OSS_SYSINFO, &si) == -1) {
            jack_error("JackOSSDriver::DisplayDeviceInfo OSS_SYSINFO failed : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        } else {
            jack_info("OSS product %s", si.product);
            jack_info("OSS version %s", si.version);
            jack_info("OSS version num %d", si.versionnum);
            jack_info("OSS numaudios %d", si.numaudios);
            jack_info("OSS numaudioengines %d", si.numaudioengines);
            jack_info("OSS numcards %d", si.numcards);
        }

        jack_info("Input capabilities - %d channels : ", fCaptureChannels);

        if (ioctl(fd, SNDCTL_DSP_GETISPACE, &info) == -1) {
            jack_error("JackOSSDriver::DisplayDeviceInfo SNDCTL_DSP_GETOSPACE failed : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        } else {
            jack_info("input space info: fragments = %d, fragstotal = %d, fragsize = %d, bytes = %d",
                info.fragments, info.fragstotal, info.fragsize, info.bytes);
        }

        if (ioctl(fd, SNDCTL_DSP_GETCAPS, &cap) == -1) {
            jack_error("JackOSSDriver::DisplayDeviceInfo SNDCTL_DSP_GETCAPS failed : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        } else {
            if (cap & DSP_CAP_DUPLEX)   jack_info(" DSP_CAP_DUPLEX");
            if (cap & DSP_CAP_REALTIME) jack_info(" DSP_CAP_REALTIME");
            if (cap & DSP_CAP_BATCH)    jack_info(" DSP_CAP_BATCH");
            if (cap & DSP_CAP_COPROC)   jack_info(" DSP_CAP_COPROC");
            if (cap & DSP_CAP_TRIGGER)  jack_info(" DSP_CAP_TRIGGER");
            if (cap & DSP_CAP_MMAP)     jack_info(" DSP_CAP_MMAP");
            if (cap & DSP_CAP_MULTI)    jack_info(" DSP_CAP_MULTI");
            if (cap & DSP_CAP_BIND)     jack_info(" DSP_CAP_BIND");
        }
    }
}

int JackOSSDriver::OpenInput()
{
    if (fCaptureChannels == 0) fCaptureChannels = 2;

    if (!fReadChannel.set_parameters(GetSampleFormat(fBits), fEngineControl->fSampleRate, fCaptureChannels)) {
        jack_error("JackOSSDriver::OpenInput unsupported sample format %#x", GetSampleFormat(fBits));
        return -1;
    }

    if (!fReadChannel.open(fCaptureDriverName, fExcl)) {
        return -1;
    }

    jack_log("JackOSSDriver::OpenInput input file descriptor = %d", fReadChannel.file_descriptor());

    if (fReadChannel.channels() != fCaptureChannels) {
        fCaptureChannels = fReadChannel.channels();
        jack_info("JackOSSDriver::OpenInput driver forced the number of capture channels %ld", fCaptureChannels);
    }

    fReadChannel.set_target_latency(0);

    // Internal buffer size required for one period.
    size_t period_bytes = fEngineControl->fBufferSize * fReadChannel.frame_size();

    // Allocate two buffers for double buffering.
    sosso::Buffer buffer((char*) calloc(period_bytes, 1), period_bytes);
    assert(buffer.data());
    fReadChannel.set_buffer(std::move(buffer), 0);
    buffer = sosso::Buffer((char*) calloc(period_bytes, 1), period_bytes);
    assert(buffer.data());
    fReadChannel.set_buffer(std::move(buffer), fEngineControl->fBufferSize);

    fReadChannel.request_sync(2);

    if (fReadChannel.sample_rate() != fEngineControl->fSampleRate) {
        jack_error("JackOSSDriver::OpenInput driver forced sample rate %ld", fReadChannel.sample_rate());
        fReadChannel.close();
        return -1;
    }

    return 0;
}

int JackOSSDriver::OpenOutput()
{
    if (fPlaybackChannels == 0) fPlaybackChannels = 2;

    if (!fWriteChannel.set_parameters(GetSampleFormat(fBits), fEngineControl->fSampleRate, fPlaybackChannels)) {
        jack_error("JackOSSDriver::OpenOutput unsupported sample format %#x", GetSampleFormat(fBits));
        return -1;
    }

    if (!fWriteChannel.open(fPlaybackDriverName, fExcl)) {
        return -1;
    }

    jack_log("JackOSSDriver::OpenOutput output file descriptor = %d", fWriteChannel.file_descriptor());

    if (fWriteChannel.channels() != fPlaybackChannels) {
        fPlaybackChannels = fWriteChannel.channels();
        jack_info("JackOSSDriver::OpenOutput driver forced the number of playback channels %ld", fPlaybackChannels);
    }

    fWriteChannel.set_target_latency(fEngineControl->fBufferSize);

    // Internal buffer size required for one period.
    size_t period_bytes = fEngineControl->fBufferSize * fWriteChannel.frame_size();

    // Allocate two buffers for double buffering.
    sosso::Buffer buffer((char*) calloc(period_bytes, 1), period_bytes);
    assert(buffer.data());
    fWriteChannel.set_buffer(std::move(buffer), 0);
    buffer = sosso::Buffer((char*) calloc(period_bytes, 1), period_bytes);
    assert(buffer.data());
    fWriteChannel.set_buffer(std::move(buffer), fEngineControl->fBufferSize);

    fWriteChannel.request_sync(2);

    if (fWriteChannel.sample_rate() != fEngineControl->fSampleRate) {
        jack_error("JackOSSDriver::OpenOutput driver forced the sample rate %ld", fWriteChannel.sample_rate());
        fWriteChannel.close();
        return -1;
    }

    return 0;
}

int JackOSSDriver::Open(jack_nframes_t nframes,
                        int user_nperiods,
                        jack_nframes_t samplerate,
                        bool capturing,
                        bool playing,
                        int inchannels,
                        int outchannels,
                        bool excl,
                        bool monitor,
                        const char* capture_driver_uid,
                        const char* playback_driver_uid,
                        jack_nframes_t capture_latency,
                        jack_nframes_t playback_latency,
                        int bits,
                        bool ignorehwbuf)
{
    // Store local settings first.
    fCapture = capturing;
    fPlayback = playing;
    fBits = bits;
    fIgnoreHW = ignorehwbuf;
    fNperiods = user_nperiods;
    fExcl = excl;

    // Generic JackAudioDriver Open
    if (JackAudioDriver::Open(nframes, samplerate, capturing, playing, inchannels, outchannels, monitor,
        capture_driver_uid, playback_driver_uid, capture_latency, playback_latency) != 0) {
        return -1;
    } else {

#ifdef JACK_MONITOR
        // Force memory page in
        memset(&gCycleTable, 0, sizeof(gCycleTable));
#endif

        if (OpenAux() < 0) {
            Close();
            return -1;
        } else {
            return 0;
        }
    }
}

int JackOSSDriver::Close()
{
#ifdef JACK_MONITOR
    FILE* file = fopen("OSSProfiling.log", "w");

    if (file) {
        jack_info("Writing OSS driver timing data....");
        for (int i = 1; i < gCycleCount; i++) {
            int d1 = gCycleTable.fTable[i].fAfterRead - gCycleTable.fTable[i].fBeforeRead;
            int d2 = gCycleTable.fTable[i].fAfterReadConvert - gCycleTable.fTable[i].fAfterRead;
            int d3 = gCycleTable.fTable[i].fAfterWrite - gCycleTable.fTable[i].fBeforeWrite;
            int d4 = gCycleTable.fTable[i].fBeforeWrite - gCycleTable.fTable[i].fBeforeWriteConvert;
            fprintf(file, "%d \t %d \t %d \t %d \t \n", d1, d2, d3, d4);
        }
        fclose(file);
    } else {
        jack_error("JackOSSDriver::Close : cannot open OSSProfiling.log file");
    }

    file = fopen("TimingOSS.plot", "w");

    if (file == NULL) {
        jack_error("JackOSSDriver::Close cannot open TimingOSS.plot file");
    } else {

        fprintf(file, "set grid\n");
        fprintf(file, "set title \"OSS audio driver timing\"\n");
        fprintf(file, "set xlabel \"audio cycles\"\n");
        fprintf(file, "set ylabel \"usec\"\n");
        fprintf(file, "plot \"OSSProfiling.log\" using 1 title \"Driver read wait\" with lines, \
                            \"OSSProfiling.log\" using 2 title \"Driver read convert duration\" with lines, \
                            \"OSSProfiling.log\" using 3 title \"Driver write wait\" with lines, \
                            \"OSSProfiling.log\" using 4 title \"Driver write convert duration\" with lines\n");

        fprintf(file, "set output 'TimingOSS.pdf\n");
        fprintf(file, "set terminal pdf\n");

        fprintf(file, "set grid\n");
        fprintf(file, "set title \"OSS audio driver timing\"\n");
        fprintf(file, "set xlabel \"audio cycles\"\n");
        fprintf(file, "set ylabel \"usec\"\n");
        fprintf(file, "plot \"OSSProfiling.log\" using 1 title \"Driver read wait\" with lines, \
                            \"OSSProfiling.log\" using 2 title \"Driver read convert duration\" with lines, \
                            \"OSSProfiling.log\" using 3 title \"Driver write wait\" with lines, \
                            \"OSSProfiling.log\" using 4 title \"Driver write convert duration\" with lines\n");

        fclose(file);
    }
#endif
    int res = JackAudioDriver::Close();
    CloseAux();
    return res;
}


int JackOSSDriver::OpenAux()
{
    // (Re-)Initialize runtime variables.
    fCycleEnd = 0;
    fLastProcessing = 0;
    fMaxJackBlocking = 0;

    int group_id = 0;

    if (fCapture) {
        if ((OpenInput() < 0)) {
            return -1;
        }
        fReadChannel.add_to_sync_group(group_id);
    }

    if (fPlayback) {
        if ((OpenOutput() < 0)) {
            return -1;
        }
        fWriteChannel.add_to_sync_group(group_id);
    }

    // Start both channels in sync if available.
    if (fCapture) {
        fReadChannel.start_sync_group(group_id);
    } else {
        fWriteChannel.start_sync_group(group_id);
    }

    // Init frame clock here to mark start time.
    if (!fFrameClock.init_clock(fEngineControl->fSampleRate)) {
        return -1;
    }

    // TODO: Improve correction limits for border cases.
    std::int64_t limit = fEngineControl->fBufferSize / 2;
    fCorrection.set_loss_limits(-limit, limit);
    limit = limit / 2;
    fCorrection.set_drift_limits(-limit, limit);

    DisplayDeviceInfo();
    return 0;
}

void JackOSSDriver::CloseAux()
{
    if (fCapture && fReadChannel.recording()) {
        free(fReadChannel.take_buffer().data());
        free(fReadChannel.take_buffer().data());
        fReadChannel.close();
    }

    if (fPlayback && fWriteChannel.playback()) {
        free(fWriteChannel.take_buffer().data());
        free(fWriteChannel.take_buffer().data());
        fWriteChannel.close();
    }
}

int JackOSSDriver::CheckTimeAndRun(std::int64_t &now)
{
    // Check current frame time.
    if (!fFrameClock.now(now)) {
        jack_error("JackOSSDriver::CheckTimeAndRun(): Frame clock failed.");
        return -1;
    }
    // Round frame time down to steppings.
    now = now - (now % fReadChannel.stepping());

    // Process read channel if wakeup time passed, or OSS buffer data available.
    if (fCapture && fReadChannel.recording()) {
        if (now >= fReadChannel.wakeup_time(fReadChannel.last_processing())) {
            if (!fReadChannel.process(now)) {
                jack_error("JackOSSDriver::CheckTimeAndRun(): Read process failed.");
                return -1;
            }
        }
    }
    // Process write channel if wakeup time passed, or OSS buffer space available.
    if (fPlayback && fWriteChannel.playback()) {
        if (now >= fWriteChannel.wakeup_time(fWriteChannel.last_processing())) {
            if (!fWriteChannel.process(now)) {
                jack_error("JackOSSDriver::CheckTimeAndRun(): Write process failed.");
                return -1;
            }
        }
    }

    return 0;
}

int JackOSSDriver::Read()
{
#ifdef JACK_MONITOR
    gCycleTable.fTable[gCycleCount].fBeforeRead = GetMicroSeconds();
#endif

    // Mark the end time of this cycle, in frames.
    fCycleEnd += fEngineControl->fBufferSize;

    // Process read and write channels at least once.
    std::int64_t now = 0;
    if (!fFrameClock.now(now)) {
        return -1;
    }
    if (now - fLastProcessing > fMaxJackBlocking) {
        fMaxJackBlocking = now - fLastProcessing;
        jack_info("Max Jack blocking time increased to %lld.", fMaxJackBlocking);
    }
    if (CheckTimeAndRun(now) != 0) {
        return -1;
    }

    // Get start time of current cycle in frames.
    std::int64_t cycle_begin = fCycleEnd - fEngineControl->fBufferSize;
    // Adjust start time to the channel we sync to.
    if (fReadChannel.recording()) {
      cycle_begin += fReadChannel.balance();
    } else {
      cycle_begin += fWriteChannel.balance();
    }
    // If we are late by more than one cycle, drop it and report an XRun.
    if (now > cycle_begin + fEngineControl->fBufferSize) {
        jack_error("JackOSSDriver::Read(): Late by %lld frames.", now - cycle_begin);
        fCycleEnd += (now - cycle_begin);
        // TODO: Check if we can map "now" to absolute time in microsecons.
        NotifyXRun(GetMicroSeconds(), (float)(fFrameClock.frames_to_time(now - cycle_begin) / 1000));
    }

    // Wait and process channels until read, or else write, buffer is finished.
    std::int64_t wakeup = now;
    while ((fReadChannel.recording() && !fReadChannel.finished(now)) ||
           (!fReadChannel.recording() && !fWriteChannel.finished(now))) {
        if (wakeup > now) {
            if (fFrameClock.sleep(wakeup)) {
                now = wakeup;
            }
        } else {
            if (CheckTimeAndRun(now) != 0) {
                return -1;
            }
            wakeup = std::min(fReadChannel.wakeup_time(now), fWriteChannel.wakeup_time(now));
        }
    }

    // Keep begin cycle time
    JackDriver::CycleTakeBeginTime();

    if (!fReadChannel.recording()) {
        return 0;
    }

    fReadChannel.log_state(now);

#ifdef JACK_MONITOR
    gCycleTable.fTable[gCycleCount].fAfterRead = GetMicroSeconds();
#endif

    // Get buffer from read channel.
    sosso::Buffer buffer = fReadChannel.take_buffer();

    for (int i = 0; i < fCaptureChannels; i++) {
        if (fGraphManager->GetConnectionsNum(fCapturePortList[i]) > 0) {
            CopyAndConvertIn(GetInputBuffer(i), buffer.data(), fEngineControl->fBufferSize, i, fCaptureChannels, fReadChannel.bytes_per_sample() * 8);
        }
    }
    buffer.reset();

    fReadChannel.set_buffer(std::move(buffer), fCycleEnd + fEngineControl->fBufferSize);

#ifdef JACK_MONITOR
    gCycleTable.fTable[gCycleCount].fAfterReadConvert = GetMicroSeconds();
#endif

    if (!fFrameClock.now(now)) {
        return -1;
    }
    fLastProcessing = now;

    return CheckTimeAndRun(now);
}

int JackOSSDriver::Write()
{
    if (!fWriteChannel.playback()) {
        return 0;
    }

    // Process read and write channels at least once.
    std::int64_t now = 0;
    if (!fFrameClock.now(now)) {
        return -1;
    }
    if (now - fLastProcessing > fMaxJackBlocking) {
        fMaxJackBlocking = now - fLastProcessing;
        jack_info("Max Jack blocking time increased to %lld.", fMaxJackBlocking);
    }
    if (CheckTimeAndRun(now) != 0) {
        return -1;
    }

    // Wait and process channels until write buffer is finished.
    std::int64_t wakeup = now;
    while (!fWriteChannel.finished(now)) {
        if (wakeup > now) {
            if (fFrameClock.sleep(wakeup)) {
                now = wakeup;
            }
        } else {
            if (CheckTimeAndRun(now) != 0) {
                return -1;
            }
            wakeup = std::min(fReadChannel.wakeup_time(now), fWriteChannel.wakeup_time(now));
        }
    }

    // Keep begin cycle time
    JackDriver::CycleTakeBeginTime();

    fWriteChannel.log_state(now);

#ifdef JACK_MONITOR
    gCycleTable.fTable[gCycleCount].fBeforeWriteConvert = GetMicroSeconds();
#endif

    sosso::Buffer buffer = fWriteChannel.take_buffer();

    memset(buffer.data(), 0, buffer.length());
    buffer.reset();
    for (int i = 0; i < fPlaybackChannels; i++) {
        if (fGraphManager->GetConnectionsNum(fPlaybackPortList[i]) > 0) {
            CopyAndConvertOut(buffer.data(), GetOutputBuffer(i), fEngineControl->fBufferSize, i, fPlaybackChannels, fWriteChannel.bytes_per_sample() * 8);
        }
    }

    // If both channels are used, correct drift relative to recording balance.
    if (fReadChannel.recording()) {
        std::int64_t old_correction = fCorrection.correction();
        fCorrection.correct(fWriteChannel.balance(), fReadChannel.balance());
        if (fCorrection.correction() != old_correction) {
            jack_info("Playback correction changed from %lld to %lld.", old_correction, fCorrection.correction());
        }
    }

    fWriteChannel.set_buffer(std::move(buffer), fCycleEnd + fEngineControl->fBufferSize + fCorrection.correction());

#ifdef JACK_MONITOR
    gCycleTable.fTable[gCycleCount].fBeforeWrite = GetMicroSeconds();
#endif

    if (!fFrameClock.now(now)) {
        return -1;
    }
    fLastProcessing = now;

    // Do a processing step here.
    if (CheckTimeAndRun(now) != 0) {
        return -1;
    }

#ifdef JACK_MONITOR
    gCycleTable.fTable[gCycleCount].fAfterWrite = GetMicroSeconds();
    gCycleCount = (gCycleCount == CYCLE_POINTS - 1) ? gCycleCount: gCycleCount + 1;
#endif

    return 0;
}

void JackOSSDriver::UpdateLatencies()
{
    // Reimplement from JackAudioDriver. Base latency is smaller, and there's
    // additional latency due to OSS playback buffer management.
    jack_latency_range_t input_range;
    jack_latency_range_t output_range;

    for (int i = 0; i < fCaptureChannels; i++) {
        input_range.max = input_range.min = (fEngineControl->fBufferSize / 2) + fCaptureLatency;
        fGraphManager->GetPort(fCapturePortList[i])->SetLatencyRange(JackCaptureLatency, &input_range);
    }

    for (int i = 0; i < fPlaybackChannels; i++) {
        // TODO: Move this half period to capture latency.
        output_range.max = (fEngineControl->fBufferSize / 2) + fPlaybackLatency;
        // Additional latency introduced by the OSS buffer.
        output_range.max += fNperiods * fEngineControl->fBufferSize;
        // Plus one period if in async mode.
        if (!fEngineControl->fSyncMode) {
            output_range.max += fEngineControl->fBufferSize;
        }
        output_range.min = output_range.max;
        fGraphManager->GetPort(fPlaybackPortList[i])->SetLatencyRange(JackPlaybackLatency, &output_range);
    }
}

int JackOSSDriver::SetBufferSize(jack_nframes_t buffer_size)
{
    // Close and reopen device, we have to adjust the OSS buffer management.
    CloseAux();
    JackAudioDriver::SetBufferSize(buffer_size); // Generic change, never fails
    return OpenAux();
}

} // end of namespace

#ifdef __cplusplus
extern "C"
{
#endif

SERVER_EXPORT jack_driver_desc_t* driver_get_descriptor()
{
    jack_driver_desc_t * desc;
    jack_driver_desc_filler_t filler;
    jack_driver_param_value_t value;

    desc = jack_driver_descriptor_construct("oss", JackDriverMaster, "OSS API based audio backend", &filler);

    value.ui = OSS_DRIVER_DEF_FS;
    jack_driver_descriptor_add_parameter(desc, &filler, "rate", 'r', JackDriverParamUInt, &value, NULL, "Sample rate", NULL);

    value.ui = OSS_DRIVER_DEF_BLKSIZE;
    jack_driver_descriptor_add_parameter(desc, &filler, "period", 'p', JackDriverParamUInt, &value, NULL, "Frames per period", NULL);

    value.ui = OSS_DRIVER_DEF_NPERIODS;
    jack_driver_descriptor_add_parameter(desc, &filler, "nperiods", 'n', JackDriverParamUInt, &value, NULL, "Number of periods to prefill output buffer", NULL);

    value.i = OSS_DRIVER_DEF_BITS;
    jack_driver_descriptor_add_parameter(desc, &filler, "wordlength", 'w', JackDriverParamInt, &value, NULL, "Word length", NULL);

    value.ui = OSS_DRIVER_DEF_INS;
    jack_driver_descriptor_add_parameter(desc, &filler, "inchannels", 'i', JackDriverParamUInt, &value, NULL, "Capture channels", NULL);

    value.ui = OSS_DRIVER_DEF_OUTS;
    jack_driver_descriptor_add_parameter(desc, &filler, "outchannels", 'o', JackDriverParamUInt, &value, NULL, "Playback channels", NULL);

    value.i = false;
    jack_driver_descriptor_add_parameter(desc, &filler, "excl", 'e', JackDriverParamBool, &value, NULL, "Exclusive and direct device access", NULL);

    strcpy(value.str, OSS_DRIVER_DEF_DEV);
    jack_driver_descriptor_add_parameter(desc, &filler, "capture", 'C', JackDriverParamString, &value, NULL, "Input device", NULL);
    jack_driver_descriptor_add_parameter(desc, &filler, "playback", 'P', JackDriverParamString, &value, NULL, "Output device", NULL);
    jack_driver_descriptor_add_parameter(desc, &filler, "device", 'd', JackDriverParamString, &value, NULL, "OSS device name", NULL);

    value.i = false;
    jack_driver_descriptor_add_parameter(desc, &filler, "ignorehwbuf", 'b', JackDriverParamBool, &value, NULL, "Ignore hardware period size", NULL);

    value.ui = 0;
    jack_driver_descriptor_add_parameter(desc, &filler, "input-latency", 'I', JackDriverParamUInt, &value, NULL, "Extra input latency", NULL);
    jack_driver_descriptor_add_parameter(desc, &filler, "output-latency", 'O', JackDriverParamUInt, &value, NULL, "Extra output latency", NULL);

    return desc;
}

SERVER_EXPORT Jack::JackDriverClientInterface* driver_initialize(Jack::JackLockedEngine* engine, Jack::JackSynchro* table, const JSList* params)
{
    int bits = OSS_DRIVER_DEF_BITS;
    jack_nframes_t srate = OSS_DRIVER_DEF_FS;
    jack_nframes_t frames_per_interrupt = OSS_DRIVER_DEF_BLKSIZE;
    const char* capture_pcm_name = OSS_DRIVER_DEF_DEV;
    const char* playback_pcm_name = OSS_DRIVER_DEF_DEV;
    bool capture = false;
    bool playback = false;
    int chan_in = 0;
    int chan_out = 0;
    bool monitor = false;
    bool excl = false;
    unsigned int nperiods = OSS_DRIVER_DEF_NPERIODS;
    const JSList *node;
    const jack_driver_param_t *param;
    bool ignorehwbuf = false;
    jack_nframes_t systemic_input_latency = 0;
    jack_nframes_t systemic_output_latency = 0;

    for (node = params; node; node = jack_slist_next(node)) {

        param = (const jack_driver_param_t *)node->data;

        switch (param->character) {

        case 'r':
            srate = param->value.ui;
            break;

        case 'p':
            frames_per_interrupt = (unsigned int)param->value.ui;
            break;

        case 'n':
            nperiods = (unsigned int)param->value.ui;
            break;

        case 'w':
            bits = param->value.i;
            break;

        case 'i':
            chan_in = (int)param->value.ui;
            break;

        case 'o':
            chan_out = (int)param->value.ui;
            break;

        case 'C':
            capture = true;
            if (strcmp(param->value.str, "none") != 0) {
                capture_pcm_name = param->value.str;
            }
            break;

        case 'P':
            playback = true;
            if (strcmp(param->value.str, "none") != 0) {
                playback_pcm_name = param->value.str;
            }
            break;

        case 'd':
            playback_pcm_name = param->value.str;
            capture_pcm_name = param->value.str;
            break;

        case 'b':
            ignorehwbuf = true;
            break;

        case 'e':
            excl = true;
            break;

        case 'I':
            systemic_input_latency = param->value.ui;
            break;

        case 'O':
            systemic_output_latency = param->value.ui;
            break;
        }
    }

    // duplex is the default
    if (!capture && !playback) {
        capture = true;
        playback = true;
    }

    Jack::JackOSSDriver* oss_driver = new Jack::JackOSSDriver("system", "oss", engine, table);
    Jack::JackDriverClientInterface* threaded_driver = new Jack::JackThreadedDriver(oss_driver);

    // Special open for OSS driver...
    if (oss_driver->Open(frames_per_interrupt, nperiods, srate, capture, playback, chan_in, chan_out,
        excl, monitor, capture_pcm_name, playback_pcm_name, systemic_input_latency, systemic_output_latency, bits, ignorehwbuf) == 0) {
        return threaded_driver;
    } else {
        delete threaded_driver; // Delete the decorated driver
        return NULL;
    }
}

#ifdef __cplusplus
}
#endif
