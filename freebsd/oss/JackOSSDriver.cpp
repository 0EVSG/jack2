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

inline jack_time_t frames_to_us(jack_nframes_t frames, jack_nframes_t sample_rate) {
    return ((frames * 1000000ULL) + (sample_rate / 2ULL)) / sample_rate;
}

inline jack_nframes_t round_up(jack_nframes_t frames, jack_nframes_t block) {
    if (block > 0) {
        frames += (block - 1);
        frames -= (frames % block);
    }
    return frames;
}

inline jack_time_t round_down(jack_time_t time, jack_time_t interval) {
    if (interval > 0) {
        time -= (time % interval);
    }
    return time;
}

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

inline int int2pow2(int x)	{ int r = 0; while ((1 << r) < x) r++; return r; }

inline jack_nframes_t us_to_samples(jack_time_t time, jack_nframes_t sample_rate) {
    return ((time * sample_rate) + 500000ULL) / 1000000ULL;
}

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

void JackOSSDriver::SetSampleFormat()
{
    switch (fBits) {

	    case 24:	/* native-endian LSB aligned 24-bits in 32-bits integer */
            fSampleFormat = AFMT_S24_NE;
            fSampleSize = 3;
			break;
		case 32:	/* native-endian 32-bit integer */
            fSampleFormat = AFMT_S32_NE;
            fSampleSize = sizeof(int);
			break;
		case 16:	/* native-endian 16-bit integer */
		default:
            fSampleFormat = AFMT_S16_NE;
            fSampleSize = sizeof(short);
			break;
    }
}

void JackOSSDriver::DisplayDeviceInfo()
{
    audio_buf_info info;
    oss_audioinfo ai_in, ai_out;
    memset(&info, 0, sizeof(audio_buf_info));
    int cap = 0;

    // Duplex cards : http://manuals.opensound.com/developer/full_duplex.html
    jack_info("Audio Interface Description :");
    jack_info("Sampling Frequency : %d, Sample Format : %d", fEngineControl->fSampleRate, fSampleFormat);

    if (fPlayback) {

        oss_sysinfo si;
        if (ioctl(fOutFD, OSS_SYSINFO, &si) == -1) {
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
        jack_info("Output block size = %d", fOutputBufferSize);

        if (ioctl(fOutFD, SNDCTL_DSP_GETOSPACE, &info) == -1)  {
            jack_error("JackOSSDriver::DisplayDeviceInfo SNDCTL_DSP_GETOSPACE failed : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        } else {
            jack_info("output space info: fragments = %d, fragstotal = %d, fragsize = %d, bytes = %d",
                info.fragments, info.fragstotal, info.fragsize, info.bytes);
        }

        if (ioctl(fOutFD, SNDCTL_DSP_GETCAPS, &cap) == -1)  {
            jack_error("JackOSSDriver::DisplayDeviceInfo SNDCTL_DSP_GETCAPS failed : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        } else {
            if (cap & DSP_CAP_DUPLEX) 	jack_info(" DSP_CAP_DUPLEX");
            if (cap & DSP_CAP_REALTIME) jack_info(" DSP_CAP_REALTIME");
            if (cap & DSP_CAP_BATCH) 	jack_info(" DSP_CAP_BATCH");
            if (cap & DSP_CAP_COPROC) 	jack_info(" DSP_CAP_COPROC");
            if (cap & DSP_CAP_TRIGGER)  jack_info(" DSP_CAP_TRIGGER");
            if (cap & DSP_CAP_MMAP) 	jack_info(" DSP_CAP_MMAP");
            if (cap & DSP_CAP_MULTI) 	jack_info(" DSP_CAP_MULTI");
            if (cap & DSP_CAP_BIND) 	jack_info(" DSP_CAP_BIND");
        }
    }

    if (fCapture) {

      	oss_sysinfo si;
        if (ioctl(fInFD, OSS_SYSINFO, &si) == -1) {
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
        jack_info("Input block size = %d", fInputBufferSize);

        if (ioctl(fInFD, SNDCTL_DSP_GETISPACE, &info) == -1) {
            jack_error("JackOSSDriver::DisplayDeviceInfo SNDCTL_DSP_GETOSPACE failed : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        } else {
            jack_info("input space info: fragments = %d, fragstotal = %d, fragsize = %d, bytes = %d",
                info.fragments, info.fragstotal, info.fragsize, info.bytes);
        }

        if (ioctl(fInFD, SNDCTL_DSP_GETCAPS, &cap) == -1) {
            jack_error("JackOSSDriver::DisplayDeviceInfo SNDCTL_DSP_GETCAPS failed : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        } else {
            if (cap & DSP_CAP_DUPLEX) 	jack_info(" DSP_CAP_DUPLEX");
            if (cap & DSP_CAP_REALTIME) jack_info(" DSP_CAP_REALTIME");
            if (cap & DSP_CAP_BATCH) 	jack_info(" DSP_CAP_BATCH");
            if (cap & DSP_CAP_COPROC) 	jack_info(" DSP_CAP_COPROC");
            if (cap & DSP_CAP_TRIGGER)  jack_info(" DSP_CAP_TRIGGER");
            if (cap & DSP_CAP_MMAP) 	jack_info(" DSP_CAP_MMAP");
            if (cap & DSP_CAP_MULTI) 	jack_info(" DSP_CAP_MULTI");
            if (cap & DSP_CAP_BIND) 	jack_info(" DSP_CAP_BIND");
        }
    }

    if (ai_in.rate_source != ai_out.rate_source) {
        jack_info("Warning : input and output are not necessarily driven by the same clock!");
    }
}

int JackOSSDriver::ProbeInBlockSize()
{
    if (fInFD > 0) {
        // Read one frame into a new hardware block so we can check its size.
        // Repeat that for multiple probes, sometimes the first reads differ.
        ssize_t bytes = 1 * fSampleSize * fCaptureChannels;
        jack_nframes_t probes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        for (int p = 0; p < 8 && bytes > 0; ++p) {
            ssize_t count = ::read(fInFD, fInputBuffer, bytes);
            if (count < 0) {
                // Read error - try again.
                continue;
            }
            bytes -= count;
            if (bytes == 0) {
                // Successfully read one frame into a new hardware block.
                oss_count_t ptr;
                if (ioctl(fInFD, SNDCTL_DSP_CURRENT_IPTR, &ptr) == 0 && ptr.fifo_samples > 0) {
                    probes[p] = 1U + ptr.fifo_samples;
                    if (probes[p] <= fEngineControl->fBufferSize) {
                        // Proceed by reading one frame into the next hardware block.
                        bytes = probes[p] * fSampleSize * fCaptureChannels;
                    } else {
                        // Hardware block size is more than a period, irregular cycle timing.
                        //! \todo Issue a warning here instead of info.
                        jack_info("JackOSSDriver::ProbeInBlockSize hardware block size %d > period", probes[p]);
                    }
                }
            }
        }

        // Examine probes of hardware block size.
        fOSSMaxBlock = 1;
        for (int p = 0; p < 8; ++p) {
            jack_info("JackOSSDriver::ProbeInBlockSize hardware block of %d", probes[p]);
            if (probes[p] > fOSSMaxBlock) {
                fOSSMaxBlock = probes[p];
            }
        }

        // Assume regular hardware block size if the last four probes are the same.
        fInBlockSize = 1;
        if (probes[7] > 0) {
            if (probes[4] == probes[5] && probes[5] == probes[6] && probes[6] == probes[7]) {
                fInBlockSize = probes[7];
            }
        }

        jack_info("JackOSSDriver::ProbeInBlockSize hardware block size %d", fInBlockSize);
        jack_info("JackOSSDriver::ProbeInBlockSize max block size %d", fOSSMaxBlock);
        if (fInBlockSize > fEngineControl->fBufferSize / 2) {
            jack_info("JackOSSDriver::ProbeInBlockSize less than two hardware blocks per cycle");
            jack_info("JackOSSDriver::ProbeInBlockSize for best results make period a multiple of %d", fInBlockSize);
        }

        // Stop recording to reset the recording buffer.
        int trigger = 0;
        ioctl(fInFD, SNDCTL_DSP_SETTRIGGER, &trigger);
        return 0;
    }

    // Default values in case of an error.
    fOSSMaxBlock = fEngineControl->fBufferSize;
    fInBlockSize = 1;
    return -1;
}

int JackOSSDriver::ProbeOutBlockSize()
{
    if (fOutFD) {
        int ret = 0;
        jack_nframes_t mark = (fNperiods * fEngineControl->fBufferSize) + 1;
        WriteSilence(mark);
        jack_nframes_t probes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        for (int p = 0; p < 8 && ret >= 0; ++p) {
            pollfd poll_fd;
            poll_fd.fd = fOutFD;
            poll_fd.events = POLLOUT;
            ret = poll(&poll_fd, 1, 500);
            if (ret <= 0) {
                jack_error("JackOSSDriver::ProbeOutBlockSize poll failed with %d", ret);
            }
            if (poll_fd.revents & POLLOUT) {
                oss_count_t ptr;
                if (ioctl(fOutFD, SNDCTL_DSP_CURRENT_OPTR, &ptr) != -1 && ptr.fifo_samples >= 0) {
                    probes[p] = mark - ptr.fifo_samples;
                    WriteSilence(probes[p]);
                }
                poll_fd.revents = 0;
            }
        }

        // Examine probes of hardware block size.
        jack_nframes_t fOutMaxBlock = 1;
        fOutBlockSize = probes[3];
        for (int p = 4; p < 8; ++p) {
            jack_info("JackOSSDriver::ProbeOutBlockSize hardware block of %d", probes[p]);
            if (probes[p] > fOutMaxBlock) {
                fOutMaxBlock = probes[p];
            }
            if (probes[p] != fOutBlockSize) {
                fOutBlockSize = 1;
            }
        }

        jack_info("JackOSSDriver::ProbeOutBlockSize hardware block size %d", fOutBlockSize);
        jack_info("JackOSSDriver::ProbeOutBlockSize max block size %d", fOutMaxBlock);
        if (fOutBlockSize > fEngineControl->fBufferSize / 2) {
            jack_info("JackOSSDriver::ProbeOutBlockSize less than two hardware blocks per cycle");
            jack_info("JackOSSDriver::ProbeOutBlockSize for best results make period a multiple of %d", fOutBlockSize);
        }

        // Stop recording to reset the recording buffer.
        int trigger = 0;
        ioctl(fOutFD, SNDCTL_DSP_SETTRIGGER, &trigger);
        return 0;
    }

    fOutBlockSize = 1;
    return -1;
}

int JackOSSDriver::WriteSilence(jack_nframes_t frames)
{
    if (fOutFD < 0) {
        return -1;
    }

    // Prefill OSS playback buffer, write some periods of silence.
    memset(fOutputBuffer, 0, fOutputBufferSize);
    unsigned int size = frames * fSampleSize * fPlaybackChannels;
    while (size > 0) {
        ssize_t chunk = (size > fOutputBufferSize) ? fOutputBufferSize : size;
        ssize_t count = ::write(fOutFD, fOutputBuffer, chunk);
        if (count <= 0) {
            jack_error("JackOSSDriver::WriteSilence error bytes written = %ld", count);
            return -1;
        }
        fOSSWriteOffset += (count / (fSampleSize * fPlaybackChannels));
        size -= count;
    }
    return 0;
}

int JackOSSDriver::WaitAndSync()
{
    if (fInFD > 0 && fOSSReadSync != 0) {
        if (fOSSReadOffset + fEngineControl->fBufferSize > 0) {
            jack_nframes_t frames = fOSSReadOffset + fEngineControl->fBufferSize;
            jack_nframes_t rounded = round_up(frames, fInBlockSize);
            fOSSReadSync += frames_to_us(rounded, fEngineControl->fSampleRate);
            fOSSReadOffset -= rounded;
        }
    }
    if (fOutFD > 0 && fOSSWriteSync != 0) {
        if (fOSSWriteOffset > fNperiods * fEngineControl->fBufferSize) {
            jack_nframes_t frames = fOSSWriteOffset - fNperiods * fEngineControl->fBufferSize;
            jack_nframes_t rounded = round_up(frames, fOutBlockSize);
            fOSSWriteSync += frames_to_us(rounded, fEngineControl->fSampleRate);
            fOSSWriteOffset -= rounded;
        }
    }
    jack_time_t poll_start = GetMicroSeconds();
    // Poll until recording and playback buffer are ready for this cycle.
    pollfd poll_fd[2];
    poll_fd[0].fd = fInFD;
    if (fInFD > 0 && poll_start < fOSSReadSync) {
        poll_fd[0].events = POLLIN;
    } else {
        poll_fd[0].events = 0;
    }
    poll_fd[1].fd = fOutFD;
    if (fOutFD > 0 && fOSSWriteSync != 0 && poll_start < fOSSWriteSync) {
        poll_fd[1].events = POLLOUT;
    } else {
        poll_fd[1].events = 0;
    }
    while (poll_fd[0].events != 0 || poll_fd[1].events != 0) {
        poll_fd[0].revents = 0;
        poll_fd[1].revents = 0;
        int ret = poll(poll_fd, 2, 500);
        jack_time_t now = GetMicroSeconds();
        if (ret <= 0) {
            jack_error("JackOSSDriver::WaitAndSync poll failed with %d after %ld us", ret, now - poll_start);
            return ret;
        }
        if (poll_fd[0].revents & POLLIN) {
            // Check the excess recording frames.
            oss_count_t ptr;
            if (ioctl(fInFD, SNDCTL_DSP_CURRENT_IPTR, &ptr) != -1 && ptr.fifo_samples >= 0) {
                if (fInBlockSize <= 1) {
                    // Irregular block size, let sync time converge slowly when late.
                    fOSSReadSync = min(fOSSReadSync, now) / 2 + now / 2;
                    fOSSReadOffset = -ptr.fifo_samples;
                } else if (ptr.fifo_samples - fEngineControl->fBufferSize >= fInBlockSize) {
                    // Too late for a reliable sync, discard.
                } else {
                    // Warn if expected offset differs by more than 48 samples.
                    long long off_diff = fOSSReadOffset + ptr.fifo_samples;
                    if (off_diff < -48 || off_diff > 48) {
                        jack_info("JackOSSDriver::WaitAndSync read fifo off_diff=%ld", off_diff);
                    }
                    // Adapt expected sync time when early or late - in whole block intervals.
                    // Account for some speed drift, but otherwise round down to earlier interval.
                    jack_time_t interval = frames_to_us(fInBlockSize, fEngineControl->fSampleRate);
                    jack_time_t remainder = fOSSReadSync % interval;
                    jack_time_t max_drift = interval / 4;
                    jack_time_t rounded = round_down((now - remainder) + max_drift, interval) + remainder;
                    //! \todo Streamline debug output.
                    if (rounded < fOSSReadSync) {
                        jack_info("JackOSSDriver::WaitAndSync capture sync early by %ld us, round to %ld us", fOSSReadSync - now, fOSSReadSync - rounded);
                    } else if (rounded > fOSSReadSync) {
                        jack_info("JackOSSDriver::WaitAndSync capture sync late by %ld us, round to %ld us", now - fOSSReadSync, rounded - fOSSReadSync);
                    }
                    // Let sync time converge slowly when late, prefer earlier sync times.
                    fOSSReadSync = min(rounded, now) / 2 + now / 2;
                    fOSSReadOffset = -ptr.fifo_samples;
                }
            }
            poll_fd[0].events = 0;
        }
        if (poll_fd[1].revents & POLLOUT) {
            // Check the remaining playback frames.
            oss_count_t ptr;
            if (ioctl(fOutFD, SNDCTL_DSP_CURRENT_OPTR, &ptr) != -1 && ptr.fifo_samples >= 0) {
                if (fOutBlockSize <= 1) {
                    // Irregular block size, let sync time converge slowly when late.
                    fOSSWriteSync = min(fOSSWriteSync, now) / 2 + now / 2;
                    fOSSWriteOffset = ptr.fifo_samples;
                } else if (ptr.fifo_samples + fOutBlockSize <= fNperiods * fEngineControl->fBufferSize) {
                    // Too late for a reliable sync, discard.
                } else {
                    // Warn if expected offset differs by more than 48 samples.
                    long long off_diff = fOSSWriteOffset - ptr.fifo_samples;
                    if (off_diff < -48 || off_diff > 48) {
                        jack_info("JackOSSDriver::WaitAndSync write fifo off_diff=%ld", off_diff);
                    }
                    // Adapt expected sync time when early or late - in whole block intervals.
                    // Account for some speed drift, but otherwise round down to earlier interval.
                    jack_time_t interval = frames_to_us(fOutBlockSize, fEngineControl->fSampleRate);
                    jack_time_t remainder = fOSSWriteSync % interval;
                    jack_time_t max_drift = interval / 4;
                    jack_time_t rounded = round_down((now - remainder) + max_drift, interval) + remainder;
                    //! \todo Streamline debug output.
                    if (rounded < fOSSWriteSync) {
                        jack_info("JackOSSDriver::WaitAndSync playback sync early by %ld us, round to %ld us", fOSSWriteSync - now, fOSSWriteSync - rounded);
                    } else if (rounded > fOSSWriteSync) {
                        jack_info("JackOSSDriver::WaitAndSync playback sync late by %ld us, round to %ld us", now - fOSSWriteSync, rounded - fOSSWriteSync);
                    }
                    // Let sync time converge slowly when late, prefer earlier sync times.
                    fOSSWriteSync = min(rounded, now) / 2 + now / 2;
                    fOSSWriteOffset = ptr.fifo_samples;
                }
            }
            poll_fd[1].events = 0;
        }
    }

    // Compute balance of read and write buffers combined.
    fBufferBalance = 0;
    if (fInFD > 0 && fOutFD > 0) {
        if (fOSSReadSync > fOSSWriteSync) {
            long long fill = us_to_samples(fOSSReadSync - fOSSWriteSync, fEngineControl->fSampleRate);
            fBufferBalance += fill;
        }
        if (fOSSWriteSync > fOSSReadSync) {
            long long omit = us_to_samples(fOSSWriteSync - fOSSReadSync, fEngineControl->fSampleRate);
            fBufferBalance -= omit;
        }
        fBufferBalance -= (fOSSWriteOffset - fOSSReadOffset);
        fBufferBalance += ((1 + fNperiods) * fEngineControl->fBufferSize);

        // Force balancing if sync times deviate too much.
        jack_time_t slack = frames_to_us((fEngineControl->fBufferSize * 2) / 3, fEngineControl->fSampleRate);
        fForceBalancing = fForceBalancing || (fOSSReadSync > fOSSWriteSync + slack);
        fForceBalancing = fForceBalancing || (fOSSWriteSync > fOSSReadSync + slack);
        // Force balancing if buffer is badly balanced.
        fForceBalancing = fForceBalancing || (abs(fBufferBalance) > (fOSSMaxBlock * 3) / 2);
    }
    return 0;
}

int JackOSSDriver::OpenInput()
{
    int flags = 0;
    int gFragFormat;
    int cur_capture_channels;
    int cur_sample_format;
    jack_nframes_t cur_sample_rate;
    audio_buf_info info;

    if (fCaptureChannels == 0) fCaptureChannels = 2;

    if ((fInFD = open(fCaptureDriverName, O_RDONLY | ((fExcl) ? O_EXCL : 0))) < 0) {
        jack_error("JackOSSDriver::OpenInput failed to open device : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        return -1;
    }

    jack_log("JackOSSDriver::OpenInput input fInFD = %d", fInFD);

    if (fExcl) {
        if (ioctl(fInFD, SNDCTL_DSP_COOKEDMODE, &flags) == -1) {
            jack_error("JackOSSDriver::OpenInput failed to set cooked mode : %s@%i, errno = %d", __FILE__, __LINE__, errno);
            goto error;
        }
    }

    cur_sample_format = fSampleFormat;
    if (ioctl(fInFD, SNDCTL_DSP_SETFMT, &fSampleFormat) == -1) {
        jack_error("JackOSSDriver::OpenInput failed to set format : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        goto error;
    }
    if (cur_sample_format != fSampleFormat) {
        //! \todo Actually change sample format and size.
        jack_info("JackOSSDriver::OpenInput driver forced the sample format %ld", fSampleFormat);
    }

    cur_capture_channels = fCaptureChannels;
    if (ioctl(fInFD, SNDCTL_DSP_CHANNELS, &fCaptureChannels) == -1) {
        jack_error("JackOSSDriver::OpenInput failed to set channels : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        goto error;
    }
    if (cur_capture_channels != fCaptureChannels) {
        jack_info("JackOSSDriver::OpenInput driver forced the number of capture channels %ld", fCaptureChannels);
    }

    cur_sample_rate = fEngineControl->fSampleRate;
    if (ioctl(fInFD, SNDCTL_DSP_SPEED, &fEngineControl->fSampleRate) == -1) {
        jack_error("JackOSSDriver::OpenInput failed to set sample rate : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        goto error;
    }
    if (cur_sample_rate != fEngineControl->fSampleRate) {
        jack_info("JackOSSDriver::OpenInput driver forced the sample rate %ld", fEngineControl->fSampleRate);
    }

    // Internal buffer size required for one period.
    fInputBufferSize = fEngineControl->fBufferSize * fSampleSize * fCaptureChannels;

    // Get the total size of the OSS recording buffer, in sample frames.
    info = {0, 0, 0, 0};
    if (ioctl(fInFD, SNDCTL_DSP_GETISPACE, &info) == -1 || info.fragsize <= 0 || info.fragstotal <= 0) {
        jack_error("JackOSSDriver::OpenInput failed to get buffer info : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        goto error;
    }
    fOSSInBuffer = info.fragstotal * info.fragsize / (fSampleSize * fCaptureChannels);

    if (fOSSInBuffer < fEngineControl->fBufferSize * (1 + fNperiods)) {
        // Total size of the OSS recording buffer is too small, resize it.
        unsigned int buf_size = fInputBufferSize * (1 + fNperiods);
        // Keep current fragment size if possible - respect OSS latency settings.
        gFragFormat = int2pow2(info.fragsize);
        unsigned int frag_size = 1U << gFragFormat;
        gFragFormat |= ((buf_size + frag_size - 1) / frag_size) << 16;
        jack_info("JackOSSDriver::OpenInput request %d fragments of %d", (gFragFormat >> 16), frag_size);
        if (ioctl(fInFD, SNDCTL_DSP_SETFRAGMENT, &gFragFormat) == -1) {
            jack_error("JackOSSDriver::OpenInput failed to set fragments : %s@%i, errno = %d", __FILE__, __LINE__, errno);
            goto error;
        }
        // Check the new OSS recording buffer size.
        info = {0, 0, 0, 0};
        if (ioctl(fInFD, SNDCTL_DSP_GETISPACE, &info) == -1 || info.fragsize <= 0 || info.fragstotal <= 0) {
            jack_error("JackOSSDriver::OpenInput failed to get buffer info : %s@%i, errno = %d", __FILE__, __LINE__, errno);
            goto error;
        }
        fOSSInBuffer = info.fragstotal * info.fragsize / (fSampleSize * fCaptureChannels);
    }

    if (fOSSInBuffer > fEngineControl->fBufferSize) {
        int mark = fInputBufferSize;
        if (ioctl(fInFD, SNDCTL_DSP_LOW_WATER, &mark) != 0) {
            jack_error("JackOSSDriver::OpenInput failed to set low water mark : %s@%i, errno = %d", __FILE__, __LINE__, errno);
            goto error;
        }
        jack_info("JackOSSDriver::OpenInput set low water mark to %d", mark);
    }

    fInputBuffer = (void*)calloc(fInputBufferSize, 1);
    assert(fInputBuffer);

    if (ProbeInBlockSize() < 0) {
      goto error;
    }

    return 0;

error:
    ::close(fInFD);
    return -1;
}

int JackOSSDriver::OpenOutput()
{
    int flags = 0;
    int gFragFormat;
    int cur_sample_format;
    int cur_playback_channels;
    jack_nframes_t cur_sample_rate;
    audio_buf_info info;

    if (fPlaybackChannels == 0) fPlaybackChannels = 2;

    if ((fOutFD = open(fPlaybackDriverName, O_WRONLY | ((fExcl) ? O_EXCL : 0))) < 0) {
       jack_error("JackOSSDriver::OpenOutput failed to open device : %s@%i, errno = %d", __FILE__, __LINE__, errno);
       return -1;
    }

    jack_log("JackOSSDriver::OpenOutput output fOutFD = %d", fOutFD);

    if (fExcl) {
        if (ioctl(fOutFD, SNDCTL_DSP_COOKEDMODE, &flags) == -1) {
            jack_error("JackOSSDriver::OpenOutput failed to set cooked mode : %s@%i, errno = %d", __FILE__, __LINE__, errno);
            goto error;
        }
    }

    cur_sample_format = fSampleFormat;
    if (ioctl(fOutFD, SNDCTL_DSP_SETFMT, &fSampleFormat) == -1) {
        jack_error("JackOSSDriver::OpenOutput failed to set format : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        goto error;
    }
    if (cur_sample_format != fSampleFormat) {
        //! \todo Actually change sample format and size.
        jack_info("JackOSSDriver::OpenOutput driver forced the sample format %ld", fSampleFormat);
    }

    cur_playback_channels = fPlaybackChannels;
    if (ioctl(fOutFD, SNDCTL_DSP_CHANNELS, &fPlaybackChannels) == -1) {
        jack_error("JackOSSDriver::OpenOutput failed to set channels : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        goto error;
    }
    if (cur_playback_channels != fPlaybackChannels) {
        jack_info("JackOSSDriver::OpenOutput driver forced the number of playback channels %ld", fPlaybackChannels);
    }

    cur_sample_rate = fEngineControl->fSampleRate;
    if (ioctl(fOutFD, SNDCTL_DSP_SPEED, &fEngineControl->fSampleRate) == -1) {
        jack_error("JackOSSDriver::OpenOutput failed to set sample rate : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        goto error;
    }
    if (cur_sample_rate != fEngineControl->fSampleRate) {
        jack_info("JackOSSDriver::OpenInput driver forced the sample rate %ld", fEngineControl->fSampleRate);
    }

    // Internal buffer size required for one period.
    fOutputBufferSize = fEngineControl->fBufferSize * fSampleSize * fPlaybackChannels;

    // Get the total size of the OSS playback buffer, in sample frames.
    info = {0, 0, 0, 0};
    if (ioctl(fOutFD, SNDCTL_DSP_GETOSPACE, &info) == -1 || info.fragsize <= 0 || info.fragstotal <= 0) {
        jack_error("JackOSSDriver::OpenOutput failed to get buffer info : %s@%i, errno = %d", __FILE__, __LINE__, errno);
        goto error;
    }
    fOSSOutBuffer = info.fragstotal * info.fragsize / (fSampleSize * fPlaybackChannels);

    if (fOSSOutBuffer < fEngineControl->fBufferSize * (1 + fNperiods)) {
        // Total size of the OSS playback buffer is too small, resize it.
        unsigned int buf_size = fOutputBufferSize * (1 + fNperiods);
        // Keep current fragment size if possible - respect OSS latency settings.
        // Some sound cards like Intel HDA may stutter when changing the fragment size.
        gFragFormat = int2pow2(info.fragsize);
        unsigned int frag_size = 1U << gFragFormat;
        gFragFormat |= ((buf_size + frag_size - 1) / frag_size) << 16;
        jack_info("JackOSSDriver::OpenOutput request %d fragments of %d", (gFragFormat >> 16), frag_size);
        if (ioctl(fOutFD, SNDCTL_DSP_SETFRAGMENT, &gFragFormat) == -1) {
            jack_error("JackOSSDriver::OpenOutput failed to set fragments : %s@%i, errno = %d", __FILE__, __LINE__, errno);
            goto error;
        }
        // Check the new OSS playback buffer size.
        info = {0, 0, 0, 0};
        if (ioctl(fOutFD, SNDCTL_DSP_GETOSPACE, &info) == -1 || info.fragsize <= 0 || info.fragstotal <= 0) {
            jack_error("JackOSSDriver::OpenOutput failed to get buffer info : %s@%i, errno = %d", __FILE__, __LINE__, errno);
            goto error;
        }
        fOSSOutBuffer = info.fragstotal * info.fragsize / (fSampleSize * fPlaybackChannels);
    }

    if (fOSSOutBuffer > fEngineControl->fBufferSize * fNperiods) {
        jack_nframes_t low = fOSSOutBuffer - (fNperiods * fEngineControl->fBufferSize);
        int mark = low * fSampleSize * fPlaybackChannels;
        if (ioctl(fOutFD, SNDCTL_DSP_LOW_WATER, &mark) != 0) {
            jack_error("JackOSSDriver::OpenOutput failed to set low water mark : %s@%i, errno = %d", __FILE__, __LINE__, errno);
            goto error;
        }
        jack_info("JackOSSDriver::OpenOutput set low water mark to %d", mark);
    }

    fOutputBuffer = (void*)calloc(fOutputBufferSize, 1);
    assert(fOutputBuffer);

    if (ProbeOutBlockSize() < 0) {
      goto error;
    }

    return 0;

error:
    ::close(fOutFD);
    return -1;
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
    //! \todo Test latencies in asynchronous mode.
    // Additional playback latency as requested by the user.
    // This way the remaining latency should be symmetric as reported by jack_iodelay.
    playback_latency += user_nperiods * nframes;
    // Generic JackAudioDriver Open
    if (JackAudioDriver::Open(nframes, samplerate, capturing, playing, inchannels, outchannels, monitor,
        capture_driver_uid, playback_driver_uid, capture_latency, playback_latency) != 0) {
        return -1;
    } else {

        fCapture = capturing;
        fPlayback = playing;
        fBits = bits;
        fIgnoreHW = ignorehwbuf;
        fNperiods = user_nperiods;
        fExcl = excl;

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
    SetSampleFormat();

    if (fCapture && (OpenInput() < 0)) {
        return -1;
    }

    if (fPlayback && (OpenOutput() < 0)) {
        return -1;
    }

    DisplayDeviceInfo();
    return 0;
}

void JackOSSDriver::CloseAux()
{
    if (fCapture && fInFD > 0) {
        close(fInFD);
        fInFD = -1;
    }

    if (fPlayback && fOutFD > 0) {
        close(fOutFD);
        fOutFD = -1;
    }

    if (fInputBuffer)
        free(fInputBuffer);
    fInputBuffer = NULL;

    if (fOutputBuffer)
        free(fOutputBuffer);
    fOutputBuffer = NULL;
}

int JackOSSDriver::Read()
{
    if (fInFD > 0 && fOSSReadSync == 0) {
        // Start recording again for the following read.
        int trigger = PCM_ENABLE_INPUT;
        ioctl(fInFD, SNDCTL_DSP_SETTRIGGER, &trigger);
        fOSSReadSync = GetMicroSeconds();
        fOSSReadOffset = 0;
        if (fInBlockSize > 1) {
            ssize_t discard = (fInBlockSize / 2) * fSampleSize * fCaptureChannels;
            discard = ::read(fInFD, fInputBuffer, discard);
            if (discard > 0) {
                fOSSReadOffset += discard / (fSampleSize * fCaptureChannels);
            }
        }
        // In duplex mode, start playback right after recording with some silence.
        // Consistently results in a total of (1 + nperiod) * period frames in flight.
        if (fOutFD > 0) {
            fOSSWriteSync = GetMicroSeconds();
            fOSSWriteOffset = 0;
            jack_nframes_t silence = (fNperiods + 1) * fEngineControl->fBufferSize;
            if (fOutBlockSize > 1) {
                silence -= (fOutBlockSize / 2);
            }
            WriteSilence(silence);

            fForceBalancing = true;
        }
    }

#ifdef JACK_MONITOR
    gCycleTable.fTable[gCycleCount].fBeforeRead = GetMicroSeconds();
#endif

    if (WaitAndSync() < 0) {
        return -1;
    }

    // Keep begin cycle time
    JackDriver::CycleTakeBeginTime();

    if (fInFD < 0) {
        return 0;
    }

    ssize_t count;

    audio_errinfo ei_in;
    count = ::read(fInFD, fInputBuffer, fInputBufferSize);

    if (count > 0) {
        jack_time_t now = GetMicroSeconds();
        long long passed = us_to_samples(now - fOSSReadSync, fEngineControl->fSampleRate);
        passed -= (passed % fInBlockSize);
        if (passed > fOSSReadOffset + fOSSInBuffer) {
            // Overrun, adjust read and write position.
            long long missed = passed - (fOSSReadOffset + fOSSInBuffer);
            jack_error("JackOSSDriver::Read missed %d frames by overrun", missed);
            fOSSReadOffset += missed;
            fOSSWriteOffset += missed;
        }
        fOSSReadOffset += count / (fSampleSize * fCaptureChannels);
    }

    static unsigned int sample_count = 0;
    if (count > 0) {
        sample_count += count / (fSampleSize * fCaptureChannels);
    }
    static unsigned int cycle_count = 0;
    if (++cycle_count % 1000 == 0) {
        jack_info("JackOSSDriver::Read buffer balance is %ld", fBufferBalance);
        oss_count_t ptr;
        if (ioctl(fInFD, SNDCTL_DSP_CURRENT_IPTR, &ptr) != -1) {
            jack_info("JackOSSDriver::Read recording samples = %ld, fifo_samples = %d", ptr.samples, ptr.fifo_samples);
        }
        if (fOutFD > 0 && ioctl(fOutFD, SNDCTL_DSP_CURRENT_OPTR, &ptr) != -1) {
            jack_info("JackOSSDriver::Read playback samples = %ld, fifo_samples = %d", ptr.samples, ptr.fifo_samples);
        }
        jack_time_t now = GetMicroSeconds();
        jack_info("JackOSSDriver::Read recording offset %ld sync %ld ago", fOSSReadOffset, now - fOSSReadSync);
        jack_info("JackOSSDriver::Read playback offset %ld sync %ld ago", fOSSWriteOffset, now - fOSSWriteSync);
        jack_info("JackOSSDriver::Read total recorded samples = %ld", sample_count);
    }

#ifdef JACK_MONITOR
    if (count > 0 && count != (int)fInputBufferSize)
        jack_log("JackOSSDriver::Read count = %ld", count / (fSampleSize * fCaptureChannels));
    gCycleTable.fTable[gCycleCount].fAfterRead = GetMicroSeconds();
#endif

    // XRun detection
    if (ioctl(fInFD, SNDCTL_DSP_GETERROR, &ei_in) == 0) {

        if (ei_in.rec_overruns > 0 ) {
            jack_error("JackOSSDriver::Read overruns = %d", ei_in.rec_overruns);
            jack_time_t cur_time = GetMicroSeconds();
            //! \todo Improve overrun notification with real time info.
            NotifyXRun(cur_time, float(cur_time - fBeginDateUst));   // Better this value than nothing...
        }

        if (ei_in.rec_errorcount > 0 && ei_in.rec_lasterror != 0) {
            jack_error("%d OSS rec event(s), last=%05d:%d", ei_in.rec_errorcount, ei_in.rec_lasterror, ei_in.rec_errorparm);
        }
    }

    if (count < 0) {
        jack_log("JackOSSDriver::Read error = %s", strerror(errno));
        return -1;
    } else if (count < (int)fInputBufferSize) {
        //! \todo Try multiple times?
        jack_error("JackOSSDriver::Read error bytes read = %ld", count);
        return -1;
    } else {

        for (int i = 0; i < fCaptureChannels; i++) {
            if (fGraphManager->GetConnectionsNum(fCapturePortList[i]) > 0) {
                CopyAndConvertIn(GetInputBuffer(i), fInputBuffer, fEngineControl->fBufferSize, i, fCaptureChannels, fBits);
            }
        }

    #ifdef JACK_MONITOR
        gCycleTable.fTable[gCycleCount].fAfterReadConvert = GetMicroSeconds();
    #endif

        return 0;
    }
}

int JackOSSDriver::Write()
{
    if (fOutFD < 0) {
        return 0;
    }

    ssize_t count;
    unsigned int skip = 0;
    audio_errinfo ei_out;

    if (fOSSWriteSync > 0) {
        jack_time_t now = GetMicroSeconds();
        long long progress = fEngineControl->fBufferSize;
        if (fForceBalancing) {
            fForceBalancing = false;
            progress += fBufferBalance;
            jack_info("JackOSSDriver::Write recording offset %ld sync %ld ago", fOSSReadOffset, now - fOSSReadSync);
            jack_info("JackOSSDriver::Write playback offset %ld sync %ld ago", fOSSWriteOffset, now - fOSSWriteSync);
            jack_info("JackOSSDriver::Write buffer balancing %ld", fBufferBalance);
        }
        long long passed = us_to_samples(now - fOSSWriteSync, fEngineControl->fSampleRate);
        long long consumed = passed - (passed % fOutBlockSize);
        long long tolerance = (fOutBlockSize > 1) ? 0 : fOSSMaxBlock;
        long long overdue = 0;
        if (consumed > fOSSWriteOffset + tolerance) {
            // Skip playback to avoid buffer accumulation through excessive input.
            overdue = consumed - fOSSWriteOffset - tolerance;
            jack_error("JackOSSDriver::Write late by %ld, skip %ld frames", passed - fOSSWriteOffset, overdue);
            jack_error("JackOSSDriver::Write %ld frame offset from sync %ld us ago", fOSSWriteOffset, now - fOSSWriteSync);
        }
        long long write_length = progress - overdue;
        if (write_length > fEngineControl->fBufferSize) {
            jack_nframes_t fill = write_length - fEngineControl->fBufferSize;
            WriteSilence(fill);
        }
        if (write_length <= 0) {
            skip += fOutputBufferSize;
            fOSSWriteOffset += progress;
        } else if (write_length < fEngineControl->fBufferSize) {
            skip += (fEngineControl->fBufferSize - write_length) * fSampleSize * fPlaybackChannels;
            fOSSWriteOffset += overdue;
        }
    }

#ifdef JACK_MONITOR
    gCycleTable.fTable[gCycleCount].fBeforeWriteConvert = GetMicroSeconds();
#endif

    memset(fOutputBuffer, 0, fOutputBufferSize);
    for (int i = 0; i < fPlaybackChannels; i++) {
        if (fGraphManager->GetConnectionsNum(fPlaybackPortList[i]) > 0) {
            CopyAndConvertOut(fOutputBuffer, GetOutputBuffer(i), fEngineControl->fBufferSize, i, fPlaybackChannels, fBits);
        }
    }

  #ifdef JACK_MONITOR
    gCycleTable.fTable[gCycleCount].fBeforeWrite = GetMicroSeconds();
  #endif

    if (skip < fOutputBufferSize) {
        count = ::write(fOutFD, ((char*)fOutputBuffer) + skip, fOutputBufferSize - skip);
    } else {
        skip = fOutputBufferSize;
        count = 0;
    }

    if (count > 0) {
        fOSSWriteOffset += (count / (fSampleSize * fPlaybackChannels));
    }

  #ifdef JACK_MONITOR
    if (count > 0 && count != (int)(fOutputBufferSize - skip))
        jack_log("JackOSSDriver::Write count = %ld", count / (fSampleSize * fPlaybackChannels));
    gCycleTable.fTable[gCycleCount].fAfterWrite = GetMicroSeconds();
    gCycleCount = (gCycleCount == CYCLE_POINTS - 1) ? gCycleCount: gCycleCount + 1;
  #endif

    static unsigned int sample_count = 0;
    if (count > 0) {
        sample_count += count / (fSampleSize * fPlaybackChannels);
    }
    static unsigned int cycle_count = 0;
    if (++cycle_count % 1000 == 0) {
        jack_info("JackOSSDriver::Write total played samples = %ld", sample_count);
    }

    // XRun detection
    if (ioctl(fOutFD, SNDCTL_DSP_GETERROR, &ei_out) == 0) {

        if (ei_out.play_underruns > 0) {
            jack_error("JackOSSDriver::Write underruns = %d", ei_out.play_underruns);
            jack_time_t cur_time = GetMicroSeconds();
            //! \todo Improve underrun notification with real time info.
            NotifyXRun(cur_time, float(cur_time - fBeginDateUst));   // Better this value than nothing...
        }

        if (ei_out.play_errorcount > 0 && ei_out.play_lasterror != 0) {
            jack_error("%d OSS play event(s), last=%05d:%d",ei_out.play_errorcount, ei_out.play_lasterror, ei_out.play_errorparm);
        }
    }

    if (count < 0) {
        jack_log("JackOSSDriver::Write error = %s", strerror(errno));
        return -1;
    } else if (count < (int)(fOutputBufferSize - skip)) {
        jack_error("JackOSSDriver::Write error bytes written = %ld", count);
        return -1;
    } else {
        return 0;
    }
}

int JackOSSDriver::SetBufferSize(jack_nframes_t buffer_size)
{
    CloseAux();
    //! \todo Adjust the latency values when changing buffer size.
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
    jack_driver_descriptor_add_parameter(desc, &filler, "excl", 'e', JackDriverParamBool, &value, NULL, "Exclusif (O_EXCL) access mode", NULL);

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
