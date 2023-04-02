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
#include "JackOSSChannel.h"
#include "JackEngineControl.h"
#include "JackGraphManager.h"
#include "JackError.h"
#include "JackTime.h"
#include "JackShmMem.h"
#include "memops.h"

#include <cstdint>
#include <sys/ioctl.h>
#include <sys/soundcard.h>
#include <fcntl.h>
#include <iostream>
#include <assert.h>
#include <stdio.h>

using namespace std;

namespace Jack
{

bool JackOSSChannel::InitialSetup(unsigned int sample_rate)
{
    fFrameStamp = 0;
    fNextWakeup = 0;
    fXRunGap = 0;
    fCorrection.clear();
    return fFrameClock.set_sample_rate(sample_rate);
}

bool JackOSSChannel::OpenCapture(const char *device, bool exclusive, int sample_format, int &channels)
{
    if (channels == 0) channels = 2;

    if (!fReadChannel.set_parameters(sample_format, fFrameClock.sample_rate(), channels)) {
        jack_error("JackOSSChannel::OpenCapture unsupported sample format %#x", sample_format);
        return false;
    }

    if (!fReadChannel.open(device, exclusive)) {
        return false;
    }

    if (fReadChannel.sample_rate() != fFrameClock.sample_rate()) {
        jack_error("JackOSSChannel::OpenCapture driver forced sample rate %ld", fReadChannel.sample_rate());
        fReadChannel.close();
        return false;
    }

    jack_log("JackOSSChannel::OpenCapture capture file descriptor = %d", fReadChannel.file_descriptor());

    if (fReadChannel.channels() != channels) {
        channels = fReadChannel.channels();
        jack_info("JackOSSChannel::OpenCapture driver forced the number of capture channels %ld", channels);
    }

    fReadChannel.memory_map();
    fReadChannel.set_target_latency(0);

    return true;
}

bool JackOSSChannel::OpenPlayback(const char *device, bool exclusive, int sample_format, int &channels)
{
    if (channels == 0) channels = 2;

    if (!fWriteChannel.set_parameters(sample_format, fFrameClock.sample_rate(), channels)) {
        jack_error("JackOSSChannel::OpenPlayback unsupported sample format %#x", sample_format);
        return false;
    }

    if (!fWriteChannel.open(device, exclusive)) {
        return false;
    }

    if (fWriteChannel.sample_rate() != fFrameClock.sample_rate()) {
        jack_error("JackOSSChannel::OpenPlayback driver forced sample rate %ld", fWriteChannel.sample_rate());
        fWriteChannel.close();
        return false;
    }

    jack_log("JackOSSChannel::OpenPlayback playback file descriptor = %d", fWriteChannel.file_descriptor());

    if (fWriteChannel.channels() != channels) {
        channels = fWriteChannel.channels();
        jack_info("JackOSSChannel::OpenPlayback driver forced the number of playback channels %ld", channels);
    }

    fWriteChannel.memory_map();
    fWriteChannel.set_target_latency(0);

    return true;
}

bool JackOSSChannel::StartChannels(unsigned int buffer_frames)
{
    int group_id = 0;

    if (fReadChannel.recording()) {
        // Allocate two recording buffers for double buffering.
        size_t buffer_size = buffer_frames * fReadChannel.frame_size();
        sosso::Buffer buffer((char*) calloc(buffer_size, 1), buffer_size);
        assert(buffer.data());
        fReadChannel.set_buffer(std::move(buffer), 0);
        buffer = sosso::Buffer((char*) calloc(buffer_size, 1), buffer_size);
        assert(buffer.data());
        fReadChannel.set_buffer(std::move(buffer), buffer_frames);
        // Add recording channel to synced start group.
        fReadChannel.add_to_sync_group(group_id);
    }

    if (fWriteChannel.playback()) {
        // Allocate two playback buffers for double buffering.
        size_t buffer_size = buffer_frames * fWriteChannel.frame_size();
        sosso::Buffer buffer((char*) calloc(buffer_size, 1), buffer_size);
        assert(buffer.data());
        fWriteChannel.set_buffer(std::move(buffer), 0);
        buffer = sosso::Buffer((char*) calloc(buffer_size, 1), buffer_size);
        assert(buffer.data());
        fWriteChannel.set_buffer(std::move(buffer), buffer_frames);
        // Add playback channel to synced start group.
        fWriteChannel.add_to_sync_group(group_id);
    }

    // Start both channels in sync if supported.
    if (fReadChannel.recording()) {
        fReadChannel.start_sync_group(group_id);
    } else {
        fWriteChannel.start_sync_group(group_id);
    }

    // Init frame clock here to mark start time.
    if (!fFrameClock.init_clock(fFrameClock.sample_rate())) {
        return false;
    }

    // TODO: Improve correction limits for border cases.
    std::int64_t limit = buffer_frames / 2;
    fCorrection.set_loss_limits(-limit, limit);
    limit = limit / 2;
    fCorrection.set_drift_limits(-limit, limit);

    return true;
}

bool JackOSSChannel::StopChannels()
{
    if (fReadChannel.recording()) {
        free(fReadChannel.take_buffer().data());
        free(fReadChannel.take_buffer().data());
        fReadChannel.memory_unmap();
        fReadChannel.close();
    }

    if (fWriteChannel.playback()) {
        free(fWriteChannel.take_buffer().data());
        free(fWriteChannel.take_buffer().data());
        fWriteChannel.memory_unmap();
        fWriteChannel.close();
    }

    return true;
}

bool JackOSSChannel::CheckTimeAndRun()
{
    // Check current frame time.
    if (!fFrameClock.now(fFrameStamp)) {
        jack_error("JackOSSChannel::CheckTimeAndRun(): Frame clock failed.");
        return false;
    }
    std::int64_t now = fFrameStamp;
    // Round frame time down to steppings.
    now = now - (now % fReadChannel.stepping());

    if (fFrameStamp < fNextWakeup) {
        return true;
    }

    // Compute processing gap in case we are late.
    std::int64_t gap = 0;
    if (fReadChannel.recording() && fReadChannel.total_end() < now) {
        gap = std::max(gap, now - fReadChannel.period_end());
    }
    if (fWriteChannel.playback() && fWriteChannel.total_end() < now) {
        gap = std::max(gap, now - fWriteChannel.period_end());
    }
    // If late by more than one period, drop it and report an XRun.
    if (gap > 0) {
        jack_error("JackOSSChannel::CheckTimeAndRun(): Late by %lld frames.", gap);
        fXRunGap += gap;
        fReadChannel.reset_buffers(fReadChannel.end_frames() + gap);
        fWriteChannel.reset_buffers(fWriteChannel.end_frames() + gap);
    }

    // Process read channel if wakeup time passed, or OSS buffer data available.
    if (fReadChannel.recording()) {
        if (now >= fReadChannel.wakeup_time(fReadChannel.last_processing())) {
            if (!fReadChannel.process(now)) {
                jack_error("JackOSSChannel::CheckTimeAndRun(): Read process failed.");
                return false;
            }
        }
    }
    // Process write channel if wakeup time passed, or OSS buffer space available.
    if (fWriteChannel.playback()) {
        if (now >= fWriteChannel.wakeup_time(fWriteChannel.last_processing())) {
            if (!fWriteChannel.process(now)) {
                jack_error("JackOSSChannel::CheckTimeAndRun(): Write process failed.");
                return false;
            }
        }
    }

    fNextWakeup = std::min(fReadChannel.wakeup_time(now), fWriteChannel.wakeup_time(now));

    return true;
}

bool JackOSSChannel::Sleep() const
{
    if (fNextWakeup > fFrameStamp) {
        return fFrameClock.sleep(fNextWakeup);
    }
    return true;
}

bool JackOSSChannel::CaptureFinished() const
{
    return fReadChannel.finished(fFrameStamp);
}

bool JackOSSChannel::PlaybackFinished() const
{
    return fWriteChannel.finished(fFrameStamp);
}

std::int64_t JackOSSChannel::PlaybackCorrection()
{
    std::int64_t correction = 0;
    // If both channels are used, correct drift relative to recording balance.
    if (fReadChannel.recording() && fWriteChannel.playback()) {
        std::int64_t previous = fCorrection.correction();
        correction = fCorrection.correct(fWriteChannel.balance(), fReadChannel.balance());
        if (correction != previous) {
            jack_info("Playback correction changed from %lld to %lld.", previous, correction);
            jack_info("Read balance %lld vs write balance %lld.", fReadChannel.balance(), fWriteChannel.balance());
        }
    }
    return correction;
}

bool JackOSSChannel::Init()
{
    return true;
}

bool JackOSSChannel::Execute()
{
    if (Lock() && CheckTimeAndRun()) {
        if (fFrameStamp >= fNextWakeup) {
            jack_info("JackOSSChannel::Execute() running.");
            fNextWakeup = fFrameStamp + 5 * 48000;
            return Unlock();
        } else {
            // Unlock mutex before going to sleep, let others process.
            std::int64_t wakeup = fNextWakeup;
            return Unlock() && fFrameClock.sleep(wakeup);
        }
    }
    return false;
}

} // end of namespace
