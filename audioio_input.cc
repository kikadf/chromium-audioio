#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>

#include "base/logging.h"
#include "media/base/audio_timestamp_helper.h"
#include "media/audio/audioio/audio_manager_audioio.h"
#include "media/audio/audio_manager.h"
#include "media/audio/audioio/audioio_input.h"

namespace media {

static const SampleFormat kSampleFormat = kSampleFormatS16;

void *AudioIOAudioInputStream::ThreadEntry(void *arg) {
  AudioIOAudioInputStream* self = static_cast<AudioIOAudioInputStream*>(arg);

  self->ThreadLoop();
  return NULL;
}

void *AudioIOAudioInputStream::ThreadPosEntry(void *arg) {
  AudioIOAudioInputStream* self = static_cast<AudioIOAudioInputStream*>(arg);

  self->ThreadLoopPos();
  return NULL;
}

AudioIOAudioInputStream::AudioIOAudioInputStream(AudioManagerBase* manager,
                                             const std::string& device_name,
                                             const AudioParameters& params)
    : manager(manager),
      params(params),
      audio_bus(AudioBus::Create(params)),
      state(kClosed),
      mutex(PTHREAD_MUTEX_INITIALIZER) {
}

AudioIOAudioInputStream::~AudioIOAudioInputStream() {
  if (state != kClosed)
    Close();
}

// Open the stream and prepares it for recording. Call Start() to actually
// begin recording.
AudioInputStream::OpenOutcome AudioIOAudioInputStream::Open() {
  struct audio_info info;

  if (state != kClosed) {
    return OpenOutcome::kFailed;
  }

  if (params.format() != AudioParameters::AUDIO_PCM_LINEAR &&
      params.format() != AudioParameters::AUDIO_PCM_LOW_LATENCY) {
    LOG(WARNING) << "[AUDIOIO] Unsupported audio format.";
    return OpenOutcome::kFailed;
  }

  AUDIO_INITINFO(&info);
  info.mode = AUMODE_RECORD;
  info.record.sample_rate = params.sample_rate();
  info.record.channels = params.channels();
  info.record.precision = SampleFormatToBitsPerChannel(kSampleFormat);
  info.record.encoding = AUDIO_ENCODING_SLINEAR;
  info.record.pause = true;

  if ((fd = open("/dev/audio", O_RDONLY)) < 0) {
    LOG(ERROR) << "[AUDIOIO] Couldn't open audio device.";
    return OpenOutcome::kFailed;
  }

  if (ioctl(fd, AUDIO_SETINFO, &info) < 0) {
    goto error;
  }

  if (ioctl(fd, AUDIO_GETINFO, &info) < 0) {
    goto error;
  }

  state = kStopped;
  buffer = new char[audio_bus->frames() * params.GetBytesPerFrame(kSampleFormat)];
  LOG(INFO) << "[AUDIOIO] InputStream opened.";
  return OpenOutcome::kSuccess;
error:
  close(fd);
  LOG(ERROR) << "[AUDIOIO] Couldn't set audio parameters.";
  return OpenOutcome::kFailed;
}

// Starts recording audio and generating AudioInputCallback::OnData().
// The input stream does not take ownership of this callback.
void AudioIOAudioInputStream::Start(AudioInputCallback* cb) {
  struct audio_info info;

  StartAgc();

  (void)ioctl(fd, AUDIO_FLUSH, NULL);
  if (ioctl(fd, AUDIO_GETINFO, &info) < 0) {
    goto error;
  }
  info.record.pause = false;
  if (ioctl(fd, AUDIO_SETINFO, &info) < 0 ) {
    goto error;
  }

  state = kRunning;
  hw_delay = 0;
  callback = cb;

  if (pthread_create(&thread, NULL, &ThreadEntry, this) != 0) {
    LOG(ERROR) << "[AUDIOIO] Failed to create real-time thread for recording.";
    goto error;
  }

  if (pthread_create(&threadpos, NULL, &ThreadPosEntry, this) != 0) {
    LOG(ERROR) << "[AUDIOIO] Failed to create real-time threadpos for recording.";
    goto error;
  }

  LOG(INFO) << "[AUDIOIO] Recording started.";
  return;
error:
  LOG(ERROR) << "[AUDIOIO] Failed to start playing audio.";
  state = kStopped;
}

// Stops recording audio. Effect might not be instantaneous as there could be
// pending audio callbacks in the queue which will be issued first before
// recording stops.
void AudioIOAudioInputStream::Stop() {
  struct audio_info info;

  if (state == kStopped) {
    return;
  }
  state = kRunning;
  pthread_join(thread, NULL);
  pthread_join(threadpos, NULL);
  (void)ioctl(fd, AUDIO_FLUSH, NULL);
  if (ioctl(fd, AUDIO_GETINFO, &info) < 0) {
    goto error;
  }
  info.record.pause = true;
  if (ioctl(fd, AUDIO_SETINFO, &info) < 0 ) {
    goto error;
  }
  state = kStopped;
  StopAgc();
  LOG(INFO) << "[AUDIOIO] Recording stopped.";
  return;
error:
  LOG(ERROR) << "[AUDIOIO] Failed to stop recording audio.";
  return;
}

// Close the stream. This also generates AudioInputCallback::OnClose(). This
// should be the last call made on this object.
void AudioIOAudioInputStream::Close() {
  if (state == kClosed) {
    goto release;
  }
  if (state == kRunning) {
    Stop();
  }
  (void)ioctl(fd, AUDIO_FLUSH, NULL);
  close(fd);
  state = kClosed;
  delete [] buffer;

release:
  manager->ReleaseInputStream(this);  // Calls the destructor
  LOG(INFO) << "[AUDIOIO] InputStream closed.";
}

// Returns the maximum microphone analog volume or 0.0 if device does not
// have volume control.
double AudioIOAudioInputStream::GetMaxVolume() {
  // Not supported
  return 0.0;
}

// Sets the microphone analog volume, with range [0, max_volume] inclusive.
void AudioIOAudioInputStream::SetVolume(double volume) {
  // Not supported. Do nothing.
}

// Returns the microphone analog volume, with range [0, max_volume] inclusive.
double AudioIOAudioInputStream::GetVolume() {
  // Not supported.
  return 0.0;
}

// Returns the current muting state for the microphone.
bool AudioIOAudioInputStream::IsMuted() {
  // Not supported.
  return false;
}

// Sets the output device from which to cancel echo, if echo cancellation is
// supported by this stream. E.g. called by WebRTC when it changes playback
// devices.
void AudioIOAudioInputStream::SetOutputDeviceForAec(
    const std::string& output_device_id) {
  // Not supported.
}

void AudioIOAudioInputStream::ThreadLoop(void) {
  size_t todo, n;
  char *data;
  unsigned int nframes;
  double normalized_volume = 0.0;

  nframes = audio_bus->frames();
  LOG(INFO) << "[AUDIOIO] Input:ThreadLoop() started.";

  while (state == kRunning) {

    GetAgcVolume(&normalized_volume);

    // read one block
    todo = nframes * params.GetBytesPerFrame(kSampleFormat);
    data = buffer;
    while (todo > 0) {
      n = read(fd, data, todo);
      if (n == 0)
        return;	// unrecoverable I/O error
      todo -= n;
      data += n;
    }
    pthread_mutex_lock(&mutex);
    hw_delay -= nframes;
    pthread_mutex_unlock(&mutex);

    // convert frames count to TimeDelta
    const base::TimeDelta delay = AudioTimestampHelper::FramesToTime(hw_delay, params.sample_rate());

    // push into bus
    audio_bus->FromInterleaved<SignedInt16SampleTypeTraits>(reinterpret_cast<int16_t*>(buffer), nframes);

    // invoke callback
    callback->OnData(audio_bus.get(), base::TimeTicks::Now() - delay, 1., {});
  }
  LOG(INFO) << "[AUDIOIO] Input:ThreadLoop() stopped.";
}

void AudioIOAudioInputStream::ThreadLoopPos(void) {
  int ret;
  struct audio_offset offset;
  struct pollfd pfd;
  pfd.fd = fd;
  pfd.events = POLLIN;

  LOG(INFO) << "[AUDIOIO] Input:ThreadLoopPos() started.";

  while (state == kRunning) {
    ret = poll(&pfd, 1, -1);
    if (ret > 0 && (pfd.revents & POLLIN)) {
      if (ioctl(fd, AUDIO_GETIOFFS, &offset) < 0) {
        LOG(ERROR) << "[AUDIOIO] Input:ThreadLoopPos: Failed to get transfered blocks.";
      }
      pthread_mutex_lock(&mutex);
      hw_delay += offset.deltablks;
      pthread_mutex_unlock(&mutex);
    } else {
      LOG(ERROR) << "[AUDIOIO] Input:ThreadLoopPos: poll error.";
    }
  }
  LOG(INFO) << "[AUDIOIO] Input:ThreadLoopPos() stopped.";
}

}  // namespace media
