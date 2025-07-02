#ifndef MEDIA_AUDIO_AUDIOIO_AUDIOIO_OUTPUT_H_
#define MEDIA_AUDIO_AUDIOIO_AUDIOIO_OUTPUT_H_

#include <pthread.h>
#include <sys/audioio.h>

#include "base/time/tick_clock.h"
#include "base/time/time.h"
#include "media/audio/audio_io.h"

namespace media {

class AudioManagerBase;

// Implementation of AudioOutputStream using audio(4)
class AudioIOAudioOutputStream : public AudioOutputStream {
 public:
  // The manager is creating this object
  AudioIOAudioOutputStream(const std::string& device_name,
                         const AudioParameters& params,
                         AudioManagerBase* manager);

  AudioIOAudioOutputStream(const AudioIOAudioOutputStream&) = delete;
  AudioIOAudioOutputStream& operator=(const AudioIOAudioOutputStream&) = delete;

  virtual ~AudioIOAudioOutputStream();

  // Implementation of AudioOutputStream.
  bool Open() override;
  void Close() override;
  void Start(AudioSourceCallback* callback) override;
  void Stop() override;
  void SetVolume(double volume) override;
  void GetVolume(double* volume) override;
  void Flush() override;

 private:
  enum StreamState {
    kClosed,            // Not opened yet
    kStopped,           // Device opened, but not started yet
    kRunning,           // Started, device playing
    kStopWait           // Stopping, waiting for the real-time thread to exit
  };

  // C-style call-backs
  static void* ThreadEntry(void *arg);

  // Continuously moves data from the producer to the device
  void ThreadLoop(void);
  // Our creator, the audio manager needs to be notified when we close.
  AudioManagerBase* manager;
  // Parameters of the source
  AudioParameters params;
  // Device file name of the audio device
  const std::string device_name;
  // Source stores data here
  std::unique_ptr<AudioBus> audio_bus;
  // Call-back that produces data to play
  AudioSourceCallback* source;
  // Handle of the audio device
  int fd;
  // Current state of the stream
  enum StreamState state;
  // High priority thread running ThreadLoop()
  pthread_t thread;
  // Protects vol, volpending
  pthread_mutex_t mutex;
  // Current volume in the AUDIO_MIN_GAIN..AUDIO_MAX_GAIN range
  int vol;
  // Set to 1 if volumes must be refreshed in the realtime thread
  int volpending;
  // Temporary buffer
  char* buffer;
};

}  // namespace media

#endif  // MEDIA_AUDIO_AUDIOIO_AUDIOIO_OUTPUT_H_
