Recorded engine sounds (optional - without them the game synthesises a placeholder engine sound).

Put 16-bit PCM WAV files in the folder of the kart class:
  rotax_max_senior/   kz2_iame/   kt100_dd/

Name each file after the throttle state and the steady engine rpm it was recorded at:
  on_<rpm>.wav    on throttle / under load   e.g. on_6000.wav  on_9000.wav  on_12000.wav
  off_<rpm>.wav   off throttle / overrun     e.g. off_7000.wav off_11000.wav

Tips
- 3-6 layers per throttle state covering idle to the limiter work well.
- Each file should loop cleanly (cut at zero crossings, 1-3 s long, steady rpm).
- Onboard recordings (mic behind the seat / on the helmet) give the driver's perspective.
- Only use recordings you made or have a licence for.
