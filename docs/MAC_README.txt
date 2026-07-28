========================================================================
  RANGEN PLAYER  —  for Mac
  Play a robot recording: picture in Foxglove, sound data over OSC.
========================================================================

You do NOT need to install anything. No Python, no ROS, no Homebrew.
Everything is inside this folder.


------------------------------------------------------------------------
BEFORE YOU START  (once)
------------------------------------------------------------------------

  You need Google Chrome.
  Safari and Firefox will show an EMPTY page — this is not a bug you can
  fix, they block the kind of connection this uses. Chrome is free:
  https://www.google.com/chrome

  Your music software (Max, Ableton, SuperCollider...) should listen for
  OSC on:                 port 9000       (UDP, on this same Mac)


------------------------------------------------------------------------
STEP 1  —  Copy the app to your Desktop
------------------------------------------------------------------------

  Drag  RangenPlayer.zip  from the USB stick onto your Desktop.
  Then double-click it on the Desktop. A folder appears.

  ! Do this on the Desktop, not on the USB stick. Unzipping on the stick
    makes the app un-clickable. (The stick's format cannot remember which
    files are programs.)

  You can leave the recordings on the USB stick — the app looks there.


------------------------------------------------------------------------
STEP 2  —  Start it
------------------------------------------------------------------------

  Open the RangenPlayer folder on your Desktop.

  RIGHT-CLICK (or Control-click) on:   Start Rangen.command
  Choose:                              Open
  A warning appears. Click:            Open

  You only have to do the right-click once. After that a normal
  double-click works.

  ! macOS may ask "Do you want the application to accept incoming
    network connections?" — click Allow.


------------------------------------------------------------------------
STEP 3  —  Pick a recording
------------------------------------------------------------------------

  A black Terminal window opens and lists the recordings it found.
  Type the number you want and press Return.


------------------------------------------------------------------------
STEP 4  —  Watch and listen
------------------------------------------------------------------------

  Chrome opens by itself and shows the robot.
  The OSC stream starts at the same moment, on port 9000.

  In Chrome you can use the play / pause button and drag the time bar.
  The sound follows the picture exactly — they are the same playback.

  To stop: click the Terminal window and press  Control-C,
  or just close the window.


------------------------------------------------------------------------
WHAT IS BEING SENT  (for the music side)
------------------------------------------------------------------------

  All values are floats, sent 50 times a second, to UDP port 9000.
  These are plain OSC messages (not bundles).

    /rangen/ee/pos                x y z     robot hand position, metres
    /rangen/ee/quat               x y z w   robot hand orientation
    /rangen/ee/vel_lin            x y z     speed, metres/second
    /rangen/ee/vel_lin/mag        one value how fast overall
    /rangen/ee/vel_ang            x y z     turning speed, rad/second
    /rangen/ee/accel_lin          x y z     acceleration, m/s^2
    /rangen/ee/accel_lin/mag      one value how hard overall

  The same addresses also exist with /act/ in the middle
  (for example /rangen/ee/act/pos). Those are the robot's MEASURED
  movement; the ones above are what it was TOLD to do. Older recordings
  only contain the commanded ones, and the /act/ values stay at zero.


------------------------------------------------------------------------
IF SOMETHING GOES WRONG
------------------------------------------------------------------------

  Chrome opens but the page stays empty
      You are probably not in Chrome. Check the browser. If you are in
      Chrome, close the tab and start the player again.

  "cannot be opened because it is from an unidentified developer"
      You double-clicked instead of right-click -> Open. Go back to
      STEP 2 and use right-click.

  Nothing at all happens when you double-click
      You unzipped on the USB stick instead of the Desktop. Go back to
      STEP 1.

  The list of recordings is empty
      Put the .mcap recordings in the "bags" folder on the USB stick, or
      in a "bags" folder next to the app, and start it again.

  No sound / no OSC
      Check your music software is listening on port 9000, and that you
      answered "Allow" to the network permission question.

  The picture stutters on a recording with video
      Ask for that take to be trimmed, or start it from Terminal with
      only the topics you need — see MAC_BUNDLE.md in the source repo.
