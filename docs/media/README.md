# Walkthrough and real RViz captures

`walkthrough.mp4` is a chaptered 1280 × 720 H.264 illustrated guide. It includes actual ROS 2 Jazzy/RViz 2 motion recorded after controller/feedback/transform checks. It has no audio narration and does not pretend to record every Fusion menu click.

The media set includes:

- `walkthrough-poster.png` and `walkthrough.md`: poster and accessible chapter transcript
- `ros-jazzy-zero.png` and `ros-jazzy-moved.png`: labeled pictures identifying ROS version, RViz, mock hardware, joint pose, and component filenames
- `rviz-zero.png` and `rviz-moved.png`: unmodified source-window captures
- `rviz-motion.mp4` and `rviz-motion.gif`: actual RViz trajectory recording and its preview
- `rviz-capture.json`: source pose, controller, feedback, transform, component-label, and file-hash receipt
- `walkthrough.json`: chapter timing, output hashes, and source/script identities

The numbered text/leader lines are visualization markers, not additional URDF links. They are locked to each source URDF frame and use the same numbering as [the component catalog](../../Example/rotary_slide_README.md). Fixed sensor frames retain their shared rigid anchor while the visual/collision bodies remain separate.

These are real GUI captures from an isolated X display, not relabeled Blender images or desktop footage containing unrelated windows. Mock control does not simulate dynamics or operate physical hardware.

Follow [the capture/build guide](../showcase.md) to regenerate the set. The walkthrough credits inherited and adapted work and directs readers to [changelog.md](../../changelog.md) for the combined baseline and source distinctions.
