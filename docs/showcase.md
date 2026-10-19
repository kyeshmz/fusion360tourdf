# Capture real ROS pictures and build the walkthrough

The showcase combines an actual native Fusion export, real RViz screenshots and joint motion, and illustrated usage instructions. The numbered labels identify the same links as the component URDF catalog; they are presentation annotations, not extra robot geometry.

## Capture RViz

Create a fresh bundle with `tools/create_demo` in Fusion, then run this command from the checkout root:

```bash
bash tools/capture_ros.sh /absolute/path/to/native-bundle artifacts
```

Docker builds the optional GUI test image with RViz, Xvfb, software OpenGL, and FFmpeg. It builds and installs the supplied native package, checks component URDFs and portable resources, starts mock controllers, and observes joint feedback and transforms before capturing each pose.

The tool runs RViz in an isolated virtual X display, not on your desktop. It captures the zero pose and the commanded `turn = pi/4`, `boom_1_extend = 0.025 m` pose, plus the real trajectory motion. Each text/leader-line marker is locked to its corresponding URDF frame, so labels follow the moving parts.

Inspect `rviz.json`, `rviz.log`, the PNGs, and `rviz-motion.mp4` under the reported `rviz-*/capture/` directory. Require `status: pass`, active controllers, trajectory error code zero, and matching observations. The capture tool cleans up only its own container/image tag and process groups. It does not connect to hardware or simulate forces.

## Build the movie and labeled pictures

Install the separate media dependency and make FFmpeg/ffprobe available on your host:

```bash
.venv/bin/python -m pip install -r requirements-media.txt
```

Pass the same native bundle and the completed RViz capture to the movie builder. Choose a fresh output directory:

```bash
.venv/bin/python tools/make_walkthrough.py \
  --bundle /absolute/path/to/native-bundle \
  --capture /absolute/path/to/rviz-RUN/capture \
  --output artifacts/walkthrough-build
```

The builder checks input hashes, component identities, controller state, trajectory success, and native-receipt identity. It produces a chaptered `walkthrough.mp4`, a transcript, a poster, a motion GIF, labeled ROS PNGs, and `walkthrough.json`.

The movie uses text instructions and native images around an actual ROS/RViz recording. It has no audio narration and is not a continuous recording of mouse clicks in Fusion. Chapters cover installation, model preparation, export, component URDFs, display/control, configuration, verification, source credits, and known gaps.

## Publish reviewed media

Visually check labels, loaded geometry, framing, and every instruction card before copying approved outputs into `docs/images/` and `docs/media/`. Keep the source/output hashes with the media and update the README links. Do not substitute Blender images for RViz footage or label a mock-controller result as physics or hardware proof. Keep the optional single-file installer's Python requirement and the source-license attribution visible in the walkthrough.

The generated example package and component index live in [Example/rotary_slide_README.md](../Example/rotary_slide_README.md). Contributor requirements are separate in [CONTRIBUTING.md](../CONTRIBUTING.md).
