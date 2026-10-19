"""Capture real RViz images and motion after verified ROS controller feedback."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.export_bundle import read_bundle
from tools.verify_ros import check_installed, stop_launch, verify_launch


def rviz_config():
    return '''Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Global Options:
    Fixed Frame: base_link
    Background Color: 245; 247; 250
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid (meters)
      Enabled: true
      Cell Size: 0.05
      Plane Cell Count: 12
      Alpha: 0.15
      Color: 110; 125; 140
    - Class: rviz_default_plugins/RobotModel
      Name: Native Fusion export
      Enabled: true
      Visual Enabled: true
      Collision Enabled: false
      Description Source: Topic
      Description Topic:
        Value: /robot_description
        Durability Policy: Transient Local
        Reliability Policy: Reliable
    - Class: rviz_default_plugins/MarkerArray
      Name: URDF component labels
      Enabled: true
      Topic:
        Value: /fusion360tourdf/component_labels
        Durability Policy: Transient Local
        Reliability Policy: Reliable
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 0.75
      Pitch: 0.42
      Yaw: -0.9
      Focal Point:
        X: -0.02
        Y: 0.05
        Z: 0.15
      Target Frame: base_link
Window Geometry:
  Width: 1280
  Height: 720
  X: 0
  Y: 0
'''


def component_labels(expected):
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray

    result, legend = MarkerArray(), []
    offsets = ((.12, -.09, .025), (-.08, -.08, .025), (.08, -.05, .05),
               (-.08, .06, .055), (.07, .05, .05), (-.07, .04, .025))
    for index, (link, pose) in enumerate(expected['zero']['link_poses'].items(), 1):
        bounds = expected['zero']['bounds'][link]
        world = [(a+b)/2 for a, b in zip(*bounds)]
        center = [sum(pose[row][column]*(world[row]-pose[row][3]) for row in range(3))
                  for column in range(3)]
        offset = offsets[(index-1) % len(offsets)]
        label = [a+b for a, b in zip(center, offset)]
        alias = link if link == 'base_link' else '_'.join(link.split('_')[-2:])
        text = Marker()
        text.header.frame_id, text.ns, text.id = link, 'components', 2*index
        text.type, text.action, text.frame_locked = Marker.TEXT_VIEW_FACING, Marker.ADD, True
        text.pose.position = Point(x=label[0], y=label[1], z=label[2])
        text.pose.orientation.w = 1.
        text.scale.z = .018
        text.color.r, text.color.g, text.color.b, text.color.a = .04, .12, .22, 1.
        text.text = f'{index}:{alias}'
        line = Marker()
        line.header.frame_id, line.ns, line.id = link, 'components', 2*index+1
        line.type, line.action, line.frame_locked = Marker.LINE_LIST, Marker.ADD, True
        line.pose.orientation.w = 1.
        line.scale.x = .0008
        line.color.r, line.color.g, line.color.b, line.color.a = .1, .35, .65, 1.
        line.points = [Point(x=p[0], y=p[1], z=p[2]) for p in (center, label)]
        result.markers.extend((text, line))
        legend.append(dict(number=index, label=alias, link=link, urdf='urdf/components/' + link + '.urdf'))
    return result, legend


def capture(bundle, install, output):
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rclpy.utilities import get_rmw_implementation_identifier
    from visualization_msgs.msg import MarkerArray

    source, expected = read_bundle(bundle)
    package = Path(get_package_share_directory(source.name)).resolve()
    if not package.is_relative_to(install.resolve()):
        raise ValueError('RViz must use the installed native package')
    output.mkdir(parents=True, exist_ok=True)
    check_installed(package, source.name[:-len('_description')], output)
    config = output/'capture.rviz'
    config.write_text(rviz_config())
    rclpy.init()
    node = rclpy.create_node('capture_component_labels')
    publisher = node.create_publisher(MarkerArray, '/fusion360tourdf/component_labels',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    labels, legend = component_labels(expected)
    report = dict(status='running', evidence_kind='real-rviz-with-verified-mock-control',
                  ros_distro=os.environ['ROS_DISTRO'], rmw_implementation=get_rmw_implementation_identifier(),
                  native_receipt_sha256=hashlib.sha256((bundle/'native.json').read_bytes()).hexdigest(),
                  package=source.name, components=legend, images={})
    rviz = recorder = None
    try:
        with (output/'rviz.log').open('w') as log, (output/'record.log').open('w') as record_log:
            rviz = subprocess.Popen(['rviz2','-d',str(config)], stdout=log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            deadline = time.monotonic()+45
            while True:
                if rviz.poll() is not None:
                    raise RuntimeError('RViz exited; inspect rviz.log')
                found = subprocess.run(['xdotool','search','--onlyvisible','--pid',str(rviz.pid)],
                                       text=True,capture_output=True)
                if found.returncode == 0 and found.stdout.strip():
                    window = found.stdout.splitlines()[-1]
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError('RViz window did not appear')
                time.sleep(.1)
            subprocess.run(['xdotool','windowsize',window,'1280','720'],check=True)
            subprocess.run(['xdotool','windowmove',window,'0','0'],check=True)

            def on_pose(name, observed):
                nonlocal recorder
                publisher.publish(labels)
                time.sleep(2)
                image = output/('rviz-'+name+'.png')
                subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','x11grab',
                    '-video_size','1280x720','-i',os.environ['DISPLAY'],'-frames:v','1','-threads','1',str(image)],check=True)
                report['images'][name] = dict(path=image.name, positions=expected[name]['positions'],
                    sha256=hashlib.sha256(image.read_bytes()).hexdigest(), observed=observed)
                if name == 'zero':
                    recorder = subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y',
                        '-f','x11grab','-framerate','20','-video_size','1280x720','-i',os.environ['DISPLAY'],
                        '-c:v','libx264','-preset','veryfast','-crf','20','-pix_fmt','yuv420p',
                        '-movflags','+faststart',str(output/'rviz-motion.mp4')],
                        stdin=subprocess.PIPE,stdout=record_log,stderr=subprocess.STDOUT,start_new_session=True)

            report['control'] = verify_launch(source.name,'control',expected,output,60,on_pose=on_pose)
            recorder.communicate(b'q',timeout=15)
            if recorder.returncode != 0:
                raise RuntimeError('RViz recording failed; inspect record.log')
            recorder = None
            movie = output/'rviz-motion.mp4'
            probe = json.loads(subprocess.run(['ffprobe','-v','error','-show_format','-show_streams',
                                              '-of','json',str(movie)],check=True,text=True,capture_output=True).stdout)
            if float(probe['format']['duration']) < 3:
                raise RuntimeError('RViz recording is shorter than the commanded trajectory')
            report['movie'] = dict(path=movie.name, sha256=hashlib.sha256(movie.read_bytes()).hexdigest(),
                                   duration=float(probe['format']['duration']))
            report['status'] = 'pass'
    except Exception:
        report.update(status='fail',traceback=traceback.format_exc())
        raise
    finally:
        if recorder is not None:
            stop_launch(recorder)
        if rviz is not None:
            stop_launch(rviz)
        node.destroy_node()
        rclpy.shutdown()
        (output/'rviz.json').write_text(json.dumps(report,indent=2)+'\n')
    print('PASS actual RViz images, labeled components, controller feedback, and motion recording')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle',type=Path,required=True)
    parser.add_argument('--install',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    capture(args.bundle.resolve(),args.install,args.output.resolve())
