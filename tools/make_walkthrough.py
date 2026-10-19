"""Build a labeled walkthrough from native CAD captures and real RViz footage."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.export_bundle import read_bundle


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_capture(bundle, capture):
    package, expected = read_bundle(bundle)
    native = json.loads((bundle/'native.json').read_text())
    report = json.loads((capture/'rviz.json').read_text())
    if report.get('status') != 'pass' or report.get('evidence_kind') != 'real-rviz-with-verified-mock-control':
        raise ValueError('A passing real RViz capture receipt is required')
    if report['native_receipt_sha256'] != digest(bundle/'native.json'):
        raise ValueError('RViz footage belongs to another native export')
    target = expected['moved']['positions']
    if package.name != 'rotary_slide_description' or len(expected['zero']['link_poses']) != 6 or set(target) != {'turn','boom_1_extend'} or not (
            math.isclose(target['turn'],math.pi/4,abs_tol=1e-8) and
            math.isclose(target['boom_1_extend'],.025,abs_tol=1e-8)):
        raise ValueError('This walkthrough template requires the rotary-slide demo and its documented trajectory')
    if report.get('ros_distro') != 'jazzy' or report.get('rmw_implementation') != 'rmw_cyclonedds_cpp':
        raise ValueError('This walkthrough template requires the recorded Jazzy/CycloneDDS environment')
    for phase in ('zero','moved'):
        if report['images'][phase]['positions'] != expected[phase]['positions']:
            raise ValueError('A recorded pose label differs from the native expectations')
    if report['control']['action_error_code'] != 0:
        raise ValueError('The recorded trajectory did not succeed')
    if any(report['control']['controllers'].get(name) != 'active' for name in
           ('joint_state_broadcaster','joint_trajectory_controller')):
        raise ValueError('Both recorded controllers must be active')
    files = [(capture/'rviz-zero.png',report['images']['zero']['sha256']),
             (capture/'rviz-moved.png',report['images']['moved']['sha256']),
             (capture/'rviz-motion.mp4',report['movie']['sha256']),
             (bundle/'fusion.png',native['images']['fusion']['sha256'])]
    if any(digest(path) != expected_hash for path, expected_hash in files):
        raise ValueError('A capture file changed since recording')
    links = set(expected['zero']['link_poses'])
    if {item['link'] for item in report['components']} != links:
        raise ValueError('Component labels differ from the native robot links')
    for item in report['components']:
        if item['urdf'] != 'urdf/components/'+item['link']+'.urdf' or not (package/item['urdf']).is_file():
            raise ValueError('A labeled component URDF is missing')
    return package, expected, report


def chapter_plan(package, components):
    robot = package.removesuffix('_description')
    continuation = chr(92)
    return [
        ('Fusion to ROS 2', ['A real six-link native CAD export.',
          'Nested repeated parts, rigid mounts, a rotary joint, and a slider.'], [], 'fusion'),
        ('1. Install the exporter', ['From the repository root, copy the complete exporter to Fusion\'s Scripts folder.',
          'macOS: ~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts',
          'Windows: %APPDATA%/Autodesk/Autodesk Fusion 360/API/Scripts',
          'The launchers require host Python 3.11+. Use --dry-run to inspect placement; --update retains backups and TOML settings.'],
          ['bash install.sh', '.\\install.cmd'], None),
        ('Optional: a single-file installer', ['Build a standard-library .pyz that bundles the exporter and its license notices.',
          'Recipients need Python 3.11+, but do not need the source checkout.',
          'The executable uses the same installer policy. It is not a native binary or a headless CAD converter.'],
          ['python3 tools/build_installer.py',
           'python3 artifacts/fusion360tourdf-install.pyz --dry-run'], None),
        ('2. Prepare the assembly', ['Use connected solid component occurrences; give moving links their own components.',
          'Choose base_link or an exact root_occurrence. Check materials and finite joint limits.',
          'Keep Capture Design History enabled. Hide geometry deliberately; hiding a joint marker does not suppress it.'], [], 'fusion'),
        ('3. Export inside Fusion', ['Utilities > Add-ins > Scripts and Add-ins > Scripts',
          'Select URDF_Exporter and Run. If absent, Add the installed folder.',
          'Choose a fresh output parent folder and leave the assembly unchanged.',
          'The installer only places files; it does not run a headless CAD export.'], [], None),
        ('4. Inspect the URDF catalog', ['The full robot retains joints and mock-control configuration.',
          'Each component URDF is a one-link model with its original visual, collision, mass, and inertia.',
          'Component files share the package\'s meshes; copy/install the full package.',
          'Display assets live in viewers/; control files remain in launch/ and config/.'],
          ['urdf/'+robot+'.urdf', 'urdf/'+robot+'.xacro',
           *[item['urdf'] for item in components]], None),
        ('Archive the package or viewers', ['A complete package ZIP includes URDF, component files, meshes, viewers, control configuration, and package metadata.',
          'A viewer-only ZIP is not a standalone robot. Extract viewers/ into its matching description package.',
          'Use copy installation with colcon build; symlink installation is not portable-mesh evidence.'],
          ['python3 -m zipfile -c '+package+'.zip '+package,
           'python3 -m zipfile -c '+robot+'_viewers.zip '+package+'/viewers'], None),
        ('5. Build and open RViz', ['Copy the generated package to your ROS workspace\'s src folder.',
          'Use Humble or Jazzy. Source your workspace after building.',
          'Choose display OR mock control; do not run both joint-state publishers together.',
          'The next segment is actual RViz footage with numbered presentation labels.'],
          ['source /opt/ros/jazzy/setup.bash', 'colcon build',
           'source install/setup.bash', 'ros2 launch '+package+' display.launch.py',
           'ros2 launch '+package+' display.launch.py gui:=false rviz:=false',
           'ros2 launch '+package+' control.launch.py rviz:=true'], None),
        ('6. Command mock joints', ['Actual ROS 2 Jazzy / RViz 2 capture.',
          'A successful trajectory rotates 45 deg and extends 25 mm.',
          'Joint-state feedback and all link transforms match the native expectations.',
          'Mock hardware is not a physics simulation or physical hardware.'], [], 'motion'),
        ('Send the same goal yourself', ['Keep control.launch.py running, then use a second sourced ROS terminal.',
          'From the repository root, send the supplied JSON-formatted YAML goal.',
          'The two joint names and positions match the native demo and recorded controller checks.'],
          ['ros2 action send_goal '+continuation,
           '  /joint_trajectory_controller/follow_joint_trajectory '+continuation,
           '  control_msgs/action/FollowJointTrajectory '+continuation,
           '  "$(cat Example/rotary_slide_goal.yaml)"'], None),
        ('7. Configure supported output', ['mesh_refinement: low, medium, or high',
          'collision_method: visual meshes or primitive_box',
          'effort / velocity: finite positive limits in the joint\'s SI units',
          'root_occurrence: automatic, an exact instance path, or design_root',
          'Rigid/revolute/continuous/slider joints, locked joints, fixed groups, and representable grounding are supported.',
          'Edit the TOML beside the installed entry script; a copied installation does not read an unrelated checkout config.'],
          ['Scripts/URDF_Exporter/xacro_export.toml'], None),
        ('8. Verify and contribute', ['Run the real native demo and verify its actual package, not a substitute fixture.',
          'The verifier checks geometry, negative controls, controllers, feedback, and transforms.',
          'Contributor setup and test recipes live in CONTRIBUTING.md.'],
          ['python3 -m unittest discover -s tests -v',
           'python3 tools/verify_export.py /path/to/native-bundle --output artifacts'], None),
        ('Know the limits', ['URDF is a tree. A closure sidecar records removed constraints but does not enforce closed-chain motion.',
          'Motion-link/mimic conversion, mesh-only physical policy, generalized joints, and general bodyless frames remain gaps.',
          'Combined STL output does not preserve per-face textures/materials.',
          'Keep the source history and retain reports; a picture is not universal CAD or physics proof.'], [], None),
        ('Community work and this integration', ['This is an integration fork of syuntoku14/fusion2urdf, not a from-scratch exporter.',
          'Frame, validation, visibility, and precision approaches credit haoyan-dev, Adriaeik, newtonjeri, and ACDC4Robot.',
          'Other pinned repositories informed comparison and regression cases, not necessarily copied code.',
          'changelog.md lists inherited work, adapted ideas, comparison-only references, and the fixes/tooling added here.',
          'LICENSES.md links the full pinned license texts; installed copies retain THIRD_PARTY_NOTICES.md and licenses/.'], [], None),
    ]


def font(size):
    from PIL import ImageFont
    return ImageFont.load_default(size=size)


def wrapped(draw, value, size, width):
    words, lines, current = value.split(), [], ''
    for word in words:
        candidate = (current+' '+word).strip()
        if current and draw.textlength(candidate,font=font(size)) > width:
            lines.append(current)
            current = word
        else:
            current = candidate
    return lines+[current]


def card(title, body, commands, image=None):
    from PIL import Image, ImageDraw, ImageOps
    result = Image.new('RGB',(1280,720),'#f5f7fa')
    draw = ImageDraw.Draw(result)
    draw.rectangle((0,0,1280,80),fill='#13283d')
    draw.text((36,22),title,font=font(34),fill='white')
    y = 110
    if image:
        picture = ImageOps.contain(Image.open(image).convert('RGB'),(620,540))
        result.paste(picture,((640-picture.width)//2,110+(540-picture.height)//2))
        left, width = 670, 570
    else:
        left, width = 48, 1184
    for text in body:
        for line in wrapped(draw,text,25,width):
            draw.text((left,y),line,font=font(25),fill='#20374d')
            y += 32
        y += 16
    for command in commands:
        for line in wrapped(draw,command,22,width):
            draw.text((left,y),line,font=font(22),fill='#075c98')
            y += 28
    if y > 695:
        raise ValueError('Chapter text exceeds the frame: '+title)
    return result


def label_screenshot(source, title, caption, legend):
    from PIL import Image, ImageDraw
    image = Image.open(source).convert('RGB')
    if image.size != (1280,720):
        raise ValueError('RViz screenshot must retain its recorded 1280x720 window')
    result = Image.new('RGB',(1280,960),'#f5f7fa')
    result.paste(image,(0,96))
    draw = ImageDraw.Draw(result)
    draw.rectangle((0,0,1280,96),fill='#13283d')
    draw.text((28,16),title,font=font(30),fill='white')
    draw.text((28,58),caption,font=font(19),fill='#c7ddec')
    for index, item in enumerate(legend):
        x, y = 24+(index%2)*630, 832+(index//2)*34
        draw.text((x,y),f"{item['number']}: {item['link']}.urdf",font=font(17),fill='#20374d')
    return result


def probe(path):
    data = json.loads(subprocess.run(['ffprobe','-v','error','-show_format','-show_streams','-of','json',str(path)],
                                    check=True,text=True,capture_output=True).stdout)
    video = next(stream for stream in data['streams'] if stream['codec_type'] == 'video')
    return dict(duration=float(data['format']['duration']),width=video['width'],height=video['height'],codec=video['codec_name'])


def make_movie(bundle, capture, output):
    from PIL import Image, ImageDraw
    package, expected, receipt = validate_capture(bundle,capture)
    if not shutil.which('ffmpeg') or not shutil.which('ffprobe'):
        raise RuntimeError('Install ffmpeg and ffprobe before making the walkthrough')
    output.mkdir(parents=True,exist_ok=False)
    for phase in ('zero','moved'):
        caption = 'CycloneDDS | GenericSystem mock hardware | '+('joint zero' if phase == 'zero' else '45 deg rotation + 25 mm extension')
        image = label_screenshot(capture/('rviz-'+phase+'.png'),
            'ROS 2 '+receipt['ros_distro'].title()+' / RViz 2 | '+phase+' pose',caption,receipt['components'])
        image.save(output/('ros-'+receipt['ros_distro']+'-'+phase+'.png'),optimize=True)
    overlay = Image.new('RGBA',(1280,720),(0,0,0,0))
    draw = ImageDraw.Draw(overlay)
    draw.rectangle((0,0,1280,64),fill='#13283d')
    draw.text((24,16),'Actual ROS 2 Jazzy / RViz 2 | verified mock-controller trajectory',font=font(27),fill='white')
    draw.rectangle((0,660,1280,720),fill='#13283d')
    draw.text((24,678),'45 deg rotary motion + 25 mm slider travel | labels correspond to component URDF files',font=font(21),fill='white')
    overlay.save(output/'motion-overlay.png')
    chapters, segments, elapsed = [], [], 0.
    encoder = ['-an','-r','20','-c:v','libx264','-preset','medium','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart']
    for index, (title, body, commands, illustration) in enumerate(chapter_plan(package.name,receipt['components'])):
        target = output/(f'chapter-{index:02}.mp4')
        if illustration == 'motion':
            arguments = ['-i',str(capture/'rviz-motion.mp4'),'-loop','1','-i',str(output/'motion-overlay.png'),
                '-filter_complex','[0:v]scale=1066:600,pad=1280:720:(ow-iw)/2:60:color=white[v];[v][1:v]overlay=0:0',
                '-t',str(receipt['movie']['duration'])]
        else:
            picture = card(title,body,commands,bundle/'fusion.png' if illustration == 'fusion' else None)
            still = output/(f'chapter-{index:02}.png')
            picture.save(still,optimize=True)
            if index == 0:
                shutil.copy2(still,output/'walkthrough-poster.png')
            arguments = ['-loop','1','-framerate','20','-i',str(still),'-t','9']
        subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y',*arguments,*encoder,str(target)],check=True)
        duration = probe(target)['duration']
        chapters.append(dict(title=title,start=elapsed,end=elapsed+duration,body=body,commands=commands))
        elapsed += duration
        segments.append(target)
    listing = output/'segments.txt'
    listing.write_text(''.join("file '"+str(path.resolve()).replace("'","'\\''")+"'\n" for path in segments))
    metadata = output/'chapters.ffmetadata'
    metadata.write_text(';FFMETADATA1\ntitle=Fusion to URDF walkthrough\n'+''.join(
        f"[CHAPTER]\nTIMEBASE=1/1000\nSTART={round(item['start']*1000)}\nEND={round(item['end']*1000)}\ntitle={item['title']}\n" for item in chapters))
    movie = output/'walkthrough.mp4'
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','concat','-safe','0','-i',str(listing),
                    '-i',str(metadata),'-map_metadata','1','-map_chapters','1','-c','copy','-movflags','+faststart',str(movie)],check=True)
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(capture/'rviz-motion.mp4'),
        '-filter_complex','fps=10,scale=800:-1:flags=lanczos,split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=3',
        '-loop','0',str(output/'rviz-motion.gif')],check=True)
    summary = dict(status='pass',native_receipt_sha256=digest(bundle/'native.json'),
        rviz_receipt_sha256=digest(capture/'rviz.json'),script_sha256=digest(__file__),
        movie=dict(path=movie.name,sha256=digest(movie),**probe(movie)),chapters=chapters,
        screenshots={phase:dict(path='ros-'+receipt['ros_distro']+'-'+phase+'.png',
            sha256=digest(output/('ros-'+receipt['ros_distro']+'-'+phase+'.png'))) for phase in ('zero','moved')})
    (output/'walkthrough.json').write_text(json.dumps(summary,indent=2)+'\n')
    (output/'walkthrough.md').write_text('# Walkthrough transcript\n\nIllustrated instructions with an actual ROS/RViz motion segment. No audio narration.\n\n'+
        '\n\n'.join('## '+item['title']+'\n\n'+'\n\n'.join(item['body'])+
        ('\n\n```text\n'+'\n'.join(item['commands'])+'\n```' if item['commands'] else '') for item in chapters)+'\n')
    print('PASS labeled RViz images and chaptered walkthrough: '+str(movie))
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle',type=Path,required=True)
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True,help='Fresh output directory')
    args = parser.parse_args()
    make_movie(args.bundle.resolve(),args.capture.resolve(),args.output.resolve())
