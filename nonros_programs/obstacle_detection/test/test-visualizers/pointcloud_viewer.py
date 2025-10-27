import math
import time
import cv2
import numpy as np
import pyrealsense2 as rs
import sys

class Viewer:
    def __init__(self, *args, **kwargs):
        self.WIN_NAME = 'RealSense'
        self.pitch, self.yaw = math.radians(0), math.radians(0)
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.distance = 2
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        self.decimate = 1
        self.scale = True
        self.color = True
        self.x_displ, self.y_displ, self.z_displ = 0, 0, 0

    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.x_displ, self.y_displ, self.z_displ = 0, 0, 0

    @property #thanks Ivan!
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def view_point(self):
        return self.translation + np.array((self.x_displ, self.y_displ, self.distance+self.z_displ), dtype=np.float32)


viewer = Viewer()

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

try:
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
except RuntimeError:
    print("No device connected, please connect a RealSense device")
    sys.exit()

# found_rgb = False
# for s in device.sensors:
#     if s.get_info(rs.camera_info.name) == 'RGB Camera':
#         found_rgb = True
#         break
# if not found_rgb:
#     print("The demo requires Depth camera with Color sensor")
#     exit(0)

config.enable_stream(rs.stream.depth, rs.format.z16, 30)
config.enable_stream(rs.stream.color, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Get stream profile and camera intrinsics
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height

# Processing blocks
pc = rs.pointcloud()
decimate = rs.decimation_filter()
#decimate.set_option(rs.option.filter_magnitude, 2 ** viewer.decimate)
colorizer = rs.colorizer()


def mouse_cb(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        viewer.mouse_btns[0] = True

    if event == cv2.EVENT_LBUTTONUP:
        viewer.mouse_btns[0] = False

    if event == cv2.EVENT_RBUTTONDOWN:
        viewer.mouse_btns[1] = True

    if event == cv2.EVENT_RBUTTONUP:
        viewer.mouse_btns[1] = False

    if event == cv2.EVENT_MBUTTONDOWN:
        viewer.mouse_btns[2] = True

    if event == cv2.EVENT_MBUTTONUP:
        viewer.mouse_btns[2] = False

    if event == cv2.EVENT_MOUSEMOVE:
        h, w = out.shape[:2]
        dx, dy = x - viewer.prev_mouse[0], y - viewer.prev_mouse[1]

        if viewer.mouse_btns[0]:
            viewer.yaw += float(dx) / w * 2
            viewer.pitch -= float(dy) / h * 2

        elif viewer.mouse_btns[1]:
            dp = np.array((dx / w, dy / h, 0), dtype=np.float32)
            viewer.translation -= np.dot(viewer.rotation, dp)

        elif viewer.mouse_btns[2]:
            dz = math.sqrt(dx**2 + dy**2) * math.copysign(0.01, -dy)
            viewer.translation[2] += dz
            viewer.distance -= dz

    if event == cv2.EVENT_MOUSEWHEEL:
        dz = math.copysign(0.1, flags)
        viewer.translation[2] += dz
        viewer.distance -= dz

    viewer.prev_mouse = (x, y)


cv2.namedWindow(viewer.WIN_NAME, cv2.WINDOW_AUTOSIZE)
cv2.resizeWindow(viewer.WIN_NAME, w, h)
cv2.setMouseCallback(viewer.WIN_NAME, mouse_cb)


def project(v):
    """project 3d vector array to 2d"""
    h, w = out.shape[:2]
    view_aspect = float(h)/w

    # ignore divide by zero for invalid depth
    with np.errstate(divide='ignore', invalid='ignore'):
        proj = v[:, :-1] / v[:, -1, np.newaxis] * \
            (w*view_aspect, h) + (w/2.0, h/2.0)

    # near clipping
    znear = 0.03
    proj[v[:, 2] < znear] = np.nan
    return proj


def view(v):
    """apply view transformation on vector array"""
    return np.dot(v - viewer.view_point, viewer.rotation) + viewer.view_point - viewer.translation


def line3d(out, pt1, pt2, color=(0x80, 0x80, 0x80), thickness=1):
    """draw a 3d line from pt1 to pt2"""
    p0 = project(pt1.reshape(-1, 3))[0]
    p1 = project(pt2.reshape(-1, 3))[0]
    if np.isnan(p0).any() or np.isnan(p1).any():
        return
    p0 = tuple(p0.astype(int))
    p1 = tuple(p1.astype(int))
    rect = (0, 0, out.shape[1], out.shape[0])
    inside, p0, p1 = cv2.clipLine(rect, p0, p1)
    if inside:
        cv2.line(out, p0, p1, color, thickness, cv2.LINE_AA)


def grid(out, pos, rotation=np.eye(3), size=1, n=10, color=(0x80, 0x80, 0x80)):
    """draw a grid on xz plane"""
    pos = np.array(pos)
    s = size / float(n)
    s2 = 0.5 * size
    for i in range(0, n+1):
        x = -s2 + i*s
        line3d(out, view(pos + np.dot((x, 0, -s2), rotation)),
               view(pos + np.dot((x, 0, s2), rotation)), color)
    for i in range(0, n+1):
        z = -s2 + i*s
        line3d(out, view(pos + np.dot((-s2, 0, z), rotation)),
               view(pos + np.dot((s2, 0, z), rotation)), color)


def axes(out, pos, rotation=np.eye(3), size=0.075, thickness=2):
    """draw 3d axes"""
    line3d(out, pos, pos +
           np.dot((0, 0, size), rotation), (0xff, 0, 0), thickness)
    line3d(out, pos, pos +
           np.dot((0, size, 0), rotation), (0, 0xff, 0), thickness)
    line3d(out, pos, pos +
           np.dot((size, 0, 0), rotation), (0, 0, 0xff), thickness)


def frustum(out, intrinsics, color=(0x40, 0x40, 0x40)):
    """draw camera's frustum"""
    orig = view([0, 0, 0])
    w, h = intrinsics.width, intrinsics.height

    for d in range(1, 6, 2):
        def get_point(x, y):
            p = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], d)
            line3d(out, orig, view(p), color)
            return p

        top_left = get_point(0, 0)
        top_right = get_point(w, 0)
        bottom_right = get_point(w, h)
        bottom_left = get_point(0, h)

        line3d(out, view(top_left), view(top_right), color)
        line3d(out, view(top_right), view(bottom_right), color)
        line3d(out, view(bottom_right), view(bottom_left), color)
        line3d(out, view(bottom_left), view(top_left), color)


def pointcloud(out, verts, texcoords, color, painter=True):
    """draw point cloud with optional painter's algorithm"""
    if painter:
        # Painter's algo, sort points from back to front

        # get reverse sorted indices by z (in view-space)
        # https://gist.github.com/stevenvo/e3dad127598842459b68
        v = view(verts)
        s = v[:, 2].argsort()[::-1]
        proj = project(v[s])
    else:
        proj = project(view(verts))

    if viewer.scale:
        proj *= 0.5**viewer.decimate

    h, w = out.shape[:2]

    # proj now contains 2d image coordinates
    j, i = proj.astype(np.uint32).T

    # create a mask to ignore out-of-bound indices
    im = (i >= 0) & (i < h)
    jm = (j >= 0) & (j < w)
    m = im & jm

    cw, ch = color.shape[:2][::-1]
    if painter:
        # sort texcoord with same indices as above
        # texcoords are [0..1] and relative to top-left pixel corner,
        # multiply by size and add 0.5 to center
        v, u = (texcoords[s] * (cw, ch) + 0.5).astype(np.uint32).T
    else:
        v, u = (texcoords * (cw, ch) + 0.5).astype(np.uint32).T
    # clip texcoords to image
    np.clip(u, 0, ch-1, out=u)
    np.clip(v, 0, cw-1, out=v)

    # perform uv-mapping
    out[i[m], j[m]] = color[u[m], v[m]]


out = np.empty((h, w, 3), dtype=np.uint8)
first = True
while True:
    # Grab camera data
    if not viewer.paused:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_frame = decimate.process(depth_frame)

        # Grab new intrinsics (may be changed by decimation)
        depth_intrinsics = rs.video_stream_profile(
            depth_frame.profile).get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_colormap = np.asanyarray(
            colorizer.colorize(depth_frame).get_data())

        if viewer.color:
            mapped_frame, color_source = color_frame, color_image
        else:
            mapped_frame, color_source = depth_frame, depth_colormap

        points = pc.calculate(depth_frame)
        pc.map_to(mapped_frame)

        # Pointcloud data to arrays
        v, t = points.get_vertices(), points.get_texture_coordinates()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
        texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

    # Render
    now = time.time()

    out.fill(0)

    grid(out, (0, 0.5, 1), size=1, n=10)
    frustum(out, depth_intrinsics)
    axes(out, view([0, 0, 0]), viewer.rotation, size=0.1, thickness=1)

    if not viewer.scale or out.shape[:2] == (h, w):
        pointcloud(out, verts, texcoords, color_source)
    else:
        tmp = np.zeros((h, w, 3), dtype=np.uint8)
        pointcloud(tmp, verts, texcoords, color_source)
        tmp = cv2.resize(
            tmp, out.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)
        np.putmask(out, tmp > 0, tmp)

    if any(viewer.mouse_btns):
        axes(out, view(viewer.view_point), viewer.rotation, thickness=4)

    dt = time.time() - now

    cv2.setWindowTitle(
        viewer.WIN_NAME, "RealSense (%dx%d) %dFPS (%.2fms) %s" %
        (w, h, 1.0/dt, dt*1000, "PAUSED" if viewer.paused else ""))

    cv2.imshow(viewer.WIN_NAME, out)
    key = cv2.waitKey(1)

    if key == ord("a"):
        viewer.y_displ += 1
    if key == ord("d"):
        viewer.y_displ -= 1
    if key == ord("w"):
        viewer.y_displ += 1
    if key == ord("s"):
        viewer.y_displ -= 1
    if key == 82:  # Up arrow key
        viewer.z_displ += 1
    if key == 84:  # Down arrow key
        viewer.z_displ -= 1

    if key == ord("r"):
        viewer.reset()

    if key == ord("p"):
        viewer.paused ^= True

    if key == ord("d"):
        viewer.decimate = (viewer.decimate + 1) % 3
        decimate.set_option(rs.option.filter_magnitude, 2 ** viewer.decimate)

    if key == ord("z"):
        viewer.scale ^= True

    if key == ord("c"):
        viewer.color ^= True

    if key == ord("s"):
        cv2.imwrite('./out.png', out)

    if key == ord("e"):
        print(np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3))
        points.export_to_ply('./out.ply', mapped_frame)

    if key in (27, ord("q")) or cv2.getWindowProperty(viewer.WIN_NAME, cv2.WND_PROP_AUTOSIZE) < 0:
        break

# Stop streaming
pipeline.stop()