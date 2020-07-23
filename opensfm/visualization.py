import pypangolin as pango
from OpenGL.GL import *
import numpy as np
import threading
import time


class Visualization(object):

    def __init__(self, img_shape):
        # self.points = []
        self.shots = []
        self.orig_img_shape = np.array(img_shape)
        self.new_img_shape = self.orig_img_shape.copy()
        # must be divisible by four
        w = self.orig_img_shape[1]
        if w % 4 != 0:
            self.new_img_shape[1] = int(np.ceil(w / 4.0) * 4)
        
        self.pm = pango.ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000)
        self.mv = pango.ModelViewLookAt(-0, 0.5, -3, 0, 0, 0, pango.AxisY)
        self.cam_width = 640
        self.cam_height = 640
        self.cam_color = [1, 1, 1]
        self.line_width = 2
        self.point_size = 3
        self.point_color = [1, 0, 0]
        self.tex_img = None
        self.im3 = None
        self.img_lock = threading.Lock()
        self.scene_lock = threading.Lock()
        self.img_changed = False

        self.size_factor = 0.1
        self.fx = self.fy = (self.cam_height + self.cam_width) / 2
        self.cx = self.cam_width / 2
        self.cy = self.cam_height / 2

        self.vertex_buffer = pango.GlBufferData()
        self.color_buffer = pango.GlBufferData()
        self.num_points = 0
        self.num_vertex_points = 0
        self.tmp_vertex_buffer = np.array([], dtype=np.float32)
        self.tmp_color_buffer = np.array([], dtype=np.uint8)
        self.update_buffer = False
        self.vertex_lock = threading.Lock()

        self.is_running = False

    def update_image(self, im):
        if im is None:
            return
        self.img_lock.acquire()
        if len(im.shape) == 2:  # gray scale
            # make it to into three channels
            self.im3 = np.dstack((im, im, im))
        else:
            self.im3 = im
        if self.im3.shape[1] != self.new_img_shape[1]:
            self.im3 = np.hstack((self.im3, np.zeros((self.new_img_shape[0], self.new_img_shape[1] - self.im3.shape[1], 3))))
        self.img_changed = True
        self.img_lock.release()

    def draw_point_vertex_buffer(self):
        if (self.update_buffer):
            num_good_pts = self.num_points
            if num_good_pts > self.num_vertex_points:
                # Reinitializse
                self.num_vertex_points = int(round(num_good_pts*1.3))
                self.vertex_buffer.\
                    Reinitialise(pango.GlArrayBuffer,
                                 self.num_vertex_points * 3 * 32,
                                 GL_DYNAMIC_DRAW)
                self.color_buffer.\
                    Reinitialise(pango.GlArrayBuffer,
                                 self.num_vertex_points * 3 * 8,
                                 GL_DYNAMIC_DRAW)
            self.vertex_lock.acquire()
            # Now upload
            self.vertex_buffer.Upload(
                self.tmp_vertex_buffer, 4 * 3 * self.num_points)
            self.color_buffer.Upload(
                self.tmp_color_buffer, 1 * 3 * self.num_points)
            self.update_buffer = False
            self.vertex_lock.release()

        self.draw_points_buffer()

    def update_point_vertex_buffer(self, rec):
        self.vertex_lock.acquire()
        self.num_points = len(rec.points)
        if self.num_points > 0:
            self.tmp_color_buffer.resize(self.num_points, 3)
            self.tmp_vertex_buffer.resize(self.num_points, 3)
            for idx, point in enumerate(rec.points.values()):
                self.tmp_color_buffer[idx, :] = np.uint8(point.color)
                self.tmp_vertex_buffer[idx, :] = point.coordinates
            self.update_buffer = True
        self.vertex_lock.release()

    def update_reconstruction(self, rec, kfs = None):
        self.scene_lock.acquire()
        self.update_point_vertex_buffer(rec)
        self.shots.clear()
        if kfs is not None:
            for shot in kfs:
                self.shots.append((shot.unique_id, shot.pose.get_cam_to_world().T))
        else:
            for shot in rec.shots.values():
                self.shots.append((shot.unique_id, shot.pose.get_cam_to_world().T))
        self.shots.sort()
        self.scene_lock.release()

    def update_texture(self):
        self.img_lock.acquire()
        if self.img_changed:
            self.tex_img.Upload(np.array(self.im3, dtype=np.uint8), GL_BGR, GL_UNSIGNED_BYTE)
            self.img_changed = False
        self.img_lock.release()

    def run_visualization(self):
        self.is_running = True
        win = pango.CreateWindowAndBind("SLAM Visualizer", 1920, 1080)
        glEnable(GL_DEPTH_TEST)
        s_cam = pango.OpenGlRenderState(self.pm, self.mv)
        ui_width = 180
        handler = pango.Handler3D(s_cam)
        d_cam = (
            pango.CreateDisplay()
            .SetBounds(
                pango.Attach(0),
                pango.Attach(1),
                pango.Attach.Pix(ui_width),
                pango.Attach(1),
                -640.0 / 480.0,
            )
            .SetHandler(handler)
        )
        d_img = (
            pango.Display("img")
            .SetAspect(self.new_img_shape[0]/self.new_img_shape[1])
        )
        disp_img = (
            pango.CreateDisplay()
            .SetBounds(pango.Attach(0), pango.Attach(0.2), pango.Attach.Pix(ui_width), pango.Attach(0.5))
            .SetLayout(pango.Layout.Equal)
            .AddDisplay(d_img)
        )

        pango.CreatePanel("ui").SetBounds(
            pango.Attach(0), pango.Attach(1), pango.Attach(0), pango.Attach.Pix(ui_width)
        )
        self.tex_img = pango.GlTexture(self.new_img_shape[1],
                                       self.new_img_shape[0], 3)
        self.update_image(self.im3)
        while not pango.ShouldQuit():
            d_cam.Activate(s_cam)
            glClearColor(0.0, 0.0, 0.0, 1.0)
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            self.draw_scene()
            d_img.Activate()
            disp_img.Activate()
            glColor4f(1.0, 1.0, 1.0, 1.0)
            self.update_texture()
            self.tex_img.RenderToViewportFlipY()
            pango.FinishFrame()
            time.sleep(0.01)

        self.is_running = False

    def draw_scene(self):
        self.scene_lock.acquire()
        self.draw_shots_and_trajectory()
        self.draw_point_vertex_buffer()
        self.scene_lock.release()

    def draw_points_buffer(self):
        if self.num_points > 0:
            glPushMatrix()
            self.color_buffer.Bind()
            glColorPointer(3,GL_UNSIGNED_BYTE, 0, None)
            glEnableClientState(GL_COLOR_ARRAY)
            self.vertex_buffer.Bind()
            glVertexPointer(3,GL_FLOAT, 0, None)
            glEnableClientState(GL_VERTEX_ARRAY)
            glPointSize(self.point_size)
            glDrawArrays(GL_POINTS, 0, self.num_points)
            glDisableClientState(GL_VERTEX_ARRAY)
            self.vertex_buffer.Unbind()
            glDisableClientState(GL_COLOR_ARRAY)
            self.color_buffer.Unbind()
            glPopMatrix()

    # def draw_trajectory(self, pos1, pos2):
    #     glLineWidth(3)
    #     glColor3f(0, 1, 0)
    #     glBegin(GL_LINES)
    #     glVertex3f(pos1[0], pos1[1], pos1[2])
    #     glVertex3f(pos2[0], pos2[1], pos2[2])
    #     glEnd()

    def draw_shots_and_trajectory(self):
        if len(self.shots) == 0:
            return
        for _, T_wc in self.shots:
            self.draw_single_shot(T_wc)
        self.draw_trajectory()
        # prev_Twc = self.shots[0][1]
        # self.draw_single_shot(prev_Twc)
        # for idx in range(1, len(self.shots)):
        #     shot = self.shots[idx]
        #     T_wc = shot[1]

        #     self.draw_single_shot(T_wc)
        #     self.draw_trajectory(T_wc[3, 0:3], prev_Twc[3, 0:3])
        #     prev_Twc = self.shots[idx - 1][1]
        #     print(shot[0], ", ", self.shots[idx - 1][0])

    def draw_trajectory(self):
        if len(self.shots) > 1:
            glLineWidth(3)
            glColor3f(0, 1, 0)
            glBegin(GL_LINES)
            for idx in range(1, len(self.shots)):
                T_wc1 = self.shots[idx-1][1]
                T_wc2 = self.shots[idx][1]
                pos1 = T_wc1[3, 0:3]
                pos2= T_wc2[3, 0:3]
                glVertex3f(pos1[0], pos1[1], pos1[2])
                glVertex3f(pos2[0], pos2[1], pos2[2])
            glEnd()

    def draw_single_shot(self, T_wc):
        sz = self.size_factor
        cx = self.cx
        cy = self.cy
        fx = self.fx
        fy = self.fy
        height = self.cam_height
        width = self.cam_width
        glPushMatrix()
        glMultMatrixf(T_wc)
        
        glColor3f(self.cam_color[0], self.cam_color[1], self.cam_color[2])

        glLineWidth(self.line_width)
        glBegin(GL_LINES)
        glVertex3f(0, 0, 0)
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz)
        glVertex3f(0, 0, 0)
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz)
        glVertex3f(0, 0, 0)
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz)
        glVertex3f(0, 0, 0)
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz)

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz)
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz)

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz)
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz)

        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz)
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz)

        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz)
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz)
        glEnd()
        glPopMatrix()

