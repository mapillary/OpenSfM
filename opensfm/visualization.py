import pypangolin as pango
from OpenGL.GL import *
import numpy as np
import threading
import time


class Visualization(object):

    def __init__(self, img_shape):
        self.points = []
        self.shots = []
        self.orig_img_shape = img_shape #np.array((370, 1226))
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
        print("self.new_img_shape", self.new_img_shape)
        self.im3 = None
        self.img_lock = threading.Lock()
        self.scene_lock = threading.Lock()
        self.img_changed = False

        self.size_factor = 0.1
        self.fx = self.fy = (self.cam_height + self.cam_width) / 2
        self.cx = self.cam_width / 2
        self.cy = self.cam_height / 2

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

    def update_reconstruction(self, rec, kfs = None):
        self.scene_lock.acquire()
        self.points.clear()
        for point in rec.points.values():
            self.points.append((point.coordinates, point.color/255.0))

        self.shots.clear()
        if kfs is not None:
            for shot in kfs:
                self.shots.append((shot.id, shot.pose.get_cam_to_world().T))
        else:
            for shot in rec.shots.values():
                self.shots.append((shot.id, shot.pose.get_cam_to_world().T))
        self.scene_lock.release()

    def update_texture(self):
        self.img_lock.acquire()
        if self.img_changed:
            self.tex_img.Upload(np.array(self.im3, dtype=np.uint8), GL_BGR, GL_UNSIGNED_BYTE)
            self.img_changed = False
        self.img_lock.release()

    def run_visualization(self):
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

    def draw_scene(self):
        self.scene_lock.acquire()
        self.draw_shots_and_trajectory()
        self.draw_points()
        self.scene_lock.release()

    def draw_points(self):
        if len(self.points) == 0:
            return
        glPointSize(self.point_size)
        glBegin(GL_POINTS)
        
        for point, color in self.points:
            glColor3f(color[0], color[1], color[2])
            glVertex3f(point[0], point[1], point[2])
        glEnd()
        

    def draw_trajectory(self, pos1, pos2):
        glLineWidth(3)
        glColor3f(0, 1, 0)
        glBegin(GL_LINES)
        glVertex3f(pos1[0], pos1[1], pos1[2])
        glVertex3f(pos2[0], pos2[1], pos2[2])
        glEnd()

    def draw_shots_and_trajectory(self):
        if len(self.shots) == 0:
            return
        prev_Twc = self.shots[0][1]
        self.draw_single_shot(prev_Twc)
        for idx in range(1, len(self.shots)):
            shot = self.shots[idx]
            T_wc = shot[1]

            self.draw_single_shot(T_wc)
            self.draw_trajectory(T_wc[3, 0:3], prev_Twc[3, 0:3])
            prev_Twc = self.shots[idx - 1][1]


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



    # def set_image(self, im):
    #     if len(im.shape) == 2: # monocular
    #         # make it to into three channels
    #         self.im3 = np.dstack((im, im, im))
    #     else:
    #         self.im3 = im
    #     if (self.orig_img_shape[1] != self.new_img_shape[1]):
    #         self.im3 = np.hstack((self.im3, np.zeros((self.new_img_shape[0], self.new_img_shape[1]-self.orig_img_shape[1],3))))
        


# def draw_camera(line_width, color, size_factor, cam_id,
#                 T_wc):

#     glPushMatrix()
#     sz = size_factor
#     width = 640
#     height = 640
#     fx = fy = (width+height) / 2
#     cx = width/2
#     cy = height/2
#     # Sophus::Matrix4f m = camToWorld.matrix().cast<float>();

#     glMultMatrixf(T_wc)
#     if (color == 0):
#         glColor3f(1, 0, 0)
#     else:
#         glColor3f(color[0], color[1], color[2])

#     glLineWidth(line_width)
#     glBegin(GL_LINES)
#     glVertex3f(0, 0, 0)
#     glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz)
#     glVertex3f(0, 0, 0)
#     glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz)
#     glVertex3f(0, 0, 0)
#     glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz)
#     glVertex3f(0, 0, 0)
#     glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz)

#     glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz)
#     glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz)

#     glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz)
#     glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz)

#     glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz)
#     glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz)

#     glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz)
#     glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz)
#     glEnd()
#     glPopMatrix()


# def draw_trajectory(pos1, pos2):
#     glLineWidth(3)
#     glColor3f(0, 1, 0)
#     glBegin(GL_LINES)
#     glVertex3f(pos1[0], pos1[1], pos1[2])
#     glVertex3f(pos2[0], pos2[1], pos2[2])
#     glEnd()


# def main():
#     win = pango.CreateWindowAndBind("pySimpleDisplay", 1920, 1080)
#     glEnable(GL_DEPTH_TEST)

#     pm = pango.ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000)
#     mv = pango.ModelViewLookAt(-0, 0.5, -3, 0, 0, 0, pango.AxisY)
#     s_cam = pango.OpenGlRenderState(pm, mv)

#     ui_width = 180

#     orig_img_shape = np.array((370, 1226))
#     new_img_shape = orig_img_shape.copy()
#     # must be divisible by four
#     # h = img_shape[0]
#     w = orig_img_shape[1]
#     if w % 4 != 0:
#         new_img_shape[1] = int(np.ceil(w/4.0)*4)
#     print(orig_img_shape, new_img_shape)
#     # exit()

#     handler = pango.Handler3D(s_cam)
#     d_cam = (
#         pango.CreateDisplay()
#         .SetBounds(
#             pango.Attach(0),
#             pango.Attach(1),
#             pango.Attach.Pix(ui_width),
#             pango.Attach(1),
#             -640.0 / 480.0,
#         )
#         .SetHandler(handler)
#     )

#     d_img = (
#         pango.Display("img")
#         # .SetAspect(1228/370.0)
#         .SetAspect(new_img_shape[0]/new_img_shape[1])
#     )

#     test = (
#         pango.CreateDisplay()
#         .SetBounds(pango.Attach(0), pango.Attach(0.2), pango.Attach.Pix(ui_width), pango.Attach(0.5))
#         .SetLayout(pango.Layout.Equal)
#         .AddDisplay(d_img)
#     )


#     pango.CreatePanel("ui").SetBounds(
#         pango.Attach(0), pango.Attach(1), pango.Attach(0), pango.Attach.Pix(ui_width)
#     )
#     var_ui = pango.Var("ui")
#     var_ui.a_Button = False
#     var_ui.a_double = (0.0, pango.VarMeta(0, 5))
#     var_ui.an_int = (2, pango.VarMeta(0, 5))
#     var_ui.a_double_log = (3.0, pango.VarMeta(1, 1e4, logscale=True))
#     var_ui.a_checkbox = (False, pango.VarMeta(toggle=True))
#     var_ui.an_int_no_input = 2
#     var_ui.a_str = "sss"
#     tex_img = pango.GlTexture(1228, 370, 3)
#     ctrl = -96
#     pango.RegisterKeyPressCallback(ctrl + ord("a"), a_callback)

#     while not pango.ShouldQuit():
#         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

#         if var_ui.a_checkbox:
#             var_ui.an_int = var_ui.a_double

#         var_ui.an_int_no_input = var_ui.an_int

#         d_cam.Activate(s_cam)
        
#         # pango.glDrawColouredCube()
#         draw_camera(2, [1,0,0], 2, "test", np.eye(4))
#         T_2 = np.eye(4)
#         T_2[:, 3] = [5,2,3,1]
#         # print(T_2)
#         draw_camera(2, [1,0,0], 2, "test", T_2.T)
#         draw_trajectory([0, 0, 0], T_2[0:3, 3])
#         d_img.Activate()
#         test.Activate()
#         glColor4f(1.0, 1.0, 1.0, 1.0)
#         im = cv2.imread('/home/fschenk/software/mapillary_repos/new_datastructures/ds_pr/OpenSfM/debug/track_000491.png')
#         # print(im)
#         im3 = np.hstack((im, np.zeros((370,2,3))))
#         # print(im3.shape)
#         # assert np.allclose(im[0, 0, :], im3[0, 0, :])
#         tex_img.Upload(np.array(im3, dtype=np.uint8), GL_BGR, GL_UNSIGNED_BYTE)
#         tex_img.RenderToViewportFlipY()
#         pango.FinishFrame()


# if __name__ == "__main__":
#     # main()
#     vis = Visualization(np.array([370, 1226]))
#     im = cv2.imread('/home/fschenk/software/mapillary_repos/new_datastructures/ds_pr/OpenSfM/debug/track_000491.png')
#     # vis.update_image(im)
#     vis.set_image(im)
#     vis.run_visualization()
