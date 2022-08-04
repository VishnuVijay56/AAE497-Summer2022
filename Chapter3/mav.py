"""
mav.py: class file for mav
    - Author: Vishnu Vijay
    - Created: 6/1/22
    - History:
        - 6/5: Switched from pyqtgraph to open3d for rendering
        - 6/7: Adding functionality for chapter 3 proj

"""

import numpy as np
import open3d as o3d
from helper import EulerRotationMatrix
import matplotlib.pyplot as plt
import matplotlib.widgets as wid

class MAV:
    ###
    # Constructor!
    # Initializes the MAV object, sets up a mesh of points for translation and rotation
    # Inputs:
    #   - state: initial state of the MAV
    ###
    def __init__(self, state):
        self.mav_state = state
        # Points that define original orientation and position of the MAV
        self.mav_body = self.get_points().T

        # Points that define the most recent orientation and position of the MAV
        self.mav_points = self.mav_body
        self.mav_points_rendering = self.mav_body

        # Mesh body of MAV
        self.mav_mesh = self.get_mesh()
        self.mav_mesh_colors = self.get_mesh_colors()

        # Updates points based on position and orientation of MAV
        self.update_mav_state()

        # Visualizer setup
        self.start_visualizer()


    ###
    # Rotates the MAV according to the passed rotation matrix
    # Inputs:
    #   - mav_points: points that describe the MAV's vertices
    #   - rot_mat: rotation matrix by which to rotate the MAV's vertices
    # Outputs:
    #   - mesh of points that describe MAV's vertices after rotation
    ###
    def rotate_mav(self, mav_points, rot_mat):
        return rot_mat @ mav_points


    ###
    # Translates the MAV from the initial coordinates to the new position
    # Inputs:
    #   - mav_points: points that describe the MAV's vertices
    #   - mav_pos: new position of the MAV
    # Outputs:
    #   - mesh of points that describe MAV's vertices after translation
    ###
    def translate_mav(self, mav_points, mav_pos):
        trans_points = mav_points.T

        for i in range(self.num_points):
            for j in range(3):
                trans_points[i][j] = trans_points[i][j] + mav_pos[j]
        return trans_points

    ###
    # Sets the global mav_state variable equal to the new state passed to function
    # Inputs:
    #   - new_state: new state to overwrite old state
    # Outputs:
    #   - N/A
    ###
    def set_mav_state(self, new_state):
        self.mav_state = new_state


    ###
    # Updates the MAV's vertices according to the values stored in the MAV global state variable
    # Calls rotate_mav() and translate_mav()
    # Inputs:
    #   - N/A
    # Outputs:
    #   - N/A
    ###
    def update_mav_state(self):
        # Update points for matplot MAV
        rot_mat = EulerRotationMatrix(self.mav_state.phi, self.mav_state.theta, self.mav_state.psi)
        rot_points = self.rotate_mav(self.mav_body, rot_mat)
        mav_pos = [self.mav_state.north, self.mav_state.east, -self.mav_state.altitude]
        self.mav_points = self.translate_mav(rot_points, mav_pos)

        # Update points for rendering MAV
        rot_mat2 = EulerRotationMatrix(-self.mav_state.phi, self.mav_state.theta, -self.mav_state.psi)
        rot_points2 = self.rotate_mav(self.mav_body, rot_mat2)
        mav_pos2 = [self.mav_state.north, -self.mav_state.east, -self.mav_state.altitude]
        self.mav_points_rendering = self.translate_mav(rot_points2, mav_pos2)


    ###
    # Used for initial setup of the MAV vertices according to hard-coded body parameters
    # Only called in class constructor
    # Inputs:
    #   - N/A
    # Outputs:
    #   - original set of points that describe the MAV's vertices relative to a body fixed reference frame
    ###
    def get_points(self):
        # MAV Body Parameters
        fuse_h = 1
        fuse_w = 1
        fuse_l1 = 2
        fuse_l2 = 1
        fuse_l3 = 4
        wing_l = 1
        wing_w = 6
        tail_h = 1
        tail_l = 1
        tail_w = 2

        # Generate Points
        points = np.array([[fuse_l1, 0, 0], #1
                           [fuse_l2, fuse_w/2, -fuse_h/2], #2
                           [fuse_l2, -fuse_w/2, -fuse_h/2], #3
                           [fuse_l2, -fuse_w/2, fuse_h/2], #4
                           [fuse_l2, fuse_w/2, fuse_h/2], #5
                           [-fuse_l3, 0, 0], #6
                           [0, wing_w/2, 0], #7
                           [-wing_l, wing_w/2, 0], #8
                           [-wing_l, -wing_w/2, 0], #9
                           [0, -wing_w/2, 0], #10
                           [-fuse_l3+tail_l, tail_w/2, 0], #11
                           [-fuse_l3, tail_w/2, 0], #12
                           [-fuse_l3, -tail_w/2, 0], #13
                           [-fuse_l3+tail_l, -tail_w/2, 0], #14
                           [-fuse_l3+tail_l, 0, 0], #15
                           [-fuse_l3, 0, -tail_h] #16
                           ])
        
        self.num_points = 16
        self.num_tri_faces = 13
        
        self.max_pos = 100
        scale = self.max_pos / 20
        points = scale * points

        return points


    ###
    # TODO: is this still relavant???
    # Inputs:
    #   - N/A
    # Outputs:
    #   - array of colors for vertices?
    ###
    def get_mesh_colors(self):
        # colors of mesh faces
        red = np.array([1., 0., 0., 1])
        green = np.array([0., 1., 0., 1])
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])
        mesh_colors = np.empty((self.num_points, 3, 4), dtype=np.float32)

        mesh_colors[0] = yellow
        mesh_colors[1] = yellow
        mesh_colors[2] = yellow
        mesh_colors[3] = yellow
        mesh_colors[4] = yellow
        mesh_colors[5] = yellow
        mesh_colors[6] = yellow
        mesh_colors[7] = yellow
        mesh_colors[8] = yellow
        mesh_colors[9] = yellow
        mesh_colors[10] = yellow
        mesh_colors[11] = yellow
        mesh_colors[12] = yellow
        mesh_colors[13] = yellow
        mesh_colors[14] = yellow
        mesh_colors[15] = yellow

        return mesh_colors


    ###
    # Generates an array of the set of points that make up the triangular faces of the MAV for rendering
    # Inputs:
    #   - N/A
    # Outputs:
    #   - array of points for triangular mesh
    ###
    def get_mesh(self):
        # setup (Nx3) array for set of meshes
        mesh = np.empty((self.num_tri_faces * 2, 3))
        
        # initialize the triangular faces required for rendering plane
        mesh[0] = [0, 1, 2] #1-2-3
        mesh[1] = [0, 1, 4] #1-2-5
        mesh[2] = [0, 3, 4] #1-4-5
        mesh[3] = [0, 2, 3] #1-3-4
        mesh[4] = [5, 1, 2] #6-2-3
        mesh[5] = [5, 1, 4] #6-2-5
        mesh[6] = [5, 2, 3] #6-3-4
        mesh[7] = [5, 3, 4] #6-4-5
        mesh[8] = [6, 7, 8] #7-8-9
        mesh[9] = [6, 8, 9] #7-9-10
        mesh[10] = [5, 14, 15] #6-15-16
        mesh[11] = [10, 11, 12] #11-12-13
        mesh[12] = [10, 12, 13] #11-13-14

        for i in range(self.num_tri_faces):
            for j in range(3):
                mesh[i + self.num_tri_faces][j] = mesh[i][2-j]

        return mesh

    
    ###
    # Displays the MAV to the screen. Essentially the 'run' function
    # Inputs:
    #   - N/A
    # Outputs:
    #   - N/A
    ###
    def display_mav(self):
        fig_plot, ax_plot = self.show_points()
        self.update_points(fig_plot, ax_plot)


    ###
    # Sets up the figure and axes for displaying the MAV and sliders
    # Inputs:
    #   - N/A
    # Outputs:
    #   - the figure everything is being placed in
    #   - the axes the MAV is being plotted on
    ###
    def show_points(self):
        fig = plt.figure()
        ax1 = plt.axes(projection = '3d')
        self.mav_scatter(ax1)
        plt.subplots_adjust(left=0.25, bottom=0.25)

        return fig, ax1

    
    ###
    # Plots the MAV on the axes, sets the axes limits, and resets the axes labels
    # Inputs:
    #   - ax_plot: the axes the points are being plotted on
    # Outputs:
    #   - N/A
    ###
    def mav_scatter(self, ax_plot):
        mav_points = self.mav_points.T
        x = mav_points[0]
        y = mav_points[1]
        z = -mav_points[2]
        ax_plot.scatter(x, y, z)
        ax_plot.set_xlim([-self.max_pos, self.max_pos])
        ax_plot.set_ylim([-self.max_pos, self.max_pos])
        ax_plot.set_zlim([-self.max_pos, self.max_pos])
        ax_plot.set_xlabel("North")
        ax_plot.set_ylabel("East")
        ax_plot.set_zlabel("Altitude")
        

    ###
    # Sets up the sliders for the MAV to be manipulated.
    # Inputs:
    #   - N/A
    # Outputs:
    #   - N/A
    ###
    def update_points(self, fig_plot, ax_plot):
        # Setup sliders to change model
        ax_north = plt.axes([0.25, 0.2, 0.65, 0.03])
        north_slider = wid.Slider(ax = ax_north, label = "North Position",
                                  valmin = -self.max_pos, valmax = self.max_pos,
                                  valinit = 0, orientation = "horizontal")
        
        ax_east = plt.axes([0.25, 0.15, 0.65, 0.03])
        east_slider = wid.Slider(ax = ax_east, label = "East Position",
                                  valmin = -self.max_pos, valmax = self.max_pos,
                                  valinit = 0, orientation = "horizontal")
        
        ax_alt = plt.axes([0.25, 0.1, 0.65, 0.03])
        alt_slider = wid.Slider(ax = ax_alt, label = "Altitude",
                                  valmin = -self.max_pos, valmax = self.max_pos,
                                  valinit = 0, orientation = "horizontal")
        
        ax_phi = plt.axes([0.2, 0.25, 0.03, 0.65])
        phi_slider = wid.Slider(ax = ax_phi, label = "Phi", #Roll Angle
                                  valmin = -180, valmax = 180,
                                  valinit = 0, orientation = "vertical")
        
        ax_theta = plt.axes([0.15, 0.25, 0.03, 0.65])
        theta_slider = wid.Slider(ax = ax_theta, label = "Theta", #Pitch Angle
                                  valmin = -90, valmax = 90,
                                  valinit = 0, orientation = "vertical")
        
        ax_psi = plt.axes([0.1, 0.25, 0.03, 0.65])
        psi_slider = wid.Slider(ax = ax_psi, label = "Psi", #Heading Angle
                                  valmin = -180, valmax = 180,
                                  valinit = 0, orientation = "vertical")
        
        # To be called when a slider is changed
        # Reassigns all state values according to slider values
        def update(val):
            self.mav_state.north = north_slider.val
            self.mav_state.east = east_slider.val
            self.mav_state.altitude = alt_slider.val
            self.mav_state.phi = phi_slider.val / 180 * np.pi
            self.mav_state.theta = -theta_slider.val / 180 * np.pi
            self.mav_state.psi = psi_slider.val / 180 * np.pi
            # Updates the points according to new state
            self.update_mav_state()
            # Clears the axes
            ax_plot.cla()
            # Plots new points
            self.mav_scatter(ax_plot)
            # Updates the 3d render according to new points
            self.update_render()
        
        # Calls update when sliders are changed
        north_slider.on_changed(update)
        east_slider.on_changed(update)
        alt_slider.on_changed(update)
        phi_slider.on_changed(update)
        theta_slider.on_changed(update)
        psi_slider.on_changed(update)

        # Show plot!
        plt.show()


    ###
    # Sets up the window for the rendering of MAV to be displayed.
    # Also displays initial rendering of MAV
    # Inputs:
    #   - N/A
    # Outputs:
    #   - N/A
    ###
    def start_visualizer(self):
        # Sets up reference frame mesh for rendering
        self.frame = o3d.geometry.TriangleMesh.create_coordinate_frame(self.max_pos / 5)

        # Create mesh for MAV
        self.rendering_R = EulerRotationMatrix(np.pi, 0, 0) # NED -> ENU coordinates
        vertices = o3d.utility.Vector3dVector(self.rotate_mav(self.mav_points_rendering.T, self.rendering_R).T)
        self.triangles = o3d.utility.Vector3iVector(self.mav_mesh)
        self.o3d_mesh = o3d.geometry.TriangleMesh(vertices, self.triangles)
        self.o3d_mesh.compute_vertex_normals()
        self.o3d_mesh.compute_triangle_normals()

        # Setup Visualizer (window)
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(width = 1920, height = 1080)
        self.vis.add_geometry(self.frame)
        self.vis.add_geometry(self.o3d_mesh)

        # Setup visualizer camera
        ctr = self.vis.get_view_control()
        ctr.set_lookat([0, 0, 0])
        ctr.set_front([1, -1, 1])
        ctr.set_up([0, 0, 5])
        ctr.set_zoom(self.max_pos / 30)


    ###
    # Updates the window where MAV is displayed based on slider updates
    # Inputs:
    #   - N/A
    # Outputs:
    #   - N/A
    ###
    def update_render(self):
        # Redraw the mesh render
        vertices = o3d.utility.Vector3dVector(self.rotate_mav(self.mav_points_rendering.T, self.rendering_R).T)
        self.o3d_mesh.vertices = vertices

        # Remove old geometry and add new geometry
        self.vis.update_geometry(self.o3d_mesh)
        self.vis.update_renderer()

        # Change where camera is looking - center on plane
        #ctr = self.vis.get_view_control()
        #ctr.set_lookat([self.mav_state.east, self.mav_state.north, self.mav_state.altitude])
        
        self.vis.poll_events()