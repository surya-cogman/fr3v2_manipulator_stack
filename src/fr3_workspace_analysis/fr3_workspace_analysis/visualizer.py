#!/usr/bin/env python3

import pyvista as pv
import numpy as np
import trimesh


class WorkspaceVisualizer:

    def __init__(self):

        self.plotter = pv.Plotter(window_size=(1400, 900))
        self.plotter.set_background("white")
        self.plotter.add_axes()
        self.plotter.show_grid()

    def add_robot(self,visual_model,visual_data):

        for geom_obj,geom_pose in zip(visual_model.geometryObjects,visual_data.oMg):
            mesh_path = geom_obj.meshPath

            if mesh_path == "":
                continue
            try:

                mesh = trimesh.load(mesh_path, force="mesh")

                vertices = np.asarray(mesh.vertices)

                faces = np.hstack((
                    np.full((len(mesh.faces), 1), 3),
                    mesh.faces
                ))

                pv_mesh = pv.PolyData(vertices, faces)

                pv_mesh.transform(
                    geom_pose.homogeneous,
                    inplace=True
                )

                self.plotter.add_mesh(
                    pv_mesh,
                    color="lightgray",
                    smooth_shading=True,
                    specular=0.2
                )

            except Exception as e:

                print(f"Failed to load {mesh_path}")
                print(e)

    def plot_workspace(self, points):

        cloud = pv.PolyData(points)

        self.plotter.add_points(
            cloud,
            color="blue",
            point_size=3,
            opacity=0.35,
            render_points_as_spheres=True,
        )

    def show(self):

        self.plotter.camera_position = "iso"
        self.plotter.show()

    def add_manipulability_workspace(self, points, manipulability):

        cloud = pv.PolyData(points)

        cloud["Manipulability"] = manipulability

        self.plotter.add_points(
            cloud,
            scalars="Manipulability",
            cmap="jet",
            point_size=2.5,
            opacity=0.5,
            render_points_as_spheres=True,
        )

        self.plotter.add_scalar_bar(
            title="Manipulability"
        )

    def add_operational_workspace(self, points):

        cloud = pv.PolyData(points)

        self.plotter.add_points(
            cloud,
            color="limegreen",
            point_size=2,
            opacity=0.1,
            render_points_as_spheres=True,

    )
        
    def add_bounding_box(self,
        xmin,
        xmax,
        ymin,
        ymax,
        zmin,
        zmax,
    ):

        bounds = (
            xmin,
            xmax,
            ymin,
            ymax,
            zmin,
            zmax,
        )

        box = pv.Box(bounds)

        self.plotter.add_mesh(
            box,
            style="wireframe",
            color="red",
            line_width=4,
        )

    def adding_title(self,title,size):
        return self.plotter.add_title(title,font_size=size)