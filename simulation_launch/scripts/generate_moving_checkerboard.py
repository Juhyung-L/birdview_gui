import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
import matplotlib.pyplot as plt

import sys

# make a helix with an elliptical base
# the ellipse is on the x-y plane and the helix stretches out in the +z direction
def generate_helix(length, width, height, trajectory_length, num_pts):
    t = np.linspace(0, trajectory_length, num_pts)
    x = length * np.cos(t)
    y = width * np.sin(t)
    z = height * t / trajectory_length

    pts = np.column_stack((x, y, z))
    return pts

# rotate the helix so that its base is on the y-z plane and stretches out in the +x direction
# move the helix in the z direction by z_off
# move the helix in the x direction by x_off
def transform_helix(pts, x_off, z_off):
    rot = Rotation.from_euler('y', 90.0, degrees=True)
    rot = rot.as_matrix()
    
    pts[:] = np.dot(rot, pts.T).T
    pts[:, 2] += z_off
    pts[:, 0] += x_off

########## functions used by scipy's minimize() ##########
# project the elliptical helix on a ellpsoid to make a ellipsoidal spiral
# coordinates computed by minimizing the distance from each point in the helix to the ellipsoid
def ellipsoid_func(pt, length, width, height, x_off, z_off):
    x, y, z = pt
    return ((x-x_off)**2)/(height**2) + (y**2)/(length**2) + ((z-z_off)**2)/(width**2) - 1

def objective_func(pt, length, width, height, x_off, z_off):
    return np.abs(ellipsoid_func(pt, length, width, height, x_off, z_off))

def project_helix_on_ellipsoid(pts, length, width, height, x_off, z_off):
    closest_pts = []
    constraint = {'type': 'eq', 'fun': lambda pt: ellipsoid_func(pt, length, width, height, x_off, z_off)}

    for pt in pts:
        result = minimize(lambda pt: np.abs(ellipsoid_func(pt, length, width, height, x_off, z_off)), pt, constraints=constraint)
        closest_pts.append(result.x)

    return np.array(closest_pts)
###########################################################

def compute_rpy_orientation(pts, origin):
    direction_vec = origin - pts[:] # direction vector between point and origin
    norm_direction_vec = direction_vec / np.linalg.norm(direction_vec, axis=1)[:, np.newaxis] # normalize the direction vector

    # the normal vector of the calibration board is the z-axis
    axes_of_rot = np.cross(np.array([0, 0, 1]), norm_direction_vec)
    angles = np.arccos(np.dot(np.array([0, 0, 1]), norm_direction_vec.T))

    resultant_array = []

    for i, (axis_of_rot, angle) in enumerate(zip(axes_of_rot, angles)):
        norm_axis_of_rot = axis_of_rot / np.linalg.norm(axis_of_rot)

        rotation = Rotation.from_rotvec(angle * norm_axis_of_rot).as_matrix()
        rpy = Rotation.from_matrix(rotation).as_euler('xyz', degrees=False)

        resultant_array.append(np.concatenate((pts[i], rpy)))

    return np.array(resultant_array)


def plot_point_cloud(pts):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c='r', marker='o')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Helical Point Cloud')

    plt.show()

if __name__=='__main__':
    if len(sys.argv) != 13:
        print("Provide 12 input arguments\n")
        exit()

    # length = 5
    # width = 5
    # height = 20
    # x_off = 3
    # z_off = 5
    # trajectory_length = 30
    # num_pts = 30
    # time_to_complete = 10

    # rows = 7
    # cols = 10
    # sq_size = 0.1

    # world_file = ""

    # parameters for generating checkerboard trajectory
    length = float(sys.argv[1])
    width = float(sys.argv[2])
    height = float(sys.argv[3])
    x_off = float(sys.argv[4])
    z_off = float(sys.argv[5])
    trajectory_length = float(sys.argv[6])
    num_pts = int(sys.argv[7])
    time_to_complete = float(sys.argv[8])

    # parameters for generating checkerboard
    rows = int(sys.argv[9])
    cols = int(sys.argv[10])
    sq_size = float(sys.argv[11])
    world_file = sys.argv[12]

    length /= 2.0
    width /= 2.0

    # project the elliptical helix on an ellpsoid
    pts = generate_helix(length, width, height, trajectory_length, num_pts)
    # plot_point_cloud(pts)

    transform_helix(pts, x_off, z_off)
    # plot_point_cloud(pts)

    ellipsoidal_spiral_pts = project_helix_on_ellipsoid(pts, length, width, height, x_off, z_off)
    # plot_point_cloud(ellipsoidal_spiral_pts)

    # at this point ellipsoidal_spiral_pts contains the an array of [x, y, z] points that lie on the ellipsoidal spiral
    # these points will be the key points that the calibration board passes
    # still need to calculate the orientation of the calibration board at these key points
    # the calibration board should face the center of the ellpsoid, which is at [0, 0, camera_z]

    origin = np.array([0, 0, z_off])
    final_pts = compute_rpy_orientation(ellipsoidal_spiral_pts, origin)

    # now write everything into a world file
    if world_file == "":
        world_file = "/home/dev_ws/src/simulation_launch/worlds/calibration.world"

    # preamble for world file
    sdf_string = """<sdf version='1.7'>
  <world name='default'>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <contact>
              <collide_bitmask>65535</collide_bitmask>
              <ode/>
            </contact>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <audio>
      <device>default</device>
    </audio>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>16.6066 -15.4926 10.0246 0 0.275643 2.35619</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>"""
    
    checkerboard_name = "_".join(["checkerboard", str(cols), str(rows), str(sq_size).replace(".", "_")])

    # add actor element
    sdf_string += """\n    <actor name='moving_checkerboard'>
      <link name='{name}_body'>""".format(name=checkerboard_name)

    # generate the SDF code for the checkerboard itself
    half_width = (cols * sq_size) / 2
    half_height = (rows * sq_size) / 2
    visual_elements = []
    for i in range(cols):
        for j in range(rows):
            # this creates either "1 1 1 1" or "0 0 0 1" depending on if the square
            # should be black or white. It may be possible to save on filespace by
            # using built-in Gazebo materials, but whether or not this is actually
            # an improvement is unclear. For one, I don't know the characteristics
            # of the built-in Gazebo materials - they may have specular reflection,
            # for example, which we don't want.
            rgba = " ".join([str((i+j) % 2)]*3) + " 1"
            x = i*sq_size + sq_size/2 - half_width
            y = j*sq_size + sq_size/2 - half_height
            element = """\n        <visual name="checker_{i}_{j}">
          <pose>{x} {y} 0 0 0 0</pose>
          <geometry>
            <box>
              <size>{size} {size} 0.0002</size>
            </box>
          </geometry>
          <material>
            <ambient>{rgba}</ambient>
            <diffuse>{rgba}</diffuse>
            <specular>{rgba}</specular>
            <emissive>{rgba}</emissive>
          </material>
        </visual>""".format(i=i, j=j, x=x, y=y, rgba=rgba, size=sq_size)
            sdf_string += element

    # Create the white backdrop, padding the checkerboard by one square on all sides
    # xsize = (cols + 2) * sq_size
    # ysize = (rows + 2) * sq_size
    # element = """\n        <visual name="checker_backdrop">
    #       <pose>0 0 0 0 0 0</pose>
    #       <geometry>
    #         <box>
    #           <size>{xsize} {ysize} 0.0001</size>
    #         </box>
    #       </geometry>
    #       <material>
    #         <ambient>1 1 1 1</ambient>
    #         <diffuse>1 1 1 1</diffuse>
    #         <specular>1 1 1 1</specular>
    #         <emissive>1 1 1 1</emissive>
    #       </material>
    #     </visual>""".format(xsize=xsize, ysize=ysize)
    # sdf_string += element

    sdf_string += """\n      </link>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="square">"""

    for i in range(num_pts):
        t = np.linspace(0, time_to_complete, num_pts)
        sdf_string += """\n          <waypoint>
            <time>{time}</time>
            <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
          </waypoint>""".format(time=t[i], 
                        x=final_pts[i][0],
                        y=final_pts[i][1],
                        z=final_pts[i][2],
                        roll=final_pts[i][3],
                        pitch=final_pts[i][4],
                        yaw=final_pts[i][5])

    sdf_string += "\n        </trajectory>"
    sdf_string += "\n      </script>"
    sdf_string += "\n    </actor>"

    sdf_string += """\n    <model name='my_entity'>
      <static>1</static>
      <link name='camera_back_link'>
        <inertial>
          <pose>0 0 0 0 -0 0</pose>
          <mass>1</mass>
          <inertia>
            <ixx>0.00166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.00166667</iyy>
            <iyz>0</iyz>
            <izz>0.00166667</izz>
          </inertia>
        </inertial>
        <collision name='camera_back_link_collision'>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <ode/>
            </contact>
            <friction>
              <ode/>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='camera_back_link_visual'>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
        </visual>
        <sensor name='camera_back' type='wideanglecamera'>
          <camera>
            <horizontal_fov>3.14159265</horizontal_fov>
            <image>
              <width>320</width>
              <height>320</height>
              <format>R8G8B8</format>
            </image>
            <clip>
              <near>0.02</near>
              <far>100</far>
            </clip>
            <lens>
              <type>stereographic</type>
              <scale_to_hfov>1</scale_to_hfov>
              <cutoff_angle>1.5707</cutoff_angle>
              <env_texture_size>512</env_texture_size>
            </lens>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
          <plugin name='camera_back_plugin' filename='libgazebo_ros_camera.so'>
            <alwaysOn>1</alwaysOn>
            <updateRate>30</updateRate>
            <cameraName>camera_back</cameraName>
            <imageTopicName>camera_back/image_raw</imageTopicName>
            <cameraInfoTopicName>camera_back/camera_info</cameraInfoTopicName>
            <frameName>camera_back_link</frameName>
            <distortionK1>0.0</distortionK1>
            <distortionK2>0.0</distortionK2>
            <distortionK3>0.0</distortionK3>
            <distortionT1>0.0</distortionT1>
            <distortionT2>0.0</distortionT2>
          </plugin>
        </sensor>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>0 0 {z} 0 -0 0</pose>
    </model>""".format(z=z_off)
    
    sdf_string += "\n  </world>"
    sdf_string += "\n</sdf>"

    with open(world_file, "w") as f:
        f.write(sdf_string)

