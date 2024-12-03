import os
import math
from jinja2 import Template

# 读取模板内容
# template_path = '/root/catkin_ws/src/Gazebo_cable/temp/temp_Mul_sep.sdf'  
# template_path = '/root/catkin_ws/src/Gazebo_cable/temp/temp_Mul_hook.sdf'  
# template_path = '/root/catkin_ws/src/Gazebo_cable/temp/temp_Mul.sdf'  
template_path = '/home/carlson/ros/multi_cable/src/multi_cable/temp/multi_cable_new.sdf'  
with open(template_path, 'r') as file:
    template_content = file.read()

# 创建Jinja2模板
template = Template(template_content)

# 定义模板变量
context = {
    'height': 0.0,
    'blue_color': "0 0 0.8 1",
    'red_color': "1 0 0 1",
    'gray_color': "0.5 0.5 0.5 1",
    'math': math,

    
    'cable_count': 6,
    'segment_length': 0.1,
    'segment_count': 10,  # TOTAL number of segments

    'segment_mass': 0.01,
    'rope_radius': 0.002,

    'base_mass': 0.1,
    'base_length': 0.02,

    'payload_mass': 1.2,
    'payload_radius': 0.15,
    'payload_height': 0.05,
    # 'angle': 0, # 与z轴夹角, 使base_link不竖直于重物上方
    # 'angle': math.pi/6, # 与z轴夹角, 使base_link不竖直于重物上方
    # 'angle': math.pi/4, # 与z轴夹角, 使base_link不竖直于重物上方
    # 'angle': math.pi/3,
    'angle': math.pi/2,

    'stiffness': 100000.0,
    'damping': 0,
    'dissipation': 0.0,
    'friction': 0.0,
    'stepsize': 0.01,
}

# 渲染模板
rendered_sdf = template.render(context)

# 检查并创建目录
output_dir = '/home/carlson/ros/multi_cable/src/multi_cable/sdf'
os.makedirs(output_dir, exist_ok=True)

# 保存渲染结果到文件
# output_file = os.path.join(output_dir, 'cable_urdfTosdf.sdf')
output_file = os.path.join(output_dir, 'cable_Mul_new.sdf')
with open(output_file, 'w') as f:
    f.write(rendered_sdf)

print(f"Generated SDF file: {output_file}")
