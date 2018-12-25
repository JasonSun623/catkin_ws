## *********************************************************
##
## File autogenerated for the robotnik_pp_planner package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 246, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [{'upper': 'PID', 'lower': 'pid', 'srcline': 123, 'name': 'PID', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::PID', 'field': 'DEFAULT::pid', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 13, 'description': 'max rotate speed', 'max': 6.283185307179586, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/hdros/catkin_ws/src/robotnik_purepursuit_planner-master/robotnik_pp_planner/cfg/robotnik_pp_planner.cfg', 'name': 'max_w', 'edit_method': '', 'default': 3.141592653589793, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 14, 'description': 'min rotate speed', 'max': 1.5707963267948966, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/hdros/catkin_ws/src/robotnik_purepursuit_planner-master/robotnik_pp_planner/cfg/robotnik_pp_planner.cfg', 'name': 'min_w', 'edit_method': '', 'default': -3.141592653589793, 'level': 0, 'min': -3.141592653589793, 'type': 'double'}, {'srcline': 15, 'description': 'kp for rotate speed', 'max': 2.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/hdros/catkin_ws/src/robotnik_purepursuit_planner-master/robotnik_pp_planner/cfg/robotnik_pp_planner.cfg', 'name': 'kp_w', 'edit_method': '', 'default': 0.5, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 16, 'description': 'ki for rotate speed', 'max': 0.5, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/hdros/catkin_ws/src/robotnik_purepursuit_planner-master/robotnik_pp_planner/cfg/robotnik_pp_planner.cfg', 'name': 'ki_w', 'edit_method': '', 'default': 0.05, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 17, 'description': 'kd for rotate speed', 'max': 0.5, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/hdros/catkin_ws/src/robotnik_purepursuit_planner-master/robotnik_pp_planner/cfg/robotnik_pp_planner.cfg', 'name': 'kd_w', 'edit_method': '', 'default': 0.005, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 19, 'description': 'max translate speed', 'max': 2.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/hdros/catkin_ws/src/robotnik_purepursuit_planner-master/robotnik_pp_planner/cfg/robotnik_pp_planner.cfg', 'name': 'max_v', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 20, 'description': 'min translate speed', 'max': 0.1, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/hdros/catkin_ws/src/robotnik_purepursuit_planner-master/robotnik_pp_planner/cfg/robotnik_pp_planner.cfg', 'name': 'min_v', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 21, 'description': 'kp for translate speed', 'max': 2.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/hdros/catkin_ws/src/robotnik_purepursuit_planner-master/robotnik_pp_planner/cfg/robotnik_pp_planner.cfg', 'name': 'kp_v', 'edit_method': '', 'default': 0.5, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 22, 'description': 'ki for translate speed', 'max': 0.5, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/hdros/catkin_ws/src/robotnik_purepursuit_planner-master/robotnik_pp_planner/cfg/robotnik_pp_planner.cfg', 'name': 'ki_v', 'edit_method': '', 'default': 0.05, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 23, 'description': 'kd for translate speed', 'max': 0.5, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/hdros/catkin_ws/src/robotnik_purepursuit_planner-master/robotnik_pp_planner/cfg/robotnik_pp_planner.cfg', 'name': 'kd_v', 'edit_method': '', 'default': 0.005, 'level': 0, 'min': 0.0, 'type': 'double'}], 'type': '', 'id': 1}, {'upper': 'TRAJECTORYCONTROL', 'lower': 'trajectorycontrol', 'srcline': 123, 'name': 'TrajectoryControl', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::TRAJECTORYCONTROL', 'field': 'DEFAULT::trajectorycontrol', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 27, 'description': 'trajectory track inc dist', 'max': 0.3, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/hdros/catkin_ws/src/robotnik_purepursuit_planner-master/robotnik_pp_planner/cfg/robotnik_pp_planner.cfg', 'name': 'look_ahead_inc', 'edit_method': '', 'default': 0.05, 'level': 0, 'min': 0.001, 'type': 'double'}, {'srcline': 28, 'description': 'trajectory track min dist', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/hdros/catkin_ws/src/robotnik_purepursuit_planner-master/robotnik_pp_planner/cfg/robotnik_pp_planner.cfg', 'name': 'd_lookahear_min', 'edit_method': '', 'default': 0.3, 'level': 0, 'min': 0.05, 'type': 'double'}, {'srcline': 29, 'description': 'trajectory track max dist', 'max': 2.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/hdros/catkin_ws/src/robotnik_purepursuit_planner-master/robotnik_pp_planner/cfg/robotnik_pp_planner.cfg', 'name': 'd_lookahear_max', 'edit_method': '', 'default': 1.1, 'level': 0, 'min': 1.0, 'type': 'double'}, {'srcline': 30, 'description': 'trajectory track max speed', 'max': 2.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/hdros/catkin_ws/src/robotnik_purepursuit_planner-master/robotnik_pp_planner/cfg/robotnik_pp_planner.cfg', 'name': 'max_speed', 'edit_method': '', 'default': 1.2, 'level': 0, 'min': 1.0, 'type': 'double'}, {'srcline': 31, 'description': 'trajectory track frequency', 'max': 100.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/hdros/catkin_ws/src/robotnik_purepursuit_planner-master/robotnik_pp_planner/cfg/robotnik_pp_planner.cfg', 'name': 'desired_freq', 'edit_method': '', 'default': 50.0, 'level': 0, 'min': 10.0, 'type': 'double'}], 'type': '', 'id': 2}], 'parameters': [], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

