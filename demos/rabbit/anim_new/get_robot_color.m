function robot_color = get_robot_color()
palette = get_palette_colors();
palette.originalblue = palette.blue;
palette.darkblue = 0.5 * palette.blue + 0.5 * palette.navy;
palette.pink = 0.75 * palette.magenta + 0.25 * palette.white;
palette.darkpink = 0.75 * palette.magenta + 0.25 * palette.black;
palette.blue = 0.8 * palette.blue + 0.2 * palette.white;
palette.lightblue = 0.4 * palette.blue + 0.6 * palette.white;
palette.lightpink = 0.4 * palette.pink + 0.6 * palette.white;


robot_color.leg1_color=palette.pink;
robot_color.leg1_joint_color = palette.darkpink;
robot_color.leg1_joint_edge_color = palette.lightpink;

robot_color.leg2_color=palette.blue;
robot_color.leg2_joint_color = palette.darkblue;
robot_color.leg2_joint_edge_color = palette.lightblue;

robot_color.torso_color=palette.blue;
robot_color.torso_mass_color=palette.darkblue;
robot_color.torso_mass_edge_color=palette.lightblue;
