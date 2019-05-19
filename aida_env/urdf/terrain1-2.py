#Python script to create the file "terrain1.urdf" which uses itself "terrain1.obj" and "bords.obj".
#"bords.obj" is loaded once while "terrain1.obj" (which is just one step) is loaded several times at different positions.
f = open("terrain1.urdf", "w")

f.write('<?xml version="0.0" ?>\n')
f.write('<robot name="terrain">\n')

#This is where "terrain.obj" is loaded for each step

for i in range(9):
	for j in range(9):
		k = i+9*j
		o = -0.01*(k%2) # z = 0.0 or -0.01
		t = '  <link name="'+str(k)+'">\n'
		t += "  <contact>\n"
		t += '    <lateral_friction value="1"/>\n'
		t += '  </contact>\n'
		t += '    <inertial>\n'
		t += '      <origin rpy="0 0 0" xyz="0 0 0"/>\n'
		t += '      <mass value=".0"/>\n'
		t += '      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>\n'
		t += '    </inertial>\n'
		t += '    <visual>\n'
		t += '      <origin rpy="0 0 0" xyz="' + str(2*i-8) + " " + str(2*j-8) + " " + str(o) + ' "/>\n'
		t += '    <geometry>\n'
		t += '        <mesh filename="terrain1.obj" scale="1 1 1"/>\n'
		t += '    </geometry>\n'
		t += '    <material name="white">'
		t += '    <color rgba="1 1 1 1"/>'
		t += '    </material>'
		t += '    </visual>\n'
		t += '    <collision>\n'
		t += '      <origin rpy="0 0 0" xyz="' + str(2*i-8) + " " + str(2*j-8) + " " + str(o) + ' "/>\n'
		t += '    <geometry>\n'
		t += '        <mesh filename="terrain1.obj" scale="1 1 1"/>\n'
		t += '    </geometry>\n'
		t += '    </collision>\n'
		t += '  </link>\n\n'
		f.write(t)
f.write("\n")

#Joints of the steps

for i in range(9):
	for j in range(9):
		k = i+9*j
		if (k>0):
			t = '<joint name="' + str(k) + '" type="fixed">\n'
			t += '    <parent link="0"/>\n'
			t += '    <child link="' + str(k) + '"/>\n'
			t += '    <origin rpy="0 0 0" xyz="0 0 0"/>\n'
			t += '</joint>\n'
			f.write(t)
f.write("\n")

#This is where "bords.obj" is loaded

t = '  <link name="bords">\n'
t += "  <contact>\n"
t += '    <lateral_friction value="1"/>\n'
t += '  </contact>\n'
t += '    <inertial>\n'
t += '      <origin rpy="0 0 0" xyz="0 0 0"/>\n'
t += '      <mass value=".0"/>\n'
t += '      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>\n'
t += '    </inertial>\n'
t += '    <visual>\n'
t += '      <origin rpy="0 0 0" xyz="0 0 0"/>\n'
t += '    <geometry>\n'
t += '        <mesh filename="bords.obj" scale="1 1 1"/>\n'
t += '    </geometry>\n'
t += '    <material name="white">'
t += '    <color rgba="1 1 1 1"/>'
t += '    </material>'
t += '    </visual>\n'
t += '  </link>\n\n'
f.write(t)

#Joint of "bords"

t = '<joint name="bords" type="fixed">\n'
t += '    <parent link="0"/>\n'
t += '    <child link="bords"/>\n'
t += '    <origin rpy="0 0 0" xyz="0 0 0"/>\n'
t += '</joint>\n'
f.write(t)


f.write("</robot>")
f.close
