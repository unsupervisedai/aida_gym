#Python script to create the file "bords.obj". The file "terrain1.obj" is written by hand.
#The vertices created forms a 10*10*2 box.
f = open("bords.obj", "w")

f.write("mtllib terrain1.mtl\n")
f.write("o Bords\n")
f.write("\n")
for i in range(10):
	for j in range(10):
		t = "v "
		x = -9.0+2.0*i
		y = -9.0+2.0*j
		t = t + str(x) + " " + str(y) + " 0.0" # z = 0.0 for the high steps
		f.write(t + "\n")
		t = "v "
		t = t + str(x) + " " + str(y) + " -0.01" # z = -0.01 for the low steps
		f.write(t + "\n")

f.write("\n")
f.write("vt 0.0 0.01\n") # z = -0.01
f.write("vt 0.0 0.0\n") # z = 0.0
f.write("vt 1.0 0.01\n") # z = -0.01
f.write("vt 1.0 0.0\n") # z = 0.0
f.write("\n")

f.write("usemtl Material\n")
f.write("s off\n")

for n in range(9):
	for m in range(10):
		t = "f "
		i = 1+2*n+20*m
		t = t + str(i) + "/1 " + str(i+1) + "/2 " + str(i+2) + "/3"
		f.write(t + "\n")
		t = "f "
		t = t + str(i) + "/1 " + str(i+2) + "/3 " + str(i+1) + "/2"
		f.write(t + "\n")
		t = "f "
		t = t + str(i+1) + "/2 " + str(i+3) + "/4 " + str(i+2) + "/3"
		f.write(t + "\n")
		t = "f "
		t = t + str(i+1) + "/2 " + str(i+2) + "/3 " + str(i+3) + "/4"
		f.write(t + "\n")

for n in range(10):
	for m in range(9):
		t = "f "
		i = 1+2*n+20*m
		t = t + str(i) + "/1 " + str(i+1) + "/2 " + str(i+20) + "/3"
		f.write(t + "\n")
		t = "f "
		t = t + str(i) + "/1 " + str(i+20) + "/3 " + str(i+1) + "/2"
		f.write(t + "\n")
		t = "f "
		t = t + str(i+1) + "/2 " + str(i+20) + "/3 " + str(i+21) + "/4"
		f.write(t + "\n")
		t = "f "
		t = t + str(i+1) + "/2 " + str(i+21) + "/4 " + str(i+20) + "/3"
		f.write(t + "\n")



f.close
