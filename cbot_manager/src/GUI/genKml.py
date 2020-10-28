def copyData():
	fin = open("in.kml")
	fout = open("out.kml","w").close()
	fout = open("out.kml","w")
	initial_data = fin.read()
	fout.seek(0)
	fout.write(initial_data)
	fin.close()
	fout.close()

def addNewPoint(lat,lon,depth):
	fout = open("out.kml","r+")
	data = str(lon) + "," + str(lat) + "," + str(depth) + "\n"
	all_lines = fout.readlines()
	initial_lines = all_lines[:-1]
	last_line = all_lines[-1]
	initial_lines.append(data)
	initial_lines.append(last_line)
	fout.seek(0)
	fout.write(str("".join(initial_lines)))
	fout.close()

if __name__=="__main__":
	print("in pykml")
	copyData()
