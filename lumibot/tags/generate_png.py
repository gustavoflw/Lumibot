from os import system

size = 30.0

for id in range(0, 3):
	system("cd png && rosrun ar_track_alvar createMarker {} -s {} && cd ..".format(id, size))
