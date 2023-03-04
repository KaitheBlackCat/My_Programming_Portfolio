import sys
import matplotlib.pyplot as plt
from matplotlib_venn import venn3, venn3_circles, venn3_unweighted
import numpy as np
import venn

version = {}
checksum = {}
toolNames = []
bug_rate = {}

# Modify if text trial is changed
Test_trial = 10

color_palatte = ('#3E64AF', '#3EAF5D', '#D74E3B')

venn_diagram_content = ('t120', 't600', 't120+dive')

for i in range(len(sys.argv) - 1):
	toolNames.append(sys.argv[i+1][:-4])
	try:
		with open(sys.argv[i+1], 'r') as f:
			wholeFile = f.readlines()
			for j, line in enumerate(wholeFile):
				buffer = list(map(int, line.split('/')))
				if j == len(wholeFile) - 1:
					checksum[sys.argv[i+1][:-4]] = buffer
				else:
					if buffer[0] != 0: # If version has at least one error detection
						buffer = [j+1, buffer[1]]
						if sys.argv[i+1][:-4] in version:
							buffer2 = version[sys.argv[i+1][:-4]]
							buffer2.append(buffer)
							version[sys.argv[i+1][:-4]] = buffer2
						else:
							version[sys.argv[i+1][:-4]] = [buffer]
			print("File " + sys.argv[i+1] + " is done.")

	except FileNotFoundError:
		print("[Error] File is not exit. Check the existance of file or the name of file you entered.")
		sys.exit()

# version format
# {(Tool name) : [[Version number 1, Bug count 1], [Version number 2, Bug count 2], ...]}

version_subset = [] # Split each version of tools into sets to make a venn diagram.

for toolname in toolNames:
	buffer = []
	for VersionData in version[toolname]:
		buffer.append(VersionData[0])

		if toolname in bug_rate:
			buffer3 = bug_rate[toolname]
			buffer3 += VersionData[1]
			bug_rate[toolname] = buffer3
		else:
			bug_rate[toolname] = VersionData[1]

	version_subset.append(set(buffer))

# We seperated the values of 120 and dive
# Let's combine

version_subset[2].update(version_subset[0])

# Draw a venn diagram.

fig, ax = plt.subplots(2, 1)
fig.suptitle("Tool Bug Detection Summary", fontsize = 16)
plt.subplots_adjust(wspace = 0.3, hspace = 0.3)

v = venn3_unweighted(version_subset, 
	  			 	venn_diagram_content, 
	  			 	ax = ax[0], 
	  			 	set_colors = color_palatte,
	  			 	alpha = 0.75)
#venn3_circles(version_subset, ax = ax[0], lw = 0.7, linestyle="dashed")

# The error that all tool detected show as orange color

v.get_patch_by_id('111').set_color('orange')

# Make a table for tool error detection

data = []

for toolname in toolNames:
	data.append([toolname, round((bug_rate[toolname] / (checksum[toolname][1] * Test_trial)) * 100, 2)])

# Draw a table
ax[1].table(cellText = data, colLabels = ['Tool Name', 'Rate (%)'], loc = "center")

plt.axis('off')
plt.show()