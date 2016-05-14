/*
	Main source for OffWorld Robotics Autonomous Mapping & Control GUI
	Displayed features include:
	- Occupancy Grid
	- Start/Stop buttons (todo)

	Compile using: catkin_make
	Setup environment using: source devel/setup.bash
	Run using: rosrun owr_gui automap
*/

#include "AutoMapGUI.h"
#include "AutoMapNode.h"
#include <cstdlib>

int main(int argc, char **argv) {
	ros::init(argc, argv, "AutoMapGUI");
	AutoMapGUI gui(WINDOW_W, WINDOW_H, &argc, argv);
	gui.run();
	return EXIT_SUCCESS;
}

AutoMapGUI::AutoMapGUI(int width, int height, int *argc, char **argv) : GLUTWindow(width, height, argc, argv, "AutoMap") {
	autoMapNode = new AutoMapNode(this);
	
	glClearColor(1, 1, 1, 0);
	glShadeModel(GL_FLAT);
	glEnable(GL_BLEND); // enables transparency
	glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
	glutKeyboardFunc(glut_keydown);
	
	gridCols = gridRows = -1;
	gridData = NULL;
	
	srand(time(NULL));
	gridCols = 100;
	gridRows = 70;
	gridData = (char *)malloc(gridCols*gridRows*sizeof(char));
	for(int i = 0;i < gridCols*gridRows;i++) {
		gridData[i] = rand();
	}
}

void AutoMapGUI::updateGrid(char *grid, int width, int height) {
	if(grid != NULL) {
		if(gridData != NULL) {
			free(gridData);
		}
		gridData = (char *)malloc(width*height*sizeof(char));
		memcpy(gridData, grid, width*height*sizeof(char));
		gridCols = width;
		gridRows = height;
	}
}

void AutoMapGUI::reshape(int w, int h) {
	GLUTWindow::reshape(w, h);
}

void AutoMapGUI::idle() {
	ros::spinOnce();
	display();
	usleep(15000);
}

void AutoMapGUI::display() {
	glClearColor(1, 1, 1, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	drawOccupancyGrid();
	glutSwapBuffers();
}

void AutoMapGUI::drawOccupancyGrid() {
	double cellWidth = (double)currWinW/(double)gridCols;
	double cellHeight = (double)currWinH/(double)gridRows;
	
	for(int i = 0;i < gridRows;i++) {
		for(int j = 0;j < gridCols;j++) {
			double data = gridData[i*gridRows+j]/100.0;
			if(data < 0) {
				glColor3f(0.5,0,0);
			} else {
				glColor3f(data, data, data);
			}
			glRectd(j*cellWidth, -i*cellHeight, (j+1)*cellWidth, -(i+1)*cellHeight);
		}
	}
}

void AutoMapGUI::keydown(unsigned char key, int x, int y) {
	if (key == 27) {
		exit(0);
	}
}
