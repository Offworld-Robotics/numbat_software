/*
	Main source for OffWorld Robotics Autonomous Mapping & Control GUI
	Displayed features include:
	- Occupancy Grid
	- Start/Stop buttons

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
	
	glClearColor(0, 0, 0, 0);
	glShadeModel(GL_FLAT);
	glutKeyboardFunc(glut_keydown);
	
	gridCols = gridRows = -1;
	gridData = NULL;
	
	glGenTextures(1, &gridTexture);
	glBindTexture(GL_TEXTURE_2D, gridTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
}

void AutoMapGUI::updateGrid(char *grid, int cols, int rows) {
	if(grid != NULL) {
		if(gridData != NULL) {
			free(gridData);
		}
		gridCols = cols;
		gridRows = rows;
		ROS_INFO("start copy: %d by %d", cols, rows);
		gridData = (unsigned char *)malloc(gridCols*gridRows*3*sizeof(unsigned char));
		for(int i = 0;i < gridCols*gridRows;i++) {
			gridData[i*3] = gridData[i*3+1] = gridData[i*3+2] = (grid[i] < 0) ? 0 : grid[i] + 155;
		}
		glBindTexture(GL_TEXTURE_2D, gridTexture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, gridCols, gridRows, 0, GL_RGB, GL_UNSIGNED_BYTE, gridData);
		ROS_INFO("end copy");
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
	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT);

	glPushMatrix();
	glEnable(GL_TEXTURE_2D);
	glColor3f(0, 0, 0);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glBindTexture(GL_TEXTURE_2D, gridTexture);

	glBegin(GL_QUADS);
		glTexCoord2d(0, 1); glVertex2d(0, 0);
		glTexCoord2d(0, 0); glVertex2d(0, -currWinH);
		glTexCoord2d(1, 0); glVertex2d(currWinH, -currWinH);
		glTexCoord2d(1, 1); glVertex2d(currWinH, 0);
	glEnd();

	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
	
	glutSwapBuffers();
}

void AutoMapGUI::keydown(unsigned char key, int x, int y) {
	if (key == 27) {
		exit(0);
	}
}
