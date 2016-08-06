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
#include <cctype>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Header.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "AutoMapGUI");
	AutoMapGUI gui(WINDOW_W, WINDOW_H, &argc, argv);
	gui.run();
	return EXIT_SUCCESS;
}

AutoMapGUI::AutoMapGUI(int width, int height, int *argc, char **argv) :
	GLUTWindow(width, height, argc, argv, "AutoMap"),
	startButton(width-200, -400, 200, 100, 0, 0, 1, "Start", AutoMapGUI::sendStartMessage, NULL),
	stopButton(width-200, -600, 200, 100, 0, 0, 1, "Stop", AutoMapGUI::sendStopMessage, NULL),
	goalButton(width-200, -800, 200, 100, 0, 0, 1, "Publish Goal", AutoMapGUI::sendGoalMessage, NULL)
	{
	autoMapNode = new AutoMapNode(this);
	
	glClearColor(0, 0, 0, 0);
	glShadeModel(GL_FLAT);
	glutKeyboardFunc(glut_keydown);
	glutKeyboardUpFunc(glut_keyup);
	glutMouseFunc(glut_mouse);
	
	gridCols = gridRows = -1;
	gridData = NULL;
	
	glGenTextures(1, &gridTexture);
	glBindTexture(GL_TEXTURE_2D, gridTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	
	showHelp = false;
	textBuffer[0] = '\0';
	bufferIndex = 0;
	
	validGoalCoords = false;
}

void AutoMapGUI::updateGrid(char *grid, int cols, int rows, std_msgs::Header header) {
	if(grid != NULL) {
		lastGridHeader = header;
		if(gridData != NULL) {
			free(gridData);
		}
		gridCols = cols;
		gridRows = rows;
		gridData = (unsigned char *)malloc(gridCols*gridRows*3*sizeof(unsigned char));
		for(int i = 0;i < gridCols*gridRows;i++) {
			gridData[i*3] = gridData[i*3+1] = gridData[i*3+2] = (grid[i] < 0) ? 0 : grid[i] + 155;
		}
		glBindTexture(GL_TEXTURE_2D, gridTexture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, gridCols, gridRows, 0, GL_RGB, GL_UNSIGNED_BYTE, gridData);
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
	
	glPushMatrix();
	glTranslated(currWinW - 350, -100, 0);
	char txt[50] = {0};
	glColor3f(1, 1, 1);
	sprintf(txt, "Input buffer: %s", textBuffer);
	glRasterPos2i(-5, -6);
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>(txt));
	glRasterPos2i(-5, -56);
	
	if(validGoalCoords) {
		glColor3f(0, 0, 1);
		sprintf(txt, "Intended goal: x = %d, y = %d", goalGridCol, goalGridRow);
	} else {
		glColor3f(1, 0, 0);
		sprintf(txt, "Intended goal: invalid");
	}
	
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>(txt));
	
	glPopMatrix();
	
	startButton.draw();
	stopButton.draw();
	goalButton.draw();
	
	if(showHelp) {
		glPushMatrix();
		glTranslated(100, -100, 0);
		glColor3f(0,0,1);
		glRecti(0, 0, 1000, -500);
		glColor3f(1,1,1);
		char text[] = "Placeholder Help Text";
		glRasterPos2i(50, -50);
		glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, reinterpret_cast<const unsigned char *>(text));
		glPopMatrix();
	}
	
	glutSwapBuffers();
}

void AutoMapGUI::keydown(unsigned char key, int x, int y) {
	if(key == 27) {
		exit(0);
	} else if(key == '\t') {
		showHelp = true;
	} else if(key == 8) {
		if(bufferIndex > 0) {
			textBuffer[--bufferIndex] = '\0';
		}
	} else if(isdigit(key) || key == ' ') {
		if(bufferIndex < BUFFER_SIZE-1) {
			textBuffer[bufferIndex++] = key;
			textBuffer[bufferIndex] = '\0';
		}
	}
	extractGoalCoords();
}

void AutoMapGUI::extractGoalCoords() {
	std::stringstream ss(textBuffer);
	bool read;
	int tmpRow, tmpCol;
	read = (ss >> tmpRow);
	if(!read) {
		validGoalCoords = false;
		return;
	}
	goalGridRow = tmpRow;
	read = (ss >> tmpCol);
	if(!read) {
		validGoalCoords = false;
		return;
	}
	goalGridCol = tmpCol;
	validGoalCoords = true;
}

void AutoMapGUI::keyup(unsigned char key, int x, int y) {
	if(key == '\t') {
		showHelp = false;
	}
}

void AutoMapGUI::mouse(int button, int state, int x, int y) {
	y = -y;
	if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
		if(startButton.isInside(x, y)) {
			startButton.click();
		} else if(stopButton.isInside(x, y)) {
			stopButton.click();
		} else if(goalButton.isInside(x, y)) {
			goalButton.click();
		}
	}
}

void AutoMapGUI::sendStartMessage() {
	ROS_INFO("publishing start message");
	std_msgs::Bool msg;
	msg.data = true;
	ros::NodeHandle n;
	ros::Publisher startPublisher = n.advertise<std_msgs::Bool>("/owr_auton_pathing/astarstart", 10);
	startPublisher.publish(msg);
	ros::spinOnce();
}

void AutoMapGUI::sendStopMessage() {
	ROS_INFO("publishing stop message");
	std_msgs::Bool msg;
	msg.data = false;
	ros::NodeHandle n;
	ros::Publisher startPublisher = n.advertise<std_msgs::Bool>("/owr_auton_pathing/astarstart", 10);
	startPublisher.publish(msg);
	ros::spinOnce();
}

void AutoMapGUI::sendGoalMessage() {
	ROS_INFO("publishing goal: row = %d, col = %d", goalGridRow, goalGridCol);
	geometry_msgs::PointStamped msg;
	msg.header = lastGridHeader;
	msg.point.x = goalGridCol;
	msg.point.y = goalGridRow;
	msg.point.z = 0;
	ros::NodeHandle n;
	ros::Publisher goalPublisher = n.advertise<geometry_msgs::PointStamped>("/owr_auton_pathing/astargoal", 10);
	goalPublisher.publish(msg);
	ros::spinOnce();
}

int AutoMapGUI::goalGridRow = -1;
int AutoMapGUI::goalGridCol = -1;
std_msgs::Header AutoMapGUI::lastGridHeader;
