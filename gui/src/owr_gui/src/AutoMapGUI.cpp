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
#include <limits>

#define PI 3.1415926535897932384626433832795

int main(int argc, char **argv) {
	ros::init(argc, argv, "AutoMapGUI");
	AutoMapGUI gui(WINDOW_W, WINDOW_H, &argc, argv);
	gui.run();
	return EXIT_SUCCESS;
}

AutoMapGUI::AutoMapGUI(int width, int height, int *argc, char **argv) :
	GLUTWindow(width, height, argc, argv, "AutoMap"),
	setStartButton(width-200, -300, 200, 100, 0, 0, 1, "Set Start Cell", NULL, NULL),
	setGoalButton(width-200, -450, 200, 100, 0, 0, 1, "Set Goal Cell", NULL, NULL),
	startButton(width-200, -600, 200, 100, 0, 0, 1, "Start", NULL, NULL),
	stopButton(width-200, -750, 200, 100, 0, 0, 1, "Stop", NULL, NULL)
	{
	autoMapNode = new AutoMapNode(this);
	
	glClearColor(0, 0, 0, 0);
	glShadeModel(GL_FLAT);
	glutKeyboardFunc(glut_keydown);
	glutKeyboardUpFunc(glut_keyup);
	glutMouseFunc(glut_mouse);
	
	gridCols = gridRows = 0;
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
	
	validBufferCoords = false;
	bufferGridRow = bufferGridCol = std::numeric_limits<unsigned int>::max();
	
	ros::NodeHandle n;
	startStopPublisher = n.advertise<std_msgs::Bool>("/owr_auton_pathing/astarstart", 10);
	goalPublisher = n.advertise<geometry_msgs::PointStamped>("/owr_auton_pathing/astargoal", 10);
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
		extractBufferCoords();
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

void AutoMapGUI::drawHelpText() {
	glPushMatrix();
	glTranslated(100, -100, 0);
	glColor3f(0,0,1);
	glRecti(0, 0, 1000, -500);
	glColor3f(1,1,1);
	char text1[] = "Type space separated integers for the row and column of the cell of interest.";
	glRasterPos2i(50, -50);
	glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, reinterpret_cast<const unsigned char *>(text1));
	char text2[] = "If the cell is valid, a marker will appear on the map.";
	glRasterPos2i(50, -100);
	glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, reinterpret_cast<const unsigned char *>(text2));
	char text3[] = "Click the Set Start Cell button to send starting point info to ROS.";
	glRasterPos2i(50, -150);
	glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, reinterpret_cast<const unsigned char *>(text3));
	char text4[] = "Click the Set Goal Cell button to send goal point info to ROS.";
	glRasterPos2i(50, -200);
	glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, reinterpret_cast<const unsigned char *>(text4));
	glPopMatrix();
}

void AutoMapGUI::drawGrid() {
	glPushMatrix();
	glEnable(GL_TEXTURE_2D);
	glColor3f(0, 0, 0);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glBindTexture(GL_TEXTURE_2D, gridTexture);

	glBegin(GL_QUADS);
		glTexCoord2d(0, 1); glVertex2d(0, 0);
		glTexCoord2d(0, 0); glVertex2d(0, -currWinH);
		glTexCoord2d(1, 0); glVertex2d(currWinH*1.3, -currWinH);
		glTexCoord2d(1, 1); glVertex2d(currWinH*1.3, 0);
	glEnd();

	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}

void AutoMapGUI::drawTextBuffer() {
	glPushMatrix();
	glTranslated(currWinW - 400, -100, 0);
	char txt[50] = {0};
	glColor3f(1, 1, 1);
	sprintf(txt, "Input buffer: %s", textBuffer);
	glRasterPos2i(0, 0);
	glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, reinterpret_cast<const unsigned char *>(txt));
	glRasterPos2i(0, -50);
	sprintf(txt, "Grid size: rows = %d, cols = %d", gridRows, gridCols);
	glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, reinterpret_cast<const unsigned char *>(txt));
	
	glRasterPos2i(0, -100);
	int pos = sprintf(txt, "Specified cell: ");
	if(validBufferCoords) {
		glColor3f(0, 0, 1);
		sprintf(txt+pos, "row = %d, col = %d", bufferGridRow, bufferGridCol);
	} else {
		glColor3f(1, 0, 0);
		sprintf(txt+pos, "invalid");
	}
	
	glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, reinterpret_cast<const unsigned char *>(txt));
	
	glPopMatrix();
}

void AutoMapGUI::display() {
	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT);
	
	drawGrid();
	drawBufferMarker();
	drawTextBuffer();
	
	drawDividingLine();
	
	setStartButton.draw();
	setGoalButton.draw();
	startButton.draw();
	stopButton.draw();
	
	if(showHelp) {
		drawHelpText();
	}
	
	glutSwapBuffers();
}

void AutoMapGUI::drawDividingLine() {
	glColor3f(0, 1, 0);
	glBegin(GL_LINES);
	glVertex2d(currWinH*1.3, 0);
	glVertex2d(currWinH*1.3, -currWinH);
	glEnd();
}

void AutoMapGUI::drawBufferMarker() {
	if(!validBufferCoords) {
		return;
	}
	double x = (((double)bufferGridCol + 0.5)/(double)gridCols)*((double)currWinH*1.3);
	double y = (((double)bufferGridRow + 0.5)/(double)gridRows)*-(double)currWinH;
	glPushMatrix();
	glTranslated(x, y, 0);
	
	glColor3f(1, 0, 0);
	double theta = 0;
	double r = 5;
	glBegin(GL_LINE_STRIP);
	for(int i = 0;i <= 32;++i, theta += 2*PI/32) {
		glVertex2d(r*cos(theta), r*sin(theta));
	}
	glEnd();
	glPopMatrix();
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
	extractBufferCoords();
}

void AutoMapGUI::extractBufferCoords() {
	std::stringstream ss(textBuffer);
	bool read;
	unsigned int tmpRow, tmpCol;
	read = (ss >> tmpRow);
	if(!read) {
		validBufferCoords = false;
		return;
	}
	read = (ss >> tmpCol);
	if(!read) {
		validBufferCoords = false;
		return;
	}
	if(tmpRow < gridRows && tmpCol < gridCols) {
		bufferGridRow = tmpRow;
		bufferGridCol = tmpCol;
		validBufferCoords = true;
	} else {
		validBufferCoords = false;
	}
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
			sendStartMessage();
		} else if(stopButton.isInside(x, y)) {
			sendStopMessage();
		} else if(setGoalButton.isInside(x, y)) {
			sendGoalMessage();
		}
	}
}

void AutoMapGUI::sendStartMessage() {
	ROS_INFO("publishing start message");
	std_msgs::Bool msg;
	msg.data = true;
	startStopPublisher.publish(msg);
	ros::spinOnce();
}

void AutoMapGUI::sendStopMessage() {
	ROS_INFO("publishing stop message");
	std_msgs::Bool msg;
	msg.data = false;
	startStopPublisher.publish(msg);
	ros::spinOnce();
}

void AutoMapGUI::sendGoalMessage() {
	if(!validBufferCoords) {
		ROS_INFO("publish goal failed: invalid coordinates");
		return;
	}
	ROS_INFO("publishing goal: row = %d, col = %d", bufferGridRow, bufferGridCol);
	geometry_msgs::PointStamped msg;
	msg.header = lastGridHeader;
	msg.point.x = bufferGridCol;
	msg.point.y = bufferGridRow;
	msg.point.z = 0;
	goalPublisher.publish(msg);
	ros::spinOnce();
}
